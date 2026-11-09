//! Clock tree synthesis over a design, as `TritonCTS::runTritonCts`: the
//! clock nets are found from the constraints, clock gates are cloned where
//! their sinks lie far apart, sinks with insertion delay get a tree of
//! their own, each net's tree is built, and the trees are written into the
//! design with their non-default rule and dummy loads.

use crate::CtsError;
use crate::buffers::{ClockBuffers, fanout_limit};
use crate::capacitance::Farads32;
use crate::clock::{Clock, ClockSink, InstKind, InstRef};
use crate::htree::{Builder, TreeContext, TreeType};
use crate::options::{CtsOptions, MACRO_MAX_DIAMETER, MAX_LEAF_SINKS, NdrStrategy, Settings};
use crate::techchar::{CharSetup, TechChar};
use pare_common::db::design::{
    NetlistDB, NonDefaultLayer, NonDefaultRule, PinOwner, PlacementStatus,
};
use pare_common::db::indices::{CellId, MacroPinId, NetId, NonDefaultRuleId, PinId};
use pare_common::db::liberty::{BufferCell, Edge, LibCellId, LibPortId, Liberty, MinMax};
use pare_common::db::library::{MacroClass, PinDirection, SignalUse};
use pare_common::db::sdc::Sdc;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::units::{FaradsPerDbu, Seconds};
use pare_resizer::WireRc;
use pare_resizer::steiner::{DEFAULT_ALPHA, SteinerTree, prim_dijkstra};
use pare_resizer::{ClockNetwork, Link};
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BTreeMap;

/// What clock tree synthesis reads.
#[derive(Clone, Copy, Debug)]
pub struct CtsInput<'a> {
    /// The Liberty cells matched to the design's masters.
    pub link: &'a Link<'a>,
    /// The constraints whose clocks get trees.
    pub sdc: &'a Sdc,
    /// The clock network before synthesis.
    pub clocks: &'a ClockNetwork,
    /// The signal wire's resistance and capacitance, for buffer choice.
    pub signal_rc: WireRc,
    /// The clock wire's resistance and capacitance.
    pub clock_rc: WireRc,
    /// The longest wire worth driving whole, as the resizer's
    /// `findMaxWireLength` gives it; without one, `TritonCTS`'s own
    /// thresholds apply.
    pub max_wire_length: Option<Dbu>,
    /// What is asked.
    pub options: CtsOptions,
}

/// What clock tree synthesis did.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CtsSummary {
    /// Clock nets a tree was built for.
    pub trees: usize,
    /// Sinks the trees reach.
    pub sinks: usize,
    /// Clock buffers inserted.
    pub buffers: usize,
    /// Clock nets created.
    pub nets: usize,
    /// Dummy loads inserted.
    pub dummies: usize,
    /// Clock gates cloned.
    pub clones: usize,
    /// Nets given the clock's non-default rule.
    pub ndr_nets: usize,
    /// The root buffer's cell.
    pub root_buffer: String,
    /// The leaf buffers' cell.
    pub sink_buffer: String,
}

/// The trees built, kept for the steps after them.
#[derive(Clone, Debug)]
pub struct ClockTrees {
    pub(crate) builders: Vec<Builder>,
    pub(crate) setup: TreeSetup,
    pub(crate) cap_per_dbu: FaradsPerDbu,
    /// What was done.
    pub summary: CtsSummary,
}

impl ClockTrees {
    /// The clock buffers, as inferred from the library.
    #[must_use]
    pub fn clock_buffers(&self) -> &[BufferCell] {
        &self.setup.buffers.all
    }

    /// The longest wire the trees were built to drive whole, as
    /// [`CtsInput::max_wire_length`] gave it.
    #[must_use]
    pub const fn max_wire_length(&self) -> Option<Dbu> {
        self.setup.settings.max_wire_length
    }
}

/// A clock gate whose sinks are split among clones of it: its output net,
/// its cell, the pin driving the net and where that pin sits.
#[derive(Clone, Copy, Debug)]
struct GateToClone {
    net: NetId,
    cell: CellId,
    driver: PinId,
    driver_at: Point<Dbu>,
}

/// What every tree of a run is built with.
#[derive(Clone, Debug)]
pub(crate) struct TreeSetup {
    pub settings: Settings,
    pub buffers: ClockBuffers,
    /// The width and height of each buffer's master.
    pub buffer_sizes: FxHashMap<BufferCell, Point<Dbu>>,
    /// Where buffers must not go.
    pub blockages: Vec<Rect<Dbu>>,
}

impl TreeSetup {
    /// What a tree builder of `db` reads.
    pub(crate) fn context(&self, db: &NetlistDB) -> TreeContext<'_> {
        TreeContext {
            settings: &self.settings,
            core: db.core_area(),
            dbu: db.dbu().get() as i32,
            root_buffer: self.buffers.root,
            sink_buffer: self.buffers.sink,
            buffer_sizes: &self.buffer_sizes,
            blockages: &self.blockages,
        }
    }
}

/// Builds and writes the clock trees of `db`.
pub fn synthesize(db: &mut NetlistDB, input: &CtsInput<'_>) -> Result<ClockTrees, CtsError> {
    let buffers = ClockBuffers::infer(input.link, db, &input.signal_rc)?;
    let mut settings = Settings::new(input.options);
    let liberty = input.link.liberty;
    let design_fanout = input.sdc.limits.max_fanout;
    let sink_fanout = fanout_limit(liberty, buffers.sink, design_fanout);
    let root_fanout = fanout_limit(liberty, buffers.root, design_fanout);
    if let Some(root) = root_fanout
        && MAX_LEAF_SINKS > root
    {
        settings.max_fanout = Some(root);
    }
    if let Some(sink) = sink_fanout {
        settings.limit_sink_clustering_sizes(sink);
        if settings.max_fanout.is_some_and(|max| sink < max) {
            settings.max_fanout = Some(sink);
        }
    }
    settings.max_wire_length = input.max_wire_length;
    let mut tech = TechChar::new(&CharSetup {
        link: input.link,
        db,
        masters: &buffers.characterized,
        sink_buffer: buffers.sink,
        clock_rc: input.clock_rc,
    })?;
    let mut cts = Cts {
        link: input.link,
        clocks: input.clocks,
        setup: TreeSetup {
            settings,
            buffers,
            buffer_sizes: FxHashMap::default(),
            blockages: Vec::new(),
        },
        builders: Vec::new(),
        visited: FxHashSet::default(),
        net_builder: FxHashMap::default(),
        clock_nets: FxHashSet::default(),
        reg_tree_index: 0,
        dummy_index: 0,
        summary: CtsSummary::default(),
    };
    cts.summary
        .root_buffer
        .clone_from(&liberty.cell(cts.setup.buffers.root.id).name);
    cts.summary
        .sink_buffer
        .clone_from(&liberty.cell(cts.setup.buffers.sink.id).name);
    cts.populate(db, input.sdc)?;
    if !cts.builders.is_empty() {
        cts.build(db, &mut tech)?;
        cts.write(db)?;
    }
    Ok(ClockTrees {
        builders: cts.builders,
        setup: cts.setup,
        cap_per_dbu: tech.cap_per_dbu(),
        summary: cts.summary,
    })
}

struct Cts<'a> {
    link: &'a Link<'a>,
    clocks: &'a ClockNetwork,
    setup: TreeSetup,
    builders: Vec<Builder>,
    visited: FxHashSet<NetId>,
    net_builder: FxHashMap<NetId, usize>,
    clock_nets: FxHashSet<NetId>,
    reg_tree_index: usize,
    dummy_index: usize,
    summary: CtsSummary,
}

/// A pin's position as `TritonCTS` takes it: the mean of its shapes'
/// centers, in integer arithmetic.
fn pin_position(db: &NetlistDB, pin: PinId) -> Point<Dbu> {
    let shapes = db.pin_shapes(pin);
    if shapes.is_empty() {
        return db.pin_position(pin);
    }
    let (mut x, mut y) = (0_i64, 0_i64);
    for s in &shapes {
        x += s.rect.min.x.get() + (s.rect.max.x.get() - s.rect.min.x.get()) / 2;
        y += s.rect.min.y.get() + (s.rect.max.y.get() - s.rect.min.y.get()) / 2;
    }
    let n = shapes.len() as i64;
    Point::new(Dbu(x / n), Dbu(y / n))
}

/// The cell pins of a net in the order `OpenDB` lists them after reading
/// it from DEF, the last connected first.
pub(crate) fn net_cell_pins(db: &NetlistDB, net: NetId) -> Vec<PinId> {
    db.nets[net]
        .pins
        .iter()
        .rev()
        .copied()
        .filter(|&p| matches!(db.pins[p].owner, PinOwner::Cell { .. }))
        .collect()
}

pub(crate) fn pin_direction(
    db: &NetlistDB,
    pin: PinId,
) -> Option<(CellId, PinDirection, SignalUse)> {
    let PinOwner::Cell { cell, pin: mp } = db.pins[pin].owner else {
        return None;
    };
    let p = db.cell_macro(cell).pin(mp);
    Some((cell, p.direction, p.signal_use))
}

/// Whether a pin takes a signal in, as `dbITerm::isInputSignal`.
pub(crate) fn is_input_signal(db: &NetlistDB, pin: PinId) -> bool {
    pin_direction(db, pin).is_some_and(|(_, d, u)| d == PinDirection::Input && !u.is_supply())
}

fn is_placed(db: &NetlistDB, cell: CellId) -> bool {
    db.cells[cell].status != PlacementStatus::Unplaced
}

pub(crate) fn is_block(db: &NetlistDB, cell: CellId) -> bool {
    db.cell_macro(cell).class == MacroClass::Block
}

/// The first signal output pin connected on an instance.
pub(crate) fn first_output(db: &NetlistDB, cell: CellId) -> Option<PinId> {
    db.cells[cell].pins.iter().copied().find(|&p| {
        pin_direction(db, p).is_some_and(|(_, d, u)| d == PinDirection::Output && !u.is_supply())
    })
}

/// The first signal input pin connected on an instance.
pub(crate) fn first_input(db: &NetlistDB, cell: CellId) -> Option<PinId> {
    db.cells[cell]
        .pins
        .iter()
        .copied()
        .find(|&p| is_input_signal(db, p))
}

/// Whether a pin ends the clock network, as `TritonCTS::isSink`: a
/// register clock, a block's pin, or any pin of a cell with no timing
/// model.
pub(crate) fn is_sink(link: &Link<'_>, db: &NetlistDB, pin: PinId) -> bool {
    let Some((cell, _, _)) = pin_direction(db, pin) else {
        return false;
    };
    let Some((lib, _)) = link.cell(db, cell) else {
        return true;
    };
    if is_block(db, cell) {
        return true;
    }
    link.pin_port(db, pin)
        .is_some_and(|(_, port)| link.liberty.cell(lib).is_register_clock(port))
}

/// The loads a tree's buffers drive and should drive, for dummy loading.
#[derive(Default)]
struct BufferLoads {
    /// The subnet each buffer drives.
    driven_subnet: FxHashMap<InstRef, usize>,
    /// The input capacitance each buffer drives.
    output: FxHashMap<InstRef, Farads32>,
    /// The heaviest load among each buffer's siblings.
    ideal: FxHashMap<InstRef, Farads32>,
    /// Whether some buffer drives less than its ideal.
    unbalanced: bool,
}

impl Cts<'_> {
    fn is_sink(&self, db: &NetlistDB, pin: PinId) -> bool {
        is_sink(self.link, db, pin)
    }

    fn input_cap(&self, db: &NetlistDB, pin: PinId) -> Farads32 {
        self.link
            .pin_port(db, pin)
            .map_or(Farads32::ZERO, |(lib, port)| {
                Farads32::of(self.link.liberty.cell(lib).ports[port].capacitance())
            })
    }

    /// The output of a clock-network cell other than `input` whose net
    /// carries the clock on, as `TritonCTS::getSingleOutput`.
    fn single_output(&self, db: &NetlistDB, cell: CellId, input: PinId) -> Option<PinId> {
        db.cells[cell].pins.iter().copied().find(|&p| {
            p != input
                && pin_direction(db, p).is_some_and(|(_, d, _)| d == PinDirection::Output)
                && self.clock_nets.contains(&db.pins[p].net)
        })
    }

    /// The nets the clocks enter on, and the clock nets of the design, as
    /// `TritonCTS::populateTritonCTS`.
    fn populate(&mut self, db: &mut NetlistDB, sdc: &Sdc) -> Result<(), CtsError> {
        for (p, pin) in db.pins.iter().enumerate() {
            if self.clocks.is_clock(PinId::new(p)) {
                let _ = self.clock_nets.insert(pin.net);
            }
        }
        let mut clock_roots: Vec<(Vec<NetId>, String)> = Vec::new();
        let mut seen = FxHashSet::default();
        for clock in &sdc.clocks {
            let mut nets: Vec<NetId> = clock.sources.iter().map(|&io| db.io_pins[io].net).collect();
            nets.sort_unstable();
            nets.dedup();
            for net in &nets {
                if !seen.insert(*net) {
                    return Err(CtsError::OverlappingClock(clock.name.clone()));
                }
            }
            clock_roots.push((nets, clock.name.clone()));
        }
        let mut occupied: FxHashSet<Point<Dbu>> = db.cells.iter().map(|c| c.location).collect();
        for (nets, name) in clock_roots {
            for net in nets {
                self.clone_clock_gaters(db, net, &mut occupied)?;
                if !self.visited.contains(&net) {
                    self.init_one_clock_tree(db, net, net, &name, None)?;
                }
            }
        }
        self.summary.trees = self.builders.len();
        Ok(())
    }

    /// Follows the clock from `driver_net` through the cells that pass it
    /// on, making a tree for each net, as `TritonCTS::initOneClockTree`.
    fn init_one_clock_tree(
        &mut self,
        db: &mut NetlistDB,
        driver_net: NetId,
        input_net: NetId,
        sdc_name: &str,
        parent: Option<usize>,
    ) -> Result<(), CtsError> {
        let builder = self.init_clock(db, driver_net, input_net, sdc_name, parent)?;
        if let Some(b) = builder {
            let _ = self.net_builder.entry(input_net).or_insert(b);
        }
        let _ = self.visited.insert(driver_net);
        let driver = net_cell_pins(db, driver_net).into_iter().find(|&p| {
            pin_direction(db, p)
                .is_some_and(|(_, d, u)| d == PinDirection::Output && !u.is_supply())
        });
        for pin in net_cell_pins(db, driver_net) {
            if Some(pin) == driver || !is_input_signal(db, pin) || self.is_sink(db, pin) {
                continue;
            }
            let Some((cell, _, _)) = pin_direction(db, pin) else {
                continue;
            };
            let Some(output) = self.single_output(db, cell, pin) else {
                continue;
            };
            let out_net = db.pins[output].net;
            if self.visited.contains(&out_net) {
                continue;
            }
            let next_parent = if builder.is_none() {
                self.net_builder.get(&input_net).copied().or(parent)
            } else {
                builder
            };
            self.init_one_clock_tree(db, out_net, input_net, sdc_name, next_parent)?;
        }
        Ok(())
    }

    /// The sink record of a clock pin.
    fn sink(&self, db: &NetlistDB, pin: PinId) -> ClockSink {
        ClockSink {
            name: db.pin_name(pin),
            location: pin_position(db, pin),
            pin,
            input_cap: self.input_cap(db, pin),
            insertion_delay: self.insertion_delay(db, pin),
        }
    }

    /// The delay through the clock tree inside the cell of `pin`, as
    /// `TritonCTS::computeInsertionDelay` takes it from Liberty: the mean
    /// of the latest rising and falling delays at zero input slew, or the
    /// one there is; zero when insertion delays are off.
    fn insertion_delay(&self, db: &NetlistDB, pin: PinId) -> Seconds {
        if !self.setup.settings.insertion_delay {
            return Seconds::ZERO;
        }
        let Some((lib, port)) = self.link.pin_port(db, pin) else {
            return Seconds::ZERO;
        };
        let port = &self.link.liberty.cell(lib).ports[port];
        let [rise, fall] =
            Edge::BOTH.map(|edge| port.clock_tree_delay(Seconds::ZERO, edge, MinMax::Max));
        if rise != Seconds::ZERO && fall != Seconds::ZERO {
            (rise + fall) / 2.0
        } else {
            rise + fall
        }
    }

    fn add_builder(&mut self, clock: Clock, input_net: NetId, parent: Option<usize>) -> usize {
        let mut builder = Builder::new(clock, parent);
        builder.top_input_net = Some(input_net);
        self.builders.push(builder);
        let index = self.builders.len() - 1;
        if let Some(p) = parent {
            self.builders[p].children.push(index);
        }
        index
    }

    /// The tree of one clock net, split into a tree for its macros and one
    /// for its registers when it drives both, as `TritonCTS::initClock`.
    fn init_clock(
        &mut self,
        db: &mut NetlistDB,
        net: NetId,
        input_net: NetId,
        sdc_name: &str,
        parent: Option<usize>,
    ) -> Result<Option<usize>, CtsError> {
        let net_name = db.nets[net].name.clone();
        let mut clock = Clock::new(net, &net_name, sdc_name);
        clock.driver_pin = db.nets[net]
            .pins
            .iter()
            .copied()
            .find(|&p| pin_direction(db, p).is_some_and(|(_, d, _)| d == PinDirection::Output));
        let mut macro_sinks = Vec::new();
        let mut register_sinks = Vec::new();
        for pin in net_cell_pins(db, net) {
            let Some((cell, _, _)) = pin_direction(db, pin) else {
                continue;
            };
            if !is_input_signal(db, pin) || !is_placed(db, cell) {
                continue;
            }
            let mut non_sink_macro = !self.is_sink(db, pin);
            if self
                .link
                .cell(db, cell)
                .is_some_and(|(_, lib)| lib.is_inverter())
            {
                let inverted = first_output(db, cell).and_then(|out| {
                    let out_net = db.pins[out].net;
                    net_cell_pins(db, out_net)
                        .into_iter()
                        .rev()
                        .find(|&p| is_input_signal(db, p))
                });
                non_sink_macro &= inverted
                    .and_then(|p| pin_direction(db, p))
                    .is_some_and(|(c, _, _)| is_block(db, c));
            }
            let has_insertion_delay = self.insertion_delay(db, pin) != Seconds::ZERO;
            if has_insertion_delay || non_sink_macro || is_block(db, cell) {
                macro_sinks.push(pin);
            } else {
                register_sinks.push(pin);
            }
        }
        if !self.setup.settings.insertion_delay
            || macro_sinks.is_empty()
            || register_sinks.is_empty()
        {
            for pin in net_cell_pins(db, net) {
                let Some((cell, _, _)) = pin_direction(db, pin) else {
                    continue;
                };
                if is_input_signal(db, pin) && is_placed(db, cell) {
                    clock.add_sink(self.sink(db, pin));
                }
            }
            if clock.sinks.len() < 2 {
                return Ok(None);
            }
            self.summary.sinks += clock.sinks.len();
            return Ok(Some(self.add_builder(clock, input_net, parent)));
        }
        for &pin in &macro_sinks {
            clock.add_sink(self.sink(db, pin));
        }
        self.summary.sinks += clock.sinks.len();
        let first = self.add_builder(clock, input_net, parent);
        self.builders[first].tree_type = TreeType::Macro;
        let (regs_net, top_buffer) =
            self.fork_register_network(db, net, &net_name, sdc_name, &register_sinks)?;
        let mut regs_clock = Clock::new(regs_net, &db.nets[regs_net].name.clone(), sdc_name);
        for &pin in &register_sinks {
            regs_clock.add_sink(self.sink(db, pin));
        }
        self.summary.sinks += regs_clock.sinks.len();
        let second = self.add_builder(regs_clock, input_net, Some(first));
        let builder = &mut self.builders[second];
        builder.tree_type = TreeType::Register;
        builder.top_buffer_name = top_buffer;
        builder.driving_net = Some(net);
        Ok(Some(first))
    }

    /// Moves the register sinks to a new net behind a root buffer, as
    /// `TritonCTS::forkRegisterClockNetwork`.
    fn fork_register_network(
        &mut self,
        db: &mut NetlistDB,
        net: NetId,
        net_name: &str,
        sdc_name: &str,
        sinks: &[PinId],
    ) -> Result<(NetId, String), CtsError> {
        let regs_name = format!("{net_name}_regs");
        let regs = db.add_net(&regs_name)?;
        db.nets[regs].signal_use = SignalUse::Clock;
        for &pin in sinks {
            db.move_pin(pin, regs);
        }
        let name = format!("clkbuf_regs_{}_{sdc_name}", self.reg_tree_index);
        self.reg_tree_index += 1;
        let center = sink_bbox_center(db, &db.nets[regs].pins.clone());
        let cell = self.make_buffer(db, self.setup.buffers.root, &name, center)?;
        let _ = self.connect_buffer(db, cell, self.setup.buffers.root, Some(net), Some(regs))?;
        Ok((regs, name))
    }

    fn make_buffer(
        &self,
        db: &mut NetlistDB,
        buffer: BufferCell,
        name: &str,
        at: Point<Dbu>,
    ) -> Result<CellId, CtsError> {
        let master = self
            .link
            .macro_of(buffer.id)
            .ok_or_else(|| CtsError::NoMaster(self.link.liberty.cell(buffer.id).name.clone()))?;
        let cell = db.add_cell(
            name.to_string(),
            master,
            at,
            Orientation::N,
            PlacementStatus::Placed,
        )?;
        Ok(cell)
    }

    /// Connects a new buffer's input and output.
    fn connect_buffer(
        &self,
        db: &mut NetlistDB,
        cell: CellId,
        buffer: BufferCell,
        input: Option<NetId>,
        output: Option<NetId>,
    ) -> Result<(Option<PinId>, Option<PinId>), CtsError> {
        let lib = self.link.liberty.cell(buffer.id);
        let pin_of = |port: LibPortId| {
            self.link
                .macro_pin_of_port(buffer.id, port)
                .ok_or_else(|| CtsError::MissingPin(lib.name.clone()))
        };
        let in_pin = match input {
            Some(net) => Some(db.connect_cell_pin(cell, pin_of(buffer.input)?, net)),
            None => None,
        };
        let out_pin = match output {
            Some(net) => Some(db.connect_cell_pin(cell, pin_of(buffer.output)?, net)),
            None => None,
        };
        Ok((in_pin, out_pin))
    }

    /// Clones each clock gate once for every group of its sinks that lies
    /// a long wire away from the others, and moves each gate to the middle
    /// of its group, as `TritonCTS::cloneClockGaters` and `findLongEdges`.
    fn clone_clock_gaters(
        &mut self,
        db: &mut NetlistDB,
        net: NetId,
        occupied: &mut FxHashSet<Point<Dbu>>,
    ) -> Result<(), CtsError> {
        let pins = net_cell_pins(db, net);
        let driver = pins.iter().copied().find(|&p| {
            pin_direction(db, p)
                .is_some_and(|(_, d, u)| d == PinDirection::Output && !u.is_supply())
        });
        let mut points: Vec<Point<Dbu>> = Vec::new();
        let mut point_pins: BTreeMap<Point<Dbu>, Vec<PinId>> = BTreeMap::new();
        for &pin in &pins {
            if Some(pin) == driver || !is_input_signal(db, pin) {
                continue;
            }
            let at = db.average_pin_position(pin);
            points.push(at);
            point_pins.entry(at).or_default().push(pin);
            if self.is_sink(db, pin) {
                continue;
            }
            let Some((cell, _, _)) = pin_direction(db, pin) else {
                continue;
            };
            let Some(output) = self.single_output(db, cell, pin) else {
                continue;
            };
            let Some((_, lib)) = self.link.cell(db, cell) else {
                continue;
            };
            if lib.is_inverter() || lib.is_buffer() {
                continue;
            }
            let out_net = db.pins[output].net;
            self.clone_clock_gaters(db, out_net, occupied)?;
        }
        let Some(driver) = driver else {
            return Ok(());
        };
        let Some((gate, _, _)) = pin_direction(db, driver) else {
            return Ok(());
        };
        let gate_cell = &db.cells[gate];
        if self.is_sink(db, driver)
            || gate_cell.status == PlacementStatus::Fixed
            || db.cell_macro(gate).class == MacroClass::Pad
            || points.is_empty()
        {
            return Ok(());
        }
        let driver_at = db.average_pin_position(driver);
        point_pins.entry(driver_at).or_default().push(driver);
        let tree = prim_dijkstra(&points, 0, DEFAULT_ALPHA);
        let clusters = long_edge_clusters(&tree, self.setup.settings.max_wire_length);
        let cluster_sinks: Vec<Vec<PinId>> = clusters
            .iter()
            .filter(|cluster| !cluster.is_empty())
            .map(|cluster| {
                cluster
                    .iter()
                    .flat_map(|&b| point_pins.get(&tree.nodes[b]).cloned().unwrap_or_default())
                    .filter(|&p| is_input_signal(db, p))
                    .collect()
            })
            .collect();
        let gate = GateToClone {
            net,
            cell: gate,
            driver,
            driver_at,
        };
        self.place_gate_clones(db, &gate, &cluster_sinks, occupied)
    }

    /// Gives each cluster of sinks a clone of `gate` at their center, the
    /// cluster nearest its driver keeping the gate itself, as
    /// `TritonCTS::cloneClockGaters`.
    fn place_gate_clones(
        &mut self,
        db: &mut NetlistDB,
        gate: &GateToClone,
        cluster_sinks: &[Vec<PinId>],
        occupied: &mut FxHashSet<Point<Dbu>>,
    ) -> Result<(), CtsError> {
        let driver_cluster = cluster_sinks
            .iter()
            .enumerate()
            .filter(|(_, sinks)| !sinks.is_empty())
            .min_by_key(|(_, sinks)| sink_bbox_center(db, sinks).manhattan(gate.driver_at))
            .map(|(n, _)| n);
        let mut clones = 0;
        for (n, sinks) in cluster_sinks.iter().enumerate() {
            let clone = if driver_cluster == Some(n) {
                gate.cell
            } else {
                clones += 1;
                self.clone_gate(db, gate, clones, sinks)?
            };
            let center = sink_bbox_center(db, sinks);
            let mut at = center;
            while occupied.contains(&at) {
                at.x += Dbu(1);
            }
            let _ = occupied.insert(at);
            let cell = &mut db.cells[clone];
            cell.location = at;
            cell.status = PlacementStatus::Placed;
        }
        Ok(())
    }

    /// Adds clone `number` of `gate`, its output on a net of its own that
    /// `sinks` move to, its inputs on the gate's nets.
    fn clone_gate(
        &mut self,
        db: &mut NetlistDB,
        gate: &GateToClone,
        number: usize,
        sinks: &[PinId],
    ) -> Result<CellId, CtsError> {
        let gate_name = db.cells[gate.cell].name.clone();
        let net_name = db.nets[gate.net].name.clone();
        let clone_net = db.add_net(&format!("clonenet_{number}_{net_name}"))?;
        db.nets[clone_net].signal_use = SignalUse::Clock;
        let _ = self.clock_nets.insert(clone_net);
        let master = db.cells[gate.cell].macro_id;
        let at = db.cells[gate.cell].location;
        let cell = db.add_cell(
            format!("clone_{number}_{gate_name}"),
            master,
            at,
            Orientation::N,
            PlacementStatus::Placed,
        )?;
        let connections: Vec<(MacroPinId, NetId)> = db.cells[gate.cell]
            .pins
            .iter()
            .filter(|&&p| p == gate.driver || is_input_signal(db, p))
            .filter_map(|&p| match db.pins[p].owner {
                PinOwner::Cell { pin, .. } => {
                    let target = if p == gate.driver {
                        clone_net
                    } else {
                        db.pins[p].net
                    };
                    Some((pin, target))
                }
                PinOwner::Io(_) => None,
            })
            .collect();
        for (pin, target) in connections {
            let _ = db.connect_cell_pin(cell, pin, target);
        }
        for &sink in sinks {
            db.move_pin(sink, clone_net);
        }
        self.summary.clones += 1;
        Ok(cell)
    }

    /// Builds every tree, as `TritonCTS::buildClockTrees`.
    fn build(&mut self, db: &NetlistDB, tech: &mut TechChar) -> Result<(), CtsError> {
        let mut sizes = FxHashMap::default();
        let buffers = &self.setup.buffers;
        for &b in buffers.all.iter().chain([&buffers.root, &buffers.sink]) {
            let master = self
                .link
                .macro_of(b.id)
                .ok_or_else(|| CtsError::NoMaster(self.link.liberty.cell(b.id).name.clone()))?;
            let _ = sizes.insert(b, db.library.cell_macro(master).size);
        }
        let root = sizes
            .get(&self.setup.buffers.root)
            .copied()
            .unwrap_or_default();
        let unit = f64::from(tech.length_unit());
        let half = |side: Dbu| Dbu(((side.get() as f64 / unit) * unit / 2.0).ceil() as i64);
        let bloat = Point::new(half(root.x), half(root.y));
        let (blockages, macro_diameter) = merge_blockages(db, bloat);
        if macro_diameter > 0.0 && self.setup.settings.macro_max_diameter.is_none() {
            self.setup.settings.macro_max_diameter = Some(macro_diameter.max(MACRO_MAX_DIAMETER));
        }
        self.setup.buffer_sizes = sizes;
        self.setup.blockages = blockages;
        let ctx = self.setup.context(db);
        for builder in &mut self.builders {
            builder.run(&ctx, tech);
        }
        Ok(())
    }

    /// Writes every tree into the design, as `TritonCTS::writeDataToDb`.
    fn write(&mut self, db: &mut NetlistDB) -> Result<(), CtsError> {
        for i in 0..self.builders.len() {
            prune_empty_subnets(&mut self.builders[i].clock);
            self.write_clock_nets(db, i)?;
            if self.setup.settings.ndr != NdrStrategy::None {
                self.write_ndr(db, i);
            }
            if self.setup.settings.dummy_loads {
                let n = self.write_dummy_loads(db, i)?;
                self.builders[i].n_dummies = n;
                self.summary.dummies += n;
            }
        }
        Ok(())
    }

    /// Creates a tree's buffers and nets, as `TritonCTS::writeClockNetsToDb`.
    fn write_clock_nets(&mut self, db: &mut NetlistDB, index: usize) -> Result<(), CtsError> {
        let top_net = self.builders[index].clock.net;
        let is_register = self.builders[index].tree_type == TreeType::Register;
        let regs_buffer = is_register
            .then(|| db.cell_id(&self.builders[index].top_buffer_name))
            .flatten();
        let mut buffers = Vec::new();
        for b in 0..self.builders[index].clock.buffers.len() {
            let inst = self.builders[index].clock.buffers[b].clone();
            let master = inst.master.unwrap_or(self.setup.buffers.root);
            let cell = self.make_buffer(db, master, &inst.name, inst.location)?;
            self.builders[index].clock.buffers[b].cell = Some(cell);
            buffers.push((cell, master));
        }
        self.summary.buffers += buffers.len();
        let clock_name = self.builders[index].clock.net_name.clone();
        let root_cell = db.cell_id(&format!("clkbuf_0_{clock_name}"));
        db.nets[top_net].signal_use = SignalUse::Clock;
        let subnet_count = self.builders[index].clock.subnets.len();
        let mut nets = Vec::with_capacity(subnet_count);
        for s in 0..subnet_count {
            let name = self.builders[index].clock.subnets[s].name.clone();
            let net = db.add_net(&name)?;
            db.nets[net].signal_use = SignalUse::Clock;
            self.builders[index].clock.subnets[s].net = Some(net);
            nets.push(net);
        }
        self.summary.nets += nets.len();
        for (s, &net) in nets.iter().enumerate() {
            let subnet = self.builders[index].clock.subnets[s].clone();
            let InstRef::Buffer(d) = subnet.driver() else {
                continue;
            };
            let (cell, master) = buffers[d];
            let _ = self.connect_buffer(db, cell, master, None, Some(net))?;
            for &sink in subnet.sinks() {
                match sink {
                    InstRef::Buffer(b) => {
                        let (cell, master) = buffers[b];
                        let (input, _) = self.connect_buffer(db, cell, master, Some(net), None)?;
                        self.builders[index].clock.buffers[b].input_pin = input;
                    }
                    InstRef::Sink(k) => {
                        if let Some(pin) = self.builders[index].clock.sinks[k].input_pin {
                            db.move_pin(pin, net);
                        }
                    }
                }
            }
        }
        if let Some(root) = root_cell {
            let master = buffers
                .iter()
                .find(|(c, _)| *c == root)
                .map_or(self.setup.buffers.root, |&(_, m)| m);
            let (input, _) = self.connect_buffer(db, root, master, Some(top_net), None)?;
            if let Some(pos) = buffers.iter().position(|(c, _)| *c == root) {
                self.builders[index].clock.buffers[pos].input_pin = input;
            }
        }
        if let (Some(regs), Some(driving)) = (regs_buffer, self.builders[index].driving_net)
            && let Some(input) = first_input(db, regs)
        {
            db.move_pin(input, driving);
        }
        Ok(())
    }

    /// Gives the upper levels of a tree the clock's non-default rule of
    /// doubled spacing, as `TritonCTS::writeClockNDRsToDb`.
    fn write_ndr(&mut self, db: &mut NetlistDB, index: usize) {
        let rule = add_clock_ndr(db);
        let clock = &self.builders[index].clock;
        let mut levels: Vec<u32> = clock
            .subnets
            .iter()
            .filter(|s| !s.leaf)
            .filter_map(|s| s.level)
            .collect();
        levels.sort_unstable();
        levels.dedup();
        let targets: Vec<u32> = match self.setup.settings.ndr {
            NdrStrategy::None => Vec::new(),
            NdrStrategy::RootOnly => vec![0],
            NdrStrategy::Half => levels[..levels.len().div_ceil(2)].to_vec(),
            NdrStrategy::Full => levels,
        };
        let mut applied = Vec::new();
        if targets.contains(&0) {
            applied.push(clock.net);
        }
        for subnet in &clock.subnets {
            if !subnet.leaf
                && subnet.level.is_some_and(|l| targets.contains(&l))
                && let Some(net) = subnet.net
            {
                applied.push(net);
            }
        }
        for &net in &applied {
            db.nets[net].non_default_rule = Some(rule);
        }
        self.summary.ndr_nets += applied.len();
    }

    /// Loads each buffer up to the heaviest load among its siblings with a
    /// dummy cell, as `TritonCTS::writeDummyLoadsToDb`.
    fn write_dummy_loads(&mut self, db: &mut NetlistDB, index: usize) -> Result<usize, CtsError> {
        let loads = self.buffer_loads(db, index);
        if !loads.unbalanced {
            return Ok(0);
        }
        let candidates = self.dummy_candidates();
        let mut inserted = 0;
        let subnet_count = self.builders[index].clock.subnets.len();
        for s in 0..subnet_count {
            let sinks: Vec<InstRef> = self.builders[index].clock.subnets[s].sinks().to_vec();
            for inst in sinks {
                let InstRef::Buffer(b) = inst else {
                    continue;
                };
                let output = loads.output.get(&inst).copied().unwrap_or_default();
                let target = loads.ideal.get(&inst).copied().unwrap_or_default();
                if sta_fuzzy_equal(output, target) {
                    continue;
                }
                let Some(&own) = loads.driven_subnet.get(&inst) else {
                    continue;
                };
                // NOTE: TritonCTS compares with its zero for no limit, so
                // without one only a sinkless subnet is passed over.
                let full = self.setup.settings.max_fanout.map_or(0, |f| f as usize);
                if self.builders[index].clock.subnets[own].sinks().len() == full {
                    continue;
                }
                let liberty = self.link.liberty;
                let delta = target - output;
                let Some(&dummy) = candidates.iter().min_by(|&&a, &&b| {
                    let da = (dummy_input_cap(liberty, a) - delta).abs();
                    let db_ = (dummy_input_cap(liberty, b) - delta).abs();
                    da.total_cmp(db_)
                }) else {
                    continue;
                };
                if self.add_dummy_load(db, index, (b, own), dummy)? {
                    inserted += 1;
                }
            }
        }
        Ok(inserted)
    }

    /// The load each buffer of tree `index` drives, the load it should, the
    /// heaviest among its siblings, and the subnet it drives.
    fn buffer_loads(&self, db: &NetlistDB, index: usize) -> BufferLoads {
        let first_input_cap = |inst: &crate::clock::ClockInst| -> Farads32 {
            let pin = if inst.kind == InstKind::Buffer {
                inst.cell.and_then(|c| first_input(db, c))
            } else {
                inst.input_pin
            };
            pin.map_or(Farads32::ZERO, |p| self.input_cap(db, p))
        };
        let clock = &self.builders[index].clock;
        let mut loads = BufferLoads::default();
        for (s, subnet) in clock.subnets.iter().enumerate() {
            let _ = loads.driven_subnet.insert(subnet.driver(), s);
            let total: Farads32 = subnet
                .sinks()
                .iter()
                .map(|&i| first_input_cap(clock.inst(i)))
                .sum();
            let _ = loads.output.insert(subnet.driver(), total);
        }
        for subnet in &clock.subnets {
            let mut max_cap = Farads32::MIN_POSITIVE;
            for &i in subnet.sinks() {
                if let InstRef::Buffer(_) = i {
                    let cap = loads.output.get(&i).copied().unwrap_or_default();
                    if cap > max_cap {
                        max_cap = cap;
                    }
                }
            }
            for &i in subnet.sinks() {
                if let InstRef::Buffer(_) = i {
                    let _ = loads.ideal.insert(i, max_cap);
                    if !sta_fuzzy_equal(loads.output.get(&i).copied().unwrap_or_default(), max_cap)
                    {
                        loads.unbalanced = true;
                    }
                }
            }
        }
        loads
    }

    /// Puts a `dummy` cell at buffer `b` of tree `index`, loading the
    /// subnet `own` it drives; whether that subnet had a net to load.
    fn add_dummy_load(
        &mut self,
        db: &mut NetlistDB,
        index: usize,
        (b, own): (usize, usize),
        dummy: BufferCell,
    ) -> Result<bool, CtsError> {
        let Some(net) = self.builders[index].clock.subnets[own].net else {
            return Ok(false);
        };
        let liberty = self.link.liberty;
        let location = self.builders[index].clock.buffers[b].location;
        let name = format!("clkload{}", self.dummy_index);
        self.dummy_index += 1;
        let cell = self.make_buffer(db, dummy, &name, location)?;
        let lib = liberty.cell(dummy.id);
        let pin = self
            .link
            .macro_pin_of_port(dummy.id, dummy.input)
            .ok_or_else(|| CtsError::MissingPin(lib.name.clone()))?;
        let input = db.connect_cell_pin(cell, pin, net);
        let clock = &mut self.builders[index].clock;
        let dummy_ref = clock.add_buffer(&name, dummy, location);
        clock.inst_mut(dummy_ref).cell = Some(cell);
        clock.inst_mut(dummy_ref).input_pin = Some(input);
        clock.subnets[own].add(dummy_ref);
        Ok(true)
    }

    /// The cells dummy loads are made of, by increasing input capacitance,
    /// as `TritonCTS::findCandidateDummyCells`: the clock buffers and the
    /// clock-cell inverters, else the inverters named `CLKINV`, else those
    /// named `INV`, else every candidate inverter.
    fn dummy_candidates(&self) -> Vec<BufferCell> {
        let liberty = self.link.liberty;
        let mut candidates = self.setup.buffers.all.clone();
        let inverters: Vec<BufferCell> = (0..liberty.cells.len())
            .map(|i| LibCellId(i as u32))
            .filter(|&id| {
                liberty.cell(id).is_inverter()
                    && self.link.is_link_cell(id)
                    && self.link.is_clock_cell_candidate(id)
            })
            .filter_map(|id| liberty.buffer_cell(id))
            .collect();
        let named = |pattern: &str| -> Vec<BufferCell> {
            inverters
                .iter()
                .copied()
                .filter(|b| {
                    liberty
                        .cell(b.id)
                        .name
                        .to_ascii_uppercase()
                        .contains(pattern)
                })
                .collect()
        };
        let clock_cells: Vec<BufferCell> = inverters
            .iter()
            .copied()
            .filter(|b| liberty.cell(b.id).is_clock_cell)
            .collect();
        let chosen = [clock_cells, named("CLKINV"), named("INV")]
            .into_iter()
            .find(|c| !c.is_empty())
            .unwrap_or(inverters);
        candidates.extend(chosen);
        candidates
            .sort_by(|&a, &b| dummy_input_cap(liberty, a).total_cmp(dummy_input_cap(liberty, b)));
        candidates
    }
}

/// The input capacitance of a dummy load cell.
fn dummy_input_cap(liberty: &Liberty, dummy: BufferCell) -> Farads32 {
    Farads32::of(liberty.cell(dummy.id).ports[dummy.input].capacitance())
}

/// `OpenSTA`'s `fuzzyEqual` on single-precision loads.
fn sta_fuzzy_equal(a: Farads32, b: Farads32) -> bool {
    pare_resizer::fuzzy::equal(f64::from(a.get()), f64::from(b.get()))
}

/// The center of the box around some pins' average positions, as
/// `dbNet::getTermBBox` and `findLongEdges` take it.
pub(crate) fn sink_bbox_center(db: &NetlistDB, sinks: &[PinId]) -> Point<Dbu> {
    let (mut lo, mut hi) = ((i64::MAX, i64::MAX), (i64::MIN, i64::MIN));
    for &s in sinks {
        let p = db.average_pin_position(s);
        lo = (lo.0.min(p.x.get()), lo.1.min(p.y.get()));
        hi = (hi.0.max(p.x.get()), hi.1.max(p.y.get()));
    }
    Point::new(
        Dbu(i64::midpoint(lo.0, hi.0)),
        Dbu(i64::midpoint(lo.1, hi.1)),
    )
}

/// Groups the nodes of a Steiner tree joined by edges shorter than
/// `threshold`, as `TritonCTS::findLongEdges` clusters them.
fn long_edge_clusters(tree: &SteinerTree, threshold: Option<Dbu>) -> Vec<Vec<usize>> {
    let threshold = threshold.unwrap_or(Dbu(0));
    let mut cluster_of: FxHashMap<usize, usize> = FxHashMap::default();
    let mut clusters: Vec<Vec<usize>> = Vec::new();
    for (b, n) in tree.branches() {
        let d = tree.nodes[b].manhattan(tree.nodes[n]);
        let from = cluster_of.get(&b).copied();
        let to = cluster_of.get(&n).copied();
        if d >= threshold {
            for node in [b, n] {
                if let std::collections::hash_map::Entry::Vacant(e) = cluster_of.entry(node) {
                    let _ = e.insert(clusters.len());
                    clusters.push(vec![node]);
                }
            }
            continue;
        }
        match (from, to) {
            (Some(f), Some(t)) => {
                let (keep, drop) = if clusters[f].len() >= clusters[t].len() {
                    (f, t)
                } else {
                    (t, f)
                };
                if keep != drop {
                    let moved = std::mem::take(&mut clusters[drop]);
                    for &node in &moved {
                        let _ = cluster_of.insert(node, keep);
                    }
                    clusters[keep].extend(moved);
                }
            }
            (Some(f), None) => {
                let _ = cluster_of.insert(n, f);
                clusters[f].push(n);
            }
            (None, Some(t)) => {
                let _ = cluster_of.insert(b, t);
                clusters[t].push(b);
            }
            (None, None) => {
                let _ = cluster_of.insert(b, clusters.len());
                let _ = cluster_of.insert(n, clusters.len());
                clusters.push(vec![b, n]);
            }
        }
    }
    clusters
}

/// The placed blocks and hard placement blockages, each group closer than a
/// buffer joined into the box around it, and the diameter macro clusters
/// may take, as `TreeBuilder::mergeBlockages`.
fn merge_blockages(db: &NetlistDB, bloat: Point<Dbu>) -> (Vec<Rect<Dbu>>, f64) {
    let mut rects: Vec<Rect<Dbu>> = Vec::new();
    let (mut max_dx, mut max_dy) = (0_i64, 0_i64);
    for (c, cell) in db.cells.iter().enumerate() {
        let id = CellId::new(c);
        if db.cell_macro(id).class == MacroClass::Block && cell.status != PlacementStatus::Unplaced
        {
            let bbox = db.cell_bbox(id);
            max_dx = max_dx.max(bbox.width().get());
            max_dy = max_dy.max(bbox.height().get());
            rects.push(bbox);
        }
    }
    rects.extend(db.hard_placement_blockages());
    let diameter = (2 * max_dx.max(max_dy)) as f64 / f64::from(db.dbu().get() as i32);
    let grown: Vec<Rect<Dbu>> = rects
        .iter()
        .map(|r| Rect::new(r.min - bloat, r.max + bloat))
        .collect();
    let mut group: Vec<usize> = (0..grown.len()).collect();
    for i in 0..grown.len() {
        for j in (i + 1)..grown.len() {
            let a = &grown[i];
            let b = &grown[j];
            let x_overlap = a.min.x.max(b.min.x) <= a.max.x.min(b.max.x);
            let y_overlap = a.min.y.max(b.min.y) <= a.max.y.min(b.max.y);
            let edge = (a.min.x.max(b.min.x) < a.max.x.min(b.max.x) && y_overlap)
                || (a.min.y.max(b.min.y) < a.max.y.min(b.max.y) && x_overlap);
            if edge {
                let (ri, rj) = (root(&mut group, i), root(&mut group, j));
                group[ri] = rj;
            }
        }
    }
    let mut boxes: BTreeMap<usize, Rect<Dbu>> = BTreeMap::new();
    for (i, rect) in rects.iter().enumerate() {
        let r = root(&mut group, i);
        let _ = boxes
            .entry(r)
            .and_modify(|b| *b = b.union(rect))
            .or_insert(*rect);
    }
    (
        boxes.into_values().collect(),
        if max_dx.max(max_dy) > 0 {
            diameter
        } else {
            0.0
        },
    )
}

/// The representative of `i`'s group, halving the path on the way.
const fn root(group: &mut [usize], mut i: usize) -> usize {
    while group[i] != i {
        group[i] = group[group[i]];
        i = group[i];
    }
    i
}

/// Drops subnets that reach no sink, with the buffers driving them, up the
/// tree as far as a buffer is left driving nothing, as `writeClockNetsToDb`
/// and `checkUpstreamConnections` destroy them.
fn prune_empty_subnets(clock: &mut Clock) {
    loop {
        let Some(empty) = clock.subnets.iter().position(|s| s.sinks().is_empty()) else {
            return;
        };
        let driver = clock.subnets[empty].driver();
        let _ = clock.subnets.remove(empty);
        for subnet in &mut clock.subnets {
            if subnet.insts.first() != Some(&driver) {
                subnet.insts.retain(|&i| i != driver);
            }
        }
    }
}

/// Adds a non-default rule doubling every routing layer's spacing at its
/// default width, as `TritonCTS::writeClockNDRsToDb` makes `CTS_NDR_*`.
fn add_clock_ndr(db: &mut NetlistDB) -> NonDefaultRuleId {
    let mut index = 0;
    let name = loop {
        let candidate = format!("CTS_NDR_{index}");
        if !db.non_default_rules.iter().any(|r| r.name == candidate) {
            break candidate;
        }
        index += 1;
    };
    let layers = db
        .library
        .routing_layers
        .iter_enumerated()
        .map(|(id, layer)| NonDefaultLayer {
            layer: id,
            width: layer.width,
            spacing: Some(net_spacing(layer, layer.width) * 2),
            wire_extension: None,
        })
        .collect();
    db.non_default_rules.push(NonDefaultRule {
        name,
        hard_spacing: false,
        layers,
        vias: Vec::new(),
        via_rules: Vec::new(),
        min_cuts: Vec::new(),
    })
}

/// The spacing two wires `width` wide need, as `TritonCTS::getNetSpacing`.
fn net_spacing(layer: &pare_common::db::library::RoutingLayer, width: Dbu) -> Dbu {
    let spacing = match &layer.spacing_table {
        Some(table) => table.spacing(width, width, Dbu::ZERO),
        None => layer
            .rules
            .width_range_spacing
            .iter()
            .filter(|r| width >= r.min_width && width <= r.max_width)
            .map(|r| r.spacing)
            .fold(layer.min_spacing, Dbu::max),
    };
    if spacing == Dbu::ZERO {
        layer.track_pitch() - layer.rules.min_width.unwrap_or(layer.width)
    } else {
        spacing
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tree rooted at node 0 over nodes at `xs` along the x axis.
    fn tree(xs: &[i64], parent: &[Option<usize>]) -> SteinerTree {
        SteinerTree {
            nodes: xs.iter().map(|&x| Point::new(Dbu(x), Dbu(0))).collect(),
            parent: parent.to_vec(),
            terminals: xs.len(),
            root: 0,
        }
    }

    #[test]
    fn without_a_wire_length_limit_every_node_is_its_own_cluster() {
        let tree = tree(&[0, 10, 20], &[None, Some(0), Some(1)]);

        let clusters = long_edge_clusters(&tree, None);

        assert_eq!(clusters, vec![vec![1], vec![0], vec![2]]);
    }

    #[test]
    fn short_edges_join_their_nodes_and_long_edges_part_them() {
        let tree = tree(&[0, 10, 1000, 1010], &[None, Some(0), Some(1), Some(2)]);

        let clusters = long_edge_clusters(&tree, Some(Dbu(100)));

        assert_eq!(clusters, vec![vec![1, 0], vec![2, 3]]);
    }

    #[test]
    fn an_edge_as_long_as_the_limit_is_long() {
        let tree = tree(&[0, 100], &[None, Some(0)]);

        let clusters = long_edge_clusters(&tree, Some(Dbu(100)));

        assert_eq!(clusters, vec![vec![1], vec![0]]);
    }

    #[test]
    fn a_short_edge_between_clusters_moves_the_smaller_into_the_larger() {
        // Node 4's edge to the root joins {1, 0} into {2, 3, 4}, leaving
        // the smaller cluster empty in its place.
        let tree = tree(
            &[0, 1, 2, 3, 4],
            &[None, Some(0), Some(3), Some(4), Some(0)],
        );

        let clusters = long_edge_clusters(&tree, Some(Dbu(100)));

        assert_eq!(clusters, vec![vec![], vec![2, 3, 4, 1, 0]]);
    }
}
