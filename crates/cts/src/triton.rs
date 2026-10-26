//! Clock tree synthesis over a design, as `TritonCTS::runTritonCts`: the
//! clock nets are found from the constraints, clock gates are cloned where
//! their sinks lie far apart, sinks with insertion delay get a tree of
//! their own, each net's tree is built, and the trees are written into the
//! design with their non-default rule and dummy loads.

use crate::CtsError;
use crate::buffers::{ClockBuffers, fanout_limit};
use crate::clock::{Clock, ClockSink, InstKind, InstRef};
use crate::htree::{Builder, TreeContext, TreeType};
use crate::options::{CtsOptions, NdrStrategy, Settings};
use crate::techchar::{CharSetup, TechChar};
use pare_common::db::design::{
    NetlistDB, NonDefaultLayer, NonDefaultRule, PinOwner, PlacementStatus,
};
use pare_common::db::indices::{CellId, LayerId, MacroPinId, NetId, NonDefaultRuleId, PinId};
use pare_common::db::liberty::LibCellId;
use pare_common::db::library::{MacroClass, PinDirection, SignalUse};
use pare_common::db::sdc::Sdc;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point as DbPoint;
use pare_common::geom::rect::Rect;
use pare_resizer::parasitics::WireRc;
use pare_resizer::steiner::{DEFAULT_ALPHA, prim_dijkstra};
use pare_resizer::{ClockNetwork, Link};
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BTreeMap;

/// What clock tree synthesis reads.
#[derive(Clone, Copy, Debug)]
pub struct CtsInput<'a> {
    /// The library link.
    pub link: &'a Link<'a>,
    /// The constraints whose clocks get trees.
    pub sdc: &'a Sdc,
    /// The clock network before synthesis.
    pub clocks: &'a ClockNetwork,
    /// The signal wire's resistance and capacitance, for buffer choice.
    pub signal_rc: WireRc,
    /// The clock wire's resistance and capacitance.
    pub clock_rc: WireRc,
    /// The longest wire worth driving whole, in database units, as the
    /// resizer's `findMaxWireLength` gives it; zero when unknown.
    pub max_wire_length: i64,
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
    pub(crate) settings: Settings,
    pub(crate) buffers: ClockBuffers,
    pub(crate) cap_per_dbu: f64,
    pub(crate) buffer_sizes: FxHashMap<LibCellId, (i64, i64)>,
    pub(crate) blockages: Vec<Rect<Dbu>>,
    /// What was done.
    pub summary: CtsSummary,
}

impl ClockTrees {
    /// The clock buffers, as inferred from the library.
    #[must_use]
    pub fn clock_buffers(&self) -> &[LibCellId] {
        &self.buffers.all
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
        && settings.num_max_leaf_sinks > root
    {
        settings.max_fanout = root;
    }
    if let Some(sink) = sink_fanout {
        settings.limit_sink_clustering_sizes(sink);
        if sink < settings.max_fanout {
            settings.max_fanout = sink;
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
        settings,
        buffers,
        builders: Vec::new(),
        buffer_sizes: FxHashMap::default(),
        blockages: Vec::new(),
        visited: FxHashSet::default(),
        net_builder: FxHashMap::default(),
        clock_nets: FxHashSet::default(),
        reg_tree_index: 0,
        dummy_index: 0,
        summary: CtsSummary::default(),
    };
    cts.summary
        .root_buffer
        .clone_from(&liberty.cell(cts.buffers.root).name);
    cts.summary
        .sink_buffer
        .clone_from(&liberty.cell(cts.buffers.sink).name);
    cts.populate(db, input.sdc)?;
    if !cts.builders.is_empty() {
        cts.build(db, &mut tech)?;
        cts.write(db)?;
    }
    Ok(ClockTrees {
        builders: cts.builders,
        settings: cts.settings,
        buffers: cts.buffers,
        cap_per_dbu: tech.cap_per_dbu(),
        buffer_sizes: cts.buffer_sizes,
        blockages: cts.blockages,
        summary: cts.summary,
    })
}

struct Cts<'a> {
    link: &'a Link<'a>,
    clocks: &'a ClockNetwork,
    settings: Settings,
    buffers: ClockBuffers,
    builders: Vec<Builder>,
    buffer_sizes: FxHashMap<LibCellId, (i64, i64)>,
    blockages: Vec<Rect<Dbu>>,
    visited: FxHashSet<NetId>,
    net_builder: FxHashMap<NetId, usize>,
    clock_nets: FxHashSet<NetId>,
    reg_tree_index: usize,
    dummy_index: usize,
    summary: CtsSummary,
}

/// A pin's position as `TritonCTS` takes it: the mean of its shapes'
/// centers, in integer arithmetic.
fn pin_position(db: &NetlistDB, pin: PinId) -> (i64, i64) {
    let shapes = db.pin_shapes(pin);
    if shapes.is_empty() {
        let p = db.pin_position(pin);
        return (p.x.get(), p.y.get());
    }
    let (mut x, mut y) = (0_i64, 0_i64);
    for s in &shapes {
        x += s.rect.min.x.get() + (s.rect.max.x.get() - s.rect.min.x.get()) / 2;
        y += s.rect.min.y.get() + (s.rect.max.y.get() - s.rect.min.y.get()) / 2;
    }
    let n = shapes.len() as i64;
    (x / n, y / n)
}

/// A pin's position as `dbITerm::getAvgXY` takes it: the mean of its
/// shapes' corners, in double precision, truncated.
pub(crate) fn avg_xy(db: &NetlistDB, pin: PinId) -> (i64, i64) {
    let shapes = db.pin_shapes(pin);
    if shapes.is_empty() {
        let p = db.pin_position(pin);
        return (p.x.get(), p.y.get());
    }
    let (mut x, mut y) = (0.0_f64, 0.0_f64);
    for s in &shapes {
        x += (s.rect.min.x.get() + s.rect.max.x.get()) as f64;
        y += (s.rect.min.y.get() + s.rect.max.y.get()) as f64;
    }
    let n = (2 * shapes.len()) as f64;
    ((x / n) as i32 as i64, (y / n) as i32 as i64)
}

/// The cell pins of a net in the order `OpenDB` lists them after reading
/// it from DEF, the last connected first.
pub(crate) fn net_cell_pins(db: &NetlistDB, net: NetId) -> Vec<PinId> {
    db.nets[net.index()]
        .pins
        .iter()
        .rev()
        .copied()
        .filter(|&p| matches!(db.pins[p.index()].owner, PinOwner::Cell { .. }))
        .collect()
}

pub(crate) fn pin_direction(
    db: &NetlistDB,
    pin: PinId,
) -> Option<(CellId, PinDirection, SignalUse)> {
    let PinOwner::Cell { cell, pin: mp } = db.pins[pin.index()].owner else {
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
    db.cells[cell.index()].status != PlacementStatus::Unplaced
}

pub(crate) fn is_block(db: &NetlistDB, cell: CellId) -> bool {
    db.cell_macro(cell).class == MacroClass::Block
}

/// The first signal output pin connected on an instance.
pub(crate) fn first_output(db: &NetlistDB, cell: CellId) -> Option<PinId> {
    db.cells[cell.index()].pins.iter().copied().find(|&p| {
        pin_direction(db, p).is_some_and(|(_, d, u)| d == PinDirection::Output && !u.is_supply())
    })
}

/// The first signal input pin connected on an instance.
pub(crate) fn first_input(db: &NetlistDB, cell: CellId) -> Option<PinId> {
    db.cells[cell.index()]
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

impl Cts<'_> {
    fn is_sink(&self, db: &NetlistDB, pin: PinId) -> bool {
        is_sink(self.link, db, pin)
    }

    fn input_cap(&self, db: &NetlistDB, pin: PinId) -> f32 {
        self.link.pin_port(db, pin).map_or(0.0, |(lib, port)| {
            self.link.liberty.cell(lib).ports[port].capacitance() as f32
        })
    }

    /// The output of a clock-network cell other than `input` whose net
    /// carries the clock on, as `TritonCTS::getSingleOutput`.
    fn single_output(&self, db: &NetlistDB, cell: CellId, input: PinId) -> Option<PinId> {
        db.cells[cell.index()].pins.iter().copied().find(|&p| {
            p != input
                && pin_direction(db, p).is_some_and(|(_, d, _)| d == PinDirection::Output)
                && self.clock_nets.contains(&db.pins[p.index()].net)
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
            let mut nets: Vec<NetId> = clock
                .sources
                .iter()
                .map(|io| db.io_pins[io.index()].net)
                .collect();
            nets.sort_unstable();
            nets.dedup();
            for net in &nets {
                if !seen.insert(*net) {
                    return Err(CtsError::OverlappingClock(clock.name.clone()));
                }
            }
            clock_roots.push((nets, clock.name.clone()));
        }
        let mut occupied: FxHashSet<(i64, i64)> = db
            .cells
            .iter()
            .map(|c| (c.location.x.get(), c.location.y.get()))
            .collect();
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
            let out_net = db.pins[output.index()].net;
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
    /// The sink record of a clock pin; its insertion delay is zero, as
    /// the libraries' clock tree path tables are not read.
    fn sink(&self, db: &NetlistDB, pin: PinId) -> ClockSink {
        ClockSink {
            name: db.pin_name(pin),
            location: pin_position(db, pin),
            pin,
            input_cap: self.input_cap(db, pin),
            insertion_delay: 0.0,
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
        let net_name = db.nets[net.index()].name.clone();
        let mut clock = Clock::new(&net_name, sdc_name);
        clock.driver_pin = db.nets[net.index()]
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
                    let out_net = db.pins[out.index()].net;
                    net_cell_pins(db, out_net)
                        .into_iter()
                        .rev()
                        .find(|&p| is_input_signal(db, p))
                });
                non_sink_macro &= inverted
                    .and_then(|p| pin_direction(db, p))
                    .is_some_and(|(c, _, _)| is_block(db, c));
            }
            if non_sink_macro || is_block(db, cell) {
                macro_sinks.push(pin);
            } else {
                register_sinks.push(pin);
            }
        }
        if !self.settings.insertion_delay || macro_sinks.is_empty() || register_sinks.is_empty() {
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
            clock.net = Some(net);
            return Ok(Some(self.add_builder(clock, input_net, parent)));
        }
        for &pin in &macro_sinks {
            clock.add_sink(self.sink(db, pin));
        }
        self.summary.sinks += clock.sinks.len();
        clock.net = Some(net);
        let first = self.add_builder(clock, input_net, parent);
        self.builders[first].tree_type = TreeType::Macro;
        let (regs_net, top_buffer) =
            self.fork_register_network(db, net, &net_name, sdc_name, &register_sinks)?;
        let mut regs_clock = Clock::new(&db.nets[regs_net.index()].name.clone(), sdc_name);
        for &pin in &register_sinks {
            regs_clock.add_sink(self.sink(db, pin));
        }
        self.summary.sinks += regs_clock.sinks.len();
        regs_clock.net = Some(regs_net);
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
        let regs = db
            .add_net(&regs_name)
            .map_err(|e| CtsError::Edit(e.to_string()))?;
        db.nets[regs.index()].signal_use = SignalUse::Clock;
        for &pin in sinks {
            db.move_pin(pin, regs);
        }
        let name = format!("clkbuf_regs_{}_{sdc_name}", self.reg_tree_index);
        self.reg_tree_index += 1;
        let center = sink_bbox_center(db, &db.nets[regs.index()].pins.clone());
        let cell = self.make_buffer(db, self.buffers.root, &name, center)?;
        let _ = self.connect_buffer(db, cell, self.buffers.root, Some(net), Some(regs))?;
        Ok((regs, name))
    }

    fn make_buffer(
        &self,
        db: &mut NetlistDB,
        buffer: LibCellId,
        name: &str,
        at: (i64, i64),
    ) -> Result<CellId, CtsError> {
        let master = self
            .link
            .macro_of(buffer)
            .ok_or_else(|| CtsError::NoMaster(self.link.liberty.cell(buffer).name.clone()))?;
        db.add_cell(
            name.to_string(),
            master,
            DbPoint::new(Dbu(at.0), Dbu(at.1)),
            Orientation::N,
            PlacementStatus::Placed,
        )
        .map_err(|e| CtsError::Edit(e.to_string()))
    }

    /// Connects a new buffer's input and output.
    fn connect_buffer(
        &self,
        db: &mut NetlistDB,
        cell: CellId,
        buffer: LibCellId,
        input: Option<NetId>,
        output: Option<NetId>,
    ) -> Result<(Option<PinId>, Option<PinId>), CtsError> {
        let lib = self.link.liberty.cell(buffer);
        let (in_port, out_port) = lib
            .buffer_ports()
            .ok_or_else(|| CtsError::NotABuffer(lib.name.clone()))?;
        let pin_of = |port: usize| {
            self.link
                .macro_pin_of_port(buffer, port)
                .map(MacroPinId::new)
                .ok_or_else(|| CtsError::NotABuffer(lib.name.clone()))
        };
        let in_pin = match input {
            Some(net) => Some(db.connect_cell_pin(cell, pin_of(in_port)?, net)),
            None => None,
        };
        let out_pin = match output {
            Some(net) => Some(db.connect_cell_pin(cell, pin_of(out_port)?, net)),
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
        occupied: &mut FxHashSet<(i64, i64)>,
    ) -> Result<(), CtsError> {
        let pins = net_cell_pins(db, net);
        let driver = pins.iter().copied().find(|&p| {
            pin_direction(db, p)
                .is_some_and(|(_, d, u)| d == PinDirection::Output && !u.is_supply())
        });
        let mut points: Vec<(i64, i64)> = Vec::new();
        let mut point_pins: BTreeMap<(i64, i64), Vec<PinId>> = BTreeMap::new();
        for &pin in &pins {
            if Some(pin) == driver || !is_input_signal(db, pin) {
                continue;
            }
            let at = avg_xy(db, pin);
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
            let out_net = db.pins[output.index()].net;
            self.clone_clock_gaters(db, out_net, occupied)?;
        }
        let Some(driver) = driver else {
            return Ok(());
        };
        let Some((gate, _, _)) = pin_direction(db, driver) else {
            return Ok(());
        };
        let gate_cell = &db.cells[gate.index()];
        if self.is_sink(db, driver)
            || gate_cell.status == PlacementStatus::Fixed
            || db.cell_macro(gate).class == MacroClass::Pad
            || points.is_empty()
        {
            return Ok(());
        }
        let driver_at = avg_xy(db, driver);
        point_pins.entry(driver_at).or_default().push(driver);
        let tree_points: Vec<DbPoint<Dbu>> = points
            .iter()
            .map(|&(x, y)| DbPoint::new(Dbu(x), Dbu(y)))
            .collect();
        let tree = prim_dijkstra(&tree_points, 0, DEFAULT_ALPHA);
        let clusters = long_edge_clusters(&tree.nodes, &tree.parent, self.settings.max_wire_length);
        self.place_gate_clones(
            db,
            net,
            gate,
            driver,
            &clusters,
            &tree.nodes,
            &point_pins,
            driver_at,
            occupied,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn place_gate_clones(
        &mut self,
        db: &mut NetlistDB,
        net: NetId,
        gate: CellId,
        driver: PinId,
        clusters: &[Vec<usize>],
        nodes: &[DbPoint<Dbu>],
        point_pins: &BTreeMap<(i64, i64), Vec<PinId>>,
        driver_at: (i64, i64),
        occupied: &mut FxHashSet<(i64, i64)>,
    ) -> Result<(), CtsError> {
        let node_sinks = |cluster: &[usize]| -> Vec<PinId> {
            cluster
                .iter()
                .flat_map(|&b| {
                    let at = (nodes[b].x.get(), nodes[b].y.get());
                    point_pins.get(&at).cloned().unwrap_or_default()
                })
                .filter(|&p| is_input_signal(db, p))
                .collect()
        };
        let cluster_sinks: Vec<Vec<PinId>> = clusters.iter().map(|c| node_sinks(c)).collect();
        let mut driver_cluster = None;
        let mut best = i64::MAX;
        for (n, sinks) in cluster_sinks.iter().enumerate() {
            if sinks.is_empty() {
                continue;
            }
            let center = sink_bbox_center(db, sinks);
            let d = (center.0 - driver_at.0).abs() + (center.1 - driver_at.1).abs();
            if d < best {
                driver_cluster = Some(n);
                best = d;
            }
        }
        let gate_name = db.cells[gate.index()].name.clone();
        let net_name = db.nets[net.index()].name.clone();
        let mut clones = 0;
        for (n, cluster) in clusters.iter().enumerate() {
            if cluster.is_empty() {
                continue;
            }
            let sinks = &cluster_sinks[n];
            let clone = if driver_cluster == Some(n) {
                gate
            } else {
                clones += 1;
                let clone_net = db
                    .add_net(&format!("clonenet_{clones}_{net_name}"))
                    .map_err(|e| CtsError::Edit(e.to_string()))?;
                db.nets[clone_net.index()].signal_use = SignalUse::Clock;
                let _ = self.clock_nets.insert(clone_net);
                let master = db.cells[gate.index()].macro_id;
                let at = db.cells[gate.index()].location;
                let cell = db
                    .add_cell(
                        format!("clone_{clones}_{gate_name}"),
                        master,
                        at,
                        Orientation::N,
                        PlacementStatus::Placed,
                    )
                    .map_err(|e| CtsError::Edit(e.to_string()))?;
                let connections: Vec<(MacroPinId, NetId)> = db.cells[gate.index()]
                    .pins
                    .iter()
                    .filter(|&&p| p == driver || is_input_signal(db, p))
                    .filter_map(|&p| match db.pins[p.index()].owner {
                        PinOwner::Cell { pin, .. } => {
                            let target = if p == driver {
                                clone_net
                            } else {
                                db.pins[p.index()].net
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
                cell
            };
            let center = sink_bbox_center(db, sinks);
            let mut at = center;
            while occupied.contains(&at) {
                at.0 += 1;
            }
            let _ = occupied.insert(at);
            let cell = &mut db.cells[clone.index()];
            cell.location = DbPoint::new(Dbu(at.0), Dbu(at.1));
            cell.status = PlacementStatus::Placed;
        }
        Ok(())
    }

    fn tree_context<'b>(
        &'b self,
        db: &NetlistDB,
        sizes: &'b FxHashMap<LibCellId, (i64, i64)>,
        blockages: &'b [Rect<Dbu>],
    ) -> TreeContext<'b> {
        TreeContext {
            settings: &self.settings,
            core: db.core_area(),
            dbu: db.dbu().get() as i32,
            root_buffer: self.buffers.root,
            sink_buffer: self.buffers.sink,
            buffer_sizes: sizes,
            blockages,
        }
    }

    /// Builds every tree, as `TritonCTS::buildClockTrees`.
    fn build(&mut self, db: &NetlistDB, tech: &mut TechChar) -> Result<(), CtsError> {
        let mut sizes = FxHashMap::default();
        for &b in self
            .buffers
            .all
            .iter()
            .chain([&self.buffers.root, &self.buffers.sink])
        {
            let master = self
                .link
                .macro_of(b)
                .ok_or_else(|| CtsError::NoMaster(self.link.liberty.cell(b).name.clone()))?;
            let size = db.library.cell_macro(master).size;
            let _ = sizes.insert(b, (size.x.get(), size.y.get()));
        }
        let (root_w, root_h) = sizes.get(&self.buffers.root).copied().unwrap_or((0, 0));
        let unit = f64::from(tech.length_unit());
        let bloat = (
            ((root_w as f64 / unit) * unit / 2.0).ceil() as i64,
            ((root_h as f64 / unit) * unit / 2.0).ceil() as i64,
        );
        let (blockages, macro_diameter) = merge_blockages(db, bloat);
        if macro_diameter > 0.0 && !self.settings.macro_max_diameter_set {
            self.settings.macro_max_diameter = macro_diameter.max(self.settings.macro_max_diameter);
            self.settings.macro_max_diameter_set = true;
        }
        let mut builders = std::mem::take(&mut self.builders);
        {
            let ctx = self.tree_context(db, &sizes, &blockages);
            for builder in &mut builders {
                builder.run(&ctx, tech);
            }
        }
        self.builders = builders;
        self.buffer_sizes = sizes;
        self.blockages = blockages;
        Ok(())
    }

    /// Writes every tree into the design, as `TritonCTS::writeDataToDb`.
    fn write(&mut self, db: &mut NetlistDB) -> Result<(), CtsError> {
        for i in 0..self.builders.len() {
            prune_empty_subnets(&mut self.builders[i].clock);
            self.write_clock_nets(db, i)?;
            if self.settings.ndr != NdrStrategy::None {
                self.write_ndr(db, i);
            }
            if self.settings.dummy_loads {
                let n = self.write_dummy_loads(db, i)?;
                self.builders[i].n_dummies = n;
                self.summary.dummies += n;
            }
        }
        Ok(())
    }

    /// Creates a tree's buffers and nets, as `TritonCTS::writeClockNetsToDb`.
    fn write_clock_nets(&mut self, db: &mut NetlistDB, index: usize) -> Result<(), CtsError> {
        let top_net = self.builders[index]
            .clock
            .net
            .ok_or_else(|| CtsError::Edit("clock tree without a net".into()))?;
        let is_register = self.builders[index].tree_type == TreeType::Register;
        let regs_buffer = is_register
            .then(|| db.cell_id(&self.builders[index].top_buffer_name))
            .flatten();
        let mut buffers = Vec::new();
        for b in 0..self.builders[index].clock.buffers.len() {
            let inst = self.builders[index].clock.buffers[b].clone();
            let master = inst.master.unwrap_or(self.buffers.root);
            let cell = self.make_buffer(db, master, &inst.name, inst.location)?;
            self.builders[index].clock.buffers[b].cell = Some(cell);
            buffers.push((cell, master));
        }
        self.summary.buffers += buffers.len();
        let clock_name = self.builders[index].clock.net_name.clone();
        let root_cell = db.cell_id(&format!("clkbuf_0_{clock_name}"));
        db.nets[top_net.index()].signal_use = SignalUse::Clock;
        let subnet_count = self.builders[index].clock.subnets.len();
        let mut nets = Vec::with_capacity(subnet_count);
        for s in 0..subnet_count {
            let name = self.builders[index].clock.subnets[s].name.clone();
            let net = db
                .add_net(&name)
                .map_err(|e| CtsError::Edit(e.to_string()))?;
            db.nets[net.index()].signal_use = SignalUse::Clock;
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
                .map_or(self.buffers.root, |&(_, m)| m);
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
        let targets: Vec<u32> = match self.settings.ndr {
            NdrStrategy::None => Vec::new(),
            NdrStrategy::RootOnly => vec![0],
            NdrStrategy::Half => levels[..levels.len().div_ceil(2)].to_vec(),
            NdrStrategy::Full => levels,
        };
        let mut applied = Vec::new();
        if targets.contains(&0)
            && let Some(net) = clock.net
        {
            applied.push(net);
        }
        for subnet in &clock.subnets {
            if !subnet.leaf
                && subnet.level.is_some_and(|l| targets.contains(&l))
                && let Some(net) = subnet.net
            {
                applied.push(net);
            }
        }
        for net in &applied {
            db.nets[net.index()].non_default_rule = Some(rule);
        }
        self.summary.ndr_nets += applied.len();
    }

    /// Loads each buffer up to the heaviest load among its siblings with a
    /// dummy cell, as `TritonCTS::writeDummyLoadsToDb`.
    fn write_dummy_loads(&mut self, db: &mut NetlistDB, index: usize) -> Result<usize, CtsError> {
        let liberty = self.link.liberty;
        let first_input_cap = |cts: &Self, db: &NetlistDB, inst: &crate::clock::ClockInst| -> f32 {
            let pin = if inst.kind == InstKind::Buffer {
                inst.cell.and_then(|c| first_input(db, c))
            } else {
                inst.input_pin
            };
            pin.map_or(0.0, |p| cts.input_cap(db, p))
        };
        let clock = &self.builders[index].clock;
        let mut driver_subnet: FxHashMap<InstRef, usize> = FxHashMap::default();
        let mut output_caps: FxHashMap<InstRef, f32> = FxHashMap::default();
        for (s, subnet) in clock.subnets.iter().enumerate() {
            let _ = driver_subnet.insert(subnet.driver(), s);
            let total: f32 = subnet
                .sinks()
                .iter()
                .map(|&i| first_input_cap(self, db, clock.inst(i)))
                .sum();
            let _ = output_caps.insert(subnet.driver(), total);
        }
        let mut need = false;
        let mut ideal: FxHashMap<InstRef, f32> = FxHashMap::default();
        for subnet in &clock.subnets {
            let mut max_cap = f32::MIN_POSITIVE;
            for &i in subnet.sinks() {
                if let InstRef::Buffer(_) = i {
                    let cap = output_caps.get(&i).copied().unwrap_or(0.0);
                    if cap > max_cap {
                        max_cap = cap;
                    }
                }
            }
            for &i in subnet.sinks() {
                if let InstRef::Buffer(_) = i {
                    let _ = ideal.insert(i, max_cap);
                    if !sta_fuzzy_equal(output_caps.get(&i).copied().unwrap_or(0.0), max_cap) {
                        need = true;
                    }
                }
            }
        }
        if !need {
            return Ok(0);
        }
        let candidates = self.dummy_candidates();
        let mut inserted = 0;
        let subnet_count = clock.subnets.len();
        for s in 0..subnet_count {
            let sinks: Vec<InstRef> = self.builders[index].clock.subnets[s].sinks().to_vec();
            for inst in sinks {
                let InstRef::Buffer(b) = inst else {
                    continue;
                };
                let output = output_caps.get(&inst).copied().unwrap_or(0.0);
                let target = ideal.get(&inst).copied().unwrap_or(0.0);
                if sta_fuzzy_equal(output, target) {
                    continue;
                }
                let Some(&own) = driver_subnet.get(&inst) else {
                    continue;
                };
                if self.builders[index].clock.subnets[own].sinks().len()
                    == self.settings.max_fanout as usize
                {
                    continue;
                }
                let delta = target - output;
                let Some(&dummy) = candidates.iter().min_by(|&&a, &&b| {
                    let da = (dummy_input_cap(liberty, a) - delta).abs();
                    let db_ = (dummy_input_cap(liberty, b) - delta).abs();
                    da.total_cmp(&db_)
                }) else {
                    continue;
                };
                let location = self.builders[index].clock.buffers[b].location;
                let name = format!("clkload{}", self.dummy_index);
                self.dummy_index += 1;
                let cell = self.make_buffer(db, dummy, &name, location)?;
                let Some(net) = self.builders[index].clock.subnets[own].net else {
                    continue;
                };
                let lib = liberty.cell(dummy);
                let (in_port, _) = lib
                    .buffer_ports()
                    .ok_or_else(|| CtsError::NotABuffer(lib.name.clone()))?;
                let pin = self
                    .link
                    .macro_pin_of_port(dummy, in_port)
                    .map(MacroPinId::new)
                    .ok_or_else(|| CtsError::NotABuffer(lib.name.clone()))?;
                let input = db.connect_cell_pin(cell, pin, net);
                let clock = &mut self.builders[index].clock;
                let dummy_ref = clock.add_buffer(&name, dummy, location);
                clock.inst_mut(dummy_ref).cell = Some(cell);
                clock.inst_mut(dummy_ref).input_pin = Some(input);
                clock.subnets[own].add(dummy_ref);
                inserted += 1;
            }
        }
        Ok(inserted)
    }

    /// The cells dummy loads are made of, by increasing input capacitance,
    /// as `TritonCTS::findCandidateDummyCells`: the clock buffers and the
    /// clock-cell inverters, else the inverters named `CLKINV`, else those
    /// named `INV`, else every candidate inverter.
    fn dummy_candidates(&self) -> Vec<LibCellId> {
        let liberty = self.link.liberty;
        let mut candidates = self.buffers.all.clone();
        let inverters: Vec<LibCellId> = (0..liberty.cells.len())
            .map(|i| LibCellId(i as u32))
            .filter(|&id| {
                liberty.cell(id).is_inverter()
                    && self.link.is_link_cell(id)
                    && self.link.is_clock_cell_candidate(id)
            })
            .collect();
        let named = |pattern: &str| -> Vec<LibCellId> {
            inverters
                .iter()
                .copied()
                .filter(|&id| liberty.cell(id).name.to_ascii_uppercase().contains(pattern))
                .collect()
        };
        let clock_cells: Vec<LibCellId> = inverters
            .iter()
            .copied()
            .filter(|&id| liberty.cell(id).is_clock_cell)
            .collect();
        let chosen = [clock_cells, named("CLKINV"), named("INV")]
            .into_iter()
            .find(|c| !c.is_empty())
            .unwrap_or(inverters);
        candidates.extend(chosen);
        candidates
            .sort_by(|&a, &b| dummy_input_cap(liberty, a).total_cmp(&dummy_input_cap(liberty, b)));
        candidates
    }
}

fn dummy_input_cap(liberty: &pare_common::db::liberty::Liberty, cell: LibCellId) -> f32 {
    let lib = liberty.cell(cell);
    lib.buffer_ports()
        .map_or(0.0, |(input, _)| lib.ports[input].capacitance() as f32)
}

/// `OpenSTA`'s `fuzzyEqual` on single-precision values.
fn sta_fuzzy_equal(a: f32, b: f32) -> bool {
    pare_resizer::fuzzy::equal(f64::from(a), f64::from(b))
}

/// The center of the box around some pins' average positions, as
/// `dbNet::getTermBBox` and `findLongEdges` take it.
fn sink_bbox_center(db: &NetlistDB, sinks: &[PinId]) -> (i64, i64) {
    let (mut lo, mut hi) = ((i64::MAX, i64::MAX), (i64::MIN, i64::MIN));
    for &s in sinks {
        let p = avg_xy(db, s);
        lo = (lo.0.min(p.0), lo.1.min(p.1));
        hi = (hi.0.max(p.0), hi.1.max(p.1));
    }
    (i64::midpoint(lo.0, hi.0), i64::midpoint(lo.1, hi.1))
}

/// Groups the nodes of a Steiner tree joined by edges shorter than
/// `threshold`, as `TritonCTS::findLongEdges` clusters them.
fn long_edge_clusters(nodes: &[DbPoint<Dbu>], parent: &[usize], threshold: i64) -> Vec<Vec<usize>> {
    let mut cluster_of: FxHashMap<usize, usize> = FxHashMap::default();
    let mut clusters: Vec<Vec<usize>> = Vec::new();
    for b in 0..nodes.len() {
        let n = parent[b];
        if b == n {
            continue;
        }
        let d = (nodes[b].x - nodes[n].x).get().abs() + (nodes[b].y - nodes[n].y).get().abs();
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
fn merge_blockages(db: &NetlistDB, bloat: (i64, i64)) -> (Vec<Rect<Dbu>>, f64) {
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
    rects.extend(db.placement_blockages.iter().copied());
    let diameter = (2 * max_dx.max(max_dy)) as f64 / f64::from(db.dbu().get() as i32);
    let grown: Vec<Rect<Dbu>> = rects
        .iter()
        .map(|r| {
            Rect::new(
                DbPoint::new(r.min.x - Dbu(bloat.0), r.min.y - Dbu(bloat.1)),
                DbPoint::new(r.max.x + Dbu(bloat.0), r.max.y + Dbu(bloat.1)),
            )
        })
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
        .iter()
        .enumerate()
        .map(|(i, layer)| NonDefaultLayer {
            layer: LayerId::new(i),
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
    });
    NonDefaultRuleId::new(db.non_default_rules.len() - 1)
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
