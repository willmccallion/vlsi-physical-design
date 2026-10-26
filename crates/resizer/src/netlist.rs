//! Views of the design the repair steps share: which pins drive and which
//! load, which pins carry a clock, which hold a constant, and names for
//! what repair adds.

use crate::library::Link;
use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{CellId, IoPinId, NetId, PinId};
use pare_common::db::liberty::{ArcRole, ClockGatePin, FuncExpr, LibCell, LibPort, Logic};
use pare_common::db::library::{PinDirection, SignalUse};
use pare_common::db::sdc::{ClockId, Sdc};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use std::collections::VecDeque;

/// Why a view of the netlist could not be built.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum NetlistError {
    /// A clock port named in the configuration is not a pin of the design.
    #[error("clock port '{0}' is not an I/O pin of the design")]
    UnknownClockPort(String),
}

/// The design with its library link.
#[derive(Clone, Copy, Debug)]
pub struct Netlist<'a> {
    /// The design.
    pub db: &'a NetlistDB,
    /// Its masters' Liberty cells.
    pub link: &'a Link<'a>,
}

impl<'a> Netlist<'a> {
    /// A view over `db` with `link`.
    #[must_use]
    pub const fn new(db: &'a NetlistDB, link: &'a Link<'a>) -> Self {
        Self { db, link }
    }

    /// The Liberty port of an instance pin.
    #[must_use]
    pub fn lib_port(&self, pin: PinId) -> Option<&'a LibPort> {
        self.lib_cell_port(pin).map(|(_, port)| port)
    }

    /// The Liberty cell and port of an instance pin.
    #[must_use]
    pub fn lib_cell_port(&self, pin: PinId) -> Option<(&'a LibCell, &'a LibPort)> {
        let (cell, port) = self.link.pin_port(self.db, pin)?;
        let lib = self.link.liberty.cell(cell);
        Some((lib, &lib.ports[port]))
    }

    /// Whether a pin belongs to a retired instance, which timing no longer
    /// sees.
    #[must_use]
    pub fn is_retired(&self, pin: PinId) -> bool {
        match self.db.pins[pin.index()].owner {
            PinOwner::Cell { cell, .. } => self.db.cells[cell.index()].retired,
            PinOwner::Io(_) => false,
        }
    }

    /// Whether a pin drives its net: an output of an instance or an input
    /// of the design.
    #[must_use]
    pub fn is_driver(&self, pin: PinId) -> bool {
        if self.is_retired(pin) {
            return false;
        }
        match self.db.pins[pin.index()].owner {
            PinOwner::Cell {
                cell,
                pin: macro_pin,
            } => self.lib_port(pin).map_or_else(
                || {
                    matches!(
                        self.db.cell_macro(cell).pin(macro_pin).direction,
                        PinDirection::Output | PinDirection::Inout
                    )
                },
                LibPort::is_output,
            ),
            PinOwner::Io(io) => matches!(
                self.db.io_pins[io.index()].direction,
                PinDirection::Input | PinDirection::Inout
            ),
        }
    }

    /// Whether a pin is driven by its net: an input of an instance or an
    /// output of the design.
    #[must_use]
    pub fn is_load(&self, pin: PinId) -> bool {
        if self.is_retired(pin) {
            return false;
        }
        match self.db.pins[pin.index()].owner {
            PinOwner::Cell {
                cell,
                pin: macro_pin,
            } => self.lib_port(pin).map_or_else(
                || {
                    matches!(
                        self.db.cell_macro(cell).pin(macro_pin).direction,
                        PinDirection::Input | PinDirection::Inout
                    )
                },
                LibPort::is_input,
            ),
            PinOwner::Io(io) => matches!(
                self.db.io_pins[io.index()].direction,
                PinDirection::Output | PinDirection::Inout
            ),
        }
    }

    /// The pins driving `net`.
    pub fn drivers(&self, net: NetId) -> impl Iterator<Item = PinId> + '_ {
        self.db.nets[net.index()]
            .pins
            .iter()
            .copied()
            .filter(move |&p| self.is_driver(p))
    }

    /// The pins `net` drives.
    pub fn loads(&self, net: NetId) -> impl Iterator<Item = PinId> + '_ {
        self.db.nets[net.index()]
            .pins
            .iter()
            .copied()
            .filter(move |&p| self.is_load(p))
    }

    /// Whether `net` connects to a pin of the design.
    #[must_use]
    pub fn has_port(&self, net: NetId) -> bool {
        self.db.nets[net.index()]
            .pins
            .iter()
            .any(|&p| matches!(self.db.pins[p.index()].owner, PinOwner::Io(_)))
    }

    /// Whether `net` connects to a pin of an instance.
    #[must_use]
    pub fn has_instance_pins(&self, net: NetId) -> bool {
        self.db.nets[net.index()]
            .pins
            .iter()
            .any(|&p| matches!(self.db.pins[p.index()].owner, PinOwner::Cell { .. }))
    }

    /// Whether `net` carries power or ground, which repair leaves alone.
    #[must_use]
    pub fn is_special(&self, net: NetId) -> bool {
        matches!(
            self.db.nets[net.index()].signal_use,
            SignalUse::Power | SignalUse::Ground
        )
    }

    /// Where a pin is, as `odb` places it: the mean of the centers of an
    /// instance pin's shapes, the center of a design pin's first shape.
    #[must_use]
    pub fn pin_location(&self, pin: PinId) -> Point<Dbu> {
        let shapes = self.db.pin_shapes(pin);
        if shapes.is_empty() {
            return self.db.pin_position(pin);
        }
        if matches!(self.db.pins[pin.index()].owner, PinOwner::Io(_)) {
            return shapes[0].rect.center();
        }
        let n = shapes.len() as i64;
        let (sx, sy) = shapes.iter().fold((0i64, 0i64), |(sx, sy), s| {
            (
                sx + s.rect.min.x.get() + s.rect.max.x.get(),
                sy + s.rect.min.y.get() + s.rect.max.y.get(),
            )
        });
        Point::new(Dbu(sx / (2 * n)), Dbu(sy / (2 * n)))
    }

    /// The pin of `instance` for Liberty port `port`, if connected.
    #[must_use]
    pub fn pin_of_port(&self, instance: CellId, port: usize) -> Option<PinId> {
        let (lib, _) = self.link.cell(self.db, instance)?;
        let macro_pin = self.link.macro_pin_of_port(lib, port)?;
        self.db.cells[instance.index()]
            .pins
            .iter()
            .copied()
            .find(|&p| matches!(self.db.pins[p.index()].owner, PinOwner::Cell { pin, .. } if pin.index() == macro_pin))
    }

    /// The level of each pin of `instance` by Liberty port name, from
    /// `levels`.
    fn port_levels(&self, instance: CellId, levels: &[Logic]) -> impl Fn(&str) -> Logic + '_ {
        let cell = self.db.cells[instance.index()].clone();
        let link = self.link;
        let db = self.db;
        let levels: Vec<Logic> = levels.to_vec();
        move |name: &str| {
            cell.pins
                .iter()
                .find(|&&p| {
                    link.pin_port(db, p)
                        .is_some_and(|(c, port)| link.liberty.cell(c).ports[port].name == name)
                })
                .map_or(Logic::Unknown, |&p| levels[p.index()])
        }
    }
}

/// The pins that hold a constant level: the outputs of tie cells and
/// whatever logic they settle, as `OpenSTA`'s `Sim` finds them without
/// looking through registers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Constants {
    level: Vec<Logic>,
}

impl Constants {
    /// Propagates constants through `netlist`.
    #[must_use]
    pub fn find(netlist: &Netlist<'_>) -> Self {
        let db = netlist.db;
        let mut level = vec![Logic::Unknown; db.pins.len()];
        let mut queue: VecDeque<CellId> = VecDeque::new();
        for cell in &db.cells {
            for &pin in &cell.pins {
                let Some(port) = netlist.lib_port(pin) else {
                    continue;
                };
                if port.three_state.is_some() {
                    continue;
                }
                let value = match port.function {
                    Some(FuncExpr::Zero) => Logic::Zero,
                    Some(FuncExpr::One) => Logic::One,
                    _ => continue,
                };
                Self::set(netlist, &mut level, &mut queue, pin, value);
            }
        }
        while let Some(instance) = queue.pop_front() {
            Self::eval_instance(netlist, &mut level, &mut queue, instance);
        }
        Self { level }
    }

    fn set(
        netlist: &Netlist<'_>,
        level: &mut [Logic],
        queue: &mut VecDeque<CellId>,
        pin: PinId,
        value: Logic,
    ) {
        if level[pin.index()] == value {
            return;
        }
        level[pin.index()] = value;
        let db = netlist.db;
        if let PinOwner::Cell { cell, .. } = db.pins[pin.index()].owner
            && netlist.is_load(pin)
            && queue.back() != Some(&cell)
        {
            queue.push_back(cell);
        }
        if netlist.is_driver(pin) {
            let net = db.pins[pin.index()].net;
            let loads: Vec<PinId> = netlist.loads(net).filter(|&p| p != pin).collect();
            for load in loads {
                Self::set(netlist, level, queue, load, value);
            }
        }
    }

    fn eval_instance(
        netlist: &Netlist<'_>,
        level: &mut [Logic],
        queue: &mut VecDeque<CellId>,
        instance: CellId,
    ) {
        let db = netlist.db;
        let Some((_, cell)) = netlist.link.cell(db, instance) else {
            return;
        };
        let port_level = netlist.port_levels(instance, level);
        let outputs: Vec<PinId> = db.cells[instance.index()]
            .pins
            .iter()
            .copied()
            .filter(|&p| netlist.lib_port(p).is_some_and(LibPort::is_output))
            .collect();
        for pin in outputs {
            let Some(port) = netlist.lib_port(pin) else {
                continue;
            };
            let value = if let Some(function) = &port.function {
                match &port.three_state {
                    Some(enable) if enable.eval(&port_level) != Logic::One => Logic::Unknown,
                    _ => function.eval(&port_level),
                }
            } else if port.clock_gate == ClockGatePin::Out {
                let gated_off = cell.ports.iter().any(|p| {
                    matches!(p.clock_gate, ClockGatePin::Clock | ClockGatePin::Enable)
                        && port_level(&p.name) == Logic::Zero
                });
                if gated_off {
                    Logic::Zero
                } else {
                    Logic::Unknown
                }
            } else {
                Logic::Unknown
            };
            if value != level[pin.index()] {
                Self::set(netlist, level, queue, pin, value);
            }
        }
    }

    /// The level a pin holds; pins added since the search are unknown.
    #[must_use]
    pub fn level(&self, pin: PinId) -> Logic {
        self.level
            .get(pin.index())
            .copied()
            .unwrap_or(Logic::Unknown)
    }

    /// Whether a pin holds a constant.
    #[must_use]
    pub fn is_constant(&self, pin: PinId) -> bool {
        self.level(pin).is_constant()
    }

    /// Whether the arc from `from_port` to `to_port` of `instance` can
    /// switch its output, given the constants on its pins: false where
    /// `OpenSTA` finds the arc's timing sense to be none, because the input
    /// is constant, the output's function reads the input but no longer
    /// depends on it, or a tristate enable is held low.
    #[must_use]
    pub fn arc_enabled(
        &self,
        netlist: &Netlist<'_>,
        instance: CellId,
        from_port: usize,
        to_port: usize,
    ) -> bool {
        let Some((_, cell)) = netlist.link.cell(netlist.db, instance) else {
            return true;
        };
        if netlist
            .pin_of_port(instance, from_port)
            .is_some_and(|pin| self.is_constant(pin))
        {
            return false;
        }
        let Some(function) = &cell.ports[to_port].function else {
            return true;
        };
        let from_name = cell.ports[from_port].name.as_str();
        if !function.ports().contains(&from_name) {
            return true;
        }
        let port_level = netlist.port_levels(instance, &self.level);
        if let Some(enable) = &cell.ports[to_port].three_state
            && enable.eval(&port_level) == Logic::Zero
        {
            return false;
        }
        function.depends_on(from_name, &port_level)
    }
}

/// The pins each clock reaches from its sources through wires and
/// combinational arcs, as `OpenSTA`'s clock network.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClockNetwork {
    clocks_of: Vec<Vec<ClockId>>,
    sources: Vec<PinId>,
    /// Whether each clock is propagated, by clock id.
    propagated: Vec<bool>,
}

impl ClockNetwork {
    /// Finds the pins the clocks entering at `clock_ports` reach, each
    /// port a clock of its own, for a design without constraints.
    pub fn from_ports(
        netlist: &Netlist<'_>,
        clock_ports: &[String],
        constants: &Constants,
    ) -> Result<Self, NetlistError> {
        let db = netlist.db;
        let mut clocks = Vec::new();
        for (i, name) in clock_ports.iter().enumerate() {
            let io = db
                .io_pin_id(name)
                .ok_or_else(|| NetlistError::UnknownClockPort(name.clone()))?;
            clocks.push((ClockId(i as u32), vec![io]));
        }
        Ok(Self::find(netlist, &clocks, constants))
    }

    /// Finds the pins the clocks of `sdc` reach.
    #[must_use]
    pub fn from_sdc(netlist: &Netlist<'_>, sdc: &Sdc, constants: &Constants) -> Self {
        let clocks: Vec<(ClockId, Vec<IoPinId>)> = sdc
            .clocks
            .iter()
            .enumerate()
            .map(|(i, c)| (ClockId(i as u32), c.sources.clone()))
            .collect();
        let mut network = Self::find(netlist, &clocks, constants);
        network.propagated = sdc.clocks.iter().map(|c| c.propagated).collect();
        network
    }

    fn find(
        netlist: &Netlist<'_>,
        clocks: &[(ClockId, Vec<IoPinId>)],
        constants: &Constants,
    ) -> Self {
        let db = netlist.db;
        let mut clocks_of: Vec<Vec<ClockId>> = vec![Vec::new(); db.pins.len()];
        let mut sources = Vec::new();
        let all_sources: Vec<PinId> = clocks
            .iter()
            .flat_map(|(_, ports)| ports.iter().map(|io| db.io_pins[io.index()].pin))
            .collect();
        for (clock, ports) in clocks {
            let mut queue: VecDeque<PinId> = VecDeque::new();
            for io in ports {
                let pin = db.io_pins[io.index()].pin;
                sources.push(pin);
                clocks_of[pin.index()].push(*clock);
                queue.push_back(pin);
            }
            let reach =
                |clocks_of: &mut Vec<Vec<ClockId>>, queue: &mut VecDeque<PinId>, next: PinId| {
                    if !clocks_of[next.index()].contains(clock) && !all_sources.contains(&next) {
                        clocks_of[next.index()].push(*clock);
                        queue.push_back(next);
                    }
                };
            while let Some(pin) = queue.pop_front() {
                if netlist.is_driver(pin) {
                    let net = db.pins[pin.index()].net;
                    for load in netlist.loads(net) {
                        reach(&mut clocks_of, &mut queue, load);
                    }
                }
                let PinOwner::Cell { cell, .. } = db.pins[pin.index()].owner else {
                    continue;
                };
                if !netlist.is_load(pin) {
                    continue;
                }
                let Some((lib, from_port)) = netlist.link.pin_port(db, pin) else {
                    continue;
                };
                let lib_cell = netlist.link.liberty.cell(lib);
                for set in &lib_cell.arc_sets {
                    if set.from != from_port || set.role != ArcRole::Combinational {
                        continue;
                    }
                    if !constants.arc_enabled(netlist, cell, set.from, set.to) {
                        continue;
                    }
                    let Some(out) = netlist.pin_of_port(cell, set.to) else {
                        continue;
                    };
                    reach(&mut clocks_of, &mut queue, out);
                }
            }
        }
        Self {
            clocks_of,
            sources,
            propagated: vec![false; clocks.len()],
        }
    }

    /// Whether a pin carries a clock; pins added since the search do not.
    #[must_use]
    pub fn is_clock(&self, pin: PinId) -> bool {
        !self.clocks_of(pin).is_empty()
    }

    /// The clocks reaching a pin.
    #[must_use]
    pub fn clocks_of(&self, pin: PinId) -> &[ClockId] {
        self.clocks_of.get(pin.index()).map_or(&[], Vec::as_slice)
    }

    /// Whether a clock pin takes its clock at the edge time, ideally,
    /// rather than through the network's delays: when any clock reaching it
    /// is ideal, or it is where a clock enters.
    #[must_use]
    pub fn is_ideal(&self, pin: PinId) -> bool {
        self.is_clock(pin)
            && (self.sources.contains(&pin)
                || self
                    .clocks_of(pin)
                    .iter()
                    .any(|c| !self.propagated.get(c.index()).copied().unwrap_or(false)))
    }

    /// Whether a clock pin's clocks arrive through the network's delays.
    #[must_use]
    pub fn is_propagated(&self, pin: PinId) -> bool {
        self.is_clock(pin) && !self.is_ideal(pin)
    }

    /// The pins the clocks enter at.
    #[must_use]
    pub fn sources(&self) -> &[PinId] {
        &self.sources
    }
}

/// Names for the instances and nets repair adds: a base name with a count
/// that only ever rises, as `OpenROAD` names them.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Namer {
    instance_index: u32,
    net_index: u32,
}

impl Namer {
    /// A namer whose counts start at one.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            instance_index: 1,
            net_index: 1,
        }
    }

    /// A free instance name of `base` and the next count.
    pub fn instance(&mut self, db: &NetlistDB, base: &str) -> String {
        loop {
            let name = format!("{base}{}", self.instance_index);
            self.instance_index += 1;
            if db.cell_id(&name).is_none() {
                return name;
            }
        }
    }

    /// A free net name of `base` and the next count.
    pub fn net(&mut self, db: &NetlistDB, base: &str) -> String {
        loop {
            let name = format!("{base}{}", self.net_index);
            self.net_index += 1;
            if db.net_id(&name).is_none() {
                return name;
            }
        }
    }

    /// `name` itself when no net has it, else `name` with the next count.
    pub fn net_if_needed(&mut self, db: &NetlistDB, name: &str) -> String {
        if db.net_id(name).is_none() {
            return name.to_string();
        }
        self.net(db, name)
    }
}
