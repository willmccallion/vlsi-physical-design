//! A Liberty timing library: cells with their pins, electrical limits,
//! boolean functions and delay tables.
//!
//! Every quantity is held in SI units once parsed: seconds, farads and ohms,
//! whatever units the file declared. Tables are looked up as `OpenSTA` looks
//! them up, interpolating linearly between the two nearest axis points and
//! extrapolating from the end pair outside the axis.

use std::collections::HashMap;
use std::fmt;

/// A cell in [`Liberty::cells`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct LibCellId(pub u32);

impl LibCellId {
    /// Index into [`Liberty::cells`].
    #[must_use]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A rising or falling transition.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Edge {
    /// Low to high.
    Rise,
    /// High to low.
    Fall,
}

impl Edge {
    /// Both edges, rise first.
    pub const BOTH: [Self; 2] = [Self::Rise, Self::Fall];

    /// Index into a pair of per-edge values, rise first.
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Rise => 0,
            Self::Fall => 1,
        }
    }
}

/// A value for each edge.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EdgeValues {
    /// For a rising edge.
    pub rise: f64,
    /// For a falling edge.
    pub fall: f64,
}

impl EdgeValues {
    /// The value for `edge`.
    #[must_use]
    pub const fn get(&self, edge: Edge) -> f64 {
        match edge {
            Edge::Rise => self.rise,
            Edge::Fall => self.fall,
        }
    }

    /// The larger of the two.
    #[must_use]
    pub const fn max(&self) -> f64 {
        self.rise.max(self.fall)
    }

    /// Sets the value for `edge`.
    pub const fn set(&mut self, edge: Edge, value: f64) {
        match edge {
            Edge::Rise => self.rise = value,
            Edge::Fall => self.fall = value,
        }
    }
}

/// The voltage thresholds a library measures transitions against, as
/// fractions of the supply, and the derating between its tables' slews and
/// measured ones.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Thresholds {
    /// Where a transition's slew measurement starts.
    pub slew_lower: EdgeValues,
    /// Where a transition's slew measurement ends.
    pub slew_upper: EdgeValues,
    /// Where an input is taken to switch.
    pub input: EdgeValues,
    /// Where an output is taken to switch.
    pub output: EdgeValues,
    /// Table slews over measured slews.
    pub slew_derate: f64,
}

impl Default for Thresholds {
    fn default() -> Self {
        Self {
            slew_lower: EdgeValues {
                rise: 0.2,
                fall: 0.2,
            },
            slew_upper: EdgeValues {
                rise: 0.8,
                fall: 0.8,
            },
            input: EdgeValues {
                rise: 0.5,
                fall: 0.5,
            },
            output: EdgeValues {
                rise: 0.5,
                fall: 0.5,
            },
            slew_derate: 1.0,
        }
    }
}

/// Library-wide defaults that apply where a pin gives no value.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Defaults {
    /// Largest transition time allowed at an output pin, in seconds.
    pub max_transition: Option<f64>,
    /// Largest load an output pin may drive, in farads.
    pub max_capacitance: Option<f64>,
    /// Largest fanout an output pin may drive.
    pub max_fanout: Option<f64>,
    /// Fanout load of an input pin.
    pub fanout_load: Option<f64>,
    /// Capacitance of an input pin without one, in farads.
    pub input_pin_cap: f64,
    /// Capacitance of an output pin without one, in farads.
    pub output_pin_cap: f64,
}

/// What a table axis varies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisVariable {
    /// The slew at the arc's input, in seconds.
    InputTransition,
    /// The capacitance the output drives, in farads.
    OutputCapacitance,
    /// The capacitance on a related output, in farads.
    RelatedOutputCapacitance,
    /// The slew at the clock pin of a check, in seconds.
    RelatedPinTransition,
    /// The slew at the constrained pin of a check, in seconds.
    ConstrainedPinTransition,
    /// A quantity lookups do not vary: a power axis.
    Other,
}

/// The quantities a lookup may vary along the axes of a table.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TableInputs {
    /// The slew at a delay arc's input, in seconds.
    pub in_slew: f64,
    /// The load a delay arc drives, in farads.
    pub load_cap: f64,
    /// The load on a related output, in farads.
    pub related_cap: f64,
    /// The slew at a check's clock pin, in seconds.
    pub related_slew: f64,
    /// The slew at a check's constrained pin, in seconds.
    pub constrained_slew: f64,
}

/// Which end of a range of delays an analysis looks at: the late one for
/// setup checks, the early one for hold checks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MinMax {
    /// The early side: the smallest delays, which hold checks fear.
    Min,
    /// The late side: the largest delays, which setup checks fear.
    Max,
}

impl MinMax {
    /// Both sides, min first.
    pub const BOTH: [Self; 2] = [Self::Min, Self::Max];

    /// Index into a pair of per-side values, min first.
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Min => 0,
            Self::Max => 1,
        }
    }

    /// The value nothing beats on this side, which a search starts from.
    #[must_use]
    pub const fn init_value(self) -> f64 {
        match self {
            Self::Min => f64::INFINITY,
            Self::Max => f64::NEG_INFINITY,
        }
    }

    /// Whether `a` is further toward this side than `b`.
    #[must_use]
    pub fn beats(self, a: f64, b: f64) -> bool {
        match self {
            Self::Min => a < b,
            Self::Max => a > b,
        }
    }

    /// The other side.
    #[must_use]
    pub const fn opposite(self) -> Self {
        match self {
            Self::Min => Self::Max,
            Self::Max => Self::Min,
        }
    }
}

/// One axis of a lookup table.
#[derive(Clone, Debug, PartialEq)]
pub struct Axis {
    /// What the axis varies.
    pub variable: AxisVariable,
    /// Its points, ascending, in SI units.
    pub values: Vec<f64>,
}

impl Axis {
    /// The index of the lower of the two points to interpolate between for
    /// `value`: the first pair below the axis, the last pair above it, as
    /// `OpenSTA` extrapolates.
    fn lower_index(&self, value: f64) -> usize {
        let n = self.values.len();
        if n <= 1 || value <= self.values[0] {
            return 0;
        }
        if value >= self.values[n - 1] {
            return n - 2;
        }
        self.values.partition_point(|&v| v <= value) - 1
    }

    /// The value the lookup takes along this axis.
    const fn pick(&self, inputs: &TableInputs) -> f64 {
        match self.variable {
            AxisVariable::InputTransition => inputs.in_slew,
            AxisVariable::OutputCapacitance => inputs.load_cap,
            AxisVariable::RelatedOutputCapacitance => inputs.related_cap,
            AxisVariable::RelatedPinTransition => inputs.related_slew,
            AxisVariable::ConstrainedPinTransition => inputs.constrained_slew,
            AxisVariable::Other => 0.0,
        }
    }
}

/// Axes a Liberty table may have: `variable_1` to `variable_3`.
const MAX_AXES: usize = 3;

/// A lookup table of up to three axes, its values in row-major order with
/// the first axis outermost.
#[derive(Clone, Debug, PartialEq)]
pub struct Table {
    /// The axes, outermost first; none for a scalar.
    pub axes: Vec<Axis>,
    /// The values, in SI units.
    pub values: Vec<f64>,
}

impl Table {
    /// A table of one value.
    #[must_use]
    pub fn scalar(value: f64) -> Self {
        Self {
            axes: Vec::new(),
            values: vec![value],
        }
    }

    /// Whether the table has the axes and values it claims.
    #[must_use]
    pub fn is_consistent(&self) -> bool {
        let expected: usize = self.axes.iter().map(|a| a.values.len()).product();
        !self.values.is_empty() && self.values.len() == expected && self.axes.len() <= 3
    }

    /// The largest point of the first axis varying output capacitance, if
    /// any.
    #[must_use]
    pub fn max_capacitance_point(&self) -> Option<f64> {
        self.axes
            .iter()
            .find(|a| a.variable == AxisVariable::OutputCapacitance)
            .and_then(|a| a.values.last().copied())
    }

    /// Every point of every axis varying output capacitance.
    pub fn capacitance_points(&self) -> impl Iterator<Item = f64> + '_ {
        self.axes
            .iter()
            .filter(|a| a.variable == AxisVariable::OutputCapacitance)
            .flat_map(|a| a.values.iter().copied())
    }

    /// The table's value at an input slew and load, interpolated and
    /// extrapolated linearly along each axis as `OpenSTA` does.
    #[must_use]
    pub fn lookup(&self, in_slew: f64, load_cap: f64) -> f64 {
        self.lookup_with_related(in_slew, load_cap, 0.0)
    }

    /// [`Table::lookup`] with a related output capacitance for tables that
    /// vary one.
    #[must_use]
    pub fn lookup_with_related(&self, in_slew: f64, load_cap: f64, related_cap: f64) -> f64 {
        self.lookup_inputs(&TableInputs {
            in_slew,
            load_cap,
            related_cap,
            ..TableInputs::default()
        })
    }

    /// A check table's value at the slews of its clock and constrained
    /// pins, as `OpenSTA`'s check table model finds it.
    #[must_use]
    pub fn check_value(&self, related_slew: f64, constrained_slew: f64) -> f64 {
        self.lookup_inputs(&TableInputs {
            related_slew,
            constrained_slew,
            ..TableInputs::default()
        })
    }

    /// The table's value at `inputs`, each axis taking the quantity it
    /// varies.
    #[must_use]
    pub fn lookup_inputs(&self, inputs: &TableInputs) -> f64 {
        let mut coordinates = [0.0; MAX_AXES];
        for (c, axis) in coordinates.iter_mut().zip(&self.axes) {
            *c = axis.pick(inputs);
        }
        self.interpolate(&coordinates[..self.axes.len().min(MAX_AXES)])
    }

    fn interpolate(&self, coordinates: &[f64]) -> f64 {
        let axes = &self.axes[..coordinates.len()];
        let mut lower = [0usize; MAX_AXES];
        let mut fraction = [0.0; MAX_AXES];
        for (k, (axis, &x)) in axes.iter().zip(coordinates).enumerate() {
            if axis.values.len() <= 1 {
                continue;
            }
            let i = axis.lower_index(x);
            let (x0, x1) = (axis.values[i], axis.values[i + 1]);
            lower[k] = i;
            fraction[k] = (x - x0) / (x1 - x0);
        }
        let mut sum = 0.0;
        for corner in 0..(1usize << axes.len()) {
            let mut weight = 1.0;
            let mut index = 0;
            for (k, axis) in axes.iter().enumerate() {
                let high = (corner >> k) & 1 == 1;
                let (i, t) = (lower[k], fraction[k]);
                let (point, w) = if axis.values.len() <= 1 {
                    if high { (0, 0.0) } else { (0, 1.0) }
                } else if high {
                    (i + 1, t)
                } else {
                    (i, 1.0 - t)
                };
                weight *= w;
                index = index * axis.values.len() + point;
            }
            if weight != 0.0 {
                sum = weight.mul_add(self.values[index], sum);
            }
        }
        sum
    }
}

/// What a timing arc describes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArcRole {
    /// Input to output through logic.
    Combinational,
    /// A register's clock to its outputs.
    RegClkToQ,
    /// A register's asynchronous clear or preset to its outputs.
    RegSetClr,
    /// A latch's enable to its outputs.
    LatchEnToQ,
    /// A tristate enable turning the output on.
    TristateEnable,
    /// A tristate enable turning the output off.
    TristateDisable,
    /// Data must settle this long before the clock edge.
    Setup,
    /// Data must hold this long after the clock edge.
    Hold,
    /// An asynchronous control must release this long before the clock
    /// edge.
    Recovery,
    /// An asynchronous control must stay asserted this long after the
    /// clock edge.
    Removal,
    /// Another constraint between two pins, such as a minimum pulse width.
    Check,
}

impl ArcRole {
    /// Whether the arc carries a delay rather than constraining one.
    #[must_use]
    pub const fn is_delay(self) -> bool {
        !self.is_check()
    }

    /// Whether the arc constrains a delay rather than carrying one.
    #[must_use]
    pub const fn is_check(self) -> bool {
        matches!(
            self,
            Self::Setup | Self::Hold | Self::Recovery | Self::Removal | Self::Check
        )
    }

    /// Which side of the delays a check constrains: the late side for
    /// setup and recovery, the early side for hold and removal; none for
    /// a delay or a check timing does not search.
    #[must_use]
    pub const fn check_side(self) -> Option<MinMax> {
        match self {
            Self::Setup | Self::Recovery => Some(MinMax::Max),
            Self::Hold | Self::Removal => Some(MinMax::Min),
            _ => None,
        }
    }

    /// Whether `OpenSTA` searches timing through the arc by default:
    /// delays other than tristate disables and asynchronous set or clear.
    #[must_use]
    pub const fn is_searched(self) -> bool {
        matches!(
            self,
            Self::Combinational | Self::RegClkToQ | Self::LatchEnToQ | Self::TristateEnable
        )
    }
}

/// How an output follows an input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimingSense {
    /// Same direction.
    PositiveUnate,
    /// Opposite direction.
    NegativeUnate,
    /// Either.
    NonUnate,
}

/// One edge pair of a timing arc with its delay and output slew tables,
/// or for a check its constraint.
#[derive(Clone, Debug, PartialEq)]
pub struct TimingArc {
    /// The edge at the arc's input: the clock edge of a check.
    pub from_edge: Edge,
    /// The edge at its output: the constrained pin's edge of a check.
    pub to_edge: Edge,
    /// Delay from input to output, in seconds, or the time a check
    /// demands between its pins.
    pub delay: Option<Table>,
    /// Transition time at the output, in seconds; none for a check.
    pub slew: Option<Table>,
}

impl TimingArc {
    /// The time a check demands at the slews of its clock and constrained
    /// pins; zero for a check without a table.
    #[must_use]
    pub fn check_value(&self, related_slew: f64, constrained_slew: f64) -> f64 {
        self.delay
            .as_ref()
            .map_or(0.0, |t| t.check_value(related_slew, constrained_slew))
    }

    /// Delay and output slew at an input slew and load, as `OpenSTA`'s
    /// gate table model gives them: zero for a missing table, and the slew
    /// never negative.
    #[must_use]
    pub fn gate_delay(&self, in_slew: f64, load_cap: f64) -> (f64, f64) {
        let delay = self
            .delay
            .as_ref()
            .map_or(0.0, |t| t.lookup(in_slew, load_cap));
        let slew = self
            .slew
            .as_ref()
            .map_or(0.0, |t| t.lookup(in_slew, load_cap).max(0.0));
        (delay, slew)
    }

    /// The slew at the largest load the slew table covers, with no input
    /// slew, over that load: `OpenSTA`'s approximation of the drive
    /// resistance. Zero without a load-dependent slew table.
    #[must_use]
    pub fn drive_resistance(&self) -> f64 {
        let Some(table) = &self.slew else {
            return 0.0;
        };
        let Some(cap) = table.max_capacitance_point() else {
            return 0.0;
        };
        table.lookup(0.0, cap).max(0.0) / cap
    }
}

/// A `timing` group: arcs from one pin to another.
#[derive(Clone, Debug, PartialEq)]
pub struct TimingArcSet {
    /// Index of the related pin in [`LibCell::ports`].
    pub from: usize,
    /// Index of the pin the group belongs to in [`LibCell::ports`].
    pub to: usize,
    /// What the arcs describe.
    pub role: ArcRole,
    /// How the output follows the input.
    pub sense: TimingSense,
    /// The state the arcs hold under, if conditional.
    pub when: Option<FuncExpr>,
    /// The edge pairs with tables.
    pub arcs: Vec<TimingArc>,
}

/// Which way a pin carries signals.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortDirection {
    /// Into the cell.
    Input,
    /// Out of the cell.
    Output,
    /// Both.
    Inout,
    /// Neither: a node inside the cell.
    Internal,
}

/// What part a pin plays in an integrated clock gate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ClockGatePin {
    /// None.
    #[default]
    None,
    /// The clock in.
    Clock,
    /// The enable.
    Enable,
    /// The test enable.
    Test,
    /// The gated clock out.
    Out,
}

/// A signal pin of a cell.
#[derive(Clone, Debug, PartialEq)]
pub struct LibPort {
    /// Pin name.
    pub name: String,
    /// Which way it carries signals.
    pub direction: PortDirection,
    /// Its capacitance for each edge, in farads.
    pub capacitance: EdgeValues,
    /// Largest load it may drive, in farads.
    pub max_capacitance: Option<f64>,
    /// Largest transition allowed at it, in seconds.
    pub max_transition: Option<f64>,
    /// Its fanout load.
    pub fanout_load: Option<f64>,
    /// The logic it carries, for an output.
    pub function: Option<FuncExpr>,
    /// The condition enabling a tristate output.
    pub three_state: Option<FuncExpr>,
    /// Whether it is a clock input.
    pub is_clock: bool,
    /// Its part in an integrated clock gate.
    pub clock_gate: ClockGatePin,
}

impl LibPort {
    /// The larger of the pin's edge capacitances.
    #[must_use]
    pub const fn capacitance(&self) -> f64 {
        self.capacitance.max()
    }

    /// Whether the pin takes signals in.
    #[must_use]
    pub const fn is_input(&self) -> bool {
        matches!(self.direction, PortDirection::Input | PortDirection::Inout)
    }

    /// Whether the pin drives signals out.
    #[must_use]
    pub const fn is_output(&self) -> bool {
        matches!(self.direction, PortDirection::Output | PortDirection::Inout)
    }
}

/// Which kind of storage a sequential group describes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SequentialKind {
    /// Edge triggered.
    FlipFlop,
    /// Level sensitive.
    Latch,
}

/// An `ff` or `latch` group.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sequential {
    /// Flip-flop or latch.
    pub kind: SequentialKind,
    /// The internal state node, such as `IQ`.
    pub output: String,
    /// Its inverse, such as `IQN`.
    pub output_inv: Option<String>,
    /// The clock (`clocked_on`) or enable.
    pub clock: Option<FuncExpr>,
    /// The next state or data in.
    pub data: Option<FuncExpr>,
    /// The asynchronous clear.
    pub clear: Option<FuncExpr>,
    /// The asynchronous preset.
    pub preset: Option<FuncExpr>,
}

impl Sequential {
    fn equiv(&self, other: &Self) -> bool {
        self.kind == other.kind
            && self.output == other.output
            && self.output_inv == other.output_inv
            && FuncExpr::equiv_opt(self.clock.as_ref(), other.clock.as_ref())
            && FuncExpr::equiv_opt(self.data.as_ref(), other.data.as_ref())
            && FuncExpr::equiv_opt(self.clear.as_ref(), other.clear.as_ref())
            && FuncExpr::equiv_opt(self.preset.as_ref(), other.preset.as_ref())
    }
}

/// A `statetable` group, kept as the file gives it so cells can be compared.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Statetable {
    /// The input pins, in column order.
    pub inputs: Vec<String>,
    /// The internal nodes the table sets, in column order.
    pub outputs: Vec<String>,
    /// The rows, each with its blanks collapsed to one space.
    pub rows: Vec<String>,
}

/// A cell of the library.
#[derive(Clone, Debug, PartialEq)]
pub struct LibCell {
    /// Cell name.
    pub name: String,
    /// Area in the library's units.
    pub area: f64,
    /// Whether the library asks tools not to use the cell.
    pub dont_use: bool,
    /// Whether the library asks tools not to change instances of it.
    pub dont_touch: bool,
    /// Whether it is an integrated clock gate.
    pub is_clock_gate: bool,
    /// Its footprint class, when given.
    pub footprint: Option<String>,
    /// Signal pins.
    pub ports: Vec<LibPort>,
    /// Power and ground pin names.
    pub pg_pins: Vec<String>,
    /// Timing groups.
    pub arc_sets: Vec<TimingArcSet>,
    /// Storage elements.
    pub sequentials: Vec<Sequential>,
    /// The state table of a cell described by one.
    pub statetable: Option<Statetable>,
}

impl LibCell {
    /// Index of the pin named `name`.
    #[must_use]
    pub fn port(&self, name: &str) -> Option<usize> {
        self.ports.iter().position(|p| p.name == name)
    }

    /// The single input and single output of a cell with exactly one of
    /// each and no other signal pin, as `OpenSTA`'s `bufferPorts`.
    #[must_use]
    pub fn buffer_ports(&self) -> Option<(usize, usize)> {
        let mut input = None;
        let mut output = None;
        for (i, port) in self.ports.iter().enumerate() {
            match port.direction {
                PortDirection::Input => {
                    if input.replace(i).is_some() {
                        return None;
                    }
                }
                PortDirection::Output => {
                    if output.replace(i).is_some() {
                        return None;
                    }
                }
                PortDirection::Inout | PortDirection::Internal => return None,
            }
        }
        Some((input?, output?))
    }

    /// Whether the cell passes its one input to its one output unchanged.
    #[must_use]
    pub fn is_buffer(&self) -> bool {
        self.buffer_ports().is_some_and(|(input, output)| {
            matches!(&self.ports[output].function, Some(FuncExpr::Port(p)) if *p == self.ports[input].name)
        })
    }

    /// Whether the cell passes its one input to its one output inverted.
    #[must_use]
    pub fn is_inverter(&self) -> bool {
        self.buffer_ports().is_some_and(|(input, output)| {
            matches!(&self.ports[output].function, Some(FuncExpr::Not(inner))
                if matches!(inner.as_ref(), FuncExpr::Port(p) if *p == self.ports[input].name))
        })
    }

    /// Whether the cell holds state.
    #[must_use]
    pub const fn is_sequential(&self) -> bool {
        !self.sequentials.is_empty()
    }

    /// The delay arc sets into `port`.
    pub fn delay_arc_sets_to(&self, port: usize) -> impl Iterator<Item = &TimingArcSet> + '_ {
        self.arc_sets
            .iter()
            .filter(move |s| s.to == port && s.role.is_delay())
    }

    /// The largest drive resistance over the delay arcs into `port`, as
    /// `OpenSTA`'s `driveResistance`; zero when no arc gives one.
    #[must_use]
    pub fn drive_resistance(&self, port: usize) -> f64 {
        self.delay_arc_sets_to(port)
            .flat_map(|s| s.arcs.iter())
            .map(TimingArc::drive_resistance)
            .filter(|&r| r > 0.0)
            .fold(0.0, f64::max)
    }

    /// The drive resistance of the first output pin, as `OpenSTA`'s
    /// `cellDriveResistance`; zero for a cell without an output.
    #[must_use]
    pub fn output_drive_resistance(&self) -> f64 {
        self.ports
            .iter()
            .position(LibPort::is_output)
            .map_or(0.0, |p| self.drive_resistance(p))
    }

    /// The mean capacitance of the input pins; `None` without inputs.
    #[must_use]
    pub fn mean_input_capacitance(&self) -> Option<f64> {
        let inputs: Vec<f64> = self
            .ports
            .iter()
            .filter(|p| p.direction == PortDirection::Input)
            .map(LibPort::capacitance)
            .collect();
        if inputs.is_empty() {
            return None;
        }
        Some(inputs.iter().sum::<f64>() / inputs.len() as f64)
    }

    /// Whether any pin has a function.
    #[must_use]
    pub fn has_functions(&self) -> bool {
        self.ports.iter().any(|p| p.function.is_some())
    }

    /// Whether two cells can stand in for one another, as `OpenSTA`'s
    /// `equivCells`: the same pins by name, direction and function, the
    /// same storage, and, for cells without functions, the same arcs.
    #[must_use]
    pub fn equivalent(&self, other: &Self) -> bool {
        self.equiv_ports(other)
            && self.equiv_sequentials(other)
            && self.statetable == other.statetable
            && (self.has_functions() || self.equiv_arc_sets(other))
    }

    fn equiv_ports(&self, other: &Self) -> bool {
        if self.ports.len() != other.ports.len() || self.pg_pins.len() != other.pg_pins.len() {
            return false;
        }
        self.ports.iter().all(|p| {
            other.port(&p.name).is_some_and(|i| {
                let q = &other.ports[i];
                q.direction == p.direction
                    && FuncExpr::equiv_opt(p.function.as_ref(), q.function.as_ref())
                    && FuncExpr::equiv_opt(p.three_state.as_ref(), q.three_state.as_ref())
            })
        }) && self.pg_pins.iter().all(|pg| other.pg_pins.contains(pg))
    }

    fn equiv_sequentials(&self, other: &Self) -> bool {
        self.sequentials.len() == other.sequentials.len()
            && self
                .sequentials
                .iter()
                .zip(&other.sequentials)
                .all(|(a, b)| a.equiv(b))
    }

    fn equiv_arc_sets(&self, other: &Self) -> bool {
        self.arc_sets.len() == other.arc_sets.len()
            && self.arc_sets.iter().all(|s| {
                other.arc_sets.iter().any(|t| {
                    self.ports[s.from].name == other.ports[t.from].name
                        && self.ports[s.to].name == other.ports[t.to].name
                        && s.role == t.role
                        && s.sense == t.sense
                        && FuncExpr::equiv_opt(s.when.as_ref(), t.when.as_ref())
                })
            })
    }
}

/// A timing library.
#[derive(Clone, Debug, PartialEq)]
pub struct Liberty {
    /// Library name.
    pub name: String,
    /// The units the file declared, which constraints are written in.
    pub units: LibraryUnits,
    /// Its voltage thresholds.
    pub thresholds: Thresholds,
    /// Values pins fall back on.
    pub defaults: Defaults,
    /// The cells.
    pub cells: Vec<LibCell>,
    cell_by_name: HashMap<String, LibCellId>,
}

/// The units a library declares, as multiples of SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LibraryUnits {
    /// Seconds per time unit.
    pub time: f64,
    /// Farads per capacitance unit.
    pub capacitance: f64,
}

impl Default for LibraryUnits {
    /// Nanoseconds and picofarads, which Liberty assumes when a file
    /// declares nothing.
    fn default() -> Self {
        Self {
            time: 1e-9,
            capacitance: 1e-12,
        }
    }
}

impl Liberty {
    /// An empty library.
    #[must_use]
    pub fn new(name: String) -> Self {
        Self {
            name,
            units: LibraryUnits::default(),
            thresholds: Thresholds::default(),
            defaults: Defaults::default(),
            cells: Vec::new(),
            cell_by_name: HashMap::new(),
        }
    }

    /// Adds a cell, or returns its name when one by that name exists.
    pub fn add_cell(&mut self, cell: LibCell) -> Result<LibCellId, String> {
        if self.cell_by_name.contains_key(&cell.name) {
            return Err(cell.name);
        }
        let id = LibCellId(self.cells.len() as u32);
        let _ = self.cell_by_name.insert(cell.name.clone(), id);
        self.cells.push(cell);
        Ok(id)
    }

    /// Takes the cells of another library into this one, which keeps its
    /// own thresholds and defaults; returns the name of a cell both have.
    pub fn absorb(&mut self, other: Self) -> Result<(), String> {
        for cell in other.cells {
            let _ = self.add_cell(cell)?;
        }
        Ok(())
    }

    /// The cell named `name`.
    #[must_use]
    pub fn cell_id(&self, name: &str) -> Option<LibCellId> {
        self.cell_by_name.get(name).copied()
    }

    /// The cell with this id.
    #[must_use]
    pub fn cell(&self, id: LibCellId) -> &LibCell {
        &self.cells[id.index()]
    }

    /// The transition limit `OpenSTA` applies at a pin: the pin's own, else
    /// for an output the library default. `None` where nothing limits it.
    #[must_use]
    pub const fn slew_limit(&self, port: &LibPort) -> Option<f64> {
        match port.max_transition {
            Some(limit) => Some(limit),
            None if port.is_output() => self.defaults.max_transition,
            None => None,
        }
    }

    /// The load limit `OpenSTA` applies at an output pin: the pin's own,
    /// else the library default.
    #[must_use]
    pub const fn capacitance_limit(&self, port: &LibPort) -> Option<f64> {
        match port.max_capacitance {
            Some(limit) => Some(limit),
            None if port.is_output() => self.defaults.max_capacitance,
            None => None,
        }
    }

    /// A pin's fanout load: its own, else the library default, else zero.
    #[must_use]
    pub fn fanout_load(&self, port: &LibPort) -> f64 {
        port.fanout_load
            .or(self.defaults.fanout_load)
            .unwrap_or(0.0)
    }
}

/// A three-valued logic level.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Logic {
    /// Low.
    Zero,
    /// High.
    One,
    /// Not known.
    Unknown,
}

impl Logic {
    const fn not(self) -> Self {
        match self {
            Self::Zero => Self::One,
            Self::One => Self::Zero,
            Self::Unknown => Self::Unknown,
        }
    }

    const fn and(self, other: Self) -> Self {
        match (self, other) {
            (Self::Zero, _) | (_, Self::Zero) => Self::Zero,
            (Self::One, Self::One) => Self::One,
            _ => Self::Unknown,
        }
    }

    const fn or(self, other: Self) -> Self {
        match (self, other) {
            (Self::One, _) | (_, Self::One) => Self::One,
            (Self::Zero, Self::Zero) => Self::Zero,
            _ => Self::Unknown,
        }
    }

    const fn xor(self, other: Self) -> Self {
        match (self, other) {
            (Self::Unknown, _) | (_, Self::Unknown) => Self::Unknown,
            (a, b) if a as u8 == b as u8 => Self::Zero,
            _ => Self::One,
        }
    }

    /// Whether the level is known.
    #[must_use]
    pub const fn is_constant(self) -> bool {
        !matches!(self, Self::Unknown)
    }
}

/// A boolean function of a cell's pins, as a Liberty `function` attribute
/// gives it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FuncExpr {
    /// Constant low.
    Zero,
    /// Constant high.
    One,
    /// A pin's value.
    Port(String),
    /// Negation.
    Not(Box<Self>),
    /// Conjunction.
    And(Box<Self>, Box<Self>),
    /// Disjunction.
    Or(Box<Self>, Box<Self>),
    /// Exclusive or.
    Xor(Box<Self>, Box<Self>),
}

/// Why a function expression could not be parsed.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("invalid function '{text}': {message}")]
pub struct FuncExprError {
    /// The expression.
    pub text: String,
    /// What was wrong.
    pub message: String,
}

impl FuncExpr {
    /// Parses a Liberty boolean expression: `!` or a trailing `'` negate,
    /// `^` is exclusive or, `*`, `&` or juxtaposition conjoin and `+` or `|`
    /// disjoin, binding in that order from tightest to loosest.
    pub fn parse(text: &str) -> Result<Self, FuncExprError> {
        let mut parser = FuncParser {
            text,
            chars: text.char_indices().peekable(),
        };
        let expr = parser.or()?;
        parser.skip_spaces();
        if let Some(&(at, c)) = parser.chars.peek() {
            return Err(parser.error(format!("unexpected '{c}' at {at}")));
        }
        Ok(expr)
    }

    /// Whether two expressions have the same shape, as `OpenSTA`'s
    /// `FuncExpr::equiv`.
    #[must_use]
    pub fn equiv(&self, other: &Self) -> bool {
        self == other
    }

    /// [`FuncExpr::equiv`] over optional expressions, two absences being
    /// equivalent.
    #[must_use]
    pub fn equiv_opt(a: Option<&Self>, b: Option<&Self>) -> bool {
        match (a, b) {
            (None, None) => true,
            (Some(a), Some(b)) => a.equiv(b),
            _ => false,
        }
    }

    /// The pin named by an expression that is just a pin.
    #[must_use]
    pub fn as_port(&self) -> Option<&str> {
        match self {
            Self::Port(p) => Some(p),
            _ => None,
        }
    }

    /// Whether the expression is a constant.
    #[must_use]
    pub const fn is_constant(&self) -> bool {
        matches!(self, Self::Zero | Self::One)
    }

    /// The pins the expression reads, each once, in order of appearance.
    #[must_use]
    pub fn ports(&self) -> Vec<&str> {
        let mut out = Vec::new();
        self.collect_ports(&mut out);
        out
    }

    fn collect_ports<'a>(&'a self, out: &mut Vec<&'a str>) {
        match self {
            Self::Zero | Self::One => {}
            Self::Port(p) => {
                if !out.contains(&p.as_str()) {
                    out.push(p);
                }
            }
            Self::Not(e) => e.collect_ports(out),
            Self::And(a, b) | Self::Or(a, b) | Self::Xor(a, b) => {
                a.collect_ports(out);
                b.collect_ports(out);
            }
        }
    }

    /// The expression's level given each pin's level.
    pub fn eval(&self, level: &impl Fn(&str) -> Logic) -> Logic {
        match self {
            Self::Zero => Logic::Zero,
            Self::One => Logic::One,
            Self::Port(p) => level(p),
            Self::Not(e) => e.eval(level).not(),
            Self::And(a, b) => a.eval(level).and(b.eval(level)),
            Self::Or(a, b) => a.eval(level).or(b.eval(level)),
            Self::Xor(a, b) => a.eval(level).xor(b.eval(level)),
        }
    }

    /// Whether the expression's value can change with `port` when the other
    /// pins hold the given levels, those unknown ranging over both: false
    /// where `OpenSTA` finds the arc's timing sense to be none.
    pub fn depends_on(&self, port: &str, known: &impl Fn(&str) -> Logic) -> bool {
        let free: Vec<&str> = self
            .ports()
            .into_iter()
            .filter(|p| *p != port && !known(p).is_constant())
            .collect();
        if free.len() > 16 {
            return true;
        }
        for assignment in 0..(1u32 << free.len()) {
            let level_of = |p: &str, port_level: Logic| -> Logic {
                if p == port {
                    return port_level;
                }
                let k = known(p);
                if k.is_constant() {
                    return k;
                }
                free.iter()
                    .position(|f| *f == p)
                    .map_or(Logic::Unknown, |i| {
                        if (assignment >> i) & 1 == 1 {
                            Logic::One
                        } else {
                            Logic::Zero
                        }
                    })
            };
            let low = self.eval(&|p| level_of(p, Logic::Zero));
            let high = self.eval(&|p| level_of(p, Logic::One));
            if low != high {
                return true;
            }
        }
        false
    }
}

impl fmt::Display for FuncExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Zero => write!(f, "0"),
            Self::One => write!(f, "1"),
            Self::Port(p) => write!(f, "{p}"),
            Self::Not(e) => write!(f, "!{e}"),
            Self::And(a, b) => write!(f, "({a} & {b})"),
            Self::Or(a, b) => write!(f, "({a} | {b})"),
            Self::Xor(a, b) => write!(f, "({a} ^ {b})"),
        }
    }
}

struct FuncParser<'a> {
    text: &'a str,
    chars: std::iter::Peekable<std::str::CharIndices<'a>>,
}

impl FuncParser<'_> {
    fn error(&self, message: String) -> FuncExprError {
        FuncExprError {
            text: self.text.to_string(),
            message,
        }
    }

    fn skip_spaces(&mut self) {
        while self.chars.peek().is_some_and(|(_, c)| c.is_whitespace()) {
            let _ = self.chars.next();
        }
    }

    fn peek(&mut self) -> Option<char> {
        self.skip_spaces();
        self.chars.peek().map(|&(_, c)| c)
    }

    fn or(&mut self) -> Result<FuncExpr, FuncExprError> {
        let mut left = self.and()?;
        while matches!(self.peek(), Some('+' | '|')) {
            let _ = self.chars.next();
            let right = self.and()?;
            left = FuncExpr::Or(Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn and(&mut self) -> Result<FuncExpr, FuncExprError> {
        let mut left = self.xor()?;
        loop {
            match self.peek() {
                Some('*' | '&') => {
                    let _ = self.chars.next();
                }
                Some(c) if c == '(' || c == '!' || Self::starts_name(c) => {}
                _ => return Ok(left),
            }
            let right = self.xor()?;
            left = FuncExpr::And(Box::new(left), Box::new(right));
        }
    }

    fn xor(&mut self) -> Result<FuncExpr, FuncExprError> {
        let mut left = self.unary()?;
        while self.peek() == Some('^') {
            let _ = self.chars.next();
            let right = self.unary()?;
            left = FuncExpr::Xor(Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn unary(&mut self) -> Result<FuncExpr, FuncExprError> {
        let mut expr = match self.peek() {
            Some('!') => {
                let _ = self.chars.next();
                let inner = self.unary()?;
                FuncExpr::Not(Box::new(inner))
            }
            Some('(') => {
                let _ = self.chars.next();
                let inner = self.or()?;
                if self.peek() != Some(')') {
                    return Err(self.error("missing ')'".into()));
                }
                let _ = self.chars.next();
                inner
            }
            Some(c) if Self::starts_name(c) => {
                let name = self.name();
                match name.as_str() {
                    "0" => FuncExpr::Zero,
                    "1" => FuncExpr::One,
                    _ => FuncExpr::Port(name),
                }
            }
            Some(c) => return Err(self.error(format!("unexpected '{c}'"))),
            None => return Err(self.error("unexpected end".into())),
        };
        while self.peek() == Some('\'') {
            let _ = self.chars.next();
            expr = FuncExpr::Not(Box::new(expr));
        }
        Ok(expr)
    }

    const fn starts_name(c: char) -> bool {
        c.is_ascii_alphanumeric() || c == '_' || c == '[' || c == ']' || c == '.' || c == '\\'
    }

    fn name(&mut self) -> String {
        let mut out = String::new();
        while let Some(&(_, c)) = self.chars.peek() {
            if Self::starts_name(c) {
                out.push(c);
                let _ = self.chars.next();
            } else {
                break;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn table_7x7_like() -> Table {
        Table {
            axes: vec![
                Axis {
                    variable: AxisVariable::InputTransition,
                    values: vec![1.0, 2.0, 4.0],
                },
                Axis {
                    variable: AxisVariable::OutputCapacitance,
                    values: vec![10.0, 20.0],
                },
            ],
            values: vec![0.0, 10.0, 1.0, 11.0, 3.0, 13.0],
        }
    }

    #[test]
    fn lookup_interpolates_bilinearly_inside_the_axes() {
        let t = table_7x7_like();
        assert!((t.lookup(1.5, 15.0) - 5.5).abs() < 1e-12);
        assert!((t.lookup(3.0, 10.0) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn lookup_extrapolates_from_the_end_pairs_outside_the_axes() {
        let t = table_7x7_like();
        assert!((t.lookup(0.0, 10.0) - (-1.0)).abs() < 1e-12);
        assert!((t.lookup(6.0, 10.0) - 5.0).abs() < 1e-12);
        assert!((t.lookup(1.0, 30.0) - 20.0).abs() < 1e-12);
    }

    #[test]
    fn lookup_returns_the_point_of_a_single_point_axis() {
        let t = Table {
            axes: vec![Axis {
                variable: AxisVariable::OutputCapacitance,
                values: vec![5.0],
            }],
            values: vec![7.0],
        };
        assert!((t.lookup(0.0, 99.0) - 7.0).abs() < 1e-12);
        assert!((Table::scalar(2.5).lookup(1.0, 1.0) - 2.5).abs() < 1e-12);
    }

    #[test]
    fn functions_parse_with_liberty_precedence() {
        let f = FuncExpr::parse("!(A1 & A2) | B ^ C").unwrap();
        assert_eq!(
            f,
            FuncExpr::Or(
                Box::new(FuncExpr::Not(Box::new(FuncExpr::And(
                    Box::new(FuncExpr::Port("A1".into())),
                    Box::new(FuncExpr::Port("A2".into()))
                )))),
                Box::new(FuncExpr::Xor(
                    Box::new(FuncExpr::Port("B".into())),
                    Box::new(FuncExpr::Port("C".into()))
                ))
            )
        );
        assert_eq!(
            FuncExpr::parse("A B'").unwrap(),
            FuncExpr::And(
                Box::new(FuncExpr::Port("A".into())),
                Box::new(FuncExpr::Not(Box::new(FuncExpr::Port("B".into()))))
            )
        );
        assert_eq!(FuncExpr::parse("0").unwrap(), FuncExpr::Zero);
        assert!(FuncExpr::parse("(A").is_err());
    }

    #[test]
    fn evaluation_follows_three_valued_logic() {
        let f = FuncExpr::parse("(S & B) | (A & !S)").unwrap();
        let s_one = |p: &str| match p {
            "S" => Logic::One,
            _ => Logic::Unknown,
        };
        assert_eq!(f.eval(&s_one), Logic::Unknown);
        assert!(f.depends_on("B", &s_one));
        assert!(!f.depends_on("A", &s_one));
        let and = FuncExpr::parse("A & B").unwrap();
        let a_zero = |p: &str| {
            if p == "A" {
                Logic::Zero
            } else {
                Logic::Unknown
            }
        };
        assert_eq!(and.eval(&a_zero), Logic::Zero);
        assert!(!and.depends_on("B", &a_zero));
        assert!(and.depends_on("B", &|_| Logic::Unknown));
    }

    fn port(name: &str, direction: PortDirection, function: Option<&str>) -> LibPort {
        LibPort {
            name: name.into(),
            direction,
            capacitance: EdgeValues {
                rise: 1e-15,
                fall: 1e-15,
            },
            max_capacitance: None,
            max_transition: None,
            fanout_load: None,
            function: function.map(|f| FuncExpr::parse(f).unwrap()),
            three_state: None,
            is_clock: false,
            clock_gate: ClockGatePin::None,
        }
    }

    fn cell(name: &str, ports: Vec<LibPort>) -> LibCell {
        LibCell {
            name: name.into(),
            area: 1.0,
            dont_use: false,
            dont_touch: false,
            is_clock_gate: false,
            footprint: None,
            ports,
            pg_pins: vec!["VDD".into(), "VSS".into()],
            arc_sets: Vec::new(),
            sequentials: Vec::new(),
            statetable: None,
        }
    }

    #[test]
    fn a_buffer_has_one_input_passed_to_one_output() {
        let buf = cell(
            "BUF",
            vec![
                port("A", PortDirection::Input, None),
                port("Z", PortDirection::Output, Some("A")),
            ],
        );
        let inv = cell(
            "INV",
            vec![
                port("A", PortDirection::Input, None),
                port("ZN", PortDirection::Output, Some("!A")),
            ],
        );
        let and = cell(
            "AND",
            vec![
                port("A1", PortDirection::Input, None),
                port("A2", PortDirection::Input, None),
                port("ZN", PortDirection::Output, Some("A1 & A2")),
            ],
        );
        assert!(buf.is_buffer() && !buf.is_inverter());
        assert!(inv.is_inverter() && !inv.is_buffer());
        assert!(!and.is_buffer() && and.buffer_ports().is_none());
    }

    #[test]
    fn cells_with_the_same_pins_and_functions_are_equivalent() {
        let x1 = cell(
            "BUF_X1",
            vec![
                port("A", PortDirection::Input, None),
                port("Z", PortDirection::Output, Some("A")),
            ],
        );
        let mut x2 = x1.clone();
        x2.name = "BUF_X2".into();
        x2.area = 2.0;
        let mut clkbuf = x1.clone();
        clkbuf.name = "CLKBUF_X1".into();
        let inv = cell(
            "INV_X1",
            vec![
                port("A", PortDirection::Input, None),
                port("ZN", PortDirection::Output, Some("!A")),
            ],
        );
        assert!(x1.equivalent(&x2));
        assert!(x1.equivalent(&clkbuf));
        assert!(!x1.equivalent(&inv));
    }

    #[test]
    fn drive_resistance_is_the_slew_over_the_largest_table_load() {
        let arc = TimingArc {
            from_edge: Edge::Rise,
            to_edge: Edge::Rise,
            delay: None,
            slew: Some(Table {
                axes: vec![
                    Axis {
                        variable: AxisVariable::InputTransition,
                        values: vec![0.0, 1.0],
                    },
                    Axis {
                        variable: AxisVariable::OutputCapacitance,
                        values: vec![1.0, 4.0],
                    },
                ],
                values: vec![1.0, 8.0, 2.0, 9.0],
            }),
        };
        assert!((arc.drive_resistance() - 2.0).abs() < 1e-12);
    }
}
