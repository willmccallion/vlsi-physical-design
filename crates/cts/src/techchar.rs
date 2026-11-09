//! The characterization of buffered clock wires, as `TritonCTS`'s
//! `TechChar`.
//!
//! Every pattern of buffers along a few lengths of clock wire is timed for
//! a range of loads and input slews, and the results, normalized to small
//! integers, form the table the tree builder picks wire segments from.

// The arithmetic keeps TritonCTS's operations, whose rounding fusing would change.
#![allow(clippy::suboptimal_flops)]

use crate::CtsError;
use crate::capacitance::Farads32;
use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::{
    AxisVariable, BufferCell, Edge, LibCell, LibPortId, Liberty, Thresholds,
};
use pare_common::units::{Farads, FaradsPerDbu, Ohms, OhmsPerDbu, Seconds};
use pare_resizer::Link;
use pare_resizer::WireRc;
use pare_resizer::dmp::{self, PiModel};
use rustc_hash::FxHashMap;
use std::cmp::Ordering;
use std::collections::BTreeMap;

const SLEW_STEPS: usize = 7;
const CAP_STEPS: usize = 20;
const WIRELENGTH_ITERATIONS: u32 = 4;
const NUM_BITS_PER_FIELD: u32 = 10;
const MAX_NORMALIZED: u32 = (1 << NUM_BITS_PER_FIELD) - 1;
/// Steps of a segment's delay per slew step, as
/// `TechChar::characterizationPostProcess` normalizes delays.
const DELAY_STEPS_PER_SLEW_STEP: f32 = 5.0;

/// One characterized stretch of clock wire.
///
/// It records its length, the load it drives and the slew it leaves there,
/// its delay, and where buffers sit along it. The lengths are in the
/// table's length unit and the other quantities in steps of the
/// characterization; the narrow fields keep `TritonCTS`'s.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct WireSegment {
    /// Length, in length units.
    pub length: u8,
    /// Load at the far end, in capacitance steps.
    pub load: u8,
    /// Slew at the far end, in slew steps.
    pub output_slew: u8,
    /// Delay, in fifths of a slew step.
    pub delay: u32,
    /// Capacitance the segment presents at its start, in capacitance steps.
    pub input_cap: u8,
    /// Slew at its start, in slew steps.
    pub input_slew: u8,
    /// Where each buffer sits, as the share of the segment's length from
    /// the previous buffer or the start.
    pub buffer_locations: Vec<f64>,
    /// The buffer at each location.
    pub buffer_masters: Vec<BufferCell>,
    /// Wire from the start to the first buffer, in length units; the whole
    /// length for a bare wire.
    pub wl_to_first_buffer: i32,
    /// Wire after the last buffer, in length units.
    pub last_wl: i32,
}

impl WireSegment {
    /// Whether any buffer sits along the segment.
    #[must_use]
    pub(crate) const fn is_buffered(&self) -> bool {
        !self.buffer_locations.is_empty()
    }
}

/// A length along clock wire in the characterization's length unit, the
/// unit the H-tree measures its distances in.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub(crate) struct SegmentLength(f64);

impl SegmentLength {
    /// The length in length units.
    #[must_use]
    pub(crate) const fn get(self) -> f64 {
        self.0
    }
}

/// The delay one length unit of the tree's own buffered clock wire adds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct WireDelayRate(Seconds);

impl WireDelayRate {
    /// The length of clock wire that delays the clock by `delay`.
    #[must_use]
    pub(crate) fn length_of(self, delay: Seconds) -> SegmentLength {
        SegmentLength(delay / self.0)
    }
}

/// What the characterization is built from.
#[derive(Clone, Copy, Debug)]
pub(crate) struct CharSetup<'a> {
    /// The Liberty cells matched to the design's masters.
    pub link: &'a Link<'a>,
    /// The design, for the buffers' masters.
    pub db: &'a NetlistDB,
    /// The buffers to characterize, by increasing load limit.
    pub masters: &'a [BufferCell],
    /// The buffer leaf clusters are driven by.
    pub sink_buffer: BufferCell,
    /// The clock wire's resistance and capacitance.
    pub clock_rc: WireRc,
}

/// The characterization table.
#[derive(Clone, Debug)]
pub(crate) struct TechChar {
    segments: Vec<WireSegment>,
    by_key: FxHashMap<u32, Vec<usize>>,
    min_segment_length: u32,
    max_segment_length: u32,
    max_capacitance: u32,
    max_slew: u32,
    actual_min_input_cap: u32,
    length_unit: u32,
    cap_per_dbu: FaradsPerDbu,
    sink_buffer_input_cap: Farads32,
    wire_delay_rate: Option<WireDelayRate>,
}

/// One buffered-wire pattern: the wire before each buffer and after the
/// last, in characterization units, and its description.
#[derive(Clone, Debug)]
struct Pattern {
    nets: Vec<u32>,
    descriptor: Vec<Piece>,
    buffers: Vec<usize>,
}

impl Pattern {
    const fn is_pure_wire(&self) -> bool {
        self.buffers.is_empty()
    }
}

/// A piece of a pattern's description: a wire's length in database units,
/// or a buffer by its index in the masters.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Piece {
    Wire(f64),
    Buffer(usize),
}

/// The timing of one pattern at one load and input slew, before
/// normalization.
#[derive(Clone, Debug)]
struct Measured {
    load: f32,
    in_slew: f32,
    wirelength: f32,
    pin_slew: f32,
    pin_arrival: f32,
    total_cap: f32,
    pure_wire: bool,
    topology: Vec<Piece>,
}

/// The key `TritonCTS` groups measurements under, ordered as its map.
#[derive(Clone, Copy, Debug)]
struct CharKey {
    load: f32,
    wirelength: f32,
    pin_slew: f32,
    total_cap: f32,
}

impl CharKey {
    const fn fields(&self) -> [f32; 4] {
        [self.load, self.wirelength, self.pin_slew, self.total_cap]
    }
}

impl PartialEq for CharKey {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for CharKey {}

impl PartialOrd for CharKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for CharKey {
    fn cmp(&self, other: &Self) -> Ordering {
        for (a, b) in self.fields().iter().zip(other.fields()) {
            match a.partial_cmp(&b).unwrap_or(Ordering::Equal) {
                Ordering::Equal => {}
                unequal => return unequal,
            }
        }
        Ordering::Equal
    }
}

/// A measurement normalized to the characterization's steps.
#[derive(Clone, Debug)]
struct Normalized {
    load: f32,
    in_slew: f32,
    wirelength: f32,
    pin_slew: f32,
    pin_arrival: f32,
    total_cap: f32,
    pure_wire: bool,
    topology: Vec<NormalizedPiece>,
}

/// A piece of a normalized description: a wire as its share of the
/// pattern's length, written and read back with six decimals as
/// `TritonCTS` does, or a buffer.
#[derive(Clone, Copy, Debug)]
enum NormalizedPiece {
    Share(f64),
    Buffer(usize),
}

/// The ranges the normalized values span.
#[derive(Clone, Copy, Debug)]
struct Bounds {
    min: u32,
    max: u32,
}

impl Bounds {
    const fn new() -> Self {
        Self {
            min: u32::MAX,
            max: 0,
        }
    }

    fn normalize(&mut self, value: f32, step: f32) -> u32 {
        let normalized = ((value / step).ceil() as u32).max(1);
        self.min = self.min.min(normalized);
        self.max = self.max.max(normalized);
        normalized
    }
}

/// The steps, axes and lengths the characterization runs over.
#[derive(Clone, Debug)]
struct Plan {
    char_buffer: usize,
    char_unit: u32,
    length_unit: u32,
    wirelengths: Vec<u32>,
    max_slew: f32,
    slew_step: f32,
    cap_step: f32,
    slews: Vec<f32>,
    loads: Vec<f32>,
    sink_buffer_input_cap: Farads32,
}

impl TechChar {
    /// Characterizes the buffers of `setup`, as `TechChar::create`.
    pub fn new(setup: &CharSetup<'_>) -> Result<Self, CtsError> {
        let liberty = setup.link.liberty;
        let plan = plan(setup)?;
        let wire = WirePerDbu {
            res: setup.clock_rc.resistance.per_dbu(setup.db.dbu()),
            cap: setup.clock_rc.capacitance.per_dbu(setup.db.dbu()),
        };
        let mut groups: BTreeMap<CharKey, Vec<Measured>> = BTreeMap::new();
        for &wirelength in &plan.wirelengths {
            for pattern in patterns(wirelength, plan.char_unit) {
                let configs =
                    buffer_configs(setup.masters.len(), pattern.buffers.len(), plan.char_buffer);
                for config in configs {
                    let chain = Chain {
                        liberty,
                        thresholds: &liberty.thresholds,
                        masters: setup.masters,
                        config: &config,
                        pattern: &pattern,
                        unit: plan.char_unit,
                        wire,
                    };
                    let topology = chain.topology();
                    for &load in &plan.loads {
                        for &in_slew in &plan.slews {
                            let measured =
                                chain.measure(&plan, load, in_slew, wirelength, &topology)?;
                            let key = CharKey {
                                load: measured.load,
                                wirelength: measured.wirelength,
                                pin_slew: measured.pin_slew,
                                total_cap: measured.total_cap,
                            };
                            groups.entry(key).or_default().push(measured);
                        }
                    }
                }
            }
        }
        let measurements: Vec<Measured> = groups.into_values().flatten().collect();
        let (lines, bounds) = post_process(&measurements, &plan);
        compile(&lines, &bounds, &plan, setup.masters, wire.cap)
    }

    /// The segment at `index`.
    #[must_use]
    pub fn segment(&self, index: usize) -> &WireSegment {
        &self.segments[index]
    }

    /// The segments of a length, load and output slew, in table order.
    #[must_use]
    pub(crate) fn segments_with(&self, length: u32, load: u32, output_slew: u32) -> &[usize] {
        self.by_key
            .get(&key(length, load, output_slew))
            .map_or(&[], Vec::as_slice)
    }

    /// The shortest characterized length, in length units.
    #[must_use]
    pub(crate) const fn min_segment_length(&self) -> u32 {
        self.min_segment_length
    }

    /// The longest characterized length, in length units.
    #[must_use]
    pub(crate) const fn max_segment_length(&self) -> u32 {
        self.max_segment_length
    }

    /// The largest normalized load.
    #[must_use]
    pub const fn max_capacitance(&self) -> u32 {
        self.max_capacitance
    }

    /// The largest normalized slew.
    #[must_use]
    pub const fn max_slew(&self) -> u32 {
        self.max_slew
    }

    /// The smallest normalized input capacitance of any segment.
    #[must_use]
    pub(crate) const fn actual_min_input_cap(&self) -> u32 {
        self.actual_min_input_cap
    }

    /// The least delay per length unit among the buffered segments that
    /// keep their slew, the rate at which a long run of the tree's own wire
    /// delays the clock; none when no buffered segment keeps its slew.
    #[must_use]
    pub(crate) const fn wire_delay_rate(&self) -> Option<WireDelayRate> {
        self.wire_delay_rate
    }

    /// The length unit, in database units.
    #[must_use]
    pub(crate) const fn length_unit(&self) -> u32 {
        self.length_unit
    }

    /// Clock wire capacitance per database unit, in farads.
    #[must_use]
    pub(crate) const fn cap_per_dbu(&self) -> FaradsPerDbu {
        self.cap_per_dbu
    }

    /// Input capacitance of the sink buffer, in farads.
    #[must_use]
    pub(crate) const fn sink_buffer_input_cap(&self) -> Farads32 {
        self.sink_buffer_input_cap
    }

    /// Copies every segment of `length` as a segment of `fake_length`, for
    /// sink regions shorter than the shortest characterized wire, as
    /// `TechChar::createFakeEntries`.
    pub(crate) fn create_fake_entries(&mut self, length: u32, fake_length: u32) {
        if length == fake_length {
            return;
        }
        for load in 1..=self.max_capacitance {
            for output_slew in 1..=self.max_slew {
                let found: Vec<usize> = self.segments_with(length, load, output_slew).to_vec();
                for index in found {
                    let mut fake = self.segments[index].clone();
                    fake.length = fake_length as u8;
                    self.push(fake);
                }
            }
        }
    }

    fn push(&mut self, segment: WireSegment) {
        let k = key(
            u32::from(segment.length),
            u32::from(segment.load),
            u32::from(segment.output_slew),
        );
        self.segments.push(segment);
        self.by_key
            .entry(k)
            .or_default()
            .push(self.segments.len() - 1);
    }
}

#[cfg(test)]
impl TechChar {
    /// A table of `segments` alone, `length_unit` database units to the
    /// tree's length unit, its ranges those of the segments.
    pub(crate) fn from_segments(segments: Vec<WireSegment>, length_unit: u32) -> Self {
        let range = |field: fn(&WireSegment) -> u8| {
            let values = segments.iter().map(|s| u32::from(field(s)));
            (values.clone().min().unwrap(), values.max().unwrap())
        };
        let (min_segment_length, max_segment_length) = range(|s| s.length);
        let mut table = Self {
            segments: Vec::new(),
            by_key: FxHashMap::default(),
            min_segment_length,
            max_segment_length,
            max_capacitance: range(|s| s.load).1,
            max_slew: range(|s| s.output_slew).1,
            actual_min_input_cap: range(|s| s.input_cap).0,
            length_unit,
            cap_per_dbu: FaradsPerDbu::new(0.0),
            sink_buffer_input_cap: Farads32::ZERO,
            wire_delay_rate: None,
        };
        for segment in segments {
            table.push(segment);
        }
        table
    }
}

const fn key(length: u32, load: u32, output_slew: u32) -> u32 {
    length | (load << NUM_BITS_PER_FIELD) | (output_slew << (2 * NUM_BITS_PER_FIELD))
}

/// Picks the characterization buffer, the lengths, loads and slews, as
/// `TechChar::initCharacterization`.
fn plan(setup: &CharSetup<'_>) -> Result<Plan, CtsError> {
    let liberty = setup.link.liberty;
    let mut char_buffer: Option<(usize, BufferCell)> = None;
    let mut max_buffer_cap = 0.0_f32;
    let mut axis_slews: Vec<f32> = Vec::new();
    let mut axis_loads: Vec<f32> = Vec::new();
    for (index, &master) in setup.masters.iter().enumerate() {
        let cell = liberty.cell(master.id);
        let (input, output) = (master.input, master.output);
        let max_cap = cell.ports[output]
            .max_capacitance
            .or_else(|| liberty.cell_defaults(cell).max_capacitance)
            .ok_or_else(|| CtsError::NoMaxCapacitance(cell.name.clone()))?
            .get() as f32;
        if max_cap >= max_buffer_cap {
            max_buffer_cap = max_cap;
            char_buffer = Some((index, master));
        }
        collect_table_axes(cell, input, output, &mut axis_slews, &mut axis_loads);
    }
    let (char_index, char_buffer) = char_buffer.ok_or(CtsError::NoBuffers)?;
    let height = setup
        .link
        .macro_of(char_buffer.id)
        .map(|m| setup.db.library.cell_macro(m).size.y.get())
        .ok_or_else(|| CtsError::NoMaster(liberty.cell(char_buffer.id).name.clone()))?;
    let char_unit = (height * 10) as u32;
    let max_wirelength = (char_unit * WIRELENGTH_ITERATIONS).max(2 * char_unit);
    let mut wirelengths = Vec::new();
    let mut wirelength = char_unit;
    while wirelength <= max_wirelength && wirelength <= WIRELENGTH_ITERATIONS * char_unit {
        wirelengths.push(wirelength);
        wirelength += char_unit;
    }
    let cell = liberty.cell(char_buffer.id);
    let (input, output) = (char_buffer.input, char_buffer.output);
    let max_slew = cell.ports[output]
        .max_transition
        .or(cell.ports[input].max_transition)
        .or_else(|| liberty.cell_defaults(cell).max_transition)
        .ok_or_else(|| CtsError::NoMaxSlew(cell.name.clone()))?
        .get() as f32;
    let max_cap = cell.ports[output]
        .max_capacitance
        .or_else(|| liberty.cell_defaults(cell).max_capacitance)
        .ok_or_else(|| CtsError::NoMaxCapacitance(cell.name.clone()))?
        .get() as f32;
    let sink = setup.sink_buffer;
    let sink_buffer_input_cap = Farads32::of(liberty.cell(sink.id).ports[sink.input].capacitance());
    let slew_step = max_slew / SLEW_STEPS as f32;
    let cap_step = max_cap / CAP_STEPS as f32;
    let (slews, loads) = if axis_slews.is_empty() && axis_loads.is_empty() {
        (
            (1..=SLEW_STEPS).map(|s| s as f32 * slew_step).collect(),
            (1..=CAP_STEPS).map(|s| s as f32 * cap_step).collect(),
        )
    } else {
        if axis_slews.is_empty() || axis_loads.is_empty() {
            return Err(CtsError::NoTableAxes(cell.name.clone()));
        }
        axis_slews.push(max_slew);
        sort_and_uniquify(&mut axis_slews);
        axis_loads.push(max_cap);
        sort_and_uniquify(&mut axis_loads);
        reduce_or_expand(&mut axis_slews, SLEW_STEPS);
        reduce_or_expand(&mut axis_loads, CAP_STEPS);
        (axis_slews, axis_loads)
    };
    Ok(Plan {
        char_buffer: char_index,
        char_unit,
        length_unit: (height * 10 / 2) as u32,
        wirelengths,
        max_slew,
        slew_step,
        cap_step,
        slews,
        loads,
        sink_buffer_input_cap,
    })
}

/// The slews and loads of the delay tables of the first timing group from
/// a buffer's input to its output.
fn collect_table_axes(
    cell: &LibCell,
    input: LibPortId,
    output: LibPortId,
    slews: &mut Vec<f32>,
    loads: &mut Vec<f32>,
) {
    let Some(set) = cell
        .arc_sets
        .iter()
        .find(|s| s.from == input && s.to == output)
    else {
        return;
    };
    for arc in &set.arcs {
        let Some(table) = &arc.delay else {
            continue;
        };
        for axis in table.axes.iter().take(2) {
            match axis.variable {
                AxisVariable::InputTransition => {
                    slews.extend(axis.values.iter().map(|&v| v as f32));
                }
                AxisVariable::OutputCapacitance => {
                    loads.extend(axis.values.iter().map(|&v| v as f32));
                }
                _ => {}
            }
        }
    }
}

/// Sorts the values and drops each within one percent of the one kept
/// before it, as `TechChar::sortAndUniquify`.
fn sort_and_uniquify(values: &mut Vec<f32>) {
    values.sort_by(f32::total_cmp);
    values.dedup();
    let mut kept: Vec<f32> = Vec::with_capacity(values.len());
    for &value in values.iter() {
        match kept.last() {
            Some(&prev) if (prev - value).abs() / value < 0.01 => {}
            _ => kept.push(value),
        }
    }
    *values = kept;
}

/// Drops the value closest to its predecessor, or splits the widest gap,
/// until `limit` values remain, as `TechChar::reduceOrExpand`.
fn reduce_or_expand(values: &mut Vec<f32>, limit: usize) {
    while values.len() > limit {
        let mut min_diff = f32::MAX;
        let mut smallest = 0;
        for i in 1..values.len() {
            let diff = (values[i] - values[i - 1]).abs();
            if diff < min_diff {
                min_diff = diff;
                smallest = i;
            }
        }
        let _ = values.remove(smallest);
    }
    while values.len() < limit && values.len() >= 2 {
        let mut max_diff = f32::MIN_POSITIVE;
        let mut largest = 0;
        for i in 1..values.len() {
            let diff = (values[i] - values[i - 1]).abs();
            if diff > max_diff {
                max_diff = diff;
                largest = i - 1;
            }
        }
        let split = f32::midpoint(values[largest], values[largest + 1]);
        values.insert(largest + 1, split);
    }
}

/// Every placement of buffers at the nodes of a wire `wirelength` long, a
/// node per characterization unit, as `TechChar::createPatterns`.
fn patterns(wirelength: u32, unit: u32) -> Vec<Pattern> {
    let nodes = wirelength / unit;
    (0..1_u32 << nodes)
        .map(|counter| {
            let mut nets = Vec::new();
            let mut descriptor = Vec::new();
            let mut buffers = Vec::new();
            let mut without_buffer = 0;
            for node in 0..nodes {
                without_buffer += 1;
                if counter & (1 << node) != 0 {
                    nets.push(without_buffer);
                    descriptor.push(Piece::Wire(f64::from(without_buffer * unit)));
                    descriptor.push(Piece::Buffer(0));
                    buffers.push(0);
                    without_buffer = 0;
                }
            }
            nets.push(without_buffer);
            if without_buffer != 0 {
                descriptor.push(Piece::Wire(f64::from(without_buffer * unit)));
            }
            Pattern {
                nets,
                descriptor,
                buffers,
            }
        })
        .collect()
}

/// The assignments of buffer types to a pattern's `nodes` buffers, in the
/// order `TechChar` steps through them: every buffer first of the type
/// `start`, the characterization buffer, then each next number in base
/// `masters`, the first buffer the lowest digit, that never decreases
/// along the chain, until there have been as many as there are such
/// assignments.
fn buffer_configs(masters: usize, nodes: usize, start: usize) -> Vec<Vec<usize>> {
    let total = masters.pow(nodes as u32);
    let digits = |mut number: usize| -> Vec<usize> {
        (0..nodes)
            .map(|_| {
                let digit = number % masters;
                number /= masters;
                digit
            })
            .collect()
    };
    let monotonic = |config: &[usize]| config.windows(2).all(|w| w[1] >= w[0]);
    let count = (0..total).filter(|&n| monotonic(&digits(n))).count();
    let mut number: usize = (0..nodes).map(|i| start * masters.pow(i as u32)).sum();
    let mut configs = vec![digits(number)];
    while configs.len() < count {
        loop {
            number = (number + 1) % total;
            let config = digits(number);
            if monotonic(&config) {
                configs.push(config);
                break;
            }
        }
    }
    configs
}

/// Clock wire resistance and capacitance per database unit.
#[derive(Clone, Copy, Debug)]
struct WirePerDbu {
    res: OhmsPerDbu,
    cap: FaradsPerDbu,
}

/// A pattern with its buffers' types, timed as `TechChar`'s characterization
/// block is: an ideal input port, then each wire's pi model and Elmore
/// delay as set by hand, the last wire carrying the load.
struct Chain<'a> {
    liberty: &'a Liberty,
    thresholds: &'a Thresholds,
    masters: &'a [BufferCell],
    config: &'a [usize],
    pattern: &'a Pattern,
    unit: u32,
    wire: WirePerDbu,
}

/// A wire's pi model and Elmore delay as `OpenSTA` stores them.
#[derive(Clone, Copy, Debug)]
struct WirePi {
    c2: f32,
    rpi: f32,
    c1: f32,
    elmore: f32,
}

impl Chain<'_> {
    /// The description with each buffer's type.
    fn topology(&self) -> Vec<Piece> {
        let mut next = 0;
        self.pattern
            .descriptor
            .iter()
            .map(|piece| match piece {
                Piece::Wire(_) => *piece,
                Piece::Buffer(_) => {
                    let buffer = Piece::Buffer(self.config[next]);
                    next += 1;
                    buffer
                }
            })
            .collect()
    }

    fn wire_pi(&self, net: usize, load: f32) -> WirePi {
        let length = f64::from(self.pattern.nets[net] * self.unit);
        let cap = length * self.wire.cap.get();
        let res = length * self.wire.res.get();
        let c2 = (cap / 2.0) as f32;
        let rpi = res as f32;
        let c1 = (cap / 2.0) as f32;
        if net + 1 == self.pattern.nets.len() {
            WirePi {
                c2,
                rpi,
                c1: c1 + load,
                elmore: rpi * (c1 + c2 + load),
            }
        } else {
            WirePi {
                c2,
                rpi,
                c1,
                elmore: (res * cap) as f32,
            }
        }
    }

    /// Arrival and slew at the output port for a transition of `edge`
    /// entering at slew `in_slew`. `TechChar` gives the wires their
    /// parasitics for rising transitions alone, so a falling one sees no
    /// wire: each buffer drives the next one's pin, the last nothing. The
    /// input port's annotated slew reaches the first buffer unchanged.
    fn propagate(&self, edge: Edge, load: f32, in_slew: f32) -> Result<(f32, f32), CtsError> {
        let t = self.thresholds;
        let with_wires = edge == Edge::Rise;
        let mut arrival = if with_wires {
            let vth = f64::from(t.input.get(edge) as f32);
            (-f64::from(self.wire_pi(0, load).elmore) * (1.0 - vth).ln()) as f32
        } else {
            0.0
        };
        let mut slew = in_slew;
        for (i, &kind) in self.config.iter().enumerate() {
            let master = self.masters[kind];
            let cell = self.liberty.cell(master.id);
            let (input, output) = (master.input, master.output);
            let (model, elmore) = if with_wires {
                let pi = self.wire_pi(i + 1, load);
                let model = PiModel {
                    c2: Farads::new(f64::from(pi.c2)),
                    rpi: Ohms::new(f64::from(pi.rpi)),
                    c1: Farads::new(f64::from(pi.c1)),
                };
                (model, Some(Seconds::new(f64::from(pi.elmore))))
            } else {
                let pin_cap = self.config.get(i + 1).map_or(0.0, |&next| {
                    let next = self.masters[next];
                    self.liberty.cell(next.id).ports[next.input]
                        .capacitance
                        .get(edge)
                        .get() as f32
                });
                let model = PiModel {
                    c2: Farads::ZERO,
                    rpi: Ohms::ZERO,
                    c1: Farads::new(f64::from(pin_cap)),
                };
                (model, None)
            };
            let mut worst: Option<(f32, f32)> = None;
            for set in cell.delay_arc_sets_to(output).filter(|s| s.from == input) {
                for arc in set.arcs.iter().filter(|a| a.to_edge == edge) {
                    let gate = dmp::gate_delay(arc, t, Seconds::new(f64::from(slew)), model);
                    let (wire_delay, load_slew) =
                        elmore.map_or((Seconds::ZERO, gate.slew), |e| gate.load_delay_slew(e));
                    let at = arrival + gate.delay.get() as f32 + wire_delay.get() as f32;
                    let s = load_slew.get() as f32;
                    worst = Some(worst.map_or((at, s), |(a, w)| (a.max(at), w.max(s))));
                }
            }
            let (at, s) = worst.ok_or_else(|| CtsError::NoDelayArc(cell.name.clone()))?;
            arrival = at;
            slew = s;
        }
        Ok((arrival, slew))
    }

    /// Times the chain at `load` and `in_slew`, as
    /// `TechChar::computeTopologyResults`.
    fn measure(
        &self,
        plan: &Plan,
        load: f32,
        in_slew: f32,
        wirelength: u32,
        topology: &[Piece],
    ) -> Result<Measured, CtsError> {
        let in_cap = if self.pattern.is_pure_wire() {
            (f64::from(load) + f64::from(wirelength) * self.wire.cap.get()) as f32
        } else {
            let first_length = match topology.first() {
                Some(Piece::Wire(length)) => *length as f32,
                _ => 0.0,
            };
            let first = self.masters[self.config[0]];
            let first_pin_cap = self.liberty.cell(first.id).ports[first.input]
                .capacitance()
                .get() as f32;
            (f64::from(first_pin_cap) + f64::from(first_length) * self.wire.cap.get()) as f32
        };
        let total_cap = (in_cap / plan.cap_step).round() * plan.cap_step;
        let (fall_arrival, fall_slew) = self.propagate(Edge::Fall, load, in_slew)?;
        let (_, rise_slew) = self.propagate(Edge::Rise, load, in_slew)?;
        let pin_slew =
            (f32::midpoint(rise_slew, fall_slew) / plan.slew_step).round() * plan.slew_step;
        Ok(Measured {
            load,
            in_slew,
            wirelength: wirelength as f32,
            pin_slew,
            pin_arrival: fall_arrival,
            total_cap,
            pure_wire: self.pattern.is_pure_wire(),
            topology: topology.to_vec(),
        })
    }
}

/// The ranges of the normalized quantities.
#[derive(Clone, Copy, Debug)]
struct AllBounds {
    slew: Bounds,
    cap: Bounds,
    wirelength: Bounds,
}

/// Keeps the measurements within the slew limit and normalizes them, as
/// `TechChar::characterizationPostProcess`.
fn post_process(measurements: &[Measured], plan: &Plan) -> (Vec<Normalized>, AllBounds) {
    let mut bounds = AllBounds {
        slew: Bounds::new(),
        cap: Bounds::new(),
        wirelength: Bounds::new(),
    };
    let mut lines = Vec::new();
    for m in measurements.iter().filter(|m| m.pin_slew <= plan.max_slew) {
        let pin_slew = bounds.slew.normalize(m.pin_slew, plan.slew_step) as f32;
        let in_slew = bounds.slew.normalize(m.in_slew, plan.slew_step) as f32;
        let total_cap = bounds.cap.normalize(m.total_cap, plan.cap_step) as f32;
        let load = bounds.cap.normalize(m.load, plan.cap_step) as f32;
        let wirelength = bounds
            .wirelength
            .normalize(m.wirelength, plan.char_unit as f32) as f32;
        let pin_arrival = (m.pin_arrival / (plan.slew_step / DELAY_STEPS_PER_SLEW_STEP)).ceil();
        let topology = m
            .topology
            .iter()
            .map(|piece| match *piece {
                Piece::Wire(length) => {
                    NormalizedPiece::Share(round_six_decimals(length / f64::from(m.wirelength)))
                }
                Piece::Buffer(b) => NormalizedPiece::Buffer(b),
            })
            .collect();
        lines.push(Normalized {
            load,
            in_slew,
            wirelength,
            pin_slew,
            pin_arrival,
            total_cap,
            pure_wire: m.pure_wire,
            topology,
        });
    }
    (lines, bounds)
}

/// `value` written with six decimals and read back, as `TechChar` stores
/// a pattern's wire shares in the text of its table.
fn round_six_decimals(value: f64) -> f64 {
    // Whatever `{:.6}` writes, NaN and infinities included, parses back.
    format!("{value:.6}").parse().unwrap_or(value)
}

/// Builds the table from the normalized measurements, as
/// `TechChar::compileLut`.
fn compile(
    lines: &[Normalized],
    bounds: &AllBounds,
    plan: &Plan,
    masters: &[BufferCell],
    cap_per_dbu: FaradsPerDbu,
) -> Result<TechChar, CtsError> {
    let ratio = plan.char_unit / plan.length_unit;
    let mut table = TechChar {
        segments: Vec::with_capacity(lines.len()),
        by_key: FxHashMap::default(),
        min_segment_length: bounds.wirelength.min * ratio,
        max_segment_length: bounds.wirelength.max * ratio,
        max_capacitance: bounds.cap.max,
        max_slew: bounds.slew.max,
        actual_min_input_cap: u32::MAX,
        length_unit: plan.length_unit,
        cap_per_dbu,
        sink_buffer_input_cap: plan.sink_buffer_input_cap,
        wire_delay_rate: None,
    };
    let limits = [
        table.min_segment_length,
        table.max_segment_length,
        bounds.cap.min,
        table.max_capacitance,
        bounds.slew.min,
        table.max_slew,
    ];
    if limits.iter().any(|&v| v > MAX_NORMALIZED) {
        return Err(CtsError::CharacterizationRange(MAX_NORMALIZED));
    }
    for line in lines {
        table.actual_min_input_cap = table.actual_min_input_cap.min(line.total_cap as u32);
        let mut pin_slew = line.pin_slew;
        if line.pure_wire && pin_slew <= line.in_slew {
            pin_slew += 1.0;
        }
        let length = line.wirelength as u32 * ratio;
        let scale = |share: f64| (share * f64::from(length)).round() as i32;
        let mut segment = WireSegment {
            length: length as u8,
            load: line.load as u32 as u8,
            output_slew: pin_slew as u32 as u8,
            delay: line.pin_arrival as u32,
            input_cap: line.total_cap as u32 as u8,
            input_slew: line.in_slew as u32 as u8,
            buffer_locations: Vec::new(),
            buffer_masters: Vec::new(),
            wl_to_first_buffer: 0,
            last_wl: 0,
        };
        let first_share = match line.topology.first() {
            Some(NormalizedPiece::Share(s)) => *s,
            _ => 0.0,
        };
        segment.wl_to_first_buffer = scale(first_share);
        if line.pure_wire {
            segment.last_wl = segment.wl_to_first_buffer;
        } else {
            let paired = line.topology.len() - line.topology.len() % 2;
            if paired < line.topology.len()
                && let NormalizedPiece::Share(s) = line.topology[paired]
            {
                segment.last_wl = scale(s);
            }
            for piece in &line.topology[..paired] {
                match *piece {
                    NormalizedPiece::Share(s) => segment.buffer_locations.push(s),
                    NormalizedPiece::Buffer(b) => segment.buffer_masters.push(masters[b]),
                }
            }
        }
        table.push(segment);
    }
    table.wire_delay_rate = least_wire_delay_rate(&table.segments, delay_step(plan));
    Ok(table)
}

/// The time a step of a segment's delay stands for.
fn delay_step(plan: &Plan) -> Seconds {
    Seconds::new(f64::from(plan.slew_step / DELAY_STEPS_PER_SLEW_STEP))
}

/// The least delay per length unit among the buffered `segments` whose
/// output slew is no worse than their input slew, so that a run of them
/// could go on without end; none when no buffered segment keeps its slew.
fn least_wire_delay_rate(segments: &[WireSegment], delay_step: Seconds) -> Option<WireDelayRate> {
    segments
        .iter()
        .filter(|s| s.is_buffered() && s.length > 0 && s.output_slew <= s.input_slew)
        .map(|s| delay_step * f64::from(s.delay) / f64::from(s.length))
        .reduce(Seconds::min)
        .map(WireDelayRate)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fixtures;
    use pare_common::db::liberty::LibCellId;
    use pare_common::db::parser::liberty;
    use std::path::Path;

    /// A library of the one clock buffer `CLKBUF_X3`, with `defaults` at
    /// library level, `limits` on its output and `timing` in its arc.
    fn clock_buffer_library(defaults: &str, limits: &str, timing: &str) -> Liberty {
        let src = format!(
            "library (x) {{
               capacitive_load_unit (1,ff) ;
               {defaults}
               lu_table_template (delay_t) {{
                 variable_1 : input_net_transition ;
                 variable_2 : total_output_net_capacitance ;
                 index_1 (\"0.005, 0.2\") ;
                 index_2 (\"1.0, 100.0\") ;
               }}
               lu_table_template (load_t) {{
                 variable_1 : total_output_net_capacitance ;
                 index_1 (\"1.0, 100.0\") ;
               }}
               cell (CLKBUF_X3) {{
                 area : 1.33 ;
                 pin (A) {{ direction : input ; capacitance : 1.4 ; }}
                 pin (Z) {{
                   direction : output ; function : \"A\" ; {limits}
                   timing () {{
                     related_pin : \"A\" ; timing_sense : positive_unate ; {timing}
                   }}
                 }}
               }}
             }}"
        );
        liberty::parse_str(&src, Path::new("x.lib")).unwrap()
    }

    /// Delay tables over input slew and load.
    const TWO_AXIS_TIMING: &str =
        "cell_rise (delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }
         cell_fall (delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }";

    /// What `plan` makes of `lib` characterizing its buffer in `db`.
    fn plan_of(db: &NetlistDB, lib: &Liberty) -> Result<Plan, CtsError> {
        let link = Link::new(db, lib, &[]).unwrap();
        let id = lib.cell_id("CLKBUF_X3").unwrap();
        let buffer = lib.buffer_cell(id).unwrap();
        let layer = db.library.routing_layer_id("metal3").unwrap();
        plan(&CharSetup {
            link: &link,
            db,
            masters: &[buffer],
            sink_buffer: buffer,
            clock_rc: WireRc::per_micron(1.5, 0.11, layer),
        })
    }

    #[test]
    fn a_planned_characterization_steps_through_the_buffers_axes() {
        let lib = clock_buffer_library(
            "",
            "max_capacitance : 180.0 ; max_transition : 0.2 ;",
            TWO_AXIS_TIMING,
        );

        let plan = plan_of(&fixtures::design(""), &lib).unwrap();

        assert_eq!(
            (plan.slews.len(), plan.loads.len()),
            (SLEW_STEPS, CAP_STEPS)
        );
        assert_eq!((plan.char_unit, plan.length_unit), (28_000, 14_000));
        assert_eq!(plan.wirelengths, [28_000, 56_000, 84_000, 112_000]);
    }

    #[test]
    fn a_characterization_buffer_without_a_slew_limit_is_refused() {
        let lib = clock_buffer_library("", "max_capacitance : 180.0 ;", TWO_AXIS_TIMING);

        let refused = plan_of(&fixtures::design(""), &lib).err();

        assert_eq!(refused, Some(CtsError::NoMaxSlew("CLKBUF_X3".to_string())));
    }

    #[test]
    fn a_buffer_without_a_load_limit_is_refused() {
        let lib = clock_buffer_library("default_max_transition : 0.2 ;", "", TWO_AXIS_TIMING);

        let refused = plan_of(&fixtures::design(""), &lib).err();

        assert_eq!(
            refused,
            Some(CtsError::NoMaxCapacitance("CLKBUF_X3".to_string()))
        );
    }

    #[test]
    fn delay_tables_with_a_load_axis_but_no_slew_axis_are_refused() {
        let lib = clock_buffer_library(
            "",
            "max_capacitance : 180.0 ; max_transition : 0.2 ;",
            "cell_rise (load_t) { values (\"0.02, 0.08\") ; }
             cell_fall (load_t) { values (\"0.02, 0.08\") ; }",
        );

        let refused = plan_of(&fixtures::design(""), &lib).err();

        assert_eq!(
            refused,
            Some(CtsError::NoTableAxes("CLKBUF_X3".to_string()))
        );
    }

    /// A segment of `length` units with `delay` steps, `input_slew` and
    /// `output_slew` steps, buffered or bare.
    fn segment(
        length: u8,
        delay: u32,
        input_slew: u8,
        output_slew: u8,
        buffered: bool,
    ) -> WireSegment {
        WireSegment {
            length,
            load: 1,
            output_slew,
            delay,
            input_cap: 1,
            input_slew,
            buffer_locations: if buffered { vec![0.5] } else { Vec::new() },
            buffer_masters: if buffered {
                vec![BufferCell {
                    id: LibCellId(0),
                    input: LibPortId::new(0),
                    output: LibPortId::new(1),
                }]
            } else {
                Vec::new()
            },
            wl_to_first_buffer: 0,
            last_wl: 0,
        }
    }

    const STEP: Seconds = Seconds::new(1e-12);

    #[test]
    fn the_rate_is_the_least_delay_per_length_of_buffered_segments_keeping_their_slew() {
        let segments = [
            segment(5, 10, 3, 3, true),
            segment(2, 6, 3, 2, true),
            segment(10, 1, 2, 4, true),
            segment(10, 1, 3, 3, false),
        ];

        let rate = least_wire_delay_rate(&segments, STEP).unwrap();

        assert_eq!(rate.length_of(Seconds::new(10e-12)), SegmentLength(5.0));
    }

    #[test]
    fn without_a_buffered_segment_keeping_its_slew_there_is_no_rate() {
        let segments = [segment(10, 1, 2, 4, true), segment(10, 1, 3, 3, false)];

        assert_eq!(least_wire_delay_rate(&segments, STEP), None);
    }

    #[test]
    fn a_value_normalizes_up_to_whole_steps_and_at_least_one() {
        let mut bounds = Bounds::new();

        let normalized = [2.5, 0.0, 4.0].map(|v| bounds.normalize(v, 1.0));

        assert_eq!(normalized, [3, 1, 4]);
        assert_eq!((bounds.min, bounds.max), (1, 4));
    }

    #[test]
    fn axis_values_within_a_percent_of_the_last_kept_are_dropped() {
        let mut values = vec![1.016, 1.0, 1.008, 1.0];

        sort_and_uniquify(&mut values);

        assert_eq!(values, [1.0, 1.016]);
    }

    #[test]
    fn too_many_axis_values_lose_the_one_closest_to_its_predecessor() {
        let mut values = vec![0.0, 1.0, 1.5, 4.0];

        reduce_or_expand(&mut values, 3);

        assert_eq!(values, [0.0, 1.0, 4.0]);
    }

    #[test]
    fn too_few_axis_values_split_the_first_widest_gap() {
        let mut values = vec![0.0, 1.0, 4.0];

        reduce_or_expand(&mut values, 5);

        assert_eq!(values, [0.0, 1.0, 1.75, 2.5, 4.0]);
    }

    #[test]
    fn a_single_axis_value_has_no_gap_to_split() {
        let mut values = vec![2.0];

        reduce_or_expand(&mut values, 3);

        assert_eq!(values, [2.0]);
    }

    #[test]
    fn patterns_place_a_buffer_or_none_at_each_node() {
        let descriptors: Vec<Vec<Piece>> =
            patterns(20, 10).into_iter().map(|p| p.descriptor).collect();

        assert_eq!(
            descriptors,
            [
                vec![Piece::Wire(20.0)],
                vec![Piece::Wire(10.0), Piece::Buffer(0), Piece::Wire(10.0)],
                vec![Piece::Wire(20.0), Piece::Buffer(0)],
                vec![
                    Piece::Wire(10.0),
                    Piece::Buffer(0),
                    Piece::Wire(10.0),
                    Piece::Buffer(0)
                ],
            ]
        );
    }

    #[test]
    fn a_pattern_ending_in_a_buffer_has_an_empty_last_net() {
        let ending_in_buffer = &patterns(20, 10)[2];

        assert_eq!(ending_in_buffer.nets, [2, 0]);
        assert!(!ending_in_buffer.is_pure_wire());
    }

    #[test]
    fn buffer_configurations_never_shrink_along_the_chain() {
        assert_eq!(
            buffer_configs(2, 2, 0),
            [vec![0, 0], vec![0, 1], vec![1, 1]]
        );
    }

    #[test]
    fn buffer_configurations_start_at_the_characterization_buffer_and_wrap() {
        assert_eq!(
            buffer_configs(2, 2, 1),
            [vec![1, 1], vec![0, 0], vec![0, 1]]
        );
    }
}
