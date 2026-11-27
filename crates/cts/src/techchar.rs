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
use pare_common::convert::{NearestFrom, SaturatingFrom, WrappingFrom};
use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::{
    AxisVariable, BufferCell, Edge, LibCell, LibPortId, Liberty, Thresholds,
};
use pare_common::geom::dbu::Dbu;
use pare_common::units::{Farads, FaradsPerDbu, Ohms, OhmsPerDbu, Seconds};
use pare_resizer::Link;
use pare_resizer::WireRc;
use pare_resizer::dmp::{self, PiModel};
use rustc_hash::FxHashMap;
use std::cmp::Ordering;
use std::collections::BTreeMap;

/// The input slews characterized, as `CtsOptions`' default `slewSteps`.
const SLEW_STEPS: usize = 7;
/// The loads characterized, as `CtsOptions`' default `capSteps`.
const CAP_STEPS: usize = 20;
/// The wire lengths characterized, in characterization units, as
/// `CtsOptions`' default `charWirelengthIterations`.
const WIRELENGTH_ITERATIONS: u32 = 4;
/// The bits `TechChar` packs each field of a segment's key into, as its
/// `NUM_BITS_PER_FIELD`.
const NUM_BITS_PER_FIELD: u32 = 10;
/// The largest normalized value the characterization takes, the most a
/// key field holds, as `TechChar`'s `MAX_NORMALIZED_VAL`.
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

    /// The load at its far end.
    #[must_use]
    pub(crate) fn load_steps(&self) -> CapSteps {
        CapSteps(u32::from(self.load))
    }

    /// The slew at its far end.
    #[must_use]
    pub(crate) fn output_slew_steps(&self) -> SlewSteps {
        SlewSteps(u32::from(self.output_slew))
    }

    /// The capacitance it presents at its start.
    #[must_use]
    pub(crate) fn input_cap_steps(&self) -> CapSteps {
        CapSteps(u32::from(self.input_cap))
    }

    /// The slew at its start.
    #[must_use]
    pub(crate) fn input_slew_steps(&self) -> SlewSteps {
        SlewSteps(u32::from(self.input_slew))
    }

    /// The wire from its start to its first buffer.
    #[must_use]
    pub(crate) fn to_first_buffer(&self) -> LengthSteps {
        LengthSteps(i64::from(self.wl_to_first_buffer))
    }

    /// The wire after its last buffer.
    #[must_use]
    pub(crate) fn after_last_buffer(&self) -> LengthSteps {
        LengthSteps(i64::from(self.last_wl))
    }
}

/// A whole number of the characterization's length units, as table lengths
/// and the wire since the last buffer count.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct LengthSteps(i64);

impl LengthSteps {
    /// `steps` length units.
    #[must_use]
    pub(crate) const fn new(steps: i64) -> Self {
        Self(steps)
    }

    /// The number of length units.
    #[must_use]
    pub(crate) const fn get(self) -> i64 {
        self.0
    }

    /// How many whole `step`s fit, rounding toward zero.
    #[must_use]
    pub(crate) const fn count_of(self, step: Self) -> i64 {
        self.0 / step.0
    }
}

pare_common::newtype_ops!(LengthSteps: Add, AddAssign, Sub, Mul<i64>);

/// A load in the characterization's capacitance steps.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct CapSteps(u32);

/// A slew in the characterization's slew steps.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct SlewSteps(u32);

macro_rules! step_count {
    ($name:ident) => {
        impl $name {
            /// `steps` steps.
            #[must_use]
            pub(crate) const fn new(steps: u32) -> Self {
                Self(steps)
            }

            /// The number of steps.
            #[must_use]
            pub(crate) const fn get(self) -> u32 {
                self.0
            }

            /// How many steps apart `self` and `other` are.
            #[must_use]
            pub(crate) const fn steps_from(self, other: Self) -> u32 {
                self.0.abs_diff(other.0)
            }

            /// Every step from the first through `last`.
            pub(crate) fn through(last: Self) -> impl Iterator<Item = Self> {
                (1..=last.0).map(Self)
            }
        }
    };
}

step_count!(CapSteps);
step_count!(SlewSteps);

impl SlewSteps {
    /// The first step, the least slew the characterization takes.
    pub(crate) const FIRST: Self = Self(1);

    /// The step after this one.
    #[must_use]
    pub(crate) const fn next(self) -> Self {
        Self(self.0 + 1)
    }
}

/// The database units one length unit of the characterization spans.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct LengthUnit(u32);

impl LengthUnit {
    /// A unit `dbu` database units long.
    #[must_use]
    pub(crate) const fn new(dbu: u32) -> Self {
        Self(dbu)
    }

    /// The unit in database units, in double precision.
    #[must_use]
    pub(crate) fn to_f64(self) -> f64 {
        f64::from(self.0)
    }

    /// The unit in database units, in the single precision `TritonCTS`
    /// scales sinks by.
    #[must_use]
    pub(crate) const fn to_f32(self) -> f32 {
        self.0 as f32
    }

    /// The unit in database units.
    #[must_use]
    pub(crate) const fn dbu(self) -> u32 {
        self.0
    }

    /// The whole length units in `length`, rounding toward zero.
    #[must_use]
    pub(crate) const fn steps_in(self, length: Dbu) -> LengthSteps {
        LengthSteps(length.div_trunc(Dbu::new(self.0 as i64)))
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
    by_key: FxHashMap<SegmentKey, Vec<usize>>,
    min_segment_length: LengthSteps,
    max_segment_length: LengthSteps,
    max_capacitance: CapSteps,
    max_slew: SlewSteps,
    actual_min_input_cap: CapSteps,
    length_unit: LengthUnit,
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

impl Measured {
    /// The key the measurement is grouped under.
    const fn key(&self) -> CharKey {
        CharKey {
            load: self.load,
            wirelength: self.wirelength,
            pin_slew: self.pin_slew,
            total_cap: self.total_cap,
        }
    }
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

/// The range the normalized values of one quantity span.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Bounds {
    min: u32,
    max: u32,
}

/// `value` in whole `step`s, rounded up and at least one, as
/// `TechChar::characterizationPostProcess` normalizes it, widening
/// `bounds` to take it in.
fn normalize(bounds: &mut Option<Bounds>, value: f32, step: f32) -> u32 {
    let normalized = u32::saturating_from((value / step).ceil()).max(1);
    *bounds = Some(bounds.map_or(
        Bounds {
            min: normalized,
            max: normalized,
        },
        |b| Bounds {
            min: b.min.min(normalized),
            max: b.max.max(normalized),
        },
    ));
    normalized
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
        let plan = plan(setup)?;
        let mut groups: BTreeMap<CharKey, Vec<Measured>> = BTreeMap::new();
        for_each_chain(setup, &plan, |chain, wirelength| {
            let topology = chain.topology();
            for &load in &plan.loads {
                for &in_slew in &plan.slews {
                    let measured = chain.measure(&plan, load, in_slew, wirelength, &topology)?;
                    groups.entry(measured.key()).or_default().push(measured);
                }
            }
            Ok(())
        })?;
        let measurements: Vec<Measured> = groups.into_values().flatten().collect();
        tabulate(setup, &plan, &measurements)
    }

    /// The segment at `index`.
    #[must_use]
    pub fn segment(&self, index: usize) -> &WireSegment {
        &self.segments[index]
    }

    /// The segments of a length, load and output slew, in table order.
    #[must_use]
    pub(crate) fn segments_with(
        &self,
        length: LengthSteps,
        load: CapSteps,
        output_slew: SlewSteps,
    ) -> &[usize] {
        self.by_key
            .get(&SegmentKey::new(length, load, output_slew))
            .map_or(&[], Vec::as_slice)
    }

    /// The shortest characterized length, in length units.
    #[must_use]
    pub(crate) const fn min_segment_length(&self) -> LengthSteps {
        self.min_segment_length
    }

    /// The longest characterized length, in length units.
    #[must_use]
    pub(crate) const fn max_segment_length(&self) -> LengthSteps {
        self.max_segment_length
    }

    /// The largest normalized load.
    #[must_use]
    pub const fn max_capacitance(&self) -> CapSteps {
        self.max_capacitance
    }

    /// The largest normalized slew.
    #[must_use]
    pub const fn max_slew(&self) -> SlewSteps {
        self.max_slew
    }

    /// The smallest normalized input capacitance of any segment.
    #[must_use]
    pub(crate) const fn actual_min_input_cap(&self) -> CapSteps {
        self.actual_min_input_cap
    }

    /// The least delay per length unit among the buffered segments that
    /// keep their slew, the rate at which a long run of the tree's own wire
    /// delays the clock; none when no buffered segment keeps its slew.
    #[must_use]
    pub(crate) const fn wire_delay_rate(&self) -> Option<WireDelayRate> {
        self.wire_delay_rate
    }

    /// The length unit.
    #[must_use]
    pub(crate) const fn length_unit(&self) -> LengthUnit {
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
    pub(crate) fn create_fake_entries(&mut self, length: LengthSteps, fake_length: LengthSteps) {
        if length == fake_length {
            return;
        }
        for load in CapSteps::through(self.max_capacitance) {
            for output_slew in SlewSteps::through(self.max_slew) {
                let found: Vec<usize> = self.segments_with(length, load, output_slew).to_vec();
                for index in found {
                    let mut fake = self.segments[index].clone();
                    fake.length = u8::wrapping_from(fake_length.get());
                    self.push(fake);
                }
            }
        }
    }

    fn push(&mut self, segment: WireSegment) {
        let key = SegmentKey::of(&segment);
        self.segments.push(segment);
        self.by_key
            .entry(key)
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
            min_segment_length: LengthSteps(i64::from(min_segment_length)),
            max_segment_length: LengthSteps(i64::from(max_segment_length)),
            max_capacitance: CapSteps(range(|s| s.load).1),
            max_slew: SlewSteps(range(|s| s.output_slew).1),
            actual_min_input_cap: CapSteps(range(|s| s.input_cap).0),
            length_unit: LengthUnit::new(length_unit),
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

/// What the table files a segment under: its length, load and output
/// slew, each in the eight bits `TechChar::computeKey` takes it in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct SegmentKey {
    length: u8,
    load: u8,
    output_slew: u8,
}

impl SegmentKey {
    /// The key a lookup of these steps lands on.
    // NOTE: kept: computeKey takes eight-bit fields, so a lookup past 255
    // wraps as the segments' own eight-bit fields do.
    fn new(length: LengthSteps, load: CapSteps, output_slew: SlewSteps) -> Self {
        Self {
            length: u8::wrapping_from(length.get()),
            load: u8::wrapping_from(load.get()),
            output_slew: u8::wrapping_from(output_slew.get()),
        }
    }

    /// The key `segment` is filed under.
    const fn of(segment: &WireSegment) -> Self {
        Self {
            length: segment.length,
            load: segment.load,
            output_slew: segment.output_slew,
        }
    }
}

/// Picks the characterization buffer, the lengths, loads and slews, as
/// `TechChar::initCharacterization`.
fn plan(setup: &CharSetup<'_>) -> Result<Plan, CtsError> {
    let liberty = setup.link.liberty;
    let CharBuffer {
        index: char_index,
        buffer: char_buffer,
        axes,
    } = pick_char_buffer(setup)?;
    let height = buffer_height(setup, char_buffer)?;
    let char_unit = u32::saturating_from(height * 10);
    let cell = liberty.cell(char_buffer.id);
    let (max_slew, max_cap) = char_limits(liberty, cell, char_buffer)?;
    let sink = setup.sink_buffer;
    let sink_buffer_input_cap = Farads32::of(liberty.cell(sink.id).ports[sink.input].capacitance());
    let slew_step = max_slew / SLEW_STEPS as f32;
    let cap_step = max_cap / CAP_STEPS as f32;
    let (slews, loads) = if axes.slews.is_empty() && axes.loads.is_empty() {
        (
            (1..=SLEW_STEPS).map(|s| s as f32 * slew_step).collect(),
            (1..=CAP_STEPS).map(|s| s as f32 * cap_step).collect(),
        )
    } else {
        table_test_points(axes, (max_slew, max_cap), &cell.name)?
    };
    Ok(Plan {
        char_buffer: char_index,
        char_unit,
        length_unit: u32::saturating_from(height * 10 / 2),
        wirelengths: wire_lengths(char_unit),
        max_slew,
        slew_step,
        cap_step,
        slews,
        loads,
        sink_buffer_input_cap,
    })
}

/// The slews and loads of the buffers' delay tables.
struct TableAxes {
    slews: Vec<f32>,
    loads: Vec<f32>,
}

/// The buffer characterized, its position among the masters, and the
/// axes of every master's delay tables.
struct CharBuffer {
    index: usize,
    buffer: BufferCell,
    axes: TableAxes,
}

/// The buffer with the largest load limit, the last of equal ones.
fn pick_char_buffer(setup: &CharSetup<'_>) -> Result<CharBuffer, CtsError> {
    let liberty = setup.link.liberty;
    let mut char_buffer: Option<(usize, BufferCell)> = None;
    let mut max_buffer_cap = 0.0_f32;
    let mut axis_slews: Vec<f32> = Vec::new();
    let mut axis_loads: Vec<f32> = Vec::new();
    for (index, &master) in setup.masters.iter().enumerate() {
        let cell = liberty.cell(master.id);
        let max_cap = max_capacitance(liberty, cell, master)?;
        if max_cap >= max_buffer_cap {
            max_buffer_cap = max_cap;
            char_buffer = Some((index, master));
        }
        collect_table_axes(
            cell,
            master.input,
            master.output,
            &mut axis_slews,
            &mut axis_loads,
        );
    }
    let (index, buffer) = char_buffer.ok_or(CtsError::NoBuffers)?;
    Ok(CharBuffer {
        index,
        buffer,
        axes: TableAxes {
            slews: axis_slews,
            loads: axis_loads,
        },
    })
}

/// A buffer's output load limit, from the cell or the library's default.
fn max_capacitance(liberty: &Liberty, cell: &LibCell, buffer: BufferCell) -> Result<f32, CtsError> {
    let limit = cell.ports[buffer.output]
        .max_capacitance
        .or_else(|| liberty.cell_defaults(cell).max_capacitance)
        .ok_or_else(|| CtsError::NoMaxCapacitance(cell.name.clone()))?;
    Ok(f32::nearest_from(limit.get()))
}

/// The characterization buffer's slew and load limits.
fn char_limits(
    liberty: &Liberty,
    cell: &LibCell,
    buffer: BufferCell,
) -> Result<(f32, f32), CtsError> {
    let max_slew = f32::nearest_from(
        cell.ports[buffer.output]
            .max_transition
            .or(cell.ports[buffer.input].max_transition)
            .or_else(|| liberty.cell_defaults(cell).max_transition)
            .ok_or_else(|| CtsError::NoMaxSlew(cell.name.clone()))?
            .get(),
    );
    Ok((max_slew, max_capacitance(liberty, cell, buffer)?))
}

/// The height of a buffer's master, in database units.
fn buffer_height(setup: &CharSetup<'_>, buffer: BufferCell) -> Result<i64, CtsError> {
    let liberty = setup.link.liberty;
    let height = setup
        .link
        .macro_of(buffer.id)
        .map(|m| setup.db.library.cell_macro(m).size.y.get())
        .ok_or_else(|| CtsError::NoMaster(liberty.cell(buffer.id).name.clone()))?;
    // NOTE: initCharacterization steps its wire lengths by ten heights
    // and never ends on a zero-height buffer; PARE refuses it.
    if height == 0 {
        return Err(CtsError::ZeroHeightBuffer(
            liberty.cell(buffer.id).name.clone(),
        ));
    }
    Ok(height)
}

/// The wire lengths characterized, a characterization unit apart, as
/// `initCharacterization` steps them: with no wire unit of the user's and
/// at least two iterations, both its bounds come to the last iteration.
fn wire_lengths(char_unit: u32) -> Vec<u32> {
    (1..=WIRELENGTH_ITERATIONS).map(|i| i * char_unit).collect()
}

/// The slews and loads to test, from the delay tables' axes with the
/// limits added, reduced or expanded to the step counts.
fn table_test_points(
    TableAxes {
        mut slews,
        mut loads,
    }: TableAxes,
    (max_slew, max_cap): (f32, f32),
    cell_name: &str,
) -> Result<(Vec<f32>, Vec<f32>), CtsError> {
    // NOTE: initCharacterization expands an empty axis, reading past it; PARE refuses.
    if slews.is_empty() || loads.is_empty() {
        return Err(CtsError::NoTableAxes(cell_name.to_owned()));
    }
    slews.push(max_slew);
    sort_and_uniquify(&mut slews);
    loads.push(max_cap);
    sort_and_uniquify(&mut loads);
    reduce_or_expand(&mut slews, SLEW_STEPS);
    reduce_or_expand(&mut loads, CAP_STEPS);
    Ok((slews, loads))
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
                    slews.extend(axis.values.iter().map(|&v| f32::nearest_from(v)));
                }
                AxisVariable::OutputCapacitance => {
                    loads.extend(axis.values.iter().map(|&v| f32::nearest_from(v)));
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
    // NOTE: reduceOrExpand reads past a single value to split it; PARE keeps it.
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

/// Calls `visit` with each chain `TechChar::create` times and its length,
/// in its order: every pattern of each characterized length, with each
/// assignment of buffer types to it.
fn for_each_chain(
    setup: &CharSetup<'_>,
    plan: &Plan,
    mut visit: impl FnMut(&Chain<'_>, u32) -> Result<(), CtsError>,
) -> Result<(), CtsError> {
    let liberty = setup.link.liberty;
    let wire = WirePerDbu::of(setup);
    let patterns_by_length = plan.wirelengths.iter().flat_map(|&wirelength| {
        patterns(wirelength, plan.char_unit)
            .into_iter()
            .map(move |pattern| (wirelength, pattern))
    });
    for (wirelength, pattern) in patterns_by_length {
        for config in buffer_configs(setup.masters.len(), pattern.buffers.len(), plan.char_buffer) {
            let chain = Chain {
                liberty,
                thresholds: &liberty.thresholds,
                masters: setup.masters,
                config: &config,
                pattern: &pattern,
                unit: plan.char_unit,
                wire,
            };
            visit(&chain, wirelength)?;
        }
    }
    Ok(())
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
    let total = masters.pow(u32::saturating_from(nodes));
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
    let mut number: usize = (0..nodes)
        .map(|i| start * masters.pow(u32::saturating_from(i)))
        .sum();
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

impl WirePerDbu {
    /// The clock wire of `setup`.
    fn of(setup: &CharSetup<'_>) -> Self {
        Self {
            res: setup.clock_rc.resistance.per_dbu(setup.db.dbu()),
            cap: setup.clock_rc.capacitance.per_dbu(setup.db.dbu()),
        }
    }
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
        let c2 = f32::nearest_from(cap / 2.0);
        let rpi = f32::nearest_from(res);
        let c1 = f32::nearest_from(cap / 2.0);
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
                elmore: f32::nearest_from(res * cap),
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
            let vth = f64::from(f32::nearest_from(t.input.get(edge)));
            f32::nearest_from(-f64::from(self.wire_pi(0, load).elmore) * (1.0 - vth).ln())
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
                    f32::nearest_from(
                        self.liberty.cell(next.id).ports[next.input]
                            .capacitance
                            .get(edge)
                            .get(),
                    )
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
                    let at = arrival
                        + f32::nearest_from(gate.delay.get())
                        + f32::nearest_from(wire_delay.get());
                    let s = f32::nearest_from(load_slew.get());
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
            f32::nearest_from(f64::from(load) + f64::from(wirelength) * self.wire.cap.get())
        } else {
            let first_length = match topology.first() {
                Some(Piece::Wire(length)) => f32::nearest_from(*length),
                _ => 0.0,
            };
            let first = self.masters[self.config[0]];
            let first_pin_cap = f32::nearest_from(
                self.liberty.cell(first.id).ports[first.input]
                    .capacitance()
                    .get(),
            );
            f32::nearest_from(
                f64::from(first_pin_cap) + f64::from(first_length) * self.wire.cap.get(),
            )
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

/// The measurements within the slew limit, normalized, and the ranges
/// they span.
#[derive(Clone, Debug)]
struct Normalization {
    lines: Vec<Normalized>,
    bounds: AllBounds,
}

/// Keeps the measurements within the slew limit and normalizes them, as
/// `TechChar::characterizationPostProcess`; none when none is within it.
fn post_process(measurements: &[Measured], plan: &Plan) -> Option<Normalization> {
    let (mut slew, mut cap, mut wire) = (None, None, None);
    let mut lines = Vec::new();
    for m in measurements.iter().filter(|m| m.pin_slew <= plan.max_slew) {
        let pin_slew = normalize(&mut slew, m.pin_slew, plan.slew_step) as f32;
        let in_slew = normalize(&mut slew, m.in_slew, plan.slew_step) as f32;
        let total_cap = normalize(&mut cap, m.total_cap, plan.cap_step) as f32;
        let load = normalize(&mut cap, m.load, plan.cap_step) as f32;
        let wirelength = normalize(&mut wire, m.wirelength, plan.char_unit as f32) as f32;
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
    Some(Normalization {
        lines,
        bounds: AllBounds {
            slew: slew?,
            cap: cap?,
            wirelength: wire?,
        },
    })
}

/// The table of `measurements` taken over `plan`, as `TechChar::create`
/// finishes it; an error when no segment keeps its slew within the limit.
fn tabulate(
    setup: &CharSetup<'_>,
    plan: &Plan,
    measurements: &[Measured],
) -> Result<TechChar, CtsError> {
    // NOTE: characterizationPostProcess seeds its ranges with UINT_MAX and 0,
    // which overflow when nothing is kept; PARE refuses an empty table.
    let normalization =
        post_process(measurements, plan).ok_or_else(|| CtsError::NoSegmentWithinSlewLimit {
            buffer: setup
                .link
                .liberty
                .cell(setup.masters[plan.char_buffer].id)
                .name
                .clone(),
            max_transition: Seconds::new(f64::from(plan.max_slew)),
        })?;
    compile(
        &normalization,
        plan,
        setup.masters,
        WirePerDbu::of(setup).cap,
    )
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
    normalization: &Normalization,
    plan: &Plan,
    masters: &[BufferCell],
    cap_per_dbu: FaradsPerDbu,
) -> Result<TechChar, CtsError> {
    let Normalization { lines, bounds } = normalization;
    let ratio = plan.char_unit / plan.length_unit;
    // The top of the load range bounds every input capacitance from above.
    let actual_min_input_cap = lines
        .iter()
        .map(|line| u32::saturating_from(line.total_cap))
        .fold(bounds.cap.max, u32::min);
    let min_segment_length = bounds.wirelength.min * ratio;
    let max_segment_length = bounds.wirelength.max * ratio;
    let limits = [
        min_segment_length,
        max_segment_length,
        bounds.cap.min,
        bounds.cap.max,
        bounds.slew.min,
        bounds.slew.max,
    ];
    if limits.iter().any(|&v| v > MAX_NORMALIZED) {
        return Err(CtsError::CharacterizationRange(MAX_NORMALIZED));
    }
    let mut table = TechChar {
        segments: Vec::with_capacity(lines.len()),
        by_key: FxHashMap::default(),
        min_segment_length: LengthSteps(i64::from(min_segment_length)),
        max_segment_length: LengthSteps(i64::from(max_segment_length)),
        max_capacitance: CapSteps(bounds.cap.max),
        max_slew: SlewSteps(bounds.slew.max),
        actual_min_input_cap: CapSteps(actual_min_input_cap),
        length_unit: LengthUnit::new(plan.length_unit),
        cap_per_dbu,
        sink_buffer_input_cap: plan.sink_buffer_input_cap,
        wire_delay_rate: None,
    };
    for line in lines {
        table.push(segment_from(line, ratio, masters));
    }
    table.wire_delay_rate = least_wire_delay_rate(&table.segments, delay_step(plan));
    Ok(table)
}

/// The table segment of one normalized measurement, its length scaled by
/// `ratio` to length units, as `TechChar::compileLut` fills a segment.
fn segment_from(line: &Normalized, ratio: u32, masters: &[BufferCell]) -> WireSegment {
    let mut pin_slew = line.pin_slew;
    if line.pure_wire && pin_slew <= line.in_slew {
        pin_slew += 1.0;
    }
    let length = u32::saturating_from(line.wirelength) * ratio;
    let scale = |share: f64| i32::saturating_from((share * f64::from(length)).round());
    let mut segment = WireSegment {
        length: u8::wrapping_from(length),
        load: u8::wrapping_from(u32::saturating_from(line.load)),
        output_slew: u8::wrapping_from(u32::saturating_from(pin_slew)),
        delay: u32::saturating_from(line.pin_arrival),
        input_cap: u8::wrapping_from(u32::saturating_from(line.total_cap)),
        input_slew: u8::wrapping_from(u32::saturating_from(line.in_slew)),
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
        return segment;
    }
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
    segment
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
mod tests;
