use super::*;
use crate::fixtures;
use pare_common::db::liberty::LibCellId;
use pare_common::db::parser::liberty;
use pare_common::testing::{SeededRng, assert_golden};
use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
use std::path::{Path, PathBuf};

/// The Liberty text of a library of the one clock buffer `CLKBUF_X3`.
#[derive(Clone, Copy, Default)]
struct BufferText<'a> {
    /// Attributes at library level.
    defaults: &'a str,
    /// Limits on the output pin.
    limits: &'a str,
    /// The tables of the arc from the input to the output.
    timing: &'a str,
    /// Timing groups of the input pin.
    input_timing: &'a str,
}

impl BufferText<'_> {
    fn library(self) -> Liberty {
        let Self {
            defaults,
            limits,
            timing,
            input_timing,
        } = self;
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
                 pin (A) {{ direction : input ; capacitance : 1.4 ; {input_timing} }}
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
}

/// Both of the buffer's limits.
const LIMITS: &str = "max_capacitance : 180.0 ; max_transition : 0.2 ;";

/// Delay tables over input slew and load.
const TWO_AXIS_TIMING: &str = "cell_rise (delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }
         cell_fall (delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }";

/// The buffer with both limits and delay tables over input slew and load.
fn two_axis_buffer() -> Liberty {
    BufferText {
        limits: LIMITS,
        timing: TWO_AXIS_TIMING,
        ..BufferText::default()
    }
    .library()
}

/// The library of the clock tree synthesis tests.
fn clock_cells() -> Liberty {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/clock.lib");
    liberty::parse_file(&path).unwrap()
}

/// What `step` makes of characterizing the buffers `masters` of `lib` in
/// `db`, the first also the sink buffer.
fn characterized<T>(
    (db, lib): (&NetlistDB, &Liberty),
    masters: &[&str],
    step: impl FnOnce(&CharSetup<'_>) -> T,
) -> T {
    let link = Link::new(db, lib, &[]).unwrap();
    let masters: Vec<BufferCell> = masters
        .iter()
        .map(|name| lib.buffer_cell(lib.cell_id(name).unwrap()).unwrap())
        .collect();
    let layer = db.library.routing_layer_id("metal3").unwrap();
    step(&CharSetup {
        link: &link,
        db,
        masters: &masters,
        sink_buffer: masters[0],
        clock_rc: WireRc {
            resistance: OhmsPerMeter::from_ohms_per_micron(1.5),
            capacitance: FaradsPerMeter::from_femtofarads_per_micron(0.11),
            layer,
        },
    })
}

/// What `plan` makes of `lib` characterizing its `CLKBUF_X3` in `db`.
fn plan_of(db: &NetlistDB, lib: &Liberty) -> Result<Plan, CtsError> {
    characterized((db, lib), &["CLKBUF_X3"], plan)
}

#[test]
fn no_measurement_within_the_slew_limit_is_refused_naming_the_buffer() {
    let lib = two_axis_buffer();

    let refused = characterized((&fixtures::design(""), &lib), &["CLKBUF_X3"], |setup| {
        let plan = plan(setup).unwrap();
        tabulate(setup, &plan, &[]).err()
    });

    assert_eq!(
        refused,
        Some(CtsError::NoSegmentWithinSlewLimit {
            buffer: "CLKBUF_X3".to_string(),
            max_transition: Seconds::new(f64::from(0.2e-9_f32)),
        })
    );
}

#[test]
fn a_characterization_buffer_of_no_height_is_refused() {
    let mut db = fixtures::design("");
    for master in &mut db.library.macros {
        master.size.y = pare_common::geom::dbu::Dbu::ZERO;
    }
    let lib = two_axis_buffer();

    let refused = plan_of(&db, &lib).err();

    assert_eq!(
        refused,
        Some(CtsError::ZeroHeightBuffer("CLKBUF_X3".to_string()))
    );
}

#[test]
fn a_planned_characterization_steps_through_the_buffers_axes() {
    let lib = two_axis_buffer();

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
    let lib = BufferText {
        limits: "max_capacitance : 180.0 ;",
        timing: TWO_AXIS_TIMING,
        ..BufferText::default()
    }
    .library();

    let refused = plan_of(&fixtures::design(""), &lib).err();

    assert_eq!(refused, Some(CtsError::NoMaxSlew("CLKBUF_X3".to_string())));
}

#[test]
fn a_buffer_without_a_load_limit_is_refused() {
    let lib = BufferText {
        defaults: "default_max_transition : 0.2 ;",
        timing: TWO_AXIS_TIMING,
        ..BufferText::default()
    }
    .library();

    let refused = plan_of(&fixtures::design(""), &lib).err();

    assert_eq!(
        refused,
        Some(CtsError::NoMaxCapacitance("CLKBUF_X3".to_string()))
    );
}

#[test]
fn delay_tables_with_a_load_axis_but_no_slew_axis_are_refused() {
    let lib = BufferText {
        limits: LIMITS,
        timing: "cell_rise (load_t) { values (\"0.02, 0.08\") ; }
             cell_fall (load_t) { values (\"0.02, 0.08\") ; }",
        ..BufferText::default()
    }
    .library();

    let refused = plan_of(&fixtures::design(""), &lib).err();

    assert_eq!(
        refused,
        Some(CtsError::NoTableAxes("CLKBUF_X3".to_string()))
    );
}

/// A segment of `length` units with `delay` steps, `input_slew` and
/// `output_slew` steps, buffered or bare.
fn segment(length: u8, delay: u32, input_slew: u8, output_slew: u8, buffered: bool) -> WireSegment {
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
    let mut bounds = None;

    let normalized = [2.5, 0.0, 4.0].map(|v| normalize(&mut bounds, v, 1.0));

    assert_eq!(normalized, [3, 1, 4]);
    assert_eq!(bounds, Some(Bounds { min: 1, max: 4 }));
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
    let descriptors: Vec<Vec<Piece>> = patterns(20, 10).into_iter().map(|p| p.descriptor).collect();

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

#[test]
fn a_buffer_without_table_axes_is_characterized_in_even_steps_to_its_limits() {
    let lib = BufferText {
        limits: LIMITS,
        timing: "cell_rise (scalar) { values (\"0.02\") ; }
             cell_fall (scalar) { values (\"0.02\") ; }",
        ..BufferText::default()
    }
    .library();

    let plan = plan_of(&fixtures::design(""), &lib).unwrap();

    let (slew_step, cap_step) = (0.2e-9_f32 / 7.0, 180e-15_f32 / 20.0);
    let slews: Vec<f32> = (1..=7_u8).map(|s| f32::from(s) * slew_step).collect();
    let loads: Vec<f32> = (1..=20_u8).map(|s| f32::from(s) * cap_step).collect();
    assert_eq!((plan.slews, plan.loads), (slews, loads));
}

#[test]
fn only_the_arcs_from_the_input_to_the_output_give_the_axes() {
    let with_self_arc = BufferText {
        limits: LIMITS,
        timing: TWO_AXIS_TIMING,
        input_timing: "timing () { related_pin : \"A\" ;
             cell_rise (load_t) { values (\"0.5, 0.9\") ; }
             cell_fall (load_t) { values (\"0.5, 0.9\") ; } }",
        ..BufferText::default()
    }
    .library();
    let design = fixtures::design("");

    let plan = plan_of(&design, &with_self_arc).unwrap();

    let without = plan_of(&design, &two_axis_buffer()).unwrap();
    assert_eq!((plan.slews, plan.loads), (without.slews, without.loads));
}

#[test]
fn a_value_exactly_a_percent_from_the_last_kept_stays() {
    let mut values = vec![100.0, 99.0];

    sort_and_uniquify(&mut values);

    assert_eq!(values, [99.0, 100.0]);
}

#[test]
fn of_axis_values_equally_close_to_their_predecessors_the_first_goes() {
    let mut values = vec![1.0, 2.0, 10.0, 11.0];

    reduce_or_expand(&mut values, 3);

    assert_eq!(values, [1.0, 10.0, 11.0]);
}

/// A key of the given fields.
const fn char_key(load: f32, wirelength: f32, pin_slew: f32, total_cap: f32) -> CharKey {
    CharKey {
        load,
        wirelength,
        pin_slew,
        total_cap,
    }
}

#[test]
fn measurement_keys_order_by_load_then_length_pin_slew_and_input_cap() {
    let mut keys = [
        char_key(2.0, 1.0, 1.0, 1.0),
        char_key(1.0, 2.0, 1.0, 1.0),
        char_key(1.0, 1.0, 2.0, 1.0),
        char_key(1.0, 1.0, 1.0, 2.0),
        char_key(1.0, 1.0, 1.0, 1.0),
    ];

    keys.sort();

    let fields: Vec<[f32; 4]> = keys.iter().map(CharKey::fields).collect();
    assert_eq!(
        fields,
        [
            [1.0, 1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0, 2.0],
            [1.0, 1.0, 2.0, 1.0],
            [1.0, 2.0, 1.0, 1.0],
            [2.0, 1.0, 1.0, 1.0],
        ]
    );
}

#[test]
fn measurement_keys_are_equal_and_compare_as_they_order() {
    let (low, high) = (char_key(1.0, 1.0, 1.0, 1.0), char_key(1.0, 1.0, 1.0, 2.0));

    assert_eq!(low, char_key(1.0, 1.0, 1.0, 1.0));
    assert_ne!(low, high);
    assert_eq!(low.partial_cmp(&high), Some(Ordering::Less));
}

#[test]
fn a_segment_reads_its_steps_from_its_fields() {
    let s = WireSegment {
        length: 2,
        load: 3,
        output_slew: 4,
        delay: 0,
        input_cap: 5,
        input_slew: 6,
        buffer_locations: Vec::new(),
        buffer_masters: Vec::new(),
        wl_to_first_buffer: 7,
        last_wl: 8,
    };

    let steps = (
        (s.to_first_buffer(), s.after_last_buffer()),
        (s.load_steps(), s.input_cap_steps()),
        (s.output_slew_steps(), s.input_slew_steps()),
    );

    assert_eq!(
        steps,
        (
            (LengthSteps(7), LengthSteps(8)),
            (CapSteps(3), CapSteps(5)),
            (SlewSteps(4), SlewSteps(6)),
        )
    );
}

/// A segment `length` units long with `load` and `output_slew` steps,
/// told apart by its `delay`.
fn keyed_segment(length: u8, load: u8, output_slew: u8, delay: u32) -> WireSegment {
    WireSegment {
        load,
        ..segment(length, delay, 1, output_slew, false)
    }
}

/// The delays of the segments of a table with a key.
fn delays_with(table: &TechChar, length: i64, load: u32, output_slew: u32) -> Vec<u32> {
    table
        .segments_with(LengthSteps(length), CapSteps(load), SlewSteps(output_slew))
        .iter()
        .map(|&i| table.segment(i).delay)
        .collect()
}

#[test]
fn every_length_load_and_slew_of_a_table_files_its_own_segments() {
    let mut segments = Vec::new();
    for length in 1..=8 {
        for load in 1..=20 {
            for slew in 1..=7 {
                let delay = u32::from(length) * 1000 + u32::from(load) * 10 + u32::from(slew);
                segments.push(keyed_segment(length, load, slew, delay));
            }
        }
    }
    let table = TechChar::from_segments(segments, 1);

    for length in 1..=8 {
        for load in 1..=20 {
            for slew in 1..=7 {
                let delay = u32::try_from(length).unwrap() * 1000 + load * 10 + slew;
                assert_eq!(delays_with(&table, length, load, slew), [delay]);
            }
        }
    }
}

#[test]
fn a_lookup_past_eight_bits_wraps_as_tritoncts_does() {
    let table = TechChar::from_segments(vec![keyed_segment(2, 3, 4, 5)], 1);

    assert_eq!(delays_with(&table, 258, 259, 260), [5]);
}

#[test]
fn fake_entries_copy_every_segment_of_a_length_as_another() {
    let mut table = TechChar::from_segments(
        vec![
            keyed_segment(2, 1, 1, 5),
            keyed_segment(2, 1, 2, 6),
            keyed_segment(3, 1, 1, 7),
        ],
        1,
    );

    table.create_fake_entries(LengthSteps(2), LengthSteps(1));

    let fakes = [1, 2].map(|slew| delays_with(&table, 1, 1, slew));
    assert_eq!(fakes, [vec![5], vec![6]]);
    assert_eq!(table.segments.len(), 5);
}

#[test]
fn fake_entries_of_a_length_as_itself_add_nothing() {
    let mut table = TechChar::from_segments(vec![keyed_segment(2, 1, 1, 5)], 1);

    table.create_fake_entries(LengthSteps(2), LengthSteps(2));

    assert_eq!(delays_with(&table, 2, 1, 1), [5]);
}

#[test]
fn buffered_segments_of_no_length_give_no_rate() {
    let segments = [segment(0, 10, 3, 3, true)];

    assert_eq!(least_wire_delay_rate(&segments, STEP), None);
}

/// A plan over the clock buffer's limits, 0.2 ns in seven steps and
/// 180 fF in twenty, the characterization unit twice the length unit.
fn limits_plan() -> Plan {
    let (max_slew, max_cap) = (0.2e-9_f32, 180e-15_f32);
    Plan {
        char_buffer: 1,
        char_unit: 28_000,
        length_unit: 14_000,
        wirelengths: vec![28_000, 56_000, 84_000, 112_000],
        max_slew,
        slew_step: max_slew / 7.0,
        cap_step: max_cap / 20.0,
        slews: Vec::new(),
        loads: Vec::new(),
        sink_buffer_input_cap: Farads32::ZERO,
    }
}

/// Two buffers to name in topologies.
fn two_masters() -> [BufferCell; 2] {
    [0, 1].map(|id| BufferCell {
        id: LibCellId(id),
        input: LibPortId::new(0),
        output: LibPortId::new(1),
    })
}

/// Normalized measurements spanning loads up to `max_load` steps.
fn spanning_loads(max_load: u32) -> Normalization {
    let one = Bounds { min: 1, max: 1 };
    Normalization {
        lines: Vec::new(),
        bounds: AllBounds {
            slew: one,
            cap: Bounds {
                min: 1,
                max: max_load,
            },
            wirelength: one,
        },
    }
}

#[test]
fn loads_within_ten_bits_compile() {
    let compiled = compile(
        &spanning_loads(1023),
        &limits_plan(),
        &two_masters(),
        FaradsPerDbu::new(0.0),
    );

    assert!(compiled.is_ok());
}

#[test]
fn loads_past_ten_bits_are_refused() {
    let compiled = compile(
        &spanning_loads(1024),
        &limits_plan(),
        &two_masters(),
        FaradsPerDbu::new(0.0),
    );

    assert_eq!(compiled.err(), Some(CtsError::CharacterizationRange(1023)));
}

/// A number of steps below `n`, as a float.
fn steps_below(rng: &mut SeededRng, n: u8) -> f32 {
    f32::from(u8::try_from(rng.below(u64::from(n))).unwrap())
}

/// A seeded measurement over `plan`: a random pattern with random buffer
/// types, its quantities on whole steps, so they tie, or between them.
fn random_measurement(rng: &mut SeededRng, plan: &Plan) -> Measured {
    let wirelength = rng.pick(&plan.wirelengths);
    let patterns = patterns(wirelength, plan.char_unit);
    let pattern = &patterns[usize::try_from(rng.below(patterns.len() as u64)).unwrap()];
    let topology = pattern
        .descriptor
        .iter()
        .map(|&piece| match piece {
            Piece::Buffer(_) => Piece::Buffer(rng.pick(&[0, 1])),
            Piece::Wire(_) => piece,
        })
        .collect();
    let jitter = rng.pick(&[0.0, 0.5]);
    Measured {
        load: (1.0 + steps_below(rng, 20)) * plan.cap_step,
        in_slew: (1.0 + steps_below(rng, 7)) * plan.slew_step,
        wirelength: wirelength as f32,
        pin_slew: (steps_below(rng, 9) + jitter) * plan.slew_step,
        pin_arrival: steps_below(rng, 200) * plan.slew_step / 23.0,
        total_cap: (steps_below(rng, 25) + jitter) * plan.cap_step,
        pure_wire: pattern.is_pure_wire(),
        topology,
    }
}

/// What normalizing and compiling `measurements` over `plan` gives.
fn table_lines(measurements: &[Measured], plan: &Plan) -> Vec<String> {
    let Some(normalization) = post_process(measurements, plan) else {
        return vec!["  none within the slew limit".to_string()];
    };
    let table = compile(&normalization, plan, &two_masters(), FaradsPerDbu::new(0.0)).unwrap();
    let mut lines = vec![format!(
        "  lengths {:?} to {:?}, max load {:?}, max slew {:?}, least input cap {:?}, rate {:?}",
        table.min_segment_length(),
        table.max_segment_length(),
        table.max_capacitance(),
        table.max_slew(),
        table.actual_min_input_cap(),
        table.wire_delay_rate(),
    )];
    lines.extend(table.segments.iter().map(|s| {
        let masters: Vec<LibCellId> = s.buffer_masters.iter().map(|b| b.id).collect();
        format!(
            "  length {} load {} slew {} delay {} input cap {} slew {} buffers at {:?} {masters:?} first {} last {}",
            s.length,
            s.load,
            s.output_slew,
            s.delay,
            s.input_cap,
            s.input_slew,
            s.buffer_locations,
            s.wl_to_first_buffer,
            s.last_wl,
        )
    }));
    lines
}

/// The tables seeded sets of measurements normalize and compile to.
fn normalization_text() -> String {
    let plan = limits_plan();
    let mut rng = SeededRng::new(37);
    let mut lines = Vec::new();
    for case in 0..40 {
        let count = rng.below(12);
        let measurements: Vec<Measured> = (0..count)
            .map(|_| random_measurement(&mut rng, &plan))
            .collect();
        lines.push(format!("case {case}: {count} measurements"));
        lines.extend(table_lines(&measurements, &plan));
    }
    lines.join("\n") + "\n"
}

/// The first, a middle and the last of the plan's loads and input slews.
fn timing_points(plan: &Plan) -> [(f32, f32); 3] {
    let (loads, slews) = (&plan.loads, &plan.slews);
    [
        (loads[0], slews[0]),
        (loads[loads.len() / 2], slews[slews.len() / 2]),
        (loads[loads.len() - 1], slews[slews.len() - 1]),
    ]
}

/// Both edges through `chain` at a load and input slew, and what its
/// measurement makes of them.
fn chain_timing(
    chain: &Chain<'_>,
    plan: &Plan,
    (load, in_slew): (f32, f32),
    wirelength: u32,
) -> String {
    let rise = chain.propagate(Edge::Rise, load, in_slew).unwrap();
    let fall = chain.propagate(Edge::Fall, load, in_slew).unwrap();
    let measured = chain
        .measure(plan, load, in_slew, wirelength, &chain.topology())
        .unwrap();
    format!(
        "  load {load:e} slew {in_slew:e}: rise {rise:?} fall {fall:?}, pin slew {:e} arrival {:e} input cap {:e}",
        measured.pin_slew, measured.pin_arrival, measured.total_cap
    )
}

/// Every chain of the clock cells' `BUF_X1` and `CLKBUF_X3` timed at a few
/// of the characterization's loads and slews.
fn chain_timing_text() -> String {
    let (design, lib) = (fixtures::design(""), clock_cells());
    characterized((&design, &lib), &["BUF_X1", "CLKBUF_X3"], |setup| {
        let plan = plan(setup).unwrap();
        let mut lines = Vec::new();
        for_each_chain(setup, &plan, |chain, wirelength| {
            lines.push(format!("{wirelength} {:?}", chain.topology()));
            for point in timing_points(&plan) {
                lines.push(chain_timing(chain, &plan, point, wirelength));
            }
            Ok(())
        })
        .unwrap();
        lines.join("\n") + "\n"
    })
}

fn golden_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}

#[test]
fn the_chains_time_as_they_always_have() {
    assert_golden(
        &golden_dir(),
        "techchar_chain_timing.txt",
        &chain_timing_text(),
    );
}

#[test]
fn measurements_normalize_and_compile_as_they_always_have() {
    assert_golden(
        &golden_dir(),
        "techchar_normalization.txt",
        &normalization_text(),
    );
}

/// The arrival and slew a chain leaves its output with.
type Timing = (f32, f32);

/// The rising and the falling timing through every chain of `CLKBUF_X3`
/// with its rising tables and the falling ones `fall`, at a few of the
/// plan's loads and slews.
fn edges_with_falling_tables(fall: &str) -> (Vec<Timing>, Vec<Timing>) {
    let lib = BufferText {
        limits: LIMITS,
        timing: &format!(
            "cell_rise (delay_t) {{ values (\"0.02, 0.08\", \"0.03, 0.09\") ; }}
             rise_transition (delay_t) {{ values (\"0.006, 0.12\", \"0.02, 0.13\") ; }}
             cell_fall {fall} fall_transition {fall}"
        ),
        ..BufferText::default()
    }
    .library();
    characterized((&fixtures::design(""), &lib), &["CLKBUF_X3"], |setup| {
        let plan = plan(setup).unwrap();
        let (mut rises, mut falls) = (Vec::new(), Vec::new());
        for_each_chain(setup, &plan, |chain, _| {
            for (load, slew) in timing_points(&plan) {
                rises.push(chain.propagate(Edge::Rise, load, slew)?);
                falls.push(chain.propagate(Edge::Fall, load, slew)?);
            }
            Ok(())
        })
        .unwrap();
        (rises, falls)
    })
}

#[test]
fn each_edge_is_timed_by_its_own_tables() {
    let quick = "(delay_t) { values (\"0.01, 0.04\", \"0.02, 0.05\") ; }";
    let slow = "(delay_t) { values (\"0.1, 0.4\", \"0.2, 0.5\") ; }";

    let (quick_rises, quick_falls) = edges_with_falling_tables(quick);
    let (slow_rises, slow_falls) = edges_with_falling_tables(slow);

    assert_eq!(quick_rises, slow_rises);
    assert_ne!(quick_falls, slow_falls);
}
