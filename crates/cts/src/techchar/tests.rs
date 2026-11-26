use super::*;
use crate::fixtures;
use pare_common::db::liberty::LibCellId;
use pare_common::db::parser::liberty;
use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
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
const TWO_AXIS_TIMING: &str = "cell_rise (delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }
         cell_fall (delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }";

/// Delay and slew tables over input slew and load.
/// What `step` makes of characterizing the buffer of `lib` in `db`.
fn characterized<T>(db: &NetlistDB, lib: &Liberty, step: impl FnOnce(&CharSetup<'_>) -> T) -> T {
    let link = Link::new(db, lib, &[]).unwrap();
    let id = lib.cell_id("CLKBUF_X3").unwrap();
    let buffer = lib.buffer_cell(id).unwrap();
    let layer = db.library.routing_layer_id("metal3").unwrap();
    step(&CharSetup {
        link: &link,
        db,
        masters: &[buffer],
        sink_buffer: buffer,
        clock_rc: WireRc {
            resistance: OhmsPerMeter::from_ohms_per_micron(1.5),
            capacitance: FaradsPerMeter::from_femtofarads_per_micron(0.11),
            layer,
        },
    })
}

/// What `plan` makes of `lib` characterizing its buffer in `db`.
fn plan_of(db: &NetlistDB, lib: &Liberty) -> Result<Plan, CtsError> {
    characterized(db, lib, plan)
}

#[test]
fn no_measurement_within_the_slew_limit_is_refused_naming_the_buffer() {
    let lib = clock_buffer_library(
        "",
        "max_capacitance : 180.0 ; max_transition : 0.2 ;",
        TWO_AXIS_TIMING,
    );

    let refused = characterized(&fixtures::design(""), &lib, |setup| {
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
    let lib = clock_buffer_library(
        "",
        "max_capacitance : 180.0 ; max_transition : 0.2 ;",
        TWO_AXIS_TIMING,
    );

    let refused = plan_of(&db, &lib).err();

    assert_eq!(
        refused,
        Some(CtsError::ZeroHeightBuffer("CLKBUF_X3".to_string()))
    );
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
