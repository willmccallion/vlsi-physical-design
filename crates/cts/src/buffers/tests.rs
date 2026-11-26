use super::*;
use pare_common::db::indices::LayerId;
use pare_common::db::parser::liberty;
use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
use std::fmt::Write as _;
use std::path::Path;

/// A library of buffers named `B<limit>`, each with that load limit
/// (none for a limit of zero) and, where given, a fanout limit.
fn buffers(limits: &[(u32, Option<u32>)], default_fanout: Option<u32>) -> Liberty {
    let mut src = String::from("library (x) {\n");
    if let Some(f) = default_fanout {
        writeln!(src, "  default_max_fanout : {f};").unwrap();
    }
    for &(cap, fanout) in limits {
        let fanout = fanout.map_or_else(String::new, |f| format!("max_fanout : {f};"));
        let load = if cap == 0 {
            String::new()
        } else {
            format!("max_capacitance : {cap};")
        };
        writeln!(
            src,
            "  cell (B{cap}) {{ pin (A) {{ direction : input; }} \
                 pin (Z) {{ direction : output; function : \"A\"; \
                 {load} {fanout} }} }}"
        )
        .unwrap();
    }
    src.push('}');
    liberty::parse_str(&src, Path::new("x.lib")).unwrap()
}

fn id(lib: &Liberty, cap: u32) -> BufferCell {
    lib.buffer_cell(lib.cell_id(&format!("B{cap}")).unwrap())
        .unwrap()
}

#[test]
fn buffers_between_the_sink_and_root_limits_are_kept_by_limit_less_close_neighbours() {
    let caps = [60, 10, 30, 32, 5, 70];
    let lib = buffers(&caps.map(|c| (c, None)), None);
    let all: Vec<BufferCell> = caps.iter().map(|&c| id(&lib, c)).collect();

    let kept = trim_and_sort(&lib, &all, id(&lib, 60), id(&lib, 10));

    assert_eq!(kept, vec![id(&lib, 10), id(&lib, 30), id(&lib, 60)]);
}

#[test]
fn no_buffer_within_the_limits_leaves_nothing_to_characterize() {
    let lib = buffers(&[(10, None), (20, None), (40, None)], None);

    let kept = trim_and_sort(&lib, &[id(&lib, 40)], id(&lib, 20), id(&lib, 10));

    assert_eq!(kept, Vec::new());
}

#[test]
fn a_buffer_without_a_load_limit_is_characterized_as_limit_zero() {
    let lib = buffers(&[(0, None), (10, None), (20, None)], None);
    let all = [id(&lib, 20), id(&lib, 10), id(&lib, 0)];

    let kept = trim_and_sort(&lib, &all, id(&lib, 10), id(&lib, 0));

    assert_eq!(kept, vec![id(&lib, 0), id(&lib, 10)]);
}

#[test]
fn a_limit_covering_the_load_only_in_double_precision_covers_it() {
    // A limit of 1 pF derated lies above the load in double precision;
    // in single precision it rounds to the load itself.
    let lib = buffers(&[(1, None), (100, None)], None);
    let load = f32::nearest_from(1e-12_f64) * 0.01_f32;

    let best = best_max_cap_buffer(&lib, &[id(&lib, 1), id(&lib, 100)], load);

    assert_eq!(best, Some(id(&lib, 1)));
}

#[test]
fn a_wires_capacitance_is_rounded_to_single_precision_once() {
    let rc = WireRc {
        resistance: OhmsPerMeter::from_ohms_per_micron(1.0),
        capacitance: FaradsPerMeter::from_femtofarads_per_micron(0.07),
        layer: LayerId::new(0),
    };
    let per_micron = rc.capacitance.per_micron();

    let cap = signal_wire_cap(&rc, 3.0);

    assert_eq!(cap.to_bits(), f32::nearest_from(per_micron * 3.0).to_bits());
    assert_ne!(
        cap.to_bits(),
        (f32::nearest_from(per_micron) * 3.0).to_bits()
    );
}

#[test]
fn buffers_without_load_limits_are_all_characterized() {
    let lib = liberty::parse_str(
        "library (x) {
               cell (N1) { pin (A) { direction : input; }
                 pin (Z) { direction : output; function : \"A\"; } }
               cell (N2) { pin (A) { direction : input; }
                 pin (Z) { direction : output; function : \"A\"; } }
             }",
        Path::new("x.lib"),
    )
    .unwrap();
    let named = |name: &str| lib.buffer_cell(lib.cell_id(name).unwrap()).unwrap();
    let all = [named("N1"), named("N2")];

    let kept = trim_and_sort(&lib, &all, all[0], all[1]);

    assert_eq!(kept, all);
}

#[test]
fn a_fractional_design_fanout_limit_is_truncated() {
    let lib = buffers(&[(10, None)], None);

    assert_eq!(fanout_limit(&lib, id(&lib, 10), Some(7.9)), Some(7));
}

#[test]
fn a_fanout_limit_below_one_is_no_limit() {
    let lib = buffers(&[(10, Some(8))], None);

    assert_eq!(fanout_limit(&lib, id(&lib, 10), Some(0.5)), None);
}

#[test]
fn the_fanout_limit_is_the_lower_of_the_design_and_the_buffer() {
    let lib = buffers(&[(10, Some(8))], Some(20));

    assert_eq!(fanout_limit(&lib, id(&lib, 10), Some(12.0)), Some(8));
    assert_eq!(fanout_limit(&lib, id(&lib, 10), Some(5.0)), Some(5));
}

#[test]
fn a_buffer_without_its_own_fanout_limit_takes_the_library_default() {
    let lib = buffers(&[(10, None)], Some(20));

    assert_eq!(fanout_limit(&lib, id(&lib, 10), None), Some(20));
}

#[test]
fn without_any_fanout_limit_there_is_none() {
    let lib = buffers(&[(10, None)], None);

    assert_eq!(fanout_limit(&lib, id(&lib, 10), None), None);
}
