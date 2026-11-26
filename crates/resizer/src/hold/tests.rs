use super::*;
use crate::testing::{
    DefSections, cells, clock_cells_source, def_text, design, pin, with_timed_session,
    with_wired_session,
};
use std::fmt::Write;

const CLOCK: &str = "create_clock -name clk -period 1.0 [get_ports clk]\n";

/// Two flops side by side on one clock, the first driving the second.
fn pair() -> pare_common::db::design::NetlistDB {
    design(&def_text(
        "pair",
        &DefSections {
            components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 12000 2800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( f2 D ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// What hold repair of [`pair`] to `hold_margin` reports, with the hold
/// slack at the second flop's input before and after it.
fn repaired_pair(hold_margin: Seconds) -> (HoldSummary, Seconds, Seconds) {
    let mut db = pair();
    let lib = cells(&clock_cells_source());
    let mut repaired = None;
    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        let end = pin(resizer.db, "f2/D");
        let before = resizer.sta().slack(end, MinMax::Min).time().unwrap();
        let options = HoldOptions {
            hold_margin,
            ..HoldOptions::default()
        };

        let summary = repair_hold(resizer, &options).unwrap();

        let after = resizer.sta().slack(end, MinMax::Min).time().unwrap();
        repaired = Some((summary, before, after));
    });
    repaired.unwrap()
}

#[test]
fn the_buffer_goes_halfway_between_the_pins_rounded_down() {
    let at = |x: i64, y: i64| Point::new(Dbu::new(x), Dbu::new(y));

    assert_eq!(midpoint(at(0, 10), at(5, 20)), at(2, 15));
    assert_eq!(midpoint(at(-5, -20), at(0, -1)), at(-3, -11));
}

#[test]
fn a_path_short_of_the_hold_margin_gets_hold_buffers_until_it_meets_it() {
    let margin = Seconds::new(0.12e-9);

    let (summary, before, after) = repaired_pair(margin);

    assert!(before < margin);
    assert!(summary.inserted > 0, "{summary:?}");
    assert!(after >= margin, "{before:?} to {after:?}");
    assert_eq!((summary.failing_before, summary.failing_after), (1, 0));
}

#[test]
fn a_path_a_hair_short_of_the_hold_margin_is_repaired() {
    let (_, before, _) = repaired_pair(Seconds::ZERO);
    let margin = before * (1.0 + 1e-7);

    let (summary, _, _) = repaired_pair(margin);

    assert!(summary.inserted > 0, "{summary:?}");
}

#[test]
fn the_buffer_budget_counts_only_the_cells_still_in_the_design() {
    let mut components = String::new();
    for i in 0..150 {
        writeln!(components, "- b{i} BUF_X1 + PLACED ( 2000 2800 ) N ;").unwrap();
    }
    let sections = DefSections {
        components: &components,
        nets: "",
        inputs: &[],
        outputs: &[],
    };
    let mut db = design(&def_text("many", &sections));
    let all = Share::new(1.0).unwrap();
    assert_eq!(buffer_budget(&db, all), 150);

    for i in 0..100 {
        let cell = db.cell_id(&format!("b{i}")).unwrap();
        let _ = db.retire_cell(cell);
    }

    assert_eq!(buffer_budget(&db, all), MIN_BUFFER_BUDGET);
}

#[test]
fn a_path_that_meets_hold_gets_no_buffer() {
    let (summary, before, after) = repaired_pair(Seconds::ZERO);

    assert!(before > Seconds::ZERO);
    assert_eq!(summary.inserted, 0);
    assert_eq!(after, before);
}

/// Two flops 300 microns apart on one clock, the first driving the
/// second.
fn apart() -> pare_common::db::design::NetlistDB {
    design(&def_text(
        "apart",
        &DefSections {
            components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 602000 2800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( f2 D ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// The slews at the first flop's output and the second's input of
/// [`apart`] on wires, and whether hold repair would leave the first at
/// its size under `constraints`.
fn hand_under(constraints: &str) -> (Seconds, Seconds, bool) {
    let mut db = apart();
    let lib = cells(&clock_cells_source());
    let mut found = (Seconds::ZERO, Seconds::ZERO, false);
    with_wired_session(&mut db, &lib, constraints, |resizer| {
        let (driver, load) = (pin(resizer.db, "f1/Q"), pin(resizer.db, "f2/D"));
        found = (
            resizer.graph().slew(driver).max(),
            resizer.graph().slew(load).max(),
            HoldRepair::load_in_hand(resizer, driver),
        );
    });
    found
}

#[test]
fn a_load_near_a_design_slew_limit_is_not_in_hand() {
    let (driver_slew, load_slew, in_hand) = hand_under(CLOCK);
    assert!(in_hand && driver_slew < load_slew);
    let limit = (driver_slew + load_slew).get() / 2.0 / (1.0 - SLACK_LIMIT_RATIO) * 1e9;
    let tight = format!("{CLOCK}set_max_transition {limit} [current_design]\n");

    let (_, _, in_hand) = hand_under(&tight);

    assert!(!in_hand);
}

#[test]
fn a_clock_buffer_with_the_most_hold_delay_per_area_is_the_hold_buffer() {
    let mut db = pair();
    let slow_clock_buffer = clock_cells_source().replace(
        "values (\"0.02, 0.08\", \"0.03, 0.09\")",
        "values (\"0.5, 0.6\", \"0.5, 0.6\")",
    );
    let lib = cells(&slow_clock_buffer);

    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        let chosen = hold_buffer(resizer).map(|b| resizer.link.liberty.cell(b.id).name.clone());

        assert_eq!(chosen.as_deref(), Some("CLKBUF_X3"));
    });
}
