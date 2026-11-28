use super::*;
use crate::testing::{
    DefSections, cells, clock_cells_source, def_text, design, pin, with_timed_session,
    with_wired_session,
};
use pare_common::db::liberty::Liberty;
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

/// `count` pairs of flops like [`pair`], on one clock, spread over three
/// columns, the first of each driving the second.
fn pairs(count: usize) -> pare_common::db::design::NetlistDB {
    let mut components = String::new();
    let mut nets = String::from("- clk ( PIN clk )");
    let mut data = String::new();
    for i in 0..count {
        let (x, y) = (2000 + 200_000 * (i % 3), 2800 + 14_000 * (i / 3));
        writeln!(components, "- a{i} DFF_X1 + PLACED ( {x} {y} ) N ;").unwrap();
        writeln!(
            components,
            "- b{i} DFF_X1 + PLACED ( {} {y} ) N ;",
            x + 10_000
        )
        .unwrap();
        write!(nets, " ( a{i} CK ) ( b{i} CK )").unwrap();
        writeln!(data, "- q{i} ( a{i} Q ) ( b{i} D ) ;").unwrap();
    }
    nets += " ;\n";
    nets += &data;
    design(&def_text(
        "pairs",
        &DefSections {
            components: &components,
            nets: &nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// What hold repair of `count` pairs reports under `options`.
fn repaired_pairs(count: usize, options: &HoldOptions) -> HoldSummary {
    let mut db = pairs(count);
    let mut summary = None;
    with_timed_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        summary = Some(repair_hold(resizer, options).unwrap());
    });
    summary.unwrap()
}

/// A hold margin above the slack of one of [`pairs`] that one buffer fixes.
const SHORT_OF_HOLD: Seconds = Seconds::new(0.08e-9);

#[test]
fn a_hold_violation_whose_fix_would_break_setup_is_left_alone() {
    let options = HoldOptions {
        hold_margin: SHORT_OF_HOLD,
        setup_margin: Seconds::new(10e-9),
        ..HoldOptions::default()
    };

    let summary = repaired_pairs(2, &options);

    assert_eq!(summary.inserted, 0);
    assert_eq!((summary.failing_before, summary.failing_after), (2, 2));
}

#[test]
fn each_failing_pair_needs_one_buffer_and_the_passes_limit_how_many_get_one() {
    let options = |max_passes| HoldOptions {
        hold_margin: SHORT_OF_HOLD,
        max_passes,
        ..HoldOptions::default()
    };

    let all = repaired_pairs(3, &options(10_000));
    let two = repaired_pairs(3, &options(2));
    let one = repaired_pairs(3, &options(1));

    assert_eq!((all.inserted, all.failing_after), (3, 0));
    assert_eq!((two.inserted, two.failing_after), (2, 1));
    assert_eq!((one.inserted, one.failing_after), (1, 2));
}

#[test]
fn repair_stops_after_the_buffer_that_passes_the_budget_of_a_hundred() {
    let options = HoldOptions {
        hold_margin: SHORT_OF_HOLD,
        ..HoldOptions::default()
    };

    let summary = repaired_pairs(150, &options);

    assert_eq!(summary.failing_before, 150);
    assert_eq!(summary.inserted, 101);
    assert_eq!(summary.failing_after, 49);
}

#[test]
fn a_path_whose_hold_slack_is_the_margin_exactly_does_not_fail() {
    let (_, before, _) = repaired_pair(Seconds::ZERO);

    let (summary, _, _) = repaired_pair(before);

    assert_eq!((summary.failing_before, summary.inserted), (0, 0));
}

#[test]
fn only_the_footprints_of_delay_cells_are_delay_cells() {
    assert!(is_delay_cell("sky130_fd_sc_hd__dlygate4sd1"));
    assert!(is_delay_cell("DLY_X2"));
    assert!(is_delay_cell("DEL_X1"));
    assert!(!is_delay_cell("BUF_X1"));
    assert!(!is_delay_cell(""));
}

/// A buffer cell of `name` with `attributes` before its pins, which
/// takes `delay_ns` whatever its load and slews.
fn slow_buffer(name: &str, area: f64, delay_ns: f64, attributes: &str) -> String {
    format!(
        "cell ({name}) {{ area : {area} ; {attributes}
          pin (A) {{ direction : input ; capacitance : 1.0 ; }}
          pin (Z) {{ direction : output ; function : \"A\" ; max_capacitance : 60.0 ;
            timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
              cell_rise (scalar) {{ values (\"{delay_ns}\") ; }}
              cell_fall (scalar) {{ values (\"{delay_ns}\") ; }}
              rise_transition (scalar) {{ values (\"0.01\") ; }}
              fall_transition (scalar) {{ values (\"0.01\") ; }} }} }} }}"
    )
}

/// An inverter as [`slow_buffer`] makes a buffer.
fn slow_inverter(name: &str, area: f64, delay_ns: f64) -> String {
    slow_buffer(name, area, delay_ns, "")
        .replace("pin (Z)", "pin (ZN)")
        .replace("function : \"A\"", "function : \"!A\"")
}

/// The name of the hold buffer of [`pair`] over the clock test cells and
/// `extra` cell groups, with a copy of the master `master` called `name`
/// for each of `copies` and the sites `edit` leaves.
fn chosen_hold_buffer(
    extra: &str,
    copies: &[(&str, &str)],
    edit: impl FnOnce(&mut pare_common::db::design::NetlistDB),
) -> String {
    let mut db = crate::testing::design_with_copies(
        &def_text(
            "pair",
            &DefSections {
                components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 12000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( f2 D ) ;\n",
                inputs: &["clk"],
                outputs: &[],
            },
        ),
        copies,
    );
    edit(&mut db);
    let mut name = String::new();
    with_timed_session(
        &mut db,
        &crate::testing::cells_with(extra),
        CLOCK,
        |resizer| {
            let chosen = hold_buffer(resizer).unwrap();
            name = resizer.link.liberty.cell(chosen.id).name.clone();
        },
    );
    name
}

#[test]
fn the_hold_buffer_has_the_most_delay_per_unit_of_area_not_the_most_delay() {
    let big = slow_buffer("BUF_BIG", 50.0, 0.5, "");

    let chosen = chosen_hold_buffer(&big, &[("BUF_X1", "BUF_BIG")], |_| {});

    assert_ne!(chosen, "BUF_BIG");
}

#[test]
fn a_smaller_buffer_within_five_percent_of_the_best_delay_per_area_is_chosen() {
    let cells = format!(
        "{}\n{}",
        slow_buffer("BUF_P", 1.0, 0.5, ""),
        slow_buffer("BUF_Q", 0.7, 0.34, "")
    );

    let chosen = chosen_hold_buffer(&cells, &[("BUF_X1", "BUF_P"), ("BUF_X1", "BUF_Q")], |_| {});

    assert_eq!(chosen, "BUF_Q");
}

#[test]
fn a_buffer_of_the_same_area_within_five_percent_does_not_displace_the_best() {
    let cells = format!(
        "{}\n{}",
        slow_buffer("BUF_R", 1.0, 0.6, ""),
        slow_buffer("BUF_S", 1.0, 0.58, "")
    );

    let chosen = chosen_hold_buffer(&cells, &[("BUF_X1", "BUF_R"), ("BUF_X1", "BUF_S")], |_| {});

    assert_eq!(chosen, "BUF_R");
}

#[test]
fn inverters_power_cells_unusable_cells_and_cells_without_a_master_are_no_hold_buffers() {
    let baseline = chosen_hold_buffer("", &[], |_| {});
    let traps = [
        (slow_inverter("TRAP", 0.5, 0.9), vec![("INV_X1", "TRAP")]),
        (
            slow_buffer("TRAP", 0.5, 0.9, "always_on : true ;"),
            vec![("BUF_X1", "TRAP")],
        ),
        (
            slow_buffer("TRAP", 0.5, 0.9, "dont_use : true ;"),
            vec![("BUF_X1", "TRAP")],
        ),
        (slow_buffer("TRAP", 0.5, 0.9, ""), vec![]),
    ];

    for (cell, copies) in traps {
        let chosen = chosen_hold_buffer(&cell, &copies, |_| {});

        assert_eq!(chosen, baseline, "{cell}");
    }
}

#[test]
fn delay_cells_alone_are_hold_buffers_where_the_library_has_any() {
    let delay_cell = slow_buffer("BUF_DLY", 5.0, 0.1, "cell_footprint : \"x_DLY_x\" ;");

    let chosen = chosen_hold_buffer(&delay_cell, &[("BUF_X1", "BUF_DLY")], |_| {});

    assert_eq!(chosen, "BUF_DLY");
}

#[test]
fn a_buffer_on_a_taller_site_than_the_shortest_is_no_hold_buffer() {
    let baseline = chosen_hold_buffer("", &[], |_| {});
    let tall = slow_buffer("BUF_TALL", 0.1, 0.9, "");

    let chosen = chosen_hold_buffer(&tall, &[("BUF_X1", "BUF_TALL")], |db| {
        let mut site = db.library.sites.iter().next().unwrap().clone();
        site.name = "tall".to_string();
        site.size.y = site.size.y * 2;
        let id = db.library.add_site(site).unwrap();
        let master = db.library.macro_id("BUF_TALL").unwrap();
        db.library.macros[master].site = Some(id);
    });

    assert_eq!(chosen, baseline);
}

/// A flop `f1` driving the data input of `f2` and the input of a buffer
/// `b` whose output goes nowhere, so that only `f2` can fail hold.
fn flop_with_a_dead_end() -> pare_common::db::design::NetlistDB {
    design(&def_text(
        "dead_end",
        &DefSections {
            components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                         - f2 DFF_X1 + PLACED ( 12000 2800 ) N ;\n\
                         - b BUF_X1 + PLACED ( 12000 30000 ) N ;\n",
            nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( f2 D ) ( b A ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// What a hold repair of [`flop_with_a_dead_end`] knows at `f1/Q`.
struct DriverSlacks {
    driver: PinId,
    load: PinId,
    /// The late and early slack of `load` and the delay of the hold buffer
    /// at the driver's load less `excluded`, per edge.
    late: EdgeValues<Seconds>,
    early: EdgeValues<Seconds>,
    delay_without: EdgeValues<Seconds>,
    delay_with: EdgeValues<Seconds>,
}

/// Reads [`DriverSlacks`] and runs `test` with the hold repair they are for.
fn on_dead_end(
    options: &HoldOptions,
    test: impl FnOnce(&Resizer<'_, Sta>, &HoldRepair<'_>, &DriverSlacks),
) {
    let mut db = flop_with_a_dead_end();
    with_timed_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let buffer = hold_buffer(resizer).unwrap();
        let (driver, load) = (pin(resizer.db, "f1/Q"), pin(resizer.db, "f2/D"));
        let dead_end = pin(resizer.db, "b/A");
        let excluded = resizer.netlist().lib_port(dead_end).unwrap().capacitance();
        let cap = resizer.load_cap(driver);
        let delays = |cap| buffer_delays(resizer.link.liberty, buffer, &resizer.targets.slews, cap);
        let sta = resizer.sta();
        let per_edge = |side| EdgeValues {
            rise: sta.slack_edge(load, Edge::Rise, side).time().unwrap(),
            fall: sta.slack_edge(load, Edge::Fall, side).time().unwrap(),
        };
        let slacks = DriverSlacks {
            driver,
            load,
            late: per_edge(MinMax::Max),
            early: per_edge(MinMax::Min),
            delay_without: delays(cap - excluded),
            delay_with: delays(cap + excluded),
        };
        let repair = HoldRepair {
            buffer,
            options,
            inserted: 0,
            resized: 0,
        };
        test(resizer, &repair, &slacks);
    });
}

#[test]
fn loads_that_do_not_fail_hold_are_left_out_of_the_load_the_hold_buffer_drives() {
    let options = HoldOptions {
        hold_margin: SHORT_OF_HOLD,
        ..HoldOptions::default()
    };
    on_dead_end(&options, |resizer, repair, found| {
        assert!(found.delay_with.rise > found.delay_without.rise);
        let setup = (found.delay_without.rise + found.delay_with.rise) / 2.0;
        let margin = found.late.rise - setup;
        let options = HoldOptions {
            setup_margin: margin,
            ..options.clone()
        };
        let repair = HoldRepair {
            options: &options,
            ..*repair
        };

        let plan = repair.plan(resizer, found.driver);

        assert!(found.late.fall - margin > found.delay_without.fall);
        assert_eq!(plan, Some(vec![found.load]));
    });
}

/// What planning for `f1/Q` of [`flop_with_a_dead_end`] gives under `options`
/// but for the setup margin `margin_of` makes of what the design reads.
fn plan_with_setup_margin(
    hold_margin: Seconds,
    margin_of: impl FnOnce(&DriverSlacks) -> Seconds,
) -> (Option<Vec<PinId>>, PinId) {
    let options = HoldOptions {
        hold_margin,
        ..HoldOptions::default()
    };
    let mut planned = None;
    on_dead_end(&options, |resizer, repair, found| {
        let options = HoldOptions {
            setup_margin: margin_of(found),
            ..options.clone()
        };
        let repair = HoldRepair {
            options: &options,
            ..*repair
        };
        planned = Some((repair.plan(resizer, found.driver), found.load));
    });
    planned.unwrap()
}

#[test]
fn a_setup_margin_comes_off_the_slack_that_pays_for_the_hold_buffer() {
    let roomy = |_: &DriverSlacks| Seconds::new(-10e-9);
    let tight = |_: &DriverSlacks| Seconds::new(10e-9);

    let (with_room, load) = plan_with_setup_margin(SHORT_OF_HOLD, roomy);
    let (without, _) = plan_with_setup_margin(SHORT_OF_HOLD, tight);

    assert_eq!(with_room, Some(vec![load]));
    assert_eq!(without, None);
}

#[test]
fn a_buffer_is_planned_only_where_the_setup_slack_exceeds_the_hold_shortfall() {
    let hold_margin = Seconds::new(1e-9);
    let (plan, _) = plan_with_setup_margin(hold_margin, |found| {
        let setup = Seconds::new(0.5e-9);
        for edge in Edge::BOTH {
            let shortfall = hold_margin - found.early.get(edge);
            let room = found.late.get(edge) - (found.late.rise - setup);
            assert!(room > found.delay_with.get(edge) && room < shortfall);
        }
        found.late.rise - setup
    });

    assert_eq!(plan, None);
}

#[test]
fn a_buffer_is_planned_only_where_the_setup_slack_exceeds_the_buffers_delay() {
    let (plan, _) = plan_with_setup_margin(SHORT_OF_HOLD, |found| {
        let setup = found.delay_without.rise / 2.0;
        for edge in Edge::BOTH {
            let shortfall = SHORT_OF_HOLD - found.early.get(edge);
            let room = found.late.get(edge) - (found.late.rise - setup);
            assert!(room > shortfall && room < found.delay_without.get(edge));
        }
        found.late.rise - setup
    });

    assert_eq!(plan, None);
}
/// A buffer as `BUF_X1` but that drives any load at almost the same delay
/// and slew.
const STRONG_BUFFER: &str = "cell (BUF_STRONG) { area : 1.0 ;
        pin (A) { direction : input ; capacitance : 0.97 ; }
        pin (Z) { direction : output ; function : \"A\" ; max_capacitance : 60.0 ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (delay_t) { values (\"0.02, 0.03\", \"0.03, 0.04\") ; }
            cell_fall (delay_t) { values (\"0.02, 0.03\", \"0.03, 0.04\") ; }
            rise_transition (delay_t) { values (\"0.005, 0.03\", \"0.01, 0.04\") ; }
            fall_transition (delay_t) { values (\"0.005, 0.03\", \"0.01, 0.04\") ; } } } }";

/// What a hold buffer put between the flops of [`apart`], with a stronger
/// buffer to size to, ends as under `constraints`: the buffers inserted and
/// resized, its size, and the size its load calls for.
fn try_hold_buffer_between_far_flops(constraints: &str) -> (usize, usize, String, String) {
    let mut db = crate::testing::design_with_copies(
        &def_text(
            "apart",
            &DefSections {
                components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 602000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( f2 D ) ;\n",
                inputs: &["clk"],
                outputs: &[],
            },
        ),
        &[("BUF_X1", "BUF_STRONG")],
    );
    let lib = crate::testing::cells_with(STRONG_BUFFER);
    let mut found = None;
    with_wired_session(&mut db, &lib, constraints, |resizer| {
        let options = HoldOptions::default();
        let buffer = hold_buffer(resizer).unwrap();
        let (driver, load) = (pin(resizer.db, "f1/Q"), pin(resizer.db, "f2/D"));
        let mut repair = HoldRepair {
            buffer,
            options: &options,
            inserted: 0,
            resized: 0,
        };
        let at = Point::new(Dbu::new(300_000), Dbu::new(2800));
        repair.try_insert(resizer, driver, &[load], at).unwrap();
        let hold = resizer.db.cell_id("hold1").unwrap();
        let output = resizer.netlist().db.cells[hold]
            .pins
            .iter()
            .copied()
            .find(|&p| resizer.netlist().is_driver(p))
            .unwrap();
        let name = |id| resizer.link.liberty.cell(id).name.clone();
        let size = resizer.link.cell(resizer.db, hold).unwrap().0;
        let called_for = resizer.find_target_cell(buffer.id, resizer.load_cap(output), false);
        found = Some((
            repair.inserted,
            repair.resized,
            name(size),
            name(called_for),
        ));
    });
    found.unwrap()
}

#[test]
fn a_hold_buffer_whose_load_is_in_hand_stays_the_smallest_size() {
    let (inserted, resized, size, _) = try_hold_buffer_between_far_flops(CLOCK);

    assert_eq!((inserted, resized, size.as_str()), (1, 0, "BUF_X1"));
}

#[test]
fn a_hold_buffer_near_its_slew_limit_is_sized_to_its_load() {
    let tight = format!("{CLOCK}set_max_transition 0.05 [current_design]\n");

    let (inserted, resized, size, called_for) = try_hold_buffer_between_far_flops(&tight);

    assert_eq!((inserted, resized), (1, 1));
    assert_ne!(size, "BUF_X1");
    assert_eq!(size, called_for);
}

/// A hold buffer put in between the flops of [`pair`] with the setup
/// margin `setup_margin` that keeps or loses it, as `allow_violations`.
fn buffer_pair_under_setup_margin(setup_margin: Seconds, allow_violations: bool) -> usize {
    let mut db = pair();
    let mut inserted = 0;
    with_timed_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let options = HoldOptions {
            setup_margin,
            allow_setup_violations: allow_violations,
            ..HoldOptions::default()
        };
        let buffer = hold_buffer(resizer).unwrap();
        let (driver, load) = (pin(resizer.db, "f1/Q"), pin(resizer.db, "f2/D"));
        let mut repair = HoldRepair {
            buffer,
            options: &options,
            inserted: 0,
            resized: 0,
        };
        let at = Point::new(Dbu::new(14_000), Dbu::new(2800));
        repair.try_insert(resizer, driver, &[load], at).unwrap();
        inserted = repair.inserted;
    });
    inserted
}

#[test]
fn a_hold_buffer_that_worsens_the_worst_setup_slack_below_the_margin_is_taken_back() {
    assert_eq!(
        buffer_pair_under_setup_margin(Seconds::new(10e-9), false),
        0
    );
    assert_eq!(buffer_pair_under_setup_margin(Seconds::ZERO, false), 1);
}

#[test]
fn a_hold_buffer_may_break_setup_where_setup_violations_are_allowed() {
    assert_eq!(buffer_pair_under_setup_margin(Seconds::new(10e-9), true), 1);
}

/// A buffer of the clock cells' size but for the input capacitance it
/// presents.
fn heavy_buffer(name: &str, capacitance_ff: f64) -> String {
    slow_buffer(name, 1.0, 0.1, "").replace(
        "capacitance : 1.0",
        &format!("capacitance : {capacitance_ff}"),
    )
}

#[test]
fn a_hold_buffer_that_raises_its_drivers_slew_by_more_than_a_fifth_is_taken_back() {
    let mut db = crate::testing::design_with_copies(
        &def_text(
            "heavy",
            &DefSections {
                components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 12000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( f2 D ) ;\n",
                inputs: &["clk"],
                outputs: &[],
            },
        ),
        &[("BUF_X1", "BUF_HEAVY")],
    );
    let lib = crate::testing::cells_with(&heavy_buffer("BUF_HEAVY", 40.0));
    let mut kept = 1;
    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        let options = HoldOptions::default();
        let heavy = resizer.link.liberty.cell_id("BUF_HEAVY").unwrap();
        let buffer = resizer.link.liberty.buffer_cell(heavy).unwrap();
        let (driver, load) = (pin(resizer.db, "f1/Q"), pin(resizer.db, "f2/D"));
        let mut repair = HoldRepair {
            buffer,
            options: &options,
            inserted: 0,
            resized: 0,
        };
        let at = Point::new(Dbu::new(7000), Dbu::new(2800));
        let before = resizer.graph().slew(driver).max();

        repair.try_insert(resizer, driver, &[load], at).unwrap();

        kept = repair.inserted;
        assert_eq!(resizer.graph().slew(driver).max(), before);
    });
    assert_eq!(kept, 0);
}

/// A buffer `b` of `BUF_X1` between flops `f1` and `f2`.
fn buffer_between_flops() -> pare_common::db::design::NetlistDB {
    design(&def_text(
        "between",
        &DefSections {
            components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                         - b BUF_X1 + PLACED ( 6000 2800 ) N ;\n\
                         - f2 DFF_X1 + PLACED ( 12000 2800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( b A ) ;\n\
                   - r ( b Z ) ( f2 D ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// The clock test cells with the first `from` of `BUF_X1`'s text made `to`.
fn cells_with_buffer_edit(from: &str, to: &str) -> Liberty {
    let source = clock_cells_source();
    let start = source.find("cell (BUF_X1)").unwrap();
    let end = start + source[start..].find("cell (AND2_X1)").unwrap();
    let edited = source[start..end].replacen(from, to, 1);
    assert_ne!(edited, source[start..end], "{from} is in BUF_X1");
    cells(&format!("{}{edited}{}", &source[..start], &source[end..]))
}

/// Whether the output of `b` of [`buffer_between_flops`] has its load in
/// hand over `lib` under `constraints`, and its slew there.
fn buffer_output_in_hand(lib: &Liberty, constraints: &str) -> (bool, Seconds) {
    let mut db = buffer_between_flops();
    let mut found = (false, Seconds::ZERO);
    with_wired_session(&mut db, lib, constraints, |resizer| {
        let output = pin(resizer.db, "b/Z");
        found = (
            HoldRepair::load_in_hand(resizer, output),
            resizer.graph().slew(output).max(),
        );
    });
    found
}

#[test]
fn a_load_near_the_drivers_load_limit_is_not_in_hand_and_a_zero_limit_limits_nothing() {
    let near = cells_with_buffer_edit("max_capacitance : 60.0", "max_capacitance : 1.3");
    let zero = cells_with_buffer_edit("max_capacitance : 60.0", "max_capacitance : 0.0");

    assert!(buffer_output_in_hand(&cells(&clock_cells_source()), CLOCK).0);
    assert!(!buffer_output_in_hand(&near, CLOCK).0);
    assert!(buffer_output_in_hand(&zero, CLOCK).0);
}

#[test]
fn a_load_near_the_drivers_own_slew_limit_is_not_in_hand() {
    let (_, slew) = buffer_output_in_hand(&cells(&clock_cells_source()), CLOCK);
    let limit = slew.get() * 1e9 * 1.1;
    let near = cells_with_buffer_edit(
        "max_capacitance : 60.0 ;",
        &format!("max_capacitance : 60.0 ; max_transition : {limit} ;"),
    );

    let (in_hand, _) = buffer_output_in_hand(&near, CLOCK);

    assert!(!in_hand);
}

#[test]
fn a_design_slew_limit_of_zero_limits_nothing() {
    let unlimited = format!("{CLOCK}set_max_transition 0 [current_design]\n");

    let (in_hand, _) = buffer_output_in_hand(&cells(&clock_cells_source()), &unlimited);

    assert!(in_hand);
}
