use super::*;
use crate::testing::{
    DefSections, cells, clock_cells_source, def_text, design, pin, with_wired_session,
};
use pare_common::db::indices::CellId;
use pare_common::db::liberty::ArcSetId;
use std::fmt::Write;

fn slacks(worst: f64, end: f64, total: f64) -> PassSlacks {
    PassSlacks {
        worst: Seconds::new(worst),
        end: Seconds::new(end),
        total: Seconds::new(total),
    }
}

#[test]
fn the_last_gasp_keeps_a_pass_only_when_neither_worst_nor_total_slack_worsens() {
    let kept = slacks(-1e-10, -1e-10, -5e-10);
    let rule = PassRule::LastGasp;

    assert!(rule.keeps(kept, slacks(-0.9e-10, -0.9e-10, -4e-10), true));
    assert!(rule.keeps(kept, kept, true));
    assert!(!rule.keeps(kept, slacks(-0.9e-10, -0.9e-10, -6e-10), true));
    assert!(!rule.keeps(kept, slacks(-1.1e-10, -0.5e-10, -4e-10), false));
    assert_eq!(rule.worse_passes_tolerated(), 0);
}

#[test]
fn the_legacy_phase_keeps_a_better_endpoint_only_off_the_worst_one() {
    let kept = slacks(-1e-10, -0.5e-10, -5e-10);
    let better_end = slacks(-1e-10, -0.4e-10, -6e-10);
    let rule = PassRule::Legacy;

    assert!(rule.keeps(kept, slacks(-0.9e-10, -0.5e-10, -6e-10), true));
    assert!(rule.keeps(kept, better_end, false));
    assert!(!rule.keeps(kept, better_end, true));
}

#[test]
fn a_pass_makes_more_moves_the_worse_its_violation_and_at_least_one() {
    assert_eq!(scaled_repairs(5, 1.0), 5);
    assert_eq!(scaled_repairs(5, 0.5), 3);
    assert_eq!(scaled_repairs(5, 0.0), 1);
    assert_eq!(scaled_repairs(5, -0.5), 1);
}

#[test]
fn an_intrinsic_delay_extrapolated_below_zero_stays_negative() {
    let lib = cells(&clock_cells_source().replacen(
        "cell_rise (delay_t) { values (\"0.03, 0.30\", \"0.04, 0.31\") ; }",
        "cell_rise (delay_t) { values (\"0.0, 0.30\", \"0.01, 0.31\") ; }",
        1,
    ));
    let buffer = lib.cell(lib.cell_id("BUF_X1").unwrap());

    let rise = intrinsic_delay(&buffer.arc_sets[ArcSetId::new(0)], Edge::Rise, Edge::Rise);

    assert!(rise < Seconds::ZERO, "{rise:?}");
}

/// Buffers in the chain between the flops of [`chain`].
const CHAIN: usize = 6;

/// A flop launching through a chain of buffers 40 microns apart into
/// another flop.
fn chain() -> pare_common::db::design::NetlistDB {
    let x = |i: usize| 2000 + 80_000 * i;
    let mut components = format!("- f1 DFF_X1 + PLACED ( {} 2800 ) N ;\n", x(0));
    let mut nets = String::from("- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n");
    let mut from = "f1 Q".to_string();
    for i in 1..=CHAIN {
        writeln!(components, "- b{i} BUF_X1 + PLACED ( {} 2800 ) N ;", x(i)).unwrap();
        writeln!(nets, "- n{i} ( {from} ) ( b{i} A ) ;").unwrap();
        from = format!("b{i} Z");
    }
    writeln!(
        components,
        "- f2 DFF_X1 + PLACED ( {} 2800 ) N ;",
        x(CHAIN + 1)
    )
    .unwrap();
    writeln!(nets, "- n{} ( {from} ) ( f2 D ) ;", CHAIN + 1).unwrap();
    design(&def_text(
        "chain",
        &DefSections {
            components: &components,
            nets: &nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

#[test]
fn a_failing_path_ends_with_better_worst_slack() {
    let mut db = chain();
    let lib = cells(&clock_cells_source());
    let clock = "create_clock -name clk -period 0.25 [get_ports clk]\n";

    with_wired_session(&mut db, &lib, clock, |resizer| {
        let before = resizer.sta().worst_slack(MinMax::Max).unwrap().slack;

        let summary = repair_setup(resizer, SetupOptions::default(), &mut |_| {}).unwrap();

        let after = resizer.sta().worst_slack(MinMax::Max).unwrap().slack;
        assert!(before < Seconds::ZERO);
        assert!(after > before, "{before:?} to {after:?}");
        assert_eq!(summary.wns_before, Some(before));
        assert_eq!(summary.wns_after, Some(after));
    });
}

#[test]
fn a_path_that_meets_its_clock_is_left_alone() {
    let mut db = chain();
    let lib = cells(&clock_cells_source());
    let clock = "create_clock -name clk -period 10.0 [get_ports clk]\n";

    with_wired_session(&mut db, &lib, clock, |resizer| {
        let summary = repair_setup(resizer, SetupOptions::default(), &mut |_| {}).unwrap();

        assert_eq!(summary.failing_before, 0);
        assert_eq!(summary.removed + summary.resized, 0, "{summary:?}");
    });
}

/// Parallel flop-to-flop paths on one clock, the path `k` of `lengths[k]`
/// buffers running `fk_a`, `bk_1`, ..., `fk_b`.
fn chains(lengths: &[usize]) -> pare_common::db::design::NetlistDB {
    let mut components = String::new();
    let mut clock_net = String::from("- clk ( PIN clk )");
    let mut data_nets = String::new();
    for (k, &count) in lengths.iter().enumerate() {
        let y = 2800 + 20_000 * k;
        let x = |i: usize| 2000 + 40_000 * i;
        writeln!(components, "- f{k}_a DFF_X1 + PLACED ( {} {y} ) N ;", x(0)).unwrap();
        let mut from = format!("f{k}_a Q");
        for i in 1..=count {
            writeln!(
                components,
                "- b{k}_{i} BUF_X1 + PLACED ( {} {y} ) N ;",
                x(i)
            )
            .unwrap();
            writeln!(data_nets, "- n{k}_{i} ( {from} ) ( b{k}_{i} A ) ;").unwrap();
            from = format!("b{k}_{i} Z");
        }
        writeln!(
            components,
            "- f{k}_b DFF_X1 + PLACED ( {} {y} ) N ;",
            x(count + 1)
        )
        .unwrap();
        writeln!(data_nets, "- n{k}_{} ( {from} ) ( f{k}_b D ) ;", count + 1).unwrap();
        write!(clock_net, " ( f{k}_a CK ) ( f{k}_b CK )").unwrap();
    }
    let nets = format!("{clock_net} ;\n{data_nets}");
    design(&def_text(
        "chains",
        &DefSections {
            components: &components,
            nets: &nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// How many buffers of each chain stand where they did, as their original
/// size, after `options` repair `chains(lengths)` at a clock of `period`
/// nanoseconds.
fn untouched_buffers(lengths: &[usize], period: f64, options: SetupOptions) -> Vec<usize> {
    let mut db = chains(lengths);
    let lib = cells(&clock_cells_source());
    let clock = format!("create_clock -name clk -period {period} [get_ports clk]\n");
    let buffers: Vec<Vec<(CellId, PinId)>> = lengths
        .iter()
        .enumerate()
        .map(|(k, &count)| {
            (1..=count)
                .map(|i| {
                    let cell = db.cell_id(&format!("b{k}_{i}")).unwrap();
                    (cell, pin(&db, &format!("b{k}_{i}/A")))
                })
                .collect()
        })
        .collect();
    let mut untouched = Vec::new();
    with_wired_session(&mut db, &lib, &clock, |resizer| {
        let _ = repair_setup(resizer, options, &mut |_| {}).unwrap();
        for chain in &buffers {
            let same = chain
                .iter()
                .filter(|&&(cell, input)| {
                    !resizer.netlist().is_retired(input)
                        && resizer.link.cell(resizer.db, cell).unwrap().1.name == "BUF_X1"
                })
                .count();
            untouched.push(same);
        }
    });
    untouched
}

#[test]
fn repairing_half_the_failing_endpoints_leaves_more_of_the_design_alone() {
    let all = SetupOptions {
        skip_last_gasp: true,
        ..SetupOptions::default()
    };
    let half = SetupOptions {
        repair_tns_share: Share::new(0.5).unwrap(),
        ..all.clone()
    };

    let all: usize = untouched_buffers(&[6, 5, 4, 3], 0.2, all).iter().sum();
    let half: usize = untouched_buffers(&[6, 5, 4, 3], 0.2, half).iter().sum();

    assert!(half > all, "{half} untouched against {all}");
}

/// Runs `test` on a repair of the design `db` under a clock of `period`
/// nanoseconds, with `options`.
fn with_repair(
    db: &mut pare_common::db::design::NetlistDB,
    period: f64,
    options: SetupOptions,
    test: impl FnOnce(&mut Repair<'_, '_>),
) {
    with_repair_over(db, &cells(&clock_cells_source()), period, options, test);
}

/// [`with_repair`] with the cells of `lib`.
fn with_repair_over(
    db: &mut pare_common::db::design::NetlistDB,
    lib: &pare_common::db::liberty::Liberty,
    period: f64,
    options: SetupOptions,
    test: impl FnOnce(&mut Repair<'_, '_>),
) {
    let clock = format!("create_clock -name clk -period {period} [get_ports clk]\n");
    with_wired_session(db, lib, &clock, |resizer| {
        let moves = Moves::new(resizer).unwrap();
        let initial_area = resizer.design_area();
        let mut ignore = |_: &SetupProgress| {};
        let mut repair = Repair {
            resizer,
            moves,
            options,
            report: &mut ignore,
            initial_area,
            iteration: 0,
            min_violation: Seconds::ZERO,
            max_violation: Seconds::ZERO,
            fallback: false,
        };
        test(&mut repair);
    });
}

#[test]
fn the_violations_a_phase_scales_by_are_those_of_its_best_and_worst_endpoint() {
    let mut db = chains(&[6, 4]);

    with_repair(&mut db, 0.2, SetupOptions::default(), |repair| {
        let violating = repair.violating_endpoints();
        let tns = repair.tns();

        repair.legacy_phase(&violating, tns).unwrap();

        assert_eq!(violating.len(), 2);
        assert_eq!(repair.min_violation, -violating[1].1);
        assert_eq!(repair.max_violation, -violating[0].1);
        assert!(repair.min_violation > Seconds::ZERO);
        assert!(repair.max_violation > repair.min_violation);
    });
}

/// How many gates the pass of `repair_endpoint` sized up, on a path of
/// six buffers whose endpoints' violations run from 0.2 to 1 ns and with
/// at most five moves a pass.
fn sizings_by_a_pass(path_slack_ns: f64, tune: impl FnOnce(&mut Repair<'_, '_>)) -> usize {
    let mut db = chains(&[6]);
    let end = pin(&db, "f0_b/D");
    let options = SetupOptions {
        max_repairs_per_pass: 5,
        ..SetupOptions::default()
    };
    let mut sized = 0;
    with_repair(&mut db, 0.2, options, |repair| {
        repair.min_violation = Seconds::new(0.2e-9);
        repair.max_violation = Seconds::new(1.0e-9);
        tune(repair);

        let changed = repair
            .repair_endpoint(end, Seconds::new(path_slack_ns * 1e-9), &[MoveKind::SizeUp])
            .unwrap();

        assert!(changed);
        sized = repair.moves.log.made(MoveKind::SizeUp);
    });
    sized
}

#[test]
fn a_pass_makes_the_moves_its_endpoints_share_of_the_violations_asks_for() {
    // Halfway between 0.2 and 1 ns violated: one move and half of four more.
    assert_eq!(sizings_by_a_pass(-0.6, |_| {}), 3);
    assert_eq!(sizings_by_a_pass(-1.0, |_| {}), 5);
    assert_eq!(sizings_by_a_pass(-0.2, |_| {}), 1);
}

#[test]
fn a_pass_after_a_pass_that_did_not_pay_makes_one_move() {
    assert_eq!(sizings_by_a_pass(-0.6, |repair| repair.fallback = true), 1);
}

#[test]
fn a_pass_without_a_spread_of_violations_makes_one_move() {
    let tune = |repair: &mut Repair<'_, '_>| repair.max_violation = repair.min_violation;

    assert_eq!(sizings_by_a_pass(-0.6, tune), 1);
}

#[test]
fn a_pass_allowed_one_move_makes_one() {
    let tune = |repair: &mut Repair<'_, '_>| repair.options.max_repairs_per_pass = 1;

    assert_eq!(sizings_by_a_pass(-0.6, tune), 1);
}

#[test]
fn taking_a_buffer_out_counts_for_the_whole_pass() {
    let mut db = chains(&[6]);
    let end = pin(&db, "f0_b/D");
    let options = SetupOptions {
        max_repairs_per_pass: 5,
        ..SetupOptions::default()
    };

    with_repair(&mut db, 0.2, options, |repair| {
        repair.min_violation = Seconds::new(0.2e-9);
        repair.max_violation = Seconds::new(1.0e-9);

        let changed = repair
            .repair_endpoint(end, Seconds::new(-0.6e-9), &[MoveKind::Unbuffer])
            .unwrap();

        assert!(changed);
        assert_eq!(repair.moves.log.made(MoveKind::Unbuffer), 1);
    });
}

/// Whether a repair at `iteration` with the phase `phase` is told to
/// stop, by a design whose total negative slack is `tns` more negative
/// than the phase's last check saw it.
fn terminated_at(iteration: usize, phase: impl FnOnce(Seconds) -> Phase) -> (bool, Phase) {
    let mut db = chains(&[6]);
    let mut outcome = None;
    with_repair(&mut db, 0.2, SetupOptions::default(), |repair| {
        repair.iteration = iteration;
        let mut phase = phase(repair.tns());
        let stopped = repair.terminate(&mut phase);
        outcome = Some((stopped, phase));
    });
    outcome.unwrap()
}

/// A phase that last saw `prev_tns` and began with `initial_tns`.
fn phase_seeing(initial_tns: Seconds, prev_tns: Seconds) -> Phase {
    Phase::new(Vec::new(), 1, initial_tns, prev_tns)
}

#[test]
fn a_repair_that_fixes_nothing_is_stopped_at_a_check_after_the_first_thousand_iterations() {
    let (stopped, _) = terminated_at(1100, |now| phase_seeing(now * 2.0, now));

    assert!(stopped);
}

#[test]
fn a_repair_that_fixes_enough_goes_on() {
    let (stopped, _) = terminated_at(1100, |now| phase_seeing(now * 4.0, now * 2.0));

    assert!(!stopped);
}

#[test]
fn a_repair_is_not_stopped_within_its_first_thousand_iterations() {
    assert!(!terminated_at(1000, |now| phase_seeing(now * 2.0, now)).0);
    assert!(!terminated_at(100, |now| phase_seeing(now * 2.0, now)).0);
}

#[test]
fn a_repair_is_judged_only_every_hundred_iterations() {
    assert!(!terminated_at(1150, |now| phase_seeing(now * 2.0, now)).0);
}

#[test]
fn a_repair_from_no_violation_fixes_no_share_of_it() {
    assert!(terminated_at(1100, |now| phase_seeing(Seconds::ZERO, now)).0);
}

#[test]
fn a_repair_fixing_just_the_rate_asked_for_goes_on() {
    let (stopped, _) = terminated_at(1100, |now| {
        let fixed = now * 2.0 - now;
        let mut phase = phase_seeing(fixed * 2.0, now * 2.0);
        phase.threshold = 0.5;
        phase
    });

    assert!(!stopped);
}

#[test]
fn a_check_remembers_the_slack_it_saw() {
    let mut db = chains(&[6]);
    with_repair(&mut db, 0.2, SetupOptions::default(), |repair| {
        repair.iteration = 1100;
        let now = repair.tns();
        let mut phase = phase_seeing(now * 2.0, now * 3.0);

        let _ = repair.terminate(&mut phase);

        assert_eq!(phase.prev_tns, now);
    });
}

#[test]
fn the_rate_asked_for_doubles_every_thousand_iterations() {
    let (_, phase) = terminated_at(2000, |now| {
        let mut phase = phase_seeing(now, now);
        phase.threshold = 0.25;
        phase
    });
    assert!((phase.threshold - 0.5).abs() < f64::EPSILON);

    let (_, phase) = terminated_at(1100, |now| {
        let mut phase = phase_seeing(now, now);
        phase.threshold = 0.25;
        phase
    });
    assert!((phase.threshold - 0.25).abs() < f64::EPSILON);
}

/// The clock cells with the rise and fall delays of `BUF_X1` growing
/// `rise_slope` and `fall_slope` nanoseconds over its 1 to 100 fF, whatever
/// the input slew.
fn buffers_slowing_by(rise_slope: f64, fall_slope: f64) -> pare_common::db::liberty::Liberty {
    let table = |slope: f64| format!("\"0.03, {}\", \"0.03, {}\"", 0.03 + slope, 0.03 + slope);
    let source = clock_cells_source();
    let source = source.replacen(
        "cell_rise (delay_t) { values (\"0.03, 0.30\", \"0.04, 0.31\") ; }",
        &format!("cell_rise (delay_t) {{ values ({}) ; }}", table(rise_slope)),
        1,
    );
    cells(&source.replacen(
        "cell_fall (delay_t) { values (\"0.03, 0.30\", \"0.04, 0.31\") ; }",
        &format!("cell_fall (delay_t) {{ values ({}) ; }}", table(fall_slope)),
        1,
    ))
}

/// The load delays of the buffers on the worst path of a chain of four,
/// beside what the slope of the edge the path takes through them gives.
fn load_delays_beside_expected(rise_slope: f64, fall_slope: f64) -> Vec<(Seconds, Seconds)> {
    let mut db = chains(&[4]);
    let end = pin(&db, "f0_b/D");
    let lib = buffers_slowing_by(rise_slope, fall_slope);
    let mut found = Vec::new();
    with_repair_over(&mut db, &lib, 0.2, SetupOptions::default(), |repair| {
        let netlist = repair.resizer.netlist();
        let path = repair
            .resizer
            .sta()
            .worst_slack_path(&netlist, end, MinMax::Max)
            .unwrap();
        for (index, point) in path.points.iter().enumerate() {
            let name = repair.resizer.db.pin_name(point.pin);
            if !name.starts_with('b') || !name.ends_with("/Z") {
                continue;
            }
            let slope = if point.edge == Edge::Rise {
                rise_slope
            } else {
                fall_slope
            };
            // Delay grows linearly with load from the 1 fF of the table's
            // first column, so at none it is `slope / 99 fF` below it.
            let expected =
                Seconds::new(slope * 1e-9 / 99e-15) * repair.resizer.load_cap(point.pin).get();
            found.push((repair.load_delay(&path, index).unwrap(), expected));
        }
    });
    found
}

#[test]
fn a_driver_s_load_delay_follows_the_arc_of_the_edge_the_path_takes() {
    for (rise_slope, fall_slope) in [(0.27, 2.97), (2.97, 0.27)] {
        let delays = load_delays_beside_expected(rise_slope, fall_slope);

        assert_eq!(delays.len(), 4);
        for (delay, expected) in delays {
            assert!(
                (delay - expected).abs() < expected * 1e-3,
                "{delay:?} {expected:?}"
            );
        }
    }
}

#[test]
fn the_intrinsic_delay_is_that_of_the_arc_between_the_edges_asked_for() {
    let lib = cells(&clock_cells_source().replacen(
        "cell_fall (delay_t) { values (\"0.03, 0.30\", \"0.04, 0.31\") ; }",
        "cell_fall (delay_t) { values (\"0.07, 0.30\", \"0.08, 0.31\") ; }",
        1,
    ));
    let buffer = lib.cell(lib.cell_id("BUF_X1").unwrap());
    let set = &buffer.arc_sets[ArcSetId::new(0)];

    let rise = intrinsic_delay(set, Edge::Rise, Edge::Rise);
    let fall = intrinsic_delay(set, Edge::Fall, Edge::Fall);

    // The tables extrapolate to no slew and no load: the first entry less
    // 0.005 ns of slew at 0.01 ns per 0.195 ns and 1 fF of load at
    // (0.30 - first entry) ns per 99 fF.
    assert!(
        (rise - Seconds::new(0.027_017e-9)).abs() < Seconds::new(1e-15),
        "{rise:?}"
    );
    assert!(
        (fall - Seconds::new(0.067_421e-9)).abs() < Seconds::new(1e-15),
        "{fall:?}"
    );
    assert_eq!(intrinsic_delay(set, Edge::Rise, Edge::Fall), Seconds::ZERO);
    assert_eq!(intrinsic_delay(set, Edge::Fall, Edge::Rise), Seconds::ZERO);
}

/// A flop launching through a clock buffer, then two weaker buffers, into
/// another flop.
fn strong_then_weak() -> pare_common::db::design::NetlistDB {
    let components = "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
        - c1 CLKBUF_X3 + PLACED ( 82000 2800 ) N ;\n\
        - b1 BUF_X1 + PLACED ( 162000 2800 ) N ;\n\
        - b2 BUF_X1 + PLACED ( 242000 2800 ) N ;\n\
        - f2 DFF_X1 + PLACED ( 322000 2800 ) N ;\n";
    let nets = "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n\
        - n0 ( f1 Q ) ( c1 A ) ;\n- n1 ( c1 Z ) ( b1 A ) ;\n\
        - n2 ( b1 Z ) ( b2 A ) ;\n- n3 ( b2 Z ) ( f2 D ) ;\n";
    design(&def_text(
        "match",
        &DefSections {
            components,
            nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// Repairs `db` under a clock of `period` nanoseconds with `sequence`
/// alone, reporting every progress line.
fn repaired_by(
    db: &mut pare_common::db::design::NetlistDB,
    period: f64,
    sequence: Vec<MoveKind>,
) -> (SetupSummary, Vec<SetupProgress>, usize) {
    let lib = cells(&clock_cells_source());
    let clock = format!("create_clock -name clk -period {period} [get_ports clk]\n");
    let before = db.cells.len();
    let mut reports = Vec::new();
    let mut summary = SetupSummary::default();
    with_wired_session(db, &lib, &clock, |resizer| {
        let options = SetupOptions {
            sequence,
            skip_last_gasp: true,
            ..SetupOptions::default()
        };
        summary = repair_setup(resizer, options, &mut |p| reports.push(p.clone())).unwrap();
    });
    (summary, reports, db.cells.len() - before)
}

#[test]
fn buffers_sized_to_their_stronger_driver_are_counted_as_resized() {
    let mut db = strong_then_weak();
    let weak = [db.cell_id("b1").unwrap(), db.cell_id("b2").unwrap()];

    let (summary, reports, _) = repaired_by(&mut db, 0.2, vec![MoveKind::SizeUpMatch]);

    let resized = weak
        .iter()
        .filter(|&&cell| db.cell_macro(cell).name == "CLKBUF_X3")
        .count();
    assert!(resized > 0);
    assert_eq!(summary.resized, resized);
    assert_eq!(
        summary.inserted + summary.removed + summary.cloned + summary.swapped,
        0
    );
    let last = reports.last().unwrap();
    assert_eq!(last.iteration, None);
    assert_eq!(last.resized, resized);
}

/// A flop launching through a buffer into six flops directly and six more
/// through buffers of their own.
fn buffer_with_twelve_loads() -> pare_common::db::design::NetlistDB {
    let mut components = String::from(
        "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n- g1 BUF_X1 + PLACED ( 42000 2800 ) N ;\n",
    );
    let mut clock_net = String::from("- clk ( PIN clk ) ( f1 CK )");
    let mut nets = String::from("- n0 ( f1 Q ) ( g1 A ) ;\n");
    let mut fanout = String::from("- n1 ( g1 Z )");
    for i in 0..6 {
        let y = 2800 + 20_000 * i;
        writeln!(components, "- d{i} DFF_X1 + PLACED ( 122000 {y} ) N ;").unwrap();
        writeln!(components, "- s{i} BUF_X1 + PLACED ( 82000 {y} ) N ;").unwrap();
        writeln!(components, "- e{i} DFF_X1 + PLACED ( 162000 {y} ) N ;").unwrap();
        write!(fanout, " ( d{i} D ) ( s{i} A )").unwrap();
        write!(clock_net, " ( d{i} CK ) ( e{i} CK )").unwrap();
        writeln!(nets, "- m{i} ( s{i} Z ) ( e{i} D ) ;").unwrap();
    }
    let nets = format!("{clock_net} ;\n{nets}{fanout} ;\n");
    design(&def_text(
        "twelve",
        &DefSections {
            components: &components,
            nets: &nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

#[test]
fn loads_split_behind_a_buffer_are_counted_as_inserted() {
    let mut db = buffer_with_twelve_loads();

    let (summary, reports, new_cells) = repaired_by(&mut db, 0.2, vec![MoveKind::SplitLoad]);

    assert!(summary.inserted > 0, "{summary:?}");
    assert_eq!(summary.inserted, new_cells);
    assert_eq!(summary.split, summary.inserted);
    let last = reports.last().unwrap();
    assert_eq!(last.inserted, summary.inserted);
}

#[test]
fn the_last_gasp_has_nothing_to_do_where_no_slack_is_negative() {
    let mut db = chains(&[3]);
    let options = SetupOptions {
        margin: Seconds::new(20e-9),
        ..SetupOptions::default()
    };

    with_repair(&mut db, 10.0, options, |repair| {
        assert!(!repair.violating_endpoints().is_empty());
        assert!(repair.tns() >= Seconds::ZERO);

        repair.last_gasp(Seconds::ZERO).unwrap();

        assert_eq!(repair.iteration, 0);
    });
}

#[test]
fn the_last_gasp_remembers_the_total_slack_of_its_last_kept_pass() {
    let mut db = chains(&[6]);
    let end = pin(&db, "f0_b/D");

    with_repair(&mut db, 0.2, SetupOptions::default(), |repair| {
        let start = repair.tns();
        let mut phase = Phase::new(LAST_GASP_SEQUENCE.to_vec(), LAST_GASP_PASSES, start, start);
        let (mut kept_worst, _) = repair.worst();

        let _ = repair
            .endpoint_passes(PassRule::LastGasp, 1, end, &mut phase, &mut kept_worst)
            .unwrap();

        assert!(repair.tns() > start, "the passes keep nothing to remember");
        assert_eq!(phase.prev_tns, repair.tns());
    });
}

#[test]
fn the_legacy_phase_leaves_the_total_slack_to_its_rate_checks() {
    let mut db = chains(&[6]);
    let end = pin(&db, "f0_b/D");

    with_repair(&mut db, 0.2, SetupOptions::default(), |repair| {
        let start = repair.tns();
        let mut phase = Phase::new(repair.options.sequence.clone(), 5, start, start);
        let (mut kept_worst, _) = repair.worst();

        let _ = repair
            .endpoint_passes(PassRule::Legacy, 1, end, &mut phase, &mut kept_worst)
            .unwrap();

        assert!(repair.tns() > start);
        assert_eq!(phase.prev_tns, start);
    });
}
