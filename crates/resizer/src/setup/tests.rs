use super::*;
use crate::testing::{
    DefSections, cells, clock_cells_source, def_text, design, with_wired_session,
};
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
