//! Design, setup and hold repair of a small generated design over the
//! clock test cells, pinned down whole.

#![allow(clippy::unwrap_used)]

mod common;

use common::{Work, at, cells_counting_fanout, clock_cells, repaired_in_session};
use pare_common::db::design::NetlistDB;
use pare_common::geom::dbu::Dbu;
use pare_common::testing;
use pare_common::units::Seconds;
use pare_resizer::{
    HoldOptions, RepairSummary, Repairer, Resizer, SetupOptions, Sta, repair_hold, repair_setup,
};
use std::fmt::Write as _;
use std::path::Path;

/// Three kinds of work for repair, as DEF cells and nets: eight flop to
/// flop paths through six gates spread across the die, a buffer fanning
/// out to 24 flops all over it, and four pairs of flops side by side.
fn timing_work() -> Work {
    let mut cells = Vec::new();
    let mut nets = Vec::new();
    for s in 0..8 {
        let row = 10 + s * 16;
        cells.push(format!("- ffa{s} DFF_X1 + PLACED {} ;", at(10, row)));
        let mut previous = format!("( ffa{s} Q )");
        for k in 0..6 {
            let master = if k % 2 == 0 { "BUF_X1" } else { "INV_X1" };
            let out = if k % 2 == 0 { "Z" } else { "ZN" };
            cells.push(format!(
                "- g{s}_{k} {master} + PLACED {} ;",
                at(60 + 140 * k, row)
            ));
            nets.push(format!("- p{s}_{k} {previous} ( g{s}_{k} A ) ;"));
            previous = format!("( g{s}_{k} {out} )");
        }
        cells.push(format!("- ffb{s} DFF_X1 + PLACED {} ;", at(950, row)));
        nets.push(format!("- p{s}_6 {previous} ( ffb{s} D ) ;"));
    }
    cells.push(format!("- fan BUF_X1 + PLACED {} ;", at(500, 70)));
    let mut fanout = String::from("- fo ( fan Z )");
    for i in 0..24 {
        cells.push(format!(
            "- fo{i} DFF_X1 + PLACED {} ;",
            at(40 * i, (i * 37) % 130)
        ));
        let _ = write!(fanout, " ( fo{i} D )");
    }
    nets.push(fanout + " ;");
    nets.push("- in0 ( PIN in0 ) ( fan A ) ;".to_string());
    for h in 0..4 {
        let row = 4 + h * 32;
        cells.push(format!("- ha{h} DFF_X1 + PLACED {} ;", at(500, row)));
        cells.push(format!("- hb{h} DFF_X1 + PLACED {} ;", at(512, row)));
        nets.push(format!("- h{h} ( ha{h} Q ) ( hb{h} D ) ;"));
    }
    Work { cells, nets }
}

/// Nets design repair splits, as DEF cells and nets: a buffer driving 180
/// flops, past the load limit of any size of it, four flops each driving one flop across the
/// die, and one driving three flops at the die's far corners.
fn long_and_loaded_nets() -> Work {
    let mut cells = Vec::new();
    let mut nets = Vec::new();
    cells.push(format!("- fan BUF_X1 + PLACED {} ;", at(500, 70)));
    let mut fanout = String::from("- fo ( fan Z )");
    for i in 0..180 {
        cells.push(format!(
            "- fo{i} DFF_X1 + PLACED {} ;",
            at(450 + (i % 10) * 10, 60 + i / 10 * 2)
        ));
        let _ = write!(fanout, " ( fo{i} D )");
    }
    nets.push(fanout + " ;");
    nets.push("- in0 ( PIN in0 ) ( fan A ) ;".to_string());
    for k in 0..4 {
        let row = 20 + k * 30;
        cells.push(format!("- ffl{k} DFF_X1 + PLACED {} ;", at(5, row)));
        cells.push(format!("- ffr{k} DFF_X1 + PLACED {} ;", at(990, row)));
        nets.push(format!("- l{k} ( ffl{k} Q ) ( ffr{k} D ) ;"));
    }
    cells.push(format!("- ffm DFF_X1 + PLACED {} ;", at(500, 135)));
    let corners = [(0, 0), (990, 0), (0, 139)];
    for (n, &(c, r)) in corners.iter().enumerate() {
        cells.push(format!("- ffc{n} DFF_X1 + PLACED {} ;", at(c, r)));
    }
    nets.push("- m ( ffm Q ) ( ffc0 D ) ( ffc1 D ) ( ffc2 D ) ;".to_string());
    Work { cells, nets }
}

/// Every cell with its master and place, as text.
fn cells_text(db: &NetlistDB) -> String {
    let mut text = String::new();
    for cell in db.cells.iter().filter(|c| !c.retired) {
        let _ = writeln!(
            text,
            "{} {} {} {}",
            cell.name,
            db.library.macros[cell.macro_id].name,
            cell.location.x.get(),
            cell.location.y.get()
        );
    }
    text
}

/// What `repair` reports of a timed session over the design of `work`
/// under a clock of `period_ns`, then every cell once the session ends.
fn repaired_text(
    work: Work,
    period_ns: f64,
    repair: impl FnOnce(&mut Resizer<'_, Sta>) -> String,
) -> String {
    let constraints = format!(
        "create_clock -name core -period {period_ns} [get_ports clk]\n\
         set_input_delay 0.1 -clock core [get_ports in0]\n"
    );
    let (reported, db) = repaired_in_session(work, &constraints, &clock_cells(), repair);
    format!("{reported}\n{}", cells_text(&db))
}

/// Design, setup and hold repair of the timing work under a clock of
/// `period_ns`, as text: what each repair did and every cell after.
fn repaired(period_ns: f64) -> String {
    repaired_text(timing_work(), period_ns, |resizer| {
        let design = Repairer::new(resizer).repair_design().unwrap();
        let mut reports = Vec::new();
        let setup = repair_setup(resizer, SetupOptions::default(), &mut |progress| {
            reports.push((progress.iteration, progress.failing));
        })
        .unwrap();
        let hold = repair_hold(
            resizer,
            &HoldOptions {
                hold_margin: Seconds::new(0.12e-9),
                ..HoldOptions::default()
            },
        )
        .unwrap();
        format!("{design:#?}\nreported (iteration, failing): {reports:?}\n{setup:#?}\n{hold:#?}")
    })
}

fn assert_golden(name: &str, actual: &str) {
    testing::assert_golden(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden"),
        name,
        actual,
    );
}

#[test]
fn repairing_design_setup_and_hold_gives_what_it_always_has() {
    assert_golden("flow.txt", &repaired(0.25));
}

#[test]
fn setup_repair_short_of_its_target_gives_what_it_always_has() {
    assert_golden("flow_unclosed.txt", &repaired(0.18));
}

/// The longest wire design repair leaves, as `repair_design -max_wire_length
/// 100`.
const MAX_WIRE_LENGTH: Dbu = Dbu::new(200_000);

#[test]
fn design_repair_splits_long_and_overloaded_nets_as_it_always_has() {
    let repaired = repaired_text(long_and_loaded_nets(), 1.0, |resizer| {
        resizer.set_max_wire_length(Some(MAX_WIRE_LENGTH));
        let design = Repairer::new(resizer).repair_design().unwrap();
        format!("{design:#?}")
    });

    assert_golden("long_and_loaded_nets.txt", &repaired);
}

/// A buffer the input port drives, loading a flop at each of `places`.
fn fanout_work(places: &[(usize, usize)]) -> Work {
    let mut cells = vec![format!("- fan BUF_X1 + PLACED {} ;", at(500, 70))];
    let mut fanout = String::from("- fo ( fan Z )");
    for (i, &(c, r)) in places.iter().enumerate() {
        cells.push(format!("- s{i} DFF_X1 + PLACED {} ;", at(c, r)));
        let _ = write!(fanout, " ( s{i} D )");
    }
    let nets = vec!["- in0 ( PIN in0 ) ( fan A ) ;".to_string(), fanout + " ;"];
    Work { cells, nets }
}

/// What design repair reports of `work` under a fanout limit of three, and
/// how many loads net `fo` has after it.
fn repaired_under_fanout_limit(work: Work) -> (RepairSummary, usize) {
    let constraints = "create_clock -name core -period 1.0 [get_ports clk]\n\
                       set_input_delay 0.1 -clock core [get_ports in0]\n\
                       set_max_fanout 3 [current_design]\n";
    let (summary, db) = repaired_in_session(work, constraints, &cells_counting_fanout(), |r| {
        Repairer::new(r).repair_design().unwrap()
    });
    let fo = db.net_id("fo").unwrap();
    (summary, db.nets[fo].pins.len() - 1)
}

#[test]
fn design_repair_splits_a_net_over_its_fanout_limit_under_repeaters() {
    let places = [(0, 0), (300, 0), (600, 0), (0, 100), (300, 100), (600, 100)];

    let (summary, fo_loads) = repaired_under_fanout_limit(fanout_work(&places));

    assert_eq!(summary.fanout_violations, 1);
    assert!(summary.inserted >= 2, "{summary:#?}");
    assert!(fo_loads <= 3, "fo loads {fo_loads}");
}

#[test]
fn loads_stacked_within_a_micron_keep_their_fanout_violation() {
    let places = [(10, 10); 6];

    let (summary, fo_loads) = repaired_under_fanout_limit(fanout_work(&places));

    assert_eq!(summary.fanout_violations, 1);
    assert_eq!(fo_loads, 6);
}
