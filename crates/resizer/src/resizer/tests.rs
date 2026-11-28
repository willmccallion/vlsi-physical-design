use super::*;
use crate::testing::{
    DefSections, cells, cells_with, clock_cells_source, def_text, design, metal2_wires, pin,
    with_session_inputs, with_timed_session, with_wired_session,
};
use pare_common::db::global_route::RouteSource;
use pare_common::db::library::{CoreKind, MacroClass, SignalUse};
use std::fmt::Write;
use std::path::Path;

/// Buffers in the chain between the two flops of [`buffered_pair`].
const CHAIN: usize = 6;

/// Two flops on one clock, the first launching through buffers `b1` to
/// `b6` into the second.
fn buffered_pair() -> NetlistDB {
    let mut components = String::from("- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n");
    let mut nets =
        String::from("- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- n0 ( f1 Q ) ( b1 A ) ;\n");
    for i in 1..=CHAIN {
        writeln!(
            components,
            "- b{i} BUF_X1 + PLACED ( {} 2800 ) N ;",
            2000 + 8000 * i
        )
        .unwrap();
        let load = if i == CHAIN {
            "f2 D".to_string()
        } else {
            format!("b{} A", i + 1)
        };
        writeln!(nets, "- n{i} ( b{i} Z ) ( {load} ) ;").unwrap();
    }
    writeln!(
        components,
        "- f2 DFF_X1 + PLACED ( {} 2800 ) N ;",
        2000 + 8000 * (CHAIN + 1)
    )
    .unwrap();
    design(&format!(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN chain ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS {} ;\n{components}END COMPONENTS\n\
             PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS {} ;\n{nets}END NETS\nEND DESIGN\n",
        CHAIN + 2,
        CHAIN + 2
    ))
}

#[test]
fn a_buffer_put_before_a_chain_leaves_every_pin_on_the_path_its_slack() {
    let mut db = buffered_pair();
    let (b1_in, end) = (pin(&db, "b1/A"), pin(&db, "f2/D"));
    let q = db.pins[b1_in].net;
    let clock = "create_clock -name clk -period 1.0 [get_ports clk]\n";

    with_timed_session(&mut db, &cells(&clock_cells_source()), clock, |resizer| {
        let buffer = resizer.buffers.lowest_drive;
        let at = Point::new(Dbu::new(10_000), Dbu::new(2800));
        let mut first = b1_in;
        for _ in 0..CHAIN {
            first = resizer
                .insert_buffer_before_loads(q, &[first], buffer, at, "rebuffer")
                .unwrap()
                .input;
        }

        let sta = resizer.sta();
        let path_slack = sta.slack(end, MinMax::Max).time().unwrap();
        let netlist = resizer.netlist();
        let mut at_pin = first;
        while at_pin != end {
            let PinOwner::Cell { cell, .. } = resizer.db.pins[at_pin].owner else {
                unreachable!()
            };
            let out = resizer.db.cells[cell]
                .pins
                .iter()
                .copied()
                .find(|&p| netlist.is_driver(p))
                .unwrap();
            for p in [at_pin, out] {
                let slack = sta.slack(p, MinMax::Max).time().unwrap();
                assert!(
                    fuzzy::equal(slack, path_slack),
                    "{}: {slack:?} vs {path_slack:?}",
                    resizer.db.pin_name(p)
                );
            }
            at_pin = netlist.loads(resizer.db.pins[out].net).next().unwrap();
        }
    });
}

fn slacks(values: &[f64]) -> Vec<(NetId, Slack)> {
    values
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            let slack = if v.is_infinite() {
                Slack::Unconstrained
            } else {
                Slack::Constrained(Seconds::new(v))
            };
            (NetId::new(i), slack)
        })
        .collect()
}

#[test]
fn the_worst_tenth_of_the_nets_is_weighted_from_the_maximum_down_to_one() {
    let mut values = vec![1.0; 17];
    values[0] = -2.0;
    values[1] = -1.0;
    let weights = timing_net_weights(&slacks(&values), 5.0);

    assert_eq!(weights.len(), 1);
    assert_eq!(weights[0].0, NetId::new(0));
    assert!((weights[0].1 - 5.0).abs() < 1e-12);
}

#[test]
fn weights_fall_linearly_with_slack() {
    let mut values = vec![1.0; 30];
    values[0] = -3.0;
    values[1] = -2.0;
    values[2] = -1.0;
    let weights = timing_net_weights(&slacks(&values), 5.0);

    assert_eq!(weights.len(), 2);
    assert!((weights[0].1 - 5.0).abs() < 1e-12);
    assert!((weights[1].1 - 3.0).abs() < 1e-12);
}

#[test]
fn equal_slacks_and_infinite_ones_give_no_weights() {
    assert_eq!(timing_net_weights(&slacks(&[0.5, 0.5, 0.5]), 5.0).len(), 0);
    assert_eq!(
        timing_net_weights(&slacks(&[f64::INFINITY, f64::INFINITY]), 5.0).len(),
        0
    );
    assert_eq!(timing_net_weights(&[], 5.0).len(), 0);
}

#[test]
fn finite_slacks_take_the_maximum_weight_when_the_worst_tenth_reaches_infinity() {
    let mut values = vec![f64::INFINITY; 20];
    values[0] = -0.3;

    let weights = timing_net_weights(&slacks(&values), 5.0);

    assert_eq!(weights, vec![(NetId::new(0), 5.0)]);
}

const CLOCK: &str = "create_clock -name clk -period 1.0 [get_ports clk]\n";

/// Two flops with buffers `b1` and `b2` between them, the first
/// fed from a net inside a hierarchy.
fn two_buffers() -> NetlistDB {
    two_buffers_with_copies(&[])
}

/// [`two_buffers`] over the clock cells and a copy of the master `master`
/// called `name` for each `(master, name)` of `copies`.
fn two_buffers_with_copies(copies: &[(&str, &str)]) -> NetlistDB {
    crate::testing::design_with_copies(
        &def_text(
            "two_buffers",
            &DefSections {
                components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - b1 BUF_X1 + PLACED ( 20000 2800 ) N ;\n\
                             - b2 BUF_X1 + PLACED ( 40000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 60000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n\
                       - u1/q ( f1 Q ) ( b1 A ) ;\n- m ( b1 Z ) ( b2 A ) ;\n\
                       - d ( b2 Z ) ( f2 D ) ;\n",
                inputs: &["clk"],
                outputs: &[],
            },
        ),
        copies,
    )
}

fn def_of(db: &NetlistDB) -> String {
    let mut text = Vec::new();
    pare_common::db::writer::def::write(db, &mut text).unwrap();
    String::from_utf8(text).unwrap()
}

/// How a test session closes its journal.
#[derive(Clone, Copy)]
enum Close {
    Restore,
    End,
}

/// The DEF of [`two_buffers`] after a session that removes `b1`, which
/// renames nets, resizes `b2` and buffers `f2`, then closes the journal
/// as `close` says.
fn after_edits(close: Close) -> String {
    let mut db = two_buffers();
    let lib = cells(&clock_cells_source());
    let rc = metal2_wires(&db);
    with_session_inputs(&mut db, &lib, CLOCK, rc, |inputs, sdc| {
        let mut resizer = Resizer::timed(inputs, sdc).unwrap();
        let trial = resizer.journal_begin();
        let b1 = resizer.db.cell_id("b1").unwrap();
        let _ = resizer.remove_buffer(b1).unwrap();
        let b2 = resizer.db.cell_id("b2").unwrap();
        let clock_buffer = resizer.link.liberty.cell_id("CLKBUF_X3").unwrap();
        assert!(resizer.replace_cell(b2, clock_buffer).unwrap());
        let d = resizer.db.net_id("d").unwrap();
        let load = pin(resizer.db, "f2/D");
        let buffer = resizer.buffers.lowest_drive;
        let at = Point::new(Dbu::new(50_000), Dbu::new(2800));
        let _ = resizer
            .insert_buffer_before_loads(d, &[load], buffer, at, "rebuffer")
            .unwrap();
        match close {
            Close::Restore => resizer.journal_restore(trial).unwrap(),
            Close::End => resizer.journal_end(trial),
        }
        let _ = resizer.finish().unwrap();
    });
    def_of(&db)
}

#[test]
fn restoring_the_journal_leaves_the_design_as_it_was() {
    let before = def_of(&two_buffers());

    let after = after_edits(Close::Restore);

    assert_eq!(after, before);
}

#[test]
fn ending_the_journal_keeps_the_edits() {
    let before = def_of(&two_buffers());

    let after = after_edits(Close::End);

    assert_ne!(after, before);
    assert!(after.contains("- rebuffer1 BUF_X1") && after.contains("- b2 CLKBUF_X3"));
    assert!(!after.contains("- b1 BUF_X1"));
}

/// The error a timed session over [`two_buffers`] with the clock cells
/// of `source` fails to start with.
fn start_error(source: &str) -> Option<RepairError> {
    let mut db = two_buffers();
    let lib = cells(source);
    let rc = metal2_wires(&db);
    with_session_inputs(&mut db, &lib, CLOCK, rc, |inputs, sdc| {
        Resizer::timed(inputs, sdc).err()
    })
}

#[test]
fn slew_thresholds_derated_out_of_shape_refuse_a_session() {
    let derated = clock_cells_source().replace(
        "slew_derate_from_library : 1.0",
        "slew_derate_from_library : 100.0",
    );

    let error = start_error(&derated);

    assert!(
        matches!(error, Some(RepairError::ShapeFactor(_))),
        "{error:?}"
    );
    assert!(start_error(&clock_cells_source()).is_none());
}

/// A buffer cell the design's library has no master for.
const MASTERLESS_BUFFER: &str = "cell (BUF_X2) { area : 1.0 ; \
        pin (A) { direction : input ; capacitance : 1.0 ; } \
        pin (Z) { direction : output ; function : \"A\" ; } }";

#[test]
fn a_size_without_a_master_is_refused() {
    let mut db = two_buffers();
    let lib = cells_with(MASTERLESS_BUFFER);

    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        let b1 = resizer.db.cell_id("b1").unwrap();
        let size = resizer.link.liberty.cell_id("BUF_X2").unwrap();

        let error = resizer.replace_cell(b1, size);

        assert_eq!(error, Err(RepairError::NoMaster("BUF_X2".to_string())));
    });
}

#[test]
fn a_master_lacking_a_connected_pin_is_refused() {
    let mut db = two_buffers();
    let lib = cells(&clock_cells_source());

    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        let f1 = resizer.db.cell_id("f1").unwrap();
        let inverter = resizer.link.liberty.cell_id("INV_X1").unwrap();

        let error = resizer.replace_cell(f1, inverter);

        assert!(
            matches!(error, Err(RepairError::Edit(EditError::MissingPin { .. }))),
            "{error:?}"
        );
    });
}

/// The clock test cells with a library fanout limit of 6, `BUF_X1`'s
/// own of 4, and a fanout load of 2 at the flops' data inputs, 1 by
/// default elsewhere.
fn fanout_cells() -> Liberty {
    let source = clock_cells_source()
        .replacen(
            "default_max_transition : 0.2 ;",
            "default_max_transition : 0.2 ; default_max_fanout : 6 ; \
                 default_fanout_load : 1.0 ;",
            1,
        )
        .replacen(
            "function : \"A\" ;\n      max_capacitance : 60.0 ;",
            "function : \"A\" ;\n      max_fanout : 4 ;\n      max_capacitance : 60.0 ;",
            1,
        )
        .replace(
            "capacitance : 1.1 ;",
            "capacitance : 1.1 ; fanout_load : 2.0 ;",
        );
    cells(&source)
}

#[test]
fn a_drivers_fanout_limit_is_the_tightest_of_its_own_its_librarys_and_the_designs() {
    let lib = fanout_cells();
    let limits = |constraints: &str| {
        let mut db = two_buffers();
        let mut found = (None, None);
        with_timed_session(&mut db, &lib, constraints, |resizer| {
            let (flop, buffer) = (pin(resizer.db, "f1/Q"), pin(resizer.db, "b2/Z"));
            found = (resizer.fanout_limit(flop), resizer.fanout_limit(buffer));
        });
        found
    };

    assert_eq!(limits(CLOCK), (Some(6.0), Some(4.0)));
    assert_eq!(
        limits(&format!("{CLOCK}set_max_fanout 5 [current_design]\n")),
        (Some(5.0), Some(4.0))
    );
}

#[test]
fn a_nets_fanout_sums_its_loads_fanout_loads() {
    let mut db = two_buffers();
    let lib = fanout_cells();

    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        let into_flop = resizer.fanout_load(pin(resizer.db, "b2/Z"));
        let into_buffer = resizer.fanout_load(pin(resizer.db, "f1/Q"));

        assert!((into_flop - 2.0).abs() < 1e-12, "{into_flop}");
        assert!((into_buffer - 1.0).abs() < 1e-12, "{into_buffer}");
    });
}

#[test]
fn a_cell_that_is_not_a_buffer_has_no_buffer_pins() {
    let mut db = two_buffers();
    let lib = cells(&clock_cells_source());

    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        let and_gate = resizer.link.liberty.cell_id("AND2_X1").unwrap();
        let f1 = resizer.db.cell_id("f1").unwrap();

        assert_eq!(
            resizer.as_buffer(and_gate),
            Err(RepairError::MissingPins("AND2_X1".to_string()))
        );
        assert_eq!(
            resizer.remove_buffer(f1),
            Err(RepairError::MissingPins("f1".to_string()))
        );
    });
}

/// Every original pin's late and early slack and late slews.
fn timing_of(resizer: &Resizer<'_, Sta>, pins: usize) -> Vec<(String, Slack, Slack, f64)> {
    let sta = resizer.sta();
    (0..pins)
        .map(PinId::new)
        .map(|p| {
            (
                resizer.db.pin_name(p),
                sta.slack(p, MinMax::Max),
                sta.slack(p, MinMax::Min),
                resizer.graph.slew(p).max().get(),
            )
        })
        .collect()
}

/// The pins whose timing differs between `before` and `after`.
fn timing_diffs(
    before: &[(String, Slack, Slack, f64)],
    after: &[(String, Slack, Slack, f64)],
) -> Vec<String> {
    before
        .iter()
        .zip(after)
        .filter(|(b, a)| b != a)
        .map(|(b, a)| format!("{b:?} -> {a:?}"))
        .collect()
}

#[test]
fn restoring_the_journal_restores_the_timing_of_removals_resizes_and_insertions() {
    let mut db = two_buffers();
    let lib = cells(&clock_cells_source());
    let rc = metal2_wires(&db);
    with_session_inputs(&mut db, &lib, CLOCK, rc, |inputs, sdc| {
        let mut resizer = Resizer::timed(inputs, sdc).unwrap();
        let pins = resizer.db.pins.len();
        let before = timing_of(&resizer, pins);
        let trial = resizer.journal_begin();
        let b1 = resizer.db.cell_id("b1").unwrap();
        let _ = resizer.remove_buffer(b1).unwrap();
        let b2 = resizer.db.cell_id("b2").unwrap();
        let clock_buffer = resizer.link.liberty.cell_id("CLKBUF_X3").unwrap();
        assert!(resizer.replace_cell(b2, clock_buffer).unwrap());
        let d = resizer.db.net_id("d").unwrap();
        let load = pin(resizer.db, "f2/D");
        let buffer = resizer.buffers.lowest_drive;
        let at = Point::new(Dbu::new(50_000), Dbu::new(2800));
        let _ = resizer
            .insert_buffer_before_loads(d, &[load], buffer, at, "rebuffer")
            .unwrap();

        resizer.journal_restore(trial).unwrap();

        let diffs = timing_diffs(&before, &timing_of(&resizer, pins));
        assert!(diffs.is_empty(), "{}", diffs.join("\n"));
    });
}

/// Two flops feeding an and gate `g` that drives two more.
fn gate_fanout() -> NetlistDB {
    design(&def_text(
        "gate",
        &DefSections {
            components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 2000 30000 ) N ;\n\
                             - g AND2_X1 + PLACED ( 40000 16000 ) N ;\n\
                             - f3 DFF_X1 + PLACED ( 80000 2800 ) N ;\n\
                             - f4 DFF_X1 + PLACED ( 80000 30000 ) N ;\n",
            nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ( f3 CK ) ( f4 CK ) ;\n\
                       - a ( f1 Q ) ( g A ) ;\n- b ( f2 Q ) ( g B ) ;\n\
                       - z ( g Z ) ( f3 D ) ( f4 D ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

#[test]
fn restoring_the_journal_restores_the_timing_of_a_pin_swap_and_a_clone() {
    let mut db = gate_fanout();
    let lib = cells(&clock_cells_source());
    let rc = metal2_wires(&db);
    with_session_inputs(&mut db, &lib, CLOCK, rc, |inputs, sdc| {
        let mut resizer = Resizer::timed(inputs, sdc).unwrap();
        let pins = resizer.db.pins.len();
        let before = timing_of(&resizer, pins);
        let (a1, a2) = (pin(resizer.db, "g/A"), pin(resizer.db, "g/B"));
        let (net_a, net_b) = (resizer.db.pins[a1].net, resizer.db.pins[a2].net);
        let trial = resizer.journal_begin();
        resizer.move_pin(a1, net_b);
        resizer.move_pin(a2, net_a);
        resizer.update_net(net_a);
        resizer.update_net(net_b);
        let and2 = resizer.link.liberty.cell_id("AND2_X1").unwrap();
        let and_cell = resizer.link.liberty.cell(and2);
        let at = Point::new(Dbu::new(60_000), Dbu::new(30_000));
        let clone = resizer.make_instance(and2, at, "clone").unwrap();
        let z = resizer.db.net_id("z").unwrap();
        let cloned_out = resizer.make_net("net", z).unwrap();
        for (port, net) in [("A", net_a), ("B", net_b), ("Z", cloned_out)] {
            let port = and_cell.port(port).unwrap();
            let _ = resizer.connect(clone, and2, port, net).unwrap();
        }
        resizer.move_pin(pin(resizer.db, "f4/D"), cloned_out);
        for net in [net_a, net_b, z, cloned_out] {
            resizer.update_net(net);
        }

        resizer.journal_restore(trial).unwrap();

        let diffs = timing_diffs(&before, &timing_of(&resizer, pins));
        assert!(diffs.is_empty(), "{}", diffs.join("\n"));
    });
}

/// The pins whose slack differs, beyond tolerance, between the session's
/// incrementally kept timing and timing found afresh.
fn stale_slacks(resizer: &Resizer<'_, Sta>) -> Vec<String> {
    let netlist = resizer.netlist();
    let fresh = Sta::new(
        &netlist,
        &resizer.graph,
        resizer.clocks,
        resizer.constants,
        resizer.sta().sdc(),
    );
    let same = |a: Slack, b: Slack| {
        a == b || matches!((a.time(), b.time()), (Some(a), Some(b)) if fuzzy::equal(a, b))
    };
    resizer
        .db
        .pins
        .ids()
        .filter(|&p| !netlist.is_retired(p))
        .flat_map(|p| MinMax::BOTH.map(|side| (p, side)))
        .filter_map(|(p, side)| {
            let (kept, found) = (resizer.sta().slack(p, side), fresh.slack(p, side));
            (!same(kept, found))
                .then(|| format!("{} {side:?}: {kept:?} vs {found:?}", resizer.db.pin_name(p)))
        })
        .collect()
}

/// One random edit of the chain of [`buffered_pair`]: a buffer resized,
/// removed or put in front of a load.
fn random_edit(resizer: &mut Resizer<'_, Sta>, draw: &mut impl FnMut(usize) -> usize) {
    let buffers: Vec<CellId> = resizer
        .db
        .cells
        .ids()
        .filter(|&c| !resizer.db.cells[c].retired && resizer.buffer_pins(c).is_some())
        .collect();
    let cell = buffers[draw(buffers.len())];
    match draw(3) {
        0 => {
            let sizes = ["BUF_X1", "CLKBUF_X3"];
            let size = resizer.link.liberty.cell_id(sizes[draw(2)]).unwrap();
            let _ = resizer.replace_cell(cell, size).unwrap();
        }
        1 if buffers.len() > 1 => {
            let _ = resizer.remove_buffer(cell).unwrap();
        }
        _ => {
            let (_, out) = resizer.buffer_pins(cell).unwrap();
            let net = resizer.db.pins[out].net;
            let loads: Vec<PinId> = resizer.netlist().loads(net).collect();
            let at = Point::new(
                Dbu::new(2000 + 4000 * i64::try_from(draw(18)).unwrap()),
                Dbu::new(2800),
            );
            let buffer = resizer.buffers.lowest_drive;
            let _ = resizer
                .insert_buffer_before_loads(net, &loads, buffer, at, "rebuffer")
                .unwrap();
        }
    }
}

#[test]
fn timing_kept_through_random_edits_matches_timing_found_afresh() {
    let mut db = buffered_pair();
    let lib = cells(&clock_cells_source());
    let rc = metal2_wires(&db);
    let mut state: u64 = 0x5eed;
    let mut draw = |n: usize| {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (state >> 33) as usize % n
    };
    with_session_inputs(&mut db, &lib, CLOCK, rc, |inputs, sdc| {
        let mut resizer = Resizer::timed(inputs, sdc).unwrap();
        for step in 0..40 {
            random_edit(&mut resizer, &mut draw);

            let stale = stale_slacks(&resizer);

            assert!(stale.is_empty(), "after edit {step}:\n{}", stale.join("\n"));
        }
    });
}

#[test]
fn removing_a_buffer_gives_the_surviving_net_the_shallower_name_of_the_two() {
    let mut db = two_buffers();
    let driver = pin(&db, "f1/Q");
    let lib = cells(&clock_cells_source());

    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        let b1 = resizer.db.cell_id("b1").unwrap();

        let survivor = resizer.remove_buffer(b1).unwrap();

        assert_eq!(resizer.db.pins[driver].net, survivor);
        assert_eq!(resizer.db.nets[survivor].name, "m");
    });
}

#[test]
fn removing_a_buffer_between_nets_of_one_depth_keeps_the_survivors_name() {
    let mut db = two_buffers();
    let lib = cells(&clock_cells_source());

    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        let b2 = resizer.db.cell_id("b2").unwrap();

        let survivor = resizer.remove_buffer(b2).unwrap();

        assert_eq!(resizer.db.nets[survivor].name, "m");
    });
}

#[test]
fn moving_a_load_and_updating_its_net_reestimates_the_wire_its_driver_sees() {
    let mut db = two_buffers();
    let (driver, net) = (pin(&db, "b1/Z"), db.net_id("m").unwrap());

    with_wired_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let before = resizer.load_cap(driver);
        let b2 = resizer.db.cell_id("b2").unwrap();
        resizer.db.cells[b2].location = Point::new(Dbu::new(70_000), Dbu::new(2800));
        assert_eq!(resizer.load_cap(driver), before);

        resizer.update_net(net);

        assert!(resizer.load_cap(driver) > before);
    });
}

#[test]
fn updating_a_moved_cell_reestimates_the_nets_on_its_pins() {
    let mut db = two_buffers();
    let (before_driver, after_driver) = (pin(&db, "b1/Z"), pin(&db, "b2/Z"));

    with_wired_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let (into, out_of) = (
            resizer.load_cap(before_driver),
            resizer.load_cap(after_driver),
        );
        let b2 = resizer.db.cell_id("b2").unwrap();
        resizer.db.cells[b2].location = Point::new(Dbu::new(70_000), Dbu::new(2800));

        resizer.update_cell(b2);

        assert!(resizer.load_cap(before_driver) > into);
        assert_ne!(resizer.load_cap(after_driver), out_of);
    });
}

/// A route source that routes nothing, so that parasitics are estimated as
/// without routes while a session counts as following them.
struct NoRoutes;

impl RouteSource for NoRoutes {
    fn update(&mut self, _: &NetlistDB, _: &[NetId]) {}

    fn begin_trial(&mut self) {}

    fn keep_trial(&mut self) {}

    fn restore_trial(&mut self) {}

    fn route(&self, _: &NetlistDB, _: NetId) -> Option<pare_common::db::global_route::NetRoute> {
        None
    }
}

/// Where `legalize` moves cells in [`legalized_location_after`].
const LEGAL_SITE: Point<Dbu> = Point::new(Dbu::new(3000), Dbu::new(5600));

/// Where `b2` stands after a session on global routes, whose legalizing
/// puts cells at [`LEGAL_SITE`], resizes it, and closes the trial as `close`
/// says.
fn legalized_location_after(close: Close) -> (Point<Dbu>, Point<Dbu>) {
    let mut db = two_buffers();
    let lib = cells(&clock_cells_source());
    let rc = metal2_wires(&db);
    let routing = crate::route_parasitics::RouteRc::new(
        &db.library,
        &[],
        &[],
        pare_common::db::indices::LayerId(0),
    );
    let mut found = None;
    with_session_inputs(&mut db, &lib, CLOCK, rc, |inputs, sdc| {
        let mut source = NoRoutes;
        let routes = GlobalRouting {
            source: &mut source,
            rc: routing,
            legalize: &|db, cell| db.cells[cell].location = LEGAL_SITE,
        };
        let mut resizer = Resizer::on_global_routes(inputs, sdc, routes).unwrap();
        let b2 = resizer.db.cell_id("b2").unwrap();
        let original = resizer.db.cells[b2].location;
        let trial = resizer.journal_begin();
        let clock_buffer = resizer.link.liberty.cell_id("CLKBUF_X3").unwrap();

        assert!(resizer.replace_cell(b2, clock_buffer).unwrap());

        let moved = resizer.db.cells[b2].location;
        match close {
            Close::Restore => resizer.journal_restore(trial).unwrap(),
            Close::End => resizer.journal_end(trial),
        }
        found = Some((original, moved, resizer.db.cells[b2].location));
    });
    let (original, moved, after) = found.unwrap();
    assert_eq!(moved, LEGAL_SITE);
    (original, after)
}

#[test]
fn a_cell_legalized_after_a_resize_goes_back_where_it_was_with_the_trial() {
    let (original, restored) = legalized_location_after(Close::Restore);

    assert_eq!(restored, original);
}

#[test]
fn a_cell_legalized_after_a_resize_stays_legal_when_the_trial_is_kept() {
    let (original, kept) = legalized_location_after(Close::End);

    assert_ne!(original, LEGAL_SITE);
    assert_eq!(kept, LEGAL_SITE);
}

/// A buffer cell of `name` taking `delay_ns` at any load.
fn scalar_buffer(name: &str, delay_ns: f64) -> String {
    format!(
        "cell ({name}) {{ area : 0.8 ;
          pin (A) {{ direction : input ; capacitance : 0.97 ; }}
          pin (Z) {{ direction : output ; function : \"A\" ; max_capacitance : 60.0 ;
            timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
              cell_rise (scalar) {{ values (\"{delay_ns}\") ; }}
              cell_fall (scalar) {{ values (\"{delay_ns}\") ; }}
              rise_transition (scalar) {{ values (\"0.01\") ; }}
              fall_transition (scalar) {{ values (\"0.01\") ; }} }} }} }}"
    )
}

/// The clock cells' cell `of`, as a cell called `name`.
fn renamed_cell(of: &str, name: &str) -> String {
    let source = clock_cells_source();
    let start = source.find(&format!("cell ({of})")).unwrap();
    let end = source[start + 1..]
        .find("  cell (")
        .map_or_else(|| source.rfind('}').unwrap(), |at| start + 1 + at);
    source[start..end].replace(of, name)
}

/// The size of `cell` of `session` that `find_target_cell` gives for `load`
/// when the cells named in `targets` meet the target slews at the loads
/// given in femtofarads and every other cell is out of reach.
fn target_size(
    extra: &str,
    copies: &[(&str, &str)],
    targets: &[(&str, f64)],
    start: &str,
    (load_ff, revisiting): (f64, bool),
) -> String {
    let mut db = crate::testing::design_with_copies(
        &def_text(
            "family",
            &DefSections {
                components: "- b BUF_X1 + PLACED ( 20000 2800 ) N ;\n\
                             - g AND2_X1 + PLACED ( 40000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( g A ) ( b A ) ;\n",
                inputs: &["clk"],
                outputs: &[],
            },
        ),
        copies,
    );
    let mut size = String::new();
    with_timed_session(&mut db, &cells_with(extra), CLOCK, |resizer| {
        for load in &mut resizer.targets.load {
            *load = Farads::new(1e-6);
        }
        for &(name, femtofarads) in targets {
            let id = resizer.link.liberty.cell_id(name).unwrap();
            resizer.targets.load[id] = Farads::new(femtofarads * 1e-15);
        }
        let from = resizer.link.liberty.cell_id(start).unwrap();
        let found = resizer.find_target_cell(from, Farads::new(load_ff * 1e-15), revisiting);
        size = resizer.link.liberty.cell(found).name.clone();
    });
    size
}

/// Two buffers `BUF_A` and `BUF_B` of delays `a` and `b` nanoseconds, the
/// size of `BUF_A` that `find_target_cell` gives for a load of 10 fF when
/// their target loads are `targets` femtofarads.
fn buffer_size_for_ten_femtofarads(a: f64, b: f64, targets: (f64, f64)) -> String {
    let extra = format!(
        "{}\n{}",
        scalar_buffer("BUF_A", a),
        scalar_buffer("BUF_B", b)
    );
    target_size(
        &extra,
        &[("BUF_X1", "BUF_A"), ("BUF_X1", "BUF_B")],
        &[("BUF_A", targets.0), ("BUF_B", targets.1)],
        "BUF_A",
        (10.0, false),
    )
}

#[test]
fn a_buffer_size_equal_to_the_current_one_does_not_displace_it() {
    assert_eq!(
        buffer_size_for_ten_femtofarads(0.20, 0.20, (8.0, 8.0)),
        "BUF_A"
    );
}

#[test]
fn a_faster_buffer_a_little_farther_from_the_load_replaces_the_current_one() {
    assert_eq!(
        buffer_size_for_ten_femtofarads(0.20, 0.15, (8.0, 12.1)),
        "BUF_B"
    );
}

#[test]
fn a_buffer_nearer_to_the_load_and_a_little_slower_replaces_the_current_one() {
    assert_eq!(
        buffer_size_for_ten_femtofarads(0.20, 0.21, (8.0, 9.0)),
        "BUF_B"
    );
}

#[test]
fn a_faster_buffer_farther_than_the_current_one_by_more_than_a_tenth_does_not_replace_it() {
    assert_eq!(
        buffer_size_for_ten_femtofarads(0.20, 0.15, (10.0, 12.0)),
        "BUF_A"
    );
    assert_eq!(
        buffer_size_for_ten_femtofarads(0.20, 0.15, (8.0, 12.5)),
        "BUF_A"
    );
}

#[test]
fn a_nearer_buffer_slower_by_more_than_a_tenth_does_not_replace_the_current_one() {
    assert_eq!(
        buffer_size_for_ten_femtofarads(0.20, 0.23, (8.0, 9.0)),
        "BUF_A"
    );
}

/// Two copies `GATE_A` and `GATE_B` of the and gate, the size of `GATE_A`
/// that `find_target_cell` gives for a load of 10 fF when their target loads
/// are `targets` femtofarads, as sizes of a cell that is not a buffer.
fn gate_size_for_ten_femtofarads(targets: (f64, f64), revisiting: bool) -> String {
    let extra = format!(
        "{}\n{}",
        renamed_cell("AND2_X1", "GATE_A"),
        renamed_cell("AND2_X1", "GATE_B")
    );
    target_size(
        &extra,
        &[("AND2_X1", "GATE_A"), ("AND2_X1", "GATE_B")],
        &[("GATE_A", targets.0), ("GATE_B", targets.1)],
        "GATE_A",
        (10.0, revisiting),
    )
}

#[test]
fn a_gate_size_equal_to_the_current_one_does_not_displace_it() {
    assert_eq!(gate_size_for_ten_femtofarads((8.0, 8.0), false), "GATE_A");
}

#[test]
fn a_gate_size_nearer_to_the_load_replaces_the_current_one_and_a_farther_one_does_not() {
    assert_eq!(gate_size_for_ten_femtofarads((8.0, 9.0), false), "GATE_B");
    assert_eq!(gate_size_for_ten_femtofarads((9.0, 14.0), false), "GATE_A");
}

#[test]
fn a_nearer_gate_size_with_a_smaller_target_replaces_the_current_one_unless_revisiting() {
    assert_eq!(gate_size_for_ten_femtofarads((12.0, 9.5), false), "GATE_B");
    assert_eq!(gate_size_for_ten_femtofarads((12.0, 9.5), true), "GATE_A");
}

#[test]
fn a_nearer_gate_size_with_a_larger_target_replaces_the_current_one_when_revisiting() {
    assert_eq!(gate_size_for_ten_femtofarads((8.0, 9.0), true), "GATE_B");
}

fn femtofarads(value: f64) -> Farads {
    Farads::new(value * 1e-15)
}

/// The decision of `cap_ok` for `f1/Q` of [`two_buffers`], which loads
/// `b1`'s input of 0.97 fF, taking on `delta_ff` femtofarads under
/// `constraints`.
fn flop_output_takes_on(constraints: &str, delta_ff: f64) -> bool {
    let mut db = two_buffers();
    let driver = pin(&db, "f1/Q");
    let mut allowed = false;
    with_timed_session(
        &mut db,
        &cells(&clock_cells_source()),
        constraints,
        |resizer| {
            allowed = resizer.cap_ok(driver, femtofarads(delta_ff));
        },
    );
    allowed
}

#[test]
fn a_driver_takes_on_load_up_to_its_limit_and_no_more() {
    let limited = format!("{CLOCK}set_max_capacitance 1.2 [current_design]\n");

    assert!(flop_output_takes_on(&limited, 0.2));
    assert!(!flop_output_takes_on(&limited, 0.3));
    assert!(!flop_output_takes_on(CLOCK, 100.0));
    assert!(flop_output_takes_on(CLOCK, 20.0));
}

#[test]
fn a_driver_over_its_limit_may_not_take_on_more_but_may_keep_or_shed_load() {
    let over = format!("{CLOCK}set_max_capacitance 0.5 [current_design]\n");

    assert!(flop_output_takes_on(&over, 0.0));
    assert!(flop_output_takes_on(&over, -0.1));
    assert!(!flop_output_takes_on(&over, 0.1));
}

#[test]
fn a_driver_with_a_load_limit_of_zero_has_none() {
    let mut db = two_buffers();
    let driver = pin(&db, "b1/Z");
    let lib = cells_with_buffer_edit("max_capacitance : 60.0", "max_capacitance : 0.0");

    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        assert!(resizer.cap_ok(driver, femtofarads(100.0)));
    });
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

/// Whether giving `b1` of [`two_buffers`] the master `replacement` keeps the
/// load of the flop driving it within `limit_ff` femtofarads, over the clock
/// cells and `extra`.
fn flop_keeps_load_through(replacement: &str, extra: &str, limit_ff: f64) -> bool {
    let mut db = two_buffers();
    let constraints = format!("{CLOCK}set_max_capacitance {limit_ff} [current_design]\n");
    let mut keeps = false;
    with_timed_session(&mut db, &cells_with(extra), &constraints, |resizer| {
        let b1 = resizer.db.cell_id("b1").unwrap();
        let size = resizer.link.liberty.cell_id(replacement).unwrap();
        keeps = resizer.replacement_keeps_loads(b1, size);
    });
    keeps
}

#[test]
fn a_cell_with_a_larger_input_is_kept_only_where_its_driver_can_take_the_difference() {
    assert!(!flop_keeps_load_through("CLKBUF_X3", "", 1.2));
    assert!(flop_keeps_load_through("CLKBUF_X3", "", 1.5));
}

#[test]
fn a_cell_with_a_smaller_input_is_always_kept() {
    let smaller =
        scalar_buffer("BUF_SMALL", 0.1).replace("capacitance : 0.97", "capacitance : 0.5");

    assert!(flop_keeps_load_through("BUF_SMALL", &smaller, 1.0));
}

#[test]
fn the_slew_shape_factor_follows_the_thresholds_and_the_derate_with_the_pessimism_added() {
    let factor = |derate: &str| {
        let source = clock_cells_source().replace(
            "slew_derate_from_library : 1.0",
            &format!("slew_derate_from_library : {derate}"),
        );
        let (db, lib) = (two_buffers(), cells(&source));
        let link = Link::new(&db, &lib, &[]).unwrap();
        slew_rc_factor(&link).unwrap()
    };

    let unit = (7.0_f64 / 3.0).ln() * (1.0 + SLEW_RC_PESSIMISM);

    assert!((factor("1.0") - unit).abs() < 1e-12);
    assert!((factor("0.5") / unit - 2.0).abs() < 1e-12);
}

#[test]
fn the_area_change_is_the_percentage_over_the_area_before_and_zero_without_one() {
    let area = CellArea::new;

    assert!((area_change_percent(area(4.0), area(5.0)) - 25.0).abs() < 1e-12);
    assert!((area_change_percent(area(4.0), area(2.0)) + 50.0).abs() < 1e-12);
    assert!(area_change_percent(area(0.0), area(5.0)).abs() < f64::MIN_POSITIVE);
}

#[test]
fn a_load_limit_is_margined_down_by_the_cap_margin_percentage() {
    let mut db = two_buffers();

    with_timed_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        resizer.options.cap_margin = 20.0;

        let margined = resizer.margined_cap(femtofarads(10.0));

        assert!((margined.get() - 8e-15).abs() < 1e-27, "{margined:?}");
    });
}

#[test]
fn a_fanout_limit_of_zero_limits_nothing() {
    let mut db = two_buffers();
    let driver = pin(&db, "f1/Q");
    let constraints = format!("{CLOCK}set_max_fanout 0 [current_design]\n");

    with_timed_session(
        &mut db,
        &cells(&clock_cells_source()),
        &constraints,
        |resizer| {
            assert_eq!(resizer.fanout_limit(driver), None);
        },
    );
}

/// An and cell reshaped as a buffer `A` to `Z` enabled by `B` being low.
const TRISTATE_BUFFER: &str = "cell (TRI) { area : 1.0 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (B) { direction : input ; capacitance : 1.0 ; }
        pin (Z) { direction : output ; function : \"A\" ; three_state : \"B\" ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } } } }";

/// A tristate buffer `t` between two flops, and the plain buffer `b` after
/// the second.
fn tristate_and_plain_drivers() -> NetlistDB {
    crate::testing::design_with_copies(
        &def_text(
            "tri",
            &DefSections {
                components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - t TRI + PLACED ( 12000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 22000 2800 ) N ;\n\
                             - b BUF_X1 + PLACED ( 32000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( t A ) ;\n\
                       - en ( PIN en ) ( t B ) ;\n- r ( t Z ) ( f2 D ) ;\n\
                       - s ( f2 Q ) ( b A ) ;\n- out ( b Z ) ( PIN out ) ;\n",
                inputs: &["clk", "en"],
                outputs: &["out"],
            },
        ),
        &[("AND2_X1", "TRI")],
    )
}

#[test]
fn only_a_tristate_driver_is_one_and_only_a_plain_driver_on_a_live_net_may_be_buffered() {
    let mut db = tristate_and_plain_drivers();
    let (tristate, plain, flop) = (pin(&db, "t/Z"), pin(&db, "b/Z"), pin(&db, "f2/Q"));
    let supply = db.net_id("s").unwrap();
    db.nets[supply].signal_use = SignalUse::Power;

    with_timed_session(&mut db, &cells_with(TRISTATE_BUFFER), CLOCK, |resizer| {
        assert!(resizer.is_tristate_driver(tristate));
        assert!(!resizer.is_tristate_driver(plain));
        assert!(!resizer.ok_to_buffer_net(tristate));
        assert!(resizer.ok_to_buffer_net(plain));
        assert!(!resizer.ok_to_buffer_net(flop));
    });
}

/// An instance of the master `master` of the clock cells made as `class`.
fn design_with_class(master: &str, class: MacroClass) -> NetlistDB {
    let mut db = crate::testing::design_with_copies(
        &def_text(
            "classed",
            &DefSections {
                components: "- c FOO + PLACED ( 12000 2800 ) N ;\n",
                nets: "- in ( PIN in ) ( c A ) ;\n- out ( c Z ) ( PIN out ) ;\n",
                inputs: &["in"],
                outputs: &["out"],
            },
        ),
        &[(master, "FOO")],
    );
    let foo = db.library.macro_id("FOO").unwrap();
    db.library.macros[foo].class = class;
    db
}

#[test]
fn a_regular_core_cell_the_library_does_not_protect_may_be_edited_and_no_other() {
    let editable = |class: MacroClass, attributes: &str| {
        let mut db = design_with_class("BUF_X1", class);
        let foo = scalar_buffer("FOO", 0.1)
            .replace("area : 0.8 ;", &format!("area : 0.8 ; {attributes}"));
        let mut allowed = false;
        with_timed_session(&mut db, &cells_with(&foo), "", |resizer| {
            allowed = resizer.is_editable(resizer.db.cell_id("c").unwrap());
        });
        allowed
    };
    let regular = MacroClass::Core(CoreKind::Regular);

    assert!(editable(regular, ""));
    assert!(!editable(regular, "dont_touch : true ;"));
    assert!(!editable(MacroClass::Core(CoreKind::Spacer), ""));
}

#[test]
fn the_slews_at_a_cells_pins_are_read_by_port_name() {
    let mut db = two_buffers();
    let pins = ["b2/A", "b2/Z"].map(|name| pin(&db, name));

    with_timed_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let b2 = resizer.db.cell_id("b2").unwrap();

        let slews = resizer.input_slews(b2);

        assert_eq!(slews.len(), 2);
        assert_eq!(slews["A"], resizer.graph.slew(pins[0]));
        assert_eq!(slews["Z"], resizer.graph.slew(pins[1]));
        assert!(slews["A"].max() > Seconds::ZERO);
    });
}

#[test]
fn an_undo_notes_the_nets_on_the_pins_of_a_cell_it_touched() {
    let db = two_buffers();
    let b1 = db.cell_id("b1").unwrap();
    let mut touched = Touched::default();

    touched.add_nets_of(&db, b1);

    let expected: FxHashSet<NetId> = ["u1/q", "m"]
        .iter()
        .map(|name| db.net_id(name).unwrap())
        .collect();
    assert_eq!(touched.nets, expected);
}

/// A cell that holds its output high.
const TIE_HIGH: &str = "cell (TIEH) { area : 0.5 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (Z) { direction : output ; function : \"1\" ; } }";

#[test]
fn the_net_slacks_for_placement_leave_out_clock_constant_and_supply_nets() {
    let mut db = crate::testing::design_with_copies(
        &def_text(
            "slacks",
            &DefSections {
                components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 20000 2800 ) N ;\n\
                             - t TIEH + PLACED ( 2000 20000 ) N ;\n\
                             - b BUF_X1 + PLACED ( 40000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- data ( f1 Q ) ( f2 D ) ;\n\
                       - high ( t Z ) ( f1 D ) ;\n- supply ( f2 Q ) ( b A ) ;\n\
                       - out ( b Z ) ( PIN out ) ;\n",
                inputs: &["clk"],
                outputs: &["out"],
            },
        ),
        &[("BUF_X1", "TIEH")],
    );
    let net = |name: &str| db.net_id(name).unwrap();
    let (data, out) = (net("data"), net("out"));
    let supply = net("supply");
    db.nets[supply].signal_use = SignalUse::Power;

    with_timed_session(&mut db, &cells_with(TIE_HIGH), CLOCK, |resizer| {
        let timed: Vec<NetId> = resizer
            .driver_net_slacks()
            .into_iter()
            .map(|(net, _)| net)
            .collect();

        assert_eq!(timed.len(), 2, "{timed:?}");
        assert!(timed.contains(&data) && timed.contains(&out));
    });
}

/// A buffer whose output slews are 0.2 ns rising and 0.1 ns falling.
const SLEW_CELL: &str = "cell (SLEWS) { area : 0.8 ;
        pin (A) { direction : input ; capacitance : 0.97 ; }
        pin (Z) { direction : output ; function : \"A\" ; max_capacitance : 60.0 ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.05\") ; }
            cell_fall (scalar) { values (\"0.05\") ; }
            rise_transition (scalar) { values (\"0.2\") ; }
            fall_transition (scalar) { values (\"0.1\") ; } } } }";

#[test]
fn the_slew_a_gate_gives_is_the_worst_of_its_edges() {
    let lib = cells_with(SLEW_CELL);
    let cell = lib.cell(lib.cell_id("SLEWS").unwrap());
    let port = cell.port("Z").unwrap();
    let slews = EdgeValues::both(Seconds::new(0.01e-9));

    let slew = gate_slew(cell, port, &slews, femtofarads(5.0));

    assert!((slew.get() - 0.2e-9).abs() < 1e-18, "{slew:?}");
}

/// An input `in` into a cell `m` of master `master` that drives a buffer
/// from each of its outputs `B` and `Z`.
fn cell_with_two_outputs() -> NetlistDB {
    crate::testing::design_with_copies(
        &def_text(
            "two_outputs",
            &DefSections {
                components: "- m AND2_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - s1 BUF_X1 + PLACED ( 12000 2800 ) N ;\n\
                             - s2 BUF_X1 + PLACED ( 12000 20000 ) N ;\n",
                nets: "- in ( PIN in ) ( m A ) ;\n- o1 ( m B ) ( s1 A ) ;\n\
                       - o2 ( m Z ) ( s2 A ) ;\n",
                inputs: &["in"],
                outputs: &[],
            },
        ),
        &[("AND2_X1", "MO_A"), ("AND2_X1", "MO_B")],
    )
}

/// A cell of `name` with an input `A` and the outputs `B` and `Z`.
fn two_output_cell(name: &str) -> String {
    format!(
        "cell ({name}) {{ area : 1.0 ;
          pin (A) {{ direction : input ; capacitance : 1.0 ; }}
          pin (B) {{ direction : output ; function : \"A\" ;
            timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
              cell_rise (scalar) {{ values (\"0.05\") ; }}
              cell_fall (scalar) {{ values (\"0.05\") ; }} }} }}
          pin (Z) {{ direction : output ; function : \"A\" ;
            timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
              cell_rise (scalar) {{ values (\"0.05\") ; }}
              cell_fall (scalar) {{ values (\"0.05\") ; }} }} }} }}"
    )
}

/// Resizes the output `Z` of `m` in [`cell_with_two_outputs`] whose master is
/// `MO_A` towards `MO_B`, which the target loads favour, after the cell was
/// resized already when `after_a_resize`; returns whether that reports a
/// change, and the master that results.
fn resize_a_two_output_cell(after_a_resize: bool) -> (bool, String) {
    let mut db = cell_with_two_outputs();
    let m = db.cell_id("m").unwrap();
    let a = db.library.macro_id("MO_A").unwrap();
    db.cells[m].macro_id = a;
    let driver = pin(&db, "m/Z");
    let extra = format!("{}\n{}", two_output_cell("MO_A"), two_output_cell("MO_B"));
    let mut found = (false, String::new());
    with_timed_session(&mut db, &cells_with(&extra), "", |resizer| {
        let load = resizer.load_cap(driver);
        for target in &mut resizer.targets.load {
            *target = Farads::new(1e-6);
        }
        let by_name = |name: &str| resizer.link.liberty.cell_id(name).unwrap();
        resizer.targets.load[by_name("MO_A")] = load / 5.0;
        resizer.targets.load[by_name("MO_B")] = load;
        if after_a_resize {
            let _ = resizer.resized_multi_output.insert(m);
        }

        let changed = resizer.resize_to_target_slew(driver).unwrap();

        let master = resizer.link.cell(resizer.db, m).unwrap().1.name.clone();
        found = (changed, master);
    });
    found
}

#[test]
fn a_two_output_cell_resized_for_the_first_time_reports_the_change() {
    assert_eq!(resize_a_two_output_cell(false), (true, "MO_B".to_string()));
}

#[test]
fn a_two_output_cell_resized_again_for_its_other_output_does_not_report_a_change() {
    assert_eq!(resize_a_two_output_cell(true), (false, "MO_B".to_string()));
}

#[test]
fn a_one_output_cell_resized_again_still_reports_the_change() {
    let mut db = two_buffers_with_copies(&[("BUF_X1", "BUF_A"), ("BUF_X1", "BUF_B")]);
    let b1 = db.cell_id("b1").unwrap();
    let driver = pin(&db, "b1/Z");
    let extra = format!(
        "{}\n{}",
        scalar_buffer("BUF_A", 0.2),
        scalar_buffer("BUF_B", 0.15)
    );

    with_timed_session(&mut db, &cells_with(&extra), CLOCK, |resizer| {
        let a = resizer.link.liberty.cell_id("BUF_A").unwrap();
        let _ = resizer.replace_cell(b1, a).unwrap();
        let load = resizer.load_cap(driver);
        for target in &mut resizer.targets.load {
            *target = Farads::new(1e-6);
        }
        let b = resizer.link.liberty.cell_id("BUF_B").unwrap();
        resizer.targets.load[a] = load / 5.0;
        resizer.targets.load[b] = load;
        let _ = resizer.resized_multi_output.insert(b1);

        let changed = resizer.resize_to_target_slew(driver).unwrap();

        assert!(changed);
        assert_eq!(resizer.link.cell(resizer.db, b1).unwrap().0, b);
    });
}

#[test]
fn an_edit_in_a_batch_leaves_the_parasitics_of_an_older_net_to_the_flush() {
    let mut db = two_buffers();
    let (driver, net) = (pin(&db, "b1/Z"), db.net_id("m").unwrap());

    with_wired_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let before = resizer.load_cap(driver);
        let batch = resizer.batch_edits();
        let b2 = resizer.db.cell_id("b2").unwrap();
        resizer.db.cells[b2].location = Point::new(Dbu::new(70_000), Dbu::new(2800));

        resizer.invalidate_parasitics(net);

        assert_eq!(resizer.load_cap(driver), before);
        resizer.flush(batch);
        assert!(resizer.load_cap(driver) > before);
    });
}

#[test]
fn a_net_a_batch_adds_is_estimated_as_it_changes() {
    let mut db = two_buffers();
    let net = db.net_id("m").unwrap();

    with_wired_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let batch = resizer.batch_edits();
        let added = resizer.make_net("added", net).unwrap();

        resizer.invalidate_parasitics(added);

        assert!(resizer.graph.parasitics(added).is_none());
        resizer.flush(batch);
    });
}

#[test]
fn a_design_ports_slew_limit_is_the_designs_and_a_pin_without_a_library_port_has_none() {
    let mut db = tristate_and_plain_drivers();
    let out = db.io_pins[db.io_pin_id("out").unwrap()].pin;
    let limited = format!("{CLOCK}set_max_transition 0.3 [current_design]\n");

    with_timed_session(&mut db, &cells_with(TRISTATE_BUFFER), &limited, |resizer| {
        let limit = resizer.load_slew_limit(out);

        assert!(
            limit.is_some_and(|l| (l.get() - 0.3e-9).abs() < 1e-18),
            "{limit:?}"
        );
    });
}

/// The flop `f` of a master with a pin `CK` that its Liberty cell lacks.
const FLOP_WITHOUT_CLOCK_PORT: &str = "cell (HALF) { area : 4.5 ;
        pin (D) { direction : input ; capacitance : 1.0 ; }
        pin (Q) { direction : output ; function : \"D\" ; } }";

#[test]
fn an_instance_pin_the_library_has_no_port_for_has_no_slew_limit() {
    let mut db = crate::testing::design_with_copies(
        &def_text(
            "half",
            &DefSections {
                components: "- f HALF + PLACED ( 2000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f CK ) ;\n- in ( PIN in ) ( f D ) ;\n\
                       - out ( f Q ) ( PIN out ) ;\n",
                inputs: &["clk", "in"],
                outputs: &["out"],
            },
        ),
        &[("DFF_X1", "HALF")],
    );
    let missing = pin(&db, "f/CK");
    let limited = "set_max_transition 0.3 [current_design]\n";

    with_timed_session(
        &mut db,
        &cells_with(FLOP_WITHOUT_CLOCK_PORT),
        limited,
        |resizer| {
            assert_eq!(resizer.load_slew_limit(missing), None);
        },
    );
}

/// A buffer cell of `name` whose output slews are `rise_ns` rising and
/// `fall_ns` falling at any load and input slew.
fn buffer_with_slews(name: &str, rise_ns: f64, fall_ns: f64) -> String {
    scalar_buffer(name, 0.1)
        .replace(
            "rise_transition (scalar) { values (\"0.01\") ; }",
            &format!("rise_transition (scalar) {{ values (\"{rise_ns}\") ; }}"),
        )
        .replace(
            "fall_transition (scalar) { values (\"0.01\") ; }",
            &format!("fall_transition (scalar) {{ values (\"{fall_ns}\") ; }}"),
        )
}

/// The targets of a buffer set of `names` over the clock cells and `extra`,
/// with a copy of `BUF_X1` for each of the names.
fn targets_of(extra: &str, names: &[&str]) -> (Targets, Liberty) {
    let copies: Vec<(&str, &str)> = names.iter().map(|&name| ("BUF_X1", name)).collect();
    let db = two_buffers_with_copies(&copies);
    let lib = cells_with(extra);
    let link = Link::new(&db, &lib, &[]).unwrap();
    let buffer = |name: &str| lib.buffer_cell(lib.cell_id(name).unwrap()).unwrap();
    let chosen: Vec<BufferCell> = names.iter().map(|&name| buffer(name)).collect();
    let set = BufferSet {
        cells: chosen,
        lowest_drive: buffer("BUF_X1"),
    };
    (find_targets(&link, &set), lib)
}

#[test]
fn target_slews_are_the_means_of_the_buffers_output_slews_per_edge() {
    let extra = format!(
        "{}\n{}",
        buffer_with_slews("BUF_P", 0.1, 0.05),
        buffer_with_slews("BUF_Q", 0.3, 0.15)
    );

    let (targets, _) = targets_of(&extra, &["BUF_P", "BUF_Q"]);

    assert!(
        (targets.slews.rise.get() - 0.2e-9).abs() < 1e-18,
        "{targets:?}"
    );
    assert!(
        (targets.slews.fall.get() - 0.1e-9).abs() < 1e-18,
        "{targets:?}"
    );
}

#[test]
fn without_buffers_the_target_slews_are_zero() {
    let (targets, _) = targets_of("", &[]);

    assert_eq!(targets.slews, EdgeValues::both(Seconds::ZERO));
}

#[test]
fn only_a_usable_cell_with_a_master_has_a_target_load() {
    let usable = renamed_cell("BUF_X1", "BUF_USABLE");
    let unusable =
        renamed_cell("BUF_X1", "BUF_UNUSABLE").replacen("area :", "dont_use : true ; area :", 1);
    let masterless = renamed_cell("BUF_X1", "BUF_MASTERLESS");
    let extra = format!("{usable}\n{unusable}\n{masterless}");
    let db = two_buffers_with_copies(&[("BUF_X1", "BUF_USABLE"), ("BUF_X1", "BUF_UNUSABLE")]);
    let lib = cells_with(&extra);
    let link = Link::new(&db, &lib, &[]).unwrap();
    let set = link.find_buffers(&db).unwrap();

    let targets = find_targets(&link, &set);

    let load = |name: &str| targets.load[lib.cell_id(name).unwrap()];
    assert!(load("BUF_USABLE") > Farads::ZERO);
    assert_eq!(load("BUF_UNUSABLE"), Farads::ZERO);
    assert_eq!(load("BUF_MASTERLESS"), Farads::ZERO);
}

#[test]
fn a_check_on_a_buffers_input_does_not_move_the_target_slews() {
    let plain = cells(&clock_cells_source());
    let checked = cells_with_buffer_edit(
        "capacitance : 0.97 ;",
        "capacitance : 0.97 ; timing () { related_pin : \"A\" ; timing_type : hold_rising ;
           rise_constraint (scalar) { values (\"0.5\") ; }
           fall_constraint (scalar) { values (\"0.5\") ; } }",
    );
    let slews = |lib: &Liberty| {
        let db = two_buffers();
        let link = Link::new(&db, lib, &[]).unwrap();
        let set = link.find_buffers(&db).unwrap();
        find_targets(&link, &set).slews
    };

    assert_eq!(slews(&checked), slews(&plain));
}

/// Seconds in picoseconds to a thousandth of one.
fn ps(time: Seconds) -> String {
    format!(
        "{:.3}",
        Seconds::new(time.get() * 1e12).get().max(-0.0) + 0.0
    )
}

/// Femtofarads to a thousandth of one.
fn ff(load: Farads) -> String {
    format!("{:.4}", load.get() * 1e15)
}

#[test]
fn the_load_that_gives_a_target_slew_over_every_arc_and_slew_of_the_clock_cells_is_as_found() {
    let lib = cells(&clock_cells_source());
    let mut lines = String::new();
    for (id, cell) in lib.cells.iter_enumerated() {
        let _ = writeln!(
            lines,
            "{} mean {}",
            cell.name,
            ff(find_target_load(
                cell,
                &EdgeValues::both(Seconds::new(0.05e-9))
            ))
        );
        for set in cell.arc_sets.iter().filter(|s| s.role.is_logic_delay()) {
            for arc in &set.arcs {
                let (from, to) = (
                    cell.ports[set.from].name.as_str(),
                    cell.ports[set.to].name.as_str(),
                );
                for in_ps in [0.0, 10.0, 60.0, 200.0] {
                    let in_slew = Seconds::new(in_ps * 1e-12);
                    let exact = |load: f64| arc.gate_delay(in_slew, Farads::new(load)).1;
                    let outs = [
                        Seconds::new(5e-12),
                        Seconds::new(30e-12),
                        Seconds::new(80e-12),
                        Seconds::new(400e-12),
                        exact(0.0),
                        exact(1e-12),
                        exact(5e-13),
                    ];
                    for out_slew in outs {
                        let found = arc_target_load(arc, in_slew, out_slew);
                        assert!(found >= Farads::ZERO, "{id:?}");
                        let _ = writeln!(
                            lines,
                            "{} {from}->{to} {:?}>{:?} in {in_ps} out {} load {}",
                            cell.name,
                            arc.from_edge,
                            arc.to_edge,
                            ps(out_slew),
                            ff(found),
                        );
                    }
                }
            }
        }
    }

    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden");
    pare_common::testing::assert_golden(&dir, "b_target_loads.txt", &lines);
}

#[test]
fn a_target_load_gives_a_slew_just_under_the_target_and_a_little_more_load_one_over() {
    let lib = cells(&clock_cells_source());
    let cell = lib.cell(lib.cell_id("BUF_X1").unwrap());
    let arc = &cell
        .arc_sets
        .iter()
        .find(|s| s.role.is_logic_delay())
        .unwrap()
        .arcs[0];
    let (in_slew, out_slew) = (Seconds::new(60e-12), Seconds::new(80e-12));

    let load = arc_target_load(arc, in_slew, out_slew);

    assert!(load > Farads::ZERO);
    assert!(arc.gate_delay(in_slew, load).1 <= out_slew);
    assert!(arc.gate_delay(in_slew, load * 1.02).1 >= out_slew);
    assert_eq!(arc_target_load(arc, in_slew, Seconds::ZERO), Farads::ZERO);
}
