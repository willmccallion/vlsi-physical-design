use super::*;
use crate::testing::{
    DefSections, cells, cells_with, clock_cells_source, def_text, design, metal2_wires, pin,
    with_session_inputs, with_timed_session,
};
use std::fmt::Write;

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
    design(&def_text(
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
    ))
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
