use crate::netlist::Netlist;
use crate::resizer::Resizer;
use crate::sta::Sta;
use crate::testing::{
    DefSections, cells, cells_with, clock_cells_source, def_text, design_with_copies, pin,
    with_timed_session,
};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::PinId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;

const CLOCK: &str = "create_clock -name clk -period 1.0 [get_ports clk]\n";

/// Buffers `b1` and `b2` both driving the net `bus` into flop `f1`, and
/// `b3` driving `other`, last of its pins, into flop `f2`; over the test
/// cells and, for each `(master, name)` of `copies`, a copy of `master`
/// called `name`.
fn shared_bus(copies: &[(&str, &str)]) -> NetlistDB {
    design_with_copies(
        &def_text(
            "bus",
            &DefSections {
                components: "- b1 BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - b2 BUF_X1 + PLACED ( 2000 30000 ) N ;\n\
                             - b3 BUF_X1 + PLACED ( 2000 60000 ) N ;\n\
                             - f1 DFF_X1 + PLACED ( 40000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 40000 60000 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n\
                       - in1 ( PIN in1 ) ( b1 A ) ;\n- in2 ( PIN in2 ) ( b2 A ) ( b3 A ) ;\n\
                       - bus ( b1 Z ) ( b2 Z ) ( f1 D ) ;\n- other ( f2 D ) ( b3 Z ) ;\n",
                inputs: &["clk", "in1", "in2"],
                outputs: &[],
            },
        ),
        copies,
    )
}

/// The nets whose drivers as the graph keeps them are not those the
/// design lists, in its order, each line headed by `step`.
fn stale_drivers(resizer: &Resizer<'_, Sta>, step: &str) -> Vec<String> {
    let netlist = resizer.netlist();
    resizer
        .db
        .nets
        .ids()
        .filter_map(|net| {
            let listed: Vec<PinId> = netlist.drivers(net).collect();
            let kept = resizer.graph.drivers(net);
            let name = &resizer.db.nets[net].name;
            (kept != listed.as_slice()).then(|| format!("{step}: {name}: {kept:?} vs {listed:?}"))
        })
        .collect()
}

/// An edit of a session over [`shared_bus`].
type SessionEdit = fn(&mut Resizer<'_, Sta>);

/// Runs `test` on a timed session over [`shared_bus`].
fn on_shared_bus(test: impl FnOnce(&mut Resizer<'_, Sta>)) {
    let mut db = shared_bus(&[]);
    with_timed_session(&mut db, &cells(&clock_cells_source()), CLOCK, test);
}

/// Puts a buffer before `f1`'s data input.
fn buffer_f1(resizer: &mut Resizer<'_, Sta>) {
    let bus = resizer.db.net_id("bus").unwrap();
    let load = pin(resizer.db, "f1/D");
    let buffer = resizer.buffers.lowest_drive;
    let at = Point::new(Dbu::new(20_000), Dbu::new(2800));
    let _ = resizer
        .insert_buffer_before_loads(bus, &[load], buffer, at, "split")
        .unwrap();
}

/// Gives `b2` the clock buffer's master.
fn resize_b2(resizer: &mut Resizer<'_, Sta>) {
    let b2 = resizer.db.cell_id("b2").unwrap();
    let clock_buffer = resizer.link.liberty.cell_id("CLKBUF_X3").unwrap();
    let _ = resizer.replace_cell(b2, clock_buffer).unwrap();
}

/// Takes buffer `name` out.
fn remove(resizer: &mut Resizer<'_, Sta>, name: &str) {
    let buffer = resizer.db.cell_id(name).unwrap();
    let _ = resizer.remove_buffer(buffer).unwrap();
}

/// Moves `b1`'s output to `other`, `b3`'s onto `bus` behind `b2`'s, then
/// `b1`'s back onto `bus` last.
fn shuffle_drivers(resizer: &mut Resizer<'_, Sta>) {
    let (bus, other) = (
        resizer.db.net_id("bus").unwrap(),
        resizer.db.net_id("other").unwrap(),
    );
    resizer.move_pin(pin(resizer.db, "b1/Z"), other);
    resizer.move_pin(pin(resizer.db, "b3/Z"), bus);
    resizer.move_pin(pin(resizer.db, "b1/Z"), bus);
}

#[test]
fn a_nets_drivers_follow_each_edit_in_the_order_the_net_lists_them() {
    on_shared_bus(|resizer| {
        let mut stale = Vec::new();

        shuffle_drivers(resizer);
        stale.extend(stale_drivers(resizer, "drivers moved"));
        resize_b2(resizer);
        stale.extend(stale_drivers(resizer, "b2 resized"));
        buffer_f1(resizer);
        stale.extend(stale_drivers(resizer, "f1 buffered"));
        remove(resizer, "b3");
        stale.extend(stale_drivers(resizer, "b3 removed"));

        assert!(stale.is_empty(), "{}", stale.join("\n"));
    });
}

#[test]
fn a_nets_drivers_come_back_in_order_when_a_trial_is_taken_back() {
    let trials: [(&str, SessionEdit); 4] = [
        ("drivers moved", shuffle_drivers),
        ("b1 removed", |r| remove(r, "b1")),
        ("f1 buffered", buffer_f1),
        ("b2 resized", resize_b2),
    ];
    on_shared_bus(|resizer| {
        let mut stale = Vec::new();

        for (step, edit) in trials {
            let trial = resizer.journal_begin();
            edit(resizer);
            resizer.journal_restore(trial).unwrap();
            stale.extend(stale_drivers(resizer, step));
        }

        assert!(stale.is_empty(), "{}", stale.join("\n"));
    });
}

/// A buffer driving from `A` and driven at `Z`, the reverse of `BUF_X1`.
const REVERSED_BUFFER: &str = "cell (BUF_REV) { area : 0.8 ;
        pin (Z) { direction : input ; capacitance : 1.0 ; }
        pin (A) { direction : output ; function : \"Z\" ;
          timing () { related_pin : \"Z\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } } } }";

#[test]
fn a_master_whose_pins_drive_otherwise_moves_the_cells_drivers() {
    let mut db = shared_bus(&[("BUF_X1", "BUF_REV")]);
    with_timed_session(&mut db, &cells_with(REVERSED_BUFFER), CLOCK, |resizer| {
        let b1 = resizer.db.cell_id("b1").unwrap();
        let reversed = resizer.link.liberty.cell_id("BUF_REV").unwrap();
        let mut stale = Vec::new();
        let trial = resizer.journal_begin();

        assert!(resizer.replace_cell(b1, reversed).unwrap());
        stale.extend(stale_drivers(resizer, "b1 reversed"));
        resizer.journal_restore(trial).unwrap();
        stale.extend(stale_drivers(resizer, "undone"));

        assert!(stale.is_empty(), "{}", stale.join("\n"));
    });
}

#[test]
fn a_driver_taken_in_again_stays_listed_once() {
    on_shared_bus(|resizer| {
        let other = resizer.db.net_id("other").unwrap();
        let b3 = pin(resizer.db, "b3/Z");

        resizer
            .graph
            .pin_connected(&Netlist::new(resizer.db, resizer.link), b3);

        assert_eq!(resizer.graph.drivers(other), [b3]);
    });
}

#[test]
fn the_only_driver_of_a_net_is_listed_even_when_the_net_lists_it_before_others() {
    on_shared_bus(|resizer| {
        let bus = resizer.db.net_id("bus").unwrap();
        let (first, second) = (pin(resizer.db, "b1/Z"), pin(resizer.db, "b2/Z"));
        resizer.graph.pin_disconnected(bus, first);
        resizer.graph.pin_disconnected(bus, second);

        resizer
            .graph
            .pin_connected(&Netlist::new(resizer.db, resizer.link), first);

        assert_eq!(resizer.graph.drivers(bus), [first]);
    });
}
