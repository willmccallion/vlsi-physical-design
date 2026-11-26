use super::*;
use crate::library::Link;
use crate::testing::{DefSections, cells, clock_cells_source, def_text, design, metal2_wires, pin};

/// A flop driving two buffers, one level with it 20 microns away and
/// one 100 microns above that, and a buffer whose output drives nothing.
fn fanout() -> NetlistDB {
    design(&def_text(
        "fanout",
        &DefSections {
            components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - near BUF_X1 + PLACED ( 42000 2800 ) N ;\n\
                             - far BUF_X1 + PLACED ( 42000 202800 ) N ;\n",
            nets: "- q ( f1 Q ) ( near A ) ( far A ) ;\n- lone ( near Z ) ;\n",
            inputs: &[],
            outputs: &[],
        },
    ))
}

/// The half perimeter of the box around `pins`' locations.
fn half_perimeter(netlist: &Netlist<'_>, pins: &[PinId]) -> Meters {
    let points: Vec<Point<Dbu>> = pins.iter().map(|&p| netlist.pin_location(p)).collect();
    let span = |axis: fn(&Point<Dbu>) -> Dbu| {
        let values = points.iter().map(axis);
        values.clone().max().unwrap() - values.min().unwrap()
    };
    Meters::from_dbu(span(|p| p.x) + span(|p| p.y), netlist.db.dbu())
}

fn relative_gap(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

#[test]
fn a_net_of_one_pin_has_no_parasitics() {
    let db = fanout();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);

    let estimate =
        NetParasitics::estimate(&netlist, db.net_id("lone").unwrap(), &metal2_wires(&db));

    assert_eq!(estimate, None);
}

#[test]
fn three_pins_get_the_wire_of_their_steiner_tree() {
    let db = fanout();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let rc = metal2_wires(&db);
    let pins = [pin(&db, "f1/Q"), pin(&db, "near/A"), pin(&db, "far/A")];

    let estimate = NetParasitics::estimate(&netlist, db.net_id("q").unwrap(), &rc).unwrap();

    let length = half_perimeter(&netlist, &pins);
    let wire_cap: f64 = estimate.node_caps().iter().map(|c| c.get()).sum();
    let wire_res: f64 = estimate
        .resistors()
        .iter()
        .filter(|r| r.resistance != JOIN_RESISTANCE)
        .map(|r| r.resistance.get())
        .sum();
    assert!(relative_gap(wire_cap, (length * rc.capacitance).get()) < 1e-9);
    assert!(relative_gap(wire_res, (length * rc.resistance).get()) < 1e-9);
    assert_eq!(estimate.pin_nodes().len(), pins.len());
}

#[test]
fn the_farther_load_sees_the_longer_elmore_delay() {
    let db = fanout();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let estimate =
        NetParasitics::estimate(&netlist, db.net_id("q").unwrap(), &metal2_wires(&db)).unwrap();

    let pi = estimate
        .pi_elmore(&netlist, pin(&db, "f1/Q"), Edge::Rise, &|_, _| {
            Farads::new(1.0e-15)
        })
        .unwrap();

    let near = pi.elmore_to(pin(&db, "near/A"));
    let far = pi.elmore_to(pin(&db, "far/A"));
    assert!(Seconds::ZERO < near && near < far, "{near:?} {far:?}");
}
