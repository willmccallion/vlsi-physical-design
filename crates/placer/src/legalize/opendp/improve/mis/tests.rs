use super::super::tests::{improver_of, tiny_from_one_spot};
use super::*;
use pare_common::db::library::SignalUse;

const TOLERANCE: f64 = 0.005;

/// The node of the cell named `name`.
fn node(imp: &Improver, name: &str) -> NodeId {
    imp.grid
        .nodes
        .ids()
        .find(|&n| imp.grid.nodes[n].name == name)
        .unwrap()
}

/// The cells set matching gathers around `name`'s.
fn neighbours_of(imp: &Improver, name: &str) -> Vec<NodeId> {
    let candidates = imp.movable_cells();
    let colors = color_cells(imp, &candidates).unwrap();
    let mut bins = Bins::new(imp, &candidates);
    bins.populate(imp, &candidates);
    gather_neighbours(imp, node(imp, name), &colors, &mut bins, &mut 0).unwrap()
}

#[test]
fn inverters_alike_are_gathered_together() {
    let imp = improver_of(tiny_from_one_spot());

    let neighbours = neighbours_of(&imp, "u2");

    assert!(neighbours.contains(&node(&imp, "u5")));
}

#[test]
fn an_inverter_with_its_rails_swapped_is_not_gathered() {
    let mut db = tiny_from_one_spot();
    let u5 = db.cell_id("u5").unwrap();
    let mut swapped = db.cell_macro(u5).clone();
    swapped.name = "INV_SWAPPED".into();
    let height = swapped.size.y;
    for pin in &mut swapped.pins {
        if matches!(pin.signal_use, SignalUse::Power | SignalUse::Ground) {
            for shape in &mut pin.shapes {
                let (lo, hi) = (shape.rect.min.y, shape.rect.max.y);
                (shape.rect.min.y, shape.rect.max.y) = (height - hi, height - lo);
            }
        }
    }
    db.cells[u5].macro_id = db.library.add_macro(swapped).unwrap();
    let imp = improver_of(db);

    let neighbours = neighbours_of(&imp, "u2");

    assert!(!neighbours.contains(&node(&imp, "u5")));
}

#[test]
fn a_pass_converges_when_it_changes_the_wirelength_within_the_tolerance() {
    assert!(converged(Dbu::new(1000), Dbu::new(995), TOLERANCE));
}

#[test]
fn a_pass_saving_more_than_the_tolerance_has_not_converged() {
    assert!(!converged(Dbu::new(1000), Dbu::new(990), TOLERANCE));
}

#[test]
fn a_pass_lengthening_the_nets_past_the_tolerance_has_not_converged() {
    assert!(!converged(Dbu::new(1000), Dbu::new(1010), TOLERANCE));
}

#[test]
fn no_wirelength_to_improve_has_converged() {
    assert!(converged(Dbu::new(0), Dbu::new(50), TOLERANCE));
}
