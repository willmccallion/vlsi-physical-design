use super::super::tests::{improver_of, tiny_from_one_spot};
use super::*;

/// The tiny design's rows allow mirroring about Y; its masters here do
/// not.
#[test]
fn a_cell_whose_master_forbids_mirroring_is_not_flipped() {
    let mut db = tiny_from_one_spot();
    for m in &mut db.library.macros {
        m.symmetry.y = false;
    }
    let mut imp = improver_of(db);

    run(&mut imp);

    let wrong: Vec<(String, Orientation)> = imp
        .movable_cells()
        .into_iter()
        .map(|n| &imp.grid.nodes[n])
        .filter(|node| !node.symmetry.allows(node.orient))
        .map(|node| (node.name.clone(), node.orient))
        .collect();
    assert_eq!(wrong, []);
}
