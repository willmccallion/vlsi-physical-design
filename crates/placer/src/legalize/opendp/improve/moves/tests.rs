use super::super::tests::{improver_of, tiny_edited_from_one_spot};
use super::*;

fn nodes(count: u32) -> Vec<NodeId> {
    (0..count).map(NodeId).collect()
}

#[test]
fn a_position_inside_a_segment_lies_between_two_cells() {
    let cells = nodes(3);

    assert_eq!(around(&cells, 1), (Some(cells[0]), Some(cells[1])));
}

#[test]
fn a_position_before_the_first_cell_has_nothing_left_of_it() {
    let cells = nodes(3);

    assert_eq!(around(&cells, 0), (None, Some(cells[0])));
}

#[test]
fn a_position_past_the_last_cell_has_nothing_right_of_it() {
    let cells = nodes(3);

    assert_eq!(around(&cells, 3), (Some(cells[2]), None));
}

#[test]
fn a_position_snaps_to_the_start_of_the_site_holding_it() {
    let (origin, spacing) = (Dbu::new(100), Dbu::new(10));

    let inside = site_holding(Dbu::new(135), origin, spacing);
    let on_edge = site_holding(Dbu::new(130), origin, spacing);

    assert_eq!((inside, on_edge), (Dbu::new(130), Dbu::new(130)));
}

#[test]
fn an_empty_segment_has_no_neighbours() {
    assert_eq!(around(&[], 0), (None, None));
}

/// The second row starts a site in, at 0.2 um, and holds six cells
/// side by side from 0.4 um: a NAND2 at 0.4, then cells at 1, 1.6, 2,
/// 2.4 and 2.8 um.
#[test]
fn pushed_cells_land_on_the_sites_of_a_row_not_starting_at_zero() {
    let mut imp = improver_of(tiny_edited_from_one_spot(|text| {
        text.replace(
            "ROW ROW_1 core 0 2000 FS DO 30 BY 1 STEP 200 0 ;",
            "ROW ROW_1 core 200 2000 FS DO 29 BY 1 STEP 200 0 ;",
        )
    }));
    let s = imp.segs_in_row[ArchRowId::new(1)][0];
    let cells = imp.cells_in_seg[s].clone();

    let pushed = imp
        .shift_right_helper(cells[0], Dbu::new(500), s, cells[1])
        .unwrap();

    assert!(pushed);
    let lefts: Vec<Dbu> = cells[1..].iter().map(|&n| imp.left(n)).collect();
    assert_eq!(lefts, [1200, 1800, 2200, 2600, 3000].map(Dbu::new));
}
