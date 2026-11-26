use super::*;
use pare_common::db::parser::{def, lef};

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

#[test]
fn rows_framing_a_gap_enclose_it() {
    let rows = [
        rect(0, 0, 30, 10),
        rect(0, 10, 10, 20),
        rect(20, 10, 30, 20),
        rect(0, 20, 30, 30),
    ];

    assert_eq!(enclosed_holes(&rows), vec![rect(10, 10, 20, 20)]);
}

#[test]
fn a_gap_reaching_the_rows_edge_encloses_nothing() {
    let rows = [rect(0, 0, 30, 10), rect(0, 10, 10, 20)];

    assert_eq!(enclosed_holes(&rows), Vec::new());
}

/// The tiny design's placer with its fixed cells on the grid.
fn tiny_placer() -> Placer {
    let path = |name: &str| {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../common/tests/fixtures")
            .join(name)
    };
    let units = def::read_units(&path("tiny.def")).unwrap();
    let library = lef::parse_files(&[path("tiny.lef")], units).unwrap();
    let db = def::parse(&path("tiny.def"), library).unwrap();
    let mut placer = Placer::new(&db).unwrap();
    placer.paint_fixed_cells();
    placer
}

/// Row 1 is valid only at sites 14 and 26: six sites left of site 20
/// is a gap of 1 um, six right of it a reach of 0.8 um past the 0.4 um
/// inverter.
#[test]
fn a_hopeless_site_moves_to_the_valid_site_nearest_by_the_cell_s_reach() {
    let mut placer = tiny_placer();
    let row = GridY::new(1);
    for x in GridX::ZERO.until(placer.rows.row_site_count()) {
        if let Some(p) = placer.pixel_mut(x, row) {
            p.valid = [14, 26].contains(&x.get());
        }
    }
    let inverter = placer
        .nodes
        .ids()
        .find(|&n| placer.nodes[n].name == "u1")
        .unwrap();
    let node = placer.nodes[inverter].clone();

    let found = placer.move_hopeless(&node, GridPt::new(GridX::new(20), row));

    assert_eq!(found.map(|f| f.0), Some(GridPt::new(GridX::new(26), row)));
}

#[test]
fn every_node_id_below_the_pixel_markers_fits() {
    let most = Pixel::DUMMY as usize - 1;

    assert_eq!(check_pixel_ids(most, Pixel::NO_GROUP as usize - 1), Ok(()));
}

#[test]
fn a_node_id_reaching_the_pixel_markers_is_an_error() {
    let count = Pixel::DUMMY as usize;

    assert_eq!(
        check_pixel_ids(count, 0),
        Err(DetailedPlacementError::TooManyCells(count))
    );
}

#[test]
fn a_group_id_reaching_the_no_group_marker_is_an_error() {
    let count = Pixel::NO_GROUP as usize;

    assert_eq!(
        check_pixel_ids(0, count),
        Err(DetailedPlacementError::TooManyRegions(count))
    );
}

#[test]
fn no_rows_enclose_nothing() {
    assert_eq!(enclosed_holes(&[]), Vec::new());
}
