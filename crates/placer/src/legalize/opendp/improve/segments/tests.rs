use super::super::tests::improver;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;

/// The tiny design's top row ends at 4 um; a filler blocks it from
/// there to the core's right edge at 6 um.
#[test]
fn a_blockage_of_the_top_row_is_seen_from_a_cell_in_it() {
    let mut imp = improver();
    let n = imp.movable_cells()[0];
    let top_row = imp.arch.rows.iter().last().unwrap().bottom;
    imp.set_position(n, Point::new(Dbu::new(3000), top_row));

    assert!(imp.inside_blockage(n, Dbu::new(5000)));
}
