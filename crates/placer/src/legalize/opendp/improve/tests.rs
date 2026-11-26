use super::*;
use crate::legalize::opendp::detailed_placement;
use pare_common::db::design::PlacementStatus;
use pare_common::db::library::{Macro, Site};
use pare_common::db::parser::{def, lef};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

/// The tiny design's six movable cells legalized from one spot, laid
/// out for improvement as `improve_placement` lays them before its
/// passes.
pub(super) fn improver() -> Improver {
    improver_of(tiny_from_one_spot())
}

/// The tiny design with its six movable cells at one spot.
pub(super) fn tiny_from_one_spot() -> NetlistDB {
    tiny_edited_from_one_spot(|text| text)
}

/// The tiny design, its DEF as `edit` rewrites it, with its six movable
/// cells at one spot.
pub(super) fn tiny_edited_from_one_spot(edit: impl Fn(String) -> String) -> NetlistDB {
    let text = edit(std::fs::read_to_string(fixture("tiny.def")).unwrap());
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    let mut db = def::parse_str(&text, &fixture("tiny.def"), library).unwrap();
    for name in ["u1", "u2", "u4", "u5", "u6", "u7"] {
        let id = db.cell_id(name).unwrap();
        let cell = &mut db.cells[id];
        cell.location = Point::new(Dbu::new(1000), Dbu::new(1000));
        cell.status = PlacementStatus::Placed;
    }
    db
}

/// `db` legalized and laid out for improvement.
pub(super) fn improver_of(mut db: NetlistDB) -> Improver {
    let _ = detailed_placement(&mut db).unwrap();
    let max_displacement = MaxDisplacement {
        x: Some(RowHeights(5)),
        y: Some(RowHeights(1)),
    };
    let mut improver = Improver::new(&db, max_displacement).unwrap();
    shift::legalize(&mut improver).unwrap();
    improver.grid.paint_fixed_cells();
    improver
}

/// Where every movable cell stands and faces, and the cells of each
/// segment.
#[derive(Debug, PartialEq, Eq)]
struct Layout {
    cells: Vec<(Dbu, Dbu, Orientation)>,
    segments: Vec<Vec<NodeId>>,
}

fn layout(imp: &Improver) -> Layout {
    Layout {
        cells: imp
            .movable_cells()
            .into_iter()
            .map(|n| (imp.left(n), imp.bottom(n), imp.grid.nodes[n].orient))
            .collect(),
        segments: imp.cells_in_seg.iter().cloned().collect(),
    }
}

/// The first move of a movable cell to a site of any segment that
/// lands legally, its trial left open.
fn first_legal_move(imp: &mut Improver) -> Option<Trial> {
    let spacing = imp.arch.rows[ArchRowId::BOTTOM].site_spacing;
    for n in imp.movable_cells() {
        let si = imp.cell_segs[n][0];
        for sj in imp.segments.ids() {
            let segment = imp.segments[sj];
            let mut x = segment.min_x;
            while x < segment.max_x {
                if x != imp.left(n)
                    && let Some(trial) = imp.try_move(n, si, x, sj).unwrap()
                {
                    return Some(trial);
                }
                x += spacing;
            }
        }
    }
    None
}

#[test]
fn a_legalized_placement_has_no_placement_problems() {
    let imp = improver();

    assert_eq!(imp.placement_problems().unwrap(), 0);
}

#[test]
fn a_rejected_move_puts_every_cell_back() {
    let mut imp = improver();
    let before = layout(&imp);
    let hpwl = imp.total_hpwl();
    let trial = first_legal_move(&mut imp).unwrap();
    assert_ne!(layout(&imp), before);

    imp.reject_trial(trial).unwrap();

    assert_eq!(layout(&imp), before);
    assert_eq!(imp.total_hpwl(), hpwl);
    assert_eq!(imp.placement_problems().unwrap(), 0);
}

#[test]
fn an_accepted_move_stays_and_stays_legal() {
    let mut imp = improver();
    let before = layout(&imp);
    let trial = first_legal_move(&mut imp).unwrap();

    imp.accept_trial(trial);

    assert_ne!(layout(&imp), before);
    assert_eq!(imp.placement_problems().unwrap(), 0);
}

#[test]
fn a_cell_aligns_down_to_the_site_grid_within_its_span() {
    let imp = improver();
    let n = imp.movable_cells()[0];
    let row = imp.arch.rows[ArchRowId::BOTTOM];
    let site = |i: i64| row.left + row.site_spacing * i;

    let aligned = imp.align(n, site(3) + Dbu::new(1), Interval::new(site(0), site(40)));

    assert_eq!(aligned, Some(site(3)));
}

#[test]
fn a_cell_aligned_past_its_span_stops_at_the_last_site_it_fits() {
    let imp = improver();
    let n = imp.movable_cells()[0];
    let row = imp.arch.rows[ArchRowId::BOTTOM];
    let site = |i: i64| row.left + row.site_spacing * i;
    let right = site(40);

    let aligned = imp.align(n, right, Interval::new(site(0), right)).unwrap();

    assert_eq!(aligned + imp.width(n), right);
}

#[test]
fn a_cell_wider_than_its_span_has_no_alignment() {
    let imp = improver();
    let n = imp.movable_cells()[0];
    let left = imp.arch.rows[ArchRowId::BOTTOM].left;

    let aligned = imp.align(
        n,
        left,
        Interval::new(left, left + imp.width(n) - Dbu::new(1)),
    );

    assert_eq!(aligned, None);
}

/// The pin `in` is a 0.1 by 0.2 um shape turned S at (1.1, 6), so it
/// spans 1.05 to 1.15 um across and 5.8 to 6 um up.
#[test]
fn an_io_pin_sits_at_the_center_of_its_shape() {
    let imp = improver();
    let pin_in = NodeId::new(imp.net.cells);

    let at = imp.pin_position(imp.net.node_pins[pin_in][0]);

    assert_eq!(at, Point::new(Dbu::new(1100), Dbu::new(5900)));
}

/// The tiny design with a fourth row on top and a cell two rows tall,
/// `tall`, starting at the bottom.
fn tiny_with_tall_cell() -> NetlistDB {
    let mut db = tiny_edited_from_one_spot(|text| {
        text.replace(
            "ROW ROW_2 core 0 4000 N DO 20 BY 1 STEP 200 0 ;",
            "ROW ROW_2 core 0 4000 N DO 20 BY 1 STEP 200 0 ;\n\
                 ROW ROW_3 core 0 6000 FS DO 20 BY 1 STEP 200 0 ;",
        )
    });
    let inv = db.cell_macro(db.cell_id("u1").unwrap()).clone();
    let tall = Macro::new(
        "TALL".into(),
        inv.class,
        Point::new(inv.size.x, inv.size.y * 2),
        inv.symmetry,
        inv.site,
    );
    let tall = db.library.add_macro(tall).unwrap();
    let _ = db
        .add_cell(
            "tall".into(),
            tall,
            Point::new(Dbu::new(3000), Dbu::new(0)),
            Orientation::N,
            PlacementStatus::Placed,
        )
        .unwrap();
    db
}

#[test]
fn a_cell_two_rows_tall_can_move_into_the_top_two_rows() {
    let mut imp = improver_of(tiny_with_tall_cell());
    let n = imp
        .grid
        .nodes
        .ids()
        .find(|&n| imp.grid.nodes[n].name == "tall")
        .unwrap();
    let (si, sj) = (imp.cell_segs[n][0], imp.segs_in_row[ArchRowId::new(2)][0]);

    let moved = imp.try_move(n, si, Dbu::new(1000), sj).unwrap();

    assert!(moved.is_some());
    assert_eq!(imp.position(n), Point::new(Dbu::new(1000), Dbu::new(4000)));
}

#[test]
fn a_cell_whose_site_no_row_has_is_an_internal_error() {
    let mut db = tiny_from_one_spot();
    let _ = detailed_placement(&mut db).unwrap();
    let core = db.library.sites[db.rows[0].site].clone();
    let other = db.library.sites.push(Site {
        name: "other".into(),
        ..core
    });
    let inv = db.cells[db.cell_id("u1").unwrap()].macro_id;
    db.library.macros[inv].site = Some(other);
    let max_displacement = MaxDisplacement {
        x: Some(RowHeights(5)),
        y: Some(RowHeights(1)),
    };

    let result = improve_placement(&mut db, max_displacement);

    assert_eq!(
        result,
        Err(ImprovementError::Internal("cell put where no row takes it"))
    );
}
