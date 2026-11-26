use super::super::Pixel;
use super::*;
use pare_common::db::parser::{def, lef};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

/// The tiny design with `members` fenced into a region of `rects`,
/// given as DEF points.
fn tiny_fenced(rects: &str, members: &str) -> NetlistDB {
    let text = std::fs::read_to_string(fixture("tiny.def"))
        .unwrap()
        .replace(
            "COMPONENTS 8 ;",
            &format!("REGIONS 1 ;\n- fence {rects} + TYPE FENCE ;\nEND REGIONS\nCOMPONENTS 8 ;"),
        )
        .replace(
            "END DESIGN",
            &format!("GROUPS 1 ;\n- g {members} + REGION fence ;\nEND GROUPS\nEND DESIGN"),
        );
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse_str(&text, Path::new("fenced.def"), library).unwrap()
}

/// The placer of `db` with its fixed cells and regions on the grid.
fn placer_with_regions(db: &NetlistDB) -> Placer {
    let mut placer = Placer::new(db).unwrap();
    placer.paint_fixed_cells();
    placer.mark_region_edges();
    placer.mark_region_sites();
    placer
}

#[test]
fn a_member_outside_its_region_is_packed_into_the_nearest_rectangle() {
    let mut db = tiny_fenced("( 0 0 ) ( 2000 2000 ) ( 3000 4000 ) ( 4000 6000 )", "u1 u2");
    let u1 = db.cell_id("u1").unwrap();
    db.cells[u1].location = Point::new(Dbu::new(3800), Dbu::new(3000));
    let mut placer = placer_with_regions(&db);
    let u1 = placer
        .nodes
        .ids()
        .find(|&n| placer.nodes[n].name == "u1")
        .unwrap();

    placer.brick_place_inner(GroupId(0)).unwrap();

    assert!(rect(3000, 4000, 4000, 6000).encloses(&placer.nodes[u1].bbox()));
}

#[test]
fn a_region_on_the_bottom_row_counts_that_row_s_height() {
    let db = tiny_fenced("( 0 0 ) ( 2000 2000 )", "u1");
    let mut placer = placer_with_regions(&db);

    placer.measure_group_utilization();

    let site_area = 10.0 * 200.0 * 2000.0;
    let expected = 400.0 * 2000.0 / site_area;
    assert!((placer.groups[GroupId(0)].util - expected).abs() < 1e-12);
}

/// The group of each of the bottom row's first ten sites.
fn bottom_row_groups(placer: &Placer) -> Vec<Option<GroupId>> {
    (0..10)
        .map(|x| {
            placer
                .pixel(GridX::new(x), GridY::ZERO)
                .and_then(Pixel::group)
        })
        .collect()
}

/// The region starts half way into the second site of the bottom row.
#[test]
fn a_region_off_the_site_grid_claims_every_whole_site_and_blocks_the_part() {
    let db = tiny_fenced("( 300 0 ) ( 2000 2000 )", "u1");

    let placer = placer_with_regions(&db);

    let g = Some(GroupId(0));
    assert_eq!(
        bottom_row_groups(&placer),
        [None, None, g, g, g, g, g, g, g, g]
    );
    let partial = placer.pixel(GridX::new(1), GridY::ZERO).unwrap();
    assert!(!partial.is_free() && !partial.valid);
}

/// Two rectangles overlap over sites 4 and 5; two more meet half way
/// into site 5 of the next pair.
#[test]
fn sites_the_region_covers_only_with_several_rectangles_are_its_own() {
    let overlapping = tiny_fenced("( 0 0 ) ( 1200 2000 ) ( 800 0 ) ( 2000 2000 )", "u1");
    let meeting = tiny_fenced("( 0 0 ) ( 1100 2000 ) ( 1100 0 ) ( 2000 2000 )", "u1");

    let groups = [overlapping, meeting].map(|db| bottom_row_groups(&placer_with_regions(&db)));

    assert_eq!(groups, [[Some(GroupId(0)); 10], [Some(GroupId(0)); 10]]);
}

#[test]
fn a_member_may_stand_across_two_rectangles_of_its_region() {
    let db = tiny_fenced("( 0 0 ) ( 1000 2000 ) ( 1000 0 ) ( 2000 2000 )", "u1");
    let placer = placer_with_regions(&db);
    let u1 = placer.nodes.iter().find(|n| n.name == "u1").unwrap();

    let allowed =
        placer.region_overlap_allowed(u1, GridX::new(4), GridY::ZERO, GridX::new(6), GridY::new(1));

    assert!(allowed);
}

#[test]
fn outer_bricks_pack_a_member_toward_the_box_corner_on_its_side() {
    let mut db = tiny_fenced("( 0 0 ) ( 2000 2000 )", "u1");
    let u1 = db.cell_id("u1").unwrap();
    db.cells[u1].location = Point::new(Dbu::new(1500), Dbu::new(1500));
    let mut placer = placer_with_regions(&db);
    let u1 = placer
        .nodes
        .ids()
        .find(|&n| placer.nodes[n].name == "u1")
        .unwrap();

    placer.brick_place_outer(GroupId(0)).unwrap();

    let at = placer.nodes[u1].bbox().min;
    assert_eq!(at, Point::new(Dbu::new(1600), Dbu::new(0)));
}
