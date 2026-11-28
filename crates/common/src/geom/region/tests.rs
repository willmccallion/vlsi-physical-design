use super::*;
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};
use std::collections::HashSet;

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::from_corners(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

fn edge(x0: i64, y0: i64, x1: i64, y1: i64) -> Edge {
    Edge {
        from: Point::new(Dbu::new(x0), Dbu::new(y0)),
        to: Point::new(Dbu::new(x1), Dbu::new(y1)),
    }
}

fn sorted<T: Ord>(mut v: Vec<T>) -> Vec<T> {
    v.sort();
    v
}

fn key(r: &Rect<Dbu>) -> (i64, i64, i64, i64) {
    (r.min.x.get(), r.min.y.get(), r.max.x.get(), r.max.y.get())
}

fn edge_key(e: &Edge) -> (i64, i64, i64, i64) {
    (e.from.x.get(), e.from.y.get(), e.to.x.get(), e.to.y.get())
}

/// An L of a 10x2 bar along the bottom and a 2x10 bar up the left side.
fn ell() -> Region {
    Region::from_rects([rect(0, 0, 10, 2), rect(0, 0, 2, 10)])
}

#[test]
fn regions_covering_the_same_area_are_equal() {
    let one = Region::from_rects([rect(0, 0, 10, 10)]);
    let pieces = Region::from_rects([rect(0, 0, 5, 10), rect(5, 0, 10, 4), rect(5, 3, 10, 10)]);

    assert_eq!(one, pieces);
}

#[test]
fn rectangles_without_area_add_nothing() {
    let region = Region::from_rects([rect(0, 0, 10, 0), rect(3, 3, 3, 8)]);

    assert!(region.is_empty());
}

#[test]
fn overlapping_rectangles_count_their_shared_area_once() {
    let region = Region::from_rects([rect(0, 0, 10, 10), rect(5, 5, 15, 15)]);

    assert_eq!(region.area(), DbuArea(175));
}

#[test]
fn an_ell_has_six_edges_with_the_region_on_their_left() {
    let edges = sorted(ell().edges().iter().map(edge_key).collect());

    let expected = sorted(
        [
            edge(0, 0, 10, 0),
            edge(10, 0, 10, 2),
            edge(10, 2, 2, 2),
            edge(2, 2, 2, 10),
            edge(2, 10, 0, 10),
            edge(0, 10, 0, 0),
        ]
        .iter()
        .map(edge_key)
        .collect(),
    );
    assert_eq!(edges, expected);
}

#[test]
fn edge_headings_follow_their_direction() {
    assert_eq!(edge(0, 0, 10, 0).heading(), Heading::East);
    assert_eq!(edge(10, 2, 2, 2).heading(), Heading::West);
    assert_eq!(edge(10, 0, 10, 2).heading(), Heading::North);
    assert_eq!(edge(0, 10, 0, 0).heading(), Heading::South);
}

#[test]
fn a_hole_is_bounded_by_edges_with_the_metal_on_their_left() {
    let ring = Region::from_rects([
        rect(0, 0, 10, 2),
        rect(0, 8, 10, 10),
        rect(0, 0, 2, 10),
        rect(8, 0, 10, 10),
    ]);

    let edges: HashSet<(i64, i64, i64, i64)> = ring.edges().iter().map(edge_key).collect();

    assert_eq!(edges.len(), 8);
    assert!(edges.contains(&edge_key(&edge(2, 2, 2, 8))));
    assert!(edges.contains(&edge_key(&edge(8, 8, 8, 2))));
    assert!(edges.contains(&edge_key(&edge(2, 8, 8, 8))));
    assert!(edges.contains(&edge_key(&edge(8, 2, 2, 2))));
}

#[test]
fn an_ell_has_two_maximal_rectangles() {
    let rects = sorted(ell().max_rectangles().iter().map(key).collect());

    assert_eq!(rects, vec![(0, 0, 2, 10), (0, 0, 10, 2)]);
}

#[test]
fn a_ring_has_four_maximal_rectangles() {
    let ring = Region::from_rects([
        rect(0, 0, 10, 2),
        rect(0, 8, 10, 10),
        rect(0, 0, 2, 10),
        rect(8, 0, 10, 10),
    ]);

    let rects = sorted(ring.max_rectangles().iter().map(key).collect());

    assert_eq!(
        rects,
        vec![(0, 0, 2, 10), (0, 0, 10, 2), (0, 8, 10, 10), (8, 0, 10, 10)]
    );
}

#[test]
fn horizontal_slices_of_an_ell_span_whole_horizontal_runs() {
    let slices = sorted(ell().horizontal_slices().iter().map(key).collect());

    assert_eq!(slices, vec![(0, 0, 10, 2), (0, 2, 2, 10)]);
}

#[test]
fn vertical_slices_of_an_ell_span_whole_vertical_runs() {
    let slices = sorted(ell().vertical_slices().iter().map(key).collect());

    assert_eq!(slices, vec![(0, 0, 2, 10), (2, 0, 10, 2)]);
}

#[test]
fn a_slice_runs_on_past_changes_elsewhere_across_the_sweep() {
    let bumped_c = Region::from_rects([
        rect(0, 0, 1, 10),
        rect(1, 0, 5, 1),
        rect(1, 9, 5, 10),
        rect(3, 1, 4, 2),
    ]);

    let slices = sorted(bumped_c.vertical_slices().iter().map(key).collect());

    assert_eq!(
        slices,
        vec![
            (0, 0, 1, 10),
            (1, 0, 3, 1),
            (1, 9, 5, 10),
            (3, 0, 4, 2),
            (4, 0, 5, 1)
        ]
    );
}

#[test]
fn slices_go_around_a_hole() {
    let ring =
        Region::from_rects([rect(0, 0, 6, 6)]).difference(&Region::from_rects([rect(2, 2, 4, 4)]));

    let horizontal = sorted(ring.horizontal_slices().iter().map(key).collect());
    let vertical = sorted(ring.vertical_slices().iter().map(key).collect());

    assert_eq!(
        horizontal,
        vec![(0, 0, 6, 2), (0, 2, 2, 4), (0, 4, 6, 6), (4, 2, 6, 4)]
    );
    assert_eq!(
        vertical,
        vec![(0, 0, 2, 6), (2, 0, 4, 2), (2, 4, 4, 6), (4, 0, 6, 6)]
    );
}

#[test]
fn a_ring_has_one_hole() {
    let ring =
        Region::from_rects([rect(0, 0, 6, 6)]).difference(&Region::from_rects([rect(2, 2, 4, 4)]));

    let holes = ring.holes();

    assert_eq!(holes, vec![Region::from_rects([rect(2, 2, 4, 4)])]);
}

#[test]
fn empty_space_meeting_the_outside_at_a_corner_is_not_a_hole() {
    let notched = Region::from_rects([
        rect(0, 0, 3, 1),
        rect(0, 0, 1, 3),
        rect(2, 0, 3, 2),
        rect(0, 2, 2, 3),
    ]);

    assert_eq!(notched.holes(), []);
}

#[test]
fn rectangles_meeting_at_a_corner_are_separate_pieces() {
    let region = Region::from_rects([rect(0, 0, 5, 5), rect(5, 5, 10, 10)]);

    assert_eq!(region.components().len(), 2);
}

#[test]
fn rectangles_sharing_an_edge_are_one_piece() {
    let region = Region::from_rects([rect(0, 0, 5, 5), rect(5, 2, 10, 4), rect(20, 0, 25, 5)]);

    let mut areas: Vec<i64> = region.components().iter().map(|c| c.area().get()).collect();
    areas.sort_unstable();

    assert_eq!(areas, vec![25, 35]);
}

#[test]
fn covering_needs_every_point_of_the_rectangle() {
    let region = ell();

    assert!(region.covers_rect(&rect(0, 0, 10, 2)));
    assert!(region.covers_rect(&rect(0, 0, 2, 10)));
    assert!(!region.covers_rect(&rect(0, 0, 3, 3)));
    assert!(region.intersects_rect(&rect(1, 1, 5, 5)));
    assert!(!region.intersects_rect(&rect(2, 2, 5, 5)));
}

#[test]
fn covering_a_rectangle_without_area_needs_every_point_of_it_edges_included() {
    let region = Region::from_rects([rect(0, 0, 5, 10), rect(5, 0, 10, 5)]);

    assert!(region.covers_rect(&rect(5, 0, 5, 10)));
    assert!(region.covers_rect(&rect(0, 10, 5, 10)));
    assert!(region.covers_rect(&rect(10, 5, 10, 5)));
    assert!(!region.covers_rect(&rect(5, 0, 5, 11)));
    assert!(!region.covers_rect(&rect(0, 10, 6, 10)));
}

/// Whether the unit cell with lower-left corner `(x, y)` lies in `rects`.
fn cell_in(rects: &[Rect<Dbu>], x: i64, y: i64) -> bool {
    rects.iter().any(|r| r.encloses(&rect(x, y, x + 1, y + 1)))
}

fn cells(region: &Region, side: i64) -> HashSet<(i64, i64)> {
    let rects: Vec<Rect<Dbu>> = region.rectangles().collect();
    (0..=side)
        .flat_map(|x| (0..=side).map(move |y| (x, y)))
        .filter(|&(x, y)| cell_in(&rects, x, y))
        .collect()
}

/// Maximal rectangles of the unit-cell set `inside`, by trying every
/// rectangle on the grid; on a grid a rectangle is maximal when no
/// one-cell extension of it fits.
fn brute_max_rectangles(inside: &HashSet<(i64, i64)>, side: i64) -> Vec<(i64, i64, i64, i64)> {
    let fits = |x0: i64, y0: i64, x1: i64, y1: i64| {
        (x0..x1).all(|x| (y0..y1).all(|y| inside.contains(&(x, y))))
    };
    let mut maximal = Vec::new();
    for x0 in 0..=side {
        for y0 in 0..=side {
            for x1 in x0 + 1..=side + 1 {
                for y1 in y0 + 1..=side + 1 {
                    let grows = fits(x0 - 1, y0, x1, y1)
                        || fits(x0, y0 - 1, x1, y1)
                        || fits(x0, y0, x1 + 1, y1)
                        || fits(x0, y0, x1, y1 + 1);
                    if fits(x0, y0, x1, y1) && !grows {
                        maximal.push((x0, y0, x1, y1));
                    }
                }
            }
        }
    }
    sorted(maximal)
}

fn continues_straight(first: &Edge, second: &Edge) -> bool {
    let meets = first.to == second.from;
    meets && first.heading() == second.heading()
}

/// Checks that `slices` partition `region`: none empty, none overlapping,
/// together all of it.
fn assert_partition(region: &Region, slices: &[Rect<Dbu>]) -> Result<(), TestCaseError> {
    let area: i64 = slices.iter().map(|r| r.area().get()).sum();

    prop_assert!(
        slices
            .iter()
            .all(|r| r.width() > Dbu::ZERO && r.height() > Dbu::ZERO)
    );
    prop_assert_eq!(&Region::from_rects(slices.iter().copied()), region);
    prop_assert_eq!(area, region.area().get());
    Ok(())
}

#[test]
fn a_rectangle_without_width_adds_nothing_even_across_another() {
    let region = Region::from_rects([rect(1, 0, 1, 8), rect(0, 0, 2, 2)]);

    assert_eq!(region, Region::from_rects([rect(0, 0, 2, 2)]));
}

#[test]
fn a_region_with_area_is_not_empty() {
    assert!(!ell().is_empty());
}

#[test]
fn a_rectangle_only_touching_the_region_does_not_intersect_it() {
    let square = Region::from_rects([rect(0, 0, 10, 10)]);

    assert!(!square.intersects_rect(&rect(10, 0, 20, 10)));
    assert!(!square.intersects_rect(&rect(0, 10, 10, 20)));
    assert!(square.intersects_rect(&rect(9, 9, 20, 20)));
}

#[test]
fn covering_starts_at_the_slab_the_rectangle_starts_in() {
    let region = Region::from_rects([rect(0, 0, 10, 5), rect(10, 0, 20, 10)]);

    assert!(region.covers_rect(&rect(10, 0, 20, 10)));
    assert!(region.covers_rect(&rect(0, 0, 20, 5)));
    assert!(!region.covers_rect(&rect(5, 0, 20, 10)));
}

#[test]
fn rectangles_meeting_at_either_diagonal_corner_are_separate_pieces() {
    let rising = Region::from_rects([rect(0, 0, 5, 5), rect(5, 5, 10, 10)]);
    let falling = Region::from_rects([rect(0, 5, 5, 10), rect(5, 0, 10, 5)]);

    assert_eq!(rising.components().len(), 2);
    assert_eq!(falling.components().len(), 2);
}

#[test]
fn a_union_merges_the_spans_it_joins() {
    let big = Region::from_rects([rect(0, 0, 10, 10)]);
    let split = Region::from_rects([rect(0, 0, 10, 4), rect(0, 6, 10, 10)]);

    assert_eq!(big.union(&split), big);
}

/// The 4-connected groups of `cells`, each as a set.
fn flood_fill(cells: &HashSet<(i64, i64)>) -> Vec<HashSet<(i64, i64)>> {
    let mut left = cells.clone();
    let mut groups = Vec::new();
    while let Some(&start) = left.iter().next() {
        let mut group = HashSet::new();
        let mut stack = vec![start];
        while let Some((x, y)) = stack.pop() {
            if left.remove(&(x, y)) {
                let _ = group.insert((x, y));
                stack.extend([(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]);
            }
        }
        groups.push(group);
    }
    groups
}

#[test]
fn a_rectangle_meeting_the_region_only_along_an_edge_shares_no_area() {
    let region = ell();

    let beside = [rect(-5, 0, 0, 5), rect(0, -5, 5, 0), rect(10, 0, 15, 2)];

    assert!(
        beside.iter().all(|r| !region.intersects_rect(r)),
        "{beside:?}"
    );
    assert!(region.intersects_rect(&rect(-5, 0, 1, 5)));
    assert!(region.intersects_rect(&rect(0, -5, 5, 1)));
}

/// One to `count` rectangles with area inside the `side` by `side` square
/// at the origin.
fn rects(count: usize, side: i64) -> impl Strategy<Value = Vec<Rect<Dbu>>> {
    prop::collection::vec(probe(side, 1), 1..=count)
}

/// A rectangle inside the `side` by `side` square at the origin, at least
/// `least` wide and high.
fn probe(side: i64, least: i64) -> impl Strategy<Value = Rect<Dbu>> {
    (0..=side - least, 0..=side - least)
        .prop_flat_map(move |(x, y)| (Just(x), Just(y), least..=side - x, least..=side - y))
        .prop_map(|(x, y, w, h)| rect(x, y, x + w, y + h))
}

fn region_config(cases: u32) -> Config {
    Config {
        cases,
        rng_seed: RngSeed::Fixed(73),
        failure_persistence: None,
        ..Config::default()
    }
}

proptest! {
    #![proptest_config(region_config(300))]

    #[test]
    fn boolean_operations_match_cell_by_cell_membership(ra in rects(4, 12), rb in rects(4, 12)) {
        let (a, b) = (Region::from_rects(ra), Region::from_rects(rb));
        let (ca, cb) = (cells(&a, 12), cells(&b, 12));

        prop_assert_eq!(cells(&a.union(&b), 12), &ca | &cb);
        prop_assert_eq!(cells(&a.intersection(&b), 12), &ca & &cb);
        prop_assert_eq!(cells(&a.difference(&b), 12), &ca - &cb);
        prop_assert_eq!(a.area().get(), i64::try_from(ca.len()).unwrap());
    }

    #[test]
    fn edges_add_up_to_the_boundary_of_each_cell_set(rs in rects(4, 10)) {
        let region = Region::from_rects(rs);
        let inside = cells(&region, 10);
        let boundary: i64 = inside
            .iter()
            .map(|&(x, y)| {
                [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
                    .iter()
                    .filter(|c| !inside.contains(c))
                    .count()
            })
            .sum::<usize>()
            .try_into()
            .unwrap();

        let length: i64 = region.edges().iter().map(|e| e.length().get()).sum();

        prop_assert_eq!(length, boundary);
    }

    #[test]
    fn every_edge_ends_where_another_starts(rs in rects(4, 10)) {
        let edges = Region::from_rects(rs).edges();

        let mut starts: Vec<(i64, i64)> = edges
            .iter()
            .map(|e| (e.from.x.get(), e.from.y.get()))
            .collect();
        let mut ends: Vec<(i64, i64)> =
            edges.iter().map(|e| (e.to.x.get(), e.to.y.get())).collect();
        starts.sort_unstable();
        ends.sort_unstable();

        prop_assert_eq!(starts, ends);
        prop_assert!(edges.iter().all(|e| e.length() > Dbu::ZERO));
    }

    #[test]
    fn no_edge_continues_straight_into_another(rs in rects(4, 10)) {
        let edges = Region::from_rects(rs).edges();

        let joins_straight = edges
            .iter()
            .any(|a| edges.iter().any(|b| a != b && continues_straight(a, b)));

        prop_assert!(!joins_straight);
    }

    #[test]
    fn components_partition_the_region(rs in rects(5, 12)) {
        let region = Region::from_rects(rs);

        let pieces = region.components();

        let joined = pieces.iter().fold(Region::new(), |acc, p| acc.union(p));
        let total: i64 = pieces.iter().map(|p| p.area().get()).sum();
        prop_assert_eq!(&joined, &region);
        prop_assert_eq!(total, region.area().get());
    }

    #[test]
    fn components_match_the_edge_connected_groups_of_cells(rs in rects(6, 12)) {
        let region = Region::from_rects(rs);

        let mut pieces: Vec<Vec<(i64, i64)>> = region
            .components()
            .iter()
            .map(|c| sorted(cells(c, 12).into_iter().collect()))
            .collect();

        let mut groups: Vec<Vec<(i64, i64)>> = flood_fill(&cells(&region, 12))
            .into_iter()
            .map(|g| sorted(g.into_iter().collect()))
            .collect();
        pieces.sort();
        groups.sort();
        prop_assert_eq!(pieces, groups);
    }

    #[test]
    fn covering_matches_cell_by_cell_membership(rs in rects(4, 12), probe in probe(12, 1)) {
        let region = Region::from_rects(rs);
        let inside = cells(&region, 12);

        let expected = (probe.min.x.get()..probe.max.x.get())
            .all(|x| (probe.min.y.get()..probe.max.y.get()).all(|y| inside.contains(&(x, y))));

        prop_assert_eq!(region.covers_rect(&probe), expected, "{:?}", probe);
    }

    #[test]
    fn covering_agrees_with_the_regions_rectangles_covering(rs in rects(3, 12), probe in probe(12, 0)) {
        let region = Region::from_rects(rs);
        let pieces: Vec<Rect<Dbu>> = region.rectangles().collect();

        prop_assert_eq!(
            region.covers_rect(&probe),
            probe.covered_by(&pieces),
            "{:?} in {:?}", probe, pieces
        );
    }
}

proptest! {
    #![proptest_config(region_config(1000))]

    #[test]
    fn slices_partition_random_regions(rs in rects(5, 12)) {
        let region = Region::from_rects(rs);

        assert_partition(&region, &region.horizontal_slices())?;
        assert_partition(&region, &region.vertical_slices())?;
    }

    #[test]
    fn slices_partition_random_regions_with_holes(outer in rects(4, 14), holes in rects(4, 14)) {
        let region = Region::from_rects(outer).difference(&Region::from_rects(holes));

        assert_partition(&region, &region.horizontal_slices())?;
        assert_partition(&region, &region.vertical_slices())?;
    }
}

proptest! {
    #![proptest_config(region_config(60))]

    #[test]
    fn maximal_rectangles_match_an_exhaustive_search(rs in rects(3, 8)) {
        let region = Region::from_rects(rs);

        let got = sorted(region.max_rectangles().iter().map(key).collect());

        prop_assert_eq!(got, brute_max_rectangles(&cells(&region, 8), 8));
    }
}
