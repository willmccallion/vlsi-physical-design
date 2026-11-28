use super::*;

const MICRON: Dbu = Dbu::new(2000);

fn loads(points: &[(i64, i64)]) -> Vec<(PinId, Point<Dbu>)> {
    points
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| (PinId::new(i), Point::new(Dbu::new(x), Dbu::new(y))))
        .collect()
}

fn pins(ids: &[usize]) -> Vec<PinId> {
    ids.iter().map(|&i| PinId::new(i)).collect()
}

/// The pins of the leaves of `region`, lower halves first.
fn leaves(region: &LoadRegion) -> Vec<Vec<PinId>> {
    match &region.halves {
        None => vec![region.pins.clone()],
        Some(halves) => halves.iter().flat_map(leaves).collect(),
    }
}

#[test]
fn loads_within_the_limit_stay_in_one_region() {
    let region = LoadRegion::split(&loads(&[(0, 0), (9000, 9000)]), 2, MICRON);

    assert_eq!(region.halves, None);
    assert_eq!(region.pins, pins(&[0, 1]));
}

#[test]
fn a_box_wider_than_tall_is_halved_across_its_width() {
    let points = [(0, 0), (1000, 3000), (7000, 0), (8000, 3000)];

    let region = LoadRegion::split(&loads(&points), 2, MICRON);

    assert!(region.pins.is_empty());
    assert_eq!(leaves(&region), [pins(&[0, 1]), pins(&[2, 3])]);
}

#[test]
fn a_box_taller_than_wide_is_halved_across_its_height() {
    let points = [(0, 0), (3000, 8000), (0, 1000), (3000, 7000)];

    let region = LoadRegion::split(&loads(&points), 2, MICRON);

    assert_eq!(leaves(&region), [pins(&[0, 2]), pins(&[1, 3])]);
}

#[test]
fn a_load_on_the_cut_goes_to_the_lower_half() {
    let points = [(0, 0), (4000, 1000), (8000, 3000)];

    let region = LoadRegion::split(&loads(&points), 2, MICRON);

    assert_eq!(leaves(&region), [pins(&[0, 1]), pins(&[2])]);
}

#[test]
fn a_box_no_wider_than_the_minimum_is_not_halved() {
    let points = [(0, 0), (2000, 9000), (1000, 4000)];

    let region = LoadRegion::split(&loads(&points), 2, MICRON);

    assert_eq!(region.halves, None);
    assert_eq!(region.pins, pins(&[0, 1, 2]));
}

#[test]
fn a_box_no_taller_than_the_minimum_is_not_halved() {
    let points = [(0, 0), (9000, 2000), (4000, 1000)];

    let region = LoadRegion::split(&loads(&points), 2, MICRON);

    assert_eq!(region.halves, None);
}

#[test]
fn halves_are_halved_again_until_each_is_within_the_limit() {
    let points = [(0, 0), (2500, 3000), (5000, 0), (7500, 3000), (10_000, 0)];

    let region = LoadRegion::split(&loads(&points), 1, MICRON);

    let sizes: Vec<usize> = leaves(&region).iter().map(Vec::len).collect();
    assert!(sizes.iter().all(|&n| n <= 1), "{sizes:?}");
    assert_eq!(sizes.iter().sum::<usize>(), points.len());
}

#[test]
fn the_middle_of_a_box_is_truncated_toward_zero() {
    let points = [(-3, 0), (-2, 1), (-1, 0), (0, 1)];

    let region = LoadRegion::split(&loads(&points), 3, Dbu::ZERO);

    assert_eq!(leaves(&region), [pins(&[0, 1, 2]), pins(&[3])]);
}

#[test]
fn no_loads_make_one_empty_region() {
    let region = LoadRegion::split(&[], 4, MICRON);

    assert_eq!(region, LoadRegion::leaf(&[]));
}
