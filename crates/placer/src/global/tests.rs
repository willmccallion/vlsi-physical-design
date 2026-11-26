use super::*;

#[test]
fn bins_hold_about_one_average_cell_at_the_target_density() {
    // ibex in Nangate45: a 239 by 239 micron die, cells of 1.98 square
    // microns on average, placed at 0.65. OpenROAD picks 128 bins a side.
    let bins = bins_for_cell_area(57_358.0, 1.983, 0.65);

    assert_eq!(bins, 128);
}

#[test]
fn bins_double_once_a_quarter_of_the_ideal_count_is_reached() {
    let four_squares = 4.0 * 128.0 * 128.0;

    let one_short = bins_for_cell_area(four_squares - 1.0, 1.0, 1.0);
    let exact = bins_for_cell_area(four_squares, 1.0, 1.0);

    assert_eq!(one_short, 128);
    assert_eq!(exact, 256);
}

#[test]
fn bins_stay_within_the_grid_limits() {
    assert_eq!(bins_for_cell_area(1.0, 100.0, 0.5), MIN_BINS);
    assert_eq!(bins_for_cell_area(1e12, 1.0, 1.0), MAX_BINS);
    assert_eq!(bins_for_cell_area(1e6, 0.0, 0.5), MIN_BINS);
}

#[test]
fn the_default_target_density_adds_a_fifth_of_the_headroom() {
    assert!((density_of(0.65) - 0.73).abs() < 1e-9);
}

#[test]
fn the_default_target_density_starts_from_the_utilization_rounded_up() {
    assert!((density_of(0.0836) - 0.282).abs() < 1e-9);
}

#[test]
fn the_default_target_density_is_at_most_one() {
    assert!((density_of(0.995) - 1.0).abs() < f64::EPSILON);
}

#[test]
fn a_utilization_far_below_zero_gives_no_target_density() {
    assert_eq!(default_target_density(-1.0), None);
}

fn density_of(utilization: f64) -> f64 {
    default_target_density(utilization).unwrap().get()
}

fn row(y: f64, height: f64) -> Rect<f64> {
    Rect::new(Point::new(0.0, y), Point::new(100.0, y + height))
}

#[test]
fn rows_separated_by_channels_report_their_pitch() {
    let rows = [row(32.0, 64.0), row(160.0, 64.0), row(288.0, 64.0)];

    assert_eq!(channel_pitch(&rows), Some(128.0));
}

#[test]
fn abutting_rows_have_no_channels() {
    let rows = [row(0.0, 1.4), row(1.4, 1.4), row(2.8, 1.4)];

    assert_eq!(channel_pitch(&rows), None);
}

#[test]
fn fillers_bring_the_free_area_to_the_target_density() {
    let cells = vec![Point::new(2.0, 1.0); 10];

    let (size, count) = filler_layout(&cells, 100.0, 0.5);

    assert_eq!(size, Point::new(2.0, 1.0));
    assert_eq!(count, 15);
}

#[test]
fn a_full_design_gets_no_fillers() {
    let cells = vec![Point::new(2.0, 1.0); 10];

    let (size, count) = filler_layout(&cells, 20.0, 0.9);

    assert_eq!(size, Point::new(2.0, 1.0));
    assert_eq!(count, 0);
}

use crate::zones::Zones;
use pare_common::db::parser::{def, lef};

/// The tiny design's placement model.
fn tiny_model() -> PlacementModel {
    let fixture = |name: &str| {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../common/tests/fixtures")
            .join(name)
    };
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    let db = def::parse(&fixture("tiny.def"), library).unwrap();
    PlacementModel::from_db(&db, &Zones::new(&db).unwrap())
}

fn params() -> GlobalParams {
    GlobalParams {
        target_density: TargetDensity::new(0.7).unwrap(),
        target_overflow: Share::new(0.1).unwrap(),
        max_iterations: 0,
        initial_rounds: 0,
        bins: 16,
        seed: 1,
        routability: None,
        timing: None,
    }
}

/// A placement of the tiny design, and the design after a repair that
/// doubled the width of its first movable cell and added a copy of its
/// second where the copy's position says.
struct Repair {
    before: Setup,
    after: PlacementModel,
    resized: CellId,
    added: CellId,
}

fn repair() -> Repair {
    let model = tiny_model();
    let mut rng = StdRng::seed_from_u64(1);
    let before = Setup::new(&model, &params(), &mut rng).unwrap();
    let mut after = model;
    let resized = before.movable[ObjectId::new(0)];
    after.sizes[resized].x *= 2.0;
    let copied = before.movable[ObjectId::new(1)];
    let added = after.sizes.push(after.sizes[copied]);
    let _ = after.density_sizes.push(after.density_sizes[copied]);
    let _ = after.movable.push(true);
    let _ = after.kinds.push(after.kinds[copied]);
    let _ = after.positions.push(Point::new(1.0, 1.0));
    let _ = after.zone_of.push(after.zone_of[copied]);
    Repair {
        before,
        after,
        resized,
        added,
    }
}

/// The object of `cell` in `setup`.
fn object_of(setup: &Setup, cell: CellId) -> ObjectId {
    setup
        .movable
        .iter_enumerated()
        .find(|&(_, &c)| c == cell)
        .map(|(k, _)| k)
        .unwrap()
}

/// The setup rebuilt after `repair`, every cell before it kept.
fn rebuilt(repair: &Repair) -> (Setup, Remap) {
    let cells_before = repair.after.num_cells() - 1;
    let cell_map: IdVec<CellId, Option<CellId>> =
        (0..cells_before).map(|i| Some(CellId::new(i))).collect();
    let mut rng = StdRng::seed_from_u64(2);
    repair
        .before
        .rebuild(&repair.after, &cell_map, &params(), &mut rng)
        .unwrap()
}

#[test]
fn a_resized_cell_keeps_its_place_and_takes_its_new_size() {
    let repair = repair();

    let (after, remap) = rebuilt(&repair);

    let old = object_of(&repair.before, repair.resized);
    let new = object_of(&after, repair.resized);
    assert_eq!(remap.old_of_new[new], Some(old));
    assert_eq!(
        after.objects.centers[new],
        repair.before.objects.centers[old]
    );
    assert_eq!(after.objects.sizes[new], repair.after.sizes[repair.resized]);
}

#[test]
fn a_new_cell_starts_where_the_repair_put_it() {
    let repair = repair();

    let (after, remap) = rebuilt(&repair);

    let new = object_of(&after, repair.added);
    let size = repair.after.sizes[repair.added];
    assert_eq!(remap.old_of_new[new], None);
    assert_eq!(
        after.objects.centers[new],
        Point::new(1.0, 1.0) + size * 0.5
    );
}

#[test]
fn the_repair_adds_the_area_its_cells_gained() {
    let repair = repair();
    let area = |s: Point<f64>| s.x * s.y;
    let resized = repair.after.sizes[repair.resized];
    let gained = area(resized) / 2.0 + area(repair.after.sizes[repair.added]);

    let (_, remap) = rebuilt(&repair);

    assert!(
        (remap.added_area - gained).abs() < 1e-9,
        "{}",
        remap.added_area
    );
}

#[test]
fn each_field_keeps_its_fillers_where_they_were() {
    let repair = repair();

    let (after, remap) = rebuilt(&repair);

    let (old, new) = (&repair.before.fields, &after.fields);
    assert!(old.iter().any(|f| f.fillers.len() > 0));
    assert_eq!(new.len(), old.len());
    for (f, field) in new.iter_enumerated() {
        let was = remap.old_field_of_new[f].unwrap();
        assert_eq!(field.fillers.len(), old[was].fillers.len());
        for (k, j) in field.fillers.iter().zip(old[was].fillers.iter()) {
            assert_eq!(remap.old_of_new[k], Some(j));
            assert_eq!(after.objects.centers[k], repair.before.objects.centers[j]);
        }
    }
}

#[test]
fn a_field_whose_cells_gained_area_raises_its_target_density() {
    let repair = repair();

    let (after, remap) = rebuilt(&repair);

    let field = &after.fields[FieldId::new(0)];
    let was = &repair.before.fields[remap.old_field_of_new[FieldId::new(0)].unwrap()];
    let expected = was.grid.target() + remap.added_area / field.grid.free_area();
    assert!((field.grid.target() - expected).abs() < 1e-12);
    assert!(field.grid.target() > was.grid.target());
}
