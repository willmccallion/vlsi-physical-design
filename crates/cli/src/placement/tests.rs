use super::*;
use crate::testing::{half_height_tiny, scratch_dir, timed, timed_config, tiny, tiny_config};

#[test]
fn cells_moved_or_turned_since_the_record_are_the_moved_ones() {
    let mut db = tiny();
    let before = CellPlaces::record(&db);
    let moved = db.cell_id("u1").unwrap();
    let turned = db.cell_id("u2").unwrap();

    db.cells[moved].location.x += Dbu::new(200);
    db.cells[turned].orient = Orientation::N;

    assert_eq!(before.moved(&db), vec![moved, turned]);
}

#[test]
fn a_placement_image_keeps_the_die_in_proportion() {
    let path = scratch_dir("placement-image").join("placed.png");

    draw_placement(&half_height_tiny(), &path).unwrap();

    assert_eq!(
        image::image_dimensions(&path).unwrap(),
        (PLACEMENT_IMAGE_PIXELS, PLACEMENT_IMAGE_PIXELS / 2)
    );
}

#[test]
fn nothing_moved_since_the_record_leaves_nothing_moved() {
    let db = tiny();

    assert_eq!(CellPlaces::record(&db).moved(&db), Vec::new());
}

fn model(db: &NetlistDB) -> PlacementModel {
    PlacementModel::from_db(db, &Zones::new(db).unwrap())
}

#[test]
fn the_density_takes_the_target_and_bins_the_config_sets() {
    let db = tiny();
    let config = tiny_config("[global_placement]\ntarget_density = 0.7\nbin_dimension = 8");

    let density = Density::of(&model(&db), &config.global_placement).unwrap();

    assert_eq!(density.target, TargetDensity::new(0.7).unwrap());
    assert_eq!(density.bins, 8);
}

#[test]
fn the_density_counts_the_cells_that_are_not_fixed() {
    let db = tiny();
    let config = tiny_config("");

    let density = Density::of(&model(&db), &config.global_placement).unwrap();

    assert_eq!(density.movable_cells, 6);
}

#[test]
fn without_a_target_or_bins_the_density_derives_them_from_the_utilization() {
    let db = tiny();
    let model = model(&db);
    let config = tiny_config("");

    let density = Density::of(&model, &config.global_placement).unwrap();

    let movable_area: f64 = model
        .density_sizes
        .iter()
        .zip(&model.movable)
        .filter(|(_, movable)| **movable)
        .map(|(size, _)| size.x * size.y)
        .sum();
    let target = global::default_target_density(movable_area / model.free_area()).unwrap();
    assert_eq!(density.target, target);
    assert_eq!(density.bins, global::default_bins(&model, target));
}

fn set_io_status(db: &mut NetlistDB, name: &str, status: PlacementStatus) {
    let pin = db.io_pin_id(name).unwrap();
    db.io_pins[pin].status = status;
}

#[test]
fn placed_io_pins_keep_their_places_without_layers_to_move_them_to() {
    let config = tiny_config("");

    let layers = io_layers_to_place(&config, &tiny(), PlacementStart::Scratch).unwrap();

    assert_eq!(layers, None);
}

#[test]
fn io_pins_are_placed_on_the_layers_the_config_names() {
    let db = tiny();
    let config = tiny_config("[io_placement]\nhorizontal_layer = \"M3\"\nvertical_layer = \"M2\"");

    let layers = io_layers_to_place(&config, &db, PlacementStart::Scratch).unwrap();

    let id = |name: &str| db.library.routing_layer_id(name).unwrap();
    assert_eq!(
        layers,
        Some(IoLayers {
            horizontal: id("M3"),
            vertical: id("M2"),
        })
    );
}

#[test]
fn a_given_placement_places_no_io_pins() {
    let mut db = tiny();
    set_io_status(&mut db, "out", PlacementStatus::Unplaced);
    let config = tiny_config("[io_placement]\nhorizontal_layer = \"M3\"\nvertical_layer = \"M2\"");

    let layers = io_layers_to_place(&config, &db, PlacementStart::Given).unwrap();

    assert_eq!(layers, None);
}

#[test]
fn fixed_io_pins_need_no_layers() {
    let mut db = tiny();
    set_io_status(&mut db, "out", PlacementStatus::Fixed);
    let config = tiny_config("[io_placement]\nhorizontal_layer = \"M3\"\nvertical_layer = \"M2\"");

    let layers = io_layers_to_place(&config, &db, PlacementStart::Scratch).unwrap();

    assert_eq!(layers, None);
}

#[test]
fn unplaced_io_pins_without_layers_to_place_them_on_are_an_error() {
    let mut db = tiny();
    set_io_status(&mut db, "out", PlacementStatus::Unplaced);

    let error = io_layers_to_place(&tiny_config(""), &db, PlacementStart::Scratch).unwrap_err();

    assert_eq!(
        error.to_string(),
        "1 I/O pins are unplaced; [io_placement] horizontal_layer and vertical_layer \
             name the layers to place them on"
    );
}

#[test]
fn a_given_placement_with_unplaced_cells_is_refused() {
    let error = keep_given_placement(&tiny()).unwrap_err();

    assert_eq!(
        error.to_string(),
        "2 cells of the given placement are unplaced"
    );
}

#[test]
fn a_given_placement_that_places_every_cell_is_kept() {
    let mut db = tiny();
    for cell in &mut db.cells {
        if cell.status == PlacementStatus::Unplaced {
            cell.status = PlacementStatus::Placed;
        }
    }

    keep_given_placement(&db).unwrap();
}

#[test]
fn only_the_pass_with_every_connection_starts_from_initial_placement() {
    let adjustments = IdVec::default();
    let full = Pass::Full {
        adjustments: &adjustments,
        repair: None,
    };

    assert_eq!(full.initial_rounds(), global::INITIAL_PLACE_ROUNDS);
    assert_eq!(Pass::SkipIo.initial_rounds(), 0);
}

/// Each cell of `db` keeping its id after a repair.
fn identity(db: &NetlistDB) -> Renumbering {
    Renumbering {
        cells: db.cells.ids().map(Some).collect(),
        nets: db.nets.ids().map(Some).collect(),
        pins: db.pins.ids().map(Some).collect(),
    }
}

/// The weight of `net` in `model`.
fn weight_of(model: &PlacementModel, net: NetId) -> f64 {
    model.nets.iter().find(|n| n.net == net).unwrap().weight
}

#[test]
fn a_timing_round_multiplies_the_weights_of_the_nets_it_weights() {
    let db = tiny();
    let n2 = db.net_id("n2").unwrap();
    let before = weight_of(&model(&db), n2);

    let reweighted = reweighted_model(&db, &identity(&db), vec![(n2, 3.0)]).unwrap();

    let (after, expected) = (weight_of(&reweighted.model, n2), before * 3.0);
    assert!((after - expected).abs() < 1e-12, "{expected} {after}");
}

#[test]
fn a_timing_round_leaves_the_weights_of_other_nets() {
    let db = tiny();
    let (n1, n2) = (db.net_id("n1").unwrap(), db.net_id("n2").unwrap());
    let before = weight_of(&model(&db), n1);

    let reweighted = reweighted_model(&db, &identity(&db), vec![(n2, 3.0)]).unwrap();

    let after = weight_of(&reweighted.model, n1);
    assert!((after - before).abs() < 1e-12, "{before} -> {after}");
}

#[test]
fn a_cell_centered_in_microns_has_its_corner_half_its_size_down_and_left() {
    let centers = IdVec::from(vec![Point::new(10.0, 20.0)]);
    let sizes = IdVec::from(vec![Point::new(2.0, 4.0)]);
    let units = DbuPerMicron::new(1000).unwrap();

    let corners = lower_left_corners(&centers, &sizes, units);

    assert_eq!(
        corners[CellId::new(0)],
        Point::new(Dbu::new(9000), Dbu::new(18000))
    );
}

#[test]
fn the_global_router_measures_the_congestion_of_every_gcell() {
    let db = timed();
    let config = timed_config("", &scratch_dir("gr-congestion"));
    let adjustments = FlowPlan::resolve(&config, &db).unwrap().layer_adjustments;
    let mut estimate = GlobalRouteCongestion::new(&db, &config, LeafClockPins::default()).unwrap();
    let grid = estimate.capacity_reduction(&adjustments);
    let centers = IdVec::filled(Point::new(10.0, 10.0), db.cells.len());

    let congestion = estimate.congestion(&centers);

    assert!(grid.nx * grid.ny > 1);
    assert_eq!(congestion.len(), grid.nx * grid.ny);
}
