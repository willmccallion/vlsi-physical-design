use super::*;
use crate::testing::{half_height_tiny, scratch_dir, tiny, tiny_config};

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
