use super::*;
use crate::testing::{scratch_dir, tiny};
use pare_common::testing::assert_golden;
use pare_verify::drc::Owner;
use std::fmt::Write as _;

/// Pixels along the longer side of the area images the tests draw.
const AREA_PIXELS: u32 = 32;

/// The pixels of the PNG at `path`, a row per line in hex RGBA.
fn pixel_rows(path: &Path) -> String {
    let img = image::open(path).unwrap().to_rgba8();
    let mut rows = String::new();
    for row in img.rows() {
        for pixel in row {
            for channel in pixel.0 {
                let _ = write!(rows, "{channel:02x}");
            }
        }
        rows.push('\n');
    }
    rows
}

fn golden_dir() -> std::path::PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}

#[test]
fn an_area_draws_every_layer_evenly_without_highlights() {
    let db = tiny();
    let layout = Layout::from_db(&db);
    let every_layer = IdVec::filled(true, db.library.routing_layers.len());
    let path = scratch_dir("area-plain").join("area.png");
    let view = AreaView {
        layout: &layout,
        area: db.die_area,
        layers: &every_layer,
        highlight: &[],
        markers: &[],
    };

    draw_area(&view, &path, AREA_PIXELS).unwrap();

    assert_golden(&golden_dir(), "area_plain.txt", &pixel_rows(&path));
}

#[test]
fn an_area_draws_its_highlighted_net_bright_and_its_markers_red() {
    let db = tiny();
    let layout = Layout::from_db(&db);
    let net = layout.owner_index(Owner::Net(db.net_id("n3").unwrap()));
    let highlight: Vec<OwnerIndex> = net.into_iter().collect();
    let mut layers = IdVec::filled(true, db.library.routing_layers.len());
    let top = db.library.routing_layers.ids().last().unwrap();
    layers[top] = false;
    let marker = Rect::new(
        Point::new(Dbu::new(1000), Dbu::new(1000)),
        Point::new(Dbu::new(3000), Dbu::new(2000)),
    );
    let path = scratch_dir("area-highlight").join("area.png");
    let view = AreaView {
        layout: &layout,
        area: db.die_area,
        layers: &layers,
        highlight: &highlight,
        markers: &[marker],
    };

    draw_area(&view, &path, AREA_PIXELS).unwrap();

    assert!(!highlight.is_empty());
    assert_golden(&golden_dir(), "area_highlight.txt", &pixel_rows(&path));
}
