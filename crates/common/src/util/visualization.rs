//! PNG renderings of placed and routed designs.

use crate::db::design::{NetlistDB, RouteElem};
use crate::db::indices::CellId;
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;
use image::{Rgb, RgbImage, Rgba, RgbaImage};
use imageproc::drawing::{draw_filled_rect_mut, draw_line_segment_mut};
use imageproc::rect::Rect as ImageRect;
use std::path::Path;

/// Maps design coordinates to image pixels, with Y pointing down.
struct PixelMap {
    min: Point<f64>,
    scale_x: f64,
    scale_y: f64,
    height: f64,
}

impl PixelMap {
    fn new(db: &NetlistDB, width: u32, height: u32) -> Option<Self> {
        let die = db.die_area;
        if die.width() <= Dbu::ZERO || die.height() <= Dbu::ZERO {
            return None;
        }
        Some(Self {
            min: Point::new(die.min.x.to_f64(), die.min.y.to_f64()),
            scale_x: f64::from(width) / die.width().to_f64(),
            scale_y: f64::from(height) / die.height().to_f64(),
            height: f64::from(height),
        })
    }

    fn map(&self, p: Point<Dbu>) -> (f64, f64) {
        (
            (p.x.to_f64() - self.min.x) * self.scale_x,
            (p.y.to_f64() - self.min.y).mul_add(-self.scale_y, self.height),
        )
    }

    fn rect(&self, db: &NetlistDB, cell: CellId, min_px: f64) -> ImageRect {
        let b = db.cell_bbox(cell);
        let (x, y_bot) = self.map(b.min);
        let w = (b.width().to_f64() * self.scale_x).max(min_px);
        let h = (b.height().to_f64() * self.scale_y).max(min_px);
        ImageRect::at(x as i32, (y_bot - h) as i32).of_size(w as u32, h as u32)
    }
}

/// Draws every cell as a filled rectangle, fixed cells in a separate color.
pub fn draw_placement(db: &NetlistDB, filename: &str, width: u32, height: u32) {
    let mut img = RgbImage::from_pixel(width, height, Rgb([20, 20, 20]));
    let Some(map) = PixelMap::new(db, width, height) else {
        return;
    };
    let color_cell = Rgb([180, 60, 60]);
    let color_fixed = Rgb([180, 120, 40]);
    for i in 0..db.num_cells() {
        let color = if db.cells[i].status.is_movable() {
            color_cell
        } else {
            color_fixed
        };
        draw_filled_rect_mut(&mut img, map.rect(db, CellId::new(i), 2.0), color);
    }
    if let Err(e) = img.save(Path::new(filename)) {
        log::error!("Failed to save placement image to {filename}: {e}");
    }
}

/// Draws cells, wires colored by layer, vias and pins.
///
/// Draws at a minimum of 4000x4000 pixels so thin wires stay visible, then
/// downsamples if a smaller image was requested.
pub fn draw_routed_design(db: &NetlistDB, filename: &str, width: u32, height: u32) {
    let w = width.max(4000);
    let h = height.max(4000);
    let mut img = RgbaImage::from_pixel(w, h, Rgba([0, 0, 0, 255]));
    let Some(map) = PixelMap::new(db, w, h) else {
        return;
    };

    let cell_color = Rgba([35, 35, 40, 255]);
    for i in 0..db.num_cells() {
        draw_filled_rect_mut(&mut img, map.rect(db, CellId::new(i), 1.0), cell_color);
    }

    let colors = [
        Rgba([0, 110, 255, 90]),
        Rgba([255, 20, 80, 90]),
        Rgba([0, 255, 100, 170]),
        Rgba([255, 215, 0, 170]),
        Rgba([180, 50, 255, 190]),
        Rgba([0, 240, 255, 190]),
    ];
    let mut wires: Vec<_> = db
        .nets
        .iter()
        .flat_map(|n| n.routing.iter())
        .filter_map(|e| match *e {
            RouteElem::Wire { layer, from, to } => Some((layer, from, to)),
            RouteElem::Via { .. } => None,
        })
        .collect();
    wires.sort_by_key(|w| w.0);
    for (layer, from, to) in wires {
        let (x1, y1) = map.map(from);
        let (x2, y2) = map.map(to);
        let color = colors[layer.index().min(colors.len() - 1)];
        draw_line_segment_mut(
            &mut img,
            (x1 as f32, y1 as f32),
            (x2 as f32, y2 as f32),
            color,
        );
        if layer.index() >= 2 {
            draw_line_segment_mut(
                &mut img,
                ((x1 + 0.5) as f32, (y1 + 0.5) as f32),
                ((x2 + 0.5) as f32, (y2 + 0.5) as f32),
                color,
            );
        }
    }

    for net in &db.nets {
        for e in &net.routing {
            if let RouteElem::Via { at, .. } = *e {
                let (x, y) = map.map(at);
                let rect = ImageRect::at(x as i32 - 1, y as i32 - 1).of_size(3, 3);
                draw_filled_rect_mut(&mut img, rect, Rgba([255, 255, 255, 200]));
            }
        }
    }

    let pin_color = Rgba([255, 255, 255, 255]);
    for pin in 0..db.pins.len() {
        let (px, py) = map.map(db.pin_position(crate::db::indices::PinId::new(pin)));
        let rect = ImageRect::at(px as i32, py as i32).of_size(2, 2);
        draw_filled_rect_mut(&mut img, rect, pin_color);
    }

    let result = if width < w || height < h {
        image::imageops::resize(&img, width, height, image::imageops::FilterType::Lanczos3)
            .save(Path::new(filename))
    } else {
        img.save(Path::new(filename))
    };
    if let Err(e) = result {
        log::error!("Failed to save routing image to {filename}: {e}");
    }
}
