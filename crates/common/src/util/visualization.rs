//! Visualization Utilities for Design Rendering.
//!
//! Provides functions to generate image visualizations of placed and routed
//! designs, including cell placements, wire routes, and routing congestion
//! heatmaps. Uses the image crate for pixel manipulation and file output.

use crate::db::core::NetlistDB;
use image::{Rgb, RgbImage, Rgba, RgbaImage};
use imageproc::drawing::{draw_filled_rect_mut, draw_line_segment_mut};
use imageproc::rect::Rect as ImageRect;
use std::path::Path;

/// Trait for routing grids that can provide congestion information.
///
/// Allows visualization code to query congestion ratios (occupancy/capacity)
/// for each grid cell to generate heatmaps. Implementations should aggregate
/// congestion across all layers for a given X,Y coordinate.
pub trait CongestionProvider {
    /// Returns the dimensions of the congestion grid (width, height).
    fn get_dims(&self) -> (u32, u32);
    /// Returns the congestion ratio for a given grid coordinate.
    ///
    /// The ratio represents occupancy divided by capacity. A value of 0.0
    /// indicates an empty cell, 1.0 indicates full capacity, and values
    /// greater than 1.0 indicate overflow (congestion violations).
    fn get_congestion_ratio(&self, x: u32, y: u32) -> f32;
}

/// Generates a congestion heatmap visualization of the routing grid.
///
/// Creates an RGB image where each pixel represents a grid cell, colored
/// according to its congestion ratio. The color scheme transitions from
/// black (empty) through blue, green, yellow, red, to white (extreme
/// congestion) to provide visual feedback on routing difficulty. The image
/// is saved to the specified filename.
pub fn draw_congestion_heatmap<G: CongestionProvider>(grid: &G, filename: &str) {
    let (w, h) = grid.get_dims();
    let mut img = RgbImage::new(w, h);

    for y in 0..h {
        for x in 0..w {
            let ratio = grid.get_congestion_ratio(x, y);

            let pixel = if ratio == 0.0 {
                Rgb([0, 0, 0])
            } else if ratio < 0.8 {
                let t = ratio / 0.8;
                Rgb([0, (255.0 * t) as u8, (255.0 * (1.0 - t)) as u8])
            } else if ratio <= 1.0 {
                let t = (ratio - 0.8) / 0.2;
                Rgb([(255.0 * t) as u8, 255, 0])
            } else if ratio <= 1.5 {
                let t = (ratio - 1.0) / 0.5;
                Rgb([255, (255.0 * (1.0 - t)) as u8, 0])
            } else {
                let t = ((ratio - 1.5) / 2.0).min(1.0);
                Rgb([255, (255.0 * t) as u8, (255.0 * t) as u8])
            };

            img.put_pixel(x, h - 1 - y, pixel);
        }
    }

    if let Err(e) = img.save(Path::new(filename)) {
        log::error!("Failed to save congestion map to {filename}: {e}");
    } else {
        log::info!("Dumped congestion map to {filename}");
    }
}

/// Generates a visualization of the cell placement.
///
/// Creates an RGB image showing all cells in the design as colored rectangles.
/// Fixed cells are drawn in a distinct color from movable cells to distinguish
/// pre-placed components. The image is scaled to fit the specified dimensions
/// while maintaining aspect ratio. Cells are drawn with minimum sizes to ensure
/// visibility even for very small cells.
pub fn draw_placement(db: &NetlistDB, filename: &str, width: u32, height: u32) {
    let mut img = RgbImage::new(width, height);
    image::imageops::replace(
        &mut img,
        &RgbImage::from_pixel(width, height, Rgb([20, 20, 20])),
        0,
        0,
    );

    let die_w = db.die_area.width();
    let die_h = db.die_area.height();
    if die_w <= 0.0 {
        return;
    }

    let scale_x = width as f64 / die_w;
    let scale_y = height as f64 / die_h;

    let map = |x: f64, y: f64| {
        (
            (x - db.die_area.min.x) * scale_x,
            (y - db.die_area.min.y).mul_add(-scale_y, height as f64),
        )
    };

    let color_cell = Rgb([180, 60, 60]);
    let color_fixed = Rgb([180, 120, 40]);

    for i in 0..db.num_cells() {
        let pos = db.positions[i];
        let cell = &db.cells[i];
        let (x, y_bot) = map(pos.x, pos.y);
        let w = (cell.width * scale_x).max(2.0);
        let h = (cell.height * scale_y).max(2.0);
        let rect = ImageRect::at(x as i32, (y_bot - h) as i32).of_size(w as u32, h as u32);

        if cell.is_fixed {
            draw_filled_rect_mut(&mut img, rect, color_fixed);
        } else {
            draw_filled_rect_mut(&mut img, rect, color_cell);
        }
    }
    let _ = img.save(Path::new(filename));
}

/// Generates a visualization of the complete routed design.
///
/// Creates an RGBA image showing cells as dark rectangles, wire segments
/// as colored lines (with different colors for each routing layer), vias
/// as white squares, and pins as small white dots. Each routing layer
/// is assigned a distinct color to visualize the layer assignment. The
/// image uses high resolution (minimum 4000x4000) to ensure wire details
/// are visible, then resizes if a smaller output size is requested.
pub fn draw_routed_design(db: &NetlistDB, filename: &str, width: u32, height: u32) {
    let w = width.max(4000);
    let h = height.max(4000);
    let mut img = RgbaImage::new(w, h);

    image::imageops::replace(
        &mut img,
        &RgbaImage::from_pixel(w, h, Rgba([0, 0, 0, 255])),
        0,
        0,
    );

    let die_w = db.die_area.width();
    let die_h = db.die_area.height();
    if die_w <= 0.0 {
        return;
    }

    let scale_x = w as f64 / die_w;
    let scale_y = h as f64 / die_h;

    let map = |x: f64, y: f64| {
        (
            (x - db.die_area.min.x) * scale_x,
            (y - db.die_area.min.y).mul_add(-scale_y, h as f64),
        )
    };

    let cell_color = Rgba([35, 35, 40, 255]);
    for i in 0..db.num_cells() {
        let pos = db.positions[i];
        let cell = &db.cells[i];
        let (x, y_bot) = map(pos.x, pos.y);
        let width = (cell.width * scale_x).max(1.0);
        let height = (cell.height * scale_y).max(1.0);
        let rect =
            ImageRect::at(x as i32, (y_bot - height) as i32).of_size(width as u32, height as u32);
        draw_filled_rect_mut(&mut img, rect, cell_color);
    }

    let colors = [
        Rgba([0, 110, 255, 90]),
        Rgba([255, 20, 80, 90]),
        Rgba([0, 255, 100, 170]),
        Rgba([255, 215, 0, 170]),
        Rgba([180, 50, 255, 190]),
        Rgba([0, 240, 255, 190]),
    ];
    let mut segments: Vec<_> = db
        .nets
        .iter()
        .flat_map(|n| n.route_segments.iter())
        .collect();
    segments.sort_by_key(|s| s.layer);

    for seg in segments {
        let (x1, y1) = map(seg.p1.x, seg.p1.y);
        let (x2, y2) = map(seg.p2.x, seg.p2.y);

        let is_via = (x1 - x2).abs() + (y1 - y2).abs() < 0.1;

        if is_via {
            if seg.layer >= 1 {
                let rect = ImageRect::at(x1 as i32 - 1, y1 as i32 - 1).of_size(3, 3);
                draw_filled_rect_mut(&mut img, rect, Rgba([255, 255, 255, 200]));
            }
        } else {
            let color_idx = (seg.layer as usize).min(colors.len() - 1);
            let color = colors[color_idx];

            draw_line_segment_mut(
                &mut img,
                (x1 as f32, y1 as f32),
                (x2 as f32, y2 as f32),
                color,
            );

            if seg.layer >= 2 {
                draw_line_segment_mut(
                    &mut img,
                    ((x1 + 0.5) as f32, (y1 + 0.5) as f32),
                    ((x2 + 0.5) as f32, (y2 + 0.5) as f32),
                    color,
                );
            }
        }
    }

    let pin_color = Rgba([255, 255, 255, 255]);
    for i in 0..db.num_cells() {
        let pos = db.positions[i];
        let cell = &db.cells[i];
        for &pin_id in &cell.pins {
            let pin_pos = db.get_pin_position(pin_id, &pos);
            let (px, py) = map(pin_pos.x, pin_pos.y);
            let rect = ImageRect::at(px as i32, py as i32).of_size(2, 2);
            draw_filled_rect_mut(&mut img, rect, pin_color);
        }
    }

    if width < w || height < h {
        let resized =
            image::imageops::resize(&img, width, height, image::imageops::FilterType::Lanczos3);
        let _ = resized.save(Path::new(filename));
    } else {
        let _ = img.save(Path::new(filename));
    }
}
