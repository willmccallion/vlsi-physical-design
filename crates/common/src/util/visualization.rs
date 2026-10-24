//! PNG renderings of placed and routed designs.

use crate::db::design::{NetlistDB, RouteElem};
use crate::db::indices::CellId;
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use crate::verify::drc::{Layout, LayoutShape, OwnerIndex};
use image::{Rgb, RgbImage, Rgba, RgbaImage};
use imageproc::drawing::{
    Blend, draw_filled_rect_mut, draw_hollow_rect_mut, draw_line_segment_mut,
};
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
            RouteElem::Wire {
                layer, from, to, ..
            } => Some((layer, from, to)),
            RouteElem::Via { .. } | RouteElem::Patch { .. } => None,
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
            match *e {
                RouteElem::Via { at, .. } => {
                    let (x, y) = map.map(at);
                    let rect = ImageRect::at(x as i32 - 1, y as i32 - 1).of_size(3, 3);
                    draw_filled_rect_mut(&mut img, rect, Rgba([255, 255, 255, 200]));
                }
                RouteElem::Patch { layer, rect } => {
                    let (x1, y1) = map.map(rect.min);
                    let (x2, y2) = map.map(rect.max);
                    let (w, h) = ((x2 - x1).abs().max(1.0), (y2 - y1).abs().max(1.0));
                    let area = ImageRect::at(x1.min(x2) as i32, y1.min(y2) as i32)
                        .of_size(w as u32, h as u32);
                    let color = colors[layer.index().min(colors.len() - 1)];
                    draw_filled_rect_mut(&mut img, area, color);
                }
                RouteElem::Wire { .. } => {}
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

/// Colours of routed metal, by routing layer from the lowest.
const LAYER_COLORS: [[u8; 3]; 10] = [
    [70, 110, 255],
    [255, 60, 90],
    [40, 220, 110],
    [255, 200, 30],
    [190, 80, 255],
    [30, 220, 240],
    [255, 140, 40],
    [160, 230, 60],
    [255, 100, 200],
    [120, 160, 255],
];

/// Colours of highlighted nets, in the order they are given.
const NET_COLORS: [[u8; 3]; 6] = [
    [255, 235, 80],
    [60, 230, 255],
    [255, 130, 40],
    [140, 255, 90],
    [255, 90, 220],
    [240, 240, 240],
];

/// Draws one area of `layout` at its true geometry, as a PNG of `size`
/// pixels along the area's longer side.
///
/// Fixed metal is grey and routed metal has a colour per layer; cuts are
/// white. When `highlight` names owners, each gets a colour of its own with
/// every shape outlined, and every other owner is dimmed. `markers` are
/// outlined in red. Only the routing layers `layers` marks, by index, are
/// drawn, with the cuts between two of them; every layer when it is empty.
///
/// # Errors
///
/// When the image cannot be written.
pub fn draw_area(
    layout: &Layout,
    area: Rect<Dbu>,
    layers: &[bool],
    highlight: &[OwnerIndex],
    markers: &[Rect<Dbu>],
    filename: &Path,
    size: u32,
) -> image::ImageResult<()> {
    let shown = |l: usize| layers.get(l).copied().unwrap_or(true);
    let longer = area.width().max(area.height()).to_f64().max(1.0);
    let scale = f64::from(size) / longer;
    let width = ((area.width().to_f64() * scale).ceil() as u32).max(1);
    let height = ((area.height().to_f64() * scale).ceil() as u32).max(1);
    let mut canvas = Blend(RgbaImage::from_pixel(
        width,
        height,
        Rgba([10, 10, 14, 255]),
    ));
    let pixels = |r: &Rect<Dbu>| -> Option<ImageRect> {
        let clipped = r.intersection(&area)?;
        let x0 = (clipped.min.x - area.min.x).to_f64() * scale;
        let x1 = (clipped.max.x - area.min.x).to_f64() * scale;
        let y0 = (area.max.y - clipped.max.y).to_f64() * scale;
        let y1 = (area.max.y - clipped.min.y).to_f64() * scale;
        Some(
            ImageRect::at(x0 as i32, y0 as i32)
                .of_size(((x1 - x0) as u32).max(1), ((y1 - y0) as u32).max(1)),
        )
    };
    let emphasis = |s: &LayoutShape| -> u8 {
        if highlight.is_empty() {
            130
        } else if highlight.contains(&s.owner) {
            220
        } else {
            45
        }
    };
    for l in (0..layout.metal_layers()).filter(|&l| shown(l)) {
        for s in layout.metal(l) {
            let Some(rect) = pixels(&s.rect) else {
                continue;
            };
            let alpha = emphasis(s);
            if let Some(k) = highlight.iter().position(|&o| o == s.owner) {
                let [r, g, b] = NET_COLORS[k % NET_COLORS.len()];
                draw_filled_rect_mut(&mut canvas, rect, Rgba([r, g, b, alpha]));
                draw_hollow_rect_mut(&mut canvas, rect, Rgba([r / 2, g / 2, b / 2, 255]));
                continue;
            }
            let color = if s.is_fixed() {
                let grey = 70 + u8::try_from(l * 12).unwrap_or(80).min(80);
                Rgba([grey, grey, grey, alpha])
            } else {
                let [r, g, b] = LAYER_COLORS[l % LAYER_COLORS.len()];
                Rgba([r, g, b, alpha])
            };
            draw_filled_rect_mut(&mut canvas, rect, color);
        }
        if l < layout.cut_layers() && shown(l + 1) {
            for s in layout.cut(l) {
                if let Some(rect) = pixels(&s.rect) {
                    draw_filled_rect_mut(&mut canvas, rect, Rgba([255, 255, 255, emphasis(s)]));
                }
            }
        }
    }
    for m in markers {
        if let Some(rect) = pixels(m) {
            draw_filled_rect_mut(&mut canvas, rect, Rgba([255, 30, 30, 70]));
            draw_hollow_rect_mut(&mut canvas, rect, Rgba([255, 40, 40, 255]));
            let outer = ImageRect::at(rect.left() - 1, rect.top() - 1)
                .of_size(rect.width() + 2, rect.height() + 2);
            draw_hollow_rect_mut(&mut canvas, outer, Rgba([255, 40, 40, 255]));
        }
    }
    canvas.0.save(filename)
}
