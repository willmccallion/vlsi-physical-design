//! PNG renderings of placed and routed designs.

use image::{Rgb, RgbImage, Rgba, RgbaImage};
use imageproc::drawing::{
    Blend, draw_filled_rect_mut, draw_hollow_rect_mut, draw_line_segment_mut,
};
use imageproc::rect::Rect as ImageRect;
use pare_common::db::design::{NetlistDB, RouteElem};
use pare_common::db::indices::{CellId, IdVec, LayerId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::drc::{Layout, LayoutShape, OwnerIndex};
use std::path::Path;

/// An image fitted to an area of the design: its pixels per database unit
/// and its size, the same scale along both axes.
#[derive(Clone, Copy, Debug)]
struct Fit {
    scale: f64,
    width: u32,
    height: u32,
}

impl Fit {
    /// `size` pixels along the longer side of `area`, and the other side in
    /// proportion, at least one pixel each.
    fn new(area: &Rect<Dbu>, size: u32) -> Self {
        let longer = area.width().max(area.height()).to_f64().max(1.0);
        let scale = f64::from(size) / longer;
        let pixels = |length: Dbu| ((length.to_f64() * scale).ceil() as u32).max(1);
        Self {
            scale,
            width: pixels(area.width()),
            height: pixels(area.height()),
        }
    }
}

/// Maps design coordinates to image pixels, with Y pointing down.
struct PixelMap {
    min: Point<f64>,
    scale: f64,
    height: f64,
}

impl PixelMap {
    /// The map of the die of `db` onto the image `fit` sizes for it.
    fn new(db: &NetlistDB, fit: Fit) -> Self {
        let die = db.die_area;
        Self {
            min: Point::new(die.min.x.to_f64(), die.min.y.to_f64()),
            scale: fit.scale,
            height: f64::from(fit.height),
        }
    }

    /// The pixel `p` falls on.
    fn map(&self, p: Point<Dbu>) -> Point<f64> {
        Point::new(
            (p.x.to_f64() - self.min.x) * self.scale,
            (p.y.to_f64() - self.min.y).mul_add(-self.scale, self.height),
        )
    }

    /// The pixels of `cell`, at least `min_px` along each side.
    fn rect(&self, db: &NetlistDB, cell: CellId, min_px: f64) -> ImageRect {
        let b = db.cell_bbox(cell);
        let bottom_left = self.map(b.min);
        let w = (b.width().to_f64() * self.scale).max(min_px);
        let h = (b.height().to_f64() * self.scale).max(min_px);
        ImageRect::at(bottom_left.x as i32, (bottom_left.y - h) as i32).of_size(w as u32, h as u32)
    }
}

/// Draws every cell as a filled rectangle, fixed cells in a separate color,
/// into an image of `size` pixels along the die's longer side saved at
/// `path`.
///
/// # Errors
///
/// When the image cannot be saved.
pub(crate) fn draw_placement(db: &NetlistDB, path: &Path, size: u32) -> image::ImageResult<()> {
    let fit = Fit::new(&db.die_area, size);
    let mut img = RgbImage::from_pixel(fit.width, fit.height, Rgb([20, 20, 20]));
    paint_cells(db, &PixelMap::new(db, fit), &mut img);
    img.save(path)
}

fn paint_cells(db: &NetlistDB, map: &PixelMap, img: &mut RgbImage) {
    let color_cell = Rgb([180, 60, 60]);
    let color_fixed = Rgb([180, 120, 40]);
    for (id, cell) in db.cells.iter_enumerated() {
        let color = if cell.status.is_movable() {
            color_cell
        } else {
            color_fixed
        };
        draw_filled_rect_mut(img, map.rect(db, id, 2.0), color);
    }
}

/// The fewest pixels across a routed design is drawn at, so thin wires
/// stay visible.
const MIN_ROUTED_PIXELS: u32 = 4000;

/// Draws cells, wires colored by layer, vias and pins into an image of
/// `size` pixels along the die's longer side saved at `path`.
///
/// Draws at [`MIN_ROUTED_PIXELS`] or more, then downsamples if a smaller
/// image was asked for.
///
/// # Errors
///
/// When the image cannot be saved.
pub(crate) fn draw_routed_design(db: &NetlistDB, path: &Path, size: u32) -> image::ImageResult<()> {
    let drawn = Fit::new(&db.die_area, size.max(MIN_ROUTED_PIXELS));
    let mut img = RgbaImage::from_pixel(drawn.width, drawn.height, Rgba([0, 0, 0, 255]));
    paint_routed(db, &PixelMap::new(db, drawn), &mut img);
    let wanted = Fit::new(&db.die_area, size);
    if (wanted.width, wanted.height) == (drawn.width, drawn.height) {
        return img.save(path);
    }
    image::imageops::resize(
        &img,
        wanted.width,
        wanted.height,
        image::imageops::FilterType::Lanczos3,
    )
    .save(path)
}

fn paint_routed(db: &NetlistDB, map: &PixelMap, img: &mut RgbaImage) {
    let cell_color = Rgba([35, 35, 40, 255]);
    for cell in db.cells.ids() {
        draw_filled_rect_mut(img, map.rect(db, cell, 1.0), cell_color);
    }

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
        let (a, b) = (map.map(from), map.map(to));
        let color = routed_color(layer);
        draw_line_segment_mut(
            img,
            (a.x as f32, a.y as f32),
            (b.x as f32, b.y as f32),
            color,
        );
        if layer.index() >= 2 {
            draw_line_segment_mut(
                img,
                ((a.x + 0.5) as f32, (a.y + 0.5) as f32),
                ((b.x + 0.5) as f32, (b.y + 0.5) as f32),
                color,
            );
        }
    }

    for net in &db.nets {
        for e in &net.routing {
            match *e {
                RouteElem::Via { at, .. } => {
                    let at = map.map(at);
                    let rect = ImageRect::at(at.x as i32 - 1, at.y as i32 - 1).of_size(3, 3);
                    draw_filled_rect_mut(img, rect, Rgba([255, 255, 255, 200]));
                }
                RouteElem::Patch { layer, rect } => {
                    let (a, b) = (map.map(rect.min), map.map(rect.max));
                    let (w, h) = ((b.x - a.x).abs().max(1.0), (b.y - a.y).abs().max(1.0));
                    let area = ImageRect::at(a.x.min(b.x) as i32, a.y.min(b.y) as i32)
                        .of_size(w as u32, h as u32);
                    draw_filled_rect_mut(img, area, routed_color(layer));
                }
                RouteElem::Wire { .. } => {}
            }
        }
    }

    let pin_color = Rgba([255, 255, 255, 255]);
    for pin in db.pins.ids() {
        let at = map.map(db.pin_position(pin));
        let rect = ImageRect::at(at.x as i32, at.y as i32).of_size(2, 2);
        draw_filled_rect_mut(img, rect, pin_color);
    }
}

/// The opacity of routed metal in the routed design's image.
const ROUTED_ALPHA: u8 = 170;

/// The colour of routed metal on `layer` in the routed design's image.
const fn routed_color(layer: LayerId) -> Rgba<u8> {
    let [r, g, b] = LAYER_COLORS[layer.index() % LAYER_COLORS.len()];
    Rgba([r, g, b, ROUTED_ALPHA])
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
pub(crate) fn draw_area(
    layout: &Layout,
    area: Rect<Dbu>,
    layers: &IdVec<LayerId, bool>,
    highlight: &[OwnerIndex],
    markers: &[Rect<Dbu>],
    path: &Path,
    size: u32,
) -> image::ImageResult<()> {
    let shown = |l: LayerId| layers.get(l).copied().unwrap_or(true);
    let Fit {
        scale,
        width,
        height,
    } = Fit::new(&area, size);
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
    for l in layout.metal_layer_ids().filter(|&l| shown(l)) {
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
                let grey = 70 + (l.index() * 12).min(80) as u8;
                Rgba([grey, grey, grey, alpha])
            } else {
                let [r, g, b] = LAYER_COLORS[l.index() % LAYER_COLORS.len()];
                Rgba([r, g, b, alpha])
            };
            draw_filled_rect_mut(&mut canvas, rect, color);
        }
        if l.above().is_some_and(shown) {
            for s in layout.cut(l.cut_above()) {
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
    canvas.0.save(path)
}
