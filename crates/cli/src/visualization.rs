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
use pare_verify::drc::{Layout, LayoutShape, OwnerIndex};
use std::path::Path;
#[cfg(test)]
use std::path::PathBuf;

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

/// One area of a layout to draw at its true geometry.
///
/// Fixed metal is grey and routed metal has a colour per layer; cuts are
/// white. When `highlight` names owners, each gets a colour of its own with
/// every shape outlined, and every other owner is dimmed. `markers` are
/// outlined in red. Only the routing layers `layers` marks, by index, are
/// drawn, with the cuts between two of them; every layer when it is empty.
pub(crate) struct AreaView<'a> {
    pub(crate) layout: &'a Layout,
    pub(crate) area: Rect<Dbu>,
    pub(crate) layers: &'a IdVec<LayerId, bool>,
    pub(crate) highlight: &'a [OwnerIndex],
    pub(crate) markers: &'a [Rect<Dbu>],
}

/// Draws `view` as a PNG of `size` pixels along the area's longer side.
///
/// # Errors
///
/// When the image cannot be written.
pub(crate) fn draw_area(view: &AreaView<'_>, path: &Path, size: u32) -> image::ImageResult<()> {
    let shown = |l: LayerId| view.layers.get(l).copied().unwrap_or(true);
    let Fit {
        scale,
        width,
        height,
    } = Fit::new(&view.area, size);
    let mut canvas = Blend(RgbaImage::from_pixel(
        width,
        height,
        Rgba([10, 10, 14, 255]),
    ));
    let pixels = AreaPixels {
        area: view.area,
        scale,
    };
    let highlight = Highlight(view.highlight);
    for l in view.layout.metal_layer_ids().filter(|&l| shown(l)) {
        paint_area_metal(&mut canvas, view.layout.metal(l), l, &pixels, &highlight);
        if l.above().is_some_and(shown) {
            paint_area_cuts(
                &mut canvas,
                view.layout.cut(l.cut_above()),
                &pixels,
                &highlight,
            );
        }
    }
    paint_markers(&mut canvas, view.markers, &pixels);
    canvas.0.save(path)
}

/// Maps an area of the design onto an image of it, `scale` pixels per
/// database unit, with Y pointing down.
struct AreaPixels {
    area: Rect<Dbu>,
    scale: f64,
}

impl AreaPixels {
    /// The pixels of the part of `r` inside the area, at least one each
    /// way; none when it lies outside.
    fn rect(&self, r: &Rect<Dbu>) -> Option<ImageRect> {
        let (area, scale) = (self.area, self.scale);
        let clipped = r.intersection(&area)?;
        let x0 = (clipped.min.x - area.min.x).to_f64() * scale;
        let x1 = (clipped.max.x - area.min.x).to_f64() * scale;
        let y0 = (area.max.y - clipped.max.y).to_f64() * scale;
        let y1 = (area.max.y - clipped.min.y).to_f64() * scale;
        Some(
            ImageRect::at(x0 as i32, y0 as i32)
                .of_size(((x1 - x0) as u32).max(1), ((y1 - y0) as u32).max(1)),
        )
    }
}

/// The owners drawn bright, each in a colour of its own, in order.
struct Highlight<'a>(&'a [OwnerIndex]);

impl Highlight<'_> {
    /// How opaque a shape of `owner` is drawn: brighter when highlighted,
    /// dimmer when others are.
    fn alpha(&self, owner: OwnerIndex) -> u8 {
        if self.0.is_empty() {
            130
        } else if self.0.contains(&owner) {
            220
        } else {
            45
        }
    }

    /// The colour of `owner`, when it is highlighted.
    fn color(&self, owner: OwnerIndex) -> Option<[u8; 3]> {
        let k = self.0.iter().position(|&o| o == owner)?;
        Some(NET_COLORS[k % NET_COLORS.len()])
    }
}

/// Paints the metal `shapes` of routing layer `l`.
fn paint_area_metal(
    canvas: &mut Blend<RgbaImage>,
    shapes: &[LayoutShape],
    l: LayerId,
    pixels: &AreaPixels,
    highlight: &Highlight<'_>,
) {
    for s in shapes {
        let Some(rect) = pixels.rect(&s.rect) else {
            continue;
        };
        let alpha = highlight.alpha(s.owner);
        if let Some([r, g, b]) = highlight.color(s.owner) {
            draw_filled_rect_mut(canvas, rect, Rgba([r, g, b, alpha]));
            draw_hollow_rect_mut(canvas, rect, Rgba([r / 2, g / 2, b / 2, 255]));
            continue;
        }
        let color = if s.is_fixed() {
            let grey = 70 + (l.index() * 12).min(80) as u8;
            Rgba([grey, grey, grey, alpha])
        } else {
            let [r, g, b] = LAYER_COLORS[l.index() % LAYER_COLORS.len()];
            Rgba([r, g, b, alpha])
        };
        draw_filled_rect_mut(canvas, rect, color);
    }
}

/// Paints the cut `shapes`, white.
fn paint_area_cuts(
    canvas: &mut Blend<RgbaImage>,
    shapes: &[LayoutShape],
    pixels: &AreaPixels,
    highlight: &Highlight<'_>,
) {
    for s in shapes {
        if let Some(rect) = pixels.rect(&s.rect) {
            let alpha = highlight.alpha(s.owner);
            draw_filled_rect_mut(canvas, rect, Rgba([255, 255, 255, alpha]));
        }
    }
}

/// Outlines each of `markers` in red, twice over, on a red tint.
fn paint_markers(canvas: &mut Blend<RgbaImage>, markers: &[Rect<Dbu>], pixels: &AreaPixels) {
    for m in markers {
        if let Some(rect) = pixels.rect(m) {
            draw_filled_rect_mut(canvas, rect, Rgba([255, 30, 30, 70]));
            draw_hollow_rect_mut(canvas, rect, Rgba([255, 40, 40, 255]));
            let outer = ImageRect::at(rect.left() - 1, rect.top() - 1)
                .of_size(rect.width() + 2, rect.height() + 2);
            draw_hollow_rect_mut(canvas, outer, Rgba([255, 40, 40, 255]));
        }
    }
}

#[cfg(test)]
mod tests {
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
            Point::new(Dbu(1000), Dbu(1000)),
            Point::new(Dbu(3000), Dbu(2000)),
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
}
