//! Checks on each connected piece of an owner's metal alone, as `FlexGC`'s
//! metal shape checks.

use super::model::{GcPolygon, GcRect, Metal, Model};
use super::rules::{MetalRules, Rules};
use super::{Marker, Markers, Violation};
use crate::db::indices::LayerId;
use crate::db::library::LayerRef;
use crate::geom::dbu::Dbu;
use crate::geom::rect::Rect;
use rayon::prelude::*;

/// Checks every piece of metal on every routing layer.
pub(super) fn check(model: &Model<'_>, rules: &Rules) -> Markers {
    model
        .metal
        .iter()
        .enumerate()
        .map(|(l, metal)| {
            let check = PieceCheck {
                metal,
                layer: LayerId::new(l),
                rules: &rules.metal[l],
                grid: rules.manufacturing_grid,
            };
            let pieces = metal
                .polygons
                .par_iter()
                .fold(Markers::default, |mut found, piece| {
                    check.check_piece(piece, &mut found);
                    found
                })
                .reduce(Markers::default, Markers::merge);
            let rects = metal
                .rects
                .par_iter()
                .fold(Markers::default, |mut found, rect| {
                    check.off_grid(rect, &mut found);
                    found
                })
                .reduce(Markers::default, Markers::merge);
            pieces.merge(rects)
        })
        .fold(Markers::default(), Markers::merge)
}

/// The shape checks of one routing layer.
struct PieceCheck<'m> {
    metal: &'m Metal<'m>,
    layer: LayerId,
    rules: &'m MetalRules,
    grid: Option<Dbu>,
}

impl PieceCheck<'_> {
    fn check_piece(&self, piece: &GcPolygon, found: &mut Markers) {
        self.min_width(piece, found);
        self.min_area(piece, found);
        self.min_hole(piece, found);
    }

    /// Every horizontal slice of the piece must be at least the minimum
    /// width across X and every vertical slice across Y, unless the owner's
    /// fixed metal covers the slice.
    fn min_width(&self, piece: &GcPolygon, found: &mut Markers) {
        let min_width = self.rules.min_width;
        let narrow = piece
            .region
            .horizontal_slices()
            .into_iter()
            .filter(|r| r.width() < min_width)
            .chain(
                piece
                    .region
                    .vertical_slices()
                    .into_iter()
                    .filter(|r| r.height() < min_width),
            );
        for slice in narrow {
            if !self
                .metal
                .fixed_within(piece.owner, &slice)
                .covers_rect(&slice)
            {
                found.add(self.marker(Violation::MinWidth, 0, piece, slice), false);
            }
        }
    }

    /// A piece with any edge of fixed metal is exempt.
    fn min_area(&self, piece: &GcPolygon, found: &mut Markers) {
        let Some(min_area) = self.rules.min_area else {
            return;
        };
        if piece.area < min_area && !piece.has_fixed_edge {
            found.add(self.marker(Violation::MinArea, 0, piece, piece.bbox), false);
        }
    }

    /// Each hole of the piece must enclose at least the area of every
    /// minimum enclosed area rule.
    fn min_hole(&self, piece: &GcPolygon, found: &mut Markers) {
        if self.rules.min_enclosed_areas.is_empty() {
            return;
        }
        for hole in piece.region.holes() {
            let Some(bbox) = hole.bounding_box() else {
                continue;
            };
            let area = hole.area();
            for (k, &min) in self.rules.min_enclosed_areas.iter().enumerate() {
                if area < min {
                    let rule = u16::try_from(k).unwrap_or(u16::MAX);
                    found.add(self.marker(Violation::MinHole, rule, piece, bbox), false);
                }
            }
        }
    }

    /// A maximal rectangle with a side off the manufacturing grid is a
    /// violation where it holds routed metal.
    fn off_grid(&self, rect: &GcRect, found: &mut Markers) {
        let Some(grid) = self.grid.filter(|g| g.get() > 0) else {
            return;
        };
        let r = rect.rect;
        let off = [r.min.x, r.min.y, r.max.x, r.max.y]
            .iter()
            .any(|v| v.get() % grid.get() != 0);
        if off && self.metal.has_route_in(rect.owner, &r) {
            let marker = Marker::new(
                Violation::OffGrid,
                0,
                LayerRef::Routing(self.layer),
                r,
                rect.owner,
                rect.owner,
            );
            found.add(marker, false);
        }
    }

    fn marker(
        &self,
        violation: Violation,
        rule: u16,
        piece: &GcPolygon,
        bbox: Rect<Dbu>,
    ) -> Marker {
        Marker::new(
            violation,
            rule,
            LayerRef::Routing(self.layer),
            bbox,
            piece.owner,
            piece.owner,
        )
    }
}
