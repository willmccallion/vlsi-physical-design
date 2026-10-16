//! Checks on each connected piece of an owner's metal alone, as `FlexGC`'s
//! metal shape checks.

use super::model::{GcPolygon, Metal, Model};
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
            };
            metal
                .polygons
                .par_iter()
                .fold(Markers::default, |mut found, piece| {
                    check.check_piece(piece, &mut found);
                    found
                })
                .reduce(Markers::default, Markers::merge)
        })
        .fold(Markers::default(), Markers::merge)
}

/// The shape checks of one routing layer.
struct PieceCheck<'m> {
    metal: &'m Metal<'m>,
    layer: LayerId,
    rules: &'m MetalRules,
}

impl PieceCheck<'_> {
    fn check_piece(&self, piece: &GcPolygon, found: &mut Markers) {
        self.min_width(piece, found);
        self.min_area(piece, found);
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
                found.add(self.marker(Violation::MinWidth, piece, slice), false);
            }
        }
    }

    /// A piece with any edge of fixed metal is exempt.
    fn min_area(&self, piece: &GcPolygon, found: &mut Markers) {
        let Some(min_area) = self.rules.min_area else {
            return;
        };
        if piece.area < min_area && !piece.has_fixed_edge {
            found.add(self.marker(Violation::MinArea, piece, piece.bbox), false);
        }
    }

    fn marker(&self, violation: Violation, piece: &GcPolygon, bbox: Rect<Dbu>) -> Marker {
        Marker::new(
            violation,
            0,
            LayerRef::Routing(self.layer),
            bbox,
            piece.owner,
            piece.owner,
        )
    }
}
