//! Checks on each connected piece of an owner's metal alone, as `FlexGC`'s
//! metal shape checks.
//!
//! A piece with any edge of fixed metal is left alone, as `FlexGC` leaves
//! it.

use super::model::Model;
use super::rules::Rules;
use super::{Marker, Markers, Violation};
use crate::db::indices::LayerId;
use crate::db::library::LayerRef;

/// Checks every piece of metal on every routing layer.
pub(super) fn check(model: &Model<'_>, rules: &Rules) -> Markers {
    let mut found = Markers::default();
    for (l, metal) in model.metal.iter().enumerate() {
        let Some(min_area) = rules.metal[l].min_area else {
            continue;
        };
        let layer = LayerRef::Routing(LayerId::new(l));
        for piece in &metal.polygons {
            if piece.area < min_area && !piece.has_fixed_edge {
                let marker = Marker::new(
                    Violation::MinArea,
                    0,
                    layer,
                    piece.bbox,
                    piece.owner,
                    piece.owner,
                );
                found.add(marker, false);
            }
        }
    }
    found
}
