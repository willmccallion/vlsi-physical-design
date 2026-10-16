//! Minimum cuts on wide metal, as `FlexGC` checks them.
//!
//! `FlexGC` does not count cuts: every cut of the same owner that meets a
//! maximal rectangle as wide as a rule asks, on the cut layers the rule
//! names, is a violation. With a length, only cuts outside the wide metal
//! count, when metal of the owner joins them to it.

use super::model::{Cut, Facing, GcRect, Metal, Model};
use super::rules::Rules;
use super::{Marker, Markers, Violation};
use crate::db::indices::LayerId;
use crate::db::library::LayerRef;
use crate::db::rules::{CutApproach, MinimumCut};
use rayon::prelude::*;

/// Checks every maximal rectangle of every routing layer with rules.
pub(super) fn check(model: &Model<'_>, rules: &Rules) -> Markers {
    model
        .metal
        .iter()
        .enumerate()
        .filter(|&(l, _)| !rules.metal[l].minimum_cuts.is_empty())
        .map(|(l, metal)| {
            let layer = LayerId::new(l);
            let check = LayerCheck {
                layer,
                metal,
                rules: &rules.metal[l].minimum_cuts,
                below: l.checked_sub(1).and_then(|c| model.cut.get(c)),
                above: model.cut.get(layer.cut_above().index()),
            };
            (0..metal.rects.len())
                .into_par_iter()
                .fold(Markers::default, |mut found, i| {
                    check.check_rect(&metal.rects[i], &mut found);
                    found
                })
                .reduce(Markers::default, Markers::merge)
        })
        .fold(Markers::default(), Markers::merge)
}

/// The minimum-cut checks of one routing layer.
struct LayerCheck<'m> {
    layer: LayerId,
    metal: &'m Metal<'m>,
    rules: &'m [MinimumCut],
    below: Option<&'m Cut>,
    above: Option<&'m Cut>,
}

impl LayerCheck<'_> {
    fn check_rect(&self, wide: &GcRect, found: &mut Markers) {
        let width = wide.width();
        let length = wide.rect.width().max(wide.rect.height());
        for (k, rule) in self.rules.iter().enumerate() {
            if width < rule.width || rule.length.is_some_and(|(l, _)| length < l) {
                continue;
            }
            let reach = rule.length.map_or(wide.rect, |(_, d)| wide.rect.expand(d));
            let below = self.below.filter(|_| rule.from != Some(CutApproach::Above));
            let above = self.above.filter(|_| rule.from != Some(CutApproach::Below));
            for cuts in [below, above].into_iter().flatten() {
                for j in cuts.near(&reach) {
                    self.check_cut(wide, &cuts.rects[j], k, rule, found);
                }
            }
        }
    }

    fn check_cut(
        &self,
        wide: &GcRect,
        cut: &GcRect,
        k: usize,
        rule: &MinimumCut,
        found: &mut Markers,
    ) {
        if cut.owner != wide.owner {
            return;
        }
        if rule.length.is_some() && (wide.rect.encloses(&cut.rect) || !self.joined(wide, cut)) {
            return;
        }
        let marker = Marker::new(
            Violation::MinimumCut,
            u16::try_from(k).unwrap_or(u16::MAX),
            LayerRef::Routing(self.layer),
            Facing::new(&wide.rect, &cut.rect).marker,
            wide.owner,
            cut.owner,
        );
        found.add(marker, wide.fixed && cut.fixed);
    }

    /// Whether a maximal rectangle of the owner on this layer meets both the
    /// cut and the wide metal.
    fn joined(&self, wide: &GcRect, cut: &GcRect) -> bool {
        self.metal
            .rects_near(&cut.rect)
            .map(|i| self.metal.rects[i])
            .any(|r| {
                r.owner == cut.owner && r.rect.touches(&cut.rect) && r.rect.touches(&wide.rect)
            })
    }
}
