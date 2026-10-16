//! Shorts and spacing between the cuts of a layer, as `FlexGC` checks them.
//!
//! Every spacing rule of the layer is checked against every nearby cut,
//! whatever its net, except that a rule for different nets leaves cuts of
//! one net to the layer's same-net rule when it has one. Cuts that touch
//! short whenever some rule reaches them.

use super::layout::OwnerIndex;
use super::model::{Cut, Facing, GcRect, Metal, Model};
use super::rules::CutRules;
use super::{Marker, Markers, Violation};
use crate::db::indices::CutLayerId;
use crate::db::library::LayerRef;
use crate::db::rules::{CutSpacing, CutSpacingCondition};
use crate::geom::dbu::Dbu;
use crate::geom::rect::Rect;
use rayon::prelude::*;

/// Checks the cuts of every cut layer.
pub(super) fn check(model: &Model<'_>, rules: &[CutRules]) -> Markers {
    model
        .cut
        .iter()
        .enumerate()
        .map(|(c, cut)| {
            let layer = CutLayerId::new(c);
            let check = LayerCheck {
                model,
                layer,
                cut,
                rules: &rules[c],
                below: model.metal.get(layer.below().index()),
                above: model.metal.get(layer.above().index()),
            };
            (0..cut.rects.len())
                .into_par_iter()
                .fold(Markers::default, |mut found, i| {
                    check.check_cut(i, &mut found);
                    found
                })
                .reduce(Markers::default, Markers::merge)
        })
        .fold(Markers::default(), Markers::merge)
}

/// The checks of one cut layer.
struct LayerCheck<'m> {
    model: &'m Model<'m>,
    layer: CutLayerId,
    cut: &'m Cut,
    rules: &'m CutRules,
    below: Option<&'m Metal<'m>>,
    above: Option<&'m Metal<'m>>,
}

impl LayerCheck<'_> {
    fn check_cut(&self, i: usize, found: &mut Markers) {
        for (k, rule) in self.rules.spacings.iter().enumerate() {
            self.check_rule(i, k, rule, found);
        }
    }

    fn is_blockage(&self, owner: OwnerIndex) -> bool {
        self.model.layout.owners()[owner.index()].is_blockage()
    }

    /// Checks cut `i` against the cuts near it under rule `k`.
    fn check_rule(&self, i: usize, k: usize, rule: &CutSpacing, found: &mut Markers) {
        let cut = self.cut.rects[i];
        let reach = match rule.condition {
            CutSpacingCondition::AdjacentCuts { within, .. } => rule.spacing.max(within),
            _ => rule.spacing,
        };
        if !self.has_adjacent_cuts(&cut, rule) {
            return;
        }
        for j in self.cut.near(&cut.rect.expand(reach)) {
            if j == i {
                continue;
            }
            let other = self.cut.rects[j];
            if !rule.same_net && cut.owner == other.owner && self.rules.has_same_net_rule() {
                continue;
            }
            let facing = Facing::new(&cut.rect, &other.rect);
            if facing.dist_x == Dbu::ZERO && facing.dist_y == Dbu::ZERO {
                let marker = Marker::new(
                    Violation::CutShort,
                    0,
                    LayerRef::Cut(self.layer),
                    facing.marker,
                    cut.owner,
                    other.owner,
                );
                found.add(marker, cut.fixed && other.fixed);
            } else {
                self.check_spacing(&cut, &other, &facing, k, rule, found);
            }
        }
    }

    /// Whether `cut` has enough neighbours for an adjacent-cuts rule to
    /// apply; other rules always apply. A blockage wider than a cut counts
    /// as the whole number of neighbours the rule asks for.
    fn has_adjacent_cuts(&self, cut: &GcRect, rule: &CutSpacing) -> bool {
        let CutSpacingCondition::AdjacentCuts { cuts, within } = rule.condition else {
            return true;
        };
        let cut_width = self.rules.width.unwrap_or(Dbu::ZERO);
        if self.is_blockage(cut.owner) && cut.width() > cut_width {
            return true;
        }
        let needed = i64::from(cuts);
        let within_sq = square(within);
        let mut count: i64 = -1;
        for j in self.cut.near(&cut.rect.expand(within)) {
            let other = self.cut.rects[j];
            if distance_sq(rule, &cut.rect, &other.rect) >= within_sq {
                continue;
            }
            if self.is_blockage(other.owner) && other.width() > cut_width {
                count += needed;
            } else {
                count += 1;
            }
        }
        count >= needed
    }

    fn check_spacing(
        &self,
        cut: &GcRect,
        other: &GcRect,
        facing: &Facing,
        k: usize,
        rule: &CutSpacing,
        found: &mut Markers,
    ) {
        if rule.condition == CutSpacingCondition::ParallelOverlap
            && (facing.prl() <= Dbu::ZERO || self.share_metal(cut, other, &facing.marker))
        {
            return;
        }
        if distance_sq(rule, &cut.rect, &other.rect) >= square(rule.spacing) {
            return;
        }
        let marker = Marker::new(
            Violation::CutSpacing,
            u16::try_from(k).unwrap_or(u16::MAX),
            LayerRef::Cut(self.layer),
            facing.marker,
            cut.owner,
            other.owner,
        );
        found.add(marker, cut.fixed && other.fixed);
    }

    /// Whether metal of either cut's owner on the layer below or above
    /// covers all of `between`, the space between them.
    fn share_metal(&self, cut: &GcRect, other: &GcRect, between: &Rect<Dbu>) -> bool {
        [self.below, self.above].into_iter().flatten().any(|metal| {
            metal.rects_near(between).map(|r| metal.rects[r]).any(|r| {
                (r.owner == cut.owner || r.owner == other.owner) && r.rect.encloses(between)
            })
        })
    }
}

fn square(d: Dbu) -> i128 {
    i128::from(d.get()) * i128::from(d.get())
}

/// The squared distance between two cuts as `rule` measures it: between
/// centers or between edges.
fn distance_sq(rule: &CutSpacing, a: &Rect<Dbu>, b: &Rect<Dbu>) -> i128 {
    if rule.center_to_center {
        let (ca, cb) = (a.center(), b.center());
        square(ca.x - cb.x) + square(ca.y - cb.y)
    } else {
        let f = Facing::new(a, b);
        square(f.dist_x) + square(f.dist_y)
    }
}
