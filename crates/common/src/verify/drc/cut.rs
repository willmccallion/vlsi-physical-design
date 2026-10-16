//! Shorts and spacing between the cuts of a layer, as `FlexGC` checks them.
//!
//! Every `SPACING` statement of the layer is checked against every nearby
//! cut, whatever its net, except that a rule for different nets leaves cuts
//! of one net to the layer's same-net rule when it has one. Cuts that touch
//! short whenever some statement reaches them. The layer's `LEF57_SPACING`
//! and `LEF58_SPACING` property rules are checked on their own afterwards,
//! with no short case and no deference to a same-net statement.

use super::layout::OwnerIndex;
use super::model::{Cut, Facing, GcRect, Metal, Model};
use super::rules::CutRules;
use super::{Marker, Markers, Violation};
use crate::db::indices::CutLayerId;
use crate::db::library::LayerRef;
use crate::db::rules::{
    CutSpacing, CutSpacingCondition, Lef58CutSpacing, Lef58CutSpacingCondition,
};
use crate::geom::dbu::Dbu;
use crate::geom::rect::Rect;

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
            (0..cut.rects.len()).fold(Markers::default(), |mut found, i| {
                check.check_cut(i, &mut found);
                found
            })
        })
        .fold(Markers::default(), Markers::merge)
}

/// The checks of one cut layer.
struct LayerCheck<'m> {
    model: &'m Model<'m>,
    layer: CutLayerId,
    cut: &'m Cut<'m>,
    rules: &'m CutRules,
    below: Option<&'m Metal<'m>>,
    above: Option<&'m Metal<'m>>,
}

impl LayerCheck<'_> {
    fn check_cut(&self, i: usize, found: &mut Markers) {
        for (k, rule) in self.rules.spacings.iter().enumerate() {
            self.check_rule(i, k, rule, found);
        }
        for (k, rule) in self.rules.lef58_spacings.iter().enumerate() {
            self.check_lef58_rule(i, self.rules.spacings.len() + k, rule, found);
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
            if distance_sq(rule.center_to_center, &cut.rect, &other.rect) >= within_sq {
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
        if distance_sq(rule.center_to_center, &cut.rect, &other.rect) >= square(rule.spacing) {
            return;
        }
        found.add(
            self.spacing_marker(k, facing, cut, other),
            cut.fixed && other.fixed,
        );
    }

    /// Checks cut `i` against the cuts near it under the property rule `k`.
    ///
    /// `FlexGC` measures a property rule's parallel run on the box between
    /// the cuts without a sign, so every pair but two cuts meeting at a
    /// corner has a run, and touching cuts get a spacing marker besides
    /// their short.
    fn check_lef58_rule(&self, i: usize, k: usize, rule: &Lef58CutSpacing, found: &mut Markers) {
        let cut = self.cut.rects[i];
        for j in self.cut.near(&cut.rect.expand(rule.spacing)) {
            if j == i {
                continue;
            }
            let other = self.cut.rects[j];
            if rule.same_net && cut.owner != other.owner {
                continue;
            }
            let facing = Facing::new(&cut.rect, &other.rect);
            let exempt = match rule.condition {
                Lef58CutSpacingCondition::ParallelOverlap => {
                    let between = &facing.marker;
                    (between.width() <= Dbu::ZERO && between.height() <= Dbu::ZERO)
                        || self.share_metal(&cut, &other, between)
                }
            };
            if exempt
                || distance_sq(rule.center_to_center, &cut.rect, &other.rect)
                    >= square(rule.spacing)
            {
                continue;
            }
            found.add(
                self.spacing_marker(k, &facing, &cut, &other),
                cut.fixed && other.fixed,
            );
        }
    }

    fn spacing_marker(&self, rule: usize, facing: &Facing, cut: &GcRect, other: &GcRect) -> Marker {
        Marker::new(
            Violation::CutSpacing,
            u16::try_from(rule).unwrap_or(u16::MAX),
            LayerRef::Cut(self.layer),
            facing.marker,
            cut.owner,
            other.owner,
        )
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

/// The squared distance between two cuts as a rule measures it: between
/// centers or between edges.
fn distance_sq(center_to_center: bool, a: &Rect<Dbu>, b: &Rect<Dbu>) -> i128 {
    if center_to_center {
        let (ca, cb) = (a.center(), b.center());
        square(ca.x - cb.x) + square(ca.y - cb.y)
    } else {
        let f = Facing::new(a, b);
        square(f.dist_x) + square(f.dist_y)
    }
}
