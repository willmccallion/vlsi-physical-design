//! Shorts, non-sufficient metal and metal spacing between the maximal
//! rectangles of a layer, as `FlexGC` checks them.

use super::layout::OwnerIndex;
use super::model::{Facing, GcRect, Metal, Model};
use super::rules::{MetalRules, Rules};
use super::{Marker, Markers, Violation};
use crate::db::indices::LayerId;
use crate::db::library::LayerRef;
use crate::db::rules::SpacingTable;
use crate::geom::dbu::Dbu;
use crate::geom::rect::Rect;
use crate::geom::region::{Heading, Region};
use rayon::prelude::*;

/// Checks every pair of nearby maximal rectangles on every routing layer.
pub(super) fn check(model: &Model<'_>, rules: &Rules) -> Markers {
    model
        .metal
        .iter()
        .enumerate()
        .map(|(l, metal)| {
            let layer = LayerCheck {
                model,
                rules,
                layer: LayerId::new(l),
                metal,
                layer_rules: &rules.metal[l],
                hard_spacing_reach: rules.max_hard_spacing(LayerId::new(l)),
            };
            (0..metal.rects.len())
                .into_par_iter()
                .fold(Markers::default, |mut found, i| {
                    layer.check_rect(i, &mut found);
                    found
                })
                .reduce(Markers::default, Markers::merge)
        })
        .fold(Markers::default(), Markers::merge)
}

/// The checks of one routing layer.
struct LayerCheck<'m> {
    model: &'m Model<'m>,
    rules: &'m Rules,
    layer: LayerId,
    metal: &'m Metal<'m>,
    layer_rules: &'m MetalRules,
    /// The most spacing a nondefault rule can add on this layer.
    hard_spacing_reach: Dbu,
}

impl LayerCheck<'_> {
    /// Checks maximal rectangle `i` against every other within the largest
    /// spacing its width can need.
    fn check_rect(&self, i: usize, found: &mut Markers) {
        let rect = self.metal.rects[i];
        let reach = self
            .layer_rules
            .ranges
            .iter()
            .filter(|r| in_range(r.min_width, r.max_width, rect.width()))
            .map(|r| r.spacing)
            .fold(
                self.layer_rules.max_spacing().max(self.hard_spacing_reach),
                Dbu::max,
            );
        for j in self.metal.rects_near(&rect.rect.expand(reach)) {
            if j != i {
                self.check_pair(&rect, &self.metal.rects[j], found);
            }
        }
    }

    fn check_pair(&self, r1: &GcRect, r2: &GcRect, found: &mut Markers) {
        let facing = Facing::new(&r1.rect, &r2.rect);
        if facing.dist_x > Dbu::ZERO || facing.dist_y > Dbu::ZERO {
            self.check_spacing(r1, r2, &facing, found);
            return;
        }
        let abut = facing.prl_x == Dbu::ZERO || facing.prl_y == Dbu::ZERO;
        let mut marker = facing.marker;
        if facing.prl_x == Dbu::ZERO {
            marker.min.x -= Dbu(1);
            marker.max.x += Dbu(1);
        }
        if facing.prl_y == Dbu::ZERO {
            marker.min.y -= Dbu(1);
            marker.max.y += Dbu(1);
        }
        if self.is_blockage(r1.owner) || self.is_blockage(r2.owner) {
            self.check_short_with_blockage(r1, r2, &marker, abut, found);
        } else {
            self.check_short(r1, r2, &marker, found);
        }
    }

    fn is_blockage(&self, owner: OwnerIndex) -> bool {
        self.model.layout.owners()[owner.index()].is_blockage()
    }

    /// Two rectangles that touch: a short between owners, or too little
    /// overlap within one owner.
    fn check_short(&self, r1: &GcRect, r2: &GcRect, marker: &Rect<Dbu>, found: &mut Markers) {
        let same_owner = r1.owner == r2.owner;
        if same_owner && self.sufficient_metal(r1, r2, marker) {
            return;
        }
        let probe = bloat_flat(marker);
        let routed =
            self.metal.has_route_in(r1.owner, &probe) || self.metal.has_route_in(r2.owner, &probe);
        let violation = if same_owner {
            Violation::NonSufficientMetal
        } else {
            Violation::Short
        };
        let marker = Marker::new(
            violation,
            0,
            LayerRef::Routing(self.layer),
            *marker,
            r1.owner,
            r2.owner,
        );
        found.add(marker, (r1.fixed && r2.fixed) || !routed);
    }

    /// Whether two touching rectangles of one owner overlap by at least the
    /// minimum width, directly or through a third rectangle of the owner.
    fn sufficient_metal(&self, r1: &GcRect, r2: &GcRect, marker: &Rect<Dbu>) -> bool {
        let min_width = self.layer_rules.min_width;
        if diagonal_reaches(marker, min_width) {
            return true;
        }
        let narrow = |r: &GcRect| r.rect.width() < min_width || r.rect.height() < min_width;
        if narrow(r1) || narrow(r2) {
            return true;
        }
        let center = marker.center();
        let probe = Rect::new(center, center).expand(min_width);
        self.metal
            .rects_near(&probe)
            .map(|k| self.metal.rects[k])
            .any(|third| {
                third != *r1
                    && third != *r2
                    && third.owner == r1.owner
                    && third.rect.encloses(marker)
                    && !narrow(&third)
                    && [r1, r2].iter().all(|r| {
                        r.rect
                            .intersection(&third.rect)
                            .is_some_and(|overlap| diagonal_reaches(&overlap, min_width))
                    })
            })
    }

    /// Touching rectangles where one belongs to a blockage: the parts of
    /// the touching area not under the other owner's fixed metal are
    /// checked as shorts with the blockage.
    fn check_short_with_blockage(
        &self,
        r1: &GcRect,
        r2: &GcRect,
        marker: &Rect<Dbu>,
        abut: bool,
        found: &mut Markers,
    ) {
        if self.is_blockage(r1.owner) && self.is_blockage(r2.owner) {
            return;
        }
        if abut && self.required_spacing(r1, r2, Dbu::ZERO).0 == Dbu::ZERO {
            return;
        }
        let (wire, blockage) = if self.is_blockage(r1.owner) {
            (r2, r1)
        } else {
            (r1, r2)
        };
        let covered = self.metal.fixed_within(wire.owner, marker);
        if covered.covers_rect(marker) {
            return;
        }
        let exposed = Region::from_rects([*marker]).difference(&covered);
        for rect in exposed.max_rectangles() {
            let part = GcRect { rect, ..*blockage };
            self.check_short(wire, &part, &rect, found);
        }
    }

    /// Two rectangles apart: whether they keep the spacing their widths and
    /// parallel run need.
    fn check_spacing(&self, r1: &GcRect, r2: &GcRect, facing: &Facing, found: &mut Markers) {
        let prl = facing.prl();
        let (required, range) = self.required_spacing(r1, r2, prl);
        let dx = i128::from(facing.dist_x.get());
        let dy = i128::from(facing.dist_y.get());
        let req = i128::from(required.get());
        if dx * dx + dy * dy >= req * req {
            return;
        }
        let between = if prl <= Dbu::ZERO {
            Between::Either
        } else if facing.dist_x == Dbu::ZERO {
            Between::Vertical
        } else {
            Between::Horizontal
        };
        if !self.has_poly_edges(r1, r2, &facing.marker, between, prl) {
            return;
        }
        let routed = self.has_route(r1, &facing.marker) || self.has_route(r2, &facing.marker);
        let violation = if range {
            Violation::SpacingRange
        } else {
            match self.layer_rules.spacing {
                SpacingTable::ParallelRunLength(_) => Violation::MetalSpacing,
                SpacingTable::TwoWidths(_) => Violation::SpacingTableTw,
            }
        };
        let marker = Marker::new(
            violation,
            0,
            LayerRef::Routing(self.layer),
            facing.marker,
            r1.owner,
            r2.owner,
        );
        found.add(marker, (r1.fixed && r2.fixed) || !routed);
    }

    /// The spacing two rectangles need at parallel run `prl`, and whether a
    /// width range rule sets it.
    fn required_spacing(&self, r1: &GcRect, r2: &GcRect, prl: Dbu) -> (Dbu, bool) {
        let width = |r: &GcRect| {
            if self.is_blockage(r.owner) && self.rules.min_spacing_for_obstructions {
                self.layer_rules.width
            } else {
                r.width()
            }
        };
        let (w1, w2) = (width(r1), width(r2));
        let mut required = self.layer_rules.spacing.spacing(w1, w2, prl);
        let mut range = false;
        if r1.owner != r2.owner {
            for rule in &self.layer_rules.ranges {
                let applies = in_range(rule.min_width, rule.max_width, w1)
                    || in_range(rule.min_width, rule.max_width, w2);
                if applies && rule.spacing > required {
                    range = true;
                    required = rule.spacing;
                }
            }
        }
        let hard = |r: &GcRect| {
            self.rules
                .hard_spacing(self.model.layout.non_default_rule(r.owner), self.layer)
        };
        (required.max(hard(r1)).max(hard(r2)), range)
    }

    /// Whether boundary edges of the two owners bound the space between
    /// them on both sides, so that it is a real gap rather than metal of
    /// one of them.
    fn has_poly_edges(
        &self,
        r1: &GcRect,
        r2: &GcRect,
        marker: &Rect<Dbu>,
        between: Between,
        prl: Dbu,
    ) -> bool {
        let (mut bottom, mut top, mut left, mut right) = (false, false, false, false);
        for e in self.metal.edges_near(marker) {
            if e.owner != r1.owner && e.owner != r2.owner {
                continue;
            }
            let (from, to) = (e.edge.from, e.edge.to);
            if prl <= Dbu::ZERO {
                match e.edge.heading() {
                    Heading::West if from.y == marker.min.y => bottom = true,
                    Heading::East if from.y == marker.max.y => top = true,
                    Heading::North if from.x == marker.min.x => left = true,
                    Heading::South if from.x == marker.max.x => right = true,
                    _ => {}
                }
            } else {
                match e.edge.heading() {
                    Heading::West
                        if from.y >= marker.min.y
                            && from.y < marker.max.y
                            && from.x > marker.min.x
                            && to.x < marker.max.x =>
                    {
                        bottom = true;
                    }
                    Heading::East
                        if from.y > marker.min.y
                            && from.y <= marker.max.y
                            && to.x > marker.min.x
                            && from.x < marker.max.x =>
                    {
                        top = true;
                    }
                    Heading::North
                        if from.x >= marker.min.x
                            && from.x < marker.max.x
                            && to.y > marker.min.y
                            && from.y < marker.max.y =>
                    {
                        left = true;
                    }
                    Heading::South
                        if from.x > marker.min.x
                            && from.x <= marker.max.x
                            && from.y > marker.min.y
                            && to.y < marker.max.y =>
                    {
                        right = true;
                    }
                    _ => {}
                }
            }
        }
        let across_y = bottom && top;
        let across_x = left && right;
        match between {
            Between::Vertical => across_y,
            Between::Horizontal => across_x,
            Between::Either => across_y || across_x,
        }
    }

    /// Whether the part of `rect` within its width of `marker` holds routed
    /// metal, that is, is not all covered by its owner's fixed metal.
    fn has_route(&self, rect: &GcRect, marker: &Rect<Dbu>) -> bool {
        let reach = marker.expand(rect.width());
        let Some(near) = reach.intersection(&rect.rect) else {
            return false;
        };
        if near.width() <= Dbu::ZERO || near.height() <= Dbu::ZERO {
            return false;
        }
        !self
            .metal
            .fixed_within(rect.owner, &near)
            .covers_rect(&near)
    }
}

/// Which pair of opposite sides of the space between two rectangles must
/// be boundary edges.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Between {
    /// The rectangles are one above the other: the bottom and top sides.
    Vertical,
    /// The rectangles are side by side: the left and right sides.
    Horizontal,
    /// The rectangles are apart diagonally: either pair.
    Either,
}

const fn in_range(min: Dbu, max: Dbu, width: Dbu) -> bool {
    min.0 <= width.0 && width.0 <= max.0
}

/// Whether the diagonal of `rect` is at least `length` long.
fn diagonal_reaches(rect: &Rect<Dbu>, length: Dbu) -> bool {
    let (w, h, l) = (
        i128::from(rect.width().get()),
        i128::from(rect.height().get()),
        i128::from(length.get()),
    );
    w * w + h * h >= l * l
}

/// `rect` widened by one unit along each axis it has no extent in.
fn bloat_flat(rect: &Rect<Dbu>) -> Rect<Dbu> {
    let mut r = *rect;
    if r.width() == Dbu::ZERO {
        r.min.x -= Dbu(1);
        r.max.x += Dbu(1);
    }
    if r.height() == Dbu::ZERO {
        r.min.y -= Dbu(1);
        r.max.y += Dbu(1);
    }
    r
}
