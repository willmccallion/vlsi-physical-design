//! Patch wires that bring thin pieces of a net's metal up to the minimum
//! area, as `TritonRoute` adds them after routing a net.

use super::DetailedRouter;
use super::judge::Candidate;
use crate::search::Search;
use crate::wiring::Patch;
use pare_common::db::library::{Direction, LayerRef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::drc::{Layout, Owner, ThinPiece, thin_pieces};

impl DetailedRouter<'_> {
    /// Patches that bring every thin piece of the candidate's metal meeting
    /// `area` up to its layer's minimum area. Each patch extends the piece
    /// along the layer's preferred direction, as `TritonRoute` places its
    /// patch wires, from one end or split between both, however `search`
    /// prices lowest.
    pub(super) fn min_area_patches(
        &self,
        candidate: &Candidate<'_>,
        area: &Rect<Dbu>,
        search: &Search<'_>,
    ) -> Vec<Patch> {
        if self.rules.metal.iter().all(|m| m.min_area.is_none()) {
            return Vec::new();
        }
        let Some(owner) = self.fixed.owner_index(Owner::Net(candidate.net)) else {
            return Vec::new();
        };
        let window = self.judge_window(area);
        let elems = self.as_committed(candidate, &window).elems(self.tech);
        let mut layout = Layout::with_owners_of(&self.fixed);
        for (layer, i) in self.fixed_tiles.near(&window) {
            let shape = match layer {
                LayerRef::Routing(l) => &self.fixed.metal(l.index())[i as usize],
                LayerRef::Cut(c) => &self.fixed.cut(c.index())[i as usize],
            };
            if shape.owner == owner && shape.rect.touches(&window) {
                layout.add_shape(layer, shape);
            }
        }
        layout.add_wiring(self.db, candidate.net, &elems, Some(&window));
        thin_pieces(&layout, &self.rules, Some(window))
            .iter()
            .filter(|piece| piece.owner == owner && piece.bbox.touches(area))
            .filter_map(|piece| self.patch_for(piece, search))
            .collect()
    }

    /// The patch that brings `piece` up to its minimum area, as the search
    /// prices lowest: laid along the layer's tracks at one end of the piece
    /// or split between its two ends, or, where the track has no room for
    /// it, grown across the tracks the same ways over the piece's length; the
    /// box around the piece and the patch when split. `None` on a layer the
    /// router cannot use.
    fn patch_for(&self, piece: &ThinPiece, search: &Search<'_>) -> Option<Patch> {
        if !self.tech.routable(piece.layer) {
            return None;
        }
        let lt = self.tech.layer(piece.layer);
        let width = search.width(piece.layer);
        let grid = self.rules.manufacturing_grid.map_or(1, |g| g.get().max(1));
        let deficit = piece.min_area.get() - piece.area.get();
        let length = round_up(round_up(deficit, width.get()) / width.get(), grid);
        let ((a0, a1), (b0, b1)) = lt.across_along_rect(&piece.bbox);
        let across = if a1 - a0 == width {
            (a0, a1)
        } else {
            let mid = a0.midpoint(a1);
            let track = lt
                .tracks_within(a0, a1)
                .map(|t| lt.tracks[t as usize])
                .min_by_key(|&t| (t - mid).abs())
                .unwrap_or(mid);
            (track - width / 2, track + width / 2)
        };
        let rect = |along: (Dbu, Dbu)| match lt.direction {
            Direction::Horizontal => {
                Rect::new(Point::new(along.0, across.0), Point::new(along.1, across.1))
            }
            Direction::Vertical => {
                Rect::new(Point::new(across.0, along.0), Point::new(across.1, along.1))
            }
        };
        let cost = |r: &Rect<Dbu>| search.shape_cost(piece.layer, r);
        let split = |length: i64, low: i64, of: i64| {
            let below = round_up(length * low / of, grid);
            (below, length - below)
        };
        let along_tracks = PATCH_SPLITS.iter().map(|&(low, of)| {
            let (below, above) = split(length, low, of);
            rect((b0 - Dbu(below), b1 + Dbu(above)))
        });
        let run = (b1 - b0).get().max(1);
        let breadth = round_up(round_up(deficit, run) / run, grid);
        let across_tracks = PATCH_SPLITS.iter().map(|&(low, of)| {
            let (below, above) = split(breadth, low, of);
            let (c0, c1) = (a0 - Dbu(below), a1 + Dbu(above));
            match lt.direction {
                Direction::Horizontal => Rect::new(Point::new(b0, c0), Point::new(b1, c1)),
                Direction::Vertical => Rect::new(Point::new(c0, b0), Point::new(c1, b1)),
            }
        });
        let chosen = along_tracks
            .chain(across_tracks)
            .min_by(|a, b| cost(a).total_cmp(&cost(b)))?;
        Some(Patch {
            layer: piece.layer,
            rect: chosen,
        })
    }
}

/// How a patch's length may be split between the low and the high end of
/// the piece it extends, as the share below over a whole: all at the low
/// end first, then all at the high end, then splits between them.
const PATCH_SPLITS: [(i64, i64); 5] = [(1, 1), (0, 1), (1, 2), (1, 4), (3, 4)];

/// `value` rounded up to a multiple of `step`, which is positive.
const fn round_up(value: i64, step: i64) -> i64 {
    let rem = value.rem_euclid(step);
    if rem == 0 { value } else { value + step - rem }
}
