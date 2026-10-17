//! Track assignment: a track for each long straight run of a global route.
//!
//! A row (or column) of gcells on one layer is a panel. Every maximal run of
//! a net's global route along a panel, an iroute, becomes a wire on one of
//! the panel's tracks spanning from its first gcell to its last: at each
//! end, far enough to pass the net's pins in that gcell and, where the
//! route changes layer or has no pin there, its centre. Within a panel, iroutes
//! take tracks longest first, each where its metal overlaps least with
//! other nets' (fixed metal counting most) and, among equals, nearest the
//! net's pins in its end gcells; iroutes left overlapping are then placed
//! again for a few rounds. Panels are assigned in parallel, odd ones after
//! even ones so that wires beside a panel's edge see their neighbours.
//! Stretches of a wire that would still touch fixed metal are left out for
//! detailed routing to go around.

use crate::access::Node;
use crate::global::{GCell, GlobalGrid, GlobalRoutes};
use crate::occupancy::{LayerSpacing, Occupancy, Owner, Query, affected_tracks};
use crate::tech::{LayerTech, RouteTech};
use crate::wiring::Edge;
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::shapes::wire_rect;
use rayon::prelude::*;
use rustc_hash::FxHashMap;
use std::collections::BTreeMap;
use std::ops::Range;

/// Cost per DBU of overlap with fixed metal of another owner.
const FIXED_WEIGHT: i64 = 64;

/// Cost per DBU of overlap with another net's wire.
const OVERLAP_WEIGHT: i64 = 8;

/// Rounds of placing overlapping iroutes again.
const REASSIGN_ROUNDS: usize = 3;

/// A net's wire along one track, from one crossing to a later one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TrackWire {
    /// The net.
    pub net: NetId,
    /// Routing layer.
    pub layer: LayerId,
    /// Track index on it.
    pub track: u32,
    /// First crossing.
    pub from: u32,
    /// Last crossing, after `from`.
    pub to: u32,
}

impl TrackWire {
    /// Every crossing it covers.
    pub fn nodes(self) -> impl Iterator<Item = Node> {
        (self.from..=self.to).map(move |cross| self.node(cross))
    }

    /// The track graph edges it uses.
    pub fn edges(self) -> impl Iterator<Item = Edge> {
        (self.from..self.to).map(move |cross| Edge::Wire(self.node(cross)))
    }

    const fn node(self, cross: u32) -> Node {
        Node {
            layer: self.layer,
            track: self.track,
            cross,
        }
    }
}

/// A row or column of gcells on one layer, along the layer's direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct PanelId {
    layer: LayerId,
    /// Row of a horizontal layer, column of a vertical one.
    index: u16,
}

/// A run of a net's global route along a panel.
#[derive(Clone, Debug)]
struct Iroute {
    net: NetId,
    /// Crossings nearest the centres of its first and last gcells.
    from: u32,
    to: u32,
    /// Across coordinates of the net's pins in its first and last gcells.
    pins: Vec<Dbu>,
}

#[derive(Debug)]
struct Panel {
    id: PanelId,
    tracks: Range<u32>,
    iroutes: Vec<Iroute>,
}

/// An iroute's metal along one track.
#[derive(Clone, Copy, Debug)]
struct Span {
    lo: Dbu,
    hi: Dbu,
    net: NetId,
    iroute: usize,
}

/// The tracks a panel's iroutes are on.
#[derive(Debug)]
struct Placed {
    tracks: Range<u32>,
    spans: Vec<Vec<Span>>,
    choice: Vec<Option<u32>>,
}

impl Placed {
    fn new(tracks: Range<u32>, iroutes: usize) -> Self {
        Self {
            spans: vec![Vec::new(); tracks.len()],
            tracks,
            choice: vec![None; iroutes],
        }
    }

    fn spans_on(&self, track: u32) -> Option<&[Span]> {
        self.tracks
            .contains(&track)
            .then(|| self.spans[(track - self.tracks.start) as usize].as_slice())
    }

    fn put(&mut self, track: u32, span: Span) {
        self.spans[(track - self.tracks.start) as usize].push(span);
        self.choice[span.iroute] = Some(track);
    }

    fn take(&mut self, iroute: usize) {
        if let Some(track) = self.choice[iroute].take() {
            self.spans[(track - self.tracks.start) as usize].retain(|s| s.iroute != iroute);
        }
    }
}

/// Places every iroute of `routes` on a track.
///
/// Metal already recorded in `occupancy` is fixed: wires avoid it and are
/// cut where they would touch it. `pins` holds, per net, where each of its
/// pins is reached.
pub fn assign_tracks(
    tech: &RouteTech,
    occupancy: &Occupancy,
    grid: &GlobalGrid,
    routes: &GlobalRoutes,
    pins: &[Vec<Point<Dbu>>],
) -> Vec<TrackWire> {
    let panels = panels(tech, grid, routes, pins);
    let mut placed: FxHashMap<PanelId, Placed> = FxHashMap::default();
    for parity in 0..2 {
        let results: Vec<(PanelId, Placed)> = panels
            .par_iter()
            .filter(|p| p.id.index % 2 == parity)
            .map(|p| {
                let neighbours: Vec<&Placed> =
                    [p.id.index.checked_sub(1), p.id.index.checked_add(1)]
                        .into_iter()
                        .flatten()
                        .filter_map(|index| {
                            placed.get(&PanelId {
                                layer: p.id.layer,
                                index,
                            })
                        })
                        .collect();
                let costing = Costing::new(tech, occupancy, p.id.layer);
                (p.id, costing.assign_panel(p, &neighbours))
            })
            .collect();
        placed.extend(results);
    }
    panels
        .par_iter()
        .flat_map_iter(|p| {
            let costing = Costing::new(tech, occupancy, p.id.layer);
            let choice = placed.get(&p.id).map(|pl| pl.choice.clone());
            p.iroutes
                .iter()
                .zip(choice.unwrap_or_default())
                .filter_map(|(ir, track)| Some(costing.clear_pieces(ir, track?)))
                .flatten()
                .collect::<Vec<_>>()
        })
        .collect()
}

/// Every panel that some iroute runs along, with its iroutes.
fn panels(
    tech: &RouteTech,
    grid: &GlobalGrid,
    routes: &GlobalRoutes,
    pins: &[Vec<Point<Dbu>>],
) -> Vec<Panel> {
    let mut nets: Vec<(NetId, &Vec<Vec<GCell>>)> =
        routes.paths.iter().map(|(&n, p)| (n, p)).collect();
    nets.sort_unstable_by_key(|&(n, _)| n);
    let mut by_panel: BTreeMap<PanelId, Vec<Iroute>> = BTreeMap::new();
    let direction = |l: LayerId| tech.layer(l).direction;
    for (net, paths) in nets {
        let net_runs = runs(direction, paths);
        let net_pins = pins.get(net.index()).map_or(&[][..], Vec::as_slice);
        for &(id, first, last) in &net_runs {
            let lt = tech.layer(id.layer);
            let ends = [first, last].map(|along| cell_along(lt.direction, id.index, along));
            let pins_in = |cell: (u16, u16)| {
                net_pins.iter().filter(move |&&p| {
                    let c = grid.cell_at(id.layer, p);
                    (c.x, c.y) == cell
                })
            };
            let reach = |cell: (u16, u16), low: bool| {
                let mut targets: Vec<Dbu> = pins_in(cell).map(|&p| lt.across_along(p).1).collect();
                let joins_run = net_runs
                    .iter()
                    .any(|&run| run.0 != id && run_covers(direction(run.0.layer), run, cell));
                if targets.is_empty() || joins_run {
                    targets.push(lt.across_along(grid.cell_rect(cell.0, cell.1).center()).1);
                }
                let along = if low {
                    targets.iter().min()
                } else {
                    targets.iter().max()
                };
                nearest_cross(lt, *along?)
            };
            let (Some(from), Some(to)) = (reach(ends[0], true), reach(ends[1], false)) else {
                continue;
            };
            if from >= to {
                continue;
            }
            let pin_across = ends
                .iter()
                .flat_map(|&cell| pins_in(cell))
                .map(|&p| lt.across_along(p).0)
                .collect();
            by_panel.entry(id).or_default().push(Iroute {
                net,
                from,
                to,
                pins: pin_across,
            });
        }
    }
    by_panel
        .into_iter()
        .map(|(id, iroutes)| {
            let lt = tech.layer(id.layer);
            let (x, y) = cell_along(lt.direction, id.index, 0);
            let ((a0, a1), _) = lt.across_along_rect(&grid.cell_rect(x, y));
            Panel {
                id,
                tracks: lt.tracks_within(a0, a1 - Dbu(1)),
                iroutes,
            }
        })
        .collect()
}

/// The (column, row) of the gcell `along` steps into a panel.
const fn cell_along(direction: Direction, panel: u16, along: u16) -> (u16, u16) {
    match direction {
        Direction::Horizontal => (along, panel),
        Direction::Vertical => (panel, along),
    }
}

/// Whether a run from [`runs`] on a layer of `direction` passes through the
/// gcell at (column, row) `cell`.
fn run_covers(
    direction: Direction,
    (id, first, last): (PanelId, u16, u16),
    cell: (u16, u16),
) -> bool {
    let (index, along) = match direction {
        Direction::Horizontal => (cell.1, cell.0),
        Direction::Vertical => (cell.0, cell.1),
    };
    index == id.index && (first..=last).contains(&along)
}

/// Maximal runs of a net's global route along one panel, as the panel and
/// the positions of the first and last gcell along it.
fn runs(
    direction: impl Fn(LayerId) -> Direction,
    paths: &[Vec<GCell>],
) -> Vec<(PanelId, u16, u16)> {
    let mut steps: BTreeMap<PanelId, Vec<u16>> = BTreeMap::new();
    for path in paths {
        for w in path.windows(2) {
            let (a, b) = (w[0], w[1]);
            let moved = a.x != b.x || a.y != b.y;
            if a.layer != b.layer || !moved {
                continue;
            }
            let (index, along) = match direction(a.layer) {
                Direction::Horizontal => (a.y, a.x.min(b.x)),
                Direction::Vertical => (a.x, a.y.min(b.y)),
            };
            steps
                .entry(PanelId {
                    layer: a.layer,
                    index,
                })
                .or_default()
                .push(along);
        }
    }
    let mut out = Vec::new();
    for (id, mut along) in steps {
        along.sort_unstable();
        along.dedup();
        let mut run: Option<(u16, u16)> = None;
        for step in along {
            run = match run {
                Some((start, end)) if step == end + 1 => Some((start, step)),
                Some((start, end)) => {
                    out.push((id, start, end + 1));
                    Some((step, step))
                }
                None => Some((step, step)),
            };
        }
        if let Some((start, end)) = run {
            out.push((id, start, end + 1));
        }
    }
    out
}

/// Index of the crossing of `lt` nearest `along`.
fn nearest_cross(lt: &LayerTech, along: Dbu) -> Option<u32> {
    let k = lt.cross.partition_point(|c| c.at < along);
    [k.checked_sub(1), (k < lt.cross.len()).then_some(k)]
        .into_iter()
        .flatten()
        .min_by_key(|&i| ((lt.cross[i].at - along).abs(), i))
        .map(crate::tech::to_u32)
}

/// Length by which `[lo, hi]` and `[olo, ohi]` overlap once one is widened
/// by `spacing`; zero when they keep that far apart.
fn overlap(lo: Dbu, hi: Dbu, olo: Dbu, ohi: Dbu, spacing: Dbu) -> i64 {
    (hi.min(ohi) - lo.max(olo) + spacing).get().max(0)
}

/// Runs of consecutive crossings in `from..=to` joined by edges that are
/// not `blocked`, each as its first and last crossing.
fn clear_runs(from: u32, to: u32, blocked: impl Fn(u32) -> bool) -> Vec<(u32, u32)> {
    let mut out = Vec::new();
    let mut start = None;
    for cross in from..to {
        match (blocked(cross), start) {
            (false, None) => start = Some(cross),
            (true, Some(s)) => {
                out.push((s, cross));
                start = None;
            }
            _ => {}
        }
    }
    if let Some(s) = start {
        out.push((s, to));
    }
    out
}

/// The cost of an iroute on a track, split into overlap and pin distance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Cost {
    overlap: i64,
    pins: i64,
}

impl Cost {
    const fn total(self) -> i64 {
        self.overlap + self.pins
    }
}

/// Costs of placing one layer's iroutes.
struct Costing<'a> {
    tech: &'a RouteTech,
    occupancy: &'a Occupancy,
    layer: LayerId,
    lt: &'a LayerTech,
}

impl<'a> Costing<'a> {
    fn new(tech: &'a RouteTech, occupancy: &'a Occupancy, layer: LayerId) -> Self {
        Self {
            tech,
            occupancy,
            layer,
            lt: tech.layer(layer),
        }
    }

    /// The wire of `net` from crossing `from` to `to` on `track`.
    fn metal(&self, net: NetId, track: u32, from: u32, to: u32) -> Option<Rect<Dbu>> {
        let width = self
            .tech
            .wire_width(self.tech.net_rule(net.index()), self.layer);
        wire_rect(self.lt.point(track, from), self.lt.point(track, to), width)
    }

    /// The spacing `net`'s wires keep on this layer.
    fn spacing(&self, net: NetId) -> Dbu {
        self.tech
            .wire_spacing(self.tech.net_rule(net.index()), self.layer)
    }

    /// Overlap of `rect` of `net` with fixed metal of other owners.
    fn fixed_overlap(&self, net: NetId, rect: &Rect<Dbu>) -> i64 {
        let query = Query::of(self.lt, rect, self.spacing(net));
        let rules = LayerSpacing::of(self.lt);
        self.occupancy
            .rect_conflicts(self.tech, self.layer, rect, Owner::Net(net.0))
            .map(|(_, o)| overlap(query.lo, query.hi, o.lo, o.hi, rules.required(o, query)))
            .sum()
    }

    fn cost(
        &self,
        ir: &Iroute,
        track: u32,
        placed: &Placed,
        neighbours: &[&Placed],
    ) -> Option<Cost> {
        let rect = self.metal(ir.net, track, ir.from, ir.to)?;
        let (_, (lo, hi)) = self.lt.across_along_rect(&rect);
        let spacing = self.spacing(ir.net);
        let mut wires = 0;
        for (t, _) in affected_tracks(self.lt, &rect, spacing) {
            let spans = placed
                .spans_on(t)
                .or_else(|| neighbours.iter().find_map(|n| n.spans_on(t)));
            for s in spans.into_iter().flatten() {
                if s.net != ir.net {
                    wires += overlap(lo, hi, s.lo, s.hi, spacing.max(self.spacing(s.net)));
                }
            }
        }
        let across = self.lt.tracks[track as usize];
        Some(Cost {
            overlap: FIXED_WEIGHT * self.fixed_overlap(ir.net, &rect) + OVERLAP_WEIGHT * wires,
            pins: ir.pins.iter().map(|&p| (across - p).abs().get()).sum(),
        })
    }

    /// The cheapest track for iroute `i` given the others placed so far.
    fn best_track(
        &self,
        panel: &Panel,
        i: usize,
        placed: &Placed,
        neighbours: &[&Placed],
    ) -> Option<u32> {
        let ir = &panel.iroutes[i];
        panel
            .tracks
            .clone()
            .filter_map(|t| Some((self.cost(ir, t, placed, neighbours)?.total(), t)))
            .min()
            .map(|(_, t)| t)
    }

    fn place(&self, panel: &Panel, i: usize, track: u32, placed: &mut Placed) {
        let ir = &panel.iroutes[i];
        if let Some(rect) = self.metal(ir.net, track, ir.from, ir.to) {
            let (_, (lo, hi)) = self.lt.across_along_rect(&rect);
            placed.put(
                track,
                Span {
                    lo,
                    hi,
                    net: ir.net,
                    iroute: i,
                },
            );
        }
    }

    /// Places a panel's iroutes, longest first, then places again those
    /// still overlapping something.
    fn assign_panel(&self, panel: &Panel, neighbours: &[&Placed]) -> Placed {
        let mut placed = Placed::new(panel.tracks.clone(), panel.iroutes.len());
        let mut order: Vec<usize> = (0..panel.iroutes.len()).collect();
        order.sort_by_key(|&i| {
            let ir = &panel.iroutes[i];
            (std::cmp::Reverse(ir.to - ir.from), ir.net, ir.from)
        });
        for &i in &order {
            if let Some(t) = self.best_track(panel, i, &placed, neighbours) {
                self.place(panel, i, t, &mut placed);
            }
        }
        for _ in 0..REASSIGN_ROUNDS {
            let mut moved = false;
            for &i in &order {
                let Some(current) = placed.choice[i] else {
                    continue;
                };
                let ir = &panel.iroutes[i];
                placed.take(i);
                let overlapping = self
                    .cost(ir, current, &placed, neighbours)
                    .is_some_and(|c| c.overlap > 0);
                let best = if overlapping {
                    self.best_track(panel, i, &placed, neighbours)
                        .unwrap_or(current)
                } else {
                    current
                };
                self.place(panel, i, best, &mut placed);
                moved |= best != current;
            }
            if !moved {
                break;
            }
        }
        placed
    }

    /// The pieces of an iroute's wire on `track` that keep clear of fixed
    /// metal.
    fn clear_pieces(&self, ir: &Iroute, track: u32) -> Vec<TrackWire> {
        let blocked = |cross: u32| {
            self.metal(ir.net, track, cross, cross + 1)
                .is_none_or(|r| self.fixed_overlap(ir.net, &r) > 0)
        };
        clear_runs(ir.from, ir.to, blocked)
            .into_iter()
            .map(|(from, to)| TrackWire {
                net: ir.net,
                layer: self.layer,
                track,
                from,
                to,
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(layer: u8, x: u16, y: u16) -> GCell {
        GCell {
            layer: LayerId(layer),
            x,
            y,
        }
    }

    fn alternating(l: LayerId) -> Direction {
        if l.index().is_multiple_of(2) {
            Direction::Vertical
        } else {
            Direction::Horizontal
        }
    }

    #[test]
    fn runs_merge_paths_along_a_panel_and_skip_layer_changes() {
        let paths = vec![
            vec![cell(1, 0, 2), cell(1, 1, 2), cell(1, 2, 2), cell(2, 2, 2)],
            vec![cell(1, 2, 2), cell(1, 3, 2)],
            vec![cell(1, 5, 2), cell(1, 6, 2)],
            vec![cell(2, 2, 2), cell(2, 2, 3)],
        ];

        let found = runs(alternating, &paths);

        let row = PanelId {
            layer: LayerId(1),
            index: 2,
        };
        let column = PanelId {
            layer: LayerId(2),
            index: 2,
        };
        assert_eq!(found, [(row, 0, 3), (row, 5, 6), (column, 2, 3)]);
    }

    #[test]
    fn a_wire_is_cut_around_its_blocked_edges() {
        let pieces = clear_runs(0, 8, |c| c == 3 || c == 4 || c == 7);

        assert_eq!(pieces, [(0, 3), (5, 7)]);
    }

    #[test]
    fn overlap_counts_the_spacing_and_is_zero_when_apart() {
        assert_eq!(overlap(Dbu(0), Dbu(100), Dbu(50), Dbu(300), Dbu(10)), 60);
        assert_eq!(overlap(Dbu(0), Dbu(100), Dbu(110), Dbu(300), Dbu(10)), 0);
    }
}
