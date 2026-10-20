//! The first detailed routing pass, region by region.
//!
//! As in `TritonRoute`'s first iteration, no net is routed whole. Wherever a
//! net's global route steps from one region into the next, a bridge joins
//! them: one step of wire across the boundary, on the track that track
//! assignment gave that run, or else on the free track nearest the middle
//! of the gcells. Inside each region, every part of the global route that
//! hangs together there becomes a piece joining its pins and bridge ends,
//! routed with the region's other pieces. The two regions a bridge joins
//! share it, so the pieces of a net form one tree. Nets the regions cannot
//! route are routed whole afterwards.

use super::region::{
    Changes, Piece, RegionGrid, RegionKey, RepairStats, Routed, colour, contains_half_open,
};
use super::{DetailedRouter, NetRoute};
use crate::access::{AccessPoint, Node};
use crate::global::GCell;
use crate::runs::EdgeSet;
use crate::search::{Costs, Marks};
use crate::wiring::Edge;
use pare_common::db::indices::NetId;
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BTreeMap;

/// How the first pass routes one net.
struct Plan {
    net: NetId,
    /// The access point each pin is reached through.
    access: Vec<AccessPoint>,
    /// One wire edge across every region boundary its global route crosses.
    bridges: Vec<Edge>,
    /// The pieces to route, by region.
    pieces: Vec<(RegionKey, Piece)>,
}

/// One part of a net's global route inside one region.
#[derive(Default)]
struct Part {
    region: Option<RegionKey>,
    groups: Vec<Vec<Node>>,
}

impl DetailedRouter<'_> {
    /// Routes `nets` region by region from their global routes and track
    /// assignment; a net that cannot be split into regions, or whose piece
    /// in some region cannot be connected, is routed whole instead.
    pub fn route_initial(&mut self, nets: &[NetId], costs: Costs) -> RepairStats {
        let grid = RegionGrid::of_pass(self, 0);
        let planned: Vec<Result<Plan, NetId>> = nets
            .par_iter()
            .map(|&net| self.plan(net, &grid).ok_or(net))
            .collect();
        let mut whole: Vec<NetId> = Vec::new();
        let mut plans: Vec<Plan> = Vec::new();
        for p in planned {
            match p {
                Ok(plan) => plans.push(plan),
                Err(net) => whole.push(net),
            }
        }
        let mut regions: BTreeMap<RegionKey, Vec<Piece>> = BTreeMap::new();
        let mut finished: Vec<(NetId, Vec<AccessPoint>)> = Vec::with_capacity(plans.len());
        let skeletons: Vec<(NetId, NetRoute)> = plans
            .par_iter()
            .map(|plan| {
                let route = NetRoute {
                    edges: self.skeleton(plan.net, &plan.bridges, &grid),
                    ..NetRoute::default()
                };
                (plan.net, route)
            })
            .collect();
        self.commit_all(skeletons);
        for plan in plans {
            for (key, piece) in plan.pieces {
                regions.entry(key).or_default().push(piece);
            }
            finished.push((plan.net, plan.access));
        }

        let mut stats = RepairStats {
            regions: regions.len(),
            ..RepairStats::default()
        };
        let mut failed: FxHashSet<NetId> = FxHashSet::default();
        for batch_colour in 0..4 {
            let batch: Vec<(Rect<Dbu>, Vec<Piece>)> = regions
                .extract_if(.., |&k, _| colour(k) == batch_colour)
                .map(|(k, pieces)| (grid.area(k), pieces))
                .collect();
            stats.pieces += batch.iter().map(|(_, p)| p.len()).sum::<usize>();
            failed.extend(self.route_batch(&batch, costs));
        }

        let mut last: Vec<(NetId, NetRoute)> = Vec::with_capacity(finished.len());
        for (net, access) in finished {
            if failed.contains(&net) {
                last.push((net, NetRoute::default()));
                whole.push(net);
                continue;
            }
            let mut route = self.routes[net.index()].clone();
            route.access = access;
            route.complete = true;
            last.push((net, route));
        }
        self.commit_all(last);
        log::debug!(
            "first pass: {} pieces in {} regions, {} nets routed whole",
            stats.pieces,
            stats.regions,
            whole.len()
        );
        whole.sort_unstable();
        let mut done = Vec::new();
        self.route_speculatively(whole, costs, &mut done);
        stats
    }

    /// Routes the pieces of regions that do not touch each other, after
    /// taking out the wiring their nets have inside them. Returns the nets
    /// with a piece that could not be connected.
    fn route_batch(&mut self, batch: &[(Rect<Dbu>, Vec<Piece>)], costs: Costs) -> Vec<NetId> {
        let mut areas_of: BTreeMap<NetId, Vec<Rect<Dbu>>> = BTreeMap::new();
        for (area, pieces) in batch {
            for piece in pieces {
                areas_of.entry(piece.net).or_default().push(*area);
            }
        }
        let taken_out: Vec<(NetId, NetRoute)> = areas_of
            .par_iter()
            .map(|(&net, areas)| {
                let mut route = self.routes[net.index()].clone();
                route.edges.retain(|e| !self.inside_one(e, areas));
                (net, route)
            })
            .collect();
        self.commit_all(taken_out);

        let routed: Vec<Vec<Routed>> = batch
            .par_iter()
            .map(|(area, pieces)| self.route_region(area, pieces, costs, Marks::default()))
            .collect();

        let mut failed = Vec::new();
        let mut changes = Changes::default();
        for ((_, pieces), routed) in batch.iter().zip(routed) {
            for (i, item) in routed.into_iter().enumerate() {
                match item.wiring {
                    Some(wiring) => changes.apply(item.net, item.removed, wiring),
                    None if i < pieces.len() => failed.push(item.net),
                    None => {}
                }
            }
        }
        self.commit_changes(changes);
        failed
    }

    /// Whether both ends of `edge` lie in the same one of `areas`.
    fn inside_one(&self, edge: Edge, areas: &[Rect<Dbu>]) -> bool {
        let (a, b) = edge.ends();
        let (pa, pb) = (self.point(a), self.point(b));
        areas
            .iter()
            .any(|r| contains_half_open(r, pa) && contains_half_open(r, pb))
    }

    /// The wiring a net starts the pass with: its bridges, and its track
    /// wires inside each region, which stand for its wiring there until the
    /// region is routed.
    fn skeleton(&self, net: NetId, bridges: &[Edge], grid: &RegionGrid) -> EdgeSet {
        let region_of = |n: Node| grid.key(self.point(n));
        self.wires[net.index()]
            .iter()
            .flat_map(|w| w.edges())
            .filter(|e| {
                let (a, b) = e.ends();
                region_of(a) == region_of(b)
            })
            .chain(bridges.iter().copied())
            .collect()
    }

    /// Splits a net's global route into pieces by region, or `None` when it
    /// has no global route, a pin without access, or a boundary crossing no
    /// bridge fits.
    fn plan(&self, net: NetId, grid: &RegionGrid) -> Option<Plan> {
        let guide = self.guides[net.index()].as_ref()?;
        let tree = &guide.tree;
        let cells = tree.cells();
        let access: Vec<AccessPoint> = self.access[net.index()]
            .iter()
            .map(|candidates| candidates.first().cloned())
            .collect::<Option<_>>()?;
        if access.len() < 2 {
            return None;
        }
        let region_of = |c: GCell| grid.key(self.gcell_corner(c));

        let mut sets: Vec<usize> = (0..cells.len()).collect();
        let mut crossings: Vec<(usize, usize)> = Vec::new();
        for (child, parent) in tree.links() {
            if region_of(cells[child]) == region_of(cells[parent]) {
                union(&mut sets, child, parent);
            } else {
                crossings.push((child, parent));
            }
        }

        let mut parts: FxHashMap<usize, Part> = FxHashMap::default();
        for (i, &c) in cells.iter().enumerate() {
            let part = parts.entry(find(&mut sets, i)).or_default();
            part.region = Some(region_of(c));
        }
        let mut bridges = Vec::with_capacity(crossings.len());
        for (child, parent) in crossings {
            let bridge = self.bridge(net, cells[child], cells[parent])?;
            for end in <[Node; 2]>::from(bridge.ends()) {
                let region = grid.key(self.point(end));
                let cell = [child, parent]
                    .into_iter()
                    .find(|&c| region_of(cells[c]) == region)?;
                parts
                    .entry(find(&mut sets, cell))
                    .or_default()
                    .groups
                    .push(vec![end]);
            }
            bridges.push(bridge);
        }
        for (k, a) in access.iter().enumerate() {
            let cell = tree.position(guide.terminals[k])?;
            if grid.key(self.point(a.node)) != region_of(cells[cell]) {
                return None;
            }
            parts
                .entry(find(&mut sets, cell))
                .or_default()
                .groups
                .push(vec![a.node]);
        }

        let mut pieces = Vec::new();
        for part in parts.into_values() {
            let mut groups = part.groups;
            groups.sort_unstable();
            groups.dedup();
            let Some(region) = part.region else {
                continue;
            };
            if groups.len() < 2 {
                continue;
            }
            pieces.push((region, Piece { net, groups }));
        }
        Some(Plan {
            net,
            access,
            bridges,
            pieces,
        })
    }

    /// The lower-left corner of a gcell.
    fn gcell_corner(&self, c: GCell) -> Point<Dbu> {
        self.grid_origin + Point::new(self.gcell * i64::from(c.x), self.gcell * i64::from(c.y))
    }

    /// A wire edge across the boundary between neighbouring gcells `a` and
    /// `b` of one layer: on the track of the net's track wire through it if
    /// there is one, else on the track nearest the middle of their row that
    /// no fixed metal blocks there.
    fn bridge(&self, net: NetId, a: GCell, b: GCell) -> Option<Edge> {
        let layer = a.layer;
        let lt = self.tech.layer(layer);
        let (low, high) = if (a.x, a.y) <= (b.x, b.y) {
            (a, b)
        } else {
            (b, a)
        };
        let (boundary, row_start) = match lt.direction {
            Direction::Horizontal if low.y == high.y && high.x == low.x + 1 => (
                self.grid_origin.x + self.gcell * i64::from(high.x),
                self.grid_origin.y + self.gcell * i64::from(low.y),
            ),
            Direction::Vertical if low.x == high.x && high.y == low.y + 1 => (
                self.grid_origin.y + self.gcell * i64::from(high.y),
                self.grid_origin.x + self.gcell * i64::from(low.x),
            ),
            _ => return None,
        };
        let after = lt.cross.partition_point(|c| c.at < boundary);
        let before = u32::try_from(after.checked_sub(1)?).ok()?;
        if after >= lt.cross.len() {
            return None;
        }
        let row = lt.tracks_within(row_start, row_start + self.gcell - Dbu(1));
        let assigned = self.wires[net.index()].iter().find(|w| {
            w.layer == layer && w.from <= before && before < w.to && row.contains(&w.track)
        });
        if let Some(w) = assigned {
            return Some(Edge::Wire(Node {
                layer,
                track: w.track,
                cross: before,
            }));
        }
        let middle = row_start + self.gcell / 2;
        let mut tracks: Vec<u32> = row.collect();
        tracks.sort_by_key(|&t| ((lt.tracks[t as usize] - middle).abs(), t));
        tracks
            .into_iter()
            .map(|track| {
                Edge::Wire(Node {
                    layer,
                    track,
                    cross: before,
                })
            })
            .find(|&e| !self.blocked_by_fixed(net, e))
    }

    /// Whether fixed metal of another owner conflicts with an edge's wire.
    fn blocked_by_fixed(&self, net: NetId, edge: Edge) -> bool {
        let owner = crate::occupancy::Owner::Net(net.0);
        let route = NetRoute {
            edges: std::iter::once(edge).collect(),
            ..NetRoute::default()
        };
        route
            .metal(self.db, self.tech, net)
            .into_iter()
            .filter(|(layer, _)| self.tech.routable(*layer))
            .any(|(layer, rect)| {
                self.occupancy
                    .rect_conflicts(self.tech, layer, &rect, owner)
                    .any(|c| {
                        c.proximity == crate::occupancy::Proximity::Within
                            && c.occupant.origin == crate::occupancy::Origin::Fixed
                    })
            })
    }
}

const fn find(sets: &mut [usize], i: usize) -> usize {
    let mut root = i;
    while sets[root] != root {
        root = sets[root];
    }
    let mut cur = i;
    while sets[cur] != root {
        let next = sets[cur];
        sets[cur] = root;
        cur = next;
    }
    root
}

fn union(sets: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find(sets, a), find(sets, b));
    if ra != rb {
        sets[ra.max(rb)] = ra.min(rb);
    }
}
