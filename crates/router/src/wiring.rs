//! A net's wiring as edges of the track graph.
//!
//! An edge is a step between neighbouring crossings on one track, or a via
//! from a crossing up to the one above it. Wiring kept as edges can be
//! ripped up piece by piece, its connectivity is graph reachability, and it
//! becomes metal and DEF wiring by merging consecutive steps along a track.

use crate::access::{AccessPoint, Node};
use crate::runs::EdgeSet;
use crate::tech::{LayerTech, RouteTech};
use pare_common::db::design::{NetlistDB, RouteElem};
use pare_common::db::indices::{CutLayerId, LayerId, NetId, ViaId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::shapes::route_elem_shapes;
use rustc_hash::FxHashMap;
use std::collections::BTreeMap;

/// One edge of the track graph.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Edge {
    /// From a crossing to the next one along its track.
    Wire(Node),
    /// From a crossing up to the crossing above it.
    Via {
        /// The crossing on the lower layer.
        lower: Node,
        /// The crossing on the layer above.
        upper: Node,
    },
}

impl Edge {
    /// The edge joining `a` and `b`, if they are neighbouring crossings on
    /// one track or crossings one layer apart.
    pub fn between(a: Node, b: Node) -> Option<Self> {
        if a.layer == b.layer && a.track == b.track && a.cross.abs_diff(b.cross) == 1 {
            return Some(Self::Wire(if a.cross < b.cross { a } else { b }));
        }
        if a.layer.index().abs_diff(b.layer.index()) == 1 {
            let (lower, upper) = if a.layer < b.layer { (a, b) } else { (b, a) };
            return Some(Self::Via { lower, upper });
        }
        None
    }

    /// The two crossings it joins.
    pub const fn ends(self) -> (Node, Node) {
        match self {
            Self::Wire(n) => (
                n,
                Node {
                    cross: n.cross + 1,
                    ..n
                },
            ),
            Self::Via { lower, upper } => (lower, upper),
        }
    }
}

/// The edges along a path of crossings, each a neighbour of the last.
pub fn path_edges(path: &[Node]) -> impl Iterator<Item = Edge> + '_ {
    path.windows(2).filter_map(|w| Edge::between(w[0], w[1]))
}

/// The routing of one net.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct NetRoute {
    /// Edges of the track graph it uses.
    pub edges: EdgeSet,
    /// Metal added beyond its wires and vias.
    pub patches: Vec<Patch>,
    /// The access point used at each pin it reaches.
    pub access: Vec<AccessPoint>,
    /// Whether every pin was connected.
    pub complete: bool,
}

/// A rectangle of metal added to a route beyond its wires and vias, written
/// to DEF as a RECT: `TritonRoute`'s patch wire, which brings a piece of
/// metal up to its layer's minimum area.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Patch {
    /// The layer.
    pub layer: LayerId,
    /// The metal.
    pub rect: Rect<Dbu>,
}

impl Patch {
    /// An order of patches by layer and position.
    const fn sort_key(&self) -> (usize, Dbu, Dbu, Dbu, Dbu) {
        let r = self.rect;
        (self.layer.index(), r.min.x, r.min.y, r.max.x, r.max.y)
    }
}

/// Wiring routed for part of a net: edges of the track graph and the
/// patches on them.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Wiring {
    /// Edges of the track graph.
    pub edges: Vec<Edge>,
    /// Patches on them.
    pub patches: Vec<Patch>,
}

impl Wiring {
    /// Whether there is nothing here.
    pub const fn is_empty(&self) -> bool {
        self.edges.is_empty() && self.patches.is_empty()
    }
}

/// Sorts `patches` and drops repeats.
pub fn tidy_patches(patches: &mut Vec<Patch>) {
    patches.sort_by_key(Patch::sort_key);
    patches.dedup();
}

impl NetRoute {
    /// Its wiring for the design: runs of wire along each track, vias, and
    /// the stub and via stack of every access point.
    pub fn elems(&self, tech: &RouteTech) -> Vec<RouteElem> {
        let point = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
        let mut out: Vec<RouteElem> = self
            .edges
            .vias()
            .iter()
            .map(|&(lower, _)| RouteElem::Via {
                via: tech.via_above(lower.layer).via,
                at: point(lower),
            })
            .collect();
        out.extend(self.edges.runs().iter().map(|run| RouteElem::Wire {
            layer: run.layer,
            from: point(run.node(run.from)),
            to: point(run.node(run.to)),
        }));
        for a in &self.access {
            out.extend(access_elems(tech, a));
        }
        out.extend(self.patches.iter().map(|p| RouteElem::Patch {
            layer: p.layer,
            rect: p.rect,
        }));
        out
    }

    /// Whether its edges join the access points of all its pins.
    pub fn is_connected(&self) -> bool {
        let mut sets = DisjointSets::default();
        for e in self.edges.iter() {
            let (a, b) = e.ends();
            sets.union(a, b);
        }
        let mut roots = self.access.iter().map(|a| sets.find(a.node));
        let first = roots.next();
        roots.all(|r| Some(r) == first)
    }

    /// Its metal, by layer.
    pub fn metal(&self, db: &NetlistDB, tech: &RouteTech, net: NetId) -> Vec<(LayerId, Rect<Dbu>)> {
        self.elems(tech)
            .iter()
            .flat_map(|e| elem_metal(db, net, e))
            .collect()
    }

    /// Its via cuts, by cut layer.
    pub fn cuts(&self, db: &NetlistDB, tech: &RouteTech) -> Vec<(CutLayerId, Rect<Dbu>)> {
        self.elems(tech)
            .iter()
            .flat_map(|e| elem_cuts(db, e))
            .collect()
    }

    /// Every via it places, with its location.
    pub fn vias(&self, tech: &RouteTech) -> Vec<(ViaId, Point<Dbu>)> {
        self.elems(tech)
            .into_iter()
            .filter_map(|e| match e {
                RouteElem::Via { via, at } => Some((via, at)),
                RouteElem::Wire { .. } | RouteElem::Patch { .. } => None,
            })
            .collect()
    }
}

/// Union-find over crossings.
#[derive(Debug, Default)]
pub(crate) struct DisjointSets {
    parent: FxHashMap<Node, Node>,
}

impl DisjointSets {
    pub(crate) fn find(&mut self, n: Node) -> Node {
        let mut root = n;
        while let Some(&p) = self.parent.get(&root) {
            if p == root {
                break;
            }
            root = p;
        }
        let mut cur = n;
        while cur != root {
            let next = self.parent.get(&cur).copied().unwrap_or(root);
            let _ = self.parent.insert(cur, root);
            cur = next;
        }
        root
    }

    pub(crate) fn union(&mut self, a: Node, b: Node) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            let _ = self.parent.insert(ra.max(rb), ra.min(rb));
        }
    }
}

/// Wire edges joining pieces of `route` of `net` that lie along one track
/// closer than the net's spacing without touching, a gap the spacing
/// between shapes of one net forbids.
///
/// Each piece of metal belongs to the track nearest its middle. Only on a
/// track with such a gap are the crossings the route uses looked up, from
/// its runs, its vias and its access points there.
pub fn gap_fills(route: &NetRoute, db: &NetlistDB, tech: &RouteTech, net: NetId) -> Vec<Edge> {
    let mut metal: BTreeMap<(LayerId, u32), Vec<(Dbu, Dbu)>> = BTreeMap::new();
    for (layer, rect) in route.metal(db, tech, net) {
        if !tech.routable(layer) {
            continue;
        }
        let lt = tech.layer(layer);
        let ((a0, a1), (b0, b1)) = lt.across_along_rect(&rect);
        let centre = a0.midpoint(a1);
        let track = lt
            .tracks_within(a0, a1)
            .min_by_key(|&t| (lt.tracks[t as usize] - centre).abs());
        if let Some(track) = track {
            metal.entry((layer, track)).or_default().push((b0, b1));
        }
    }
    let mut fills = Vec::new();
    for ((layer, track), spans) in metal {
        let spacing = tech.wire_spacing(tech.net_rule(net.index()), layer);
        let gaps = narrow_gaps(spans, spacing);
        if gaps.is_empty() {
            continue;
        }
        let used = UsedCrossings::of(route, layer, track);
        let lt = tech.layer(layer);
        for (a, b) in gaps {
            let from = used.last_at_or_before(lt, a);
            let to = used.first_at_or_after(lt, b);
            if let (Some(from), Some(to)) = (from, to) {
                fills.extend((from..to).map(|cross| {
                    Edge::Wire(Node {
                        layer,
                        track,
                        cross,
                    })
                }));
            }
        }
    }
    fills
}

/// The gaps between the merged `spans` of metal along a track that are
/// wider than zero but narrower than `spacing`, as the end of one piece and
/// the start of the next.
fn narrow_gaps(mut spans: Vec<(Dbu, Dbu)>, spacing: Dbu) -> Vec<(Dbu, Dbu)> {
    spans.sort_unstable();
    let mut merged: Vec<(Dbu, Dbu)> = Vec::new();
    for (lo, hi) in spans {
        match merged.last_mut() {
            Some(last) if lo <= last.1 => last.1 = last.1.max(hi),
            _ => merged.push((lo, hi)),
        }
    }
    merged
        .windows(2)
        .filter(|w| w[1].0 - w[0].1 < spacing)
        .map(|w| (w[0].1, w[1].0))
        .collect()
}

/// The crossings a route uses on one track: stretches of consecutive
/// crossings its runs cover, and single crossings its vias and access
/// points stand on.
#[derive(Debug, Default)]
struct UsedCrossings {
    stretches: Vec<(u32, u32)>,
    single: Vec<u32>,
}

impl UsedCrossings {
    fn of(route: &NetRoute, layer: LayerId, track: u32) -> Self {
        let runs = route.edges.runs();
        let first = runs.partition_point(|r| (r.layer, r.track) < (layer, track));
        let stretches = runs[first..]
            .iter()
            .take_while(|r| r.layer == layer && r.track == track)
            .map(|r| (r.from, r.to))
            .collect();
        let on_track = |n: Node| n.layer == layer && n.track == track;
        let single = route
            .edges
            .vias()
            .iter()
            .flat_map(|&pair| <[Node; 2]>::from(pair))
            .chain(route.access.iter().map(|a| a.node))
            .filter(|&n| on_track(n))
            .map(|n| n.cross)
            .collect();
        Self { stretches, single }
    }

    /// The last used crossing at or before `along`.
    fn last_at_or_before(&self, lt: &LayerTech, along: Dbu) -> Option<u32> {
        let at = |c: u32| lt.cross[c as usize].at;
        let k = lt.cross.partition_point(|c| c.at <= along);
        let in_stretches = self.stretches.iter().filter_map(|&(from, to)| {
            let last = u32::try_from(k).ok()?.checked_sub(1)?;
            (last >= from).then(|| last.min(to))
        });
        let singles = self.single.iter().copied().filter(|&c| at(c) <= along);
        in_stretches.chain(singles).max()
    }

    /// The first used crossing at or after `along`.
    fn first_at_or_after(&self, lt: &LayerTech, along: Dbu) -> Option<u32> {
        let at = |c: u32| lt.cross[c as usize].at;
        let k = u32::try_from(lt.cross.partition_point(|c| c.at < along)).ok();
        let in_stretches = self.stretches.iter().filter_map(|&(from, to)| {
            let first = k?;
            (first <= to).then(|| first.max(from))
        });
        let singles = self.single.iter().copied().filter(|&c| at(c) >= along);
        in_stretches.chain(singles).min()
    }
}

/// The stub and via stack joining an access point's node to its pin.
pub fn access_elems(tech: &RouteTech, a: &AccessPoint) -> Vec<RouteElem> {
    let node = tech.layer(a.node.layer).point(a.node.track, a.node.cross);
    let mut out = Vec::new();
    if node != a.at {
        out.push(RouteElem::Wire {
            layer: a.node.layer,
            from: node,
            to: a.at,
        });
    }
    out.extend(a.vias.iter().map(|&via| RouteElem::Via { via, at: a.at }));
    out
}

/// The cuts of one piece of wiring.
pub fn elem_cuts(db: &NetlistDB, elem: &RouteElem) -> Vec<(CutLayerId, Rect<Dbu>)> {
    match *elem {
        RouteElem::Wire { .. } | RouteElem::Patch { .. } => Vec::new(),
        RouteElem::Via { via, at } => db
            .library
            .via(via)
            .shapes
            .iter()
            .filter_map(|s| match s.layer {
                LayerRef::Cut(c) => Some((c, s.rect.translate(at))),
                LayerRef::Routing(_) => None,
            })
            .collect(),
    }
}

/// The metal of one piece of `net`'s wiring on routing layers, at the
/// widths the net's rule gives it, as the checker sees it.
pub fn elem_metal(db: &NetlistDB, net: NetId, elem: &RouteElem) -> Vec<(LayerId, Rect<Dbu>)> {
    route_elem_shapes(db, net, elem)
        .into_iter()
        .filter_map(|s| match s.layer {
            LayerRef::Routing(l) => Some((l, s.rect)),
            LayerRef::Cut(_) => None,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(layer: u8, track: u32, cross: u32) -> Node {
        Node {
            layer: LayerId(layer),
            track,
            cross,
        }
    }

    #[test]
    fn neighbours_on_a_track_or_a_layer_apart_are_joined_by_one_edge() {
        assert_eq!(
            Edge::between(node(1, 4, 7), node(1, 4, 6)),
            Some(Edge::Wire(node(1, 4, 6)))
        );
        assert_eq!(
            Edge::between(node(2, 9, 3), node(1, 4, 7)),
            Some(Edge::Via {
                lower: node(1, 4, 7),
                upper: node(2, 9, 3)
            })
        );
        assert_eq!(Edge::between(node(1, 4, 7), node(1, 5, 7)), None);
    }

    #[test]
    fn only_gaps_narrower_than_the_spacing_are_reported() {
        let spans = vec![
            (Dbu(0), Dbu(100)),
            (Dbu(150), Dbu(300)),
            (Dbu(280), Dbu(400)),
            (Dbu(600), Dbu(700)),
        ];

        assert_eq!(narrow_gaps(spans, Dbu(70)), [(Dbu(100), Dbu(150))]);
    }

    #[test]
    fn joined_crossings_share_a_root() {
        let mut sets = DisjointSets::default();

        sets.union(node(1, 4, 1), node(1, 4, 2));
        sets.union(node(1, 4, 3), node(1, 4, 2));

        assert_eq!(sets.find(node(1, 4, 3)), sets.find(node(1, 4, 1)));
        assert_ne!(sets.find(node(1, 4, 4)), sets.find(node(1, 4, 1)));
    }

    #[test]
    fn a_path_that_doubles_back_uses_each_step_once() {
        let path = [node(1, 4, 5), node(1, 4, 6), node(1, 4, 7), node(1, 4, 6)];

        let mut edges: Vec<Edge> = path_edges(&path).collect();
        edges.sort_unstable();
        edges.dedup();

        assert_eq!(
            edges,
            [Edge::Wire(node(1, 4, 5)), Edge::Wire(node(1, 4, 6))]
        );
    }
}
