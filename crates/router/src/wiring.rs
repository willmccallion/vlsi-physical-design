//! A net's wiring as edges of the track graph.
//!
//! An edge is a step between neighbouring crossings on one track, a jog
//! across to the same crossing of the next track, or a via from a crossing
//! up to the one above it. Wiring kept as edges can be
//! ripped up piece by piece, its connectivity is graph reachability, and it
//! becomes metal and DEF wiring by merging consecutive steps along a track.

use crate::access::{AccessPoint, Node};
use crate::runs::{EdgeSet, Run};
use crate::tech::{CrossIdx, IndexRange, TrackIdx};
use crate::tech::{LayerTech, RouteTech};
use pare_common::db::design::{NetlistDB, RouteElem};
use pare_common::db::indices::{CutLayerId, LayerId, NetId};
use pare_common::db::library::LayerRef;
use pare_common::db::shapes::route_elem_shapes;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::{Interval, merge_touching};
use pare_common::geom::rect::Rect;
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
    /// From a crossing across to the same crossing of the next track: a
    /// wire against its layer's preferred direction.
    Jog(Node),
}

impl Edge {
    /// The edge joining `a` and `b`, if they are neighbouring crossings on
    /// one track, the same crossing of neighbouring tracks, or crossings one
    /// layer apart.
    pub fn between(a: Node, b: Node) -> Option<Self> {
        if a.layer == b.layer && a.track == b.track && a.cross.offset_from(b.cross).abs() == 1 {
            return Some(Self::Wire(if a.cross < b.cross { a } else { b }));
        }
        if a.layer == b.layer && a.cross == b.cross && a.track.0.abs_diff(b.track.0) == 1 {
            return Some(Self::Jog(if a.track < b.track { a } else { b }));
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
                    cross: n.cross.next(),
                    ..n
                },
            ),
            Self::Via { lower, upper } => (lower, upper),
            Self::Jog(n) => (
                n,
                Node {
                    track: n.track.plus(1),
                    ..n
                },
            ),
        }
    }
}

/// The edges along a path of crossings, each a neighbour of the last.
pub(crate) fn path_edges(path: &[Node]) -> impl Iterator<Item = Edge> + '_ {
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
    /// The routing layer of the patch.
    pub layer: LayerId,
    /// The patch's metal, in database units.
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
pub(crate) fn tidy_patches(patches: &mut Vec<Patch>) {
    patches.sort_by_key(Patch::sort_key);
    patches.dedup();
}

impl NetRoute {
    /// The part of it at `area`: the wire edges whose stretch along their
    /// track meets it, the vias, access points and patches in it.
    #[must_use]
    pub fn near(&self, tech: &RouteTech, area: &Rect<Dbu>) -> Self {
        let point = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
        let clip = |run: Run| {
            let lt = tech.layer(run.layer);
            let (Interval { lo: a0, hi: a1 }, Interval { lo: b0, hi: b1 }) =
                lt.across_along_rect(area);
            if !lt.tracks_within(a0, a1).contains(run.track) {
                return None;
            }
            let crosses = lt.cross_within(b0, b1);
            let from = run.from.max(crosses.start.prev().unwrap_or(crosses.start));
            let to = run.to.min(crosses.end);
            (from < to).then_some(Run { from, to, ..run })
        };
        Self {
            edges: self.edges.clipped(clip, |(a, b)| {
                area.contains(point(a)) || area.contains(point(b))
            }),
            patches: self
                .patches
                .iter()
                .copied()
                .filter(|p| p.rect.touches(area))
                .collect(),
            access: self
                .access
                .iter()
                .filter(|a| {
                    Rect::new(a.at, a.at)
                        .union(&Rect::new(point(a.node), point(a.node)))
                        .touches(area)
                })
                .cloned()
                .collect(),
            complete: self.complete,
        }
    }

    /// Its wiring for the design: runs of wire along each track, vias, runs
    /// of jogs across tracks, and the stub and via stack of every access
    /// point. A wire ending at the node of an access point that touches its
    /// pin there stops at the node, as `FlexDRWorker::checkPathSegStyle`
    /// cuts a path off at an access point with no path of its own.
    pub fn elems(&self, tech: &RouteTech) -> Vec<RouteElem> {
        let point = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
        let on_pin: Vec<Node> = self
            .access
            .iter()
            .filter(|a| a.pin_layer == a.node.layer && a.at == point(a.node))
            .map(|a| a.node)
            .collect();
        let extension = |n: Node| on_pin.contains(&n).then_some(Dbu::ZERO);
        let wire = |from: Node, to: Node| RouteElem::Wire {
            layer: from.layer,
            from: point(from),
            to: point(to),
            extensions: (extension(from), extension(to)),
        };
        let mut out: Vec<RouteElem> = self
            .edges
            .vias()
            .iter()
            .map(|&(lower, _)| RouteElem::Via {
                via: tech.via_above(lower.layer).via,
                at: point(lower),
            })
            .collect();
        out.extend(
            self.edges
                .runs()
                .iter()
                .map(|run| wire(run.node(run.from), run.node(run.to))),
        );
        out.extend(
            jog_runs(self.edges.jogs())
                .into_iter()
                .map(|(first, last)| {
                    let end = Node {
                        track: last.track.plus(1),
                        ..last
                    };
                    wire(first, end)
                }),
        );
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
}

/// The jogs, as each run of them across consecutive tracks at one crossing
/// of one layer, by its first and last jog.
fn jog_runs(jogs: &[Node]) -> Vec<(Node, Node)> {
    let mut across: Vec<Node> = jogs.to_vec();
    across.sort_unstable_by_key(|n| (n.layer, n.cross, n.track));
    across
        .chunk_by(|a, b| a.layer == b.layer && a.cross == b.cross && a.track.plus(1) == b.track)
        .map(|run| (run[0], run[run.len() - 1]))
        .collect()
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
pub(crate) fn gap_fills(
    route: &NetRoute,
    db: &NetlistDB,
    tech: &RouteTech,
    net: NetId,
) -> Vec<Edge> {
    let mut metal: BTreeMap<(LayerId, TrackIdx), Vec<Interval>> = BTreeMap::new();
    for (layer, rect) in route.metal(db, tech, net) {
        if !tech.routable(layer) {
            continue;
        }
        let lt = tech.layer(layer);
        let (Interval { lo: a0, hi: a1 }, Interval { lo: b0, hi: b1 }) =
            lt.across_along_rect(&rect);
        let centre = a0.midpoint(a1);
        let track = lt
            .tracks_within(a0, a1)
            .min_by_key(|&t| (lt.tracks[t] - centre).abs());
        if let Some(track) = track {
            metal
                .entry((layer, track))
                .or_default()
                .push(Interval::new(b0, b1));
        }
    }
    let mut fills = Vec::new();
    for ((layer, track), spans) in metal {
        let spacing = tech.wire_spacing(tech.net_rule(net), layer);
        let gaps = narrow_gaps(spans, spacing);
        if gaps.is_empty() {
            continue;
        }
        let used = UsedCrossings::of(route, layer, track);
        let lt = tech.layer(layer);
        for gap in gaps {
            let from = used.last_at_or_before(lt, gap.lo);
            let to = used.first_at_or_after(lt, gap.hi);
            if let (Some(from), Some(to)) = (from, to) {
                fills.extend(
                    (IndexRange {
                        start: from,
                        end: to,
                    })
                    .map(|cross| {
                        Edge::Wire(Node {
                            layer,
                            track,
                            cross,
                        })
                    }),
                );
            }
        }
    }
    fills
}

/// The gaps between the merged `spans` of metal along a track that are
/// wider than zero but narrower than `spacing`, as the end of one piece and
/// the start of the next.
fn narrow_gaps(mut spans: Vec<Interval>, spacing: Dbu) -> Vec<Interval> {
    spans.sort_unstable();
    merge_touching(spans)
        .windows(2)
        .map(|w| Interval::new(w[0].hi, w[1].lo))
        .filter(|gap| gap.len() < spacing)
        .collect()
}

/// The crossings a route uses on one track: stretches of consecutive
/// crossings its runs cover, and single crossings its vias, jogs and access
/// points stand on.
#[derive(Debug, Default)]
struct UsedCrossings {
    stretches: Vec<(CrossIdx, CrossIdx)>,
    single: Vec<CrossIdx>,
}

impl UsedCrossings {
    fn of(route: &NetRoute, layer: LayerId, track: TrackIdx) -> Self {
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
            .chain(
                route
                    .edges
                    .jogs()
                    .iter()
                    .flat_map(|&n| <[Node; 2]>::from(Edge::Jog(n).ends())),
            )
            .chain(route.access.iter().map(|a| a.node))
            .filter(|&n| on_track(n))
            .map(|n| n.cross)
            .collect();
        Self { stretches, single }
    }

    /// The last used crossing at or before `along`.
    fn last_at_or_before(&self, lt: &LayerTech, along: Dbu) -> Option<CrossIdx> {
        let at = |c: CrossIdx| lt.cross[c].at;
        let k = lt.cross.as_slice().partition_point(|c| c.at <= along);
        let in_stretches = self.stretches.iter().filter_map(|&(from, to)| {
            let last = k.checked_sub(1).map(CrossIdx::new)?;
            (last >= from).then(|| last.min(to))
        });
        let singles = self.single.iter().copied().filter(|&c| at(c) <= along);
        in_stretches.chain(singles).max()
    }

    /// The first used crossing at or after `along`.
    fn first_at_or_after(&self, lt: &LayerTech, along: Dbu) -> Option<CrossIdx> {
        let at = |c: CrossIdx| lt.cross[c].at;
        let first = CrossIdx::new(lt.cross.as_slice().partition_point(|c| c.at < along));
        let in_stretches = self
            .stretches
            .iter()
            .filter(|&&(_, to)| first <= to)
            .map(|&(from, _)| first.max(from));
        let singles = self.single.iter().copied().filter(|&c| at(c) >= along);
        in_stretches.chain(singles).min()
    }
}

/// The stub and via stack joining an access point's node to its pin.
pub(crate) fn access_elems(tech: &RouteTech, a: &AccessPoint) -> Vec<RouteElem> {
    let node = tech.layer(a.node.layer).point(a.node.track, a.node.cross);
    let mut out = Vec::new();
    if node != a.at {
        out.push(RouteElem::Wire {
            layer: a.node.layer,
            from: node,
            to: a.at,
            extensions: (None, None),
        });
    }
    out.extend(a.vias.iter().map(|&via| RouteElem::Via { via, at: a.at }));
    out
}

/// The cuts of one piece of wiring.
pub(crate) fn elem_cuts(db: &NetlistDB, elem: &RouteElem) -> Vec<(CutLayerId, Rect<Dbu>)> {
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
pub(crate) fn elem_metal(
    db: &NetlistDB,
    net: NetId,
    elem: &RouteElem,
) -> Vec<(LayerId, Rect<Dbu>)> {
    route_elem_shapes(db, net, elem)
        .into_iter()
        .filter_map(|s| match s.layer {
            LayerRef::Routing(l) => Some((l, s.rect)),
            LayerRef::Cut(_) => None,
        })
        .collect()
}

#[cfg(test)]
mod tests;
