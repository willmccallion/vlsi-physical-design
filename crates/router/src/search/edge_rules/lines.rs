//! The lines of edges out of the access points of macro and I/O pins, on
//! which `TritonRoute` turns its grid cost off for the pin's own net and on
//! for every other (`FlexDRWorker::initMazeCost_ap_helper` and
//! `initMazeCost_ap_planarGrid_helper`).

use crate::access::{Node, Way};
use crate::tech::{RouteTech, TrackIdx};
use crate::wiring::Edge;
use pare_common::db::indices::{LayerId, NetId};
use pare_common::geom::dbu::Dbu;
use rustc_hash::FxHashSet;

/// How far a line runs from its access point, in default widths of the
/// point's layer: `planarGridBloatNumWidth`.
const LINE_WIDTHS: i64 = 10;

/// What grid cost an edge pays besides running against its layer's
/// preferred direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum GridCost {
    /// None but that: a jog pays, a wire or via does not.
    AsLaidOut,
    /// None at all, on a line out of the net's own access point.
    Off,
    /// The grid cost on any edge, on a line out of another net's access
    /// point only.
    On,
}

/// The edges on the lines out of access points, and whose points' lines
/// they are on.
#[derive(Debug, Default)]
pub(super) struct Lines {
    edges: FxHashSet<Edge>,
    owners: FxHashSet<(Edge, NetId)>,
}

impl Lines {
    /// Adds the lines out of `net`'s access point at `node`, one each way
    /// along and across its tracks: each step from it while the steps
    /// after the first run no further than [`LINE_WIDTHS`] widths, with the
    /// vias up and down at every node a step leaves from.
    pub(super) fn add(&mut self, tech: &RouteTech, net: NetId, node: Node) {
        let reach = tech.layer(node.layer).width * LINE_WIDTHS;
        for way in Way::PLANAR {
            let mut at = node;
            let mut walked = Dbu::ZERO;
            while walked <= reach {
                let Some((edge, next)) = step(tech, at, way) else {
                    break;
                };
                self.insert(edge, net);
                for via in vias_at(tech, at) {
                    self.insert(via, net);
                }
                walked += step(tech, next, way)
                    .map_or(Dbu::ZERO, |(_, after)| distance(tech, next, after));
                at = next;
            }
        }
    }

    fn insert(&mut self, edge: Edge, net: NetId) {
        let _ = self.edges.insert(edge);
        let _ = self.owners.insert((edge, net));
    }

    /// The grid cost `edge` pays `net`, or a search of no net's.
    // NOTE: TritonRoute turns a pin's lines on for other nets only once the
    // pin's net is routed in the worker (mazeNetEnd); PARE takes every net
    // as routed, so they are always on for other nets.
    pub(super) fn grid_cost(&self, net: Option<NetId>, edge: Edge) -> GridCost {
        if !self.edges.contains(&edge) {
            GridCost::AsLaidOut
        } else if net.is_some_and(|n| self.owners.contains(&(edge, n))) {
            GridCost::Off
        } else {
            GridCost::On
        }
    }
}

/// The edge from `at` by `way`, a planar way, with the node it reaches;
/// none where the graph has no such edge.
fn step(tech: &RouteTech, at: Node, way: Way) -> Option<(Edge, Node)> {
    let lt = tech.layer(at.layer);
    match way {
        Way::AlongHigher => {
            let next = Node {
                cross: at.cross.next(),
                ..at
            };
            (next.cross.index() < lt.cross.len()).then_some((Edge::Wire(at), next))
        }
        Way::AlongLower => {
            let next = Node {
                cross: at.cross.prev()?,
                ..at
            };
            Some((Edge::Wire(next), next))
        }
        Way::AcrossHigher => {
            let next = Node {
                track: at.track.plus(1),
                ..at
            };
            tech.jog_allowed(at).then_some((Edge::Jog(at), next))
        }
        Way::AcrossLower => {
            let next = Node {
                track: TrackIdx::new(at.track.index().checked_sub(1)?),
                ..at
            };
            tech.jog_allowed(next).then_some((Edge::Jog(next), next))
        }
        Way::Down | Way::Up => None,
    }
}

/// The vias up and down from `at`, where the layer above or below has a
/// node right there.
fn vias_at(tech: &RouteTech, at: Node) -> impl Iterator<Item = Edge> {
    let above = node_above(tech, at).map(|upper| Edge::Via { lower: at, upper });
    let below = (at.layer > tech.min_layer)
        .then(|| LayerId::new(at.layer.index() - 1))
        .and_then(|layer| {
            let lt = tech.layer(at.layer);
            Some(Node {
                layer,
                track: lt.cross[at.cross].down?,
                cross: lt.track_down[at.track]?,
            })
        })
        .map(|lower| Edge::Via { lower, upper: at });
    above.into_iter().chain(below)
}

/// The node right above `at` on the next routable layer up, if it has one.
fn node_above(tech: &RouteTech, at: Node) -> Option<Node> {
    if at.layer >= tech.max_layer {
        return None;
    }
    let lt = tech.layer(at.layer);
    Some(Node {
        layer: LayerId::new(at.layer.index() + 1),
        track: lt.cross[at.cross].up?,
        cross: lt.track_up[at.track]?,
    })
}

/// The length of the edge between neighbouring nodes `a` and `b`.
fn distance(tech: &RouteTech, a: Node, b: Node) -> Dbu {
    let point = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
    point(a).manhattan(point(b))
}
