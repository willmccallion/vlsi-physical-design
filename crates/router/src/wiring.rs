//! A net's wiring as edges of the track graph.
//!
//! An edge is a step between neighbouring crossings on one track, or a via
//! from a crossing up to the one above it. Wiring kept as edges can be
//! ripped up piece by piece, its connectivity is graph reachability, and it
//! becomes metal and DEF wiring by merging consecutive steps along a track.

use crate::access::{AccessPoint, Node};
use crate::tech::RouteTech;
use pare_common::db::design::{NetlistDB, RouteElem};
use pare_common::db::indices::{LayerId, ViaId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::shapes::wire_rect;
use rustc_hash::FxHashMap;

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
#[derive(Clone, Debug, Default)]
pub struct NetRoute {
    /// Edges of the track graph it uses.
    pub edges: Vec<Edge>,
    /// The access point used at each pin it reaches.
    pub access: Vec<AccessPoint>,
    /// Whether every pin was connected.
    pub complete: bool,
}

impl NetRoute {
    /// Its wiring for the design: runs of wire along each track, vias, and
    /// the stub and via stack of every access point.
    pub fn elems(&self, tech: &RouteTech) -> Vec<RouteElem> {
        let point = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
        let mut wires: Vec<Node> = Vec::new();
        let mut out = Vec::new();
        for &e in &self.edges {
            match e {
                Edge::Wire(n) => wires.push(n),
                Edge::Via { lower, .. } => out.push(RouteElem::Via {
                    via: tech.via_above(lower.layer).via,
                    at: point(lower),
                }),
            }
        }
        wires.sort_unstable();
        wires.dedup();
        let mut i = 0;
        while i < wires.len() {
            let start = wires[i];
            let mut end = start;
            while i + 1 < wires.len()
                && wires[i + 1].layer == end.layer
                && wires[i + 1].track == end.track
                && wires[i + 1].cross == end.cross + 1
            {
                i += 1;
                end = wires[i];
            }
            let last = Node {
                cross: end.cross + 1,
                ..end
            };
            out.push(RouteElem::Wire {
                layer: start.layer,
                from: point(start),
                to: point(last),
            });
            i += 1;
        }
        for a in &self.access {
            out.extend(access_elems(tech, a));
        }
        out
    }

    /// Whether its edges join the access points of all its pins.
    pub fn is_connected(&self) -> bool {
        let mut sets = DisjointSets::default();
        for e in &self.edges {
            let (a, b) = e.ends();
            sets.union(a, b);
        }
        let mut roots = self.access.iter().map(|a| sets.find(a.node));
        let first = roots.next();
        roots.all(|r| Some(r) == first)
    }

    /// Its metal, by layer.
    pub fn metal(&self, db: &NetlistDB, tech: &RouteTech) -> Vec<(LayerId, Rect<Dbu>)> {
        self.elems(tech)
            .iter()
            .flat_map(|e| elem_metal(db, e))
            .collect()
    }

    /// Every via it places, with its location.
    pub fn vias(&self, tech: &RouteTech) -> Vec<(ViaId, Point<Dbu>)> {
        self.elems(tech)
            .into_iter()
            .filter_map(|e| match e {
                RouteElem::Via { via, at } => Some((via, at)),
                RouteElem::Wire { .. } => None,
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

/// The metal of one piece of wiring on routing layers.
pub fn elem_metal(db: &NetlistDB, elem: &RouteElem) -> Vec<(LayerId, Rect<Dbu>)> {
    match *elem {
        RouteElem::Wire { layer, from, to } => wire_rect(from, to, db.library.layer(layer).width)
            .map(|r| (layer, r))
            .into_iter()
            .collect(),
        RouteElem::Via { via, at } => db
            .library
            .via(via)
            .shapes
            .iter()
            .filter_map(|s| match s.layer {
                LayerRef::Routing(l) => Some((l, s.rect.translate(at))),
                LayerRef::Cut(_) => None,
            })
            .collect(),
    }
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
