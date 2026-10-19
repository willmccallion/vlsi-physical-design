//! Logs of chosen nets or areas, detailed enough to see why routing does
//! what it does there: each search's route with what it cost, part by part,
//! how each region's result was judged and whether it was kept, and where
//! violations raised the conflict history.

use crate::access::Node;
use crate::occupancy::Owner;
use crate::search::Explanation;
use crate::tech::RouteTech;
use crate::wiring::Edge;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::NetId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::rect::Rect;
use rustc_hash::FxHashSet;
use std::fmt::Write as _;

/// What to trace: the given nets wherever they are routed, and every net
/// routed in a region meeting the given area.
#[derive(Clone, Debug, Default)]
pub struct Trace {
    nets: FxHashSet<NetId>,
    area: Option<Rect<Dbu>>,
}

impl Trace {
    /// A trace of `nets` and of the regions meeting `area`.
    pub fn new(nets: impl IntoIterator<Item = NetId>, area: Option<Rect<Dbu>>) -> Self {
        Self {
            nets: nets.into_iter().collect(),
            area,
        }
    }

    /// Whether anything is traced.
    pub fn is_empty(&self) -> bool {
        self.nets.is_empty() && self.area.is_none()
    }

    /// Whether to trace `net`'s routing in the region `region`.
    pub(super) fn wants(&self, net: NetId, region: &Rect<Dbu>) -> bool {
        self.nets.contains(&net) || self.meets(region)
    }

    /// Whether the traced area meets `region`.
    pub(super) fn meets(&self, region: &Rect<Dbu>) -> bool {
        self.area.is_some_and(|a| a.touches(region))
    }

    /// Whether to trace a region holding `nets`.
    pub(super) fn wants_region(
        &self,
        region: &Rect<Dbu>,
        mut nets: impl Iterator<Item = NetId>,
    ) -> bool {
        self.meets(region) || nets.any(|n| self.nets.contains(&n))
    }
}

/// A region as `(x0, y0)-(x1, y1)` in database units.
pub(super) fn region_text(r: &Rect<Dbu>) -> String {
    format!(
        "({}, {})-({}, {})",
        r.min.x.get(),
        r.min.y.get(),
        r.max.x.get(),
        r.max.y.get()
    )
}

/// The name of an owner of metal.
pub(super) fn owner_text(db: &NetlistDB, owner: Owner) -> String {
    match owner {
        Owner::Net(n) => db.nets[n as usize].name.clone(),
        Owner::Obstacle => "an obstacle".to_string(),
    }
}

/// A route as its runs along tracks and its vias, in database units: each
/// maximal run of wire edges on one track as `layer (x0, y0)-(x1, y1)` and
/// each via as `layer^ (x, y)` on the layer below it.
pub(super) fn route_text(db: &NetlistDB, tech: &RouteTech, edges: &[Edge]) -> String {
    let mut wires: Vec<Node> = Vec::new();
    let mut vias: Vec<Node> = Vec::new();
    for edge in edges {
        match *edge {
            Edge::Wire(node) => wires.push(node),
            Edge::Via { lower, .. } => vias.push(lower),
        }
    }
    wires.sort_unstable();
    let mut out = String::new();
    let runs = wires.chunk_by(|prev, next| {
        prev.layer == next.layer && prev.track == next.track && prev.cross + 1 == next.cross
    });
    for run in runs {
        let (start, end) = (run[0], run[run.len() - 1]);
        let lt = tech.layer(start.layer);
        let from = lt.point(start.track, start.cross);
        let to = lt.point(end.track, end.cross + 1);
        let _ = write!(
            out,
            " {} ({}, {})-({}, {})",
            db.library.layer(start.layer).name,
            from.x.get(),
            from.y.get(),
            to.x.get(),
            to.y.get()
        );
    }
    for via in vias {
        let at = tech.layer(via.layer).point(via.track, via.cross);
        let _ = write!(
            out,
            " {}^ ({}, {})",
            db.library.layer(via.layer).name,
            at.x.get(),
            at.y.get()
        );
    }
    out.trim_start().to_string()
}

/// An explanation as its total and its parts.
pub(super) fn cost_text(db: &NetlistDB, e: &Explanation) -> String {
    let mut conflicts = e.conflicts.clone();
    conflicts.sort_by(|a, b| b.1.total_cmp(&a.1));
    let mut out = format!(
        "{:.0} = length {:.0} + {} vias {:.0} + history {:.0} + own vias {:.0}",
        e.total(),
        e.length,
        e.vias,
        e.via_cost,
        e.history,
        e.own_vias
    );
    for (owner, cost) in conflicts {
        let _ = write!(out, " + {cost:.0} near {}", owner_text(db, owner));
    }
    out
}
