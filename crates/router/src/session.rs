//! Global routes kept between global and detailed routing, as `OpenROAD`
//! keeps its global router's routes while repair edits the design.
//!
//! A net is routed again once an edit moves one of its pins to another
//! gcell, and its route is handed out as segments between gcell centers
//! with where each pin meets them.

use crate::access::{access_points, fellow_boxes, order_toward};
use crate::detailed::{PinJudge, PinTerminal};
use crate::global::{GCell, GlobalGrid, GlobalNet, GlobalRoutes, NetTree, reroute};
use crate::tech::{RouteSetup, RouteTech};
use crate::{GlobalStage, Progress, RouteError, check_routable, gcell_size, global_stage};
use pare_common::db::design::{NetlistDB, PinOwner, Renumbering};
use pare_common::db::global_route::{
    NetRoute, PinGridLocation, RoutePoint, RouteSegment, RouteSource,
};
use pare_common::db::indices::{CellId, LayerId, MacroId, NetId, PinId};
use pare_common::db::library::LayerRef;
use pare_common::db::shapes::is_signal_net;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rustc_hash::{FxHashMap, FxHashSet};

/// A design's global routes, kept in step with its netlist.
#[derive(Debug)]
pub struct GlobalSession {
    tech: RouteTech,
    grid: GlobalGrid,
    routes: GlobalRoutes,
    /// The pins' terminals of each routed net, whose route `routes` holds
    /// with the net it was routed for.
    nets: FxHashMap<NetId, Vec<PinTerminal>>,
    judge: PinJudge,
    /// Each pin's terminal as global routing found it.
    initial: FxHashMap<PinId, PinTerminal>,
    /// Each cell's master, location and orientation as global routing
    /// found them; a pin of a cell that keeps them keeps its terminal.
    placed: Vec<(MacroId, Point<Dbu>, Orientation)>,
    rounds: usize,
    /// While a trial is open, each net it changed as it stood before.
    trial: Option<FxHashMap<NetId, Before>>,
}

/// A net as it stood when an open trial first changed it.
#[derive(Debug)]
struct Before {
    /// Its pins' terminals then; none when it had no route.
    terminals: Option<Vec<PinTerminal>>,
    /// Its route, once the trial has taken it off the grid; none while
    /// it is still there, or when it had none.
    lifted: Option<NetTree>,
}

impl GlobalSession {
    /// Finds pin access and routes every signal net of `db` globally under
    /// `setup`.
    ///
    /// # Errors
    ///
    /// When `db` cannot be routed under `config`.
    pub fn new(
        db: &NetlistDB,
        setup: &RouteSetup<'_>,
        report: &mut dyn FnMut(&Progress<'_>),
    ) -> Result<Self, RouteError> {
        check_routable(db)?;
        let tech = RouteTech::new(db, setup)?;
        let config = setup.config;
        let gcell = gcell_size(&tech, config);
        let (grid, routes, nets, judge) = {
            let GlobalStage {
                dr, grid, global, ..
            } = global_stage(db, &tech, config, gcell, None, report)?;
            let mut nets = FxHashMap::default();
            for &net in global.trees.keys() {
                if let Some(terminals) = dr.pin_terminals(net, &grid) {
                    let _ = nets.insert(net, terminals);
                }
            }
            (grid, global, nets, dr.into_pin_judge())
        };
        let initial = nets
            .values()
            .flat_map(|terminals| terminals.iter().map(|t| (t.pin, *t)))
            .collect();
        let placed = db
            .cells
            .iter()
            .map(|c| (c.macro_id, c.location, c.orient))
            .collect();
        Ok(Self {
            tech,
            grid,
            routes,
            nets,
            judge,
            initial,
            placed,
            rounds: config.global_iterations,
            trial: None,
        })
    }

    /// The technology, with each net's rule as `db` has it now, grid and
    /// routes, for detailed routing.
    pub(crate) fn into_parts(self, db: &NetlistDB) -> (RouteTech, GlobalGrid, GlobalRoutes) {
        let mut tech = self.tech;
        tech.follow_net_rules(db);
        (tech, self.grid, self.routes)
    }

    /// Follows the design's renumbering after cells and nets were removed:
    /// the routes of nets it dropped are lifted, the rest keep theirs.
    pub fn renumber(&mut self, renumbering: &Renumbering) {
        debug_assert!(self.trial.is_none(), "renumbering inside a trial");
        let pin = |p: PinId| renumbering.pins.get(p).copied().flatten();
        let mut nets = FxHashMap::default();
        let mut trees = FxHashMap::default();
        for (net, terminals) in std::mem::take(&mut self.nets) {
            let tree = self.routes.trees.remove(&net);
            let Some(new) = renumbering.nets.get(net).copied().flatten() else {
                if let Some(placed) = tree {
                    let _ = self.grid.lift(placed);
                }
                continue;
            };
            let terminals = terminals
                .into_iter()
                .filter_map(|t| {
                    Some(PinTerminal {
                        pin: pin(t.pin)?,
                        ..t
                    })
                })
                .collect();
            if let Some(mut placed) = tree {
                placed.renumber(new);
                let _ = trees.insert(new, placed);
            }
            let _ = nets.insert(new, terminals);
        }
        self.nets = nets;
        self.routes.trees = trees;
        self.initial = std::mem::take(&mut self.initial)
            .into_iter()
            .filter_map(|(p, t)| {
                let new = pin(p)?;
                Some((new, PinTerminal { pin: new, ..t }))
            })
            .collect();
        let mut placed = Vec::with_capacity(self.placed.len());
        for (old, &at) in self.placed.iter().enumerate() {
            if renumbering
                .cells
                .get(CellId::new(old))
                .copied()
                .flatten()
                .is_some()
            {
                placed.push(at);
            }
        }
        self.placed = placed;
    }

    /// The terminal of `pin` on `net`: as global routing found it while
    /// its cell stays as it was, else from the pin's best access point now,
    /// the one nearest `fellows`, the box of the net's other pins.
    fn terminal(
        &self,
        db: &NetlistDB,
        net: NetId,
        pin: PinId,
        fellows: Option<&Rect<Dbu>>,
    ) -> Option<PinTerminal> {
        if self.unchanged(db, pin)
            && let Some(&terminal) = self.initial.get(&pin)
        {
            return Some(terminal);
        }
        let clean = |layer: LayerId, metal: &[_]| self.judge.clean(net, layer, metal);
        let mut candidates = access_points(db, &self.tech, pin, &clean);
        if let Some(fellows) = fellows {
            order_toward(&mut candidates, fellows);
        }
        let best = candidates.first()?;
        Some(PinTerminal::of(pin, best, &self.tech, &self.grid))
    }

    /// Whether `pin` belongs to a port or to a cell with the master,
    /// location and orientation it had when routing began.
    fn unchanged(&self, db: &NetlistDB, pin: PinId) -> bool {
        match db.pins[pin].owner {
            PinOwner::Io(_) => true,
            PinOwner::Cell { cell, .. } => {
                let c = &db.cells[cell];
                !c.retired
                    && self.placed.get(cell.index()) == Some(&(c.macro_id, c.location, c.orient))
            }
        }
    }

    /// The terminals of every pin of `net`, or none when it is not a signal
    /// net with two pins or a pin has no access point.
    fn terminals(&self, db: &NetlistDB, net: NetId) -> Option<Vec<PinTerminal>> {
        if !is_signal_net(db, net) {
            return None;
        }
        let pins = &db.nets[net].pins;
        let points: Vec<Point<Dbu>> = pins.iter().map(|&p| db.pin_position(p)).collect();
        pins.iter()
            .zip(fellow_boxes(&points))
            .map(|(&pin, fellows)| self.terminal(db, net, pin, fellows.as_ref()))
            .collect()
    }

    /// Notes `net` as it stands, if an open trial has not changed it yet.
    fn remember(&mut self, net: NetId) {
        let Some(trial) = self.trial.as_mut() else {
            return;
        };
        let _ = trial.entry(net).or_insert_with(|| Before {
            terminals: self.nets.get(&net).cloned(),
            lifted: None,
        });
    }

    /// Takes `net`'s route off the grid and forgets it, keeping it for an
    /// open trial when it is the route the net had before the trial.
    fn rip_up(&mut self, net: NetId) {
        self.remember(net);
        if self.nets.remove(&net).is_none() {
            return;
        }
        let Some(placed) = self.routes.trees.remove(&net) else {
            return;
        };
        let lifted = self.grid.lift(placed);
        if let Some(before) = self.trial.as_mut().and_then(|t| t.get_mut(&net))
            && before.terminals.is_some()
            && before.lifted.is_none()
        {
            before.lifted = Some(lifted);
        }
    }

    /// Puts `net` back as it stood before the trial: its first route on
    /// the grid again, in place of any the trial made, with its terminals.
    fn put_back(&mut self, net: NetId, before: Before) {
        let Before { terminals, lifted } = before;
        let original_on_grid = terminals.is_some() && lifted.is_none();
        if !original_on_grid && let Some(placed) = self.routes.trees.remove(&net) {
            let _ = self.grid.lift(placed);
        }
        if let Some(tree) = lifted {
            let _ = self.routes.trees.insert(net, self.grid.place(tree));
        }
        match terminals {
            Some(terminals) => {
                let _ = self.nets.insert(net, terminals);
            }
            None => {
                let _ = self.nets.remove(&net);
            }
        }
    }

    /// Routes `dirty` against everything else on the grid and keeps their
    /// routes with their pins' terminals.
    fn route_dirty(&mut self, dirty: Vec<(GlobalNet, Vec<PinTerminal>)>) {
        if dirty.is_empty() {
            return;
        }
        let global_nets: Vec<GlobalNet> = dirty.iter().map(|(n, _)| n.clone()).collect();
        let mut routed = reroute(&mut self.grid, &global_nets, self.rounds);
        for (global_net, terminals) in dirty {
            let net = global_net.net;
            if let Some(placed) = routed.trees.remove(&net) {
                let _ = self.routes.trees.insert(net, placed);
                let _ = self.nets.insert(net, terminals);
            }
        }
    }
}

/// The multiset of where the pins meet the grid, as `OpenROAD` compares
/// nets' pin positions on the grid (`GlobalRouter::pinPositionsChanged`).
fn grid_positions(terminals: &[PinTerminal]) -> Vec<(GCell, LayerId)> {
    let mut positions: Vec<(GCell, LayerId)> =
        terminals.iter().map(|t| (t.cell, t.pin_layer)).collect();
    positions.sort_unstable_by_key(|(c, l)| (c.layer, c.x, c.y, *l));
    positions
}

impl RouteSource for GlobalSession {
    fn update(&mut self, db: &NetlistDB, nets: &[NetId]) {
        let mut seen = FxHashSet::default();
        let mut dirty = Vec::new();
        for &net in nets {
            if !seen.insert(net) {
                continue;
            }
            let fresh = self.terminals(db, net);
            if let (Some(old), Some(new)) = (self.nets.get(&net), &fresh)
                && grid_positions(old) == grid_positions(new)
            {
                if old != new {
                    self.remember(net);
                    let _ = self.nets.insert(net, new.clone());
                }
                continue;
            }
            self.rip_up(net);
            if let Some(terminals) = fresh {
                let cells = terminals.iter().map(|t| t.cell).collect();
                let global_net = GlobalNet::of(db, &self.tech, net, cells);
                dirty.push((global_net, terminals));
            }
        }
        self.route_dirty(dirty);
    }

    fn begin_trial(&mut self) {
        debug_assert!(self.trial.is_none(), "trials do not nest");
        self.trial = Some(FxHashMap::default());
    }

    fn keep_trial(&mut self) {
        self.trial = None;
    }

    fn restore_trial(&mut self) {
        let Some(trial) = self.trial.take() else {
            return;
        };
        let mut changed: Vec<(NetId, Before)> = trial.into_iter().collect();
        changed.sort_unstable_by_key(|(net, _)| *net);
        for (net, before) in changed {
            self.put_back(net, before);
        }
    }

    fn route(&self, db: &NetlistDB, net: NetId) -> Option<NetRoute> {
        let terminals = self.nets.get(&net)?;
        let tree = self.routes.trees.get(&net)?.paths();
        Some(NetRoute {
            segments: route_segments(&|c| self.grid.cell_center(c), tree, terminals),
            pins: terminals
                .iter()
                .map(|t| PinGridLocation {
                    pin: t.pin,
                    at: route_pin_position(db, t.pin),
                    grid: self.grid.cell_center(t.cell),
                    layer: t.pin_layer,
                })
                .collect(),
        })
    }
}

/// Where `OpenROAD`'s global router takes a pin to be: the lower left of
/// the last shape of the pin on a routing layer, as
/// `GlobalRouter::makeItermPins` sets it.
fn route_pin_position(db: &NetlistDB, pin: PinId) -> Point<Dbu> {
    db.pin_shapes(pin)
        .iter()
        .rev()
        .find(|s| matches!(s.layer, LayerRef::Routing(_)))
        .map_or_else(|| db.pin_position(pin), |s| s.rect.min)
}

/// The segments of a net's tree as `OpenROAD`'s global router gives them:
/// a via between each pair of layers a column of the tree climbs, the
/// vias from each pin's own layer up to where the tree starts, and a wire
/// along each straight run of the tree between the points where it
/// branches, turns, climbs or meets a pin, as `GlobalRouter::mergeSegments`
/// leaves them.
fn route_segments(
    center: &dyn Fn(GCell) -> Point<Dbu>,
    tree: &[Vec<GCell>],
    terminals: &[PinTerminal],
) -> Vec<RouteSegment> {
    let mut links: Vec<(GCell, GCell)> = Vec::new();
    let mut seen_links = FxHashSet::default();
    let mut add_link = |a: GCell, b: GCell, links: &mut Vec<(GCell, GCell)>| {
        let key = if (a.layer, a.x, a.y) <= (b.layer, b.x, b.y) {
            (a, b)
        } else {
            (b, a)
        };
        if a != b && seen_links.insert(key) {
            links.push(key);
        }
    };
    for path in tree {
        for w in path.windows(2) {
            add_link(w[0], w[1], &mut links);
        }
    }
    for t in terminals {
        for layer in t.pin_layer.index()..t.cell.layer.index() {
            let at = |l: usize| GCell {
                layer: LayerId::new(l),
                ..t.cell
            };
            add_link(at(layer), at(layer + 1), &mut links);
        }
    }
    let point = |c: GCell| RoutePoint {
        at: center(c),
        layer: c.layer,
    };
    let mut segments: Vec<RouteSegment> = links
        .iter()
        .filter(|(a, b)| a.layer != b.layer)
        .map(|&(a, b)| RouteSegment {
            from: point(a),
            to: point(b),
        })
        .collect();
    segments.extend(
        straight_runs(&links, terminals)
            .into_iter()
            .map(|(a, b)| RouteSegment {
                from: point(a),
                to: point(b),
            }),
    );
    segments
}

/// The straight runs the planar links make, each between two cells where
/// the tree branches, turns, ends, climbs or meets a pin.
fn straight_runs(links: &[(GCell, GCell)], terminals: &[PinTerminal]) -> Vec<(GCell, GCell)> {
    let mut planar: FxHashMap<GCell, Vec<GCell>> = FxHashMap::default();
    let mut climbs: FxHashSet<GCell> = FxHashSet::default();
    for &(a, b) in links {
        if a.layer == b.layer {
            planar.entry(a).or_default().push(b);
            planar.entry(b).or_default().push(a);
        } else {
            let _ = climbs.insert(a);
            let _ = climbs.insert(b);
        }
    }
    let mut attached: FxHashSet<GCell> = FxHashSet::default();
    for t in terminals {
        let _ = attached.insert(t.cell);
        let _ = attached.insert(GCell {
            layer: LayerId::new(t.pin_layer.index() + 1),
            ..t.cell
        });
    }
    let is_break = |c: GCell| {
        if climbs.contains(&c) || attached.contains(&c) {
            return true;
        }
        match planar.get(&c).map(Vec::as_slice) {
            Some([p, q]) => {
                let straight = i32::from(p.x) + i32::from(q.x) == 2 * i32::from(c.x)
                    && i32::from(p.y) + i32::from(q.y) == 2 * i32::from(c.y);
                !straight
            }
            _ => true,
        }
    };
    let mut runs = Vec::new();
    let mut walked: FxHashSet<(GCell, GCell)> = FxHashSet::default();
    let mut starts: Vec<GCell> = planar.keys().copied().filter(|&c| is_break(c)).collect();
    starts.sort_unstable_by_key(|c| (c.layer, c.x, c.y));
    for start in starts {
        for &first in &planar[&start] {
            if walked.contains(&(start, first)) {
                continue;
            }
            let (mut previous, mut current) = (start, first);
            let _ = walked.insert((previous, current));
            let _ = walked.insert((current, previous));
            while !is_break(current) {
                let next = planar[&current]
                    .iter()
                    .copied()
                    .find(|&n| n != previous)
                    .unwrap_or(previous);
                previous = current;
                current = next;
                let _ = walked.insert((previous, current));
                let _ = walked.insert((current, previous));
            }
            runs.push((start, current));
        }
    }
    runs
}

#[cfg(test)]
mod tests;
