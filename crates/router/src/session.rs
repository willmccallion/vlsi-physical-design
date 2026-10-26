//! Global routes kept between global and detailed routing, as `OpenROAD`
//! keeps its global router's routes while repair edits the design.
//!
//! A net is routed again once an edit moves one of its pins to another
//! gcell, and its route is handed out as segments between gcell centers
//! with where each pin meets them.

use crate::access::access_points;
use crate::detailed::{PinJudge, PinTerminal};
use crate::global::{GCell, GlobalGrid, GlobalNet, GlobalRoutes, reroute, track_demand};
use crate::tech::RouteTech;
use crate::{GlobalStage, RouteError, check_routable, gcell_size, global_stage};
use pare_common::db::design::{NetlistDB, PinOwner, Renumbering};
use pare_common::db::global_route::{
    NetRoute, PinGridLocation, RoutePoint, RouteSegment, RouteSource,
};
use pare_common::db::indices::{LayerId, MacroId, NetId, PinId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::util::config::RoutingConfig;
use pare_common::verify::shapes::is_signal_net;
use rustc_hash::{FxHashMap, FxHashSet};

/// A design's global routes, kept in step with its netlist.
#[derive(Debug)]
pub struct GlobalSession {
    tech: RouteTech,
    grid: GlobalGrid,
    routes: GlobalRoutes,
    /// Each routed net as it was routed, with its pins' terminals.
    nets: FxHashMap<NetId, (GlobalNet, Vec<PinTerminal>)>,
    judge: PinJudge,
    /// Each pin's terminal as global routing found it.
    initial: FxHashMap<PinId, PinTerminal>,
    /// Each cell's master, location and orientation as global routing
    /// found them; a pin of a cell that keeps them keeps its terminal.
    placed: Vec<(MacroId, Point<Dbu>, Orientation)>,
    rounds: usize,
}

impl GlobalSession {
    /// Finds pin access and routes every signal net of `db` globally.
    ///
    /// # Errors
    ///
    /// When `db` cannot be routed under `config`.
    pub fn new(db: &NetlistDB, config: &RoutingConfig) -> Result<Self, RouteError> {
        check_routable(db, config)?;
        let tech = RouteTech::new(db, config)?;
        let gcell = gcell_size(&tech, config);
        let (grid, routes, nets, judge) = {
            let GlobalStage {
                dr, grid, global, ..
            } = global_stage(db, &tech, config, gcell, None)?;
            let mut nets = FxHashMap::default();
            for &net in global.paths.keys() {
                if let Some(terminals) = dr.pin_terminals(net, &grid) {
                    let global_net = GlobalNet {
                        net,
                        terminals: terminals.iter().map(|t| t.cell).collect(),
                        demand: track_demand(db, &tech, net),
                    };
                    let _ = nets.insert(net, (global_net, terminals));
                }
            }
            (grid, global, nets, dr.into_pin_judge())
        };
        let initial = nets
            .values()
            .flat_map(|(_, terminals)| terminals.iter().map(|t| (t.pin, *t)))
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
        })
    }

    /// The technology, grid, routes and routed nets, for detailed routing.
    pub(crate) fn into_parts(
        self,
    ) -> (
        RouteTech,
        GlobalGrid,
        GlobalRoutes,
        FxHashMap<NetId, GlobalNet>,
    ) {
        let nets = self
            .nets
            .into_iter()
            .map(|(net, (global_net, _))| (net, global_net))
            .collect();
        (self.tech, self.grid, self.routes, nets)
    }

    /// Follows the design's renumbering after cells and nets were removed:
    /// the routes of nets it dropped are lifted, the rest keep theirs.
    pub fn renumber(&mut self, renumbering: &Renumbering) {
        let pin = |p: PinId| renumbering.pins.get(p.index()).copied().flatten();
        let mut nets = FxHashMap::default();
        let mut paths = FxHashMap::default();
        for (net, (mut global_net, terminals)) in std::mem::take(&mut self.nets) {
            let tree = self.routes.paths.remove(&net);
            let Some(new) = renumbering.nets.get(net.index()).copied().flatten() else {
                if let Some(tree) = tree {
                    self.grid.lift(&global_net, &tree);
                }
                continue;
            };
            global_net.net = new;
            let terminals = terminals
                .into_iter()
                .filter_map(|t| {
                    Some(PinTerminal {
                        pin: pin(t.pin)?,
                        ..t
                    })
                })
                .collect();
            if let Some(tree) = tree {
                let _ = paths.insert(new, tree);
            }
            let _ = nets.insert(new, (global_net, terminals));
        }
        self.nets = nets;
        self.routes.paths = paths;
        self.initial = std::mem::take(&mut self.initial)
            .into_iter()
            .filter_map(|(p, t)| {
                let new = pin(p)?;
                Some((new, PinTerminal { pin: new, ..t }))
            })
            .collect();
        let mut placed = Vec::with_capacity(self.placed.len());
        for (old, &at) in self.placed.iter().enumerate() {
            if renumbering.cells.get(old).copied().flatten().is_some() {
                placed.push(at);
            }
        }
        self.placed = placed;
    }

    /// The terminal of `pin` on `net`: as global routing found it while
    /// its cell stays as it was, else from the pin's best access point now.
    fn terminal(&self, db: &NetlistDB, net: NetId, pin: PinId) -> Option<PinTerminal> {
        if self.unchanged(db, pin)
            && let Some(&terminal) = self.initial.get(&pin)
        {
            return Some(terminal);
        }
        let clean = |layer: LayerId, metal: &_| self.judge.clean(net, layer, metal);
        let candidates = access_points(db, &self.tech, pin, &clean);
        let best = candidates.first()?;
        Some(PinTerminal::of(pin, best, &self.tech, &self.grid))
    }

    /// Whether `pin` belongs to a port or to a cell with the master,
    /// location and orientation it had when routing began.
    fn unchanged(&self, db: &NetlistDB, pin: PinId) -> bool {
        match db.pins[pin.index()].owner {
            PinOwner::Io(_) => true,
            PinOwner::Cell { cell, .. } => {
                let c = &db.cells[cell.index()];
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
        db.nets[net.index()]
            .pins
            .iter()
            .map(|&pin| self.terminal(db, net, pin))
            .collect()
    }

    /// Takes `net`'s route off the grid and forgets it.
    fn rip_up(&mut self, net: NetId) {
        let Some((global_net, _)) = self.nets.remove(&net) else {
            return;
        };
        if let Some(tree) = self.routes.paths.remove(&net) {
            self.grid.lift(&global_net, &tree);
        }
    }

    /// Routes `dirty` against everything else on the grid and keeps their
    /// routes with their pins' terminals.
    fn route_dirty(&mut self, dirty: Vec<(GlobalNet, Vec<PinTerminal>)>) {
        if dirty.is_empty() {
            return;
        }
        let global_nets: Vec<GlobalNet> = dirty.iter().map(|(n, _)| n.clone()).collect();
        let routed = reroute(&mut self.grid, &global_nets, self.rounds);
        for (global_net, terminals) in dirty {
            let net = global_net.net;
            if let Some(tree) = routed.paths.get(&net) {
                let _ = self.routes.paths.insert(net, tree.clone());
                let _ = self.nets.insert(net, (global_net, terminals));
            }
        }
    }
}

/// The multiset of where the pins meet the grid, as `OpenROAD` compares
/// nets' pin positions on the grid.
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
            if let (Some((_, old)), Some(new)) = (self.nets.get_mut(&net), &fresh)
                && grid_positions(old) == grid_positions(new)
            {
                old.clone_from(new);
                continue;
            }
            self.rip_up(net);
            if let Some(terminals) = fresh {
                let global_net = GlobalNet {
                    net,
                    terminals: terminals.iter().map(|t| t.cell).collect(),
                    demand: track_demand(db, &self.tech, net),
                };
                dirty.push((global_net, terminals));
            }
        }
        self.route_dirty(dirty);
    }

    fn route(&self, db: &NetlistDB, net: NetId) -> Option<NetRoute> {
        let (_, terminals) = self.nets.get(&net)?;
        let tree = self.routes.paths.get(&net)?;
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
/// the last shape of the pin on a routing layer.
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
/// branches, turns, climbs or meets a pin.
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
mod tests {
    use super::*;

    const fn cell(layer: u8, x: u16, y: u16) -> GCell {
        GCell {
            layer: LayerId(layer),
            x,
            y,
        }
    }

    fn center(c: GCell) -> Point<Dbu> {
        Point::new(Dbu(i64::from(c.x) * 10 + 5), Dbu(i64::from(c.y) * 10 + 5))
    }

    fn terminal(pin: usize, cell: GCell, pin_layer: u8) -> PinTerminal {
        PinTerminal {
            pin: PinId::new(pin),
            cell,
            pin_layer: LayerId(pin_layer),
        }
    }

    /// A gcell's column, row and layer.
    type At = (i64, i64, u8);

    /// Each segment as its ends' gcell coordinates and layers, sorted.
    fn ends(segments: &[RouteSegment]) -> Vec<(At, At)> {
        let at = |p: &RoutePoint| {
            (
                (p.at.x.get() - 5) / 10,
                (p.at.y.get() - 5) / 10,
                u8::try_from(p.layer.index()).unwrap(),
            )
        };
        let mut out: Vec<_> = segments
            .iter()
            .map(|s| {
                let (a, b) = (at(&s.from), at(&s.to));
                if a <= b { (a, b) } else { (b, a) }
            })
            .collect();
        out.sort_unstable();
        out
    }

    #[test]
    fn a_straight_run_is_one_wire_and_pins_climb_from_their_own_layer() {
        let tree = vec![vec![
            cell(1, 0, 0),
            cell(1, 1, 0),
            cell(1, 2, 0),
            cell(1, 3, 0),
        ]];
        let terminals = [terminal(0, cell(1, 0, 0), 0), terminal(1, cell(1, 3, 0), 0)];

        let segments = route_segments(&center, &tree, &terminals);

        assert_eq!(
            ends(&segments),
            vec![
                ((0, 0, 0), (0, 0, 1)),
                ((0, 0, 1), (3, 0, 1)),
                ((3, 0, 0), (3, 0, 1)),
            ]
        );
    }

    #[test]
    fn a_run_breaks_where_a_branch_joins_it_turns_or_climbs() {
        let tree = vec![
            vec![
                cell(2, 0, 1),
                cell(2, 1, 1),
                cell(2, 2, 1),
                cell(2, 3, 1),
                cell(1, 3, 1),
                cell(1, 3, 0),
            ],
            vec![cell(2, 1, 1), cell(1, 1, 1), cell(1, 1, 2), cell(1, 1, 3)],
        ];
        let terminals = [
            terminal(0, cell(2, 0, 1), 2),
            terminal(1, cell(1, 3, 0), 1),
            terminal(2, cell(1, 1, 3), 1),
        ];

        let segments = route_segments(&center, &tree, &terminals);

        assert_eq!(
            ends(&segments),
            vec![
                ((0, 1, 2), (1, 1, 2)),
                ((1, 1, 1), (1, 1, 2)),
                ((1, 1, 1), (1, 3, 1)),
                ((1, 1, 2), (3, 1, 2)),
                ((3, 0, 1), (3, 1, 1)),
                ((3, 1, 1), (3, 1, 2)),
            ]
        );
    }
}
