//! A net as a tree of wires, junctions, loads and the buffers that could
//! drive them, as `OpenROAD`'s `BufferedNet`: the shape repair reasons
//! about when it splits a net or chooses where buffers go.
//!
//! Nodes are shared, so the many candidate trees the rebuffering search
//! keeps alive can point at the same subtrees. Times inside a tree are
//! held in whole femtoseconds, as `OpenROAD`'s `FixedDelay`, so that two
//! options differing only by rounding compare equal.

use crate::netlist::Netlist;
use crate::parasitics::{NetParasitics, WireRc, WireShape};
use crate::route_parasitics::RouteRc;
use crate::steiner::{self, SteinerTree};
use pare_common::db::global_route::{NetRoute, RoutePoint};
use pare_common::db::indices::{LayerId, PinId};
use pare_common::db::liberty::{Edge, LibCellId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use rustc_hash::FxHashMap;
use std::collections::HashSet;
use std::rc::Rc;

/// A time in whole femtoseconds, as `OpenROAD`'s `FixedDelay`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Fixed(i64);

impl Fixed {
    /// Zero.
    pub const ZERO: Self = Self(0);
    /// A time beyond any path.
    pub const INF: Self = Self(100_000_000_000_000_000);

    const FEMTOSECONDS_PER_SECOND: f64 = 1e15;

    /// `seconds` rounded toward zero to femtoseconds, saturating at the
    /// infinities.
    #[must_use]
    pub fn from_seconds(seconds: f64) -> Self {
        if seconds >= Self::INF.to_seconds() {
            return Self::INF;
        }
        if seconds <= -Self::INF.to_seconds() {
            return -Self::INF;
        }
        Self((seconds * Self::FEMTOSECONDS_PER_SECOND) as i64)
    }

    /// The time in seconds.
    #[must_use]
    pub fn to_seconds(self) -> f64 {
        self.0 as f64 / Self::FEMTOSECONDS_PER_SECOND
    }

    /// `a + t (b - a)`.
    #[must_use]
    pub fn lerp(a: Self, b: Self, t: f64) -> Self {
        if t >= 1.0 {
            return b;
        }
        Self(a.0 + ((b.0 - a.0) as f64 * t) as i64)
    }
}

impl std::ops::Add for Fixed {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0.saturating_add(rhs.0))
    }
}

impl std::ops::Sub for Fixed {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0.saturating_sub(rhs.0))
    }
}

impl std::ops::Neg for Fixed {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

/// Which edges a subtree's slack was taken over, as `OpenROAD`'s
/// `slackTransition`: none until a load has been timed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlackEdges {
    /// One edge.
    One(Edge),
    /// Both.
    Both,
}

impl SlackEdges {
    /// The edges to time a buffer for.
    #[must_use]
    pub const fn edges(self) -> &'static [Edge] {
        match self {
            Self::One(Edge::Rise) => &[Edge::Rise],
            Self::One(Edge::Fall) => &[Edge::Fall],
            Self::Both => &Edge::BOTH,
        }
    }

    /// The edges either of two subtrees was timed over.
    #[must_use]
    pub fn combine(a: Option<Self>, b: Option<Self>) -> Option<Self> {
        match (a, b) {
            (None, b) => b,
            (a, None) => a,
            (Some(a), Some(b)) if a == b => Some(a),
            _ => Some(Self::Both),
        }
    }
}

/// What a node is.
#[derive(Clone, Debug)]
pub enum Kind {
    /// A load pin.
    Load {
        /// The pin.
        pin: PinId,
    },
    /// Two subtrees meeting.
    Junction {
        /// One side.
        left: BnetPtr,
        /// The other.
        right: BnetPtr,
    },
    /// A wire from this node's point to the subtree's.
    Wire {
        /// What the wire reaches.
        to: BnetPtr,
        /// The routing layer it runs on, or none for a wire at the signal
        /// wire RC.
        layer: Option<LayerId>,
    },
    /// A via at this node's point between two routing layers, the
    /// subtree starting on the second.
    Via {
        /// What the via reaches.
        to: BnetPtr,
        /// The layer above the node.
        layer: LayerId,
        /// The layer the subtree starts on.
        to_layer: LayerId,
    },
    /// A buffer driving the subtree.
    Buffer {
        /// The buffer's cell.
        cell: LibCellId,
        /// What it drives.
        to: BnetPtr,
    },
}

/// A shared node.
pub type BnetPtr = Rc<Bnet>;

/// One node of a buffered net with what the search knows about the
/// subtree under it.
#[derive(Clone, Debug)]
pub struct Bnet {
    /// What the node is.
    pub kind: Kind,
    /// Where it sits.
    pub at: Point<Dbu>,
    /// The capacitance the subtree presents, in farads.
    pub cap: f64,
    /// The fanout the subtree presents.
    pub fanout: f64,
    /// The tightest slew limit among the subtree's loads, in seconds.
    pub max_load_slew: f64,
    /// The area of the buffers in the subtree.
    pub area: f64,
    /// The slack at the node's point for the subtree's loads.
    pub slack: Fixed,
    /// Which edges the slack covers.
    pub slack_edges: Option<SlackEdges>,
    /// The delay from this node to the subtree's top.
    pub delay: Fixed,
    /// The arrival at this node, set while recovering area.
    pub arrival_delay: Fixed,
}

impl Bnet {
    /// A load node.
    #[must_use]
    pub fn load(at: Point<Dbu>, pin: PinId, cap: f64, fanout: f64, max_load_slew: f64) -> BnetPtr {
        Rc::new(Self {
            kind: Kind::Load { pin },
            at,
            cap,
            fanout,
            max_load_slew,
            area: 0.0,
            slack: Fixed::ZERO,
            slack_edges: None,
            delay: Fixed::ZERO,
            arrival_delay: Fixed::ZERO,
        })
    }

    /// Two subtrees joined at `at`, with the slack the tighter of theirs.
    #[must_use]
    pub fn junction(at: Point<Dbu>, left: BnetPtr, right: BnetPtr) -> BnetPtr {
        Rc::new(Self {
            cap: left.cap + right.cap,
            fanout: left.fanout + right.fanout,
            max_load_slew: left.max_load_slew.min(right.max_load_slew),
            area: left.area + right.area,
            slack: left.slack.min(right.slack),
            slack_edges: SlackEdges::combine(left.slack_edges, right.slack_edges),
            delay: Fixed::ZERO,
            arrival_delay: Fixed::ZERO,
            at,
            kind: Kind::Junction { left, right },
        })
    }

    /// A wire from `at` to the subtree on `layer`, its Elmore delay
    /// through the wire's own capacitance taken off the subtree's slack.
    #[must_use]
    pub fn wire(at: Point<Dbu>, to: BnetPtr, layer: Option<LayerId>, rc: &TreeRc<'_>) -> BnetPtr {
        let length = manhattan(at, to.at);
        let meters = length as f64 / rc.dbu * 1e-6;
        let (res_per_meter, cap_per_meter) = rc.wire(layer);
        let resistance = meters * res_per_meter;
        let capacitance = meters * cap_per_meter;
        let delay = if length == 0 {
            Fixed::ZERO
        } else {
            Fixed::from_seconds(resistance * (capacitance / 2.0 + to.cap))
        };
        Rc::new(Self {
            cap: to.cap + capacitance,
            fanout: to.fanout,
            max_load_slew: to.max_load_slew,
            area: to.area,
            slack: to.slack - delay,
            slack_edges: to.slack_edges,
            delay,
            arrival_delay: Fixed::ZERO,
            at,
            kind: Kind::Wire { to, layer },
        })
    }

    /// A via at `at` from `layer` to the subtree on `to_layer`, its delay
    /// through the cuts into the subtree's capacitance taken off the
    /// subtree's slack.
    #[must_use]
    pub fn via(
        at: Point<Dbu>,
        layer: LayerId,
        to_layer: LayerId,
        to: BnetPtr,
        rc: &TreeRc<'_>,
    ) -> BnetPtr {
        let delay = Fixed::from_seconds(rc.via(layer, to_layer) * to.cap);
        Rc::new(Self {
            cap: to.cap,
            fanout: to.fanout,
            max_load_slew: to.max_load_slew,
            area: to.area,
            slack: to.slack - delay,
            slack_edges: to.slack_edges,
            delay,
            arrival_delay: Fixed::ZERO,
            at,
            kind: Kind::Via {
                to,
                layer,
                to_layer,
            },
        })
    }

    /// A buffer at `at` driving the subtree, presenting its own input.
    #[must_use]
    pub fn buffer(at: Point<Dbu>, size: &BufferSize, to: BnetPtr, delay: Fixed) -> BnetPtr {
        Rc::new(Self {
            cap: size.input_cap,
            fanout: size.input_fanout,
            max_load_slew: size.input_max_slew,
            area: to.area + size.area,
            slack: to.slack - delay,
            slack_edges: to.slack_edges,
            delay,
            arrival_delay: Fixed::ZERO,
            at,
            kind: Kind::Buffer {
                cell: size.cell,
                to,
            },
        })
    }

    /// The same node with another slack and edges.
    #[must_use]
    pub fn with_slack(&self, slack: Fixed, edges: Option<SlackEdges>) -> BnetPtr {
        Rc::new(Self {
            slack,
            slack_edges: edges,
            kind: self.kind.clone(),
            ..*self
        })
    }

    /// The same node with another arrival.
    #[must_use]
    pub fn with_arrival(&self, arrival_delay: Fixed) -> BnetPtr {
        Rc::new(Self {
            arrival_delay,
            kind: self.kind.clone(),
            ..*self
        })
    }

    /// The subtree under a wire, via or buffer, or none for a load or
    /// junction.
    #[must_use]
    pub const fn inner(&self) -> Option<&BnetPtr> {
        match &self.kind {
            Kind::Wire { to, .. } | Kind::Via { to, .. } | Kind::Buffer { to, .. } => Some(to),
            Kind::Load { .. } | Kind::Junction { .. } => None,
        }
    }

    /// The routing layer of a wire, or the layer above a via; none for
    /// other nodes and wires at the signal wire RC.
    #[must_use]
    pub const fn layer(&self) -> Option<LayerId> {
        match &self.kind {
            Kind::Wire { layer, .. } => *layer,
            Kind::Via { layer, .. } => Some(*layer),
            _ => None,
        }
    }

    /// The length of the wire at this node, if it is one.
    #[must_use]
    pub fn length(&self) -> i64 {
        match &self.kind {
            Kind::Wire { to, .. } => manhattan(self.at, to.at),
            _ => 0,
        }
    }

    /// The longest wire from this node down to a load or buffer, as
    /// `OpenROAD`'s `maxLoadWireLength`.
    #[must_use]
    pub fn max_load_wire_length(&self) -> i64 {
        match &self.kind {
            Kind::Wire { to, .. } => self.length() + to.max_load_wire_length(),
            Kind::Via { to, .. } => to.max_load_wire_length(),
            Kind::Junction { left, right } => left
                .max_load_wire_length()
                .max(right.max_load_wire_length()),
            Kind::Load { .. } | Kind::Buffer { .. } => 0,
        }
    }

    /// How many buffers the subtree holds.
    #[must_use]
    pub fn buffer_count(&self) -> usize {
        match &self.kind {
            Kind::Buffer { to, .. } => to.buffer_count() + 1,
            Kind::Wire { to, .. } | Kind::Via { to, .. } => to.buffer_count(),
            Kind::Junction { left, right } => left.buffer_count() + right.buffer_count(),
            Kind::Load { .. } => 0,
        }
    }

    /// The load pins under the subtree.
    #[must_use]
    pub fn loads(&self) -> Vec<PinId> {
        let mut loads = Vec::new();
        self.collect_loads(&mut loads);
        loads
    }

    fn collect_loads(&self, into: &mut Vec<PinId>) {
        match &self.kind {
            Kind::Load { pin } => into.push(*pin),
            Kind::Wire { to, .. } | Kind::Via { to, .. } | Kind::Buffer { to, .. } => {
                to.collect_loads(into);
            }
            Kind::Junction { left, right } => {
                left.collect_loads(into);
                right.collect_loads(into);
            }
        }
    }

    /// The measures a replacement subtree must match or beat, as
    /// `OpenROAD`'s `Metrics`.
    #[must_use]
    pub fn metrics(&self) -> Metrics {
        Metrics {
            max_load_wire_length: self.max_load_wire_length(),
            slack: self.slack,
            cap: self.cap,
            max_load_slew: self.max_load_slew,
            fanout: self.fanout,
        }
    }

    /// Whether the subtree is no worse than `target` in every measure.
    #[must_use]
    pub fn fits(&self, target: &Metrics) -> bool {
        self.max_load_wire_length() <= target.max_load_wire_length
            && self.slack >= target.slack
            && crate::fuzzy::less_equal(self.cap, target.cap)
            && crate::fuzzy::greater_equal(self.max_load_slew, target.max_load_slew)
            && crate::fuzzy::less_equal(self.fanout, target.fanout)
    }

    /// The node under any wires and vias, as `stripWireOnBnet`.
    #[must_use]
    pub fn under_wires(self: &BnetPtr) -> BnetPtr {
        let mut node = Rc::clone(self);
        while let Kind::Wire { to, .. } | Kind::Via { to, .. } = &node.kind {
            node = Rc::clone(to);
        }
        node
    }

    /// The node under any wires, vias and buffers, as
    /// `stripWiresAndBuffersOnBnet`.
    #[must_use]
    pub fn under_wires_and_buffers(self: &BnetPtr) -> BnetPtr {
        let mut node = Rc::clone(self);
        loop {
            match &node.kind {
                Kind::Wire { to, .. } | Kind::Via { to, .. } | Kind::Buffer { to, .. } => {
                    node = Rc::clone(to);
                }
                _ => return node,
            }
        }
    }

    /// The layer of the first wire below this node through vias and
    /// buffers, as the rebuffer's `findWireLayer`: none when a load or
    /// junction comes first or the wire is at the signal wire RC.
    #[must_use]
    pub fn wire_layer_below(&self) -> Option<LayerId> {
        let mut node = self;
        loop {
            match &node.kind {
                Kind::Wire { layer, .. } => return *layer,
                Kind::Via { to, .. } | Kind::Buffer { to, .. } => node = to,
                Kind::Load { .. } | Kind::Junction { .. } => return None,
            }
        }
    }
}

/// A buffer the search may insert, with what its input presents and what
/// its output can drive, as `OpenROAD`'s `BufferSize`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BufferSize {
    /// The cell.
    pub cell: LibCellId,
    /// Its input capacitance, in farads.
    pub input_cap: f64,
    /// Its input's fanout load.
    pub input_fanout: f64,
    /// Its input's slew limit, in seconds.
    pub input_max_slew: f64,
    /// Its output's capacitance, in farads.
    pub output_cap: f64,
    /// Its area.
    pub area: f64,
    /// Its delay with no load and no input slew.
    pub intrinsic_delay: Fixed,
    /// The largest load it may drive within its limits.
    pub margined_max_cap: f64,
    /// Its drive resistance, in ohms.
    pub drive_resistance: f64,
}

/// The measures of a subtree.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Metrics {
    /// See [`Bnet::max_load_wire_length`].
    pub max_load_wire_length: i64,
    /// See [`Bnet::slack`].
    pub slack: Fixed,
    /// See [`Bnet::cap`].
    pub cap: f64,
    /// See [`Bnet::max_load_slew`].
    pub max_load_slew: f64,
    /// See [`Bnet::fanout`].
    pub fanout: f64,
}

impl Metrics {
    /// The same measures with another slack.
    #[must_use]
    pub const fn with_slack(self, slack: Fixed) -> Self {
        Self { slack, ..self }
    }
}

/// What a load pin presents, as the tree needs it.
pub trait LoadModel {
    /// Capacitance, fanout load and slew limit of a load pin.
    fn load(&self, pin: PinId) -> (f64, f64, f64);
}

/// Manhattan distance in database units.
#[must_use]
pub const fn manhattan(a: Point<Dbu>, b: Point<Dbu>) -> i64 {
    (a.x.get() - b.x.get()).abs() + (a.y.get() - b.y.get()).abs()
}

/// The resistance and capacitance the wires and vias of a tree take, as
/// `BufferedNet::wireRC` and `BufferedNet::viaResistance` find them.
#[derive(Clone, Copy, Debug)]
pub struct TreeRc<'a> {
    /// The signal wire RC, for wires without a layer.
    pub signal: WireRc,
    /// Each layer's, for wires and vias along global routes.
    pub layers: Option<&'a RouteRc>,
    /// Database units per micron.
    pub dbu: f64,
}

impl TreeRc<'_> {
    /// Ohms and farads per meter of wire on `layer`, or at the signal wire
    /// RC without one; zero for a layer `set_layer_rc` left out.
    #[must_use]
    pub fn wire(&self, layer: Option<LayerId>) -> (f64, f64) {
        match layer {
            None => (self.signal.resistance, self.signal.capacitance),
            Some(layer) => self.layers.map_or((0.0, 0.0), |l| l.layer_rc(layer)),
        }
    }

    /// The resistance of the cuts from routing layer `a` to `b`.
    #[must_use]
    pub fn via(&self, a: LayerId, b: LayerId) -> f64 {
        let Some(layers) = self.layers else {
            return 0.0;
        };
        (a.index().min(b.index())..a.index().max(b.index()))
            .map(|cut| layers.via_resistance(LayerId::new(cut)))
            .sum()
    }
}

/// The buffered net of `driver`'s net over its parasitics, or none for a
/// net with no loads.
///
/// It follows the net's Steiner tree as `Resizer::makeBufferedNetSteiner`
/// does, or its global route as `Resizer::makeBufferedNetGroute` does.
#[must_use]
pub fn from_parasitics(
    netlist: &Netlist<'_>,
    parasitics: &NetParasitics,
    driver: PinId,
    rc: &TreeRc<'_>,
    loads: &dyn LoadModel,
) -> Option<BnetPtr> {
    match &parasitics.shape {
        WireShape::Steiner {
            tree,
            terminal_pins,
        } => from_steiner(netlist, tree, terminal_pins, driver, rc, loads),
        WireShape::Route(route) => from_route(netlist, route, driver, rc, loads),
    }
}

fn from_steiner(
    netlist: &Netlist<'_>,
    tree: &SteinerTree,
    terminal_pins: &[Vec<PinId>],
    driver: PinId,
    rc: &TreeRc<'_>,
    loads: &dyn LoadModel,
) -> Option<BnetPtr> {
    let children = tree.children();
    let mut visited: HashSet<(i64, i64)> = HashSet::new();
    let mut leaves_at = |node: usize| -> Vec<BnetPtr> {
        if node >= tree.terminals {
            return Vec::new();
        }
        let at = tree.nodes[node];
        if !visited.insert((at.x.get(), at.y.get())) {
            return Vec::new();
        }
        terminal_pins[node]
            .iter()
            .copied()
            .filter(|&pin| pin != driver && netlist.is_load(pin))
            .map(|pin| {
                let (cap, fanout, max_slew) = loads.load(pin);
                Bnet::load(at, pin, cap, fanout, max_slew)
            })
            .collect()
    };
    build(tree, &children, None, tree.root, &mut leaves_at, rc)
}

/// The buffered net along a global route from the driver's grid point,
/// as `makeBufferedNetGroute`: loads at their pins' grid points, wires
/// between points on one layer and vias between layers.
fn from_route(
    netlist: &Netlist<'_>,
    route: &NetRoute,
    driver: PinId,
    rc: &TreeRc<'_>,
    loads: &dyn LoadModel,
) -> Option<BnetPtr> {
    let mut pins_at: FxHashMap<RoutePoint, Vec<PinId>> = FxHashMap::default();
    let mut root = None;
    for location in &route.pins {
        let at = RoutePoint {
            at: location.grid,
            layer: location.layer,
        };
        pins_at.entry(at).or_default().push(location.pin);
        if location.pin == driver {
            root = Some(at);
        }
    }
    let Some(root) = root else {
        log::warn!(
            "driver pin {} not found in global routes",
            netlist.db.pin_name(driver)
        );
        return None;
    };
    let mut adjacent: FxHashMap<RoutePoint, Vec<RoutePoint>> = FxHashMap::default();
    for segment in &route.segments {
        adjacent.entry(segment.from).or_default().push(segment.to);
        adjacent.entry(segment.to).or_default().push(segment.from);
    }
    let mut walk = RouteWalk {
        netlist,
        pins_at: &pins_at,
        adjacent: &adjacent,
        rc,
        loads,
        visited: HashSet::new(),
    };
    let tree = walk.build(None, root)?;
    let expected = route.pins.len() - 1;
    if tree.loads().len() != expected {
        log::error!(
            "Failed to build tree from global routes for pin '{}': found route to {} pins, expected {expected}",
            netlist.db.pin_name(driver),
            tree.loads().len(),
        );
        return None;
    }
    Some(tree)
}

/// The walk `makeBufferedNet` makes over a global route.
struct RouteWalk<'a, 'n> {
    netlist: &'a Netlist<'n>,
    pins_at: &'a FxHashMap<RoutePoint, Vec<PinId>>,
    adjacent: &'a FxHashMap<RoutePoint, Vec<RoutePoint>>,
    rc: &'a TreeRc<'a>,
    loads: &'a dyn LoadModel,
    visited: HashSet<RoutePoint>,
}

impl RouteWalk<'_, '_> {
    /// The subtree from `to` away from `from`, hung on a wire or via from
    /// `from`; none for a subtree without loads or one closing a loop.
    fn build(&mut self, from: Option<RoutePoint>, to: RoutePoint) -> Option<BnetPtr> {
        if !self.visited.insert(to) {
            log::debug!("loop found in global route");
            return None;
        }
        let mut built: Option<BnetPtr> = None;
        for &pin in self.pins_at.get(&to).map_or(&[][..], Vec::as_slice) {
            if self.netlist.is_load(pin) {
                let (cap, fanout, max_slew) = self.loads.load(pin);
                built = Some(join(
                    built,
                    Bnet::load(to.at, pin, cap, fanout, max_slew),
                    to.at,
                ));
            }
        }
        let next: Vec<RoutePoint> = self.adjacent.get(&to).cloned().unwrap_or_default();
        for adjacent in next {
            if Some(adjacent) == from {
                continue;
            }
            if let Some(sub) = self.build(Some(to), adjacent) {
                built = Some(join(built, sub, to.at));
            }
        }
        let built = built?;
        let Some(from) = from else {
            return Some(built);
        };
        Some(if from.at == to.at {
            Bnet::via(from.at, from.layer, to.layer, built, self.rc)
        } else {
            Bnet::wire(from.at, built, Some(to.layer), self.rc)
        })
    }
}

/// A buffered net over the Steiner tree of `root` and the given subtrees'
/// points, as `Resizer::makeBufferedNetSteinerOverBnets`; none without
/// sinks.
#[must_use]
pub fn over_subtrees(root: Point<Dbu>, sinks: &[BnetPtr], rc: &TreeRc<'_>) -> Option<BnetPtr> {
    if sinks.is_empty() {
        return None;
    }
    let mut points = vec![root];
    points.extend(sinks.iter().map(|s| s.at));
    let tree = steiner::prim_dijkstra(&points, 0, steiner::DEFAULT_ALPHA);
    let children = tree.children();
    let mut visited: HashSet<(i64, i64)> = HashSet::new();
    let mut leaves_at = |node: usize| -> Vec<BnetPtr> {
        if node >= tree.terminals {
            return Vec::new();
        }
        let at = tree.nodes[node];
        if !visited.insert((at.x.get(), at.y.get())) {
            return Vec::new();
        }
        sinks.iter().filter(|s| s.at == at).map(Rc::clone).collect()
    };
    build(&tree, &children, None, tree.root, &mut leaves_at, rc)
}

/// Walks the tree from `to`, joining the leaves at each node and the
/// subtrees under it, and hanging the result on a wire from `from`.
fn build(
    tree: &SteinerTree,
    children: &[Vec<usize>],
    from: Option<usize>,
    to: usize,
    leaves_at: &mut dyn FnMut(usize) -> Vec<BnetPtr>,
    rc: &TreeRc<'_>,
) -> Option<BnetPtr> {
    let at = tree.nodes[to];
    let mut built: Option<BnetPtr> = None;
    for leaf in leaves_at(to) {
        built = Some(join(built, leaf, at));
    }
    for &child in &children[to] {
        if let Some(sub) = build(tree, children, Some(to), child, leaves_at, rc) {
            built = Some(join(built, sub, at));
        }
    }
    let built = built?;
    match from {
        Some(from) if tree.nodes[from] != at => Some(Bnet::wire(tree.nodes[from], built, None, rc)),
        _ => Some(built),
    }
}

fn join(existing: Option<BnetPtr>, next: BnetPtr, at: Point<Dbu>) -> BnetPtr {
    match existing {
        None => next,
        Some(left) => Bnet::junction(at, left, next),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_delays_round_to_femtoseconds_and_saturate() {
        assert_eq!(Fixed::from_seconds(1.5e-12), Fixed(1500));
        assert_eq!(Fixed::from_seconds(1e3), Fixed::INF);
        assert_eq!(Fixed::from_seconds(-1e3), -Fixed::INF);
        assert_eq!(Fixed::lerp(Fixed(0), Fixed(100), 0.25), Fixed(25));
        assert!((Fixed(2000).to_seconds() - 2e-12).abs() < 1e-24);
    }

    #[test]
    fn a_wire_takes_its_elmore_delay_off_the_slack() {
        let rc = WireRc::per_micron(1.0, 1.0, pare_common::db::indices::LayerId::new(0));
        let load = Bnet::load(
            Point::new(Dbu(2000), Dbu(0)),
            PinId::new(0),
            1e-15,
            1.0,
            1e-9,
        );
        let load = load.with_slack(Fixed::from_seconds(1e-9), Some(SlackEdges::One(Edge::Rise)));

        let tree_rc = TreeRc {
            signal: rc,
            layers: None,
            dbu: 2000.0,
        };

        let wire = Bnet::wire(Point::new(Dbu(0), Dbu(0)), load, None, &tree_rc);

        // One micron of wire: 1 ohm and 1 fF; delay = 1 * (0.5 fF + 1 fF).
        assert_eq!(wire.length(), 2000);
        assert!((wire.cap - 2e-15).abs() < 1e-30);
        assert_eq!(wire.delay, Fixed(1));
        assert_eq!(wire.slack, Fixed(1_000_000 - 1));
        assert_eq!(wire.max_load_wire_length(), 2000);
    }

    #[test]
    fn a_junction_combines_its_sides() {
        let a = Bnet::load(Point::new(Dbu(0), Dbu(0)), PinId::new(0), 1e-15, 1.0, 2e-9)
            .with_slack(Fixed(10), Some(SlackEdges::One(Edge::Rise)));
        let b = Bnet::load(Point::new(Dbu(0), Dbu(0)), PinId::new(1), 2e-15, 1.0, 1e-9)
            .with_slack(Fixed(5), Some(SlackEdges::One(Edge::Fall)));

        let junction = Bnet::junction(Point::new(Dbu(0), Dbu(0)), a, b);

        assert!((junction.cap - 3e-15).abs() < 1e-30);
        assert!((junction.fanout - 2.0).abs() < 1e-12);
        assert!((junction.max_load_slew - 1e-9).abs() < 1e-24);
        assert_eq!(junction.slack, Fixed(5));
        assert_eq!(junction.slack_edges, Some(SlackEdges::Both));
        assert_eq!(junction.loads(), vec![PinId::new(0), PinId::new(1)]);
    }
}
