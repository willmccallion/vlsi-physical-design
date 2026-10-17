//! Design rule checks on exact geometry, as `OpenROAD`'s detailed router
//! checks a routed design.
//!
//! This follows `TritonRoute`'s checker (`FlexGC`): each owner's shapes on a
//! metal layer merge into polygons, whose maximal rectangles and boundary
//! edges the rules look at, while cuts are checked one by one. A violation
//! made of fixed metal alone (cells, I/O pins, special wiring, blockages) is
//! one routing cannot fix; `OpenROAD` leaves those out, and they are
//! reported here apart from the rest, split by whether placement could have
//! avoided them.

mod cut;
mod eol;
mod layout;
mod metal;
mod min_cut;
mod min_step;
mod model;
mod rules;
mod shape;
#[cfg(test)]
mod tests;
mod tile;

pub use layout::{Layout, LayoutShape, Origin, Owner, OwnerIndex};
pub use rules::{CutRules, MetalRules, NonDefaultRules, Rules};
pub use tile::{check_tiled, check_window};

/// How many halos wide a tile of [`check_drc`] is.
const TILE_HALOS: i64 = 32;

use crate::db::design::NetlistDB;
use crate::db::indices::LayerId;
use crate::db::library::LayerRef;
use crate::geom::dbu::{Dbu, DbuArea};
use crate::geom::rect::Rect;
use model::Model;
use std::collections::{HashMap, HashSet};

/// A kind of design rule violation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Violation {
    /// Metal of different owners touching.
    Short,
    /// Cuts touching.
    CutShort,
    /// Two pieces of one owner's metal overlapping by less than the minimum
    /// width.
    NonSufficientMetal,
    /// Metal closer than the layer's spacing rule or parallel-run table
    /// allows.
    MetalSpacing,
    /// Metal closer than the layer's two-widths spacing table allows.
    SpacingTableTw,
    /// Metal closer than a width-range spacing rule allows.
    SpacingRange,
    /// Cuts closer than a cut spacing rule allows.
    CutSpacing,
    /// A connected piece of metal smaller than the layer's minimum area.
    MinArea,
    /// Metal narrower than the layer's minimum width.
    MinWidth,
    /// A cut on metal wide enough for a minimum-cut rule.
    MinimumCut,
    /// A hole in metal smaller than the layer's minimum enclosed area.
    MinHole,
    /// Routed metal off the manufacturing grid.
    OffGrid,
    /// A line end closer to facing metal than the end-of-line rule allows.
    EndOfLineSpacing,
    /// More short boundary edges in a row than the minimum step rule allows.
    MinStep,
}

impl Violation {
    /// Whether the kind is reported for information only and does not
    /// decide whether a routing passes: the minimum step rule, which
    /// `OpenROAD` never checks when the LEF gives it as a property.
    pub const fn is_advisory(self) -> bool {
        matches!(self, Self::MinStep)
    }

    /// The name `OpenROAD` reports the violation by.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Short => "Short",
            Self::CutShort => "Cut Short",
            Self::NonSufficientMetal => "NS Metal",
            Self::MetalSpacing => "Metal Spacing",
            Self::SpacingTableTw => "SpacingTableTw",
            Self::SpacingRange => "SpacingRange",
            Self::CutSpacing => "Cut Spacing",
            Self::MinArea => "Min Area",
            Self::MinWidth => "Min Width",
            Self::MinimumCut => "Minimum Cut",
            Self::MinHole => "Min Hole",
            Self::OffGrid => "Off Grid",
            Self::EndOfLineSpacing => "EOL Spacing",
            Self::MinStep => "Min Step",
        }
    }
}

/// One violation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Marker {
    /// What kind of violation it is.
    pub violation: Violation,
    /// Which of the layer's rules of that kind it breaks.
    pub rule: u16,
    /// The layer.
    pub layer: LayerRef,
    /// Where it is.
    pub bbox: Rect<Dbu>,
    /// The two owners involved, in order; the same owner twice when an
    /// owner's metal breaks a rule with itself.
    pub owners: (OwnerIndex, OwnerIndex),
}

impl Marker {
    fn new(
        violation: Violation,
        rule: u16,
        layer: LayerRef,
        bbox: Rect<Dbu>,
        a: OwnerIndex,
        b: OwnerIndex,
    ) -> Self {
        Self {
            violation,
            rule,
            layer,
            bbox,
            owners: (a.min(b), a.max(b)),
        }
    }

    /// An order of markers by what they are and where.
    fn sort_key(&self) -> impl Ord + use<> {
        let layer = match self.layer {
            LayerRef::Routing(l) => (0, l.index()),
            LayerRef::Cut(c) => (1, c.index()),
        };
        let b = self.bbox;
        (
            self.violation,
            self.rule,
            layer,
            (b.min.x, b.min.y, b.max.x, b.max.y),
            self.owners,
        )
    }
}

/// Markers being collected: those routing can fix, and those among fixed
/// metal only.
#[derive(Debug, Default)]
struct Markers {
    violations: Vec<Marker>,
    fixed: Vec<Marker>,
}

impl Markers {
    fn add(&mut self, marker: Marker, fixed_only: bool) {
        if fixed_only {
            self.fixed.push(marker);
        } else {
            self.violations.push(marker);
        }
    }

    fn merge(mut self, other: Self) -> Self {
        self.violations.extend(other.violations);
        self.fixed.extend(other.fixed);
        self
    }
}

/// Markers sorted into the classes a report has, without the owner table.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Findings {
    /// Violations involving routed metal, which routing can fix.
    pub violations: Vec<Marker>,
    /// Violations of advisory rules involving routed metal.
    pub advisory: Vec<Marker>,
    /// Violations between a net's routed metal inside one of its own pins
    /// and the obstruction of the cell that covers that pin, which no
    /// routing can avoid.
    pub pin_obstruction: Vec<Marker>,
    /// Violations among fixed metal only that a placed cell or I/O pin
    /// takes part in.
    pub placement: Vec<Marker>,
    /// Violations among fixed metal only that the input alone makes.
    pub input: Vec<Marker>,
}

impl Findings {
    /// Both sets of findings together.
    #[must_use]
    pub fn merge(mut self, other: Self) -> Self {
        self.violations.extend(other.violations);
        self.advisory.extend(other.advisory);
        self.pin_obstruction.extend(other.pin_obstruction);
        self.placement.extend(other.placement);
        self.input.extend(other.input);
        self
    }
}

/// What a design rule check found.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DrcReport {
    /// Violations involving routed metal, which routing can fix, as
    /// `OpenROAD` reports them.
    pub violations: Vec<Marker>,
    /// Violations of advisory rules involving routed metal, reported but
    /// not held against the routing.
    pub advisory: Vec<Marker>,
    /// Violations between a net's routed metal inside one of its own pins
    /// and the obstruction covering that pin, which the library makes
    /// unavoidable and `OpenROAD` still reports.
    pub pin_obstruction: Vec<Marker>,
    /// Violations among fixed metal only where a cell or I/O pin placement
    /// can move takes part, which placement could have avoided.
    pub placement: Vec<Marker>,
    /// Violations among fixed metal only that the input alone makes: fixed
    /// cells and I/O pins, special wiring and blockages.
    pub input: Vec<Marker>,
    /// The owner each index of the markers stands for.
    pub owners: Vec<Owner>,
}

impl DrcReport {
    /// The report of `found`, each class sorted and without repeats.
    fn from_findings(found: Findings, owners: Vec<Owner>) -> Self {
        Self {
            violations: deduplicated(found.violations),
            advisory: deduplicated(found.advisory),
            pin_obstruction: deduplicated(found.pin_obstruction),
            placement: deduplicated(found.placement),
            input: deduplicated(found.input),
            owners,
        }
    }

    /// How many violations of each kind there are, by kind.
    pub fn counts(&self) -> Vec<(Violation, usize)> {
        counts(&self.violations)
    }

    /// How many advisory violations there are, by kind.
    pub fn advisory_counts(&self) -> Vec<(Violation, usize)> {
        counts(&self.advisory)
    }

    /// How many fixed-only violations placement takes part in, by kind.
    pub fn placement_counts(&self) -> Vec<(Violation, usize)> {
        counts(&self.placement)
    }

    /// How many fixed-only violations the input alone makes, by kind.
    pub fn input_counts(&self) -> Vec<(Violation, usize)> {
        counts(&self.input)
    }
}

fn counts(markers: &[Marker]) -> Vec<(Violation, usize)> {
    let mut kinds: Vec<Violation> = markers.iter().map(|m| m.violation).collect();
    kinds.sort_unstable();
    let mut out: Vec<(Violation, usize)> = Vec::new();
    for kind in kinds {
        match out.last_mut() {
            Some((last, n)) if *last == kind => *n += 1,
            _ => out.push((kind, 1)),
        }
    }
    out
}

/// Checks every shape of `db` against the rules of its library and its
/// nondefault rules, tile by tile.
///
/// `OpenROAD` reads every wire at its layer's default width, so the wiring
/// of nets under a nondefault rule must also meet the shape rules at that
/// width, and is checked for them a second time that way.
pub fn check_drc(db: &NetlistDB) -> DrcReport {
    let layout = Layout::from_db(db);
    let rules = Rules::from_design(db);
    let halo = rules.max_reach() * 2;
    let mut report = check_tiled(&layout, &rules, halo * TILE_HALOS, halo);
    let default_width = Layout::default_width_view_of_rule_nets(db, &layout);
    let found = shape::check(&Model::new(&default_width), &rules);
    report.violations = deduplicated(
        report
            .violations
            .into_iter()
            .chain(found.violations)
            .collect(),
    );
    report
}

/// A connected piece of an owner's routed metal below its layer's minimum
/// area, as the minimum area rule would mark it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ThinPiece {
    /// Whose metal it is.
    pub owner: OwnerIndex,
    /// The layer.
    pub layer: LayerId,
    /// The box around it.
    pub bbox: Rect<Dbu>,
    /// Its area.
    pub area: DbuArea,
    /// The area its layer requires.
    pub min_area: DbuArea,
}

/// Every piece of metal in `layout` short of its layer's minimum area.
///
/// A piece with a fixed edge is exempt. With a `window`, only the shapes
/// meeting it are looked at, and a piece reaching the window's edge, which
/// may be cut short there, is never reported.
pub fn thin_pieces(layout: &Layout, rules: &Rules, window: Option<Rect<Dbu>>) -> Vec<ThinPiece> {
    let model = window.map_or_else(|| Model::new(layout), |w| Model::in_window(layout, w));
    shape::thin_pieces(&model, rules)
}

/// Checks every shape of `layout` against `rules` in one piece.
pub fn check(layout: &Layout, rules: &Rules) -> DrcReport {
    let model = Model::new(layout);
    let found = classify(run_checks(&model, rules), &model);
    DrcReport::from_findings(found, layout.owners().to_vec())
}

/// Every rule's markers on `model`.
fn run_checks(model: &Model<'_>, rules: &Rules) -> Markers {
    metal::check(model, rules)
        .merge(eol::check(model, rules))
        .merge(min_step::check(model, rules))
        .merge(cut::check(model, &rules.cut))
        .merge(shape::check(model, rules))
        .merge(min_cut::check(model, rules))
}

/// Sorts `found` into the report's classes: advisory kinds apart from the
/// rest, markers of pins under their own cell's obstruction apart from
/// what routing can fix, and fixed-only markers by whether placement takes
/// part in them.
fn classify(found: Markers, model: &Model<'_>) -> Findings {
    let (advisory, routed): (Vec<Marker>, Vec<Marker>) = deduplicated(found.violations)
        .into_iter()
        .partition(|m| m.violation.is_advisory());
    let (pin_obstruction, violations) = split_pin_obstructions(model, routed);
    let seen: HashSet<Marker> = violations.iter().chain(&pin_obstruction).copied().collect();
    let (placement, input) = deduplicated(
        found
            .fixed
            .into_iter()
            .filter(|m| !seen.contains(m))
            .collect(),
    )
    .into_iter()
    .partition(|m| placement_takes_part(model, m));
    Findings {
        violations,
        advisory,
        pin_obstruction,
        placement,
        input,
    }
}

/// Splits off the routed markers between a net and a cell's obstruction
/// that lie inside a fixed shape of that net on the marker's layer: the
/// net's pin sits under the obstruction, so any metal reaching it breaks
/// the rule.
fn split_pin_obstructions(model: &Model<'_>, routed: Vec<Marker>) -> (Vec<Marker>, Vec<Marker>) {
    let owners = model.layout.owners();
    let net_of = |m: &Marker| match (owners[m.owners.0.index()], owners[m.owners.1.index()]) {
        (Owner::Net(_), Owner::Obstruction(_)) => Some(m.owners.0),
        (Owner::Obstruction(_), Owner::Net(_)) => Some(m.owners.1),
        _ => None,
    };
    let candidates: Vec<(usize, OwnerIndex, usize)> = routed
        .iter()
        .enumerate()
        .filter_map(|(i, m)| match m.layer {
            LayerRef::Routing(l) => net_of(m).map(|n| (i, n, l.index())),
            LayerRef::Cut(_) => None,
        })
        .collect();
    if candidates.is_empty() {
        return (Vec::new(), routed);
    }
    let wanted: HashSet<(OwnerIndex, usize)> = candidates.iter().map(|&(_, n, l)| (n, l)).collect();
    let mut pins: HashMap<(OwnerIndex, usize), Vec<Rect<Dbu>>> = HashMap::new();
    for &(_, l) in &wanted {
        for s in model.layout.metal(l) {
            if s.is_fixed() && wanted.contains(&(s.owner, l)) {
                pins.entry((s.owner, l)).or_default().push(s.rect);
            }
        }
    }
    let mut under = vec![false; routed.len()];
    for (i, n, l) in candidates {
        under[i] = pins
            .get(&(n, l))
            .is_some_and(|rects| rects.iter().any(|r| r.encloses(&routed[i].bbox)));
    }
    let mut flagged = under.into_iter();
    routed
        .into_iter()
        .partition(|_| flagged.next().unwrap_or(false))
}

/// Whether metal of a cell or I/O pin placement can move is part of the
/// fixed-only marker `m`.
fn placement_takes_part(model: &Model<'_>, m: &Marker) -> bool {
    match m.layer {
        LayerRef::Routing(l) => model
            .metal
            .get(l.index())
            .is_some_and(|metal| metal.placed_metal_meets(m.owners, &m.bbox)),
        LayerRef::Cut(c) => model
            .cut
            .get(c.index())
            .is_some_and(|cut| cut.placed_cut_meets(m.owners, &m.bbox)),
    }
}

fn deduplicated(mut markers: Vec<Marker>) -> Vec<Marker> {
    markers.sort_by_key(Marker::sort_key);
    markers.dedup();
    markers
}
