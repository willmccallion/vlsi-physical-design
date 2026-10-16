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
mod layout;
mod metal;
mod min_cut;
mod model;
mod rules;
mod shape;
#[cfg(test)]
mod tests;

pub use layout::{Layout, LayoutShape, Origin, Owner, OwnerIndex};
pub use rules::{CutRules, MetalRules, NonDefaultRules, Rules};

use crate::db::design::NetlistDB;
use crate::db::library::LayerRef;
use crate::geom::dbu::Dbu;
use crate::geom::rect::Rect;
use model::Model;
use std::collections::HashSet;

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
}

impl Violation {
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

/// What a design rule check found.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DrcReport {
    /// Violations involving routed metal, which routing can fix, as
    /// `OpenROAD` reports them.
    pub violations: Vec<Marker>,
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
    /// How many violations of each kind there are, by kind.
    pub fn counts(&self) -> Vec<(Violation, usize)> {
        counts(&self.violations)
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
/// nondefault rules.
///
/// `OpenROAD` reads every wire at its layer's default width, so the wiring
/// of nets under a nondefault rule must also meet the shape rules at that
/// width, and is checked for them a second time that way.
pub fn check_drc(db: &NetlistDB) -> DrcReport {
    let layout = Layout::from_db(db);
    let rules = Rules::from_design(db);
    let mut report = check(&layout, &rules);
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

/// Checks every shape of `layout` against `rules`.
pub fn check(layout: &Layout, rules: &Rules) -> DrcReport {
    let model = Model::new(layout);
    let found = metal::check(&model, rules)
        .merge(cut::check(&model, &rules.cut))
        .merge(shape::check(&model, rules))
        .merge(min_cut::check(&model, rules));
    let violations = deduplicated(found.violations);
    let seen: HashSet<Marker> = violations.iter().copied().collect();
    let fixed = deduplicated(
        found
            .fixed
            .into_iter()
            .filter(|m| !seen.contains(m))
            .collect(),
    );
    let (placement, input) = fixed
        .into_iter()
        .partition(|m| placement_takes_part(&model, m));
    DrcReport {
        violations,
        placement,
        input,
        owners: layout.owners().to_vec(),
    }
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
