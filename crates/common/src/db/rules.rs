//! Technology design rules read from LEF.
//!
//! Spacing tables, end-of-line, minimum step, corner, area and minimum-cut
//! rules on metal layers; spacing and enclosure rules on cut layers; via
//! generation rules; and library-wide rules relating layers or cells.

use crate::db::indices::{CutLayerId, LayerId};
use crate::db::library::LayerRef;
use crate::geom::dbu::{Dbu, DbuArea};
use crate::geom::point::Point;
use crate::geom::rect::Rect;

/// Spacing between two shapes that depends on their widths and on how far
/// they run side by side.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SpacingTable {
    /// LEF `SPACINGTABLE PARALLELRUNLENGTH`.
    ParallelRunLength(ParallelRunLengthTable),
    /// LEF `SPACINGTABLE TWOWIDTHS`.
    TwoWidths(TwoWidthsTable),
}

impl SpacingTable {
    /// The spacing two shapes `width_a` and `width_b` wide need when they run
    /// side by side for `run`, zero or less when they do not face each other.
    pub fn spacing(&self, width_a: Dbu, width_b: Dbu, run: Dbu) -> Dbu {
        match self {
            Self::ParallelRunLength(t) => t.spacing(width_a.max(width_b), run),
            Self::TwoWidths(t) => t.spacing(width_a, width_b, run),
        }
    }

    /// The spacing of the narrowest shapes at no run.
    pub fn min_spacing(&self) -> Dbu {
        match self {
            Self::ParallelRunLength(t) => t.rows.first().and_then(|(_, s)| s.first()),
            Self::TwoWidths(t) => t.rows.first().and_then(|r| r.spacings.first()),
        }
        .copied()
        .unwrap_or_default()
    }

    /// The largest spacing any two shapes can need.
    pub fn max_spacing(&self) -> Dbu {
        match self {
            Self::ParallelRunLength(t) => t.rows.iter().flat_map(|(_, s)| s.iter().copied()).max(),
            Self::TwoWidths(t) => t.rows.iter().flat_map(|r| r.spacings.iter().copied()).max(),
        }
        .unwrap_or_default()
    }
}

/// Spacing by the width of the wider of two shapes and their parallel run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParallelRunLengthTable {
    /// Run length thresholds, increasing from zero.
    pub run_lengths: Vec<Dbu>,
    /// Width thresholds, increasing from zero, each with one spacing per run
    /// length threshold.
    pub rows: Vec<(Dbu, Vec<Dbu>)>,
}

impl ParallelRunLengthTable {
    /// The spacing two shapes need when the wider is `width` wide and they
    /// run side by side for `run`. A width threshold applies from its value
    /// up, a run length one only beyond it, as `OpenROAD` reads the table.
    pub fn spacing(&self, width: Dbu, run: Dbu) -> Dbu {
        let row = self
            .rows
            .iter()
            .rposition(|(w, _)| *w <= width)
            .unwrap_or(0);
        let column = self.run_lengths.iter().rposition(|&r| r < run).unwrap_or(0);
        self.rows
            .get(row)
            .and_then(|(_, spacings)| spacings.get(column))
            .copied()
            .unwrap_or_default()
    }
}

/// Spacing by the widths of both shapes, each row also requiring a parallel
/// run before it applies.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TwoWidthsTable {
    /// Rows by increasing width; row `i` holds the spacing to a shape of
    /// each row's width in column order.
    pub rows: Vec<TwoWidthsRow>,
}

/// One width of a [`TwoWidthsTable`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TwoWidthsRow {
    /// The row applies to shapes wider than this.
    pub width: Dbu,
    /// The row applies only to shapes running side by side at least this
    /// far; `None` places no condition.
    pub run_length: Option<Dbu>,
    /// Spacing to a shape in each row, one per row.
    pub spacings: Vec<Dbu>,
}

impl TwoWidthsTable {
    /// The spacing of two shapes, indexed as `OpenDB` does: the last row whose
    /// width the shape exceeds, stepping down past rows whose run length
    /// condition fails.
    pub fn spacing(&self, width_a: Dbu, width_b: Dbu, run: Dbu) -> Dbu {
        let run = run.max(Dbu::ZERO);
        let (row, column) = (self.index(width_a, run), self.index(width_b, run));
        self.rows
            .get(row)
            .and_then(|r| r.spacings.get(column))
            .copied()
            .unwrap_or_default()
    }

    fn index(&self, width: Dbu, run: Dbu) -> usize {
        let wider_than = self.rows.iter().take_while(|r| r.width < width).count();
        (0..wider_than.max(1))
            .rev()
            .find(|&i| self.rows[i].run_length.is_none_or(|need| run >= need))
            .unwrap_or(0)
    }
}

/// Extra spacing at the end of a narrow wire (LEF `SPACING ... ENDOFLINE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EndOfLineSpacing {
    /// Spacing required beyond the line end.
    pub spacing: Dbu,
    /// The rule applies to line ends narrower than this.
    pub eol_width: Dbu,
    /// How far to either side of the line end the spacing is checked.
    pub within: Dbu,
    /// A further condition on metal running parallel to the wire.
    pub parallel_edge: Option<ParallelEdge>,
}

/// The `PARALLELEDGE` condition of an end-of-line rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParallelEdge {
    /// Parallel metal closer than this triggers the rule.
    pub spacing: Dbu,
    /// How far back from the line end parallel metal is looked for.
    pub within: Dbu,
    /// Whether parallel metal must be present on both sides.
    pub two_edges: bool,
}

/// Spacing for shapes whose width lies in a range (LEF `SPACING ... RANGE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WidthRangeSpacing {
    /// The spacing.
    pub spacing: Dbu,
    /// Narrowest shape the rule applies to.
    pub min_width: Dbu,
    /// Widest shape the rule applies to.
    pub max_width: Dbu,
}

/// A minimum edge length rule (LEF `MINSTEP`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MinStep {
    /// Edges shorter than this are steps.
    pub length: Dbu,
    /// How many consecutive short edges are allowed, if limited.
    pub max_edges: Option<u32>,
}

/// A minimum area of metal enclosed by a ring (LEF `MINENCLOSEDAREA`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MinEnclosedArea {
    /// The smallest allowed hole.
    pub area: DbuArea,
    /// The rule applies when the ring is at most this wide, if given.
    pub width: Option<Dbu>,
}

/// The number of cuts a via must have where it lands on wide metal (LEF
/// `MINIMUMCUT`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MinimumCut {
    /// Cuts required.
    pub cuts: u32,
    /// The rule applies to metal at least this wide.
    pub width: Dbu,
    /// Cuts closer together than this count as one group, if given.
    pub within: Option<Dbu>,
    /// Which via layer the rule restricts, if only one.
    pub from: Option<CutApproach>,
    /// The rule applies only to wide metal at least this long, within this
    /// distance of the via.
    pub length: Option<(Dbu, Dbu)>,
}

/// The side a via connects to a metal layer from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutApproach {
    /// Only vias on the cut layer above (`FROMABOVE`).
    Above,
    /// Only vias on the cut layer below (`FROMBELOW`).
    Below,
}

/// Spacing from a convex corner (LEF58 `CORNERSPACING CONVEXCORNER`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConvexCornerSpacing {
    /// Corners on a line end narrower than this are exempt, if given.
    pub except_eol: Option<Dbu>,
    /// Spacing by shape width, as `(width, spacing)` pairs in increasing width.
    pub spacing_by_width: Vec<(Dbu, Dbu)>,
}

/// Rules on a metal layer beyond its default width and spacing.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RoutingLayerRules {
    /// Narrowest legal shape.
    pub min_width: Option<Dbu>,
    /// Widest legal shape.
    pub max_width: Option<Dbu>,
    /// Smallest legal area of a connected piece of metal.
    pub min_area: Option<DbuArea>,
    /// Smallest legal holes.
    pub min_enclosed_areas: Vec<MinEnclosedArea>,
    /// Spacing rules for shapes in a width range.
    pub width_range_spacing: Vec<WidthRangeSpacing>,
    /// End-of-line spacing rules.
    pub end_of_line: Vec<EndOfLineSpacing>,
    /// Minimum edge length rules.
    pub min_steps: Vec<MinStep>,
    /// Convex corner spacing.
    pub corner_spacing: Option<ConvexCornerSpacing>,
    /// Cut counts required on wide metal.
    pub minimum_cuts: Vec<MinimumCut>,
}

/// A spacing rule between cuts (LEF cut layer `SPACING`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CutSpacing {
    /// The spacing.
    pub spacing: Dbu,
    /// Measured between cut centers rather than edges.
    pub center_to_center: bool,
    /// Applies only between cuts of the same net.
    pub same_net: bool,
    /// When the rule applies.
    pub condition: CutSpacingCondition,
}

/// The condition under which a [`CutSpacing`] applies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutSpacingCondition {
    /// Between any two cuts.
    Always,
    /// To a cut that has at least `cuts` neighbours closer than `within`.
    AdjacentCuts {
        /// Neighbours that trigger the rule.
        cuts: u32,
        /// Distance within which a cut is a neighbour.
        within: Dbu,
    },
    /// Between cuts whose enclosing metal overlaps (`PARALLELOVERLAP`).
    ParallelOverlap,
}

/// How far metal must extend past a cut (LEF cut layer `ENCLOSURE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Enclosure {
    /// The metal layer the rule applies to, if only one.
    pub side: Option<CutApproach>,
    /// Required overhang on one pair of opposite sides.
    pub overhang_1: Dbu,
    /// Required overhang on the other pair.
    pub overhang_2: Dbu,
    /// The rule applies when the metal is at least this wide, if given.
    pub min_width: Option<Dbu>,
    /// The rule applies when the metal is at least this long, if given.
    pub min_length: Option<Dbu>,
}

/// Rules on a cut layer.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CutLayerRules {
    /// Cut spacing rules.
    pub spacings: Vec<CutSpacing>,
    /// Metal enclosure rules.
    pub enclosures: Vec<Enclosure>,
}

/// A rule for generating via arrays between two metal layers (LEF `VIARULE
/// ... GENERATE`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ViaRule {
    /// Rule name.
    pub name: String,
    /// Whether routers may use it for ordinary connections.
    pub is_default: bool,
    /// The metal below the cut.
    pub bottom: ViaRuleMetal,
    /// The metal above the cut.
    pub top: ViaRuleMetal,
    /// The cut layer.
    pub cut: CutLayerId,
    /// One cut, centered on the origin.
    pub cut_rect: Rect<Dbu>,
    /// Center-to-center pitch of the cut array.
    pub cut_pitch: Point<Dbu>,
}

/// One metal layer of a [`ViaRule`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ViaRuleMetal {
    /// The layer.
    pub layer: LayerId,
    /// Overhang past the cut array on one pair of sides and the other.
    pub enclosure: (Dbu, Dbu),
    /// The range of wire widths the rule is for, if limited.
    pub width_range: Option<(Dbu, Dbu)>,
}

/// How spacing is measured between shapes that do not face each other.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ClearanceMeasure {
    /// Straight-line distance between corners.
    #[default]
    Euclidean,
    /// The larger of the X and Y distances.
    MaxXy,
}

/// A limit on vias stacked directly on each other (LEF `MAXVIASTACK`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MaxViaStack {
    /// The most vias that may stack.
    pub vias: u32,
    /// The metal layers the limit applies between, if not the whole stack.
    pub range: Option<(LayerId, LayerId)>,
}

/// Spacing between same-net shapes on two layers (LEF `SPACING ... SAMENET`
/// block).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SameNetSpacing {
    /// One layer.
    pub first: LayerRef,
    /// The other layer, possibly the same.
    pub second: LayerRef,
    /// The spacing.
    pub spacing: Dbu,
    /// Whether cuts on the two layers may stack directly on each other.
    pub stack: bool,
}

/// A cell edge class used by cell edge spacing rules.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct EdgeType(pub String);

/// The edges of a cell an edge type is assigned to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CellSide {
    /// The left edge.
    Left,
    /// The right edge.
    Right,
    /// Both the left and right edges.
    Both,
    /// The top edge.
    Top,
    /// The bottom edge.
    Bottom,
}

/// An edge type on some edges of a cell master (LEF57/LEF58 `EDGETYPE`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellEdge {
    /// Which edges.
    pub side: CellSide,
    /// The edge type.
    pub edge_type: EdgeType,
}

/// Minimum distance between abutting cell edges of two types (LEF57/LEF58
/// `CELLEDGESPACINGTABLE`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellEdgeSpacing {
    /// One edge type.
    pub first: EdgeType,
    /// The other edge type.
    pub second: EdgeType,
    /// The required distance between the two edges.
    pub spacing: Dbu,
}

/// Rules of the whole technology rather than one layer.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct LibraryRules {
    /// The grid every shape must lie on, if given.
    pub manufacturing_grid: Option<Dbu>,
    /// How spacing is measured diagonally.
    pub clearance_measure: ClearanceMeasure,
    /// Whether obstruction shapes use minimum spacing rather than width-based
    /// spacing (`USEMINSPACING OBS ON`).
    pub min_spacing_for_obstructions: bool,
    /// Via stacking limit.
    pub max_via_stack: Option<MaxViaStack>,
    /// Same-net spacing between layers.
    pub same_net_spacing: Vec<SameNetSpacing>,
    /// Spacing between cell edges by edge type.
    pub cell_edge_spacing: Vec<CellEdgeSpacing>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn two_widths() -> TwoWidthsTable {
        let row = |width, run_length: Option<i64>, spacings: [i64; 3]| TwoWidthsRow {
            width: Dbu(width),
            run_length: run_length.map(Dbu),
            spacings: spacings.map(Dbu).to_vec(),
        };
        TwoWidthsTable {
            rows: vec![
                row(0, None, [50, 50, 64]),
                row(74, Some(0), [50, 60, 74]),
                row(212, Some(302), [64, 74, 112]),
            ],
        }
    }

    #[test]
    fn two_widths_rows_apply_only_to_shapes_wider_than_their_width() {
        let table = two_widths();

        assert_eq!(table.spacing(Dbu(74), Dbu(74), Dbu(1000)), Dbu(50));
        assert_eq!(table.spacing(Dbu(75), Dbu(75), Dbu(1000)), Dbu(60));
    }

    #[test]
    fn two_widths_rows_step_down_until_the_run_length_condition_holds() {
        let table = two_widths();

        assert_eq!(table.spacing(Dbu(300), Dbu(300), Dbu(1000)), Dbu(112));
        assert_eq!(table.spacing(Dbu(300), Dbu(300), Dbu(302)), Dbu(112));
        assert_eq!(table.spacing(Dbu(300), Dbu(300), Dbu(301)), Dbu(60));
    }

    #[test]
    fn two_widths_treats_shapes_apart_as_no_run() {
        let table = two_widths();

        assert_eq!(table.spacing(Dbu(300), Dbu(10), Dbu(-40)), Dbu(50));
    }

    #[test]
    fn two_widths_extremes_are_its_first_and_largest_entries() {
        let table = SpacingTable::TwoWidths(two_widths());

        assert_eq!(table.min_spacing(), Dbu(50));
        assert_eq!(table.max_spacing(), Dbu(112));
    }
}
