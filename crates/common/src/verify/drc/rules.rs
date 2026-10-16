//! The rules the checker applies, read from the library as `OpenROAD`'s
//! detailed router reads them.

use crate::db::design::{NetlistDB, NonDefaultRule};
use crate::db::indices::{LayerId, NonDefaultRuleId};
use crate::db::library::Library;
use crate::db::rules::{
    CutSpacing, CutSpacingCondition, EndOfLineSpacing, Lef58CutSpacing, MinStep, MinimumCut,
    ParallelRunLengthTable, SpacingTable, WidthRangeSpacing,
};
use crate::geom::dbu::{Dbu, DbuArea};

/// Rules of one metal layer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MetalRules {
    /// Default wire width, which obstructions are measured as when
    /// [`Rules::min_spacing_for_obstructions`] holds.
    pub width: Dbu,
    /// Narrowest legal shape.
    pub min_width: Dbu,
    /// Spacing by width and parallel run. A plain `SPACING` is a table of
    /// one entry, and a spacing table replaces it.
    pub spacing: SpacingTable,
    /// Spacing for shapes in a width range.
    pub ranges: Vec<WidthRangeSpacing>,
    /// Spacing at line ends.
    pub end_of_line: Vec<EndOfLineSpacing>,
    /// Shortest legal boundary edges.
    pub min_steps: Vec<MinStep>,
    /// Smallest legal area of a connected piece of metal.
    pub min_area: Option<DbuArea>,
    /// Cuts that vias to wide metal need.
    pub minimum_cuts: Vec<MinimumCut>,
    /// Smallest holes metal may enclose. `TritonRoute` skips rules limited
    /// to narrow rings, so only those without a width are kept.
    pub min_enclosed_areas: Vec<DbuArea>,
}

impl MetalRules {
    /// The most spacing any two shapes on the layer can need, before range
    /// rules.
    pub fn max_spacing(&self) -> Dbu {
        self.spacing.max_spacing()
    }
}

/// Rules of one cut layer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CutRules {
    /// Width of a cut, if given.
    pub width: Option<Dbu>,
    /// Spacing rules between cuts from `SPACING` statements.
    pub spacings: Vec<CutSpacing>,
    /// Spacing rules between cuts from `LEF57_SPACING` and `LEF58_SPACING`
    /// properties, which `OpenROAD` checks on their own terms.
    pub lef58_spacings: Vec<Lef58CutSpacing>,
}

impl CutRules {
    /// Whether the layer has a `SPACING` statement for cuts of the same net.
    pub fn has_same_net_rule(&self) -> bool {
        self.spacings.iter().any(|s| s.same_net)
    }
}

/// What a nondefault rule asks of its nets beyond the layer rules.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NonDefaultRules {
    /// The hard spacing the rule's wires need on each routing layer, where
    /// the rule makes its spacing hard.
    pub hard_spacing: Vec<Option<Dbu>>,
}

impl NonDefaultRules {
    fn from_rule(rule: &NonDefaultRule, layers: usize) -> Self {
        let mut hard_spacing = vec![None; layers];
        if rule.hard_spacing {
            for l in &rule.layers {
                if let Some(slot) = hard_spacing.get_mut(l.layer.index()) {
                    *slot = l.spacing;
                }
            }
        }
        Self { hard_spacing }
    }
}

/// The rules of every layer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rules {
    /// Rules of each routing layer.
    pub metal: Vec<MetalRules>,
    /// Rules of each cut layer.
    pub cut: Vec<CutRules>,
    /// Whether obstructions count as wires of their layer's default width
    /// when looking up spacing.
    pub min_spacing_for_obstructions: bool,
    /// The grid every routed shape must lie on, if the library sets one.
    pub manufacturing_grid: Option<Dbu>,
    /// The design's nondefault rules, by their index.
    pub non_default: Vec<NonDefaultRules>,
}

impl Rules {
    /// The rules of `db`'s library and its nondefault rules.
    pub fn from_design(db: &NetlistDB) -> Self {
        let layers = db.library.routing_layers.len();
        Self {
            non_default: db
                .non_default_rules
                .iter()
                .map(|r| NonDefaultRules::from_rule(r, layers))
                .collect(),
            ..Self::from_library(&db.library)
        }
    }

    /// The hard spacing nondefault rule `rule` asks of its wires on `layer`,
    /// or zero.
    pub fn hard_spacing(&self, rule: Option<NonDefaultRuleId>, layer: LayerId) -> Dbu {
        rule.and_then(|r| self.non_default.get(r.index()))
            .and_then(|n| n.hard_spacing.get(layer.index()).copied().flatten())
            .unwrap_or(Dbu::ZERO)
    }

    /// The most hard spacing any nondefault rule asks of its wires on `layer`.
    pub fn max_hard_spacing(&self, layer: LayerId) -> Dbu {
        self.non_default
            .iter()
            .filter_map(|n| n.hard_spacing.get(layer.index()).copied().flatten())
            .max()
            .unwrap_or(Dbu::ZERO)
    }

    /// The farthest any rule looks from a shape: the distance within which
    /// every other shape a check of it can involve lies.
    pub fn max_reach(&self) -> Dbu {
        let metal = self.metal.iter().enumerate().flat_map(|(l, m)| {
            let layer = LayerId::new(l);
            let eol = m.end_of_line.iter().map(|e| {
                e.spacing + e.within + e.parallel_edge.map_or(Dbu::ZERO, |p| p.spacing + p.within)
            });
            let cuts = m
                .minimum_cuts
                .iter()
                .filter_map(|c| c.length.map(|(_, within)| within).or(c.within));
            [m.max_spacing(), self.max_hard_spacing(layer)]
                .into_iter()
                .chain(m.ranges.iter().map(|r| r.spacing))
                .chain(eol)
                .chain(cuts)
                .collect::<Vec<_>>()
        });
        let cut = self.cut.iter().flat_map(|c| {
            let statements = c.spacings.iter().map(|s| match s.condition {
                CutSpacingCondition::AdjacentCuts { within, .. } => s.spacing.max(within),
                _ => s.spacing,
            });
            let properties = c.lef58_spacings.iter().map(|s| s.spacing);
            statements.chain(properties).collect::<Vec<_>>()
        });
        metal.chain(cut).max().unwrap_or(Dbu::ZERO)
    }

    /// The rules of `library`, with no nondefault rules.
    pub fn from_library(library: &Library) -> Self {
        let metal = library
            .routing_layers
            .iter()
            .map(|l| MetalRules {
                width: l.width,
                min_width: l.rules.min_width.unwrap_or(l.width).min(l.width),
                spacing: l.spacing_table.clone().unwrap_or_else(|| {
                    SpacingTable::ParallelRunLength(ParallelRunLengthTable {
                        run_lengths: vec![Dbu::ZERO],
                        rows: vec![(Dbu::ZERO, vec![l.min_spacing])],
                    })
                }),
                ranges: l.rules.width_range_spacing.clone(),
                end_of_line: l.rules.end_of_line.clone(),
                min_steps: l.rules.min_steps.clone(),
                min_area: l.rules.min_area,
                minimum_cuts: l.rules.minimum_cuts.clone(),
                min_enclosed_areas: l
                    .rules
                    .min_enclosed_areas
                    .iter()
                    .filter(|r| r.width.is_none())
                    .map(|r| r.area)
                    .collect(),
            })
            .collect();
        let cut = library
            .cut_layers
            .iter()
            .map(|c| CutRules {
                width: c.width,
                spacings: c.rules.spacings.clone(),
                lef58_spacings: c.rules.lef58_spacings.clone(),
            })
            .collect();
        Self {
            metal,
            cut,
            min_spacing_for_obstructions: library.rules.min_spacing_for_obstructions,
            manufacturing_grid: library.rules.manufacturing_grid,
            non_default: Vec::new(),
        }
    }
}
