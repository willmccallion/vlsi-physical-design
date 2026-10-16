//! The rules the checker applies, read from the library as `OpenROAD`'s
//! detailed router reads them.

use crate::db::library::Library;
use crate::db::rules::{
    CutSpacing, MinimumCut, ParallelRunLengthTable, SpacingTable, WidthRangeSpacing,
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
    /// Spacing rules between cuts.
    pub spacings: Vec<CutSpacing>,
}

impl CutRules {
    /// Whether the layer has a rule for cuts of the same net.
    pub fn has_same_net_rule(&self) -> bool {
        self.spacings.iter().any(|s| s.same_net)
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
}

impl Rules {
    /// The rules of `library`.
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
            })
            .collect();
        Self {
            metal,
            cut,
            min_spacing_for_obstructions: library.rules.min_spacing_for_obstructions,
            manufacturing_grid: library.rules.manufacturing_grid,
        }
    }
}
