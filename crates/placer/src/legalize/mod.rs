//! Legalization: moving globally placed cells onto rows without overlap.

pub mod abacus;
pub mod opendp;
pub mod site;

use crate::zones::Zones;
use pare_common::db::design::NetlistDB;
use pare_common::db::library::MacroClass;

/// Why legalization failed.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum LegalizationError {
    /// Abacus found no room.
    #[error(transparent)]
    Abacus(#[from] abacus::LegalizeError),
    /// Detailed placement found no room.
    #[error(transparent)]
    DetailedPlacement(#[from] opendp::DetailedPlacementError),
}

/// How the placement was made legal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Legalization {
    /// By `OpenDP`'s detailed placement.
    DetailedPlacement(opendp::DetailedPlacement),
    /// By Abacus, which also places movable macros.
    Abacus,
}

/// Makes the placement legal as OpenROAD-flow-scripts' `detailed_placement`
/// does.
///
/// A design with macros the placer moves, which `OpenDP` leaves to a macro
/// placer, is legalized with Abacus, which places them too.
///
/// # Errors
///
/// When a cell finds no legal spot.
pub fn legalize(db: &mut NetlistDB, zones: &Zones) -> Result<Legalization, LegalizationError> {
    let moves_macros = db.cells.iter().any(|c| {
        !c.retired
            && c.status.is_movable()
            && !matches!(db.library.cell_macro(c.macro_id).class, MacroClass::Core(_))
    });
    if moves_macros {
        abacus::legalize(db, zones)?;
        return Ok(Legalization::Abacus);
    }
    Ok(Legalization::DetailedPlacement(opendp::detailed_placement(
        db,
    )?))
}
