//! Legalization Algorithms.
//!
//! Legalization algorithms take the analytically placed design (which may
//! have overlapping cells) and move cells to legal positions that satisfy
//! placement constraints: no overlaps, alignment to placement rows, and
//! adherence to die boundaries.

pub mod abacus;
/// Tetris-style legalization algorithm for cell placement.
///
/// Implements a simpler legalization algorithm that places cells one at a time
/// in the nearest available slot, similar to the Tetris game. Cells are sorted
/// by X coordinate and placed in the best available position near their ideal
/// location, searching nearby rows if the ideal row is full. Simpler than
/// Abacus but may produce less optimal results for designs with high utilization.
/// Maintains occupancy intervals for each row to track available space efficiently.
pub mod tetris;
