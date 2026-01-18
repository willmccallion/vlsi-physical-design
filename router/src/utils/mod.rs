/// Coordinate conversion between physical and grid space.
///
/// Provides conversion between physical coordinates (floating-point microns) and
/// discrete grid coordinates (integer indices). Handles coordinate system
/// transformations including origin offsets and scaling factors. The converter
/// maintains scaling factors and origin offsets to transform between continuous
/// physical coordinate space and discrete grid space used by routing algorithms.
/// Accounts for die area boundaries and grid step sizes to ensure accurate
/// coordinate mapping.
pub mod conversion;
