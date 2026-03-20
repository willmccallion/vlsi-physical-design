//! Type-Safe Index Types for Netlist Database.
//!
//! This module defines strongly-typed index wrappers for cells, nets, pins,
//! and rows. These types prevent accidental mixing of indices from different
//! collections and provide compile-time safety. The indices are implemented
//! as transparent wrappers around u32 with zero-cost abstraction.

use std::fmt::Debug;

/// Macro that generates a type-safe index type for a given entity.
///
/// Creates a newtype wrapper around u32 with methods for conversion to/from
/// usize (for vector indexing) and a Debug implementation that includes the
/// type name for clarity in error messages.
macro_rules! define_index {
    ($name:ident) => {
        /// Type-safe index wrapper for netlist database entities.
        ///
        /// Prevents accidental mixing of indices from different collections (cells,
        /// nets, pins, rows) at compile time. The underlying u32 representation
        /// provides zero-cost abstraction with no runtime overhead. The transparent
        /// representation ensures compatibility with external APIs that expect raw
        /// integer indices.
        #[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        #[repr(transparent)]
        pub struct $name(pub u32);

        impl $name {
            /// Creates a new index from a usize value.
            ///
            /// Converts the usize to u32 internally. Used when creating indices
            /// from vector lengths or array positions.
            #[inline(always)]
            pub const fn new(id: usize) -> Self {
                Self(id as u32)
            }
            /// Returns the index as a usize for vector indexing.
            ///
            /// Converts the internal u32 representation to usize, which is
            /// required for indexing into Rust vectors and slices.
            #[inline(always)]
            pub const fn index(&self) -> usize {
                self.0 as usize
            }
        }

        impl Debug for $name {
            /// Formats the index for debugging output.
            ///
            /// Includes the type name and the numeric value to make error
            /// messages more informative when debugging index-related issues.
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "{}({})", stringify!($name), self.0)
            }
        }
    };
}

define_index!(CellId);
define_index!(NetId);
define_index!(PinId);
define_index!(RowId);
