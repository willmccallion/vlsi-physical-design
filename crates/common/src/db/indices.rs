//! Typed indices into the design and library tables.

macro_rules! define_index {
    ($(#[$doc:meta])* $name:ident($repr:ty)) => {
        $(#[$doc])*
        #[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub struct $name(pub $repr);

        impl $name {
            /// Creates an index from a table position.
            #[inline]
            pub const fn new(index: usize) -> Self {
                Self(index as $repr)
            }

            /// Returns the table position.
            #[inline]
            pub const fn index(self) -> usize {
                self.0 as usize
            }
        }

        impl std::fmt::Debug for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "{}({})", stringify!($name), self.0)
            }
        }
    };
}

define_index!(
    /// A placed instance in [`NetlistDB::cells`](crate::db::design::NetlistDB::cells).
    CellId(u32)
);
define_index!(
    /// A net in [`NetlistDB::nets`](crate::db::design::NetlistDB::nets).
    NetId(u32)
);
define_index!(
    /// A net connection in [`NetlistDB::pins`](crate::db::design::NetlistDB::pins).
    PinId(u32)
);
define_index!(
    /// A top-level I/O pin in [`NetlistDB::io_pins`](crate::db::design::NetlistDB::io_pins).
    IoPinId(u32)
);
define_index!(
    /// A cell master in [`Library::macros`](crate::db::library::Library::macros).
    MacroId(u32)
);
define_index!(
    /// A pin of a cell master, indexing [`Macro::pins`](crate::db::library::Macro::pins).
    MacroPinId(u32)
);
define_index!(
    /// A placement site in [`Library::sites`](crate::db::library::Library::sites).
    SiteId(u32)
);
define_index!(
    /// A via definition in [`Library::vias`](crate::db::library::Library::vias).
    ViaId(u32)
);
define_index!(
    /// A routing layer, numbered from the bottom of the stack.
    LayerId(u8)
);
define_index!(
    /// The cut layer between routing layers `n` and `n + 1`.
    CutLayerId(u8)
);

impl LayerId {
    /// The cut layer directly above this routing layer.
    pub const fn cut_above(self) -> CutLayerId {
        CutLayerId(self.0)
    }
}

impl CutLayerId {
    /// The routing layer below this cut.
    pub const fn below(self) -> LayerId {
        LayerId(self.0)
    }

    /// The routing layer above this cut.
    pub const fn above(self) -> LayerId {
        LayerId(self.0 + 1)
    }
}
