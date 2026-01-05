use std::fmt::Debug;

macro_rules! define_index {
    ($name:ident) => {
        #[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        #[repr(transparent)]
        pub struct $name(pub u32);

        impl $name {
            #[inline(always)]
            pub fn new(id: usize) -> Self {
                Self(id as u32)
            }
            #[inline(always)]
            pub fn index(&self) -> usize {
                self.0 as usize
            }
        }

        impl Debug for $name {
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
