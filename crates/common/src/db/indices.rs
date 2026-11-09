//! Typed indices into the design and library tables.

use std::fmt;
use std::marker::PhantomData;
use std::ops::{Index, IndexMut};

/// A typed position in one kind of table, as [`define_index!`](crate::define_index) makes them.
pub trait TableIndex: Copy {
    /// The index of table position `index`.
    fn from_position(index: usize) -> Self;
    /// The table position.
    fn position(self) -> usize;
}

/// Defines a typed table index: a newtype over an unsigned integer that
/// only indexes the [`IdVec`] tables keyed by it.
#[macro_export]
macro_rules! define_index {
    ($(#[$doc:meta])* $name:ident($repr:ty)) => {
        $(#[$doc])*
        #[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub struct $name(pub $repr);

        impl $name {
            /// Creates an index from a table position.
            #[inline]
            pub const fn new(index: usize) -> Self {
                debug_assert!(index <= <$repr>::MAX as usize, "table position past the index type");
                Self(index as $repr)
            }

            /// Returns the table position.
            #[inline]
            pub const fn index(self) -> usize {
                self.0 as usize
            }
        }

        impl $crate::db::indices::TableIndex for $name {
            #[inline]
            fn from_position(index: usize) -> Self {
                Self::new(index)
            }

            #[inline]
            fn position(self) -> usize {
                self.index()
            }
        }

        impl std::fmt::Debug for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "{}({})", stringify!($name), self.0)
            }
        }
    };
}

/// A table indexed only by its own index type `I`, so that indexing it
/// with another table's index does not compile:
///
/// ```compile_fail
/// use pare_common::db::indices::{CellId, IdVec, NetId};
/// let names: IdVec<CellId, &str> = vec!["u1"].into();
/// let _ = names[NetId(0)];
/// ```
pub struct IdVec<I, T> {
    entries: Vec<T>,
    index: PhantomData<fn(I) -> I>,
}

impl<I: TableIndex, T> IdVec<I, T> {
    /// An empty table.
    pub const fn new() -> Self {
        Self {
            entries: Vec::new(),
            index: PhantomData,
        }
    }

    /// An empty table with room for `capacity` entries.
    pub fn with_capacity(capacity: usize) -> Self {
        Vec::with_capacity(capacity).into()
    }

    /// The table of `len` copies of `entry`.
    pub fn filled(entry: T, len: usize) -> Self
    where
        T: Clone,
    {
        vec![entry; len].into()
    }

    /// How many entries the table has.
    pub const fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the table has no entries.
    pub const fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Appends copies of `entry` until the table has `len` entries; a table
    /// that long already is left alone, so no index goes stale.
    pub fn grow_to(&mut self, len: usize, entry: T)
    where
        T: Clone,
    {
        if self.entries.len() < len {
            self.entries.resize(len, entry);
        }
    }

    /// Sets every entry to `entry`.
    pub fn fill(&mut self, entry: T)
    where
        T: Clone,
    {
        self.entries.fill(entry);
    }

    /// The index the next [`Self::push`] gives its entry.
    pub fn next_id(&self) -> I {
        I::from_position(self.entries.len())
    }

    /// Appends `entry`, returning its index.
    pub fn push(&mut self, entry: T) -> I {
        let id = self.next_id();
        self.entries.push(entry);
        id
    }

    /// The entry at `id`, if the table has one there.
    pub fn get(&self, id: I) -> Option<&T> {
        self.entries.get(id.position())
    }

    /// The entry at `id` to change, if the table has one there.
    pub fn get_mut(&mut self, id: I) -> Option<&mut T> {
        self.entries.get_mut(id.position())
    }

    /// Every index of the table, in order.
    pub fn ids(&self) -> impl Iterator<Item = I> + Clone + use<I, T> {
        (0..self.entries.len()).map(I::from_position)
    }

    /// Every entry, in order.
    pub fn iter(&self) -> std::slice::Iter<'_, T> {
        self.entries.iter()
    }

    /// Every entry to change, in order.
    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, T> {
        self.entries.iter_mut()
    }

    /// Every entry with its index, in order.
    pub fn iter_enumerated(&self) -> impl Iterator<Item = (I, &T)> {
        self.entries
            .iter()
            .enumerate()
            .map(|(i, entry)| (I::from_position(i), entry))
    }

    /// Every entry to change with its index, in order.
    pub fn iter_enumerated_mut(&mut self) -> impl Iterator<Item = (I, &mut T)> {
        self.entries
            .iter_mut()
            .enumerate()
            .map(|(i, entry)| (I::from_position(i), entry))
    }
}

impl<I: TableIndex + Send, T: Sync> IdVec<I, T> {
    /// Every entry with its index, in parallel.
    pub fn par_iter_enumerated(&self) -> impl rayon::iter::IndexedParallelIterator<Item = (I, &T)> {
        use rayon::prelude::*;
        self.entries
            .par_iter()
            .enumerate()
            .map(|(i, entry)| (I::from_position(i), entry))
    }
}

impl<I: TableIndex + Send, T: Send> IdVec<I, T> {
    /// Every entry to change with its index, in parallel.
    pub fn par_iter_enumerated_mut(
        &mut self,
    ) -> impl rayon::iter::IndexedParallelIterator<Item = (I, &mut T)> {
        use rayon::prelude::*;
        self.entries
            .par_iter_mut()
            .enumerate()
            .map(|(i, entry)| (I::from_position(i), entry))
    }
}

impl<I: TableIndex, T> Default for IdVec<I, T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<I, T: Clone> Clone for IdVec<I, T> {
    fn clone(&self) -> Self {
        Self {
            entries: self.entries.clone(),
            index: PhantomData,
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.entries.clone_from(&source.entries);
    }
}

impl<I, T: PartialEq> PartialEq for IdVec<I, T> {
    fn eq(&self, other: &Self) -> bool {
        self.entries == other.entries
    }
}

impl<I, T: Eq> Eq for IdVec<I, T> {}

impl<I, T: fmt::Debug> fmt::Debug for IdVec<I, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.entries.fmt(f)
    }
}

impl<I, T> From<Vec<T>> for IdVec<I, T> {
    fn from(entries: Vec<T>) -> Self {
        Self {
            entries,
            index: PhantomData,
        }
    }
}

impl<I, T> FromIterator<T> for IdVec<I, T> {
    fn from_iter<It: IntoIterator<Item = T>>(entries: It) -> Self {
        Vec::from_iter(entries).into()
    }
}

impl<'a, I, T> IntoIterator for &'a IdVec<I, T> {
    type Item = &'a T;
    type IntoIter = std::slice::Iter<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.entries.iter()
    }
}

impl<'a, I, T> IntoIterator for &'a mut IdVec<I, T> {
    type Item = &'a mut T;
    type IntoIter = std::slice::IterMut<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.entries.iter_mut()
    }
}

impl<I, T> IntoIterator for IdVec<I, T> {
    type Item = T;
    type IntoIter = std::vec::IntoIter<T>;

    fn into_iter(self) -> Self::IntoIter {
        self.entries.into_iter()
    }
}

impl<I: TableIndex, T> Index<I> for IdVec<I, T> {
    type Output = T;

    #[inline]
    fn index(&self, id: I) -> &T {
        &self.entries[id.position()]
    }
}

impl<I: TableIndex, T> IndexMut<I> for IdVec<I, T> {
    #[inline]
    fn index_mut(&mut self, id: I) -> &mut T {
        &mut self.entries[id.position()]
    }
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
    /// A special net in [`NetlistDB::special_nets`](crate::db::design::NetlistDB::special_nets).
    SpecialNetId(u32)
);
define_index!(
    /// A routing blockage in
    /// [`NetlistDB::routing_blockages`](crate::db::design::NetlistDB::routing_blockages).
    BlockageId(u32)
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
    /// A via generation rule in [`Library::via_rules`](crate::db::library::Library::via_rules).
    ViaRuleId(u32)
);
define_index!(
    /// A placement region in [`NetlistDB::regions`](crate::db::design::NetlistDB::regions).
    RegionId(u32)
);
define_index!(
    /// A non-default routing rule in
    /// [`NetlistDB::non_default_rules`](crate::db::design::NetlistDB::non_default_rules).
    NonDefaultRuleId(u32)
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

    /// The cut layer directly below this routing layer, if any.
    pub const fn cut_below(self) -> Option<CutLayerId> {
        match self.0.checked_sub(1) {
            Some(cut) => Some(CutLayerId(cut)),
            None => None,
        }
    }

    /// The routing layer directly below this one, if any.
    pub const fn below(self) -> Option<Self> {
        match self.0.checked_sub(1) {
            Some(layer) => Some(Self(layer)),
            None => None,
        }
    }

    /// The routing layer directly above this one.
    #[must_use]
    pub const fn above(self) -> Self {
        Self(self.0 + 1)
    }

    /// The routing layer one step from this one toward `target`, or `target`
    /// itself when this is it.
    #[must_use]
    pub const fn toward(self, target: Self) -> Self {
        if self.0 < target.0 {
            Self(self.0 + 1)
        } else if self.0 > target.0 {
            Self(self.0 - 1)
        } else {
            self
        }
    }

    /// Every routing layer from this one up to `top`, both included.
    pub fn through(self, top: Self) -> impl Iterator<Item = Self> + use<> {
        (self.0..=top.0).map(Self)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_push_returns_the_index_its_entry_is_found_at() {
        let mut names: IdVec<CellId, &str> = IdVec::new();

        let (a, b) = (names.push("a"), names.push("b"));

        assert_eq!((a, b), (CellId(0), CellId(1)));
        assert_eq!((names[a], names[b]), ("a", "b"));
    }

    #[test]
    fn the_next_id_is_the_one_the_next_push_returns() {
        let mut names: IdVec<NetId, &str> = vec!["n0"].into();

        let next = names.next_id();

        assert_eq!(names.push("n1"), next);
    }

    #[test]
    fn entries_enumerate_with_their_indices_in_order() {
        let names: IdVec<PinId, &str> = vec!["p0", "p1"].into();

        let listed: Vec<(PinId, &str)> = names.iter_enumerated().map(|(i, &n)| (i, n)).collect();

        assert_eq!(listed, vec![(PinId(0), "p0"), (PinId(1), "p1")]);
        assert_eq!(names.ids().collect::<Vec<_>>(), vec![PinId(0), PinId(1)]);
    }

    #[test]
    fn an_index_past_the_end_gets_nothing() {
        let names: IdVec<CellId, &str> = IdVec::filled("x", 2);

        assert_eq!(names.get(CellId(1)), Some(&"x"));
        assert_eq!(names.get(CellId(2)), None);
    }
}
