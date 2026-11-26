//! Where violations were found lately, as `TritonRoute` keeps marker cost
//! on its grid graph: a counter on each crossing and each via of the track
//! graph that a violation marked, raised by every violation marking it and
//! decayed after every net a worker routes. A step through a place whose
//! counter is above zero pays the round's marker cost, however high the
//! counter is.

use crate::access::Node;
use pare_common::convert::SaturatingFrom;
use rustc_hash::FxHashMap;

/// What one violation adds to the counter of each place it marks, as
/// `TritonRoute`'s `addMarkerCost`.
const MARK: u8 = 10;

/// The marker counters of one worker.
#[derive(Clone, Debug, Default)]
pub(crate) struct Marks {
    /// Crossings, by node.
    planar: FxHashMap<Node, u8>,
    /// Vias, by the node they rise from.
    vias: FxHashMap<Node, u8>,
}

impl Marks {
    /// Marks the crossing `node`.
    pub(crate) fn mark_planar(&mut self, node: Node) {
        let count = self.planar.entry(node).or_insert(0);
        *count = count.saturating_add(MARK);
    }

    /// Marks the via rising from `lower`.
    pub(crate) fn mark_via(&mut self, lower: Node) {
        let count = self.vias.entry(lower).or_insert(0);
        *count = count.saturating_add(MARK);
    }

    /// Whether the crossing `node` is marked.
    pub(crate) fn planar(&self, node: Node) -> bool {
        self.planar.contains_key(&node)
    }

    /// Whether the via rising from `lower` is marked.
    pub(crate) fn via(&self, lower: Node) -> bool {
        self.vias.contains_key(&lower)
    }

    /// Scales every counter by `factor`, rounding down as `TritonRoute`'s
    /// integer counters do, and forgets those that reach zero.
    pub(crate) fn decay(&mut self, factor: f64) {
        for counts in [&mut self.planar, &mut self.vias] {
            counts.retain(|_, count| {
                *count = u8::saturating_from(f64::from(*count) * factor);
                *count > 0
            });
        }
    }
}

#[cfg(test)]
mod tests;
