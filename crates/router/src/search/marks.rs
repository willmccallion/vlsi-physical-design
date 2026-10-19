//! Where violations were found lately, as `TritonRoute` keeps marker cost
//! on its grid graph: a counter on each crossing and each via of the track
//! graph that a violation marked, raised by every violation marking it and
//! decayed after every net a worker routes. A step through a place whose
//! counter is above zero pays the round's marker cost, however high the
//! counter is.

use crate::access::Node;
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
                *count = (f64::from(*count) * factor) as u8;
                *count > 0
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::indices::LayerId;

    fn node(cross: u32) -> Node {
        Node {
            layer: LayerId::new(1),
            track: 4,
            cross,
        }
    }

    #[test]
    fn one_violation_fades_after_ten_decays_at_the_early_rate() {
        let mut marks = Marks::default();
        marks.mark_planar(node(3));

        for _ in 0..9 {
            marks.decay(0.95);
        }
        let after_nine = marks.planar(node(3));
        marks.decay(0.95);

        assert!(after_nine);
        assert!(!marks.planar(node(3)));
    }

    #[test]
    fn two_violations_on_one_place_last_longer_than_one() {
        let mut once = Marks::default();
        once.mark_via(node(3));
        let mut twice = once.clone();
        twice.mark_via(node(3));

        for _ in 0..10 {
            once.decay(0.95);
            twice.decay(0.95);
        }

        assert!(!once.via(node(3)));
        assert!(twice.via(node(3)));
    }

    #[test]
    fn a_counter_stops_at_its_largest_value() {
        let mut marks = Marks::default();

        for _ in 0..40 {
            marks.mark_planar(node(0));
        }

        assert_eq!(marks.planar.get(&node(0)), Some(&u8::MAX));
    }

    #[test]
    fn crossings_and_vias_are_marked_apart() {
        let mut marks = Marks::default();

        marks.mark_planar(node(2));

        assert!(marks.planar(node(2)));
        assert!(!marks.via(node(2)));
    }
}
