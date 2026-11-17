//! The per-node state of an A* search, kept between searches.

use crate::tech::index_u32;

/// The cost and parent of each node an A* search has reached. It is kept
/// between searches and cleared by bumping a generation stamp, so starting
/// a search costs nothing however many nodes the last one touched.
#[derive(Debug, Default)]
pub(super) struct StampedTable {
    cost: Vec<f64>,
    parent: Vec<u32>,
    stamp: Vec<u32>,
    generation: u32,
}

/// How the table stores a node without a parent.
const NO_PARENT: u32 = u32::MAX;

impl StampedTable {
    /// Forgets every node, sized for `nodes` nodes.
    pub(super) fn reset(&mut self, nodes: usize) {
        if self.stamp.len() < nodes {
            self.cost.resize(nodes, 0.0);
            self.parent.resize(nodes, NO_PARENT);
            self.stamp.resize(nodes, 0);
        }
        self.generation = self.generation.wrapping_add(1);
        if self.generation == 0 {
            self.stamp.fill(0);
            self.generation = 1;
        }
    }

    /// The cost `node` was reached at in this search; none before it is.
    pub(super) fn cost(&self, node: usize) -> Option<f64> {
        (self.stamp[node] == self.generation).then(|| self.cost[node])
    }

    /// Records `node` as reached at `cost` from `parent`, none at a root.
    pub(super) fn set(&mut self, node: usize, cost: f64, parent: Option<usize>) {
        self.stamp[node] = self.generation;
        self.cost[node] = cost;
        self.parent[node] = parent.map_or(NO_PARENT, index_u32);
    }

    /// The node `node` was last reached from; none at a root.
    pub(super) fn parent(&self, node: usize) -> Option<usize> {
        let parent = self.parent[node];
        (parent != NO_PARENT).then_some(parent as usize)
    }

    /// Lets go of the table when it holds more than `nodes` nodes.
    pub(super) fn release_beyond(&mut self, nodes: usize) {
        if self.stamp.len() > nodes {
            *self = Self::default();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_reset_forgets_every_node_it_had_reached() {
        let mut table = StampedTable::default();
        table.reset(4);
        table.set(2, 1.5, None);

        table.reset(4);

        assert_eq!(table.cost(2), None);
    }

    #[test]
    fn a_reached_node_keeps_its_cost_and_parent() {
        let mut table = StampedTable::default();
        table.reset(4);

        table.set(3, 2.0, Some(1));
        table.set(1, 0.0, None);

        assert_eq!((table.cost(3), table.parent(3)), (Some(2.0), Some(1)));
        assert_eq!(table.parent(1), None);
    }
}
