pub mod dense;

pub use dense::DenseGrid;

use eda_common::geom::coord::GridCoord;

pub trait RoutingGrid: Sync + Send {
    fn width(&self) -> u32;
    fn height(&self) -> u32;
    fn layers(&self) -> u8;

    fn set_obstacle(&mut self, coord: GridCoord);
    fn clear_obstacle(&mut self, coord: GridCoord);
    fn is_obstacle(&self, coord: GridCoord) -> bool;

    fn add_wire(&mut self, coord: GridCoord);
    fn remove_wire(&mut self, coord: GridCoord);

    fn get_cost(&self, coord: GridCoord, collision_penalty: f64) -> f64;
    fn update_history(&mut self, history_increment: f64);
    fn decay_history(&mut self, decay_factor: f64);

    fn max_occupancy(&self) -> u32;
    fn is_congested(&self, coord: GridCoord) -> bool;
    fn total_conflicts(&self) -> usize;

    fn capacity(&self, coord: GridCoord) -> u32;
    fn set_penalty(&mut self, penalty: f64);
}
