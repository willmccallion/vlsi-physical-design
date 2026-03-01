//! R-Tree Spatial Index for Efficient Range Queries.
//!
//! Provides a spatial index structure based on R-trees for fast lookup of
//! rectangles that overlap with a query region. Used for collision detection
//! and spatial queries in placement algorithms.

use super::rect::Rect;
use rstar::{AABB, RTree};

/// Spatial index structure for efficient rectangle overlap queries.
///
/// Maintains an R-tree of indexed rectangles, allowing logarithmic-time
/// queries for all rectangles that intersect a given query region. This
/// is used during placement to quickly find cells that may overlap with
/// a given region without checking every cell in the design.
pub struct SpatialIndex {
    tree: RTree<IndexedRect>,
}

/// A rectangle with an associated identifier for indexing.
///
/// Wraps a rectangle with an integer ID so that query results can identify
/// which rectangle matched the query. This is used to map query results
/// back to cell indices or other entities in the design.
struct IndexedRect {
    rect: Rect,
    id: usize,
}

impl rstar::RTreeObject for IndexedRect {
    /// The envelope type used by the R-tree.
    type Envelope = AABB<[f64; 2]>;

    /// Returns the axis-aligned bounding box for this rectangle.
    ///
    /// Required by the rstar library to enable spatial indexing. The envelope
    /// is computed from the rectangle's min and max corners.
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [self.rect.min.x, self.rect.min.y],
            [self.rect.max.x, self.rect.max.y],
        )
    }
}

impl Default for SpatialIndex {
    fn default() -> Self {
        Self::new()
    }
}

impl SpatialIndex {
    /// Creates a new empty spatial index.
    ///
    /// Initializes an R-tree with no entries. Rectangles can be added
    /// incrementally using the insert method.
    pub fn new() -> Self {
        Self { tree: RTree::new() }
    }

    /// Inserts a rectangle into the spatial index with the given identifier.
    ///
    /// Adds the rectangle to the R-tree so it can be found by subsequent
    /// queries. The ID is stored with the rectangle and returned in query
    /// results to identify which rectangle matched.
    pub fn insert(&mut self, rect: Rect, id: usize) {
        self.tree.insert(IndexedRect { rect, id });
    }

    /// Queries the spatial index for all rectangles that overlap the query region.
    ///
    /// Returns a vector of IDs for all indexed rectangles that intersect
    /// the given query rectangle. The query is performed efficiently using
    /// the R-tree structure, avoiding linear scans through all rectangles.
    pub fn query(&self, rect: Rect) -> Vec<usize> {
        let aabb = AABB::from_corners([rect.min.x, rect.min.y], [rect.max.x, rect.max.y]);
        self.tree
            .locate_in_envelope_intersecting(&aabb)
            .map(|item| item.id)
            .collect()
    }
}
