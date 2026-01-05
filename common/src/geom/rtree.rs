use super::rect::Rect;
use rstar::{AABB, RTree};

pub struct SpatialIndex {
    tree: RTree<IndexedRect>,
}

struct IndexedRect {
    rect: Rect,
    id: usize,
}

impl rstar::RTreeObject for IndexedRect {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [self.rect.min.x, self.rect.min.y],
            [self.rect.max.x, self.rect.max.y],
        )
    }
}

impl SpatialIndex {
    pub fn new() -> Self {
        Self { tree: RTree::new() }
    }

    pub fn insert(&mut self, rect: Rect, id: usize) {
        self.tree.insert(IndexedRect { rect, id });
    }

    pub fn query(&self, rect: Rect) -> Vec<usize> {
        let aabb = AABB::from_corners([rect.min.x, rect.min.y], [rect.max.x, rect.max.y]);
        self.tree
            .locate_in_envelope_intersecting(&aabb)
            .map(|item| item.id)
            .collect()
    }
}
