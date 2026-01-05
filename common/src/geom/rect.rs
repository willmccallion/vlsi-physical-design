use super::point::Point;

#[derive(Clone, Copy, Debug, Default)]
pub struct Rect {
    pub min: Point<f64>,
    pub max: Point<f64>,
}

impl Rect {
    pub fn new(min: Point<f64>, max: Point<f64>) -> Self {
        Self { min, max }
    }

    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }
    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }
    pub fn area(&self) -> f64 {
        self.width() * self.height()
    }

    pub fn overlaps(&self, other: &Rect) -> bool {
        self.min.x < other.max.x
            && self.max.x > other.min.x
            && self.min.y < other.max.y
            && self.max.y > other.min.y
    }

    pub fn contains(&self, p: Point<f64>) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }
}
