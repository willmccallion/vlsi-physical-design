#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GridCoord {
    pub x: u32,
    pub y: u32,
    pub z: u8,
}

impl GridCoord {
    pub fn new(x: u32, y: u32, z: u8) -> Self {
        Self { x, y, z }
    }
}
