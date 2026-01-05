use eda_common::geom::coord::GridCoord;
use eda_common::geom::point::Point;

pub struct GridConverter {
    scale_x: f64,
    scale_y: f64,
    offset_x: f64,
    offset_y: f64,
    grid_w: u32,
    grid_h: u32,
}

impl GridConverter {
    pub fn new(die_w: f64, die_h: f64, grid_w: u32, grid_h: u32) -> Self {
        Self {
            scale_x: grid_w as f64 / die_w,
            scale_y: grid_h as f64 / die_h,
            offset_x: 0.0,
            offset_y: 0.0,
            grid_w,
            grid_h,
        }
    }

    pub fn from_steps(
        step_x: f64,
        step_y: f64,
        off_x: f64,
        off_y: f64,
        grid_w: u32,
        grid_h: u32,
    ) -> Self {
        Self {
            scale_x: 1.0 / step_x,
            scale_y: 1.0 / step_y,
            offset_x: off_x,
            offset_y: off_y,
            grid_w,
            grid_h,
        }
    }

    pub fn to_grid(&self, p: Point<f64>, layer: u8) -> GridCoord {
        let raw_x = (p.x - self.offset_x) * self.scale_x;
        let raw_y = (p.y - self.offset_y) * self.scale_y;

        let x = raw_x.round().max(0.0).min((self.grid_w - 1) as f64) as u32;
        let y = raw_y.round().max(0.0).min((self.grid_h - 1) as f64) as u32;

        GridCoord::new(x, y, layer)
    }

    pub fn to_world(&self, g: GridCoord) -> Point<f64> {
        Point::new(
            (g.x as f64 / self.scale_x) + self.offset_x,
            (g.y as f64 / self.scale_y) + self.offset_y,
        )
    }
}
