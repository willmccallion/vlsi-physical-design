//! LEF/DEF placement orientations.

use super::dbu::Dbu;
use super::point::Point;
use super::rect::Rect;
use std::fmt;
use std::str::FromStr;

/// One of the eight DEF orientations.
///
/// `W`/`S`/`E` rotate counter-clockwise by 90/180/270 degrees; the `F` forms
/// mirror the corresponding rotation about the Y axis.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Orientation {
    /// No rotation.
    #[default]
    N,
    /// Rotated 180 degrees.
    S,
    /// Rotated 270 degrees counter-clockwise.
    E,
    /// Rotated 90 degrees counter-clockwise.
    W,
    /// Mirrored about the Y axis.
    FN,
    /// Mirrored about the X axis.
    FS,
    /// `E`, then mirrored about the Y axis.
    FE,
    /// `W`, then mirrored about the Y axis.
    FW,
}

impl Orientation {
    /// All eight orientations.
    pub const ALL: [Self; 8] = [
        Self::N,
        Self::S,
        Self::E,
        Self::W,
        Self::FN,
        Self::FS,
        Self::FE,
        Self::FW,
    ];

    /// The DEF keyword for this orientation.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::N => "N",
            Self::S => "S",
            Self::E => "E",
            Self::W => "W",
            Self::FN => "FN",
            Self::FS => "FS",
            Self::FE => "FE",
            Self::FW => "FW",
        }
    }

    /// Whether the orientation swaps width and height.
    pub const fn is_rotated_quarter(self) -> bool {
        matches!(self, Self::E | Self::W | Self::FE | Self::FW)
    }

    /// Rotates or mirrors a point about the origin.
    pub const fn apply(self, p: Point<Dbu>) -> Point<Dbu> {
        let (x, y) = (p.x.0, p.y.0);
        let (tx, ty) = match self {
            Self::N => (x, y),
            Self::W => (-y, x),
            Self::S => (-x, -y),
            Self::E => (y, -x),
            Self::FN => (-x, y),
            Self::FW => (y, x),
            Self::FS => (x, -y),
            Self::FE => (-y, -x),
        };
        Point::new(Dbu(tx), Dbu(ty))
    }

    /// Rotates or mirrors a rectangle about the origin.
    pub fn apply_rect(self, r: &Rect<Dbu>) -> Rect<Dbu> {
        Rect::from_corners(self.apply(r.min), self.apply(r.max))
    }

    /// Size of a `width` x `height` box after orientation.
    pub const fn oriented_size(self, size: Point<Dbu>) -> Point<Dbu> {
        if self.is_rotated_quarter() {
            Point::new(size.y, size.x)
        } else {
            size
        }
    }

    /// Maps a rectangle in macro coordinates to placed coordinates.
    ///
    /// DEF places a component so that the lower-left corner of its oriented
    /// bounding box sits at `location`.
    pub fn place_rect(self, r: &Rect<Dbu>, size: Point<Dbu>, location: Point<Dbu>) -> Rect<Dbu> {
        let body = self.apply_rect(&Rect::new(Point::default(), size));
        self.apply_rect(r).translate(location - body.min)
    }

    /// Maps a point in macro coordinates to placed coordinates.
    pub fn place_point(self, p: Point<Dbu>, size: Point<Dbu>, location: Point<Dbu>) -> Point<Dbu> {
        let body = self.apply_rect(&Rect::new(Point::default(), size));
        self.apply(p) + location - body.min
    }
}

impl fmt::Display for Orientation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A string that is not a DEF orientation.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("unknown orientation '{0}'")]
pub struct UnknownOrientation(pub String);

impl FromStr for Orientation {
    type Err = UnknownOrientation;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|o| o.as_str() == s)
            .ok_or_else(|| UnknownOrientation(s.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: i64, y: i64) -> Point<Dbu> {
        Point::new(Dbu(x), Dbu(y))
    }

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::from_corners(pt(x0, y0), pt(x1, y1))
    }

    const SIZE: Point<Dbu> = Point::new(Dbu(10), Dbu(4));
    const AT: Point<Dbu> = Point::new(Dbu(100), Dbu(200));
    const PIN: Point<Dbu> = Point::new(Dbu(1), Dbu(1));

    #[test]
    fn every_orientation_keeps_the_body_at_the_location() {
        for o in Orientation::ALL {
            let body = o.place_rect(&rect(0, 0, 10, 4), SIZE, AT);
            let size = o.oriented_size(SIZE);

            assert_eq!(body, Rect::new(AT, AT + size), "{o}");
        }
    }

    #[test]
    fn pin_near_the_lower_left_corner_lands_where_each_orientation_puts_it() {
        let expected = [
            (Orientation::N, pt(101, 201)),
            (Orientation::S, pt(109, 203)),
            (Orientation::FN, pt(109, 201)),
            (Orientation::FS, pt(101, 203)),
            (Orientation::W, pt(103, 201)),
            (Orientation::E, pt(101, 209)),
            (Orientation::FW, pt(101, 201)),
            (Orientation::FE, pt(103, 209)),
        ];

        for (o, want) in expected {
            assert_eq!(o.place_point(PIN, SIZE, AT), want, "{o}");
        }
    }

    #[test]
    fn orientations_parse_from_their_def_names() {
        for o in Orientation::ALL {
            assert_eq!(o.as_str().parse::<Orientation>(), Ok(o));
        }
        assert!("NE".parse::<Orientation>().is_err());
    }
}
