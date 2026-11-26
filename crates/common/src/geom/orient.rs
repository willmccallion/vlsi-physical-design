//! LEF/DEF placement orientations.

use super::dbu::Dbu;
use super::point::Point;
use super::rect::Rect;
use std::fmt;
use std::str::FromStr;

crate::keyword_enum! {
    /// One of the eight DEF orientations.
    ///
    /// `W`/`S`/`E` rotate counter-clockwise by 90/180/270 degrees; the `F` forms
    /// mirror the corresponding rotation about the Y axis.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    pub enum Orientation {
        /// No rotation.
        #[default]
        N = "N",
        /// Rotated 180 degrees.
        S = "S",
        /// Rotated 270 degrees counter-clockwise.
        E = "E",
        /// Rotated 90 degrees counter-clockwise.
        W = "W",
        /// Mirrored about the Y axis.
        FN = "FN",
        /// Mirrored about the X axis.
        FS = "FS",
        /// `E`, then mirrored about the Y axis.
        FE = "FE",
        /// `W`, then mirrored about the Y axis.
        FW = "FW",
    }
}

impl Orientation {
    /// The orientation mirrored about the vertical axis, as `OpenDP`'s
    /// `orientMirrorY`.
    #[must_use]
    pub const fn mirrored_about_y(self) -> Self {
        match self {
            Self::N => Self::FN,
            Self::FN => Self::N,
            Self::S => Self::FS,
            Self::FS => Self::S,
            Self::E => Self::FE,
            Self::FE => Self::E,
            Self::W => Self::FW,
            Self::FW => Self::W,
        }
    }

    /// Whether the orientation swaps width and height.
    pub(crate) const fn is_rotated_quarter(self) -> bool {
        matches!(self, Self::E | Self::W | Self::FE | Self::FW)
    }

    /// Rotates or mirrors a point about the origin.
    pub const fn apply(self, p: Point<Dbu>) -> Point<Dbu> {
        let (x, y) = (p.x.get(), p.y.get());
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
        Point::new(Dbu::new(tx), Dbu::new(ty))
    }

    /// Rotates or mirrors a rectangle about the origin.
    pub(crate) fn apply_rect(self, r: &Rect<Dbu>) -> Rect<Dbu> {
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
    pub(crate) fn place_point(
        self,
        p: Point<Dbu>,
        size: Point<Dbu>,
        location: Point<Dbu>,
    ) -> Point<Dbu> {
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
        Self::from_keyword(s).ok_or_else(|| UnknownOrientation(s.to_string()))
    }
}

#[cfg(test)]
mod tests;
