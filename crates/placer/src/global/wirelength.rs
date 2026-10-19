//! Weighted-average wirelength: a smooth stand-in for half-perimeter
//! wirelength whose error shrinks with the smoothing length `gamma`.

use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use std::ops::Range;

/// Where a net connects.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Terminal {
    /// At `offset` from the center of a placed object.
    Object {
        /// The object.
        index: usize,
        /// Offset from its center.
        offset: Point<f64>,
    },
    /// At a fixed point.
    Fixed(Point<f64>),
}

/// The nets global placement pulls on, with their terminals stored flat.
#[derive(Debug, Default)]
pub(super) struct Netlist {
    nets: Vec<(Range<usize>, f64)>,
    terminals: Vec<Terminal>,
    /// The box of each terminal's pin shapes, around the terminal.
    extents: Vec<Rect<f64>>,
}

/// Wirelength at one placement.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct Wirelength {
    /// The weighted-average estimate.
    pub(super) smooth: f64,
    /// Exact half-perimeter wirelength.
    pub(super) hpwl: f64,
}

impl Netlist {
    /// Adds a net of `pins`, each a terminal with the box of its pin's
    /// shapes around it; nets with fewer than two pins are left out.
    pub(super) fn push(
        &mut self,
        pins: impl IntoIterator<Item = (Terminal, Rect<f64>)>,
        weight: f64,
    ) {
        let start = self.terminals.len();
        for (terminal, extent) in pins {
            self.terminals.push(terminal);
            self.extents.push(extent);
        }
        if self.terminals.len() - start < 2 {
            self.terminals.truncate(start);
            self.extents.truncate(start);
            return;
        }
        self.nets.push((start..self.terminals.len(), weight));
    }

    /// Number of nets.
    pub(super) const fn len(&self) -> usize {
        self.nets.len()
    }

    /// Number of terminals on each of `objects` objects.
    pub(super) fn pin_counts(&self, objects: usize) -> Vec<f64> {
        let mut counts = vec![0.0; objects];
        for t in &self.terminals {
            if let Terminal::Object { index, .. } = *t {
                counts[index] += 1.0;
            }
        }
        counts
    }

    /// Each net's box over its pins' shapes at `centers`, as `OpenROAD`'s
    /// `dbNet::getTermBBox`.
    pub(super) fn term_boxes<'a>(
        &'a self,
        centers: &'a [Point<f64>],
    ) -> impl Iterator<Item = Rect<f64>> + 'a {
        self.nets.iter().filter_map(move |(range, _)| {
            self.terminals[range.clone()]
                .iter()
                .zip(&self.extents[range.clone()])
                .map(|(&t, e)| {
                    let at = position(t, centers);
                    Rect::new(at + e.min, at + e.max)
                })
                .reduce(|a, b| {
                    Rect::new(
                        Point::new(a.min.x.min(b.min.x), a.min.y.min(b.min.y)),
                        Point::new(a.max.x.max(b.max.x), a.max.y.max(b.max.y)),
                    )
                })
        })
    }

    /// Writes the gradient of the weighted-average wirelength with respect
    /// to each object's center into `grads`, using `scratch` for per-terminal
    /// gradients.
    pub(super) fn gradient(
        &self,
        centers: &[Point<f64>],
        gamma: f64,
        scratch: &mut Vec<Point<f64>>,
        grads: &mut [Point<f64>],
    ) -> Wirelength {
        scratch.resize(self.terminals.len(), Point::new(0.0, 0.0));
        let mut slices = Vec::with_capacity(self.nets.len());
        let mut rest = scratch.as_mut_slice();
        for (range, _) in &self.nets {
            let (head, tail) = rest.split_at_mut(range.len());
            slices.push(head);
            rest = tail;
        }
        let total = self
            .nets
            .par_iter()
            .zip(slices)
            .map_init(AxisBuffers::default, |buf, ((range, weight), out)| {
                let terminals = &self.terminals[range.clone()];
                let x = net_axis(terminals, centers, gamma, |p| p.x, buf);
                for (o, g) in out.iter_mut().zip(&buf.grads) {
                    o.x = g * weight;
                }
                let y = net_axis(terminals, centers, gamma, |p| p.y, buf);
                for (o, g) in out.iter_mut().zip(&buf.grads) {
                    o.y = g * weight;
                }
                Wirelength {
                    smooth: (x.smooth + y.smooth) * weight,
                    hpwl: (x.hpwl + y.hpwl) * weight,
                }
            })
            .collect::<Vec<Wirelength>>()
            .into_iter()
            .fold(Wirelength::default(), |a, b| Wirelength {
                smooth: a.smooth + b.smooth,
                hpwl: a.hpwl + b.hpwl,
            });

        grads.fill(Point::new(0.0, 0.0));
        for (t, g) in self.terminals.iter().zip(scratch.iter()) {
            if let Terminal::Object { index, .. } = *t {
                grads[index] += *g;
            }
        }
        total
    }
}

fn position(t: Terminal, centers: &[Point<f64>]) -> Point<f64> {
    match t {
        Terminal::Object { index, offset } => centers[index] + offset,
        Terminal::Fixed(p) => p,
    }
}

/// Per-thread space for one net's coordinates and derivatives on one axis.
#[derive(Debug, Default)]
struct AxisBuffers {
    coords: Vec<f64>,
    grads: Vec<f64>,
}

/// The weighted-average extent of one net along one axis, writing each
/// terminal's derivative into `buf.grads`.
fn net_axis(
    terminals: &[Terminal],
    centers: &[Point<f64>],
    gamma: f64,
    axis: impl Fn(Point<f64>) -> f64,
    buf: &mut AxisBuffers,
) -> Wirelength {
    let AxisBuffers { coords, grads } = buf;
    coords.clear();
    coords.extend(terminals.iter().map(|&t| axis(position(t, centers))));
    let max = coords.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min = coords.iter().copied().fold(f64::INFINITY, f64::min);

    let (mut sum_hi, mut weighted_hi, mut sum_lo, mut weighted_lo) = (0.0, 0.0, 0.0, 0.0);
    for &c in coords.iter() {
        let hi = ((c - max) / gamma).exp();
        let lo = ((min - c) / gamma).exp();
        sum_hi += hi;
        weighted_hi = c.mul_add(hi, weighted_hi);
        sum_lo += lo;
        weighted_lo = c.mul_add(lo, weighted_lo);
    }
    let wa_hi = weighted_hi / sum_hi;
    let wa_lo = weighted_lo / sum_lo;

    grads.clear();
    grads.extend(coords.iter().map(|&c| {
        let hi = ((c - max) / gamma).exp() / sum_hi;
        let lo = ((min - c) / gamma).exp() / sum_lo;
        let rise = (c - wa_hi) / gamma;
        let fall = (c - wa_lo) / gamma;
        hi.mul_add(1.0 + rise, -(lo * (1.0 - fall)))
    }));
    Wirelength {
        smooth: wa_hi - wa_lo,
        hpwl: max - min,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn object(index: usize) -> (Terminal, Rect<f64>) {
        point(Terminal::Object {
            index,
            offset: Point::new(0.0, 0.0),
        })
    }

    fn point(terminal: Terminal) -> (Terminal, Rect<f64>) {
        let origin = Point::new(0.0, 0.0);
        (terminal, Rect::new(origin, origin))
    }

    #[test]
    fn a_small_gamma_approaches_half_perimeter_wirelength() {
        let mut netlist = Netlist::default();
        netlist.push(
            [
                object(0),
                object(1),
                point(Terminal::Fixed(Point::new(5.0, 1.0))),
            ],
            1.0,
        );
        let centers = [Point::new(0.0, 0.0), Point::new(3.0, 4.0)];
        let mut grads = [Point::new(0.0, 0.0); 2];

        let wl = netlist.gradient(&centers, 0.01, &mut Vec::new(), &mut grads);

        assert!((wl.hpwl - 9.0).abs() < 1e-12);
        assert!((wl.smooth - 9.0).abs() < 1e-6);
    }

    #[test]
    fn the_gradient_matches_a_finite_difference() {
        let mut netlist = Netlist::default();
        netlist.push([object(0), object(1), object(2)], 2.0);
        let centers = [
            Point::new(0.0, 1.0),
            Point::new(2.0, 3.0),
            Point::new(1.5, -1.0),
        ];
        let mut grads = [Point::new(0.0, 0.0); 3];
        let gamma = 0.7;
        let _ = netlist.gradient(&centers, gamma, &mut Vec::new(), &mut grads);

        let h = 1e-6;
        let mut moved = centers;
        moved[2].x += h;
        let mut ignored = [Point::new(0.0, 0.0); 3];
        let up = netlist.gradient(&moved, gamma, &mut Vec::new(), &mut ignored);
        moved[2].x -= h;
        moved[2].x -= h;
        let down = netlist.gradient(&moved, gamma, &mut Vec::new(), &mut ignored);

        let numeric = (up.smooth - down.smooth) / (2.0 * h);
        assert!(
            (grads[2].x - numeric).abs() < 1e-5,
            "{} vs {numeric}",
            grads[2].x
        );
    }

    #[test]
    fn single_terminal_nets_are_dropped() {
        let mut netlist = Netlist::default();

        netlist.push([object(0)], 1.0);

        assert!(netlist.nets.is_empty() && netlist.terminals.is_empty());
    }
}
