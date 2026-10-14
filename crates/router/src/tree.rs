//! The order in which a net's pins are joined into a tree.
//!
//! Every pin after the first is joined to the nearest pin joined before it,
//! which makes each connection short and local. Small nets use exact Prim
//! ordering; large ones take pins along a Hilbert curve and find the nearest
//! earlier pin through a bucket grid, which keeps nets with tens of
//! thousands of pins fast.

use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use rustc_hash::FxHashMap;

/// Nets up to this size are ordered by exact Prim.
const PRIM_LIMIT: usize = 256;

/// One step of building a tree: join `pin` to the already joined `parent`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Join {
    /// The pin to connect.
    pub pin: usize,
    /// The nearest pin connected before it.
    pub parent: usize,
}

/// The first pin and the joins that connect the rest, in order.
pub fn connection_order(points: &[Point<Dbu>]) -> (usize, Vec<Join>) {
    if points.len() <= PRIM_LIMIT {
        prim(points)
    } else {
        curve_order(points)
    }
}

fn manhattan(a: Point<Dbu>, b: Point<Dbu>) -> i64 {
    ((a.x - b.x).abs() + (a.y - b.y).abs()).get()
}

fn prim(points: &[Point<Dbu>]) -> (usize, Vec<Join>) {
    let n = points.len();
    let mut joined = vec![false; n];
    let mut best: Vec<(i64, usize)> = vec![(i64::MAX, 0); n];
    let mut joins = Vec::with_capacity(n.saturating_sub(1));
    let mut current = 0;
    joined[0] = true;
    for _ in 1..n {
        for (i, b) in best.iter_mut().enumerate() {
            if !joined[i] {
                let d = manhattan(points[i], points[current]);
                if d < b.0 {
                    *b = (d, current);
                }
            }
        }
        let Some(next) = (0..n)
            .filter(|&i| !joined[i])
            .min_by_key(|&i| (best[i].0, i))
        else {
            break;
        };
        joined[next] = true;
        joins.push(Join {
            pin: next,
            parent: best[next].1,
        });
        current = next;
    }
    (0, joins)
}

/// Position of a point along a Hilbert curve over a 2^16 grid.
fn hilbert(mut x: u32, mut y: u32) -> u64 {
    let mut d = 0u64;
    let mut s = 1u32 << 15;
    while s > 0 {
        let rx = u32::from(x & s > 0);
        let ry = u32::from(y & s > 0);
        d += u64::from(s) * u64::from(s) * u64::from((3 * rx) ^ ry);
        if ry == 0 {
            if rx == 1 {
                x = 0xffff - x;
                y = 0xffff - y;
            }
            std::mem::swap(&mut x, &mut y);
        }
        s /= 2;
    }
    d
}

fn curve_order(points: &[Point<Dbu>]) -> (usize, Vec<Join>) {
    let min_x = points.iter().map(|p| p.x).min().unwrap_or_default();
    let min_y = points.iter().map(|p| p.y).min().unwrap_or_default();
    let max_x = points.iter().map(|p| p.x).max().unwrap_or_default();
    let max_y = points.iter().map(|p| p.y).max().unwrap_or_default();
    let span = (max_x - min_x).max(max_y - min_y).get().max(1);
    let scale = |v: Dbu, lo: Dbu| u32::try_from((v - lo).get() * 65_535 / span).unwrap_or(0);
    let mut order: Vec<usize> = (0..points.len()).collect();
    order.sort_by_key(|&i| {
        (
            hilbert(scale(points[i].x, min_x), scale(points[i].y, min_y)),
            i,
        )
    });

    let cells_per_side = (points.len() as f64).sqrt().ceil().max(1.0) as i64;
    let bucket = (span / cells_per_side).max(1);
    let key = |p: Point<Dbu>| ((p.x - min_x).get() / bucket, (p.y - min_y).get() / bucket);
    let mut buckets: FxHashMap<(i64, i64), Vec<usize>> = FxHashMap::default();

    let first = order[0];
    buckets.entry(key(points[first])).or_default().push(first);
    let mut joins = Vec::with_capacity(points.len() - 1);
    for &i in &order[1..] {
        let p = points[i];
        let (cx, cy) = key(p);
        let mut nearest: Option<(i64, usize)> = None;
        let mut ring = 0;
        loop {
            for bx in cx - ring..=cx + ring {
                for by in cy - ring..=cy + ring {
                    if (bx - cx).abs() != ring && (by - cy).abs() != ring {
                        continue;
                    }
                    for &j in buckets.get(&(bx, by)).into_iter().flatten() {
                        let d = manhattan(p, points[j]);
                        if nearest.is_none_or(|(b, k)| (d, j) < (b, k)) {
                            nearest = Some((d, j));
                        }
                    }
                }
            }
            // Anything beyond this ring is at least `ring * bucket` away.
            if nearest.is_some_and(|(d, _)| d <= ring * bucket) || ring > cells_per_side + 1 {
                break;
            }
            ring += 1;
        }
        if let Some((_, parent)) = nearest {
            joins.push(Join { pin: i, parent });
        }
        buckets.entry((cx, cy)).or_default().push(i);
    }
    (first, joins)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: i64, y: i64) -> Point<Dbu> {
        Point::new(Dbu(x), Dbu(y))
    }

    #[test]
    fn prim_joins_each_pin_to_its_nearest_joined_pin() {
        let points = [pt(0, 0), pt(100, 0), pt(10, 0), pt(110, 5)];

        let (first, joins) = connection_order(&points);

        assert_eq!(first, 0);
        assert_eq!(
            joins,
            [
                Join { pin: 2, parent: 0 },
                Join { pin: 1, parent: 2 },
                Join { pin: 3, parent: 1 },
            ]
        );
    }

    #[test]
    fn large_nets_join_every_pin_to_its_nearest_earlier_pin() {
        let points: Vec<Point<Dbu>> = (0..1000)
            .map(|i| pt((i * 37 % 400) * 10, (i * 91 % 250) * 10))
            .collect();

        let (first, joins) = connection_order(&points);

        assert_eq!(joins.len(), 999);
        let mut earlier = vec![first];
        for j in joins {
            let nearest = earlier
                .iter()
                .map(|&e| manhattan(points[j.pin], points[e]))
                .min()
                .unwrap();
            assert!(
                earlier.contains(&j.parent),
                "{j:?} joined before its parent"
            );
            assert_eq!(manhattan(points[j.pin], points[j.parent]), nearest, "{j:?}");
            earlier.push(j.pin);
        }
    }

    #[test]
    fn the_hilbert_curve_walks_a_corner_block_one_neighbour_at_a_time() {
        let mut cells: Vec<(u64, u32, u32)> = (0..8)
            .flat_map(|x| (0..8).map(move |y| (hilbert(x, y), x, y)))
            .collect();
        cells.sort_unstable();

        let positions: Vec<u64> = cells.iter().map(|c| c.0).collect();
        assert_eq!(positions, (0..64).collect::<Vec<u64>>());
        for w in cells.windows(2) {
            assert_eq!(
                w[0].1.abs_diff(w[1].1) + w[0].2.abs_diff(w[1].2),
                1,
                "{w:?}"
            );
        }
    }
}
