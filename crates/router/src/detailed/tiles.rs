//! The repair of the last few violations, as `TritonRoute` does it
//! (`guideTilesFlow` and `stubbornTilesFlow` in `OpenROAD`'s
//! `src/drt/src/dr/FlexDR.cpp`): workers over the global routes of the
//! nets in each violation, and workers on tiles around clusters of
//! violations, each tile tried with several costs and the best result kept.
//! A worker rips its nets up in its own view of the wiring, so the workers
//! of one cluster run side by side; only the one kept changes the
//! committed wiring.

use super::region::{Changes, RepairStats, Routed, Scope, Verdict};
use super::{DetailedRouter, NetGuide};
use crate::search::Costs;
use pare_common::db::indices::NetId;
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use rustc_hash::FxHashMap;
use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap};

/// Largest span, in gcells past the first, of a cluster of violations.
const CLUSTER_SPAN: i64 = 4;

/// Span, in gcells past the first, of a tile around a cluster: tiles are
/// seven gcells wide.
const TILE_SPAN: i64 = 6;

/// Factors the conflict and marker costs of a stubborn tile are tried at.
const COST_FACTORS: [f64; 3] = [1.0, 0.5, 2.0];

/// A box of gcells by index, both ends included.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct CellBox {
    x0: i64,
    y0: i64,
    x1: i64,
    y1: i64,
}

impl CellBox {
    const fn merge(self, other: Self) -> Self {
        Self {
            x0: if self.x0 < other.x0 {
                self.x0
            } else {
                other.x0
            },
            y0: if self.y0 < other.y0 {
                self.y0
            } else {
                other.y0
            },
            x1: if self.x1 > other.x1 {
                self.x1
            } else {
                other.x1
            },
            y1: if self.y1 > other.y1 {
                self.y1
            } else {
                other.y1
            },
        }
    }

    const fn dx(self) -> i64 {
        self.x1 - self.x0
    }

    const fn dy(self) -> i64 {
        self.y1 - self.y0
    }

    fn cells(self) -> impl Iterator<Item = (i64, i64)> {
        (self.x0..=self.x1).flat_map(move |x| (self.y0..=self.y1).map(move |y| (x, y)))
    }
}

/// Clusters violation boxes greedily, merging any two whose merged box
/// spans at most [`CLUSTER_SPAN`] gcells past the first each way, as
/// `TritonRoute`'s `mergeBoxes`.
fn merge_boxes(mut boxes: Vec<CellBox>) -> Vec<CellBox> {
    'merging: loop {
        for i in 0..boxes.len() {
            for j in i + 1..boxes.len() {
                let merged = boxes[i].merge(boxes[j]);
                if merged.dx() <= CLUSTER_SPAN && merged.dy() <= CLUSTER_SPAN {
                    boxes[i] = merged;
                    let _ = boxes.remove(j);
                    continue 'merging;
                }
            }
        }
        return boxes;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Side {
    East,
    West,
    North,
    South,
}

/// Which cluster each gcell taken by a tile belongs to.
struct Taken {
    owner: FxHashMap<(i64, i64), usize>,
    /// The gcells of the die.
    limit: CellBox,
}

impl Taken {
    fn take(&mut self, b: CellBox, id: usize) {
        for cell in b.cells() {
            let _ = self.owner.insert(cell, id);
        }
    }

    fn taken_by_other(&self, b: CellBox, id: usize) -> bool {
        b.cells()
            .any(|cell| self.owner.get(&cell).is_some_and(|&o| o != id))
    }

    /// Grows `b` toward `side` one gcell at a time, at most `most`, while
    /// it stays on the die and off other clusters' tiles; returns how far
    /// it grew.
    fn grow(&self, b: &mut CellBox, id: usize, side: Side, most: i64) -> i64 {
        for step in 1..=most {
            let mut grown = *b;
            match side {
                Side::East => grown.x1 += 1,
                Side::West => grown.x0 -= 1,
                Side::North => grown.y1 += 1,
                Side::South => grown.y0 -= 1,
            }
            let off_die = grown.x0 < self.limit.x0
                || grown.y0 < self.limit.y0
                || grown.x1 > self.limit.x1
                || grown.y1 > self.limit.y1;
            if off_die || self.taken_by_other(grown, id) {
                return step - 1;
            }
            *b = grown;
        }
        most
    }
}

/// One way of growing a cluster into a tile: how far toward each side.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Growth {
    east: i64,
    west: i64,
    north: i64,
    south: i64,
}

/// Grows each cluster into tiles of up to [`TILE_SPAN`] gcells past the
/// first, as `TritonRoute`'s `expandBoxes`: one with the cluster in the
/// middle, then up to four with it at the west, east, south and north
/// side, the clusters with the fewest tiles growing first, no tile
/// reaching onto another cluster's.
fn expand_boxes(clusters: &[CellBox], limit: CellBox) -> Vec<BTreeSet<CellBox>> {
    let mut taken = Taken {
        owner: FxHashMap::default(),
        limit,
    };
    for (id, &b) in clusters.iter().enumerate() {
        taken.take(b, id);
    }
    let mut tiles: Vec<BTreeSet<CellBox>> = Vec::with_capacity(clusters.len());
    let mut queue: BinaryHeap<Reverse<(usize, usize, usize, Growth)>> = BinaryHeap::new();
    let mut order = 0;
    for (id, &cluster) in clusters.iter().enumerate() {
        let mut centred = cluster;
        let wide = TILE_SPAN - cluster.dx();
        let tall = TILE_SPAN - cluster.dy();
        let east = taken.grow(&mut centred, id, Side::East, wide / 2);
        let west = taken.grow(&mut centred, id, Side::West, wide - east);
        let north = taken.grow(&mut centred, id, Side::North, tall / 2);
        let south = taken.grow(&mut centred, id, Side::South, tall - north);
        let sided = [
            Growth {
                east: 0,
                west: wide,
                north,
                south,
            },
            Growth {
                east: wide,
                west: 0,
                north,
                south,
            },
            Growth {
                east,
                west,
                north: 0,
                south: tall,
            },
            Growth {
                east,
                west,
                north: tall,
                south: 0,
            },
        ];
        for growth in sided {
            queue.push(Reverse((1, order, id, growth)));
            order += 1;
        }
        tiles.push(BTreeSet::from([centred]));
        taken.take(centred, id);
    }
    while let Some(Reverse((done, at, id, growth))) = queue.pop() {
        if tiles[id].len() != done {
            queue.push(Reverse((tiles[id].len(), at, id, growth)));
            continue;
        }
        let mut tile = clusters[id];
        let _ = taken.grow(&mut tile, id, Side::East, growth.east);
        let _ = taken.grow(&mut tile, id, Side::West, growth.west);
        let _ = taken.grow(&mut tile, id, Side::North, growth.north);
        let _ = taken.grow(&mut tile, id, Side::South, growth.south);
        let _ = tiles[id].insert(tile);
        taken.take(tile, id);
    }
    tiles
}

/// Splits workers into batches whose `reaches` do not touch, each worker
/// going into the first batch it fits, as `TritonRoute`'s
/// `getWorkerBatchesBoxes`.
fn batches(reaches: &[Rect<Dbu>]) -> Vec<Vec<usize>> {
    let mut batches: Vec<Vec<usize>> = Vec::new();
    for (i, reach) in reaches.iter().enumerate() {
        let fits = |batch: &Vec<usize>| batch.iter().all(|&j| !reaches[j].touches(reach));
        match batches.iter_mut().find(|b| fits(b)) {
            Some(batch) => batch.push(i),
            None => batches.push(vec![i]),
        }
    }
    batches
}

/// Whether a box is taller than wide, as `odb::Rect::getDir` tells.
fn upright(r: &Rect<Dbu>) -> bool {
    r.width() < r.height()
}

/// Merges boxes running the same way that touch, as `TritonRoute`'s
/// guides flow merges its workers.
fn merge_guides(mut boxes: Vec<Rect<Dbu>>) -> Vec<Rect<Dbu>> {
    let mut i = 0;
    while i < boxes.len() {
        let mut j = i + 1;
        while j < boxes.len() {
            if upright(&boxes[i]) == upright(&boxes[j]) && boxes[i].touches(&boxes[j]) {
                boxes[i] = boxes[i].union(&boxes[j]);
                let _ = boxes.remove(j);
                j = i + 1;
            } else {
                j += 1;
            }
        }
        i += 1;
    }
    boxes
}

/// What one worker did.
struct Outcome {
    routed: Vec<Routed>,
    verdict: Verdict,
}

impl DetailedRouter<'_> {
    /// Routes again, over the global routes of the nets in each violation,
    /// the nets `scope` selects there.
    pub fn repair_guides(&mut self, scope: Scope, costs: Costs) -> RepairStats {
        let mut boxes = Vec::new();
        for m in self.markers() {
            for net in m.nets.iter().flatten() {
                let Some(guide) = &self.guides[net.index()] else {
                    continue;
                };
                let runs = self.guide_runs(guide);
                boxes.extend(runs.into_iter().filter(|r| r.touches(&m.bbox)));
            }
        }
        let tiles = merge_guides(boxes).into_iter().map(|b| vec![b]).collect();
        self.repair_tiles(tiles, scope, &[costs])
    }

    /// Routes again, on tiles around each cluster of violations, the nets
    /// `scope` selects there, each tile with every mix of the conflict and
    /// marker costs times [`COST_FACTORS`], keeping per cluster the result
    /// with the fewest violations.
    pub fn repair_stubborn(&mut self, scope: Scope, costs: Costs) -> RepairStats {
        let limit = self.gcell_limit();
        let boxes = self
            .markers()
            .iter()
            .map(|m| self.cell_box(&m.bbox, limit))
            .collect();
        let tiles = expand_boxes(&merge_boxes(boxes), limit)
            .into_iter()
            .map(|set| set.into_iter().map(|b| self.cell_box_area(b)).collect())
            .collect();
        let variants: Vec<Costs> = COST_FACTORS
            .iter()
            .flat_map(|&c| {
                COST_FACTORS.iter().map(move |&m| Costs {
                    conflict: costs.conflict * c,
                    marker: costs.marker * m,
                    ..costs
                })
            })
            .collect();
        self.repair_tiles(tiles, scope, &variants)
    }

    /// Runs a worker for every tile of every cluster at every one of
    /// `variants`, clusters far enough apart side by side, and commits per
    /// cluster the result with the fewest violations unless it is worse
    /// than the wiring it replaces.
    fn repair_tiles(
        &mut self,
        clusters: Vec<Vec<Rect<Dbu>>>,
        scope: Scope,
        variants: &[Costs],
    ) -> RepairStats {
        let clusters: Vec<(Rect<Dbu>, Vec<Rect<Dbu>>)> = clusters
            .into_iter()
            .filter_map(|tiles| {
                let reach = tiles
                    .iter()
                    .map(|t| t.expand(self.halo))
                    .reduce(|a, b| a.union(&b))?;
                Some((reach, tiles))
            })
            .collect();
        let reaches: Vec<Rect<Dbu>> = clusters.iter().map(|(reach, _)| *reach).collect();
        let mut stats = RepairStats {
            regions: clusters.len(),
            ..RepairStats::default()
        };
        for batch in batches(&reaches) {
            let jobs: Vec<(usize, Rect<Dbu>, Costs)> = batch
                .iter()
                .flat_map(|&c| {
                    clusters[c]
                        .1
                        .iter()
                        .flat_map(move |&tile| variants.iter().map(move |&v| (c, tile, v)))
                })
                .collect();
            let outcomes: Vec<Outcome> = jobs
                .par_iter()
                .map(|&(_, tile, costs)| self.tile_worker(&tile, scope, costs))
                .collect();
            let mut best: BTreeMap<usize, usize> = BTreeMap::new();
            for (k, &(c, _, _)) in jobs.iter().enumerate() {
                let better = best
                    .get(&c)
                    .is_none_or(|&b| outcomes[k].verdict.after < outcomes[b].verdict.after);
                if better {
                    let _ = best.insert(c, k);
                }
            }
            let kept: BTreeSet<usize> = best.into_values().collect();
            let mut changes = Changes::default();
            for (k, outcome) in outcomes.into_iter().enumerate() {
                if !kept.contains(&k) {
                    continue;
                }
                stats.pieces += outcome.routed.len();
                if outcome.verdict.rejected() {
                    continue;
                }
                stats.improved += usize::from(outcome.verdict.improved());
                for item in outcome.routed {
                    if let Some(wiring) = item.wiring {
                        changes.apply(item.net, item.removed, wiring);
                    }
                }
            }
            self.commit_changes(changes);
        }
        stats
    }

    /// Routes again inside `tile` the nets `scope` selects for the
    /// violations touching it, ripping them up in the worker's own view.
    fn tile_worker(&self, tile: &Rect<Dbu>, scope: Scope, costs: Costs) -> Outcome {
        let touching: Vec<usize> = (0..self.markers().len())
            .filter(|&i| self.markers()[i].bbox.touches(tile))
            .collect();
        let mut nets: Vec<NetId> = touching
            .iter()
            .flat_map(|&i| self.nets_of(&self.markers()[i], scope))
            .collect();
        nets.sort_unstable();
        nets.dedup();
        let marks = self.seed_marks(tile, &touching);
        let routed = self.route_ripping(tile, &nets, costs, marks);
        let verdict = self.judge_change(tile, &routed, |_, item| &item.removed);
        Outcome { routed, verdict }
    }

    /// The straight runs of a net's global route, layer by layer, as the
    /// areas of their gcells.
    fn guide_runs(&self, guide: &NetGuide) -> Vec<Rect<Dbu>> {
        let mut lines: BTreeMap<(usize, bool, i64), Vec<i64>> = BTreeMap::new();
        for c in guide.tree.cells() {
            let across = self.tech.layer(c.layer).direction == Direction::Horizontal;
            let (line, along) = if across {
                (i64::from(c.y), i64::from(c.x))
            } else {
                (i64::from(c.x), i64::from(c.y))
            };
            lines
                .entry((c.layer.index(), across, line))
                .or_default()
                .push(along);
        }
        let mut runs = Vec::new();
        for ((_, across, line), mut along) in lines {
            along.sort_unstable();
            along.dedup();
            let mut start = 0;
            for k in 1..=along.len() {
                if k < along.len() && along[k] == along[k - 1] + 1 {
                    continue;
                }
                let (a0, a1) = (along[start], along[k - 1]);
                let run = if across {
                    CellBox {
                        x0: a0,
                        y0: line,
                        x1: a1,
                        y1: line,
                    }
                } else {
                    CellBox {
                        x0: line,
                        y0: a0,
                        x1: line,
                        y1: a1,
                    }
                };
                runs.push(self.cell_box_area(run));
                start = k;
            }
        }
        runs
    }

    /// The gcells of the die, by index.
    fn gcell_limit(&self) -> CellBox {
        let span = |extent: Dbu| (extent.get() + self.gcell.get() - 1) / self.gcell.get();
        let die = self.db.die_area;
        CellBox {
            x0: 0,
            y0: 0,
            x1: (span(die.width()) - 1).max(0),
            y1: (span(die.height()) - 1).max(0),
        }
    }

    /// The gcells `rect` lies in, within `limit`.
    fn cell_box(&self, rect: &Rect<Dbu>, limit: CellBox) -> CellBox {
        let index = |at: Dbu, from: Dbu, last: i64| {
            (at - from)
                .get()
                .div_euclid(self.gcell.get())
                .clamp(0, last)
        };
        CellBox {
            x0: index(rect.min.x, self.grid_origin.x, limit.x1),
            y0: index(rect.min.y, self.grid_origin.y, limit.y1),
            x1: index(rect.max.x, self.grid_origin.x, limit.x1),
            y1: index(rect.max.y, self.grid_origin.y, limit.y1),
        }
    }

    /// The area of the gcells of `b`.
    fn cell_box_area(&self, b: CellBox) -> Rect<Dbu> {
        let corner = |x: i64, y: i64| self.grid_origin + Point::new(self.gcell * x, self.gcell * y);
        Rect::new(corner(b.x0, b.y0), corner(b.x1 + 1, b.y1 + 1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn cell(x: i64, y: i64) -> CellBox {
        CellBox {
            x0: x,
            y0: y,
            x1: x,
            y1: y,
        }
    }

    const DIE: CellBox = CellBox {
        x0: 0,
        y0: 0,
        x1: 99,
        y1: 99,
    };

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    #[test]
    fn violations_a_few_gcells_apart_form_one_cluster() {
        let clusters = merge_boxes(vec![cell(10, 10), cell(14, 12)]);

        assert_eq!(
            clusters,
            [CellBox {
                x0: 10,
                y0: 10,
                x1: 14,
                y1: 12
            }]
        );
    }

    #[test]
    fn violations_farther_apart_stay_separate_clusters() {
        let clusters = merge_boxes(vec![cell(10, 10), cell(15, 10)]);

        assert_eq!(clusters, [cell(10, 10), cell(15, 10)]);
    }

    #[test]
    fn a_lone_violation_gets_a_centred_tile_and_four_shifted_ones() {
        let tiles = expand_boxes(&[cell(50, 50)], DIE);

        let expected = BTreeSet::from([
            CellBox {
                x0: 47,
                y0: 47,
                x1: 53,
                y1: 53,
            },
            CellBox {
                x0: 44,
                y0: 47,
                x1: 50,
                y1: 53,
            },
            CellBox {
                x0: 50,
                y0: 47,
                x1: 56,
                y1: 53,
            },
            CellBox {
                x0: 47,
                y0: 44,
                x1: 53,
                y1: 50,
            },
            CellBox {
                x0: 47,
                y0: 50,
                x1: 53,
                y1: 56,
            },
        ]);
        assert_eq!(tiles, [expected]);
    }

    #[test]
    fn tiles_stop_at_the_die_edge() {
        let tiles = expand_boxes(&[cell(1, 50)], DIE);

        assert!(tiles[0].iter().all(|t| t.x0 >= 0));
        assert!(tiles[0].contains(&CellBox {
            x0: 0,
            y0: 47,
            x1: 4,
            y1: 53
        }));
    }

    #[test]
    fn tiles_of_one_cluster_never_reach_onto_another_clusters_tiles() {
        let tiles = expand_boxes(&[cell(50, 50), cell(56, 50)], DIE);

        for a in &tiles[0] {
            for b in &tiles[1] {
                assert!(a.x1 < b.x0 || b.x1 < a.x0 || a.y1 < b.y0 || b.y1 < a.y0);
            }
        }
    }

    #[test]
    fn workers_whose_reaches_touch_go_into_different_batches() {
        let reaches = [rect(0, 0, 10, 10), rect(10, 0, 20, 10), rect(30, 0, 40, 10)];

        let split = batches(&reaches);

        assert_eq!(split, [vec![0, 2], vec![1]]);
    }

    #[test]
    fn touching_guides_running_the_same_way_merge() {
        let merged = merge_guides(vec![rect(0, 0, 30, 10), rect(30, 0, 60, 10)]);

        assert_eq!(merged, [rect(0, 0, 60, 10)]);
    }

    #[test]
    fn touching_guides_running_across_each_other_stay_apart() {
        let merged = merge_guides(vec![rect(0, 0, 30, 10), rect(20, 0, 30, 40)]);

        assert_eq!(merged.len(), 2);
    }
}
