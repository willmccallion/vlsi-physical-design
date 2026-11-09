//! `ShiftLegalizer`: puts every movable cell in its segments and clumps
//! overlapping cells apart, which on a legal placement moves nothing.

use super::{ImprovementError, Improver, NodeId, Result, SegmentId};
use pare_common::db::indices::IdVec;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;

/// A run of cells pushed together, as `ShiftLegalizer`'s `Clump`.
#[derive(Clone, Debug, Default)]
struct Clump {
    vertices: Vec<Vertex>,
    weight: f64,
    weighted_position: f64,
    position: Dbu,
}

pare_common::define_index!(
    /// A vertex of the shifting constraint graph: every network node, then
    /// each segment's left stand-in, then each segment's right stand-in.
    Vertex(u32)
);

pare_common::define_index!(
    /// A clump, as `ShiftLegalizer` numbers them.
    ClumpId(u32)
);

/// How the constraint graph numbers its vertices.
#[derive(Clone, Copy, Debug)]
struct Vertices {
    nodes: usize,
    segments: usize,
}

impl Vertices {
    const fn count(self) -> usize {
        self.nodes + 2 * self.segments
    }

    const fn node(n: NodeId) -> Vertex {
        Vertex::new(n.index())
    }

    const fn left_end(self, s: SegmentId) -> Vertex {
        Vertex::new(self.nodes + s.index())
    }

    const fn right_end(self, s: SegmentId) -> Vertex {
        Vertex::new(self.nodes + self.segments + s.index())
    }
}

/// The weight of the stand-ins pinning each segment's ends.
const END_WEIGHT: f64 = 1.0e8;

pub(super) fn legalize(imp: &mut Improver) -> Result<()> {
    imp.collect_cells();
    imp.find_blockages();
    imp.find_segments()?;
    let original: IdVec<NodeId, Point<Dbu>> = imp.net.nodes().map(|n| imp.position(n)).collect();
    let mut cells = Vec::new();
    if !imp.single_height.is_empty() {
        let single = imp.single_height.clone();
        imp.assign_cells_to_segments(&single)?;
        cells.extend(single);
    }
    for spanned in 2..imp.multi_height.len() {
        let multi = imp.multi_height[spanned].clone();
        if !multi.is_empty() {
            imp.assign_cells_to_segments(&multi)?;
            cells.extend(multi);
        }
    }
    if !ordered_without_cycles(imp, &cells) {
        return Err(ImprovementError::Internal(
            "Cells incorrectly ordered during shifting",
        ));
    }
    shift(imp, &cells)?;
    let displaced = cells.iter().any(|&n| imp.position(n) != original[n]);
    if displaced {
        log::warn!("unexpected displacement while seeding detailed improvement");
    }
    let problems = imp.placement_problems()?;
    if problems > 0 {
        log::warn!("{problems} placement problems while seeding detailed improvement");
    }
    Ok(())
}

/// Whether the segments' left-to-right orders admit one order of all the
/// cells, as the topological sort `ShiftLegalizer::legalize` checks.
fn ordered_without_cycles(imp: &Improver, cells: &[NodeId]) -> bool {
    let size = imp.net.node_count();
    let mut incoming: IdVec<NodeId, usize> = IdVec::filled(0, size);
    let mut outgoing: IdVec<NodeId, Vec<NodeId>> = IdVec::filled(Vec::new(), size);
    for seg in &imp.cells_in_seg {
        for pair in seg.windows(2) {
            incoming[pair[1]] += 1;
            outgoing[pair[0]].push(pair[1]);
        }
    }
    let mut order: Vec<NodeId> = cells
        .iter()
        .copied()
        .filter(|&n| incoming[n] == 0)
        .collect();
    let mut i = 0;
    while i < order.len() {
        for &next in &outgoing[order[i]] {
            incoming[next] -= 1;
            if incoming[next] == 0 {
                order.push(next);
            }
        }
        i += 1;
    }
    order.len() == cells.len()
}

/// The constraint graph's clumps and where each vertex sits in them.
struct Clumping {
    clumps: IdVec<ClumpId, Clump>,
    owner: IdVec<Vertex, Option<ClumpId>>,
    offset: IdVec<Vertex, Dbu>,
}

impl Clumping {
    fn add(&mut self, vertex: Vertex, left: Dbu, weight: f64, position: Dbu) {
        let id = self.clumps.push(Clump {
            vertices: vec![vertex],
            weight,
            weighted_position: weight * left.to_f64(),
            position,
        });
        self.owner[vertex] = Some(id);
    }

    fn owner(&self, vertex: Vertex) -> Result<ClumpId> {
        self.owner[vertex].ok_or(ImprovementError::Internal(
            "a cell in no clump during shifting",
        ))
    }

    /// Merges clump `r` into clump `l`, `dist` to its right.
    fn merge(&mut self, r: ClumpId, l: ClumpId, dist: Dbu) {
        let moved = std::mem::take(&mut self.clumps[r].vertices);
        for &vertex in &moved {
            self.offset[vertex] += dist;
            self.owner[vertex] = Some(l);
        }
        let (weight, weighted) = (self.clumps[r].weight, self.clumps[r].weighted_position);
        let left = &mut self.clumps[l];
        left.vertices.extend(moved);
        left.weighted_position += weighted - dist.to_f64() * weight;
        left.weight += weight;
        left.position = Dbu((left.weighted_position / left.weight).floor() as i64);
    }

    /// The clump the worst overlap of `r` on its left neighbours is with and
    /// how far it must sit right of it, as `ShiftLegalizer::violated`.
    fn violated(
        &self,
        r: ClumpId,
        incoming: &IdVec<Vertex, Vec<Vertex>>,
        width: &IdVec<Vertex, Dbu>,
    ) -> Result<Option<(ClumpId, Dbu)>> {
        let mut worst: Option<(Dbu, ClumpId, Dbu)> = None;
        for &right in &self.clumps[r].vertices {
            for &left in &incoming[right] {
                let t = self.owner(left)?;
                if t == r {
                    continue;
                }
                let pdst = self.clumps[r].position + self.offset[right];
                let psrc = self.clumps[t].position + self.offset[left];
                let diff = pdst - (psrc + width[left]);
                if diff < Dbu::ZERO && worst.is_none_or(|w| diff < w.0) {
                    worst = Some((
                        diff,
                        t,
                        self.offset[left] + width[left] - self.offset[right],
                    ));
                }
            }
        }
        Ok(worst.map(|(_, l, dist)| (l, dist)))
    }
}

/// Clumps each segment's cells between stand-ins pinned to its ends,
/// as `ShiftLegalizer::shift` and `ShiftLegalizer::clump`.
fn shift(imp: &mut Improver, cells: &[NodeId]) -> Result<()> {
    let vertices = Vertices {
        nodes: imp.net.node_count(),
        segments: imp.segments.len(),
    };
    let mut width: IdVec<Vertex, Dbu> = IdVec::filled(Dbu::ZERO, vertices.count());
    for n in imp.net.nodes() {
        width[Vertices::node(n)] = imp.width(n);
    }
    let mut incoming: IdVec<Vertex, Vec<Vertex>> = IdVec::filled(Vec::new(), vertices.count());
    for s in imp.segments.ids() {
        let mut order = Vec::with_capacity(imp.cells_in_seg[s].len() + 2);
        order.push(vertices.left_end(s));
        order.extend(imp.cells_in_seg[s].iter().map(|&n| Vertices::node(n)));
        order.push(vertices.right_end(s));
        for pair in order.windows(2) {
            incoming[pair[1]].push(pair[0]);
        }
    }
    let mut clumps = Clumping {
        clumps: IdVec::with_capacity(2 * vertices.segments + cells.len()),
        owner: IdVec::filled(None, vertices.count()),
        offset: IdVec::filled(Dbu::ZERO, vertices.count()),
    };
    for s in imp.segments.ids() {
        let min_x = imp.segments[s].min_x;
        clumps.add(vertices.left_end(s), min_x, END_WEIGHT, min_x);
    }
    for &n in cells {
        let mut position = imp.left(n);
        for &s in &imp.cell_segs[n] {
            let seg = &imp.segments[s];
            position = position.max(seg.min_x).min(seg.max_x - imp.width(n));
        }
        clumps.add(Vertices::node(n), imp.left(n), 1.0, position);
    }
    for s in imp.segments.ids() {
        let max_x = imp.segments[s].max_x;
        clumps.add(vertices.right_end(s), max_x, END_WEIGHT, max_x);
    }
    for c in clumps.clumps.ids() {
        let mut r = c;
        while let Some((l, dist)) = clumps.violated(r, &incoming, &width)? {
            clumps.merge(r, l, dist);
            r = l;
        }
    }
    for &n in cells {
        let row = imp.cell_segs[n]
            .iter()
            .map(|&s| imp.segments[s].row)
            .min()
            .ok_or(ImprovementError::Internal(
                "Shifting failed during legalization",
            ))?;
        let vertex = Vertices::node(n);
        let x = clumps.clumps[clumps.owner(vertex)?].position + clumps.offset[vertex];
        let y = imp.arch.rows[row].bottom;
        imp.set_position(n, Point::new(x, y));
    }
    Ok(())
}
