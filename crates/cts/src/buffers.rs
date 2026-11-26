//! The buffers a clock tree is built from, as `TritonCTS` picks them: the
//! inferred clock buffers, the root and sink buffers by load limit, and the
//! buffers characterized.

use crate::CtsError;
use pare_common::convert::{NearestFrom, SaturatingFrom};
use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::{BufferCell, Liberty};
use pare_common::units::Farads;
use pare_resizer::Link;
use pare_resizer::WireRc;

/// The share of a buffer's load limit a sink or root buffer may use, as
/// `CtsOptions`' default `sinkBufferMaxCapDerate`, a double.
const MAX_CAP_DERATE: f64 = 0.01;

/// How close, as a share of its own, a buffer's load limit may come to the
/// one kept before it and still be dropped from the characterized list, as
/// `TechChar::trimSortBufferList` fixes it.
const CLOSE_LIMIT_SHARE: f64 = 0.1;

/// The buffers of a clock tree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ClockBuffers {
    /// Every clock buffer, in library order.
    pub all: Vec<BufferCell>,
    /// The buffer at each tree's root and along its trunk.
    pub root: BufferCell,
    /// The buffer driving each cluster of sinks.
    pub sink: BufferCell,
    /// The buffers characterized, by increasing load limit.
    pub characterized: Vec<BufferCell>,
}

impl ClockBuffers {
    /// Infers the clock buffers and picks the root and sink buffers for
    /// wires as long as the core with signal wire RC `signal_rc`, as
    /// `TritonCTS::setupCharacterization` with no buffers given.
    pub(crate) fn infer(
        link: &Link<'_>,
        db: &NetlistDB,
        signal_rc: &WireRc,
    ) -> Result<Self, CtsError> {
        let all = link.infer_clock_buffers(db);
        if all.is_empty() {
            return Err(CtsError::NoBuffers);
        }
        let liberty = link.liberty;
        let core = db.core_area();
        let dbu = db.dbu().get() as f32;
        let core_length = (core.width().get().max(core.height().get()) as f32) / dbu;
        let root = if all.len() == 1 {
            all[0]
        } else {
            let half_core = signal_wire_cap(signal_rc, f64::from(core_length) / 2.0);
            best_max_cap_buffer(liberty, &all, half_core).ok_or(CtsError::NoBuffers)?
        };
        let sink = if all.len() == 1 {
            all[0]
        } else {
            let whole_core = signal_wire_cap(signal_rc, f64::from(core_length));
            best_max_cap_buffer(liberty, &all, whole_core).ok_or(CtsError::NoBuffers)?
        };
        let characterized = trim_and_sort(liberty, &all, root, sink);
        Ok(Self {
            all,
            root,
            sink,
            characterized,
        })
    }
}

/// The capacitance of `length_um` microns of signal wire, worked out in
/// double precision and rounded once to single, as
/// `TritonCTS::selectRootBuffer` and `selectSinkBuffer` estimate it.
fn signal_wire_cap(signal_rc: &WireRc, length_um: f64) -> f32 {
    f32::nearest_from(signal_rc.capacitance.per_micron() * length_um)
}

/// The smallest buffer whose derated load limit covers `total_cap`, or the
/// one with the largest limit when none does, as
/// `TritonCTS::selectBestMaxCapBuffer`; none when no buffer has a limit.
fn best_max_cap_buffer(
    liberty: &Liberty,
    buffers: &[BufferCell],
    total_cap: f32,
) -> Option<BufferCell> {
    let mut best = None;
    let mut best_area = f32::MAX;
    let mut next_best = None;
    let mut best_cap = 0.0_f32;
    for &buffer in buffers {
        let cell = liberty.cell(buffer.id);
        let area = f32::nearest_from(cell.area.get());
        let limit = cell.ports[buffer.output]
            .max_capacitance
            .map(|c| f32::nearest_from(c.get()));
        if let Some(max_cap) = limit
            && f64::from(max_cap) * MAX_CAP_DERATE > f64::from(total_cap)
            && area < best_area
        {
            best = Some(buffer);
            best_area = area;
        }
        let max_cap = limit.unwrap_or(0.0);
        if max_cap > best_cap {
            next_best = Some(buffer);
            best_cap = max_cap;
        }
    }
    best.or(next_best)
}

/// A buffer's load limit, zero without one, as `TechChar::getMaxCapLimit`
/// reads it.
fn max_cap_limit(liberty: &Liberty, buffer: BufferCell) -> f32 {
    // NOTE: getMaxCapLimit reads a missing limit as zero, which decides the
    // buffers characterized; kept so the trees match TritonCTS's.
    f32::nearest_from(
        liberty.cell(buffer.id).ports[buffer.output]
            .max_capacitance
            .map_or(0.0, Farads::get),
    )
}

/// The inferred buffers to characterize, as `TechChar::trimSortBufferList`
/// trims an inferred list: those whose load limit lies between the root's
/// and the sink's, sorted by limit, each too close to the one kept before
/// it dropped.
fn trim_and_sort(
    liberty: &Liberty,
    buffers: &[BufferCell],
    root: BufferCell,
    sink: BufferCell,
) -> Vec<BufferCell> {
    let limit = |b: BufferCell| max_cap_limit(liberty, b);
    let (sink_cap, root_cap) = (limit(sink), limit(root));
    let (low, high) = (sink_cap.min(root_cap), sink_cap.max(root_cap));
    let mut list: Vec<BufferCell> = buffers
        .iter()
        .copied()
        .filter(|&b| (low..=high).contains(&limit(b)))
        .collect();
    list.sort_by(|&a, &b| limit(a).total_cmp(&limit(b)));
    let Some((&first, rest)) = list.split_first() else {
        return list;
    };
    let mut kept = vec![first];
    let mut prev = limit(first);
    for &buffer in rest {
        let current = limit(buffer);
        // Written as trimSortBufferList drops, so two missing limits (0 / 0) keep.
        let is_close = f64::from((prev - current).abs() / current) < CLOSE_LIMIT_SHARE;
        if !is_close {
            kept.push(buffer);
            prev = current;
        }
    }
    kept
}

/// The most fanout a buffer may drive under the constraints and the
/// library, as `TritonCTS::getBufferFanoutLimit`; `None` without a limit,
/// or with one below a single sink.
pub(crate) fn fanout_limit(
    liberty: &Liberty,
    buffer: BufferCell,
    design_limit: Option<f64>,
) -> Option<u32> {
    let cell = liberty.cell(buffer.id);
    let port_limit = cell.ports[buffer.output]
        .max_fanout
        .or_else(|| liberty.cell_defaults(cell).max_fanout);
    [design_limit, port_limit]
        .into_iter()
        .flatten()
        .map(u32::saturating_from)
        .min()
        // NOTE: kept: TritonCTS truncates a limit below one to 0 and its callers skip it.
        .filter(|&limit| limit > 0)
}

#[cfg(test)]
mod tests;
