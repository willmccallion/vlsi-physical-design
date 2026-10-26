//! The buffers a clock tree is built from, as `TritonCTS` picks them: the
//! inferred clock buffers, the root and sink buffers by load limit, and the
//! buffers characterized.

use crate::CtsError;
use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::{LibCellId, Liberty};
use pare_resizer::Link;
use pare_resizer::parasitics::WireRc;

/// The share of a buffer's load limit a sink or root buffer may use, as
/// `CtsOptions`' default `sinkBufferMaxCapDerate`.
const MAX_CAP_DERATE: f32 = 0.01;

/// The buffers of a clock tree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClockBuffers {
    /// Every clock buffer, in library order.
    pub all: Vec<LibCellId>,
    /// The buffer at each tree's root and along its trunk.
    pub root: LibCellId,
    /// The buffer driving each cluster of sinks.
    pub sink: LibCellId,
    /// The buffers characterized, by increasing load limit.
    pub characterized: Vec<LibCellId>,
}

impl ClockBuffers {
    /// Infers the clock buffers and picks the root and sink buffers for
    /// wires as long as the core with signal wire RC `signal_rc`, as
    /// `TritonCTS::setupCharacterization` with no buffers given.
    pub fn infer(link: &Link<'_>, db: &NetlistDB, signal_rc: &WireRc) -> Result<Self, CtsError> {
        let all = link.infer_clock_buffers(db);
        if all.is_empty() {
            return Err(CtsError::NoBuffers);
        }
        let liberty = link.liberty;
        let core = db.core_area();
        let dbu = db.dbu().get() as f32;
        let core_length = (core.width().get().max(core.height().get()) as f32) / dbu;
        let wire_cap_per_um = (signal_rc.capacitance * 1e-6) as f32;
        let root = if all.len() == 1 {
            all[0]
        } else {
            best_max_cap_buffer(liberty, &all, wire_cap_per_um * core_length / 2.0)?
        };
        let sink = if all.len() == 1 {
            all[0]
        } else {
            best_max_cap_buffer(liberty, &all, wire_cap_per_um * core_length)?
        };
        let characterized = trim_and_sort(liberty, &all, root, sink, true);
        Ok(Self {
            all,
            root,
            sink,
            characterized,
        })
    }
}

/// The smallest buffer whose derated load limit covers `total_cap`, or the
/// one with the largest limit when none does, as
/// `TritonCTS::selectBestMaxCapBuffer`.
fn best_max_cap_buffer(
    liberty: &Liberty,
    buffers: &[LibCellId],
    total_cap: f32,
) -> Result<LibCellId, CtsError> {
    let mut best = None;
    let mut best_area = f32::MAX;
    let mut next_best = None;
    let mut best_cap = 0.0_f32;
    for &buffer in buffers {
        let cell = liberty.cell(buffer);
        let (_, output) = cell
            .buffer_ports()
            .ok_or_else(|| CtsError::NotABuffer(cell.name.clone()))?;
        let area = cell.area as f32;
        let limit = cell.ports[output].max_capacitance.map(|c| c as f32);
        if let Some(max_cap) = limit
            && max_cap * MAX_CAP_DERATE > total_cap
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
    best.or(next_best).ok_or(CtsError::NoBuffers)
}

/// A buffer's load limit, zero without one.
fn max_cap_limit(liberty: &Liberty, buffer: LibCellId) -> f32 {
    let cell = liberty.cell(buffer);
    cell.buffer_ports()
        .and_then(|(_, output)| cell.ports[output].max_capacitance)
        .unwrap_or(0.0) as f32
}

/// The buffers to characterize, as `TechChar::trimSortBufferList`: for an
/// inferred list, those whose load limit lies between the root's and the
/// sink's, sorted by limit, each within a tenth of the one before dropped.
fn trim_and_sort(
    liberty: &Liberty,
    buffers: &[LibCellId],
    root: LibCellId,
    sink: LibCellId,
    inferred: bool,
) -> Vec<LibCellId> {
    let mut list = buffers.to_vec();
    if inferred {
        let (sink_cap, root_cap) = (max_cap_limit(liberty, sink), max_cap_limit(liberty, root));
        let (low, high) = (sink_cap.min(root_cap), sink_cap.max(root_cap));
        list.retain(|&b| (low..=high).contains(&max_cap_limit(liberty, b)));
    }
    list.sort_by(|&a, &b| max_cap_limit(liberty, a).total_cmp(&max_cap_limit(liberty, b)));
    if inferred && !list.is_empty() {
        let mut kept = vec![list[0]];
        let mut prev = max_cap_limit(liberty, list[0]);
        for &buffer in &list[1..] {
            let current = max_cap_limit(liberty, buffer);
            if (prev - current).abs() / current >= 0.1 {
                kept.push(buffer);
                prev = current;
            }
        }
        list = kept;
    }
    list
}

/// The most fanout a buffer may drive under the constraints and the
/// library, as `TritonCTS::getBufferFanoutLimit`; `None` without a limit.
pub(crate) fn fanout_limit(
    liberty: &Liberty,
    buffer: LibCellId,
    design_limit: Option<f64>,
) -> Option<u32> {
    let cell = liberty.cell(buffer);
    let port_limit = cell
        .buffer_ports()
        .and_then(|(_, output)| cell.ports[output].max_fanout)
        .or_else(|| liberty.cell_defaults(cell).max_fanout);
    [design_limit, port_limit]
        .into_iter()
        .flatten()
        .map(|l| l as u32)
        .min()
}
