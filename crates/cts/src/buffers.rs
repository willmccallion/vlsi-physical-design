//! The buffers a clock tree is built from, as `TritonCTS` picks them: the
//! inferred clock buffers, the root and sink buffers by load limit, and the
//! buffers characterized.

use crate::CtsError;
use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::{BufferCell, Liberty};
use pare_common::units::Farads;
use pare_resizer::Link;
use pare_resizer::WireRc;

/// The share of a buffer's load limit a sink or root buffer may use, as
/// `CtsOptions`' default `sinkBufferMaxCapDerate`.
const MAX_CAP_DERATE: f32 = 0.01;

/// How close, as a share of its own, a buffer's load limit may come to the
/// one kept before it and still be dropped from the characterized list.
const CLOSE_LIMIT_SHARE: f32 = 0.1;

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
        let wire_cap_per_um = (signal_rc.capacitance.get() * 1e-6) as f32;
        let root = if all.len() == 1 {
            all[0]
        } else {
            best_max_cap_buffer(liberty, &all, wire_cap_per_um * core_length / 2.0)
                .ok_or(CtsError::NoBuffers)?
        };
        let sink = if all.len() == 1 {
            all[0]
        } else {
            best_max_cap_buffer(liberty, &all, wire_cap_per_um * core_length)
                .ok_or(CtsError::NoBuffers)?
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
        let area = cell.area as f32;
        let limit = cell.ports[buffer.output]
            .max_capacitance
            .map(|c| c.get() as f32);
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
    best.or(next_best)
}

/// A buffer's load limit, zero without one, as `TechChar::getMaxCapLimit`
/// reads it.
fn max_cap_limit(liberty: &Liberty, buffer: BufferCell) -> f32 {
    // NOTE: getMaxCapLimit reads a missing limit as zero, which decides the
    // buffers characterized; kept so the trees match TritonCTS's.
    liberty.cell(buffer.id).ports[buffer.output]
        .max_capacitance
        .map_or(0.0, Farads::get) as f32
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
        if (prev - current).abs() / current >= CLOSE_LIMIT_SHARE {
            kept.push(buffer);
            prev = current;
        }
    }
    kept
}

/// The most fanout a buffer may drive under the constraints and the
/// library, as `TritonCTS::getBufferFanoutLimit`; `None` without a limit.
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
        .map(|l| l as u32)
        .min()
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::parser::liberty;
    use std::fmt::Write as _;
    use std::path::Path;

    /// A library of buffers named `B<limit>`, each with that load limit
    /// and, where given, a fanout limit.
    fn buffers(limits: &[(u32, Option<u32>)], default_fanout: Option<u32>) -> Liberty {
        let mut src = String::from("library (x) {\n");
        if let Some(f) = default_fanout {
            writeln!(src, "  default_max_fanout : {f};").unwrap();
        }
        for &(cap, fanout) in limits {
            let fanout = fanout.map_or_else(String::new, |f| format!("max_fanout : {f};"));
            writeln!(
                src,
                "  cell (B{cap}) {{ pin (A) {{ direction : input; }} \
                 pin (Z) {{ direction : output; function : \"A\"; \
                 max_capacitance : {cap}; {fanout} }} }}"
            )
            .unwrap();
        }
        src.push('}');
        liberty::parse_str(&src, Path::new("x.lib")).unwrap()
    }

    fn id(lib: &Liberty, cap: u32) -> BufferCell {
        lib.buffer_cell(lib.cell_id(&format!("B{cap}")).unwrap())
            .unwrap()
    }

    #[test]
    fn buffers_between_the_sink_and_root_limits_are_kept_by_limit_less_close_neighbours() {
        let caps = [60, 10, 30, 32, 5, 70];
        let lib = buffers(&caps.map(|c| (c, None)), None);
        let all: Vec<BufferCell> = caps.iter().map(|&c| id(&lib, c)).collect();

        let kept = trim_and_sort(&lib, &all, id(&lib, 60), id(&lib, 10));

        assert_eq!(kept, vec![id(&lib, 10), id(&lib, 30), id(&lib, 60)]);
    }

    #[test]
    fn no_buffer_within_the_limits_leaves_nothing_to_characterize() {
        let lib = buffers(&[(10, None), (20, None), (40, None)], None);

        let kept = trim_and_sort(&lib, &[id(&lib, 40)], id(&lib, 20), id(&lib, 10));

        assert_eq!(kept, Vec::new());
    }

    #[test]
    fn the_fanout_limit_is_the_lower_of_the_design_and_the_buffer() {
        let lib = buffers(&[(10, Some(8))], Some(20));

        assert_eq!(fanout_limit(&lib, id(&lib, 10), Some(12.0)), Some(8));
        assert_eq!(fanout_limit(&lib, id(&lib, 10), Some(5.0)), Some(5));
    }

    #[test]
    fn a_buffer_without_its_own_fanout_limit_takes_the_library_default() {
        let lib = buffers(&[(10, None)], Some(20));

        assert_eq!(fanout_limit(&lib, id(&lib, 10), None), Some(20));
    }

    #[test]
    fn without_any_fanout_limit_there_is_none() {
        let lib = buffers(&[(10, None)], None);

        assert_eq!(fanout_limit(&lib, id(&lib, 10), None), None);
    }
}
