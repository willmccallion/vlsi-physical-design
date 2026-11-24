//! Buffer sizes and the wire lengths and loads each buffer can drive.

use super::{CAP_MARGIN, ELMORE_SKEW_FACTOR, SLEW_MARGIN};
use crate::bnet::{BufferSize, Fixed};
use crate::dmp::{self, PiModel};
use crate::library::{BufferSet, Link};
use crate::parasitics::WireRc;
use crate::resizer::{RepairError, Resizer, find_targets, meters_to_dbu};
use crate::sta::Sta;
use pare_common::db::liberty::{ArcRole, BufferCell, EdgeValues};
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::units::{Farads, FaradsPerMeter, Meters, Ohms, OhmsPerMeter, Seconds};
use std::ops::Mul;

pub(super) fn margined<Q: Mul<f64, Output = Q>>(limit: Q, percent: f64) -> Q {
    limit * (1.0 - percent / 100.0)
}

/// What a buffer presents and can drive, as `BufferSize` with
/// `characterizeBufferLimits`.
pub(super) fn buffer_size(resizer: &Resizer<'_, Sta>, buffer: BufferCell) -> BufferSize {
    let lib = resizer.link.liberty.cell(buffer.id);
    let (input, output) = (buffer.input, buffer.output);
    let intrinsic = lib
        .delay_arc_sets_to(output)
        .flat_map(|s| s.arcs.iter())
        .map(|a| a.gate_delay(Seconds::ZERO, Farads::ZERO).0)
        .filter(|&d| d > Seconds::ZERO)
        .fold(Seconds::ZERO, Seconds::max);
    let cap_limit = resizer
        .link
        .liberty
        .capacitance_limit(lib, &lib.ports[output])
        .map_or(Farads::INFINITY, |l| margined(l, CAP_MARGIN));
    let margined_max_cap = cap_limit.min(load_limit_by_driver_slew(resizer, buffer));
    BufferSize {
        buffer,
        input_cap: lib.ports[input].capacitance(),
        input_fanout: resizer.link.liberty.fanout_load(lib, &lib.ports[input]),
        input_max_slew: resizer.max_input_slew(lib, &lib.ports[input]),
        output_cap: lib.ports[output].capacitance(),
        area: lib.area,
        intrinsic_delay: Fixed::from_seconds(intrinsic),
        margined_max_cap,
        drive_resistance: lib.drive_resistance(output),
    }
}

/// The load at which a buffer's own slew reaches its limit, by bisection,
/// as `findBufferLoadLimitImpliedByDriverSlew`.
fn load_limit_by_driver_slew(resizer: &Resizer<'_, Sta>, buffer: BufferCell) -> Farads {
    let lib = resizer.link.liberty.cell(buffer.id);
    let (input, output) = (buffer.input, buffer.output);
    let Some(max_slew) = resizer.link.liberty.slew_limit(lib, &lib.ports[output]) else {
        return Farads::INFINITY;
    };
    let max_slew = margined(max_slew, SLEW_MARGIN).get();
    let in_slew = margined(resizer.max_input_slew(lib, &lib.ports[input]), SLEW_MARGIN);
    let objective = |load: f64| -> f64 {
        lib.delay_arc_sets_to(output)
            .flat_map(|s| s.arcs.iter())
            .map(|a| a.gate_delay(in_slew, Farads::new(load)).1.get())
            .fold(f64::NEG_INFINITY, f64::max)
            - max_slew
    };
    let drive = lib.drive_resistance(output).get();
    if drive == 0.0 {
        return Farads::INFINITY;
    }
    let mut cap1 = 0.0;
    let mut cap2 = max_slew / drive * 2.0;
    let tol = 0.01;
    let mut diff1 = objective(cap2);
    let mut iterations = 0;
    while (cap1 - cap2).abs() > cap1.max(cap2) * tol && iterations < 200 {
        iterations += 1;
        if diff1 < 0.0 {
            cap1 = cap2;
            cap2 *= 2.0;
            diff1 = objective(cap2);
        } else {
            let cap3 = f64::midpoint(cap1, cap2);
            let diff2 = objective(cap3);
            if diff2 < 0.0 {
                cap1 = cap3;
            } else {
                cap2 = cap3;
                diff1 = diff2;
            }
        }
    }
    Farads::new(cap1)
}

/// The step of a wire search bound by the least of `limits`, at least one
/// database unit; none, so one step spans any wire, with no limit at all.
pub(super) fn least_length<const N: usize>(limits: [Option<Dbu>; N]) -> Option<Dbu> {
    limits
        .into_iter()
        .flatten()
        .min()
        .map(|least| least.max(Dbu::new(1)))
}

/// The wire a buffer can drive before its load's slew limit is reached, in
/// database units, as `wireLengthLimitImpliedByLoadSlew`; none without a
/// limit, and an error where even no wire breaks the slew limit.
// The quadratic keeps OpenROAD's grouping so it can be checked against it.
#[allow(clippy::suboptimal_flops)]
pub(super) fn wire_length_limit_by_load_slew(
    resizer: &Resizer<'_, Sta>,
    size: &BufferSize,
) -> Result<Option<Dbu>, RepairError> {
    let rc = resizer.graph.wire_rc();
    let dbu = resizer.db.dbu();
    let (res, cap) = (rc.resistance.get(), rc.capacitance.get());
    let (input_cap, drive_resistance) = (size.input_cap.get(), size.drive_resistance.get());
    let max_slew = margined(size.input_max_slew, SLEW_MARGIN).get();
    let a = res * cap;
    let b = res * input_cap + drive_resistance * cap;
    let c = drive_resistance * input_cap - max_slew / ELMORE_SKEW_FACTOR;
    // NOTE: wireLengthLimitImpliedByLoadSlew divides by a zero `a` for a
    // wire without resistance or capacitance; PARE sets no limit there.
    if a <= 0.0 {
        return Ok(None);
    }
    let unreachable =
        || RepairError::WireLengthLimit(resizer.link.liberty.cell(size.buffer.id).name.clone());
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return Err(unreachable());
    }
    let meters = (-b + discriminant.sqrt()) / (2.0 * a);
    if meters > 1.0 {
        return Ok(None);
    }
    let length = meters_to_dbu(Meters::new(meters), dbu);
    if meters < 0.0 || length == Dbu::ZERO {
        return Err(unreachable());
    }
    Ok(Some(length))
}

/// The wire a buffer can drive before its load limit is reached, as
/// `wireLengthLimitImpliedByMaxCap`.
pub(super) fn wire_length_limit_by_max_cap(
    resizer: &Resizer<'_, Sta>,
    size: &BufferSize,
) -> Option<Dbu> {
    let lib = resizer.link.liberty.cell(size.buffer.id);
    let limit = resizer
        .link
        .liberty
        .capacitance_limit(lib, &lib.ports[size.buffer.output])?;
    let rc = resizer.graph.wire_rc();
    if rc.capacitance <= FaradsPerMeter::ZERO {
        return None;
    }
    let dbu = resizer.db.dbu();
    let slack = margined(limit, CAP_MARGIN) - size.input_cap;
    Some(meters_to_dbu(
        Meters::new((slack / rc.capacitance).get().max(0.0)),
        dbu,
    ))
}

/// The buffers worth inserting for speed: of the repair buffers by rising
/// input capacitance, those no other buffer beats at every load, as
/// `findFastBuffers`.
pub(super) fn find_fast_buffers(resizer: &Resizer<'_, Sta>) -> Vec<BufferCell> {
    let input_cap = |buffer: BufferCell| resizer.link.buffer_input_capacitance(buffer);
    let mut by_input: Vec<BufferCell> = resizer.buffers.cells.clone();
    by_input.sort_by(|&a, &b| input_cap(a).total_cmp(&input_cap(b)));
    let r_max = resizer
        .link
        .buffer_drive_resistance(resizer.buffers.lowest_drive);
    let mut fast: Vec<BufferCell> = Vec::new();
    for size in by_input {
        if fast.is_empty() {
            fast.push(size);
            continue;
        }
        let back = fast[fast.len() - 1];
        if !outmatched(resizer, size, back, r_max) {
            while let Some(&last) = fast.last() {
                if outmatched(resizer, last, size, r_max) {
                    let _ = fast.pop();
                } else {
                    break;
                }
            }
            fast.push(size);
        }
    }
    fast
}

/// Whether `better` is at least as fast as `worse` at every load either
/// table covers, once its extra input capacitance is charged against it,
/// as `bufferSizeOutmatched`.
fn outmatched(
    resizer: &Resizer<'_, Sta>,
    worse: BufferCell,
    better: BufferCell,
    r_max: Ohms,
) -> bool {
    let liberty = resizer.link.liberty;
    let (w, b) = (liberty.cell(worse.id), liberty.cell(better.id));
    let (w_in, w_out, b_in) = (worse.input, worse.output, better.input);
    let extra_input = (b.ports[b_in].capacitance() - w.ports[w_in].capacitance()).max(Farads::ZERO);
    let penalty = r_max * extra_input;
    let limit = liberty.capacitance_limit(w, &w.ports[w_out]);
    let mut points: Vec<Farads> = Vec::new();
    for buffer in [worse, better] {
        let cell = liberty.cell(buffer.id);
        let (input, output) = (buffer.input, buffer.output);
        for set in cell
            .arc_sets
            .iter()
            .filter(|s| s.role == ArcRole::Combinational && s.from == input && s.to == output)
        {
            for arc in &set.arcs {
                if let Some(table) = &arc.delay {
                    points.extend(table.capacitance_points());
                }
            }
        }
    }
    if points.is_empty() {
        return false;
    }
    points.sort_by(Farads::total_cmp);
    points.dedup();
    let cutoff = w.ports[w_in].capacitance().min(b.ports[b_in].capacitance()) * 20.0;
    for p in points {
        if limit.is_none_or(|l| p <= l) && p < cutoff {
            let bd = resizer.buffer_delay(better, p);
            let wd = resizer.buffer_delay(worse, p);
            if bd + penalty > wd {
                return false;
            }
        }
    }
    true
}

/// The wire length beyond which a buffer is worth inserting, in database
/// units.
///
/// The shortest, over the repair buffers, at which one buffer driving the
/// wire whole is slower than two each driving half, as `findMaxWireLength`.
pub fn max_wire_length(resizer: &Resizer<'_, Sta>) -> Option<Dbu> {
    wire_length_limit(
        resizer.link,
        &resizer.buffers,
        &resizer.targets.slews,
        resizer.graph.wire_rc(),
        resizer.db.dbu(),
    )
}

/// `max_wire_length` for `buffers` of `link` on wire of `rc`, without a
/// design: the limit depends on nothing else.
pub fn max_wire_length_of(
    link: &Link<'_>,
    buffers: &BufferSet,
    rc: WireRc,
    dbu: DbuPerMicron,
) -> Option<Dbu> {
    let targets = find_targets(link, buffers);
    wire_length_limit(link, buffers, &targets.slews, rc, dbu)
}

fn wire_length_limit(
    link: &Link<'_>,
    buffers: &BufferSet,
    slews: &EdgeValues<Seconds>,
    rc: WireRc,
    dbu: DbuPerMicron,
) -> Option<Dbu> {
    if rc.resistance <= OhmsPerMeter::ZERO {
        return None;
    }
    let mut shortest = Meters::INFINITY;
    for &buffer in &buffers.cells {
        let meters = buffer_max_wire_length(link, buffer, slews, rc);
        shortest = shortest.min(meters);
    }
    if !shortest.is_finite() {
        return None;
    }
    Some(meters_to_dbu(shortest, dbu))
}

/// `findMaxWireLength` for one buffer.
fn buffer_max_wire_length(
    link: &Link<'_>,
    buffer: BufferCell,
    slews: &EdgeValues<Seconds>,
    rc: WireRc,
) -> Meters {
    let lib = link.liberty.cell(buffer.id);
    let (input, output) = (buffer.input, buffer.output);
    let drive = lib.drive_resistance(output);
    let delay = |length: f64| -> f64 {
        let length = Meters::new(length);
        let resistance = length * rc.resistance;
        let capacitance = length * rc.capacitance;
        let mut worst = -Seconds::INFINITY;
        for set in lib.delay_arc_sets_to(output) {
            for arc in &set.arcs {
                let load_cap = lib.ports[input].capacitance.get(arc.to_edge);
                let model = PiModel {
                    c2: capacitance / 2.0,
                    rpi: resistance,
                    c1: capacitance / 2.0 + load_cap,
                };
                let elmore = resistance * (capacitance / 2.0 + load_cap);
                let result = dmp::gate_delay(
                    arc,
                    &link.liberty.thresholds,
                    slews.get(arc.from_edge),
                    model,
                );
                let (wire_delay, _) = result.load_delay_slew(elmore);
                worst = worst.max(result.delay + wire_delay);
            }
        }
        worst.get()
    };
    let diff = |length: f64| delay(length) - 2.0 * delay(length / 2.0);
    let mut low = 0.0;
    let mut high = (drive / rc.resistance).get();
    if !high.is_finite() || high <= 0.0 {
        return Meters::INFINITY;
    }
    let tol = 0.01;
    let mut diff_high = diff(high);
    let mut iterations = 0;
    while (low - high).abs() > low.max(high) * tol && iterations < 200 {
        iterations += 1;
        if diff_high < 0.0 {
            low = high;
            high *= 2.0;
            diff_high = diff(high);
        } else {
            let mid = f64::midpoint(low, high);
            if diff(mid) < 0.0 {
                low = mid;
            } else {
                high = mid;
                diff_high = diff(mid);
            }
        }
    }
    Meters::new(low)
}
