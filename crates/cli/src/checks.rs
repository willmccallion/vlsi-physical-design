//! The placement and routing checks a run ends with.

use crate::report::Report;
use crate::ui;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::CellId;
use pare_common::db::library::LayerRef;
use pare_common::verify::drc::{DrcReport, Marker, check_drc};
use pare_common::verify::placement::{PlacementViolation, placement_violations};
use pare_common::verify::routing::check_routing;

/// Prints placement violations by kind; returns whether there were none.
pub(crate) fn report_placement(db: &NetlistDB, report: &mut Report) -> bool {
    let violations = placement_violations(db);
    report.checks.placement_violations = violations.len();
    if violations.is_empty() {
        ui::check("Placement is legal.");
        return true;
    }
    if log::log_enabled!(log::Level::Debug) {
        for v in &violations {
            log::debug!("placement violation: {}", describe_violation(db, v));
        }
    }
    let count = |f: fn(&PlacementViolation) -> bool| violations.iter().filter(|v| f(v)).count();
    ui::fail(&format!("Placement has {} violations", violations.len()));
    ui::stat_row(
        "Outside die:",
        &count(|v| matches!(v, PlacementViolation::OutsideDie(_))).to_string(),
        "Overlaps:",
        &count(|v| matches!(v, PlacementViolation::Overlap(..))).to_string(),
    );
    ui::stat_row(
        "Not in a row:",
        &count(|v| matches!(v, PlacementViolation::NotInRow(_))).to_string(),
        "Off site:",
        &count(|v| matches!(v, PlacementViolation::OffSite(_))).to_string(),
    );
    ui::stat_row(
        "Wrong orient:",
        &count(|v| matches!(v, PlacementViolation::WrongOrientation(_))).to_string(),
        "In blockage:",
        &count(|v| matches!(v, PlacementViolation::InBlockage(_))).to_string(),
    );
    ui::stat_row(
        "Outside region:",
        &count(|v| matches!(v, PlacementViolation::OutsideRegion(_))).to_string(),
        "In a fence:",
        &count(|v| matches!(v, PlacementViolation::InFence(_))).to_string(),
    );
    ui::stat_row(
        "Unplaced:",
        &count(|v| matches!(v, PlacementViolation::Unplaced(_))).to_string(),
        "",
        "",
    );
    false
}

/// A violation with the names, locations and sizes of the cells in it.
fn describe_violation(db: &NetlistDB, v: &PlacementViolation) -> String {
    let cell = |c: CellId| {
        let bbox = db.cell_bbox(c);
        format!(
            "{} {:?} at ({}, {})-({}, {})",
            db.cells[c].name,
            db.cells[c].region.map(|r| db.regions[r].name.as_str()),
            bbox.min.x.get(),
            bbox.min.y.get(),
            bbox.max.x.get(),
            bbox.max.y.get()
        )
    };
    match *v {
        PlacementViolation::Overlap(a, b) => format!("overlap {} with {}", cell(a), cell(b)),
        PlacementViolation::Unplaced(c) => format!("unplaced {}", cell(c)),
        PlacementViolation::OutsideDie(c) => format!("outside die {}", cell(c)),
        PlacementViolation::InBlockage(c) => format!("in blockage {}", cell(c)),
        PlacementViolation::NotInRow(c) => format!("not in a row {}", cell(c)),
        PlacementViolation::OffSite(c) => format!("off site {}", cell(c)),
        PlacementViolation::WrongOrientation(c) => format!("wrong orientation {}", cell(c)),
        PlacementViolation::OutsideRegion(c) => format!("outside region {}", cell(c)),
        PlacementViolation::InFence(c) => format!("in a fence {}", cell(c)),
    }
}

/// Prints the connectivity and design rule check results; returns whether
/// every net is connected and nothing routed breaks a rule.
pub(crate) fn report_routing(db: &NetlistDB, run: &mut Report) -> bool {
    let report = check_routing(db);
    for open in &report.opens {
        log::debug!(
            "open: net {} '{}' in {} pieces",
            open.net.index(),
            db.nets[open.net].name,
            open.components
        );
    }
    let units = db.dbu();
    let wirelength: f64 = report
        .stats
        .wirelength
        .iter()
        .map(|&l| units.to_microns(l))
        .sum();
    ui::stat_row(
        "Opens:",
        &report.opens.len().to_string(),
        "Wirelength (um):",
        &format!("{wirelength:.0}"),
    );
    ui::stat_row(
        "Vias:",
        &report.stats.vias.to_string(),
        "Off track:",
        &report.stats.off_track.to_string(),
    );
    ui::stat_row(
        "Wrong way:",
        &report.stats.wrong_way.to_string(),
        "Non-Manhattan:",
        &report.stats.non_manhattan.to_string(),
    );
    let drc = check_drc(db);
    for m in &drc.violations {
        log::debug!("drc: {}", describe_marker(db, &drc, m));
    }
    for m in &drc.advisory {
        log::debug!("drc advisory: {}", describe_marker(db, &drc, m));
    }
    for m in &drc.pin_obstruction {
        log::debug!("drc pin in obstruction: {}", describe_marker(db, &drc, m));
    }
    for m in &drc.placement {
        log::debug!("drc from placement: {}", describe_marker(db, &drc, m));
    }
    for m in &drc.input {
        log::debug!("drc in the input: {}", describe_marker(db, &drc, m));
    }
    ui::stat_row(
        "DRC violations:",
        &drc.violations.len().to_string(),
        "From placement:",
        &drc.placement.len().to_string(),
    );
    ui::stat_row(
        "In the input:",
        &drc.input.len().to_string(),
        "Advisory:",
        &drc.advisory.len().to_string(),
    );
    ui::stat_row(
        "Pins in obstructions:",
        &drc.pin_obstruction.len().to_string(),
        "",
        "",
    );
    for (kind, count) in drc.counts() {
        ui::stat_row(&format!("  {}:", kind.name()), &count.to_string(), "", "");
    }
    for (kind, count) in drc.advisory_counts() {
        ui::stat_row(
            &format!("  {} (advisory):", kind.name()),
            &count.to_string(),
            "",
            "",
        );
    }
    let by_kind = |counts: Vec<(pare_common::verify::drc::Violation, usize)>| {
        counts
            .into_iter()
            .map(|(kind, n)| (kind.name().to_string(), n))
            .collect()
    };
    run.checks.opens = report.opens.len();
    run.checks.wirelength_um = wirelength;
    run.checks.vias = report.stats.vias;
    run.checks.violations = by_kind(drc.counts());
    run.checks.advisory = by_kind(drc.advisory_counts());
    run.checks.pin_obstructions = drc.pin_obstruction.len();
    run.checks.from_placement = drc.placement.len();
    run.checks.in_input = drc.input.len();
    if report.is_connected() && drc.violations.is_empty() {
        ui::check("Routing is connected and clean.");
        true
    } else {
        ui::fail("Routing has opens or design rule violations.");
        false
    }
}

/// A violation, where it is and what it involves, for the debug log.
fn describe_marker(db: &NetlistDB, drc: &DrcReport, m: &Marker) -> String {
    let units = db.dbu();
    let layer = match m.layer {
        LayerRef::Routing(l) => db.library.layer(l).name.as_str(),
        LayerRef::Cut(c) => db.library.cut_layers[c].name.as_str(),
    };
    let name = |i: usize| drc.owners[i].name(db);
    format!(
        "{} on {layer} at ({:.4}, {:.4}) - ({:.4}, {:.4}) between {} and {}",
        m.violation.name(),
        units.to_microns(m.bbox.min.x),
        units.to_microns(m.bbox.min.y),
        units.to_microns(m.bbox.max.x),
        units.to_microns(m.bbox.max.y),
        name(m.owners.0.index()),
        name(m.owners.1.index()),
    )
}
