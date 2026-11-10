//! The placement and routing checks a run ends with.

use crate::report::Report;
use crate::ui;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::CellId;
use pare_common::db::library::LayerRef;
use pare_common::verify::drc::{DrcReport, Marker, OwnerIndex, check_drc};
use pare_common::verify::placement::{PlacementViolation, placement_violations, violation_counts};
use pare_common::verify::routing::{RoutingReport, check_routing};

/// Prints placement violations by kind; returns whether there were none.
pub(crate) fn report_placement(db: &NetlistDB, report: &mut Report) -> bool {
    let violations = placement_violations(db);
    report.checks.placement_violations = violations.len();
    if violations.is_empty() {
        ui::pass("Placement is legal.");
        return true;
    }
    if log::log_enabled!(log::Level::Debug) {
        for v in &violations {
            log::debug!("placement violation: {}", describe_violation(db, *v));
        }
    }
    ui::fail(&format!("Placement has {} violations", violations.len()));
    for (kind, count) in violation_counts(&violations) {
        ui::stat(&format!("  {}:", kind.name()), &count.to_string());
    }
    false
}

/// A violation with the names, locations and sizes of the cells in it.
fn describe_violation(db: &NetlistDB, v: PlacementViolation) -> String {
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
    match v {
        PlacementViolation::Overlap(a, b) => format!("Overlap: {} with {}", cell(a), cell(b)),
        PlacementViolation::Unplaced(c)
        | PlacementViolation::OutsideDie(c)
        | PlacementViolation::InBlockage(c)
        | PlacementViolation::NotInRow(c)
        | PlacementViolation::OffSite(c)
        | PlacementViolation::WrongOrientation(c)
        | PlacementViolation::OutsideRegion(c)
        | PlacementViolation::InFence(c) => format!("{}: {}", v.kind().name(), cell(c)),
    }
}

/// Prints the connectivity and design rule check results; returns whether
/// every net is connected and nothing routed breaks a rule.
pub(crate) fn report_routing(db: &NetlistDB, report: &mut Report) -> bool {
    let connectivity = check_routing(db);
    log_opens(db, &connectivity);
    let wirelength = wirelength_um(db, &connectivity);
    show_connectivity(&connectivity, wirelength);
    let drc = check_drc(db);
    log_markers(db, &drc);
    show_drc(&drc);
    report
        .checks
        .record_routing(&connectivity, &drc, wirelength);
    let clean = connectivity.is_connected() && drc.violations.is_empty();
    if clean {
        ui::pass("Routing is connected and clean.");
    } else {
        ui::fail("Routing has opens or design rule violations.");
    }
    clean
}

/// Logs each open net for debugging.
fn log_opens(db: &NetlistDB, connectivity: &RoutingReport) {
    for open in &connectivity.opens {
        log::debug!(
            "open: net {} '{}' in {} pieces",
            open.net.index(),
            db.nets[open.net].name,
            open.components
        );
    }
}

/// The total wirelength of the routing, in microns.
fn wirelength_um(db: &NetlistDB, connectivity: &RoutingReport) -> f64 {
    let units = db.dbu();
    connectivity
        .stats
        .wirelength
        .iter()
        .map(|&l| units.to_microns(l))
        .sum()
}

/// Shows the opens and the wiring's statistics.
fn show_connectivity(connectivity: &RoutingReport, wirelength_um: f64) {
    let stats = &connectivity.stats;
    ui::stat_pair(
        ("Opens:", &connectivity.opens.len().to_string()),
        ("Wirelength (um):", &format!("{wirelength_um:.0}")),
    );
    ui::stat_pair(
        ("Vias:", &stats.vias.to_string()),
        ("Off track:", &stats.off_track.to_string()),
    );
    ui::stat_pair(
        ("Wrong way:", &stats.wrong_way.to_string()),
        ("Non-Manhattan:", &stats.non_manhattan.to_string()),
    );
}

/// Logs every marker of each class for debugging.
fn log_markers(db: &NetlistDB, drc: &DrcReport) {
    if !log::log_enabled!(log::Level::Debug) {
        return;
    }
    let classes = [
        ("drc", &drc.violations),
        ("drc advisory", &drc.advisory),
        ("drc pin in obstruction", &drc.pin_obstruction),
        ("drc from placement", &drc.placement),
        ("drc in the input", &drc.input),
    ];
    for (class, markers) in classes {
        for m in markers {
            log::debug!("{class}: {}", describe_marker(db, drc, m));
        }
    }
}

/// Shows the design rule violations by class and kind.
fn show_drc(drc: &DrcReport) {
    ui::stat_pair(
        ("DRC violations:", &drc.violations.len().to_string()),
        ("From placement:", &drc.placement.len().to_string()),
    );
    ui::stat_pair(
        ("In the input:", &drc.input.len().to_string()),
        ("Advisory:", &drc.advisory.len().to_string()),
    );
    ui::stat(
        "Pins in obstructions:",
        &drc.pin_obstruction.len().to_string(),
    );
    for (kind, count) in drc.counts() {
        ui::stat(&format!("  {}:", kind.name()), &count.to_string());
    }
    for (kind, count) in drc.advisory_counts() {
        ui::stat(
            &format!("  {} (advisory):", kind.name()),
            &count.to_string(),
        );
    }
}

/// A violation, where it is and what it involves.
pub(crate) fn describe_marker(db: &NetlistDB, drc: &DrcReport, m: &Marker) -> String {
    let units = db.dbu();
    let layer = match m.layer {
        LayerRef::Routing(l) => db.library.layer(l).name.as_str(),
        LayerRef::Cut(c) => db.library.cut_layers[c].name.as_str(),
    };
    let name = |o: OwnerIndex| drc.owners[o].name(db);
    format!(
        "{} on {layer} at ({:.4}, {:.4}) - ({:.4}, {:.4}) between {} and {}",
        m.violation.name(),
        units.to_microns(m.bbox.min.x),
        units.to_microns(m.bbox.min.y),
        units.to_microns(m.bbox.max.x),
        units.to_microns(m.bbox.max.y),
        name(m.owners.0),
        name(m.owners.1),
    )
}
