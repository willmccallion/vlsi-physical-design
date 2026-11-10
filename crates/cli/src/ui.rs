//! Structured CLI output with colored formatting.
//!
//! All functions write to stderr so stdout stays clean for piping.
//! Uses `yansi` for ANSI colors with automatic TTY detection.

use pare_common::db::library::Direction;
use pare_common::units::Seconds;
use pare_placer::global;
use pare_router::{LayerLoad, RoundReport};
use std::time::Duration;
use yansi::Paint;

/// Prints the PARE banner and a horizontal rule.
pub(crate) fn banner() {
    eprintln!(
        "\n  {}  {}",
        format!("PARE v{}", env!("CARGO_PKG_VERSION")).bold(),
        "Placement And Routing Engine".dim()
    );
    rule();
}

/// Prints a dimmed horizontal rule.
pub(crate) fn rule() {
    eprintln!("  {}", "-".repeat(60).dim());
}

/// Prints a section title (e.g. "Placement", "Routing").
pub(crate) fn section(title: &str) {
    eprintln!("\n  {}", title.bold());
    eprintln!("  {}", "-".repeat(40).dim());
}

/// Prints a colored sub-phase label (e.g. "Legalization", "Verification").
pub(crate) fn phase(title: &str) {
    eprintln!("\n  {}", title.cyan().bold());
}

/// Prints a labeled header line: "  > Label  value".
pub(crate) fn header(label: &str, value: &str) {
    eprintln!("  {} {}  {}", ">".cyan().bold(), label.cyan().bold(), value);
}

/// Prints one stat, its label dimmed.
pub(crate) fn stat(label: &str, value: &str) {
    eprintln!("    {:<18} {}", label.dim(), value);
}

/// Prints two stats side by side, each given as its label and value, the
/// labels dimmed.
pub(crate) fn stat_pair(left: (&str, &str), right: (&str, &str)) {
    eprintln!(
        "    {:<18} {:<14} {:<18} {}",
        left.0.dim(),
        left.1,
        right.0.dim(),
        right.1
    );
}

/// Prints the header row for placement iteration table.
pub(crate) fn placement_table_header() {
    eprintln!(
        "\n    {}",
        format!(
            "{:<8} {:>12} {:>12} {:>12} {:>10}",
            "Iter", "HPWL", "Overflow", "Penalty", "Step"
        )
        .dim()
    );
    eprintln!("    {}", "-".repeat(58).dim());
}

/// Prints a single placement iteration row.
pub(crate) fn placement_iter(p: &global::Progress) {
    eprintln!(
        "    {:<8} {:>12.0} {:>12.4} {:>12.3e} {:>10.3e}",
        p.iteration, p.hpwl, p.overflow, p.penalty, p.step
    );
}

/// Prints the header of the global routing table: each layer's resources,
/// the demand on them and the overflow, as `OpenROAD` reports them.
pub(crate) fn gr_layer_table_header() {
    eprintln!(
        "    {}",
        format!(
            "{:<10} {:<10} {:>12} {:>12} {:>8} {:>10}",
            "Layer", "Direction", "Resource", "Demand", "Usage", "Overflow"
        )
        .dim()
    );
}

/// Prints one layer of the global routing table.
pub(crate) fn gr_layer_row(load: &LayerLoad<'_>) {
    let direction = match load.direction {
        Direction::Horizontal => "horizontal",
        Direction::Vertical => "vertical",
    };
    let percent = if load.capacity == 0 {
        0.0
    } else {
        100.0 * load.usage as f64 / load.capacity as f64
    };
    eprintln!(
        "    {:<10} {direction:<10} {:>12} {:>12} {percent:>7.1}% {:>10}",
        load.name, load.capacity, load.usage, load.overflow
    );
}

/// Prints the header of the detailed routing table: the design rule
/// violations left after each round, the nets in them, the pieces routed
/// again, and the round's conflict cost per unit of length.
pub(crate) fn dr_table_header() {
    eprintln!(
        "\n    {}",
        format!(
            "{:<8} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
            "DR Iter", "Markers", "Nets", "Ripped", "Conflict", "Flow", "Time"
        )
        .dim()
    );
    eprintln!("    {}", "-".repeat(8 + 11 * 6).dim());
}

/// Prints one detailed routing round.
pub(crate) fn dr_iter(r: &RoundReport) {
    eprintln!(
        "    {:<8} {:>10} {:>10} {:>10} {:>10.1} {:>10} {:>8}ms",
        r.round,
        r.markers,
        r.conflicted,
        r.repaired,
        r.conflict_cost,
        r.flow,
        r.elapsed.as_millis()
    );
}

/// Prints the header of the timing repair table: the moves made so far,
/// the area change, the worst and total negative slack, the endpoints
/// still failing and the worst of them.
pub(crate) fn timing_table_header() {
    eprintln!(
        "\n    {}",
        format!(
            "{:<6} {:>7} {:>7} {:>8} {:>6} {:>5} {:>7} {:>8} {:>9} {:>6}  {}",
            "Iter",
            "Removed",
            "Resized",
            "Inserted",
            "Cloned",
            "Swaps",
            "Area",
            "WNS",
            "TNS",
            "Viol",
            "Worst"
        )
        .dim()
    );
    eprintln!("    {}", "-".repeat(96).dim());
}

/// One row of the timing repair table.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TimingRow<'a> {
    /// The iteration, or a label for the final row.
    pub(crate) iter: &'a str,
    /// Buffers removed so far.
    pub(crate) removed: usize,
    /// Gates resized so far.
    pub(crate) resized: usize,
    /// Buffers inserted so far.
    pub(crate) inserted: usize,
    /// Gates cloned so far.
    pub(crate) cloned: usize,
    /// Pins swapped so far.
    pub(crate) swaps: usize,
    /// Cell area change since the start, as a percentage.
    pub(crate) area_change_percent: f64,
    /// Worst slack.
    pub(crate) wns: Seconds,
    /// Total negative slack.
    pub(crate) tns: Seconds,
    /// Endpoints still failing.
    pub(crate) failing: usize,
    /// The name of the endpoint with the worst slack, empty when there is none.
    pub(crate) worst: &'a str,
}

/// Prints one timing repair progress row.
pub(crate) fn timing_iter(row: &TimingRow<'_>) {
    eprintln!(
        "    {:<6} {:>7} {:>7} {:>8} {:>6} {:>5} {:>6.1}% {:>8.3} {:>9.1} {:>6}  {}",
        row.iter,
        row.removed,
        row.resized,
        row.inserted,
        row.cloned,
        row.swaps,
        row.area_change_percent,
        row.wns.nanos(),
        row.tns.nanos(),
        row.failing,
        row.worst
    );
}

/// Prints a green PASS line, for a check that passed.
pub(crate) fn pass(msg: &str) {
    eprintln!("  {} {}", "PASS".green().bold(), msg);
}

/// Prints a cyan NOTE line, for what a step did or starts from.
pub(crate) fn note(msg: &str) {
    eprintln!("  {} {}", "NOTE".cyan().bold(), msg);
}

/// Prints a red FAIL line.
pub(crate) fn fail(msg: &str) {
    eprintln!("  {} {}", "FAIL".red().bold(), msg);
}

/// Prints a dimmed "Wrote" line with the file path highlighted.
pub(crate) fn wrote(path: &std::path::Path) {
    eprintln!("    {} {}", "->".dim(), path.display());
}

/// A wall time in seconds, to a tenth.
pub(crate) fn seconds(elapsed: Duration) -> String {
    format!("{:.1}s", elapsed.as_secs_f64())
}

/// Prints a dimmed phase timing line.
pub(crate) fn phase_time(elapsed: Duration) {
    eprintln!("    {}", format!("time: {}", seconds(elapsed)).dim());
}

/// Prints a "Done in Xs" line with a rule.
pub(crate) fn done(elapsed: Duration) {
    eprintln!();
    rule();
    eprintln!("  {} {}\n", "Done in".bold(), seconds(elapsed));
}
