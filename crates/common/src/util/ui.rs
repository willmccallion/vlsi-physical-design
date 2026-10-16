//! Structured CLI output with colored formatting.
//!
//! All functions write to stderr so stdout stays clean for piping.
//! Uses `yansi` for ANSI colors with automatic TTY detection.

use crate::db::library::Direction;
use yansi::Paint;

/// Prints the PARE banner and a horizontal rule.
pub fn banner() {
    eprintln!(
        "\n  {}  {}",
        "PARE v0.1.0".bold(),
        "Placement And Routing Engine".dim()
    );
    rule();
}

/// Prints a dimmed horizontal rule.
pub fn rule() {
    eprintln!("  {}", "-".repeat(60).dim());
}

/// Prints a section title (e.g. "Placement", "Routing").
pub fn section(title: &str) {
    eprintln!("\n  {}", title.bold());
    eprintln!("  {}", "-".repeat(40).dim());
}

/// Prints a colored sub-phase label (e.g. "Legalization", "Verification").
pub fn phase(title: &str) {
    eprintln!("\n  {}", title.cyan().bold());
}

/// Prints a labeled header line: "  > Label  value".
pub fn header(label: &str, value: &str) {
    eprintln!("  {} {}  {}", ">".cyan().bold(), label.cyan().bold(), value);
}

/// Prints a two-column stat row with dimmed labels.
pub fn stat_row(l1: &str, v1: &str, l2: &str, v2: &str) {
    eprintln!("    {:<18} {:<14} {:<18} {}", l1.dim(), v1, l2.dim(), v2);
}

/// Prints the header row for placement iteration table.
pub fn placement_table_header() {
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
pub fn placement_iter(iter: usize, hpwl: f64, overflow: f64, penalty: f64, step: f64) {
    eprintln!("    {iter:<8} {hpwl:>12.0} {overflow:>12.4} {penalty:>12.3e} {step:>10.3e}");
}

/// Prints the header row for routing iteration table.
pub fn routing_table_header(phase: &str) {
    eprintln!(
        "\n    {}",
        format!(
            "{} {:<8} {:>10} {:>10} {:>10} {:>10}",
            phase, "Iter", "Overflow", "Ripped", "Penalty", "Time"
        )
        .dim()
    );
    eprintln!("    {}", "-".repeat(2 + 8 + 10 * 4 + phase.len()).dim());
}

/// Prints the header of the global routing table: each layer's resources,
/// the demand on them and the overflow, as `OpenROAD` reports them.
pub fn gr_layer_table_header() {
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
pub fn gr_layer_row(layer: &str, direction: Direction, capacity: u64, usage: u64, overflow: u64) {
    let direction = match direction {
        Direction::Horizontal => "horizontal",
        Direction::Vertical => "vertical",
    };
    let percent = if capacity == 0 {
        0.0
    } else {
        100.0 * usage as f64 / capacity as f64
    };
    eprintln!(
        "    {layer:<10} {direction:<10} {capacity:>12} {usage:>12} {percent:>7.1}% {overflow:>10}"
    );
}

/// Prints the header of the detailed routing table: the design rule
/// violations left after each round, the nets in them, and the pieces
/// routed again.
pub fn dr_table_header() {
    eprintln!(
        "\n    {}",
        format!(
            "{:<8} {:>10} {:>10} {:>10} {:>10} {:>10}",
            "DR Iter", "Markers", "Nets", "Ripped", "Penalty", "Time"
        )
        .dim()
    );
    eprintln!("    {}", "-".repeat(8 + 11 * 5).dim());
}

/// Prints one detailed routing round.
pub fn dr_iter(
    iter: usize,
    markers: usize,
    nets: usize,
    ripped: usize,
    penalty: f64,
    time_ms: u128,
) {
    eprintln!("    {iter:<8} {markers:>10} {nets:>10} {ripped:>10} {penalty:>10.1} {time_ms:>8}ms");
}

/// Prints a single routing iteration row.
pub fn routing_iter(
    phase: &str,
    iter: usize,
    overflow: usize,
    ripped: usize,
    penalty: f64,
    time_ms: u128,
) {
    eprintln!(
        "    {}{:<8} {:>10} {:>10} {:>10.1} {:>8}ms",
        " ".repeat(phase.len() + 1),
        iter,
        overflow,
        ripped,
        penalty,
        time_ms
    );
}

/// Prints a green PASS line.
pub fn check(msg: &str) {
    eprintln!("  {} {}", "PASS".green().bold(), msg);
}

/// Prints a red FAIL line.
pub fn fail(msg: &str) {
    eprintln!("  {} {}", "FAIL".red().bold(), msg);
}

/// Prints a dimmed "Wrote" line with the file path highlighted.
pub fn wrote(path: &str) {
    eprintln!("    {} {}", "->".dim(), path);
}

/// Prints a dimmed phase timing line.
pub fn phase_time(elapsed_secs: f64) {
    eprintln!("    {}", format!("time: {elapsed_secs:.1}s").dim());
}

/// Prints a "Done in Xs" line with a rule.
pub fn done(elapsed_secs: f64) {
    eprintln!();
    rule();
    eprintln!("  {} {:.1}s\n", "Done in".bold(), elapsed_secs);
}

/// Prints a carriage-return progress line (overwrites current line).
pub fn progress(label: &str, cur: usize, total: usize) {
    eprint!("\r  {} {}/{}\x1b[K", label.cyan(), cur, total);
    let _ = std::io::Write::flush(&mut std::io::stderr());
}

/// Clears the progress line.
pub fn progress_clear() {
    eprint!("\r\x1b[K");
    let _ = std::io::Write::flush(&mut std::io::stderr());
}
