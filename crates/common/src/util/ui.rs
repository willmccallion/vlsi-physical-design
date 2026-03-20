//! Structured CLI output with colored formatting.
//!
//! All functions write to stderr so stdout stays clean for piping.
//! Uses `yansi` for ANSI colors with automatic TTY detection.

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
    eprintln!(
        "    {:<18} {:<14} {:<18} {}",
        l1.dim(),
        v1,
        l2.dim(),
        v2
    );
}

/// Prints the header row for placement iteration table.
pub fn placement_table_header() {
    eprintln!(
        "\n    {}",
        format!(
            "{:<8} {:>12} {:>12} {:>12} {:>10}",
            "Iter", "Wirelength", "Overflow", "AvgMove", "Step"
        )
        .dim()
    );
    eprintln!("    {}", "-".repeat(58).dim());
}

/// Prints a single placement iteration row.
pub fn placement_iter(iter: usize, wl: f64, overflow: f64, avg_move: f64, step: f64) {
    eprintln!(
        "    {iter:<8} {wl:>12.0} {overflow:>12.4} {avg_move:>12.4} {step:>10.5}"
    );
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

/// Prints a single routing iteration row.
pub fn routing_iter(phase: &str, iter: usize, overflow: usize, ripped: usize, penalty: f64, time_ms: u128) {
    eprintln!(
        "    {}{:<8} {:>10} {:>10} {:>10.1} {:>8}ms",
        " ".repeat(phase.len() + 1),
        iter, overflow, ripped, penalty, time_ms
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
    eprint!(
        "\r  {} {}/{}\x1b[K",
        label.cyan(),
        cur,
        total
    );
    let _ = std::io::Write::flush(&mut std::io::stderr());
}

/// Clears the progress line.
pub fn progress_clear() {
    eprint!("\r\x1b[K");
    let _ = std::io::Write::flush(&mut std::io::stderr());
}
