//! Bookshelf `.pl` placement writer.

use crate::db::design::NetlistDB;
use crate::geom::dbu::Dbu;
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// Writes every cell's location, orientation and fixed status as a `.pl` file.
pub fn write_pl(db: &NetlistDB, path: &Path) -> io::Result<()> {
    let mut out = BufWriter::new(std::fs::File::create(path)?);
    let units = i64::from(db.dbu().get());
    writeln!(out, "UCLA pl 1.0")?;
    writeln!(out)?;
    for c in &db.cells {
        write!(
            out,
            "{} {} {} : {}",
            c.name,
            decimal(c.location.x, units),
            decimal(c.location.y, units),
            c.orient
        )?;
        if !c.status.is_movable() {
            write!(out, " /FIXED")?;
        }
        writeln!(out)?;
    }
    out.flush()
}

/// Formats `value / units` exactly, without trailing zeros.
fn decimal(value: Dbu, units: i64) -> String {
    let v = value.get();
    let sign = if v < 0 { "-" } else { "" };
    let (whole, frac) = (v.abs() / units, v.abs() % units);
    if frac == 0 {
        return format!("{sign}{whole}");
    }
    let digits = units.to_string().len() - 1;
    let frac = format!("{frac:0digits$}");
    format!("{sign}{whole}.{}", frac.trim_end_matches('0'))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decimals_are_exact_and_trimmed() {
        assert_eq!(decimal(Dbu(12_350), 100), "123.5");
        assert_eq!(decimal(Dbu(-5), 100), "-0.05");
        assert_eq!(decimal(Dbu(400), 100), "4");
    }
}
