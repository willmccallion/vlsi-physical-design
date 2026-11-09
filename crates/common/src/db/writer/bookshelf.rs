//! Bookshelf `.pl` placement writer.

use crate::db::design::NetlistDB;
use crate::geom::dbu::{Dbu, DbuPerMicron};
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// Writes every cell's location, orientation and fixed status as a `.pl` file.
pub fn write_pl(db: &NetlistDB, path: &Path) -> io::Result<()> {
    let mut out = BufWriter::new(std::fs::File::create(path)?);
    writeln!(out, "UCLA pl 1.0")?;
    writeln!(out)?;
    for c in &db.cells {
        write!(
            out,
            "{} {} {} : {}",
            c.name,
            microns(c.location.x, db.dbu()),
            microns(c.location.y, db.dbu()),
            c.orient
        )?;
        if !c.status.is_movable() {
            write!(out, " /FIXED")?;
        }
        writeln!(out)?;
    }
    out.flush()
}

/// Formats `value` in microns exactly, without trailing zeros.
fn microns(value: Dbu, dbu: DbuPerMicron) -> String {
    let units = i64::from(dbu.get());
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
    fn microns_are_exact_and_trimmed() {
        let dbu = DbuPerMicron::new(100).unwrap();
        assert_eq!(microns(Dbu(12_350), dbu), "123.5");
        assert_eq!(microns(Dbu(-5), dbu), "-0.05");
        assert_eq!(microns(Dbu(400), dbu), "4");
    }
}
