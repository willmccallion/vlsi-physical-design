//! Bookshelf `.pl` placement writer.

use crate::db::design::NetlistDB;
use crate::geom::dbu::{Dbu, DbuPerMicron};
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// Writes every cell's location, orientation and fixed status as a `.pl` file.
///
/// Fails when the design's database unit has no exact decimal form in
/// microns; every resolution LEF allows has one.
pub fn write_pl(db: &NetlistDB, path: &Path) -> io::Result<()> {
    let format = MicronFormat::new(db.dbu()).ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "1/{} um has no exact decimal form for a Bookshelf .pl",
                db.dbu().get()
            ),
        )
    })?;
    let mut out = BufWriter::new(std::fs::File::create(path)?);
    writeln!(out, "UCLA pl 1.0")?;
    writeln!(out)?;
    for c in &db.cells {
        write!(
            out,
            "{} {} {} : {}",
            c.name,
            format.microns(c.location.x),
            format.microns(c.location.y),
            c.orient
        )?;
        if !c.status.is_movable() {
            write!(out, " /FIXED")?;
        }
        writeln!(out)?;
    }
    out.flush()
}

/// Writes database units as exact decimal microns.
struct MicronFormat {
    units: i64,
    /// Decimal places that show any fraction of a micron exactly.
    places: usize,
    /// One database unit in units of the last decimal place.
    place_value: i128,
}

impl MicronFormat {
    /// The format at `dbu`, or `None` when its database unit is not a
    /// terminating decimal of a micron, as only resolutions of the form
    /// 2^a * 5^b are. Any `u32` of that form divides 10^32, within `i128`.
    fn new(dbu: DbuPerMicron) -> Option<Self> {
        let units = i64::from(dbu.get());
        let mut power: i128 = 1;
        let mut places = 0;
        while power % i128::from(units) != 0 {
            power = power.checked_mul(10)?;
            places += 1;
        }
        Some(Self {
            units,
            places,
            place_value: power / i128::from(units),
        })
    }

    /// `value` in microns, without trailing zeros.
    fn microns(&self, value: Dbu) -> String {
        let v = value.get();
        let sign = if v < 0 { "-" } else { "" };
        let (whole, frac) = (v.abs() / self.units, v.abs() % self.units);
        if frac == 0 {
            return format!("{sign}{whole}");
        }
        let digits = format!(
            "{:0width$}",
            i128::from(frac) * self.place_value,
            width = self.places
        );
        format!("{sign}{whole}.{}", digits.trim_end_matches('0'))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn microns(value: Dbu, dbu: DbuPerMicron) -> String {
        MicronFormat::new(dbu).unwrap().microns(value)
    }

    #[test]
    fn microns_are_exact_and_trimmed() {
        let dbu = DbuPerMicron::new(100).unwrap();
        assert_eq!(microns(Dbu(12_350), dbu), "123.5");
        assert_eq!(microns(Dbu(-5), dbu), "-0.05");
        assert_eq!(microns(Dbu(400), dbu), "4");
    }

    #[test]
    fn microns_are_exact_at_a_resolution_other_than_a_power_of_ten() {
        let dbu = DbuPerMicron::new(2000).unwrap();
        assert_eq!(microns(Dbu(1000), dbu), "0.5");
        assert_eq!(microns(Dbu(-2001), dbu), "-1.0005");
    }

    #[test]
    fn microns_are_exact_at_the_finest_binary_resolution() {
        let dbu = DbuPerMicron::new(1 << 31).unwrap();
        assert_eq!(microns(Dbu(1), dbu), "0.0000000004656612873077392578125");
    }

    #[test]
    fn a_resolution_without_exact_decimals_has_no_format() {
        assert!(MicronFormat::new(DbuPerMicron::new(3).unwrap()).is_none());
    }
}
