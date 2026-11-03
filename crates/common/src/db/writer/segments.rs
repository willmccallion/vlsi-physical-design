//! Global route segment files, as `OpenROAD`'s `write_global_route_segments`
//! writes them and `read_global_route_segments` reads them.

use crate::db::design::NetlistDB;
use crate::db::global_route::{RoutePoint, RouteSegment};
use crate::db::indices::NetId;
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// Writes the routes of `nets` to `path`.
pub fn write_file(
    db: &NetlistDB,
    nets: &[(NetId, &[RouteSegment])],
    path: &Path,
) -> io::Result<()> {
    let mut out = BufWriter::new(std::fs::File::create(path)?);
    write(db, nets, &mut out)?;
    out.flush()
}

/// Writes each net with a route: its name, then one segment per line
/// between parentheses.
pub fn write(
    db: &NetlistDB,
    nets: &[(NetId, &[RouteSegment])],
    out: &mut impl Write,
) -> io::Result<()> {
    let point = |p: &RoutePoint| {
        format!(
            "{} {} {}",
            p.at.x.get(),
            p.at.y.get(),
            db.library.layer(p.layer).name
        )
    };
    for &(net, segments) in nets {
        if segments.is_empty() {
            continue;
        }
        writeln!(out, "{}\n(", db.nets[net.index()].name)?;
        for s in segments {
            writeln!(out, "{} {}", point(&s.from), point(&s.to))?;
        }
        writeln!(out, ")")?;
    }
    Ok(())
}
