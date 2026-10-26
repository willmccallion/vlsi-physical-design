//! Global route segment files, as `OpenROAD`'s `read_global_route_segments`
//! reads them.

use crate::db::design::NetlistDB;
use crate::db::global_route::{RoutePoint, RouteSegment};
use crate::db::indices::NetId;
use crate::db::parser::lexer::{ParseError, read_file};
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;
use std::path::{Path, PathBuf};

/// Reads the routes in the file at `path`.
pub fn parse_file(
    path: &Path,
    db: &NetlistDB,
) -> Result<Vec<(NetId, Vec<RouteSegment>)>, ParseError> {
    parse_str(&read_file(path)?, path, db)
}

/// Reads routes: a net's name on a line of its own, then its segments as
/// `x y layer x y layer`, with lines of one parenthesis ignored.
pub fn parse_str(
    text: &str,
    path: &Path,
    db: &NetlistDB,
) -> Result<Vec<(NetId, Vec<RouteSegment>)>, ParseError> {
    let error = |line: usize, message: String| ParseError::Syntax {
        path: PathBuf::from(path),
        line,
        message,
    };
    let mut routes: Vec<(NetId, Vec<RouteSegment>)> = Vec::new();
    for (index, line) in text.lines().enumerate() {
        let number = index + 1;
        let tokens: Vec<&str> = line.split_whitespace().collect();
        match tokens.as_slice() {
            [] | ["(" | ")"] => {}
            [name] => {
                let net = db
                    .net_id(name)
                    .ok_or_else(|| error(number, format!("cannot find net '{name}'")))?;
                routes.push((net, Vec::new()));
            }
            [x0, y0, l0, x1, y1, l1] => {
                let point = |x: &str, y: &str, layer: &str| -> Result<RoutePoint, ParseError> {
                    let coordinate = |v: &str| {
                        v.parse::<i64>()
                            .map(Dbu)
                            .map_err(|_| error(number, format!("bad coordinate '{v}'")))
                    };
                    let layer = db
                        .library
                        .routing_layer_id(layer)
                        .ok_or_else(|| error(number, format!("cannot find layer '{layer}'")))?;
                    Ok(RoutePoint {
                        at: Point::new(coordinate(x)?, coordinate(y)?),
                        layer,
                    })
                };
                let segment = RouteSegment {
                    from: point(x0, y0, l0)?,
                    to: point(x1, y1, l1)?,
                };
                let Some((_, segments)) = routes.last_mut() else {
                    return Err(error(number, "segment before any net".into()));
                };
                segments.push(segment);
            }
            _ => return Err(error(number, format!("cannot read '{line}'"))),
        }
    }
    Ok(routes)
}
