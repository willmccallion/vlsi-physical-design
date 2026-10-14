//! Bookshelf placement benchmark parser.
//!
//! Bookshelf has no technology, so the library is synthesized: one Bookshelf
//! unit is one micron at [`BOOKSHELF_DBU`] database units, every node size and
//! pin-offset combination becomes a macro, each pin is a wire-width square on
//! the bottom layer, and [`synthesize_layer_stack`] builds the metal stack
//! from the row height. The die is the bounding box of the rows and of every
//! fixed node.

use crate::db::design::{NetlistDB, PlacementStatus, Row, TrackAxis, TrackDef};
use crate::db::indices::{CellId, LayerId, MacroId, MacroPinId, SiteId};
use crate::db::library::{
    CoreKind, CutLayer, Direction, LayerRef, Library, Macro, MacroClass, MacroPin, PinDirection,
    RoutingLayer, Shape, SignalUse, Site, Symmetry, ViaDef,
};
use crate::db::parser::lexer::{ParseError, read_file};
use crate::geom::dbu::{Dbu, DbuPerMicron};
use crate::geom::orient::Orientation;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Database units per Bookshelf unit.
pub const BOOKSHELF_DBU: DbuPerMicron = match DbuPerMicron::new(100) {
    Some(units) => units,
    None => panic!("the Bookshelf resolution must be nonzero"),
};

/// Number of routing tracks per row height in the synthesized stack.
const TRACKS_PER_ROW: i64 = 16;

/// Parses a Bookshelf design from its `.aux` file.
pub fn parse(aux: &Path, num_layers: usize) -> Result<NetlistDB, ParseError> {
    let files = AuxFiles::read(aux)?;
    let units = BOOKSHELF_DBU;
    let nodes = read_nodes(&files.nodes, units)?;
    let scl = read_scl(&files.scl, units)?;
    let nets = read_nets(&files.nets, &nodes, units)?;

    let row_height = scl.row_height(&files.scl)?;
    let mut library = Library::new(units);
    synthesize_layer_stack(&mut library, row_height, num_layers).map_err(|message| {
        ParseError::Syntax {
            path: files.scl.clone(),
            line: 1,
            message,
        }
    })?;

    let name = aux
        .file_stem()
        .map_or_else(String::new, |s| s.to_string_lossy().into_owned());
    let mut db = NetlistDB::new(name, library);
    let site = scl
        .add_rows(&mut db, row_height)
        .map_err(|message| ParseError::Syntax {
            path: files.scl.clone(),
            line: 1,
            message,
        })?;
    add_cells(&mut db, &nodes, &nets, site).map_err(|message| ParseError::Syntax {
        path: files.nodes.clone(),
        line: 1,
        message,
    })?;
    read_placement(&mut db, &files.pl)?;
    add_nets(&mut db, &nets);
    db.die_area = die_area(&db);
    add_tracks(&mut db);
    Ok(db)
}

/// Updates cell locations, orientations and fixed status from a `.pl` file.
pub fn read_placement(db: &mut NetlistDB, path: &Path) -> Result<(), ParseError> {
    let units = db.dbu();
    for (line_no, parts) in data_lines(&read_file(path)?) {
        let err = |message: String| ParseError::Syntax {
            path: path.to_path_buf(),
            line: line_no,
            message,
        };
        let [name, x, y, rest @ ..] = parts.as_slice() else {
            return Err(err("expected 'name x y [: orient] [/FIXED]'".into()));
        };
        let cell = db
            .cell_id(name)
            .ok_or_else(|| err(format!("unknown node '{name}'")))?;
        let x = number(x, units).map_err(&err)?;
        let y = number(y, units).map_err(&err)?;
        let mut orient = Orientation::N;
        let mut fixed = false;
        for (i, word) in rest.iter().enumerate() {
            match *word {
                ":" => {
                    let o = rest
                        .get(i + 1)
                        .ok_or_else(|| err("missing orientation after ':'".into()))?;
                    orient = o
                        .parse()
                        .map_err(|e: crate::geom::orient::UnknownOrientation| err(e.to_string()))?;
                }
                "/FIXED" | "/FIXED_NI" => fixed = true,
                _ => {}
            }
        }
        let c = &mut db.cells[cell.index()];
        c.location = Point::new(x, y);
        c.orient = orient;
        if fixed {
            c.status = PlacementStatus::Fixed;
        } else if c.status == PlacementStatus::Unplaced {
            c.status = PlacementStatus::Placed;
        }
    }
    Ok(())
}

/// Adds `num_layers` alternating routing layers (horizontal first), the cut
/// layers between them and one default via per cut.
///
/// The pitch is a sixteenth of the row height, the wire width and spacing are
/// half a pitch, and vias are wire-width squares.
pub fn synthesize_layer_stack(
    library: &mut Library,
    row_height: Dbu,
    num_layers: usize,
) -> Result<(), String> {
    if num_layers < 2 {
        return Err(format!("need at least 2 routing layers, got {num_layers}"));
    }
    let pitch = row_height / TRACKS_PER_ROW;
    let width = pitch / 2;
    if width <= Dbu::ZERO {
        return Err(format!(
            "row height {row_height} is too small for a layer stack"
        ));
    }
    let half = width / 2;
    let square = Rect::new(
        Point::new(-half, -half),
        Point::new(width - half, width - half),
    );
    for i in 0..num_layers {
        if i > 0 {
            let cut = library
                .add_cut_layer(CutLayer {
                    name: format!("V{i}"),
                    width: Some(width),
                    spacing: Some(width),
                })
                .map_err(|e| e.to_string())?;
            let shapes = vec![
                Shape {
                    layer: LayerRef::Routing(cut.below()),
                    rect: square,
                },
                Shape {
                    layer: LayerRef::Cut(cut),
                    rect: square,
                },
                Shape {
                    layer: LayerRef::Routing(cut.above()),
                    rect: square,
                },
            ];
            let _ = library
                .add_via(ViaDef {
                    name: format!("VIA{i}{}", i + 1),
                    is_default: true,
                    cut,
                    shapes,
                    from_def: false,
                })
                .map_err(|e| e.to_string())?;
        }
        let direction = if i % 2 == 0 {
            Direction::Horizontal
        } else {
            Direction::Vertical
        };
        let _ = library
            .add_routing_layer(RoutingLayer {
                name: format!("M{}", i + 1),
                direction,
                pitch_x: pitch,
                pitch_y: pitch,
                offset_x: Some(pitch / 2),
                offset_y: Some(pitch / 2),
                width,
                min_spacing: pitch - width,
            })
            .map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn number(text: &str, units: DbuPerMicron) -> Result<Dbu, String> {
    let value: f64 = text
        .parse()
        .map_err(|_| format!("expected a number, found '{text}'"))?;
    units.to_dbu(value).map_err(|e| e.to_string())
}

/// Lines with content, split into words, skipping headers and comments.
fn data_lines(src: &str) -> Vec<(usize, Vec<&str>)> {
    src.lines()
        .enumerate()
        .filter_map(|(i, line)| {
            let line = line.trim();
            let skip = line.is_empty()
                || line.starts_with('#')
                || line.starts_with("UCLA")
                || line.starts_with("NumNodes")
                || line.starts_with("NumTerminals")
                || line.starts_with("NumNets")
                || line.starts_with("NumPins")
                || line.starts_with("NumRows");
            (!skip).then(|| (i + 1, line.split_whitespace().collect()))
        })
        .collect()
}

struct AuxFiles {
    nodes: PathBuf,
    nets: PathBuf,
    pl: PathBuf,
    scl: PathBuf,
}

impl AuxFiles {
    fn read(aux: &Path) -> Result<Self, ParseError> {
        let src = read_file(aux)?;
        let dir = aux.parent().unwrap_or_else(|| Path::new("."));
        let find = |ext: &str| {
            src.split_whitespace()
                .find(|w| Path::new(w).extension().is_some_and(|e| e == ext))
                .map(|w| dir.join(w))
                .ok_or_else(|| ParseError::Syntax {
                    path: aux.to_path_buf(),
                    line: 1,
                    message: format!("no .{ext} file listed"),
                })
        };
        Ok(Self {
            nodes: find("nodes")?,
            nets: find("nets")?,
            pl: find("pl")?,
            scl: find("scl")?,
        })
    }
}

struct Node {
    name: String,
    size: Point<Dbu>,
    terminal: bool,
}

fn read_nodes(path: &Path, units: DbuPerMicron) -> Result<Vec<Node>, ParseError> {
    let src = read_file(path)?;
    data_lines(&src)
        .into_iter()
        .map(|(line, parts)| {
            let err = |message: String| ParseError::Syntax {
                path: path.to_path_buf(),
                line,
                message,
            };
            let [name, w, h, rest @ ..] = parts.as_slice() else {
                return Err(err("expected 'name width height [terminal]'".into()));
            };
            Ok(Node {
                name: (*name).to_string(),
                size: Point::new(
                    number(w, units).map_err(&err)?,
                    number(h, units).map_err(&err)?,
                ),
                terminal: rest.iter().any(|w| w.starts_with("terminal")),
            })
        })
        .collect()
}

/// One net connection: node index, offset from the node center, direction.
struct NetPin {
    node: usize,
    offset: Point<Dbu>,
    direction: PinDirection,
}

struct BookshelfNet {
    name: String,
    pins: Vec<NetPin>,
}

fn read_nets(
    path: &Path,
    nodes: &[Node],
    units: DbuPerMicron,
) -> Result<Vec<BookshelfNet>, ParseError> {
    let src = read_file(path)?;
    let index: HashMap<&str, usize> = nodes
        .iter()
        .enumerate()
        .map(|(i, n)| (n.name.as_str(), i))
        .collect();
    let mut nets: Vec<BookshelfNet> = Vec::new();
    for (line, parts) in data_lines(&src) {
        let err = |message: String| ParseError::Syntax {
            path: path.to_path_buf(),
            line,
            message,
        };
        if parts[0] == "NetDegree" {
            let name = parts
                .get(3)
                .map_or_else(|| format!("n{}", nets.len()), |n| (*n).to_string());
            nets.push(BookshelfNet {
                name,
                pins: Vec::new(),
            });
            continue;
        }
        let net = nets
            .last_mut()
            .ok_or_else(|| err("pin before any NetDegree".into()))?;
        let node = *index
            .get(parts[0])
            .ok_or_else(|| err(format!("unknown node '{}'", parts[0])))?;
        let direction = match parts.get(1).copied() {
            Some("I") => PinDirection::Input,
            Some("O") => PinDirection::Output,
            _ => PinDirection::Inout,
        };
        let offset = match parts.as_slice() {
            [_, _, ":", dx, dy, ..] => Point::new(
                number(dx, units).map_err(&err)?,
                number(dy, units).map_err(&err)?,
            ),
            _ => Point::default(),
        };
        net.pins.push(NetPin {
            node,
            offset,
            direction,
        });
    }
    Ok(nets)
}

struct SclRow {
    y: Dbu,
    height: Dbu,
    site_width: Dbu,
    site_spacing: Dbu,
    x: Dbu,
    num_sites: u32,
}

struct Scl {
    rows: Vec<SclRow>,
}

impl Scl {
    fn row_height(&self, path: &Path) -> Result<Dbu, ParseError> {
        let first = self.rows.first().ok_or_else(|| ParseError::Syntax {
            path: path.to_path_buf(),
            line: 1,
            message: "no rows defined".into(),
        })?;
        if let Some(r) = self
            .rows
            .iter()
            .find(|r| r.height != first.height || r.site_width != first.site_width)
        {
            return Err(ParseError::Syntax {
                path: path.to_path_buf(),
                line: 1,
                message: format!(
                    "rows of different heights or site widths ({} vs {}) are not supported",
                    first.height, r.height
                ),
            });
        }
        Ok(first.height)
    }

    fn add_rows(&self, db: &mut NetlistDB, row_height: Dbu) -> Result<SiteId, String> {
        let site_width = self.rows.first().map_or(Dbu(1), |r| r.site_width);
        let site = db
            .library
            .add_site(Site {
                name: "core".into(),
                size: Point::new(site_width, row_height),
                symmetry: Symmetry {
                    x: false,
                    y: true,
                    r90: false,
                },
            })
            .map_err(|e| e.to_string())?;
        for (i, r) in self.rows.iter().enumerate() {
            db.rows.push(Row {
                name: format!("ROW_{i}"),
                site,
                origin: Point::new(r.x, r.y),
                orient: Orientation::N,
                num_x: r.num_sites,
                num_y: 1,
                step: Point::new(r.site_spacing, Dbu::ZERO),
            });
        }
        Ok(site)
    }
}

fn read_scl(path: &Path, units: DbuPerMicron) -> Result<Scl, ParseError> {
    let src = read_file(path)?;
    let mut rows = Vec::new();
    let mut current: Option<SclRow> = None;
    for (line, parts) in data_lines(&src) {
        let err = |message: String| ParseError::Syntax {
            path: path.to_path_buf(),
            line,
            message,
        };
        let value = |i: usize| -> Result<Dbu, ParseError> {
            let text = parts
                .get(i)
                .ok_or_else(|| err(format!("missing value for {}", parts[0])))?;
            number(text, units).map_err(&err)
        };
        match parts[0] {
            "CoreRow" => {
                current = Some(SclRow {
                    y: Dbu::ZERO,
                    height: Dbu::ZERO,
                    site_width: Dbu(i64::from(units.get())),
                    site_spacing: Dbu(i64::from(units.get())),
                    x: Dbu::ZERO,
                    num_sites: 0,
                });
            }
            "End" => rows.extend(current.take()),
            key => {
                let row = current
                    .as_mut()
                    .ok_or_else(|| err(format!("'{key}' outside a CoreRow")))?;
                match key {
                    "Coordinate" => row.y = value(2)?,
                    "Height" => row.height = value(2)?,
                    "Sitewidth" => row.site_width = value(2)?,
                    "Sitespacing" => row.site_spacing = value(2)?,
                    "SubrowOrigin" => {
                        row.x = value(2)?;
                        row.num_sites =
                            parts.get(5).and_then(|n| n.parse().ok()).ok_or_else(|| {
                                err("expected 'SubrowOrigin : x NumSites : n'".into())
                            })?;
                    }
                    _ => {}
                }
            }
        }
    }
    Ok(Scl { rows })
}

/// Node size, terminal flag and pin offsets: nodes that agree share a macro.
type MacroKey = (Point<Dbu>, bool, Vec<Point<Dbu>>);

/// Creates one macro per distinct node size and pin-offset list, then the cells.
fn add_cells(
    db: &mut NetlistDB,
    nodes: &[Node],
    nets: &[BookshelfNet],
    site: SiteId,
) -> Result<(), String> {
    let mut pins_of: Vec<Vec<(Point<Dbu>, PinDirection)>> = vec![Vec::new(); nodes.len()];
    for net in nets {
        for p in &net.pins {
            pins_of[p.node].push((p.offset, p.direction));
        }
    }
    let row_height = db.library.site(site).size.y;
    let pin_layer = LayerRef::Routing(LayerId(0));
    let width = db.library.layer(LayerId(0)).width;
    let half = width / 2;

    let mut macro_of: HashMap<MacroKey, MacroId> = HashMap::new();
    for (node, pins) in nodes.iter().zip(&pins_of) {
        let offsets: Vec<Point<Dbu>> = pins.iter().map(|p| p.0).collect();
        let key = (node.size, node.terminal, offsets);
        let macro_id = if let Some(&id) = macro_of.get(&key) {
            id
        } else {
            let class = if node.terminal {
                MacroClass::Pad
            } else if node.size.y == row_height {
                MacroClass::Core(CoreKind::Regular)
            } else {
                MacroClass::Block
            };
            let core_site = matches!(class, MacroClass::Core(_)).then_some(site);
            let mut m = Macro::new(
                format!("BLK{}", macro_of.len()),
                class,
                node.size,
                Symmetry::default(),
                core_site,
            );
            let center = Rect::new(Point::default(), node.size).center();
            for (k, (offset, direction)) in pins.iter().enumerate() {
                let at = center + *offset;
                let rect = Rect::new(
                    Point::new(at.x - half, at.y - half),
                    Point::new(at.x - half + width, at.y - half + width),
                );
                let _ = m.add_pin(MacroPin {
                    name: format!("p{k}"),
                    direction: *direction,
                    signal_use: SignalUse::Signal,
                    shapes: vec![Shape {
                        layer: pin_layer,
                        rect,
                    }],
                });
            }
            let id = db.library.add_macro(m).map_err(|e| e.to_string())?;
            let _ = macro_of.insert(key, id);
            id
        };
        let status = if node.terminal {
            PlacementStatus::Fixed
        } else {
            PlacementStatus::Unplaced
        };
        let _ = db
            .add_cell(
                node.name.clone(),
                macro_id,
                Point::default(),
                Orientation::N,
                status,
            )
            .map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn add_nets(db: &mut NetlistDB, nets: &[BookshelfNet]) {
    let mut next_pin = vec![0u32; db.cells.len()];
    for net in nets {
        let id = db.net_or_insert(&net.name);
        for p in &net.pins {
            let k = next_pin[p.node];
            next_pin[p.node] += 1;
            let _ = db.connect_cell_pin(CellId::new(p.node), MacroPinId(k), id);
        }
    }
}

fn die_area(db: &NetlistDB) -> Rect<Dbu> {
    let rows = db.rows.iter().map(|r| r.bbox(&db.library));
    let fixed = (0..db.cells.len())
        .filter(|&i| !db.cells[i].status.is_movable())
        .map(|i| db.cell_bbox(CellId::new(i)));
    rows.chain(fixed)
        .reduce(|a, b| a.union(&b))
        .unwrap_or_default()
}

/// Adds tracks in each layer's preferred direction across the die.
fn add_tracks(db: &mut NetlistDB) {
    let die = db.die_area;
    for (i, layer) in db.library.routing_layers.iter().enumerate() {
        let pitch = layer.track_pitch();
        let (axis, lo, extent) = match layer.direction {
            Direction::Horizontal => (TrackAxis::Y, die.min.y, die.height()),
            Direction::Vertical => (TrackAxis::X, die.min.x, die.width()),
        };
        let count = ((extent - pitch / 2).get() / pitch.get() + 1).max(0);
        db.tracks.push(TrackDef {
            axis,
            start: lo + pitch / 2,
            count: u32::try_from(count).unwrap_or(u32::MAX),
            step: pitch,
            layers: vec![LayerId::new(i)],
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthesized_stack_alternates_and_has_a_via_per_cut() {
        let mut lib = Library::new(BOOKSHELF_DBU);

        synthesize_layer_stack(&mut lib, Dbu(6400), 4).unwrap();

        let dirs: Vec<_> = lib.routing_layers.iter().map(|l| l.direction).collect();
        assert_eq!(
            dirs,
            [
                Direction::Horizontal,
                Direction::Vertical,
                Direction::Horizontal,
                Direction::Vertical
            ]
        );
        assert_eq!(lib.routing_layers[0].track_pitch(), Dbu(400));
        assert_eq!(lib.cut_layers.len(), 3);
        assert!((0..3).all(|c| lib.default_via(crate::db::indices::CutLayerId(c)).is_some()));
    }
}
