//! DEF writer.

use crate::db::design::{NetlistDB, PinOwner, RouteElem};
use crate::db::library::{LayerRef, Library};
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;
use std::fmt::Write as _;
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// Writes the design as DEF 5.8 to `path`.
pub fn write_file(db: &NetlistDB, path: &Path) -> io::Result<()> {
    let file = std::fs::File::create(path)?;
    let mut out = BufWriter::with_capacity(1 << 20, file);
    write(db, &mut out)?;
    out.flush()
}

/// Writes the design as DEF 5.8.
pub fn write(db: &NetlistDB, out: &mut impl Write) -> io::Result<()> {
    let lib = &db.library;
    writeln!(out, "VERSION 5.8 ;")?;
    writeln!(out, "DIVIDERCHAR \"/\" ;")?;
    writeln!(out, "BUSBITCHARS \"[]\" ;")?;
    writeln!(out, "DESIGN {} ;", db.name)?;
    writeln!(out, "UNITS DISTANCE MICRONS {} ;", lib.dbu_per_micron.get())?;
    writeln!(
        out,
        "DIEAREA {} {} ;",
        pt(db.die_area.min),
        pt(db.die_area.max)
    )?;

    for r in &db.rows {
        writeln!(
            out,
            "ROW {} {} {} {} {} DO {} BY {} STEP {} {} ;",
            r.name,
            lib.site(r.site).name,
            r.origin.x,
            r.origin.y,
            r.orient,
            r.num_x,
            r.num_y,
            r.step.x,
            r.step.y
        )?;
    }

    for t in &db.tracks {
        write!(
            out,
            "TRACKS {} {} DO {} STEP {}",
            t.axis.as_str(),
            t.start,
            t.count,
            t.step
        )?;
        if !t.layers.is_empty() {
            write!(out, " LAYER")?;
            for &l in &t.layers {
                write!(out, " {}", lib.layer(l).name)?;
            }
        }
        writeln!(out, " ;")?;
    }

    write_vias(lib, out)?;
    write_components(db, out)?;
    write_pins(db, out)?;
    write_blockages(db, out)?;
    write_special_nets(db, out)?;
    write_nets(db, out)?;
    writeln!(out, "END DESIGN")
}

fn pt(p: Point<Dbu>) -> String {
    format!("( {} {} )", p.x, p.y)
}

fn write_vias(lib: &Library, out: &mut impl Write) -> io::Result<()> {
    let vias: Vec<_> = lib.vias.iter().filter(|v| v.from_def).collect();
    if vias.is_empty() {
        return Ok(());
    }
    writeln!(out, "VIAS {} ;", vias.len())?;
    for v in vias {
        write!(out, "- {}", v.name)?;
        for s in &v.shapes {
            write!(
                out,
                " + RECT {} {} {}",
                lib.layer_name(s.layer),
                pt(s.rect.min),
                pt(s.rect.max)
            )?;
        }
        writeln!(out, " ;")?;
    }
    writeln!(out, "END VIAS")
}

fn write_components(db: &NetlistDB, out: &mut impl Write) -> io::Result<()> {
    writeln!(out, "COMPONENTS {} ;", db.cells.len())?;
    for c in &db.cells {
        write!(
            out,
            "- {} {}",
            c.name,
            db.library.cell_macro(c.macro_id).name
        )?;
        if let Some(kw) = c.status.def_keyword() {
            write!(out, " + {kw} {} {}", pt(c.location), c.orient)?;
        }
        writeln!(out, " ;")?;
    }
    writeln!(out, "END COMPONENTS")
}

fn write_pins(db: &NetlistDB, out: &mut impl Write) -> io::Result<()> {
    if db.io_pins.is_empty() {
        return Ok(());
    }
    writeln!(out, "PINS {} ;", db.io_pins.len())?;
    for p in &db.io_pins {
        write!(
            out,
            "- {} + NET {} + DIRECTION {} + USE {}",
            p.name,
            db.nets[p.net.index()].name,
            p.direction.as_str(),
            p.signal_use.as_str()
        )?;
        for s in &p.shapes {
            write!(
                out,
                "\n  + LAYER {} {} {}",
                db.library.layer_name(s.layer),
                pt(s.rect.min),
                pt(s.rect.max)
            )?;
        }
        if let Some(kw) = p.status.def_keyword() {
            write!(out, "\n  + {kw} {} {}", pt(p.location), p.orient)?;
        }
        writeln!(out, " ;")?;
    }
    writeln!(out, "END PINS")
}

fn write_blockages(db: &NetlistDB, out: &mut impl Write) -> io::Result<()> {
    let count = db.routing_blockages.len() + db.placement_blockages.len();
    if count == 0 {
        return Ok(());
    }
    writeln!(out, "BLOCKAGES {count} ;")?;
    for s in &db.routing_blockages {
        writeln!(
            out,
            "- LAYER {} RECT {} {} ;",
            db.library.layer_name(s.layer),
            pt(s.rect.min),
            pt(s.rect.max)
        )?;
    }
    for r in &db.placement_blockages {
        writeln!(out, "- PLACEMENT RECT {} {} ;", pt(r.min), pt(r.max))?;
    }
    writeln!(out, "END BLOCKAGES")
}

fn write_special_nets(db: &NetlistDB, out: &mut impl Write) -> io::Result<()> {
    if db.special_nets.is_empty() {
        return Ok(());
    }
    let lib = &db.library;
    writeln!(out, "SPECIALNETS {} ;", db.special_nets.len())?;
    for n in &db.special_nets {
        write!(out, "- {}", n.name)?;
        for (owner, pin) in &n.connections {
            write!(out, " ( {owner} {pin} )")?;
        }
        write!(out, " + USE {}", n.signal_use.as_str())?;
        let mut keyword = "+ ROUTED";
        for w in &n.wires {
            write!(out, "\n  {keyword} {} {}", lib.layer(w.layer).name, w.width)?;
            if let Some(shape) = &w.shape {
                write!(out, " + SHAPE {shape}")?;
            }
            write!(out, " {} {}", pt(w.from), pt(w.to))?;
            keyword = "NEW";
        }
        for &(via, at) in &n.vias {
            let v = lib.via(via);
            write!(
                out,
                "\n  {keyword} {} 0 {} {}",
                lib.layer(v.bottom()).name,
                pt(at),
                v.name
            )?;
            keyword = "NEW";
        }
        writeln!(out, " ;")?;
    }
    writeln!(out, "END SPECIALNETS")
}

fn write_nets(db: &NetlistDB, out: &mut impl Write) -> io::Result<()> {
    let lib = &db.library;
    writeln!(out, "NETS {} ;", db.nets.len())?;
    let mut line = String::new();
    for net in &db.nets {
        line.clear();
        let _ = write!(line, "- {}", net.name);
        for &pin in &net.pins {
            match db.pins[pin.index()].owner {
                PinOwner::Cell { cell, pin } => {
                    let c = &db.cells[cell.index()];
                    let name = &lib.cell_macro(c.macro_id).pin(pin).name;
                    let _ = write!(line, " ( {} {} )", c.name, name);
                }
                PinOwner::Io(io) => {
                    let _ = write!(line, " ( PIN {} )", db.io_pins[io.index()].name);
                }
            }
        }
        let _ = write!(line, " + USE {}", net.signal_use.as_str());
        if (net.weight - 1.0).abs() > f64::EPSILON {
            let _ = write!(line, " + WEIGHT {}", net.weight);
        }
        let mut keyword = "+ ROUTED";
        for elem in &net.routing {
            match *elem {
                RouteElem::Wire { layer, from, to } => {
                    let _ = write!(
                        line,
                        "\n  {keyword} {} {} {}",
                        lib.layer(layer).name,
                        pt(from),
                        pt(to)
                    );
                }
                RouteElem::Via { via, at } => {
                    let v = lib.via(via);
                    let _ = write!(
                        line,
                        "\n  {keyword} {} {} {}",
                        lib.layer_name(LayerRef::Routing(v.bottom())),
                        pt(at),
                        v.name
                    );
                }
            }
            keyword = "NEW";
        }
        writeln!(out, "{line} ;")?;
    }
    writeln!(out, "END NETS")
}
