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

    for g in &db.gcell_grid {
        writeln!(
            out,
            "GCELLGRID {} {} DO {} STEP {} ;",
            g.axis.as_str(),
            g.start,
            g.count,
            g.step
        )?;
    }

    write_vias(lib, out)?;
    write_non_default_rules(db, out)?;
    write_regions(db, out)?;
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

/// A path point, with its end extension when it has one.
fn path_point(p: Point<Dbu>, extension: Dbu) -> String {
    if extension == Dbu::ZERO {
        pt(p)
    } else {
        format!("( {} {} {extension} )", p.x, p.y)
    }
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

fn write_non_default_rules(db: &NetlistDB, out: &mut impl Write) -> io::Result<()> {
    if db.non_default_rules.is_empty() {
        return Ok(());
    }
    let lib = &db.library;
    writeln!(out, "NONDEFAULTRULES {} ;", db.non_default_rules.len())?;
    for r in &db.non_default_rules {
        write!(out, "- {}", r.name)?;
        if r.hard_spacing {
            write!(out, "\n  + HARDSPACING")?;
        }
        for l in &r.layers {
            write!(
                out,
                "\n  + LAYER {} WIDTH {}",
                lib.layer(l.layer).name,
                l.width
            )?;
            if let Some(s) = l.spacing {
                write!(out, " SPACING {s}")?;
            }
            if let Some(e) = l.wire_extension {
                write!(out, " WIREEXT {e}")?;
            }
        }
        for &v in &r.vias {
            write!(out, "\n  + VIA {}", lib.via(v).name)?;
        }
        for &v in &r.via_rules {
            write!(out, "\n  + VIARULE {}", lib.via_rules[v.index()].name)?;
        }
        for &(cut, n) in &r.min_cuts {
            write!(
                out,
                "\n  + MINCUTS {} {n}",
                lib.cut_layers[cut.index()].name
            )?;
        }
        writeln!(out, " ;")?;
    }
    writeln!(out, "END NONDEFAULTRULES")
}

fn write_regions(db: &NetlistDB, out: &mut impl Write) -> io::Result<()> {
    if db.regions.is_empty() {
        return Ok(());
    }
    writeln!(out, "REGIONS {} ;", db.regions.len())?;
    for r in &db.regions {
        write!(out, "- {}", r.name)?;
        for rect in &r.rects {
            write!(out, " {} {}", pt(rect.min), pt(rect.max))?;
        }
        if let Some(kind) = r.kind.def_keyword() {
            write!(out, " + TYPE {kind}")?;
        }
        writeln!(out, " ;")?;
    }
    writeln!(out, "END REGIONS")
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
        if let Some(r) = c.region {
            write!(out, " + REGION {}", db.regions[r.index()].name)?;
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
            let (from_ext, to_ext) = w.extensions;
            write!(
                out,
                " {} {}",
                path_point(w.from, from_ext),
                path_point(w.to, to_ext)
            )?;
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
        for s in &n.rects {
            write!(
                out,
                "\n  + RECT {} {} {}",
                lib.layer_name(s.layer),
                pt(s.rect.min),
                pt(s.rect.max)
            )?;
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
        if let Some(r) = net.non_default_rule {
            let _ = write!(
                line,
                " + NONDEFAULTRULE {}",
                db.non_default_rules[r.index()].name
            );
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
