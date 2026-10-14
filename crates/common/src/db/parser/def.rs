//! DEF (Design Exchange Format) parser.
//!
//! Reads the die area, rows, tracks, vias, components, pins, blockages,
//! special nets and nets of a design on top of a [`Library`] built from LEF.
//! `REGIONS`, `GROUPS`, `GCELLGRID`, `SCANCHAINS` and similar sections are
//! skipped, so region constraints are not honoured.

use crate::db::design::{
    NetlistDB, PlacementStatus, RouteElem, Row, SpecialNet, SpecialWire, TrackAxis, TrackDef,
};
use crate::db::indices::{LayerId, NetId, ViaId};
use crate::db::library::{LayerRef, Library, PinDirection, Shape, SignalUse};
use crate::db::parser::lef::{parse_pin_direction, parse_signal_use, via_from_shapes};
use crate::db::parser::lexer::{Lexer, ParseError, read_file};
use crate::geom::dbu::{Dbu, DbuPerMicron};
use crate::geom::orient::Orientation;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use std::collections::HashMap;
use std::path::Path;

/// Reads the `UNITS DISTANCE MICRONS` value of a DEF file.
///
/// LEF geometry must be converted at this resolution, so this runs before
/// the LEF files are parsed.
pub fn read_units(path: &Path) -> Result<DbuPerMicron, ParseError> {
    let src = read_file(path)?;
    let mut lx = Lexer::new(&src, path);
    while let Some(t) = lx.next_token() {
        match t.text {
            "UNITS" => {
                lx.expect("DISTANCE")?;
                lx.expect("MICRONS")?;
                let line = lx.line();
                let value: u32 = lx.number("database units per micron")?;
                return DbuPerMicron::new(value)
                    .ok_or_else(|| lx.error(line, "UNITS DISTANCE MICRONS must be positive"));
            }
            "COMPONENTS" | "NETS" | "PINS" => break,
            _ => {}
        }
    }
    Err(lx.error(
        1,
        "no UNITS DISTANCE MICRONS statement before the design body",
    ))
}

/// Parses a DEF file into a design over `library`.
pub fn parse(path: &Path, library: Library) -> Result<NetlistDB, ParseError> {
    parse_str(&read_file(path)?, path, library)
}

/// Parses DEF text into a design over `library`; `path` is used in errors.
pub fn parse_str(src: &str, path: &Path, library: Library) -> Result<NetlistDB, ParseError> {
    let mut parser = DefParser {
        lx: Lexer::new(src, path),
        db: NetlistDB::new(String::new(), library),
        pending_io: Vec::new(),
        pending_by_name: HashMap::new(),
    };
    parser.parse()?;
    parser.connect_remaining_io_pins()?;
    Ok(parser.db)
}

/// An I/O pin read from `PINS` but not yet connected, so that connection
/// order follows the `NETS` section.
struct PendingIo {
    line: usize,
    name: String,
    net: String,
    direction: PinDirection,
    signal_use: SignalUse,
    shapes: Vec<Shape>,
    placement: Placement,
}

struct DefParser<'a> {
    lx: Lexer<'a>,
    db: NetlistDB,
    pending_io: Vec<Option<PendingIo>>,
    pending_by_name: HashMap<String, usize>,
}

impl<'a> DefParser<'a> {
    fn parse(&mut self) -> Result<(), ParseError> {
        while let Some(t) = self.lx.next_token() {
            match t.text {
                "DESIGN" => {
                    self.db.name = self.word("design name")?.to_string();
                    self.lx.expect(";")?;
                }
                "UNITS" => self.units()?,
                "DIEAREA" => self.die_area()?,
                "ROW" => self.row()?,
                "TRACKS" => self.tracks()?,
                "VIAS" => self.section("VIAS", Self::def_via)?,
                "COMPONENTS" => self.section("COMPONENTS", Self::component)?,
                "PINS" => self.section("PINS", Self::io_pin)?,
                "BLOCKAGES" => self.section("BLOCKAGES", Self::blockage)?,
                "SPECIALNETS" => self.section("SPECIALNETS", Self::special_net)?,
                "NETS" => self.section("NETS", Self::net)?,
                "PROPERTYDEFINITIONS"
                | "REGIONS"
                | "GROUPS"
                | "SCANCHAINS"
                | "NONDEFAULTRULES"
                | "STYLES"
                | "FILLS"
                | "PINPROPERTIES"
                | "COMPONENTMASKSHIFT" => {
                    self.skip_section(t.text)?;
                }
                "END" => {
                    self.lx.expect("DESIGN")?;
                    return Ok(());
                }
                _ => self.lx.skip_statement()?,
            }
        }
        Ok(())
    }

    fn word(&mut self, what: &str) -> Result<&'a str, ParseError> {
        Ok(self.lx.expect_token(what)?.text)
    }

    fn dbu(&mut self, what: &str) -> Result<Dbu, ParseError> {
        Ok(Dbu(self.lx.number(what)?))
    }

    fn point(&mut self) -> Result<Point<Dbu>, ParseError> {
        self.lx.expect("(")?;
        let x = self.dbu("x coordinate")?;
        let y = self.dbu("y coordinate")?;
        self.lx.expect(")")?;
        Ok(Point::new(x, y))
    }

    fn rect(&mut self) -> Result<Rect<Dbu>, ParseError> {
        let a = self.point()?;
        let b = self.point()?;
        Ok(Rect::from_corners(a, b))
    }

    fn orientation(&mut self) -> Result<Orientation, ParseError> {
        let t = self.lx.expect_token("orientation")?;
        t.text
            .parse()
            .map_err(|e: crate::geom::orient::UnknownOrientation| {
                self.lx.error(t.line, e.to_string())
            })
    }

    fn routing_layer(&mut self) -> Result<LayerId, ParseError> {
        let t = self.lx.expect_token("layer name")?;
        self.db.library.routing_layer_id(t.text).ok_or_else(|| {
            self.lx
                .error(t.line, format!("unknown routing layer '{}'", t.text))
        })
    }

    fn any_layer(&mut self) -> Result<LayerRef, ParseError> {
        let t = self.lx.expect_token("layer name")?;
        self.db
            .library
            .layer_ref(t.text)
            .ok_or_else(|| self.lx.error(t.line, format!("unknown layer '{}'", t.text)))
    }

    fn via_id(&self, line: usize, name: &str) -> Result<ViaId, ParseError> {
        self.db
            .library
            .via_id(name)
            .ok_or_else(|| self.lx.error(line, format!("unknown via '{name}'")))
    }

    /// Skips the rest of a `+ KEYWORD ...` option.
    fn skip_option(&mut self) {
        while self
            .lx
            .peek()
            .is_some_and(|t| t.text != "+" && t.text != ";")
        {
            let _ = self.lx.next_token();
        }
    }

    fn skip_section(&mut self, name: &str) -> Result<(), ParseError> {
        loop {
            if self.lx.expect_token(&format!("'END {name}'"))?.text == "END" && self.lx.accept(name)
            {
                return Ok(());
            }
        }
    }

    /// Parses `n ; - item ... ; ... END name` with `item` handling each entry.
    fn section(
        &mut self,
        name: &str,
        mut item: impl FnMut(&mut Self) -> Result<(), ParseError>,
    ) -> Result<(), ParseError> {
        let _count: u64 = self.lx.number("item count")?;
        self.lx.expect(";")?;
        loop {
            let t = self.lx.expect_token(&format!("'END {name}'"))?;
            match t.text {
                "-" => item(self)?,
                "END" => {
                    self.lx.expect(name)?;
                    return Ok(());
                }
                other => {
                    return Err(self.lx.error(
                        t.line,
                        format!("expected '-' or 'END {name}', found '{other}'"),
                    ));
                }
            }
        }
    }

    fn units(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        self.lx.expect("DISTANCE")?;
        self.lx.expect("MICRONS")?;
        let value: u32 = self.lx.number("database units per micron")?;
        self.lx.expect(";")?;
        let lib = self.db.library.dbu_per_micron.get();
        if value != lib {
            return Err(self.lx.error(
                line,
                format!("DEF uses {value} units per micron but the library was built at {lib}"),
            ));
        }
        Ok(())
    }

    fn die_area(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let mut points = Vec::new();
        while !self.lx.accept(";") {
            points.push(self.point()?);
        }
        let [a, b] = points[..] else {
            return Err(self
                .lx
                .error(line, "only rectangular DIEAREA (two points) is supported"));
        };
        self.db.die_area = Rect::from_corners(a, b);
        Ok(())
    }

    fn row(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("row name")?.to_string();
        let site_name = self.word("site name")?;
        let site = self
            .db
            .library
            .site_id(site_name)
            .ok_or_else(|| self.lx.error(line, format!("unknown site '{site_name}'")))?;
        let x = self.dbu("row x")?;
        let y = self.dbu("row y")?;
        let orient = self.orientation()?;
        let size = orient.oriented_size(self.db.library.site(site).size);
        let (mut num_x, mut num_y) = (1, 1);
        let mut step = size;
        if self.lx.accept("DO") {
            num_x = self.lx.number("site count")?;
            self.lx.expect("BY")?;
            num_y = self.lx.number("site count")?;
            if self.lx.accept("STEP") {
                step = Point::new(self.dbu("step x")?, self.dbu("step y")?);
            }
        }
        self.skip_to_semicolon()?;
        self.db.rows.push(Row {
            name,
            site,
            origin: Point::new(x, y),
            orient,
            num_x,
            num_y,
            step,
        });
        Ok(())
    }

    fn skip_to_semicolon(&mut self) -> Result<(), ParseError> {
        while !self.lx.accept(";") {
            let _ = self.lx.expect_token("';'")?;
        }
        Ok(())
    }

    fn tracks(&mut self) -> Result<(), ParseError> {
        let t = self.lx.expect_token("track axis")?;
        let axis = match t.text {
            "X" => TrackAxis::X,
            "Y" => TrackAxis::Y,
            other => {
                return Err(self
                    .lx
                    .error(t.line, format!("unknown track axis '{other}'")));
            }
        };
        let start = self.dbu("track start")?;
        self.lx.expect("DO")?;
        let count = self.lx.number("track count")?;
        self.lx.expect("STEP")?;
        let step = self.dbu("track step")?;
        if self.lx.accept("MASK") {
            let _ = self.word("mask number")?;
            let _ = self.lx.accept("SAMEMASK");
        }
        let mut layers = Vec::new();
        if self.lx.accept("LAYER") {
            while !self.lx.accept(";") {
                layers.push(self.routing_layer()?);
            }
        } else {
            self.lx.expect(";")?;
        }
        self.db.tracks.push(TrackDef {
            axis,
            start,
            count,
            step,
            layers,
        });
        Ok(())
    }

    fn def_via(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("via name")?;
        let mut shapes = Vec::new();
        let mut rule = ViaRuleParams::default();
        while self.lx.accept("+") {
            let t = self.lx.expect_token("via option")?;
            match t.text {
                "RECT" => {
                    let layer = self.any_layer()?;
                    if self.lx.accept("+") {
                        self.lx.expect("MASK")?;
                        let _ = self.word("mask number")?;
                    }
                    let rect = self.rect()?;
                    shapes.push(Shape { layer, rect });
                }
                "VIARULE" => {
                    let _ = self.word("via rule name")?;
                    rule.used = true;
                }
                "CUTSIZE" => rule.cut_size = self.dbu_pair("cut size")?,
                "LAYERS" => {
                    let layers = [self.any_layer()?, self.any_layer()?, self.any_layer()?];
                    let (cuts, metals): (Vec<LayerRef>, Vec<LayerRef>) =
                        layers.iter().partition(|l| matches!(l, LayerRef::Cut(_)));
                    let mut metals: Vec<LayerId> = metals
                        .into_iter()
                        .filter_map(|l| match l {
                            LayerRef::Routing(id) => Some(id),
                            LayerRef::Cut(_) => None,
                        })
                        .collect();
                    metals.sort();
                    let (&[cut], &[bottom, top]) = (cuts.as_slice(), metals.as_slice()) else {
                        return Err(self.lx.error(
                            t.line,
                            format!("via '{name}': + LAYERS needs one cut and two metal layers"),
                        ));
                    };
                    if layers != [LayerRef::Routing(bottom), cut, LayerRef::Routing(top)] {
                        log::warn!(
                            "{}:{}: via '{name}' lists + LAYERS out of bottom/cut/top order; \
                             using the cut layer's neighbours",
                            self.lx.path().display(),
                            t.line
                        );
                    }
                    rule.cut = Some(cut);
                    rule.bottom = Some(bottom);
                    rule.top = Some(top);
                }
                "CUTSPACING" => rule.cut_spacing = self.dbu_pair("cut spacing")?,
                "ENCLOSURE" => {
                    rule.bottom_enclosure = self.dbu_pair("enclosure")?;
                    rule.top_enclosure = self.dbu_pair("enclosure")?;
                }
                "ROWCOL" => {
                    rule.rows = self.lx.number("row count")?;
                    rule.cols = self.lx.number("column count")?;
                }
                "ORIGIN" => rule.origin = self.dbu_pair("origin")?,
                "OFFSET" => {
                    rule.bottom_offset = self.dbu_pair("offset")?;
                    rule.top_offset = self.dbu_pair("offset")?;
                }
                other => {
                    return Err(self.lx.error(
                        t.line,
                        format!("via '{name}': unsupported option '{other}'"),
                    ));
                }
            }
        }
        self.lx.expect(";")?;
        if rule.used {
            let generated = rule
                .shapes()
                .map_err(|m| self.lx.error(line, format!("via '{name}': {m}")))?;
            shapes.extend(generated);
        }
        let via = via_from_shapes(name, false, shapes, true).map_err(|m| self.lx.error(line, m))?;
        let _ = self
            .db
            .library
            .add_via(via)
            .map_err(|e| self.lx.error(line, e.to_string()))?;
        Ok(())
    }

    fn dbu_pair(&mut self, what: &str) -> Result<Point<Dbu>, ParseError> {
        Ok(Point::new(self.dbu(what)?, self.dbu(what)?))
    }

    /// Reads the rest of `+ PLACED|FIXED|COVER ( x y ) orient` or `+ UNPLACED`.
    fn placement(&mut self, keyword: &str) -> Result<Placement, ParseError> {
        let status = match keyword {
            "PLACED" => PlacementStatus::Placed,
            "FIXED" => PlacementStatus::Fixed,
            "COVER" => PlacementStatus::Cover,
            _ => return Ok(Placement::default()),
        };
        let location = self.point()?;
        let orient = self.orientation()?;
        Ok(Placement {
            status,
            location,
            orient,
        })
    }

    fn component(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("component name")?.to_string();
        let macro_name = self.word("macro name")?;
        let macro_id = self.db.library.macro_id(macro_name).ok_or_else(|| {
            self.lx.error(
                line,
                format!("component '{name}' uses macro '{macro_name}', which no LEF defines"),
            )
        })?;
        let mut placement = Placement::default();
        while self.lx.accept("+") {
            let t = self.lx.expect_token("component option")?;
            match t.text {
                "PLACED" | "FIXED" | "COVER" | "UNPLACED" => placement = self.placement(t.text)?,
                _ => self.skip_option(),
            }
        }
        self.lx.expect(";")?;
        let _ = self
            .db
            .add_cell(
                name,
                macro_id,
                placement.location,
                placement.orient,
                placement.status,
            )
            .map_err(|e| self.lx.error(line, e.to_string()))?;
        Ok(())
    }

    fn io_pin(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("pin name")?.to_string();
        let mut net = None;
        let mut direction = PinDirection::default();
        let mut signal_use = SignalUse::Signal;
        let mut shapes = Vec::new();
        let mut placement = Placement::default();
        let mut ports = 0;
        while self.lx.accept("+") {
            let t = self.lx.expect_token("pin option")?;
            match t.text {
                "NET" => net = Some(self.word("net name")?),
                "DIRECTION" => {
                    let word = self.word("direction")?;
                    direction = parse_pin_direction(word).ok_or_else(|| {
                        self.lx.error(t.line, format!("unknown direction '{word}'"))
                    })?;
                }
                "USE" => {
                    let word = self.word("use")?;
                    signal_use = parse_signal_use(word)
                        .ok_or_else(|| self.lx.error(t.line, format!("unknown use '{word}'")))?;
                }
                "PORT" => {
                    ports += 1;
                    if ports > 1 {
                        return Err(self.lx.error(
                            t.line,
                            format!("pin '{name}': multiple PORTs are not supported"),
                        ));
                    }
                }
                "LAYER" => {
                    let layer = self.any_layer()?;
                    while self.lx.peek().is_some_and(|t| t.text != "(") {
                        let _ = self.lx.next_token();
                    }
                    let rect = self.rect()?;
                    shapes.push(Shape { layer, rect });
                }
                "PLACED" | "FIXED" | "COVER" | "UNPLACED" => placement = self.placement(t.text)?,
                "POLYGON" | "VIA" => {
                    return Err(self.lx.error(
                        t.line,
                        format!("pin '{name}': {} geometry is not supported", t.text),
                    ));
                }
                _ => self.skip_option(),
            }
        }
        self.lx.expect(";")?;
        let net = net.ok_or_else(|| self.lx.error(line, format!("pin '{name}' has no NET")))?;
        if self.pending_by_name.contains_key(&name) || self.db.io_pin_id(&name).is_some() {
            return Err(self.lx.error(line, format!("duplicate pin '{name}'")));
        }
        let _ = self
            .pending_by_name
            .insert(name.clone(), self.pending_io.len());
        self.pending_io.push(Some(PendingIo {
            line,
            name,
            net: net.to_string(),
            direction,
            signal_use,
            shapes,
            placement,
        }));
        Ok(())
    }

    fn connect_io_pin(&mut self, pending: PendingIo) -> Result<(), ParseError> {
        let net = self.db.net_or_insert(&pending.net);
        if pending.signal_use.is_supply() {
            self.db.nets[net.index()].signal_use = pending.signal_use;
        }
        let line = pending.line;
        let _ = self
            .db
            .add_io_pin(
                pending.name,
                net,
                pending.direction,
                pending.signal_use,
                pending.shapes,
                pending.placement.location,
                pending.placement.orient,
                pending.placement.status,
            )
            .map_err(|e| self.lx.error(line, e.to_string()))?;
        Ok(())
    }

    /// Connects pins that no `NETS` entry mentioned, in `PINS` order.
    fn connect_remaining_io_pins(&mut self) -> Result<(), ParseError> {
        let pending = std::mem::take(&mut self.pending_io);
        for p in pending.into_iter().flatten() {
            self.connect_io_pin(p)?;
        }
        Ok(())
    }

    fn blockage(&mut self) -> Result<(), ParseError> {
        let t = self.lx.expect_token("blockage kind")?;
        match t.text {
            "LAYER" => {
                let layer = self.any_layer()?;
                while self.lx.accept("+") {
                    self.skip_option();
                }
                while self.lx.accept("RECT") {
                    let rect = self.rect()?;
                    self.db.routing_blockages.push(Shape { layer, rect });
                }
            }
            "PLACEMENT" => {
                let mut hard = true;
                while self.lx.accept("+") {
                    let opt = self.word("blockage option")?;
                    hard &= opt != "SOFT" && opt != "PARTIAL";
                    self.skip_option();
                }
                while self.lx.accept("RECT") {
                    let rect = self.rect()?;
                    if hard {
                        self.db.placement_blockages.push(rect);
                    } else {
                        log::warn!("Ignoring a soft or partial placement blockage at {rect:?}");
                    }
                }
            }
            other => {
                return Err(self
                    .lx
                    .error(t.line, format!("unknown blockage kind '{other}'")));
            }
        }
        if let Some(t) = self.lx.peek()
            && t.text == "POLYGON"
        {
            return Err(self.lx.error(t.line, "POLYGON blockages are not supported"));
        }
        self.lx.expect(";")
    }

    /// Reads a path point, where `*` repeats the previous coordinate.
    fn path_point(&mut self, prev: Option<Point<Dbu>>) -> Result<Point<Dbu>, ParseError> {
        let line = self.lx.line();
        self.lx.expect("(")?;
        let x = self.path_coord(prev.map(|p| p.x))?;
        let y = self.path_coord(prev.map(|p| p.y))?;
        if !self.lx.accept(")") {
            return Err(self.lx.error(line, "wire end extensions are not supported"));
        }
        Ok(Point::new(x, y))
    }

    fn path_coord(&mut self, prev: Option<Dbu>) -> Result<Dbu, ParseError> {
        let t = self.lx.expect_token("coordinate")?;
        if t.text == "*" {
            return prev.ok_or_else(|| self.lx.error(t.line, "'*' with no previous point"));
        }
        t.text.parse().map(Dbu).map_err(|_| {
            self.lx
                .error(t.line, format!("expected a coordinate, found '{}'", t.text))
        })
    }

    fn special_net(&mut self) -> Result<(), ParseError> {
        let name = self.word("net name")?.to_string();
        let mut connections = Vec::new();
        while self.lx.accept("(") {
            let owner = self.word("component name")?.to_string();
            let pin = self.word("pin name")?.to_string();
            while self.lx.expect_token("')'")?.text != ")" {}
            connections.push((owner, pin));
        }
        let mut net = SpecialNet {
            name,
            signal_use: SignalUse::Signal,
            connections,
            wires: Vec::new(),
            vias: Vec::new(),
        };
        while self.lx.accept("+") {
            let t = self.lx.expect_token("special net option")?;
            match t.text {
                "USE" => {
                    let word = self.word("use")?;
                    net.signal_use = parse_signal_use(word)
                        .ok_or_else(|| self.lx.error(t.line, format!("unknown use '{word}'")))?;
                }
                "ROUTED" | "FIXED" | "COVER" | "SHIELD" => {
                    if t.text == "SHIELD" {
                        let _ = self.word("shielded net")?;
                    }
                    self.special_wiring(&mut net)?;
                }
                "RECT" | "POLYGON" | "VIA" => {
                    return Err(self.lx.error(
                        t.line,
                        format!("special net '{}': + {} is not supported", net.name, t.text),
                    ));
                }
                _ => self.skip_option(),
            }
        }
        self.lx.expect(";")?;
        self.db.special_nets.push(net);
        Ok(())
    }

    /// Reads `layer width [+ SHAPE s] [+ STYLE n] path [NEW layer width ...]*`.
    fn special_wiring(&mut self, net: &mut SpecialNet) -> Result<(), ParseError> {
        loop {
            let layer = self.routing_layer()?;
            let width = self.dbu("wire width")?;
            let mut shape = None;
            while self.lx.accept("+") {
                let t = self.lx.expect_token("wire option")?;
                match t.text {
                    "SHAPE" => shape = Some(self.word("shape")?.to_string()),
                    "STYLE" | "MASK" => {
                        let _ = self.word("value")?;
                    }
                    other => {
                        return Err(self
                            .lx
                            .error(t.line, format!("unsupported special wire option '{other}'")));
                    }
                }
            }
            let mut prev: Option<Point<Dbu>> = None;
            while let Some(t) = self.lx.peek() {
                match t.text {
                    "(" => {
                        let p = self.path_point(prev)?;
                        if let Some(from) = prev {
                            net.wires.push(SpecialWire {
                                layer,
                                width,
                                from,
                                to: p,
                                shape: shape.clone(),
                            });
                        }
                        prev = Some(p);
                    }
                    "NEW" | "+" | ";" => break,
                    _ => {
                        let t = self.lx.expect_token("via name")?;
                        let via = self.via_id(t.line, t.text)?;
                        let at =
                            prev.ok_or_else(|| self.lx.error(t.line, "via before any point"))?;
                        if self.lx.peek().is_some_and(|n| n.text == "DO") {
                            return Err(self
                                .lx
                                .error(t.line, "via arrays (DO ... BY) are not supported"));
                        }
                        net.vias.push((via, at));
                    }
                }
            }
            if !self.lx.accept("NEW") {
                return Ok(());
            }
        }
    }

    fn net(&mut self) -> Result<(), ParseError> {
        let name = self.word("net name")?;
        let net = self.db.net_or_insert(name);
        while self.lx.accept("(") {
            self.net_connection(net)?;
        }
        while self.lx.accept("+") {
            let t = self.lx.expect_token("net option")?;
            match t.text {
                "USE" => {
                    let word = self.word("use")?;
                    self.db.nets[net.index()].signal_use = parse_signal_use(word)
                        .ok_or_else(|| self.lx.error(t.line, format!("unknown use '{word}'")))?;
                }
                "WEIGHT" => self.db.nets[net.index()].weight = self.lx.number("net weight")?,
                "ROUTED" | "FIXED" | "COVER" | "NOSHIELD" => self.net_wiring(net)?,
                _ => self.skip_option(),
            }
        }
        self.lx.expect(";")
    }

    fn net_connection(&mut self, net: NetId) -> Result<(), ParseError> {
        let line = self.lx.line();
        let owner = self.word("component name")?;
        let pin_name = self.word("pin name")?;
        while self.lx.expect_token("')'")?.text != ")" {}
        let net_name = self.db.nets[net.index()].name.clone();
        if owner == "PIN" {
            let pending = self
                .pending_by_name
                .get(pin_name)
                .and_then(|&i| self.pending_io[i].take())
                .ok_or_else(|| {
                    self.lx.error(
                        line,
                        format!(
                            "net '{net_name}' uses unknown or already connected pin '{pin_name}'"
                        ),
                    )
                })?;
            if pending.net != net_name {
                return Err(self.lx.error(
                    line,
                    format!(
                        "pin '{pin_name}' is on net '{}', not '{net_name}'",
                        pending.net
                    ),
                ));
            }
            return self.connect_io_pin(pending);
        }
        if owner == "*" {
            return Err(self.lx.error(
                line,
                format!("net '{net_name}': '*' connections are not supported"),
            ));
        }
        let cell = self.db.cell_id(owner).ok_or_else(|| {
            self.lx.error(
                line,
                format!("net '{net_name}' uses unknown component '{owner}'"),
            )
        })?;
        let m = self.db.cell_macro(cell);
        let pin = m.pin_id(pin_name).ok_or_else(|| {
            self.lx.error(
                line,
                format!(
                    "net '{net_name}' uses pin '{pin_name}', which macro '{}' does not have",
                    m.name
                ),
            )
        })?;
        if m.pin(pin).reference_point().is_none() {
            return Err(self.lx.error(
                line,
                format!(
                    "net '{net_name}' uses pin {owner}/{pin_name}, which has no routing geometry"
                ),
            ));
        }
        let _ = self.db.connect_cell_pin(cell, pin, net);
        Ok(())
    }

    /// Reads regular wiring: `layer path [NEW layer path]*`.
    fn net_wiring(&mut self, net: NetId) -> Result<(), ParseError> {
        loop {
            let mut layer = self.routing_layer()?;
            let mut prev: Option<Point<Dbu>> = None;
            while let Some(t) = self.lx.peek() {
                match t.text {
                    "(" => {
                        let p = self.path_point(prev)?;
                        if let Some(from) = prev {
                            self.db.nets[net.index()].routing.push(RouteElem::Wire {
                                layer,
                                from,
                                to: p,
                            });
                        }
                        prev = Some(p);
                    }
                    "NEW" | "+" | ";" => break,
                    "TAPER" => {
                        let _ = self.lx.next_token();
                    }
                    "TAPERRULE" | "STYLE" | "MASK" => {
                        let _ = self.lx.next_token();
                        let _ = self.word("value")?;
                    }
                    "VIRTUAL" | "RECT" => {
                        return Err(self
                            .lx
                            .error(t.line, format!("{} wiring is not supported", t.text)));
                    }
                    _ => {
                        let t = self.lx.expect_token("via name")?;
                        let via = self.via_id(t.line, t.text)?;
                        let at =
                            prev.ok_or_else(|| self.lx.error(t.line, "via before any point"))?;
                        let def = self.db.library.via(via);
                        layer = if def.bottom() == layer {
                            def.top()
                        } else if def.top() == layer {
                            def.bottom()
                        } else {
                            return Err(self.lx.error(
                                t.line,
                                format!("via '{}' does not touch the current wiring layer", t.text),
                            ));
                        };
                        self.db.nets[net.index()]
                            .routing
                            .push(RouteElem::Via { via, at });
                    }
                }
            }
            if !self.lx.accept("NEW") {
                return Ok(());
            }
        }
    }
}

/// Where and how a component or pin is placed.
#[derive(Default)]
struct Placement {
    status: PlacementStatus,
    location: Point<Dbu>,
    orient: Orientation,
}

/// Parameters of a DEF via generated from a `VIARULE`.
#[derive(Default)]
struct ViaRuleParams {
    used: bool,
    cut_size: Point<Dbu>,
    bottom: Option<LayerId>,
    cut: Option<LayerRef>,
    top: Option<LayerId>,
    cut_spacing: Point<Dbu>,
    bottom_enclosure: Point<Dbu>,
    top_enclosure: Point<Dbu>,
    rows: i64,
    cols: i64,
    origin: Point<Dbu>,
    bottom_offset: Point<Dbu>,
    top_offset: Point<Dbu>,
}

impl ViaRuleParams {
    /// The cut array and its enclosing metal, centered on the via origin.
    fn shapes(&self) -> Result<Vec<Shape>, String> {
        let (Some(bottom), Some(cut), Some(top)) = (self.bottom, self.cut, self.top) else {
            return Err("a VIARULE via needs + LAYERS".into());
        };
        let rows = self.rows.max(1);
        let cols = self.cols.max(1);
        let array = Point::new(
            self.cut_size.x * cols + self.cut_spacing.x * (cols - 1),
            self.cut_size.y * rows + self.cut_spacing.y * (rows - 1),
        );
        let lo = Point::new(Dbu(-(array.x.0 / 2)), Dbu(-(array.y.0 / 2)));
        let array_rect = Rect::new(lo, lo + array);

        let mut shapes = Vec::new();
        for r in 0..rows {
            for c in 0..cols {
                let min = lo
                    + Point::new(
                        (self.cut_size.x + self.cut_spacing.x) * c,
                        (self.cut_size.y + self.cut_spacing.y) * r,
                    );
                shapes.push(Shape {
                    layer: cut,
                    rect: Rect::new(min, min + self.cut_size).translate(self.origin),
                });
            }
        }
        let enclose = |enc: Point<Dbu>, offset: Point<Dbu>| {
            Rect::new(array_rect.min - enc, array_rect.max + enc).translate(self.origin + offset)
        };
        shapes.push(Shape {
            layer: LayerRef::Routing(bottom),
            rect: enclose(self.bottom_enclosure, self.bottom_offset),
        });
        shapes.push(Shape {
            layer: LayerRef::Routing(top),
            rect: enclose(self.top_enclosure, self.top_offset),
        });
        Ok(shapes)
    }
}
