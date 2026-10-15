//! LEF (Library Exchange Format) parser.
//!
//! Reads routing and cut layers, fixed vias, sites and cell masters into a
//! [`Library`]. Masterslice, overlap and implant layers are recognised and
//! their shapes dropped; `VIARULE`, `NONDEFAULTRULE`, electrical and antenna
//! data are skipped.

use crate::db::library::{
    CoreKind, CutLayer, Direction, LayerRef, Library, Macro, MacroClass, MacroPin, PinDirection,
    RoutingLayer, Shape, SignalUse, Site, SpacingTable, Symmetry, ViaDef,
};
use crate::db::parser::lexer::{Lexer, ParseError, read_file};
use crate::geom::dbu::{Dbu, DbuPerMicron};
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use std::collections::HashSet;
use std::path::PathBuf;

/// Parses LEF files in order into one library at the given resolution.
///
/// The technology LEF must come before the cell LEFs that use its layers.
pub fn parse_files(paths: &[PathBuf], dbu: DbuPerMicron) -> Result<Library, ParseError> {
    let mut library = Library::new(dbu);
    let mut ignored_layers = HashSet::new();
    let mut last_layer = None;
    for path in paths {
        let src = read_file(path)?;
        let mut parser = LefParser {
            lx: Lexer::new(&src, path),
            lib: &mut library,
            dbu,
            ignored_layers: &mut ignored_layers,
            last_layer: &mut last_layer,
        };
        parser.parse()?;
    }
    if library.routing_layers.is_empty() {
        return Err(ParseError::Syntax {
            path: paths.first().cloned().unwrap_or_default(),
            line: 1,
            message: "no routing layers defined".into(),
        });
    }
    Ok(library)
}

/// The kind of the most recently defined layer, to check the stack alternates.
#[derive(Clone, Copy, PartialEq, Eq)]
enum StackKind {
    Routing,
    Cut,
}

struct LefParser<'a, 'l> {
    lx: Lexer<'a>,
    lib: &'l mut Library,
    dbu: DbuPerMicron,
    ignored_layers: &'l mut HashSet<String>,
    last_layer: &'l mut Option<StackKind>,
}

impl<'a> LefParser<'a, '_> {
    fn parse(&mut self) -> Result<(), ParseError> {
        while let Some(t) = self.lx.next_token() {
            match t.text {
                "LAYER" => self.layer()?,
                "VIA" => self.via()?,
                "SITE" => self.site()?,
                "MACRO" => self.cell_macro()?,
                "UNITS" | "PROPERTYDEFINITIONS" | "SPACING" => self.skip_block(t.text)?,
                "VIARULE" | "NONDEFAULTRULE" => {
                    let name = self.word("rule name")?;
                    self.skip_block(name)?;
                }
                "END" => {
                    if self.lx.accept("LIBRARY") {
                        return Ok(());
                    }
                    return Err(self.lx.error(t.line, "unexpected END"));
                }
                _ => self.lx.skip_statement()?,
            }
        }
        Ok(())
    }

    fn word(&mut self, what: &str) -> Result<&'a str, ParseError> {
        Ok(self.lx.expect_token(what)?.text)
    }

    fn microns(&mut self, what: &str) -> Result<Dbu, ParseError> {
        let line = self.lx.line();
        let value: f64 = self.lx.number(what)?;
        self.dbu
            .to_dbu(value)
            .map_err(|e| self.lx.error(line, e.to_string()))
    }

    fn point(&mut self) -> Result<Point<Dbu>, ParseError> {
        let x = self.microns("x coordinate")?;
        let y = self.microns("y coordinate")?;
        Ok(Point::new(x, y))
    }

    fn rect(&mut self) -> Result<Rect<Dbu>, ParseError> {
        let a = self.point()?;
        let b = self.point()?;
        Ok(Rect::from_corners(a, b))
    }

    /// Skips to the `END name` that closes a block.
    fn skip_block(&mut self, name: &str) -> Result<(), ParseError> {
        loop {
            if self.lx.expect_token(&format!("'END {name}'"))?.text == "END" && self.lx.accept(name)
            {
                return Ok(());
            }
        }
    }

    fn end_block(&mut self, name: &str) -> Result<(), ParseError> {
        let line = self.lx.line();
        let found = self.word("block name")?;
        if found == name {
            Ok(())
        } else {
            Err(self
                .lx
                .error(line, format!("expected 'END {name}', found 'END {found}'")))
        }
    }

    fn layer(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("layer name")?;
        let mut kind = None;
        let mut direction = None;
        let mut pitch = None;
        let mut offset = None;
        let mut width = None;
        let mut spacings = Vec::new();
        let mut spacing_table = None;

        loop {
            let t = self.lx.expect_token(&format!("'END {name}'"))?;
            match t.text {
                "END" => {
                    self.end_block(name)?;
                    break;
                }
                "TYPE" => {
                    kind = Some(self.word("layer type")?);
                    self.lx.skip_statement()?;
                }
                "DIRECTION" => {
                    direction = Some(match self.word("direction")? {
                        "HORIZONTAL" => Direction::Horizontal,
                        "VERTICAL" => Direction::Vertical,
                        other => {
                            return Err(self
                                .lx
                                .error(t.line, format!("unknown direction '{other}'")));
                        }
                    });
                    self.lx.skip_statement()?;
                }
                "PITCH" => pitch = Some(self.one_or_two()?),
                "OFFSET" => offset = Some(self.one_or_two()?),
                "WIDTH" => {
                    width = Some(self.microns("width")?);
                    self.lx.skip_statement()?;
                }
                "SPACING" => {
                    let value = self.microns("spacing")?;
                    if self.lx.accept(";") {
                        spacings.push(value);
                    } else {
                        self.lx.skip_statement()?;
                    }
                }
                "SPACINGTABLE" => {
                    if let Some(table) = self.spacing_table()? {
                        spacings.push(table.spacing(Dbu::ZERO, Dbu::ZERO));
                        spacing_table = Some(table);
                    }
                }
                _ => self.lx.skip_statement()?,
            }
        }

        let missing = |what: &str| self.lx.error(line, format!("layer '{name}' has no {what}"));
        match kind {
            Some("ROUTING") => {
                let direction = direction.ok_or_else(|| missing("DIRECTION"))?;
                let (pitch_x, pitch_y) = pitch.ok_or_else(|| missing("PITCH"))?;
                let width = width.ok_or_else(|| missing("WIDTH"))?;
                let min_spacing = spacings
                    .iter()
                    .copied()
                    .min()
                    .ok_or_else(|| missing("SPACING"))?;
                self.stack(StackKind::Routing, name, line)?;
                let _ = self
                    .lib
                    .add_routing_layer(RoutingLayer {
                        name: name.to_string(),
                        direction,
                        pitch_x,
                        pitch_y,
                        offset_x: offset.map(|o| o.0),
                        offset_y: offset.map(|o| o.1),
                        width,
                        min_spacing,
                        spacing_table,
                    })
                    .map_err(|e| self.lx.error(line, e.to_string()))?;
            }
            Some("CUT") => {
                self.stack(StackKind::Cut, name, line)?;
                let _ = self
                    .lib
                    .add_cut_layer(CutLayer {
                        name: name.to_string(),
                        width,
                        spacing: spacings.iter().copied().min(),
                    })
                    .map_err(|e| self.lx.error(line, e.to_string()))?;
            }
            Some("MASTERSLICE" | "OVERLAP" | "IMPLANT") => {
                let _ = self.ignored_layers.insert(name.to_string());
            }
            Some(other) => {
                return Err(self.lx.error(line, format!("unknown layer type '{other}'")));
            }
            None => return Err(missing("TYPE")),
        }
        Ok(())
    }

    /// Checks that routing and cut layers alternate, starting with routing.
    fn stack(&mut self, kind: StackKind, name: &str, line: usize) -> Result<(), ParseError> {
        let ok = match kind {
            StackKind::Routing => *self.last_layer != Some(StackKind::Routing),
            StackKind::Cut => *self.last_layer == Some(StackKind::Routing),
        };
        if !ok {
            return Err(self.lx.error(
                line,
                format!("layer '{name}' breaks the routing/cut alternation of the stack"),
            ));
        }
        *self.last_layer = Some(kind);
        Ok(())
    }

    /// Reads `value ;` or `x y ;`.
    fn one_or_two(&mut self) -> Result<(Dbu, Dbu), ParseError> {
        let first = self.microns("value")?;
        if self.lx.accept(";") {
            return Ok((first, first));
        }
        let second = self.microns("value")?;
        self.lx.expect(";")?;
        Ok((first, second))
    }

    /// Reads a parallel run length spacing table; other kinds of table are
    /// skipped.
    fn spacing_table(&mut self) -> Result<Option<SpacingTable>, ParseError> {
        if !self.lx.accept("PARALLELRUNLENGTH") {
            self.lx.skip_statement()?;
            return Ok(None);
        }
        let line = self.lx.line();
        let mut run_lengths = Vec::new();
        while !self.lx.accept("WIDTH") {
            run_lengths.push(self.microns("parallel run length")?);
        }
        let mut rows = Vec::new();
        loop {
            let width = self.microns("width")?;
            let spacings = (0..run_lengths.len())
                .map(|_| self.microns("spacing"))
                .collect::<Result<Vec<Dbu>, ParseError>>()?;
            rows.push((width, spacings));
            if self.lx.accept(";") {
                break;
            }
            self.lx.expect("WIDTH")?;
        }
        if run_lengths.is_empty() {
            return Err(self
                .lx
                .error(line, "spacing table has no run lengths".to_string()));
        }
        Ok(Some(SpacingTable { run_lengths, rows }))
    }

    /// Resolves a layer name, or `None` for layers whose shapes are dropped.
    fn layer_ref(&self, line: usize, name: &str) -> Result<Option<LayerRef>, ParseError> {
        if let Some(l) = self.lib.layer_ref(name) {
            return Ok(Some(l));
        }
        if self.ignored_layers.contains(name) {
            return Ok(None);
        }
        Err(self.lx.error(line, format!("unknown layer '{name}'")))
    }

    fn rect_after_mask(&mut self) -> Result<Rect<Dbu>, ParseError> {
        if self.lx.accept("MASK") {
            let _ = self.word("mask number")?;
        }
        let rect = self.rect()?;
        self.lx.expect(";")?;
        Ok(rect)
    }

    /// Reads `LAYER`/`RECT`/`VIA` geometry statements up to a bare `END`.
    fn geometry(&mut self, origin: Point<Dbu>) -> Result<Vec<Shape>, ParseError> {
        let mut shapes = Vec::new();
        let mut layer = None;
        loop {
            let t = self.lx.expect_token("'END'")?;
            match t.text {
                "END" => return Ok(shapes),
                "LAYER" => {
                    let name = self.word("layer name")?;
                    layer = Some(self.layer_ref(t.line, name)?);
                    self.lx.skip_statement()?;
                }
                "RECT" => {
                    let rect = self.rect_after_mask()?.translate(origin);
                    match layer {
                        Some(Some(layer)) => shapes.push(Shape { layer, rect }),
                        Some(None) => {}
                        None => return Err(self.lx.error(t.line, "RECT before any LAYER")),
                    }
                }
                "VIA" => {
                    let at = self.point()? + origin;
                    let name = self.word("via name")?;
                    let id = self
                        .lib
                        .via_id(name)
                        .ok_or_else(|| self.lx.error(t.line, format!("unknown via '{name}'")))?;
                    shapes.extend(self.lib.via(id).shapes.iter().map(|s| Shape {
                        layer: s.layer,
                        rect: s.rect.translate(at),
                    }));
                    self.lx.skip_statement()?;
                }
                "POLYGON" => {
                    return Err(self.lx.error(t.line, "POLYGON geometry is not supported"));
                }
                _ => self.lx.skip_statement()?,
            }
        }
    }

    fn via(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("via name")?;
        let mut is_default = false;
        let mut top_of_stack_only = false;
        loop {
            match self.lx.peek().map(|t| t.text) {
                Some("DEFAULT") => is_default = true,
                Some("TOPOFSTACKONLY") => top_of_stack_only = true,
                Some("GENERATED") => {}
                _ => break,
            }
            let _ = self.lx.next_token();
        }
        let is_default = is_default && !top_of_stack_only;

        let mut shapes = Vec::new();
        let mut layer = None;
        loop {
            let t = self.lx.expect_token(&format!("'END {name}'"))?;
            match t.text {
                "END" => {
                    self.end_block(name)?;
                    break;
                }
                "LAYER" => {
                    let lname = self.word("layer name")?;
                    layer = self.layer_ref(t.line, lname)?;
                    self.lx.skip_statement()?;
                }
                "RECT" => {
                    let rect = self.rect_after_mask()?;
                    let layer = layer.ok_or_else(|| self.lx.error(t.line, "RECT before LAYER"))?;
                    shapes.push(Shape { layer, rect });
                }
                "VIARULE" => {
                    return Err(self.lx.error(
                        t.line,
                        format!("via '{name}': VIARULE-generated LEF vias are not supported"),
                    ));
                }
                _ => self.lx.skip_statement()?,
            }
        }

        let via = via_from_shapes(name, is_default, shapes, false)
            .map_err(|message| self.lx.error(line, message))?;
        let _ = self
            .lib
            .add_via(via)
            .map_err(|e| self.lx.error(line, e.to_string()))?;
        Ok(())
    }

    fn site(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("site name")?;
        let mut size = None;
        let mut symmetry = Symmetry::default();
        loop {
            let t = self.lx.expect_token(&format!("'END {name}'"))?;
            match t.text {
                "END" => {
                    self.end_block(name)?;
                    break;
                }
                "SIZE" => size = Some(self.size()?),
                "SYMMETRY" => symmetry = self.symmetry()?,
                _ => self.lx.skip_statement()?,
            }
        }
        let size = size.ok_or_else(|| self.lx.error(line, format!("site '{name}' has no SIZE")))?;
        let _ = self
            .lib
            .add_site(Site {
                name: name.to_string(),
                size,
                symmetry,
            })
            .map_err(|e| self.lx.error(line, e.to_string()))?;
        Ok(())
    }

    fn size(&mut self) -> Result<Point<Dbu>, ParseError> {
        let w = self.microns("width")?;
        self.lx.expect("BY")?;
        let h = self.microns("height")?;
        self.lx.expect(";")?;
        Ok(Point::new(w, h))
    }

    fn symmetry(&mut self) -> Result<Symmetry, ParseError> {
        let mut s = Symmetry::default();
        loop {
            let t = self.lx.expect_token("';'")?;
            match t.text.to_ascii_uppercase().as_str() {
                ";" => return Ok(s),
                "X" => s.x = true,
                "Y" => s.y = true,
                "R90" => s.r90 = true,
                other => {
                    return Err(self.lx.error(t.line, format!("unknown symmetry '{other}'")));
                }
            }
        }
    }

    fn macro_class(&mut self, line: usize) -> Result<MacroClass, ParseError> {
        let class = self.word("macro class")?.to_ascii_uppercase();
        let sub = match self.lx.peek() {
            Some(t) if t.text != ";" => Some(self.word("macro subclass")?.to_ascii_uppercase()),
            _ => None,
        };
        self.lx.skip_statement()?;
        let unknown =
            |what: &str, value: &str| self.lx.error(line, format!("unknown {what} '{value}'"));
        Ok(match class.as_str() {
            "CORE" => MacroClass::Core(match sub.as_deref() {
                None => CoreKind::Regular,
                Some("FEEDTHRU") => CoreKind::Feedthru,
                Some("TIEHIGH") => CoreKind::TieHigh,
                Some("TIELOW") => CoreKind::TieLow,
                Some("SPACER") => CoreKind::Spacer,
                Some("ANTENNACELL") => CoreKind::AntennaCell,
                Some("WELLTAP") => CoreKind::WellTap,
                Some(other) => return Err(unknown("CORE subclass", other)),
            }),
            "BLOCK" => MacroClass::Block,
            "PAD" => MacroClass::Pad,
            "ENDCAP" => MacroClass::Endcap,
            "COVER" => MacroClass::Cover,
            "RING" => MacroClass::Ring,
            other => return Err(unknown("macro class", other)),
        })
    }

    fn cell_macro(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("macro name")?;
        let mut class = None;
        let mut origin = Point::default();
        let mut size = None;
        let mut symmetry = Symmetry::default();
        let mut site = None;
        let mut pins = Vec::new();
        let mut obstructions = Vec::new();

        loop {
            let t = self.lx.expect_token(&format!("'END {name}'"))?;
            match t.text {
                "END" => {
                    self.end_block(name)?;
                    break;
                }
                "CLASS" => class = Some(self.macro_class(t.line)?),
                "ORIGIN" => {
                    origin = self.point()?;
                    self.lx.expect(";")?;
                }
                "SIZE" => size = Some(self.size()?),
                "SYMMETRY" => symmetry = self.symmetry()?,
                "SITE" => {
                    let site_name = self.word("site name")?;
                    site = Some(self.lib.site_id(site_name).ok_or_else(|| {
                        self.lx.error(t.line, format!("unknown site '{site_name}'"))
                    })?);
                    self.lx.skip_statement()?;
                }
                "PIN" => pins.push(self.macro_pin(origin)?),
                "OBS" => obstructions.extend(self.geometry(origin)?),
                "DENSITY" => while self.lx.expect_token("'END'")?.text != "END" {},
                _ => self.lx.skip_statement()?,
            }
        }

        let class =
            class.ok_or_else(|| self.lx.error(line, format!("macro '{name}' has no CLASS")))?;
        let size =
            size.ok_or_else(|| self.lx.error(line, format!("macro '{name}' has no SIZE")))?;
        let mut m = Macro::new(name.to_string(), class, size, symmetry, site);
        m.obstructions = obstructions;
        for pin in pins {
            let pin_name = pin.name.clone();
            if m.add_pin(pin).is_none() {
                return Err(self
                    .lx
                    .error(line, format!("macro '{name}' repeats pin '{pin_name}'")));
            }
        }
        let _ = self
            .lib
            .add_macro(m)
            .map_err(|e| self.lx.error(line, e.to_string()))?;
        Ok(())
    }

    fn macro_pin(&mut self, origin: Point<Dbu>) -> Result<MacroPin, ParseError> {
        let name = self.word("pin name")?;
        let mut direction = PinDirection::default();
        let mut signal_use = SignalUse::default();
        let mut shapes = Vec::new();
        loop {
            let t = self.lx.expect_token(&format!("'END {name}'"))?;
            match t.text {
                "END" => {
                    self.end_block(name)?;
                    break;
                }
                "DIRECTION" => {
                    let word = self.word("direction")?;
                    direction = parse_pin_direction(word).ok_or_else(|| {
                        self.lx
                            .error(t.line, format!("unknown pin direction '{word}'"))
                    })?;
                    self.lx.skip_statement()?;
                }
                "USE" => {
                    let word = self.word("pin use")?;
                    signal_use = parse_signal_use(word).ok_or_else(|| {
                        self.lx.error(t.line, format!("unknown pin use '{word}'"))
                    })?;
                    self.lx.skip_statement()?;
                }
                "PORT" => shapes.extend(self.geometry(origin)?),
                _ => self.lx.skip_statement()?,
            }
        }
        Ok(MacroPin {
            name: name.to_string(),
            direction,
            signal_use,
            shapes,
        })
    }
}

/// Builds a via from its shapes: exactly one cut layer, with metal only on
/// the routing layers directly below and above it.
pub(crate) fn via_from_shapes(
    name: &str,
    is_default: bool,
    shapes: Vec<Shape>,
    from_def: bool,
) -> Result<ViaDef, String> {
    let mut cuts = shapes.iter().filter_map(|s| match s.layer {
        LayerRef::Cut(c) => Some(c),
        LayerRef::Routing(_) => None,
    });
    let cut = cuts
        .next()
        .ok_or_else(|| format!("via '{name}' has no cut shape"))?;
    if cuts.any(|c| c != cut) {
        return Err(format!("via '{name}' spans more than one cut layer"));
    }
    let stray = shapes.iter().any(|s| match s.layer {
        LayerRef::Routing(l) => l != cut.below() && l != cut.above(),
        LayerRef::Cut(_) => false,
    });
    if stray {
        return Err(format!(
            "via '{name}' has metal on a layer its cut does not touch"
        ));
    }
    Ok(ViaDef {
        name: name.to_string(),
        is_default,
        cut,
        shapes,
        from_def,
    })
}

/// Parses a LEF/DEF pin direction keyword.
pub(crate) fn parse_pin_direction(word: &str) -> Option<PinDirection> {
    match word {
        "INPUT" => Some(PinDirection::Input),
        "OUTPUT" => Some(PinDirection::Output),
        "INOUT" => Some(PinDirection::Inout),
        "FEEDTHRU" => Some(PinDirection::Feedthru),
        _ => None,
    }
}

/// Parses a LEF/DEF `USE` keyword.
pub(crate) fn parse_signal_use(word: &str) -> Option<SignalUse> {
    match word {
        "SIGNAL" => Some(SignalUse::Signal),
        "POWER" => Some(SignalUse::Power),
        "GROUND" => Some(SignalUse::Ground),
        "CLOCK" => Some(SignalUse::Clock),
        "ANALOG" => Some(SignalUse::Analog),
        "SCAN" => Some(SignalUse::Scan),
        "RESET" => Some(SignalUse::Reset),
        "TIEOFF" => Some(SignalUse::Tieoff),
        _ => None,
    }
}
