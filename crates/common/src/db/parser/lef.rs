//! LEF (Library Exchange Format) parser.
//!
//! Reads routing and cut layers with their design rules, fixed vias, via
//! generation rules, sites and cell masters into a [`Library`]. Masterslice,
//! overlap and implant layers are recognised and their shapes dropped.
//! Electrical and antenna data are skipped by name; any other statement the
//! parser does not know is skipped with a warning, once per keyword and file.

use crate::db::indices::LayerId;
use crate::db::library::{
    CoreKind, CutLayer, Direction, LayerRef, Library, Macro, MacroClass, MacroPin, PinDirection,
    RoutingLayer, Shape, SignalUse, Site, SiteClass, Symmetry, ViaDef,
};
use crate::db::parser::lef_rules::{RoutingSpacing, RuleReader};
use crate::db::parser::lexer::{Lexer, ParseError, Token, read_file};
use crate::db::rules::{
    CellEdge, ClearanceMeasure, CutLayerRules, MaxViaStack, RoutingLayerRules, SameNetSpacing,
    ViaRule, ViaRuleMetal,
};
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
            warned: HashSet::new(),
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

/// The kind of layer a `LAYER` block defines, from its `TYPE`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum LayerKind {
    Routing,
    Cut,
    Ignored,
}

/// Statements that carry nothing placement or routing uses.
const TOP_IGNORED: &[&str] = &[
    "VERSION",
    "BUSBITCHARS",
    "DIVIDERCHAR",
    "NAMESCASESENSITIVE",
];
const LAYER_IGNORED: &[&str] = &[
    "CAPACITANCE",
    "RESISTANCE",
    "EDGECAPACITANCE",
    "THICKNESS",
    "HEIGHT",
    "SHRINKAGE",
    "CAPMULTIPLIER",
    "ACCURRENTDENSITY",
    "DCCURRENTDENSITY",
];
const MACRO_IGNORED: &[&str] = &["FOREIGN", "EEQ", "SOURCE", "POWER", "FIXEDMASK"];
const PIN_IGNORED: &[&str] = &[
    "SHAPE",
    "NETEXPR",
    "TAPERRULE",
    "SUPPLYSENSITIVITY",
    "GROUNDSENSITIVITY",
    "CAPACITANCE",
];
const VIA_IGNORED: &[&str] = &["RESISTANCE"];

fn is_ignored(list: &[&str], keyword: &str) -> bool {
    list.contains(&keyword) || keyword.starts_with("ANTENNA")
}

/// A metal or cut layer named in a via rule, with its statements.
#[derive(Default)]
struct ViaRuleLayer {
    layer: Option<LayerRef>,
    enclosure: Option<(Dbu, Dbu)>,
    width_range: Option<(Dbu, Dbu)>,
    cut_rect: Option<Rect<Dbu>>,
    cut_pitch: Option<Point<Dbu>>,
}

struct LefParser<'a, 'l> {
    lx: Lexer<'a>,
    lib: &'l mut Library,
    dbu: DbuPerMicron,
    ignored_layers: &'l mut HashSet<String>,
    last_layer: &'l mut Option<StackKind>,
    warned: HashSet<(&'static str, &'a str)>,
}

impl<'a> LefParser<'a, '_> {
    fn parse(&mut self) -> Result<(), ParseError> {
        while let Some(t) = self.lx.next_token() {
            match t.text {
                "LAYER" => self.layer()?,
                "VIA" => self.via()?,
                "VIARULE" => self.via_rule()?,
                "SITE" => self.site()?,
                "MACRO" => self.cell_macro()?,
                "SPACING" => self.same_net_spacing()?,
                "PROPERTYDEFINITIONS" => self.property_definitions()?,
                "UNITS" => self.skip_block("UNITS")?,
                "MANUFACTURINGGRID" => {
                    self.lib.rules.manufacturing_grid = Some(self.microns("grid")?);
                    self.lx.expect(";")?;
                }
                "CLEARANCEMEASURE" => {
                    self.lib.rules.clearance_measure = match self.word("clearance measure")? {
                        "EUCLIDEAN" => ClearanceMeasure::Euclidean,
                        "MAXXY" => ClearanceMeasure::MaxXy,
                        other => {
                            return Err(self
                                .lx
                                .error(t.line, format!("unknown clearance measure '{other}'")));
                        }
                    };
                    self.lx.expect(";")?;
                }
                "USEMINSPACING" => {
                    self.lx.expect("OBS")?;
                    self.lib.rules.min_spacing_for_obstructions = self.on_off()?;
                    self.lx.expect(";")?;
                }
                "MAXVIASTACK" => self.max_via_stack()?,
                "NONDEFAULTRULE" => {
                    let name = self.word("rule name")?;
                    self.warn(t, "LEF NONDEFAULTRULE (DEF rules are read)");
                    self.skip_block(name)?;
                }
                "END" => {
                    if self.lx.accept("LIBRARY") {
                        return Ok(());
                    }
                    return Err(self.lx.error(t.line, "unexpected END"));
                }
                kw if TOP_IGNORED.contains(&kw) => self.lx.skip_statement()?,
                _ => self.skip_unknown(t, "library")?,
            }
        }
        Ok(())
    }

    /// Warns once per file about a statement kind that is not read.
    fn warn(&mut self, t: Token<'a>, context: &'static str) {
        if self.warned.insert((context, t.text)) {
            log::warn!(
                "{}:{}: ignoring {context} statement '{}'",
                self.lx.path().display(),
                t.line,
                t.text
            );
        }
    }

    fn skip_unknown(&mut self, t: Token<'a>, context: &'static str) -> Result<(), ParseError> {
        self.warn(t, context);
        self.lx.skip_statement()
    }

    fn word(&mut self, what: &str) -> Result<&'a str, ParseError> {
        Ok(self.lx.expect_token(what)?.text)
    }

    const fn rules(&mut self) -> RuleReader<'_, 'a> {
        RuleReader::new(&mut self.lx, self.dbu)
    }

    fn microns(&mut self, what: &str) -> Result<Dbu, ParseError> {
        self.rules().microns(what)
    }

    fn on_off(&mut self) -> Result<bool, ParseError> {
        let t = self.lx.expect_token("ON or OFF")?;
        match t.text {
            "ON" => Ok(true),
            "OFF" => Ok(false),
            other => Err(self
                .lx
                .error(t.line, format!("expected ON or OFF, found '{other}'"))),
        }
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
        let mut routing = RoutingLayerRules::default();
        let mut cut = CutLayerRules::default();

        loop {
            let t = self.lx.expect_token(&format!("'END {name}'"))?;
            if t.text == "END" {
                self.end_block(name)?;
                break;
            }
            if t.text == "TYPE" {
                kind = Some(self.layer_kind()?);
                continue;
            }
            let Some(kind) = kind else {
                return Err(self
                    .lx
                    .error(t.line, format!("layer '{name}' must give its TYPE first")));
            };
            match (kind, t.text) {
                (LayerKind::Ignored, _) => self.lx.skip_statement()?,
                (_, "WIDTH") => {
                    width = Some(self.microns("width")?);
                    self.lx.expect(";")?;
                }
                (_, "PROPERTY") => self.layer_properties(kind, &mut routing, &mut cut)?,
                (LayerKind::Routing, "DIRECTION") => {
                    direction = Some(match self.word("direction")? {
                        "HORIZONTAL" => Direction::Horizontal,
                        "VERTICAL" => Direction::Vertical,
                        other => {
                            return Err(self
                                .lx
                                .error(t.line, format!("unknown direction '{other}'")));
                        }
                    });
                    self.lx.expect(";")?;
                }
                (LayerKind::Routing, "PITCH") => pitch = Some(self.one_or_two()?),
                (LayerKind::Routing, "OFFSET") => offset = Some(self.one_or_two()?),
                (LayerKind::Routing, "SPACING") => match self.rules().routing_spacing()? {
                    RoutingSpacing::Plain(s) => spacings.push(s),
                    RoutingSpacing::WidthRange(r) => routing.width_range_spacing.push(r),
                    RoutingSpacing::EndOfLine(r) => routing.end_of_line.push(r),
                },
                (LayerKind::Routing, "SPACINGTABLE") => {
                    let table = self.rules().spacing_table()?;
                    spacings.push(table.min_spacing());
                    spacing_table = Some(table);
                }
                (LayerKind::Routing, "MINWIDTH") => {
                    routing.min_width = Some(self.microns("minimum width")?);
                    self.lx.expect(";")?;
                }
                (LayerKind::Routing, "MAXWIDTH") => {
                    routing.max_width = Some(self.microns("maximum width")?);
                    self.lx.expect(";")?;
                }
                (LayerKind::Routing, "AREA") => {
                    routing.min_area = Some(self.rules().area("minimum area")?);
                    self.lx.expect(";")?;
                }
                (LayerKind::Routing, "MINENCLOSEDAREA") => {
                    let rule = self.rules().min_enclosed_area()?;
                    routing.min_enclosed_areas.push(rule);
                }
                (LayerKind::Routing, "MINSTEP") => {
                    let rule = self.rules().min_step()?;
                    routing.min_steps.push(rule);
                }
                (LayerKind::Routing, "MINIMUMCUT") => {
                    let rule = self.rules().minimum_cut()?;
                    routing.minimum_cuts.push(rule);
                }
                (LayerKind::Cut, "SPACING") => {
                    let rule = self.rules().cut_spacing()?;
                    cut.spacings.push(rule);
                }
                (LayerKind::Cut, "ENCLOSURE") => {
                    let rule = self.rules().enclosure()?;
                    cut.enclosures.push(rule);
                }
                (_, kw) if is_ignored(LAYER_IGNORED, kw) => self.lx.skip_statement()?,
                _ => self.skip_unknown(t, "layer")?,
            }
        }

        let missing = |what: &str| self.lx.error(line, format!("layer '{name}' has no {what}"));
        match kind {
            Some(LayerKind::Routing) => {
                let direction = direction.ok_or_else(|| missing("DIRECTION"))?;
                let (pitch_x, pitch_y) = pitch.ok_or_else(|| missing("PITCH"))?;
                let width = width.ok_or_else(|| missing("WIDTH"))?;
                let min_spacing = spacings
                    .iter()
                    .copied()
                    .min()
                    .ok_or_else(|| missing("SPACING or SPACINGTABLE"))?;
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
                        rules: routing,
                    })
                    .map_err(|e| self.lx.error(line, e.to_string()))?;
            }
            Some(LayerKind::Cut) => {
                self.stack(StackKind::Cut, name, line)?;
                let _ = self
                    .lib
                    .add_cut_layer(CutLayer {
                        name: name.to_string(),
                        width,
                        spacing: CutLayer::unconditional_spacing(width, &cut),
                        rules: cut,
                    })
                    .map_err(|e| self.lx.error(line, e.to_string()))?;
            }
            Some(LayerKind::Ignored) => {
                let _ = self.ignored_layers.insert(name.to_string());
            }
            None => return Err(missing("TYPE")),
        }
        Ok(())
    }

    fn layer_kind(&mut self) -> Result<LayerKind, ParseError> {
        let t = self.lx.expect_token("layer type")?;
        let kind = match t.text {
            "ROUTING" => LayerKind::Routing,
            "CUT" => LayerKind::Cut,
            "MASTERSLICE" | "OVERLAP" | "IMPLANT" => LayerKind::Ignored,
            other => {
                return Err(self
                    .lx
                    .error(t.line, format!("unknown layer type '{other}'")));
            }
        };
        self.lx.skip_statement()?;
        Ok(kind)
    }

    /// Reads `PROPERTY name value ... ;` on a layer, parsing the LEF57/LEF58
    /// rules whose syntax the model covers.
    fn layer_properties(
        &mut self,
        kind: LayerKind,
        routing: &mut RoutingLayerRules,
        cut: &mut CutLayerRules,
    ) -> Result<(), ParseError> {
        while !self.lx.accept(";") {
            let name = self.lx.expect_token("property name")?;
            let value = self.lx.expect_token("property value")?;
            let mut sub = Lexer::starting_at(value.text, self.lx.path(), value.line);
            let mut rules = RuleReader::new(&mut sub, self.dbu);
            match (kind, name.text) {
                (LayerKind::Routing, "LEF57_SPACING" | "LEF58_SPACING") => {
                    while let Some(t) = rules.next_keyword("SPACING")? {
                        match rules.routing_spacing()? {
                            RoutingSpacing::Plain(_) => {
                                return Err(self.lx.error(
                                    t.line,
                                    format!("{} repeats a plain SPACING", name.text),
                                ));
                            }
                            RoutingSpacing::WidthRange(r) => routing.width_range_spacing.push(r),
                            RoutingSpacing::EndOfLine(r) => routing.end_of_line.push(r),
                        }
                    }
                }
                (LayerKind::Routing, "LEF57_MINSTEP" | "LEF58_MINSTEP") => {
                    while rules.next_keyword("MINSTEP")?.is_some() {
                        routing.min_steps.push(rules.min_step()?);
                    }
                }
                (LayerKind::Routing, "LEF58_CORNERSPACING") => {
                    while rules.next_keyword("CORNERSPACING")?.is_some() {
                        routing.corner_spacing = Some(rules.corner_spacing()?);
                    }
                }
                (LayerKind::Cut, "LEF57_SPACING" | "LEF58_SPACING") => {
                    while rules.next_keyword("SPACING")?.is_some() {
                        cut.spacings.push(rules.cut_spacing()?);
                    }
                }
                _ => self.warn(name, "layer PROPERTY"),
            }
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

    /// Reads the `SPACING ... END SPACING` block of same-net rules.
    fn same_net_spacing(&mut self) -> Result<(), ParseError> {
        loop {
            let t = self.lx.expect_token("'END SPACING'")?;
            match t.text {
                "END" => return self.end_block("SPACING"),
                "SAMENET" => {
                    let first = self.known_layer(t.line)?;
                    let second = self.known_layer(t.line)?;
                    let spacing = self.microns("same-net spacing")?;
                    let stack = self.lx.accept("STACK");
                    self.lx.expect(";")?;
                    self.lib.rules.same_net_spacing.push(SameNetSpacing {
                        first,
                        second,
                        spacing,
                        stack,
                    });
                }
                other => {
                    return Err(self
                        .lx
                        .error(t.line, format!("unknown SPACING block rule '{other}'")));
                }
            }
        }
    }

    fn known_layer(&mut self, line: usize) -> Result<LayerRef, ParseError> {
        let name = self.word("layer name")?;
        self.lib
            .layer_ref(name)
            .ok_or_else(|| self.lx.error(line, format!("unknown layer '{name}'")))
    }

    fn known_routing_layer(&mut self, line: usize) -> Result<LayerId, ParseError> {
        match self.known_layer(line)? {
            LayerRef::Routing(id) => Ok(id),
            LayerRef::Cut(_) => Err(self.lx.error(line, "expected a routing layer")),
        }
    }

    /// Reads `PROPERTYDEFINITIONS`, keeping the library cell edge spacing
    /// table that ISPD benchmarks give as a property default.
    fn property_definitions(&mut self) -> Result<(), ParseError> {
        loop {
            let t = self.lx.expect_token("'END PROPERTYDEFINITIONS'")?;
            if t.text == "END" {
                return self.end_block("PROPERTYDEFINITIONS");
            }
            let name = self.lx.expect_token("property name")?;
            let mut value = None;
            let _type = self.word("property type")?;
            loop {
                let v = self.lx.expect_token("';'")?;
                if v.text == ";" {
                    break;
                }
                value = Some(v);
            }
            let is_edge_table = matches!(
                name.text,
                "LEF57_CELLEDGESPACINGTABLE" | "LEF58_CELLEDGESPACINGTABLE"
            );
            match (t.text, value) {
                ("LIBRARY", Some(v)) if is_edge_table => {
                    let mut sub = Lexer::starting_at(v.text, self.lx.path(), v.line);
                    let mut rules = RuleReader::new(&mut sub, self.dbu);
                    while rules.next_keyword("CELLEDGESPACINGTABLE")?.is_some() {
                        let table = rules.cell_edge_spacing_table()?;
                        self.lib.rules.cell_edge_spacing.extend(table);
                    }
                }
                ("LIBRARY", Some(_)) => self.warn(name, "library PROPERTY"),
                _ => {}
            }
        }
    }

    /// `MAXVIASTACK n [RANGE bottom top] ;`
    fn max_via_stack(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let vias = self.lx.number("via count")?;
        let range = if self.lx.accept("RANGE") {
            Some((
                self.known_routing_layer(line)?,
                self.known_routing_layer(line)?,
            ))
        } else {
            None
        };
        self.lx.expect(";")?;
        self.lib.rules.max_via_stack = Some(MaxViaStack { vias, range });
        Ok(())
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

    /// Reads `LAYER name [EXCEPTPGNET] ;` in port or obstruction geometry.
    fn geometry_layer(&mut self, line: usize) -> Result<Option<LayerRef>, ParseError> {
        let name = self.word("layer name")?;
        let layer = self.layer_ref(line, name)?;
        let _ = self.lx.accept("EXCEPTPGNET");
        if self.lx.accept("SPACING") || self.lx.accept("DESIGNRULEWIDTH") {
            return Err(self.lx.error(
                line,
                format!(
                    "layer '{name}': SPACING and DESIGNRULEWIDTH on geometry are not supported"
                ),
            ));
        }
        self.lx.expect(";")?;
        Ok(layer)
    }

    /// Reads `LAYER`/`RECT`/`VIA` geometry statements up to a bare `END`.
    fn geometry(&mut self, origin: Point<Dbu>) -> Result<Vec<Shape>, ParseError> {
        let mut shapes = Vec::new();
        let mut layer = None;
        loop {
            let t = self.lx.expect_token("'END'")?;
            match t.text {
                "END" => return Ok(shapes),
                "LAYER" => layer = Some(self.geometry_layer(t.line)?),
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
                    self.lx.expect(";")?;
                }
                "CLASS" => self.lx.skip_statement()?,
                "POLYGON" | "PATH" | "WIDTH" => {
                    return Err(self
                        .lx
                        .error(t.line, format!("{} geometry is not supported", t.text)));
                }
                _ => self.skip_unknown(t, "geometry")?,
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
                    self.lx.expect(";")?;
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
                "POLYGON" => {
                    return Err(self
                        .lx
                        .error(t.line, format!("via '{name}': POLYGON is not supported")));
                }
                kw if VIA_IGNORED.contains(&kw) => self.lx.skip_statement()?,
                _ => self.skip_unknown(t, "via")?,
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

    /// `VIARULE name GENERATE [DEFAULT]` with one cut and two metal layers.
    fn via_rule(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("via rule name")?;
        if !self.lx.accept("GENERATE") {
            return Err(self.lx.error(
                line,
                format!("via rule '{name}': only VIARULE ... GENERATE is supported"),
            ));
        }
        let is_default = self.lx.accept("DEFAULT");
        let mut layers: Vec<ViaRuleLayer> = Vec::new();
        loop {
            let t = self.lx.expect_token(&format!("'END {name}'"))?;
            if t.text == "END" {
                self.end_block(name)?;
                break;
            }
            if t.text == "LAYER" {
                let layer = self.known_layer(t.line)?;
                self.lx.expect(";")?;
                layers.push(ViaRuleLayer {
                    layer: Some(layer),
                    ..ViaRuleLayer::default()
                });
                continue;
            }
            let Some(current) = layers.last_mut() else {
                return Err(self.lx.error(
                    t.line,
                    format!("via rule '{name}': {} before LAYER", t.text),
                ));
            };
            match t.text {
                "ENCLOSURE" => {
                    current.enclosure =
                        Some((self.microns("enclosure")?, self.microns("enclosure")?));
                    self.lx.expect(";")?;
                }
                "WIDTH" => {
                    let lo = self.microns("minimum width")?;
                    self.lx.expect("TO")?;
                    current.width_range = Some((lo, self.microns("maximum width")?));
                    self.lx.expect(";")?;
                }
                "RECT" => {
                    current.cut_rect = Some(self.rect()?);
                    self.lx.expect(";")?;
                }
                "SPACING" => {
                    let x = self.microns("x pitch")?;
                    self.lx.expect("BY")?;
                    current.cut_pitch = Some(Point::new(x, self.microns("y pitch")?));
                    self.lx.expect(";")?;
                }
                "RESISTANCE" => self.lx.skip_statement()?,
                other => {
                    return Err(self.lx.error(
                        t.line,
                        format!("via rule '{name}': unsupported statement '{other}'"),
                    ));
                }
            }
        }
        let rule = via_rule_from_layers(name, is_default, &layers)
            .map_err(|message| self.lx.error(line, message))?;
        self.lib.via_rules.push(rule);
        Ok(())
    }

    fn site(&mut self) -> Result<(), ParseError> {
        let line = self.lx.line();
        let name = self.word("site name")?;
        let mut size = None;
        let mut symmetry = Symmetry::default();
        let mut class = SiteClass::default();
        loop {
            let t = self.lx.expect_token(&format!("'END {name}'"))?;
            match t.text {
                "END" => {
                    self.end_block(name)?;
                    break;
                }
                "SIZE" => size = Some(self.size()?),
                "SYMMETRY" => symmetry = self.symmetry()?,
                "CLASS" => {
                    let word = self.word("site class")?;
                    class = match word.to_ascii_uppercase().as_str() {
                        "CORE" => SiteClass::Core,
                        "PAD" => SiteClass::Pad,
                        _ => {
                            return Err(self
                                .lx
                                .error(t.line, format!("unknown site class '{word}'")));
                        }
                    };
                    self.lx.expect(";")?;
                }
                _ => self.skip_unknown(t, "site")?,
            }
        }
        let size = size.ok_or_else(|| self.lx.error(line, format!("site '{name}' has no SIZE")))?;
        let _ = self
            .lib
            .add_site(Site {
                name: name.to_string(),
                class,
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
        self.lx.expect(";")?;
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
        let mut edges = Vec::new();

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
                "PROPERTY" => edges.extend(self.macro_properties()?),
                kw if MACRO_IGNORED.contains(&kw) => self.lx.skip_statement()?,
                _ => self.skip_unknown(t, "macro")?,
            }
        }

        let class =
            class.ok_or_else(|| self.lx.error(line, format!("macro '{name}' has no CLASS")))?;
        let size =
            size.ok_or_else(|| self.lx.error(line, format!("macro '{name}' has no SIZE")))?;
        let mut m = Macro::new(name.to_string(), class, size, symmetry, site);
        m.obstructions = obstructions;
        m.edges = edges;
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

    /// Reads `PROPERTY name value ... ;` on a macro, keeping its edge types.
    fn macro_properties(&mut self) -> Result<Vec<CellEdge>, ParseError> {
        let mut edges = Vec::new();
        while !self.lx.accept(";") {
            let name = self.lx.expect_token("property name")?;
            let value = self.lx.expect_token("property value")?;
            if !matches!(name.text, "LEF57_EDGETYPE" | "LEF58_EDGETYPE") {
                self.warn(name, "macro PROPERTY");
                continue;
            }
            let mut sub = Lexer::starting_at(value.text, self.lx.path(), value.line);
            let mut rules = RuleReader::new(&mut sub, self.dbu);
            while rules.next_keyword("EDGETYPE")?.is_some() {
                edges.push(rules.cell_edge()?);
            }
        }
        Ok(edges)
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
                    self.lx.expect(";")?;
                }
                "PORT" => shapes.extend(self.geometry(origin)?),
                kw if is_ignored(PIN_IGNORED, kw) => self.lx.skip_statement()?,
                _ => self.skip_unknown(t, "pin")?,
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

/// Builds a via rule from its `LAYER` entries: exactly one cut layer with a
/// cut and pitch, between two metal layers with an enclosure each.
fn via_rule_from_layers(
    name: &str,
    is_default: bool,
    layers: &[ViaRuleLayer],
) -> Result<ViaRule, String> {
    let mut cut = None;
    let mut metals = Vec::new();
    for l in layers {
        match l.layer {
            Some(LayerRef::Cut(c)) => {
                if cut.replace((c, l)).is_some() {
                    return Err(format!("via rule '{name}' has more than one cut layer"));
                }
            }
            Some(LayerRef::Routing(m)) => metals.push((m, l)),
            None => {}
        }
    }
    let Some((cut, cut_layer)) = cut else {
        return Err(format!("via rule '{name}' has no cut layer"));
    };
    metals.sort_by_key(|(m, _)| *m);
    let [(bottom, b), (top, t)] = metals[..] else {
        return Err(format!("via rule '{name}' needs exactly two metal layers"));
    };
    if bottom != cut.below() || top != cut.above() {
        return Err(format!(
            "via rule '{name}' names metal layers its cut does not touch"
        ));
    }
    let metal = |layer: LayerId, l: &ViaRuleLayer| -> Result<ViaRuleMetal, String> {
        Ok(ViaRuleMetal {
            layer,
            enclosure: l
                .enclosure
                .ok_or_else(|| format!("via rule '{name}' gives no ENCLOSURE on a metal layer"))?,
            width_range: l.width_range,
        })
    };
    Ok(ViaRule {
        name: name.to_string(),
        is_default,
        bottom: metal(bottom, b)?,
        top: metal(top, t)?,
        cut,
        cut_rect: cut_layer
            .cut_rect
            .ok_or_else(|| format!("via rule '{name}' gives no cut RECT"))?,
        cut_pitch: cut_layer
            .cut_pitch
            .ok_or_else(|| format!("via rule '{name}' gives no cut SPACING"))?,
    })
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

/// Parses a LEF/DEF pin direction keyword, in any case.
pub(crate) fn parse_pin_direction(word: &str) -> Option<PinDirection> {
    match word.to_ascii_uppercase().as_str() {
        "INPUT" => Some(PinDirection::Input),
        "OUTPUT" => Some(PinDirection::Output),
        "INOUT" => Some(PinDirection::Inout),
        "FEEDTHRU" => Some(PinDirection::Feedthru),
        _ => None,
    }
}

/// Parses a LEF/DEF `USE` keyword, in any case.
pub(crate) fn parse_signal_use(word: &str) -> Option<SignalUse> {
    match word.to_ascii_uppercase().as_str() {
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
