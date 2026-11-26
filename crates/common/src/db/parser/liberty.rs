//! Liberty parser.
//!
//! Reads a library's units, thresholds, defaults, table templates and cells
//! into a [`Liberty`], scaling every time, capacitance and resistance to SI
//! units. Buses expand to one pin per bit. Power, leakage and wire-load data
//! are skipped. Bundles are refused, since a model without them would be
//! wrong.

use crate::db::indices::IdVec;
use crate::db::liberty::{
    ArcRole, Axis, AxisVariable, ClockGatePin, ClockTreePath, Defaults, Edge, EdgeValues, FuncExpr,
    LibCell, LibPort, LibPortId, Liberty, LibraryUnits, MinMax, PortDirection, Sequential,
    SequentialKind, Statetable, Table, TimingArc, TimingArcSet, TimingSense,
};
use crate::db::parser::lexer::{ParseError, read_file};
use crate::units::{CellArea, Farads, Seconds, Watts};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Parses one Liberty file.
pub fn parse_file(path: &Path) -> Result<Liberty, ParseError> {
    let src = read_file(path)?;
    parse_str(&src, path)
}

/// Parses Liberty text, naming `path` in errors.
pub fn parse_str(src: &str, path: &Path) -> Result<Liberty, ParseError> {
    let mut lexer = Lexer::new(src, path);
    let root = parse_statements(&mut lexer, None)?;
    let library = root
        .groups
        .iter()
        .find(|g| g.name == "library")
        .ok_or_else(|| ParseError::File {
            path: path.to_path_buf(),
            message: "no library group".into(),
        })?;
    Interpreter::new(path).library(library)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Tok<'a> {
    Word(&'a str),
    Str(&'a str),
    LParen,
    RParen,
    LBrace,
    RBrace,
    Colon,
    Semi,
    Comma,
}

struct Lexer<'a> {
    src: &'a str,
    pos: usize,
    line: usize,
    path: &'a Path,
    pushed_back: Option<(Tok<'a>, usize)>,
}

impl<'a> Lexer<'a> {
    const fn new(src: &'a str, path: &'a Path) -> Self {
        Self {
            src,
            pos: 0,
            line: 1,
            path,
            pushed_back: None,
        }
    }

    /// Returns a token so the next call to `next` yields it again.
    const fn push_back(&mut self, tok: Tok<'a>, line: usize) {
        self.pushed_back = Some((tok, line));
    }

    fn error(&self, line: usize, message: String) -> ParseError {
        ParseError::Syntax {
            path: self.path.to_path_buf(),
            line,
            message,
        }
    }

    fn rest(&self) -> &'a str {
        &self.src[self.pos..]
    }

    fn advance(&mut self, bytes: usize) {
        let skipped = &self.src[self.pos..self.pos + bytes];
        self.line += skipped.matches('\n').count();
        self.pos += bytes;
    }

    fn skip_blanks(&mut self) -> Result<(), ParseError> {
        loop {
            let rest = self.rest();
            if rest.starts_with("/*") {
                let end = rest
                    .find("*/")
                    .ok_or_else(|| self.error(self.line, "unterminated comment".into()))?;
                self.advance(end + 2);
            } else if rest.starts_with("//") {
                let end = rest.find('\n').unwrap_or(rest.len());
                self.advance(end);
            } else if rest.starts_with("\\\n") {
                self.advance(2);
            } else if rest.starts_with("\\\r\n") {
                self.advance(3);
            } else if let Some(c) = rest.chars().next().filter(|c| c.is_whitespace()) {
                self.advance(c.len_utf8());
            } else {
                return Ok(());
            }
        }
    }

    fn next(&mut self) -> Result<Option<(Tok<'a>, usize)>, ParseError> {
        if let Some(pushed) = self.pushed_back.take() {
            return Ok(Some(pushed));
        }
        self.skip_blanks()?;
        let line = self.line;
        let rest = self.rest();
        let Some(c) = rest.chars().next() else {
            return Ok(None);
        };
        let simple = match c {
            '(' => Some(Tok::LParen),
            ')' => Some(Tok::RParen),
            '{' => Some(Tok::LBrace),
            '}' => Some(Tok::RBrace),
            ':' => Some(Tok::Colon),
            ';' => Some(Tok::Semi),
            ',' => Some(Tok::Comma),
            _ => None,
        };
        if let Some(tok) = simple {
            self.advance(1);
            return Ok(Some((tok, line)));
        }
        if c == '"' {
            let body = &rest[1..];
            let end = body
                .find('"')
                .ok_or_else(|| self.error(line, "unterminated string".into()))?;
            let text = &body[..end];
            self.advance(end + 2);
            return Ok(Some((Tok::Str(text), line)));
        }
        let end = rest
            .find(|ch: char| ch.is_whitespace() || "(){}:;,\"".contains(ch))
            .unwrap_or(rest.len());
        if end == 0 {
            return Err(self.error(line, format!("unexpected character '{c}'")));
        }
        let word = &rest[..end];
        self.advance(end);
        Ok(Some((Tok::Word(word), line)))
    }
}

/// An attribute or group argument: a bare word or a quoted string.
#[derive(Clone, Copy, Debug)]
struct Value<'a>(&'a str);

#[derive(Debug)]
struct Attr<'a> {
    name: &'a str,
    values: Vec<Value<'a>>,
    line: usize,
}

#[derive(Debug, Default)]
struct Group<'a> {
    name: &'a str,
    args: Vec<Value<'a>>,
    line: usize,
    attrs: Vec<Attr<'a>>,
    groups: Vec<Self>,
}

impl<'a> Group<'a> {
    fn attr(&self, name: &str) -> Option<&Attr<'a>> {
        self.attrs.iter().find(|a| a.name == name)
    }

    fn attr_text(&self, name: &str) -> Option<&'a str> {
        self.attr(name).and_then(|a| a.values.first()).map(|v| v.0)
    }

    fn groups_named<'s>(&'s self, name: &'s str) -> impl Iterator<Item = &'s Group<'a>> + 's {
        self.groups.iter().filter(move |g| g.name == name)
    }
}

/// Reads statements up to a closing brace (or the end of input when `name`
/// is `None`) into a group.
fn parse_statements<'a>(
    lx: &mut Lexer<'a>,
    name: Option<&'a str>,
) -> Result<Group<'a>, ParseError> {
    let mut group = Group {
        name: name.unwrap_or(""),
        ..Group::default()
    };
    loop {
        let Some((tok, line)) = lx.next()? else {
            if name.is_some() {
                return Err(lx.error(line_of_end(lx), "missing '}'".into()));
            }
            return Ok(group);
        };
        match tok {
            Tok::RBrace if name.is_some() => return Ok(group),
            Tok::Word(word) => parse_statement(lx, word, line, &mut group)?,
            Tok::Semi => {}
            other => return Err(lx.error(line, format!("unexpected {other:?}"))),
        }
    }
}

const fn line_of_end(lx: &Lexer<'_>) -> usize {
    lx.line
}

fn parse_statement<'a>(
    lx: &mut Lexer<'a>,
    word: &'a str,
    line: usize,
    into: &mut Group<'a>,
) -> Result<(), ParseError> {
    match lx.next()? {
        Some((Tok::Colon, _)) => {
            let mut values = Vec::new();
            loop {
                match lx.next()? {
                    Some((Tok::Semi, _)) => break,
                    Some((Tok::Word(w) | Tok::Str(w), _)) => values.push(Value(w)),
                    Some((Tok::RBrace, l)) => {
                        return Err(lx.error(l, format!("attribute '{word}' lacks ';'")));
                    }
                    Some((Tok::Comma, _)) => {}
                    Some((tok, l)) => {
                        return Err(lx.error(l, format!("unexpected {tok:?} in '{word}'")));
                    }
                    None => return Err(lx.error(line, format!("attribute '{word}' lacks ';'"))),
                }
            }
            into.attrs.push(Attr {
                name: word,
                values,
                line,
            });
            Ok(())
        }
        Some((Tok::LParen, _)) => {
            let args = parse_args(lx, word, line)?;
            match lx.next()? {
                Some((Tok::LBrace, _)) => {
                    let mut group = parse_statements(lx, Some(word))?;
                    group.args = args;
                    group.line = line;
                    into.groups.push(group);
                    Ok(())
                }
                Some((Tok::Semi, _)) => {
                    into.attrs.push(Attr {
                        name: word,
                        values: args,
                        line,
                    });
                    Ok(())
                }
                Some((Tok::RBrace, l)) => {
                    into.attrs.push(Attr {
                        name: word,
                        values: args,
                        line,
                    });
                    lx.push_back(Tok::RBrace, l);
                    Ok(())
                }
                Some((tok, l)) => {
                    Err(lx.error(l, format!("unexpected {tok:?} after '{word}(...)'")))
                }
                None => Err(lx.error(line, format!("'{word}' ends the file"))),
            }
        }
        Some((tok, l)) => Err(lx.error(l, format!("unexpected {tok:?} after '{word}'"))),
        None => Err(lx.error(line, format!("'{word}' ends the file"))),
    }
}

fn parse_args<'a>(
    lx: &mut Lexer<'a>,
    word: &str,
    line: usize,
) -> Result<Vec<Value<'a>>, ParseError> {
    let mut args = Vec::new();
    loop {
        match lx.next()? {
            Some((Tok::RParen, _)) => return Ok(args),
            Some((Tok::Word(w) | Tok::Str(w), _)) => args.push(Value(w)),
            Some((Tok::Comma, _)) => {}
            Some((tok, l)) => {
                return Err(lx.error(l, format!("unexpected {tok:?} in arguments of '{word}'")));
            }
            None => return Err(lx.error(line, format!("'{word}(' is never closed"))),
        }
    }
}

#[derive(Clone, Debug)]
struct Template {
    axes: Vec<TemplateAxis>,
    indices: Vec<Option<Vec<f64>>>,
}

/// The variable along one axis of a table template, as named and, when it
/// is one timing tables are looked up along, as read.
#[derive(Clone, Debug)]
struct TemplateAxis {
    name: String,
    variable: Option<AxisVariable>,
}

/// How a timing table is looked up, which fixes the axes it may vary along.
#[derive(Clone, Copy, Debug)]
enum TableModel {
    /// A delay or slew table.
    Gate,
    /// A check's constraint table.
    Check,
}

impl TableModel {
    /// Whether tables of this model may vary along `variable`, as
    /// `GateTableModel::checkAxis` and `CheckTableModel::checkAxis` allow.
    const fn accepts(self, variable: AxisVariable) -> bool {
        match self {
            Self::Gate => matches!(
                variable,
                AxisVariable::InputTransition
                    | AxisVariable::OutputCapacitance
                    | AxisVariable::RelatedOutputCapacitance
            ),
            Self::Check => matches!(
                variable,
                AxisVariable::RelatedPinTransition
                    | AxisVariable::ConstrainedPinTransition
                    | AxisVariable::RelatedOutputCapacitance
            ),
        }
    }
}

/// A `type` group: the bits of a bus.
#[derive(Clone, Debug)]
struct BusType {
    bit_from: i64,
    bit_to: i64,
}

impl BusType {
    /// The bit indices in declaration order.
    fn bits(&self) -> Vec<i64> {
        if self.bit_from >= self.bit_to {
            (self.bit_to..=self.bit_from).rev().collect()
        } else {
            (self.bit_from..=self.bit_to).collect()
        }
    }
}

struct Interpreter {
    path: PathBuf,
    units: LibraryUnits,
    defaults: Defaults,
    templates: HashMap<String, Template>,
    bus_types: HashMap<String, BusType>,
}

impl Interpreter {
    fn new(path: &Path) -> Self {
        Self {
            path: path.to_path_buf(),
            units: LibraryUnits::default(),
            defaults: Defaults::default(),
            templates: HashMap::new(),
            bus_types: HashMap::new(),
        }
    }

    fn error(&self, line: usize, message: String) -> ParseError {
        ParseError::Syntax {
            path: self.path.clone(),
            line,
            message,
        }
    }

    fn number(&self, attr: &Attr<'_>) -> Result<f64, ParseError> {
        let text = attr
            .values
            .first()
            .ok_or_else(|| self.error(attr.line, format!("'{}' has no value", attr.name)))?
            .0;
        text.trim().parse::<f64>().map_err(|_| {
            self.error(
                attr.line,
                format!("'{}' is not a number: '{text}'", attr.name),
            )
        })
    }

    fn number_attr(&self, group: &Group<'_>, name: &str) -> Result<Option<f64>, ParseError> {
        group.attr(name).map(|a| self.number(a)).transpose()
    }

    fn bool_attr(group: &Group<'_>, name: &str) -> bool {
        group
            .attr_text(name)
            .is_some_and(|v| v.eq_ignore_ascii_case("true"))
    }

    fn function(&self, attr: &Attr<'_>) -> Result<FuncExpr, ParseError> {
        let text = attr
            .values
            .iter()
            .map(|v| v.0)
            .collect::<Vec<_>>()
            .join(" ");
        FuncExpr::parse(&text).map_err(|e| self.error(attr.line, e.to_string()))
    }

    fn function_attr(&self, group: &Group<'_>, name: &str) -> Result<Option<FuncExpr>, ParseError> {
        group.attr(name).map(|a| self.function(a)).transpose()
    }

    fn library(mut self, group: &Group<'_>) -> Result<Liberty, ParseError> {
        let name = group.args.first().map_or("library", |v| v.0).to_string();
        let mut lib = Liberty::new(name);
        self.units = self.read_units(group)?;
        lib.units = self.units;
        lib.thresholds = self.thresholds(group)?;
        self.defaults = self.defaults(group)?;
        lib.defaults = self.defaults;
        for template in group.groups_named("lu_table_template") {
            let (name, t) = self.template(template)?;
            let _ = self.templates.insert(name, t);
        }
        for bus_type in group.groups_named("type") {
            let (name, t) = self.bus_type(bus_type)?;
            let _ = self.bus_types.insert(name, t);
        }
        for cell in group.groups_named("cell") {
            let parsed = self.cell(cell)?;
            let line = cell.line;
            let _ = lib
                .add_cell(parsed)
                .map_err(|e| self.error(line, format!("cell '{}' is defined twice", e.name)))?;
        }
        Ok(lib)
    }

    fn read_units(&self, group: &Group<'_>) -> Result<LibraryUnits, ParseError> {
        let mut units = self.units;
        if let Some(attr) = group.attr("time_unit") {
            let text = attr.values.first().map_or("", |v| v.0);
            units.time = unit_factor(
                text,
                &[
                    ("ps", 1e-12),
                    ("ns", 1e-9),
                    ("us", 1e-6),
                    ("ms", 1e-3),
                    ("s", 1.0),
                ],
            )
            .map(Seconds::new)
            .ok_or_else(|| self.error(attr.line, format!("unknown time unit '{text}'")))?;
        }
        if let Some(attr) = group.attr("capacitive_load_unit") {
            let [scale, unit] = attr.values.as_slice() else {
                return Err(
                    self.error(attr.line, "capacitive_load_unit wants (scale, unit)".into())
                );
            };
            let scale: f64 = scale.0.trim().parse().map_err(|_| {
                self.error(
                    attr.line,
                    format!("capacitive_load_unit scale '{}' is not a number", scale.0),
                )
            })?;
            let factor = match unit.0.trim().to_ascii_lowercase().as_str() {
                "ff" => 1e-15,
                "pf" => 1e-12,
                "nf" => 1e-9,
                other => {
                    return Err(
                        self.error(attr.line, format!("unknown capacitance unit '{other}'"))
                    );
                }
            };
            units.capacitance = Farads::new(scale * factor);
        }
        if let Some(attr) = group.attr("leakage_power_unit") {
            let text = attr.values.first().map_or("", |v| v.0);
            units.leakage_power = unit_factor(
                text,
                &[
                    ("kw", 1e3),
                    ("w", 1.0),
                    ("mw", 1e-3),
                    ("uw", 1e-6),
                    ("nw", 1e-9),
                    ("pw", 1e-12),
                    ("fw", 1e-15),
                ],
            )
            .map(Watts::new)
            .ok_or_else(|| self.error(attr.line, format!("unknown leakage power unit '{text}'")))?;
        }
        Ok(units)
    }

    /// The value of each `leakage_power` group of a cell; like `OpenSTA`,
    /// which only warns, a group without one is passed over.
    fn state_leakages(&self, cell: &Group<'_>) -> Result<Vec<Watts>, ParseError> {
        let mut out = Vec::new();
        for group in cell.groups_named("leakage_power") {
            if let Some(value) = self.number_attr(group, "value")? {
                out.push(value * self.units.leakage_power);
            }
        }
        Ok(out)
    }

    fn thresholds(&self, group: &Group<'_>) -> Result<crate::db::liberty::Thresholds, ParseError> {
        let mut t = crate::db::liberty::Thresholds::default();
        let pct = |name: &str, slot: &mut f64| -> Result<(), ParseError> {
            if let Some(v) = self.number_attr(group, name)? {
                *slot = v / 100.0;
            }
            Ok(())
        };
        pct("slew_lower_threshold_pct_rise", &mut t.slew_lower.rise)?;
        pct("slew_lower_threshold_pct_fall", &mut t.slew_lower.fall)?;
        pct("slew_upper_threshold_pct_rise", &mut t.slew_upper.rise)?;
        pct("slew_upper_threshold_pct_fall", &mut t.slew_upper.fall)?;
        pct("input_threshold_pct_rise", &mut t.input.rise)?;
        pct("input_threshold_pct_fall", &mut t.input.fall)?;
        pct("output_threshold_pct_rise", &mut t.output.rise)?;
        pct("output_threshold_pct_fall", &mut t.output.fall)?;
        if let Some(v) = self.number_attr(group, "slew_derate_from_library")? {
            t.slew_derate = v;
        }
        Ok(t)
    }

    fn defaults(&self, group: &Group<'_>) -> Result<Defaults, ParseError> {
        let time = self.units.time;
        let cap = self.units.capacitance;
        Ok(Defaults {
            max_transition: self
                .number_attr(group, "default_max_transition")?
                .map(|v| v * time),
            max_capacitance: self
                .number_attr(group, "default_max_capacitance")?
                .map(|v| v * cap),
            max_fanout: self.number_attr(group, "default_max_fanout")?,
            fanout_load: self.number_attr(group, "default_fanout_load")?,
            input_pin_cap: self
                .number_attr(group, "default_input_pin_cap")?
                .unwrap_or(0.0)
                * cap,
            output_pin_cap: self
                .number_attr(group, "default_output_pin_cap")?
                .unwrap_or(0.0)
                * cap,
        })
    }

    fn template(&self, group: &Group<'_>) -> Result<(String, Template), ParseError> {
        let name = group
            .args
            .first()
            .ok_or_else(|| self.error(group.line, "lu_table_template without a name".into()))?
            .0
            .to_string();
        let mut axes = Vec::new();
        let mut indices = Vec::new();
        for k in 1..=3 {
            let Some(var) = group.attr_text(&format!("variable_{k}")) else {
                break;
            };
            let variable = axis_variable(var);
            let index = match (variable, group.attr(&format!("index_{k}"))) {
                (Some(variable), Some(attr)) => Some(self.axis_values(attr, variable)?),
                _ => None,
            };
            axes.push(TemplateAxis {
                name: var.to_string(),
                variable,
            });
            indices.push(index);
        }
        Ok((name, Template { axes, indices }))
    }

    fn bus_type(&self, group: &Group<'_>) -> Result<(String, BusType), ParseError> {
        let name = group
            .args
            .first()
            .ok_or_else(|| self.error(group.line, "type without a name".into()))?
            .0
            .to_string();
        let bit = |attr: &str| -> Result<i64, ParseError> {
            let a = group
                .attr(attr)
                .ok_or_else(|| self.error(group.line, format!("type '{name}' lacks {attr}")))?;
            let text = a.values.first().map_or("", |v| v.0);
            text.trim().parse().map_err(|_| {
                self.error(
                    a.line,
                    format!("type '{name}' has a non-integer {attr} '{text}'"),
                )
            })
        };
        let bus_type = BusType {
            bit_from: bit("bit_from")?,
            bit_to: bit("bit_to")?,
        };
        Ok((name, bus_type))
    }

    fn axis_values(&self, attr: &Attr<'_>, variable: AxisVariable) -> Result<Vec<f64>, ParseError> {
        let scale = match variable {
            AxisVariable::InputTransition
            | AxisVariable::RelatedPinTransition
            | AxisVariable::ConstrainedPinTransition => self.units.time.get(),
            AxisVariable::OutputCapacitance | AxisVariable::RelatedOutputCapacitance => {
                self.units.capacitance.get()
            }
        };
        let values = self.numbers(attr)?;
        Ok(values.into_iter().map(|v| v * scale).collect())
    }

    fn numbers(&self, attr: &Attr<'_>) -> Result<Vec<f64>, ParseError> {
        let mut out = Vec::new();
        for value in &attr.values {
            for piece in value.0.split(',') {
                let piece = piece.trim();
                if piece.is_empty() {
                    continue;
                }
                let v: f64 = piece.parse().map_err(|_| {
                    self.error(
                        attr.line,
                        format!("'{}' holds a non-number '{piece}'", attr.name),
                    )
                })?;
                out.push(v);
            }
        }
        Ok(out)
    }

    fn cell(&self, group: &Group<'_>) -> Result<LibCell, ParseError> {
        let mut cell = self.cell_attributes(group)?;
        let buses = self.add_ports(&mut cell, group)?;
        self.add_pin_timing(&mut cell, group)?;
        self.add_bus_timing(&mut cell, &buses)?;
        for (kind, name) in [
            (SequentialKind::FlipFlop, "ff"),
            (SequentialKind::Latch, "latch"),
        ] {
            for seq in group.groups_named(name) {
                cell.sequentials.push(self.sequential(seq, kind)?);
            }
        }
        Ok(cell)
    }

    /// A cell group's name and attributes, without its ports, arcs or
    /// sequentials.
    fn cell_attributes(&self, group: &Group<'_>) -> Result<LibCell, ParseError> {
        let name = group
            .args
            .first()
            .ok_or_else(|| self.error(group.line, "cell without a name".into()))?
            .0
            .to_string();
        if let Some(g) = group.groups_named("bundle").next() {
            return Err(self.error(
                g.line,
                format!("cell '{name}' has a 'bundle' group, which is not supported"),
            ));
        }
        Ok(LibCell {
            name,
            area: CellArea::new(self.number_attr(group, "area")?.unwrap_or(0.0)),
            dont_use: Self::bool_attr(group, "dont_use"),
            dont_touch: Self::bool_attr(group, "dont_touch"),
            is_clock_gate: group.attr("clock_gating_integrated_cell").is_some(),
            is_clock_cell: Self::bool_attr(group, "is_clock_cell"),
            library: 0,
            is_power_cell: Self::bool_attr(group, "always_on")
                || Self::bool_attr(group, "is_isolation_cell")
                || Self::bool_attr(group, "is_level_shifter"),
            is_isolation_cell: Self::bool_attr(group, "is_isolation_cell"),
            // OpenSTA reads `is_pad`, then `pad_cell` over it.
            is_pad: group
                .attr_text("pad_cell")
                .or_else(|| group.attr_text("is_pad"))
                .is_some_and(|v| v.eq_ignore_ascii_case("true")),
            footprint: group.attr_text("cell_footprint").map(str::to_string),
            user_function_class: group.attr_text("user_function_class").map(str::to_string),
            ports: IdVec::new(),
            pg_pins: group
                .groups_named("pg_pin")
                .filter_map(|g| g.args.first())
                .map(|v| v.0.to_string())
                .collect(),
            arc_sets: IdVec::new(),
            sequentials: Vec::new(),
            statetable: group
                .groups_named("statetable")
                .next()
                .map(|g| self.statetable(g))
                .transpose()?,
            leakage: self
                .number_attr(group, "cell_leakage_power")?
                .map(|v| v * self.units.leakage_power),
            state_leakages: self.state_leakages(group)?,
        })
    }

    /// Adds the ports of the cell group's pins, then of its buses' bits,
    /// returning the buses.
    fn add_ports<'g, 'a>(
        &self,
        cell: &mut LibCell,
        group: &'g Group<'a>,
    ) -> Result<Vec<Bus<'g, 'a>>, ParseError> {
        for pin in group.groups_named("pin") {
            for arg in &pin.args {
                let _ = cell.ports.push(self.port(pin, arg.0)?);
            }
        }
        let mut buses = Vec::new();
        for group in group.groups_named("bus") {
            let bus = self.bus(group, &cell.name)?;
            for bit in &bus.bits {
                let _ = cell.ports.push(self.bus_bit_port(&bus, bit)?);
            }
            buses.push(bus);
        }
        Ok(buses)
    }

    /// Adds the timing arcs of the cell group's pins.
    fn add_pin_timing(&self, cell: &mut LibCell, group: &Group<'_>) -> Result<(), ParseError> {
        for pin in group.groups_named("pin") {
            for arg in &pin.args {
                let to = cell
                    .port(arg.0)
                    .ok_or_else(|| self.error(pin.line, format!("pin '{}' vanished", arg.0)))?;
                for timing in pin.groups_named("timing") {
                    self.timing(cell, to, timing)?;
                }
            }
        }
        Ok(())
    }

    /// Adds the timing arcs of each bus bit: the bus's own, then those of
    /// the pin groups within it that name the bit.
    fn add_bus_timing(&self, cell: &mut LibCell, buses: &[Bus<'_, '_>]) -> Result<(), ParseError> {
        for bus in buses {
            for bit in &bus.bits {
                let to = cell.port(bit).ok_or_else(|| {
                    self.error(bus.group.line, format!("bus bit '{bit}' vanished"))
                })?;
                for timing in bus.group.groups_named("timing") {
                    self.timing(cell, to, timing)?;
                }
                for pin in bus.pins_naming(bit) {
                    for timing in pin.groups_named("timing") {
                        self.timing(cell, to, timing)?;
                    }
                }
            }
        }
        Ok(())
    }

    /// A `bus` group's bits and the pin groups within it.
    fn bus<'g, 'a>(&self, group: &'g Group<'a>, cell: &str) -> Result<Bus<'g, 'a>, ParseError> {
        let name = group
            .args
            .first()
            .ok_or_else(|| {
                self.error(
                    group.line,
                    format!("cell '{cell}' has a bus without a name"),
                )
            })?
            .0;
        let bits = self.bus_bits(group, name, cell)?;
        let sub_pins = group
            .groups_named("pin")
            .map(|pin| Ok((pin, self.sub_pin_bits(pin, name, &bits)?)))
            .collect::<Result<_, ParseError>>()?;
        Ok(Bus {
            group,
            bits,
            sub_pins,
        })
    }

    /// The bit pins bus `name` declares, named `name[bit]`.
    fn bus_bits(&self, bus: &Group<'_>, name: &str, cell: &str) -> Result<Vec<String>, ParseError> {
        let type_name = bus.attr_text("bus_type").ok_or_else(|| {
            self.error(
                bus.line,
                format!("bus '{name}' of '{cell}' has no bus_type"),
            )
        })?;
        let bus_type = self.bus_types.get(type_name).ok_or_else(|| {
            self.error(
                bus.line,
                format!("bus '{name}' of '{cell}' has unknown type '{type_name}'"),
            )
        })?;
        Ok(bus_type
            .bits()
            .iter()
            .map(|i| format!("{name}[{i}]"))
            .collect())
    }

    /// The bits of bus `bus` a pin group within it names: by each argument
    /// one bit, a range `bus[from:to]` from `from` to `to`, or the whole
    /// bus, as `OpenSTA`'s `PortNameBitIterator` reads them.
    fn sub_pin_bits(
        &self,
        pin: &Group<'_>,
        bus: &str,
        bits: &[String],
    ) -> Result<Vec<String>, ParseError> {
        let mut named = Vec::new();
        for arg in pin.args.iter().map(|a| a.0) {
            if arg == bus {
                named.extend(bits.iter().cloned());
                continue;
            }
            let expanded = match bit_range(arg) {
                Some((base, from, to)) if base == bus => range_bits(base, from, to),
                _ => vec![arg.to_string()],
            };
            for bit in expanded {
                if !bits.contains(&bit) {
                    return Err(
                        self.error(pin.line, format!("pin '{bit}' is not a bit of bus '{bus}'"))
                    );
                }
                named.push(bit);
            }
        }
        Ok(named)
    }

    /// A bus bit's pin: the bus's, with what pin groups within the bus
    /// state about the bit in their place, as `OpenSTA` reads it.
    fn bus_bit_port(&self, bus: &Bus<'_, '_>, bit: &str) -> Result<LibPort, ParseError> {
        let mut port = self.port(bus.group, bit)?;
        for pin in bus.pins_naming(bit) {
            if let Some(direction) = self.direction(pin, bit)? {
                port.direction = direction;
            }
            self.read_port_attributes(pin, &mut port)?;
        }
        Ok(port)
    }

    fn port(&self, group: &Group<'_>, name: &str) -> Result<LibPort, ParseError> {
        let direction = self
            .direction(group, name)?
            .ok_or_else(|| self.error(group.line, format!("pin '{name}' has no direction")))?;
        let default_cap = match direction {
            PortDirection::Input | PortDirection::Inout => self.defaults.input_pin_cap,
            PortDirection::Output | PortDirection::Internal => self.defaults.output_pin_cap,
        };
        let mut port = LibPort {
            name: name.to_string(),
            direction,
            capacitance: EdgeValues::both(default_cap),
            max_capacitance: None,
            max_transition: None,
            max_fanout: None,
            fanout_load: None,
            function: None,
            three_state: None,
            is_clock: false,
            clock_gate: ClockGatePin::None,
            clock_tree_paths: Vec::new(),
        };
        self.read_port_attributes(group, &mut port)?;
        Ok(port)
    }

    /// The direction a pin group states, if it states one.
    fn direction(
        &self,
        group: &Group<'_>,
        name: &str,
    ) -> Result<Option<PortDirection>, ParseError> {
        match group.attr_text("direction") {
            Some("input") => Ok(Some(PortDirection::Input)),
            Some("output") => Ok(Some(PortDirection::Output)),
            Some("inout") => Ok(Some(PortDirection::Inout)),
            Some("internal") => Ok(Some(PortDirection::Internal)),
            Some(other) => {
                Err(self.error(group.line, format!("pin '{name}' has direction '{other}'")))
            }
            None => Ok(None),
        }
    }

    /// Sets on `port` what `group` states about it, leaving the rest.
    fn read_port_attributes(
        &self,
        group: &Group<'_>,
        port: &mut LibPort,
    ) -> Result<(), ParseError> {
        let cap = self.units.capacitance;
        let both = self.number_attr(group, "capacitance")?.map(|v| v * cap);
        let rise = self
            .number_attr(group, "rise_capacitance")?
            .map(|v| v * cap);
        let fall = self
            .number_attr(group, "fall_capacitance")?
            .map(|v| v * cap);
        if let Some(rise) = rise.or(both) {
            port.capacitance.rise = rise;
        }
        if let Some(fall) = fall.or(both) {
            port.capacitance.fall = fall;
        }
        port.max_capacitance = self
            .number_attr(group, "max_capacitance")?
            .map(|v| v * cap)
            .or(port.max_capacitance);
        port.max_transition = self
            .number_attr(group, "max_transition")?
            .map(|v| v * self.units.time)
            .or(port.max_transition);
        port.max_fanout = self.number_attr(group, "max_fanout")?.or(port.max_fanout);
        port.fanout_load = self.number_attr(group, "fanout_load")?.or(port.fanout_load);
        if let Some(function) = self.function_attr(group, "function")? {
            port.function = Some(function);
        }
        if let Some(three_state) = self.function_attr(group, "three_state")? {
            port.three_state = Some(three_state);
        }
        if group.attr("clock").is_some() {
            port.is_clock = Self::bool_attr(group, "clock");
        }
        if let Some(gate) = clock_gate_pin(group) {
            port.clock_gate = gate;
        }
        Ok(())
    }

    fn timing(
        &self,
        cell: &mut LibCell,
        to: LibPortId,
        group: &Group<'_>,
    ) -> Result<(), ParseError> {
        let timing_type = group.attr_text("timing_type").unwrap_or("combinational");
        if let Some(bound) = clock_tree_path_bound(timing_type) {
            let path = self.clock_tree_path(group, bound)?;
            cell.ports[to].clock_tree_paths.push(path);
            return Ok(());
        }
        let kind = arc_role(timing_type).ok_or_else(|| {
            // NOTE: LibertyReader::readTimingType warns (1244) and reads an
            // unknown type as combinational; PARE refuses to guess its role.
            let line = group.attr("timing_type").map_or(group.line, |a| a.line);
            self.error(line, format!("unknown timing_type '{timing_type}'"))
        })?;
        let stated_sense = stated_sense(group);
        let when = self.function_attr(group, "when")?;
        let tables = self.arc_tables(group, kind.role)?;
        let related = group
            .attr("related_pin")
            .ok_or_else(|| self.error(group.line, "timing group without related_pin".into()))?;
        for related_name in related.values.iter().flat_map(|v| v.0.split_whitespace()) {
            let from = cell.port(related_name).ok_or_else(|| {
                self.error(
                    related.line,
                    format!(
                        "cell '{}' has no pin '{related_name}'; a related bus is not supported",
                        cell.name
                    ),
                )
            })?;
            let sense = stated_sense
                .or_else(|| inferred_sense(&cell.ports[to], kind.role, related_name))
                .unwrap_or(TimingSense::NonUnate);
            let _ = cell.arc_sets.push(TimingArcSet {
                from,
                to,
                role: kind.role,
                sense,
                when: when.clone(),
                arcs: timing_arcs(kind, sense, &tables),
            });
        }
        Ok(())
    }

    /// The clock tree path a `min_clock_tree_path` or `max_clock_tree_path`
    /// timing group gives on its `bound` side: its sense, unknown taken as
    /// non-unate as `OpenSTA` builds its arcs, and its delay tables.
    fn clock_tree_path(
        &self,
        group: &Group<'_>,
        bound: MinMax,
    ) -> Result<ClockTreePath, ParseError> {
        let delay = |name: &str| {
            group
                .groups_named(name)
                .next()
                .map(|g| self.table(g, TableModel::Gate))
                .transpose()
        };
        Ok(ClockTreePath {
            bound,
            sense: stated_sense(group).unwrap_or(TimingSense::NonUnate),
            delays: EdgeValues {
                rise: delay("cell_rise")?,
                fall: delay("cell_fall")?,
            },
        })
    }

    /// The delay and slew tables, or the constraint tables of a check, a
    /// timing group of `role` gives.
    fn arc_tables(&self, group: &Group<'_>, role: ArcRole) -> Result<ArcTables, ParseError> {
        let (model, names): (TableModel, &[(&str, Edge, TableKind)]) = if role.is_delay() {
            (
                TableModel::Gate,
                &[
                    ("cell_rise", Edge::Rise, TableKind::Delay),
                    ("cell_fall", Edge::Fall, TableKind::Delay),
                    ("rise_transition", Edge::Rise, TableKind::Slew),
                    ("fall_transition", Edge::Fall, TableKind::Slew),
                ],
            )
        } else if role.check_side().is_some() {
            (
                TableModel::Check,
                &[
                    ("rise_constraint", Edge::Rise, TableKind::Delay),
                    ("fall_constraint", Edge::Fall, TableKind::Delay),
                ],
            )
        } else {
            return Ok(ArcTables(Vec::new()));
        };
        let tables = names
            .iter()
            .map(|&(name, edge, kind)| {
                group
                    .groups_named(name)
                    .next()
                    .map(|g| self.table(g, model).map(|t| (edge, kind, t)))
                    .transpose()
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(ArcTables(tables.into_iter().flatten().collect()))
    }

    fn table(&self, group: &Group<'_>, model: TableModel) -> Result<Table, ParseError> {
        let template_name = group.args.first().map_or("scalar", |v| v.0);
        let template = if template_name == "scalar" {
            Template {
                axes: Vec::new(),
                indices: Vec::new(),
            }
        } else {
            self.templates.get(template_name).cloned().ok_or_else(|| {
                self.error(
                    group.line,
                    format!("unknown table template '{template_name}'"),
                )
            })?
        };
        let mut axes = Vec::new();
        for (k, axis) in template.axes.iter().enumerate() {
            let variable = self.table_variable(group, axis, model)?;
            let values = match group.attr(&format!("index_{}", k + 1)) {
                Some(attr) => self.axis_values(attr, variable)?,
                None => template.indices[k].clone().ok_or_else(|| {
                    self.error(
                        group.line,
                        format!(
                            "'{}' gives no index_{} and its template has none",
                            group.name,
                            k + 1
                        ),
                    )
                })?,
            };
            axes.push(Axis { variable, values });
        }
        let values_attr = group
            .attr("values")
            .ok_or_else(|| self.error(group.line, format!("'{}' has no values", group.name)))?;
        let values: Vec<Seconds> = self
            .numbers(values_attr)?
            .into_iter()
            .map(|v| v * self.units.time)
            .collect();
        let table = Table { axes, values };
        if !table.is_consistent() {
            return Err(self.error(
                group.line,
                format!(
                    "'{}' has {} values for its axes",
                    group.name,
                    table.values.len()
                ),
            ));
        }
        Ok(table)
    }

    /// The variable a `model` table varies along `axis` of its template.
    fn table_variable(
        &self,
        group: &Group<'_>,
        axis: &TemplateAxis,
        model: TableModel,
    ) -> Result<AxisVariable, ParseError> {
        // NOTE: LibertyReader::readTableModel warns (1251) and keeps such a
        // table, looking it up at 0 along the axis; PARE refuses to time by it.
        axis.variable.filter(|&v| model.accepts(v)).ok_or_else(|| {
            self.error(
                group.line,
                format!("'{}' cannot vary along '{}'", group.name, axis.name),
            )
        })
    }

    fn statetable(&self, group: &Group<'_>) -> Result<Statetable, ParseError> {
        let names = |v: Option<&Value<'_>>| -> Vec<String> {
            v.map(|v| v.0.split_whitespace().map(str::to_string).collect())
                .unwrap_or_default()
        };
        let table = group
            .attr("table")
            .ok_or_else(|| self.error(group.line, "statetable without a table".into()))?;
        let text = table
            .values
            .iter()
            .map(|v| v.0)
            .collect::<Vec<_>>()
            .join(" ");
        let rows = text
            .split(',')
            .map(|row| {
                row.split_whitespace()
                    .filter(|w| *w != "\\")
                    .collect::<Vec<_>>()
                    .join(" ")
            })
            .filter(|row| !row.is_empty())
            .collect();
        Ok(Statetable {
            inputs: names(group.args.first()),
            outputs: names(group.args.get(1)),
            rows,
        })
    }

    fn sequential(
        &self,
        group: &Group<'_>,
        kind: SequentialKind,
    ) -> Result<Sequential, ParseError> {
        let output = group
            .args
            .first()
            .ok_or_else(|| {
                self.error(
                    group.line,
                    format!("'{}' group without a state", group.name),
                )
            })?
            .0
            .to_string();
        let (clock, data) = match kind {
            SequentialKind::FlipFlop => ("clocked_on", "next_state"),
            SequentialKind::Latch => ("enable", "data_in"),
        };
        Ok(Sequential {
            kind,
            output,
            output_inv: group.args.get(1).map(|v| v.0.to_string()),
            clock: self.function_attr(group, clock)?,
            data: self.function_attr(group, data)?,
            clear: self.function_attr(group, "clear")?,
            preset: self.function_attr(group, "preset")?,
        })
    }
}

/// The SI factor of a unit string such as `1ns` or `10ps`.
fn unit_factor(text: &str, units: &[(&str, f64)]) -> Option<f64> {
    let text = text.trim();
    let split = text.find(|c: char| c.is_ascii_alphabetic())?;
    let (number, unit) = text.split_at(split);
    let scale: f64 = if number.is_empty() {
        1.0
    } else {
        number.parse().ok()?
    };
    let unit = unit.to_ascii_lowercase();
    units
        .iter()
        .find(|(name, _)| *name == unit)
        .map(|(_, factor)| scale * factor)
}

/// The variable a template axis named `name` varies, when timing tables are
/// looked up along it.
fn axis_variable(name: &str) -> Option<AxisVariable> {
    match name {
        "input_net_transition" | "input_transition_time" => Some(AxisVariable::InputTransition),
        "total_output_net_capacitance" => Some(AxisVariable::OutputCapacitance),
        "related_out_total_output_net_capacitance" => Some(AxisVariable::RelatedOutputCapacitance),
        "related_pin_transition" => Some(AxisVariable::RelatedPinTransition),
        "constrained_pin_transition" => Some(AxisVariable::ConstrainedPinTransition),
        _ => None,
    }
}

/// The sense of an arc from `from` that its timing group leaves out, as
/// `OpenSTA` finds it: from the output's function for a combinational arc,
/// from its `three_state` condition for a tristate one.
fn inferred_sense(to: &LibPort, role: ArcRole, from: &str) -> Option<TimingSense> {
    match role {
        ArcRole::Combinational => to.function.as_ref()?.port_timing_sense(from),
        ArcRole::TristateEnable => {
            Some(to.three_state.as_ref()?.port_timing_sense(from)?.opposite())
        }
        ArcRole::TristateDisable => to.three_state.as_ref()?.port_timing_sense(from),
        _ => None,
    }
}

/// A `bus` group: its bits, and each pin group within it with the bits it
/// names.
struct Bus<'g, 'a> {
    group: &'g Group<'a>,
    bits: Vec<String>,
    sub_pins: Vec<(&'g Group<'a>, Vec<String>)>,
}

impl<'g, 'a> Bus<'g, 'a> {
    /// The pin groups within the bus that name `bit`.
    fn pins_naming<'s>(&'s self, bit: &'s str) -> impl Iterator<Item = &'g Group<'a>> + 's {
        self.sub_pins
            .iter()
            .filter(move |(_, bits)| bits.iter().any(|b| b == bit))
            .map(|&(pin, _)| pin)
    }
}

/// The bus and ends of a range name `bus[from:to]`.
fn bit_range(name: &str) -> Option<(&str, i64, i64)> {
    let (base, rest) = name.split_once('[')?;
    let (from, to) = rest.strip_suffix(']')?.split_once(':')?;
    Some((base, from.trim().parse().ok()?, to.trim().parse().ok()?))
}

/// The bit names of `bus` from `from` to `to`, either way round.
fn range_bits(bus: &str, from: i64, to: i64) -> Vec<String> {
    let indices: Vec<i64> = if from <= to {
        (from..=to).collect()
    } else {
        (to..=from).rev().collect()
    };
    indices.iter().map(|i| format!("{bus}[{i}]")).collect()
}

/// The part a pin group gives its pin in an integrated clock gate, if any.
fn clock_gate_pin(group: &Group<'_>) -> Option<ClockGatePin> {
    [
        ("clock_gate_clock_pin", ClockGatePin::Clock),
        ("clock_gate_enable_pin", ClockGatePin::Enable),
        ("clock_gate_test_pin", ClockGatePin::Test),
        ("clock_gate_out_pin", ClockGatePin::Out),
    ]
    .into_iter()
    .find(|&(name, _)| Interpreter::bool_attr(group, name))
    .map(|(_, gate)| gate)
}

/// What a timing group's `timing_type` makes of its arcs: their role, the
/// edge a clock or enable launches them on and the one output edge they
/// are restricted to, if any.
#[derive(Clone, Copy)]
struct ArcKind {
    role: ArcRole,
    from_edge: Option<Edge>,
    to_edge: Option<Edge>,
}

/// The tables a timing group gives, each with its output edge and kind.
struct ArcTables(Vec<(Edge, TableKind, Table)>);

/// What a timing group's table gives an arc.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TableKind {
    /// Its delay, or a check's constraint.
    Delay,
    /// The slew it leaves.
    Slew,
}

impl ArcTables {
    fn get(&self, edge: Edge, kind: TableKind) -> Option<Table> {
        self.0
            .iter()
            .find(|(e, k, _)| *e == edge && *k == kind)
            .map(|(_, _, t)| t.clone())
    }
}

/// The sense a timing group states, if it states one.
fn stated_sense(group: &Group<'_>) -> Option<TimingSense> {
    match group.attr_text("timing_sense") {
        Some("positive_unate") => Some(TimingSense::PositiveUnate),
        Some("negative_unate") => Some(TimingSense::NegativeUnate),
        Some("non_unate") => Some(TimingSense::NonUnate),
        _ => None,
    }
}

/// The arcs of a timing group of `kind` and `sense` over its `tables`.
fn timing_arcs(kind: ArcKind, sense: TimingSense, tables: &ArcTables) -> Vec<TimingArc> {
    if kind.role.is_delay() {
        return delay_edge_pairs(kind, sense)
            .into_iter()
            .filter(|&(_, to)| kind.to_edge.is_none_or(|restricted| restricted == to))
            .filter_map(|(from, to)| {
                let delay = tables.get(to, TableKind::Delay);
                let slew = tables.get(to, TableKind::Slew);
                (delay.is_some() || slew.is_some()).then_some(TimingArc {
                    from_edge: from,
                    to_edge: to,
                    delay,
                    slew,
                })
            })
            .collect();
    }
    let (Some(clock_edge), Some(_)) = (kind.from_edge, kind.role.check_side()) else {
        return Vec::new();
    };
    Edge::BOTH
        .into_iter()
        .filter_map(|to| {
            tables
                .get(to, TableKind::Delay)
                .map(|constraint| TimingArc {
                    from_edge: clock_edge,
                    to_edge: to,
                    delay: Some(constraint),
                    slew: None,
                })
        })
        .collect()
}

/// The input and output edges a delay arc of `kind` and `sense` joins, in
/// the order `OpenSTA`'s `LibertyBuilder` makes them, so whatever walks a
/// cell's arcs meets them, and breaks ties, as `OpenSTA` does.
fn delay_edge_pairs(kind: ArcKind, sense: TimingSense) -> Vec<(Edge, Edge)> {
    match (kind.role, kind.from_edge, sense) {
        (ArcRole::RegClkToQ | ArcRole::LatchEnToQ, Some(f), _) => {
            vec![(f, Edge::Rise), (f, Edge::Fall)]
        }
        (_, _, TimingSense::PositiveUnate) => {
            vec![(Edge::Rise, Edge::Rise), (Edge::Fall, Edge::Fall)]
        }
        (_, _, TimingSense::NegativeUnate) => {
            vec![(Edge::Rise, Edge::Fall), (Edge::Fall, Edge::Rise)]
        }
        (_, _, TimingSense::NonUnate) => vec![
            (Edge::Fall, Edge::Fall),
            (Edge::Rise, Edge::Fall),
            (Edge::Rise, Edge::Rise),
            (Edge::Fall, Edge::Rise),
        ],
    }
}

/// The side a clock tree path `timing_type` bounds, or none for any other.
fn clock_tree_path_bound(timing_type: &str) -> Option<MinMax> {
    match timing_type {
        "min_clock_tree_path" => Some(MinMax::Min),
        "max_clock_tree_path" => Some(MinMax::Max),
        _ => None,
    }
}

/// The role a `timing_type` names, with the input edge it fixes for a clock
/// and the output edge it restricts to, where it does, or `None` for a type
/// `OpenSTA`'s `TimingType` does not know.
fn arc_role(timing_type: &str) -> Option<ArcKind> {
    let (role, from_edge, to_edge) = match timing_type {
        "combinational" => (ArcRole::Combinational, None, None),
        "combinational_rise" => (ArcRole::Combinational, None, Some(Edge::Rise)),
        "combinational_fall" => (ArcRole::Combinational, None, Some(Edge::Fall)),
        "rising_edge" => (ArcRole::RegClkToQ, Some(Edge::Rise), None),
        "falling_edge" => (ArcRole::RegClkToQ, Some(Edge::Fall), None),
        "clear" => (ArcRole::RegSetClr, None, Some(Edge::Fall)),
        "preset" => (ArcRole::RegSetClr, None, Some(Edge::Rise)),
        "three_state_enable" => (ArcRole::TristateEnable, None, None),
        "three_state_enable_rise" => (ArcRole::TristateEnable, None, Some(Edge::Rise)),
        "three_state_enable_fall" => (ArcRole::TristateEnable, None, Some(Edge::Fall)),
        "three_state_disable" => (ArcRole::TristateDisable, None, None),
        "three_state_disable_rise" => (ArcRole::TristateDisable, None, Some(Edge::Rise)),
        "three_state_disable_fall" => (ArcRole::TristateDisable, None, Some(Edge::Fall)),
        "setup_rising" => (ArcRole::Setup, Some(Edge::Rise), None),
        "setup_falling" => (ArcRole::Setup, Some(Edge::Fall), None),
        "hold_rising" => (ArcRole::Hold, Some(Edge::Rise), None),
        "hold_falling" => (ArcRole::Hold, Some(Edge::Fall), None),
        "recovery_rising" => (ArcRole::Recovery, Some(Edge::Rise), None),
        "recovery_falling" => (ArcRole::Recovery, Some(Edge::Fall), None),
        "removal_rising" => (ArcRole::Removal, Some(Edge::Rise), None),
        "removal_falling" => (ArcRole::Removal, Some(Edge::Fall), None),
        "min_pulse_width"
        | "minimum_period"
        | "retaining_time"
        | "nochange_high_high"
        | "nochange_high_low"
        | "nochange_low_high"
        | "nochange_low_low"
        | "non_seq_hold_falling"
        | "non_seq_hold_rising"
        | "non_seq_setup_falling"
        | "non_seq_setup_rising"
        | "skew_falling"
        | "skew_rising" => (ArcRole::Check, None, None),
        _ => return None,
    };
    Some(ArcKind {
        role,
        from_edge,
        to_edge,
    })
}

#[cfg(test)]
mod tests;
