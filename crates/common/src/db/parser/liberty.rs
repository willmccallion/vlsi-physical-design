//! Liberty parser.
//!
//! Reads a library's units, thresholds, defaults, table templates and cells
//! into a [`Liberty`], scaling every time, capacitance and resistance to SI
//! units. Buses expand to one pin per bit. Power, leakage and wire-load data
//! are skipped. Bundles are refused, since a model without them would be
//! wrong.

use crate::db::liberty::{
    ArcRole, Axis, AxisVariable, ClockGatePin, Defaults, Edge, EdgeValues, FuncExpr, LibCell,
    LibPort, Liberty, LibraryUnits, PortDirection, Sequential, SequentialKind, Statetable, Table,
    TimingArc, TimingArcSet, TimingSense,
};
use crate::db::parser::lexer::{ParseError, read_file};
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
        .ok_or_else(|| lexer.error(1, "no library group".into()))?;
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

/// Scale factors from the file's units to SI.
#[derive(Clone, Copy, Debug)]
struct Units {
    time: f64,
    capacitance: f64,
}

#[derive(Clone, Debug)]
struct Template {
    variables: Vec<AxisVariable>,
    indices: Vec<Option<Vec<f64>>>,
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
    units: Units,
    defaults: Defaults,
    templates: HashMap<String, Template>,
    bus_types: HashMap<String, BusType>,
}

impl Interpreter {
    fn new(path: &Path) -> Self {
        Self {
            path: path.to_path_buf(),
            units: Units {
                time: 1e-9,
                capacitance: 1e-12,
            },
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
        lib.units = LibraryUnits {
            time: self.units.time,
            capacitance: self.units.capacitance,
        };
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
                .map_err(|name| self.error(line, format!("cell '{name}' is defined twice")))?;
        }
        Ok(lib)
    }

    fn read_units(&self, group: &Group<'_>) -> Result<Units, ParseError> {
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
            units.capacitance = scale * factor;
        }
        Ok(units)
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
        let mut variables = Vec::new();
        let mut indices = Vec::new();
        for k in 1..=3 {
            let Some(var) = group.attr_text(&format!("variable_{k}")) else {
                break;
            };
            let variable = axis_variable(var);
            variables.push(variable);
            let index = group
                .attr(&format!("index_{k}"))
                .map(|a| self.axis_values(a, variable))
                .transpose()?;
            indices.push(index);
        }
        Ok((name, Template { variables, indices }))
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
            | AxisVariable::ConstrainedPinTransition => self.units.time,
            AxisVariable::OutputCapacitance | AxisVariable::RelatedOutputCapacitance => {
                self.units.capacitance
            }
            AxisVariable::Other => 1.0,
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
        let mut cell = LibCell {
            name,
            area: self.number_attr(group, "area")?.unwrap_or(0.0),
            dont_use: Self::bool_attr(group, "dont_use"),
            dont_touch: Self::bool_attr(group, "dont_touch"),
            is_clock_gate: group.attr("clock_gating_integrated_cell").is_some(),
            footprint: group.attr_text("cell_footprint").map(str::to_string),
            ports: Vec::new(),
            pg_pins: group
                .groups_named("pg_pin")
                .filter_map(|g| g.args.first())
                .map(|v| v.0.to_string())
                .collect(),
            arc_sets: Vec::new(),
            sequentials: Vec::new(),
            statetable: group
                .groups_named("statetable")
                .next()
                .map(|g| self.statetable(g))
                .transpose()?,
        };
        for pin in group.groups_named("pin") {
            for arg in &pin.args {
                cell.ports.push(self.port(pin, arg.0)?);
            }
        }
        let mut bus_bits: Vec<(&Group<'_>, Vec<String>)> = Vec::new();
        for bus in group.groups_named("bus") {
            let bits = self.bus_bits(bus, &cell.name)?;
            for bit in &bits {
                let overriding = bus
                    .groups_named("pin")
                    .find(|g| g.args.iter().any(|a| a.0 == bit));
                let mut port = self.port(bus, bit)?;
                if let Some(g) = overriding {
                    let own = self.port(g, bit)?;
                    port.capacitance = own.capacitance;
                    port.max_capacitance = own.max_capacitance.or(port.max_capacitance);
                    port.max_transition = own.max_transition.or(port.max_transition);
                    port.function = own.function.or(port.function);
                }
                cell.ports.push(port);
            }
            bus_bits.push((bus, bits));
        }
        for pin in group.groups_named("pin") {
            for arg in &pin.args {
                let to = cell
                    .port(arg.0)
                    .ok_or_else(|| self.error(pin.line, format!("pin '{}' vanished", arg.0)))?;
                for timing in pin.groups_named("timing") {
                    self.timing(&mut cell, to, timing)?;
                }
            }
        }
        for (bus, bits) in bus_bits {
            for bit in &bits {
                let to = cell
                    .port(bit)
                    .ok_or_else(|| self.error(bus.line, format!("bus bit '{bit}' vanished")))?;
                for timing in bus.groups_named("timing") {
                    self.timing(&mut cell, to, timing)?;
                }
                for pin in bus
                    .groups_named("pin")
                    .filter(|g| g.args.iter().any(|a| a.0 == bit))
                {
                    for timing in pin.groups_named("timing") {
                        self.timing(&mut cell, to, timing)?;
                    }
                }
            }
        }
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

    /// The bit pins a `bus` group declares, named `bus[bit]`.
    fn bus_bits(&self, bus: &Group<'_>, cell: &str) -> Result<Vec<String>, ParseError> {
        let name = bus
            .args
            .first()
            .ok_or_else(|| self.error(bus.line, format!("cell '{cell}' has a bus without a name")))?
            .0;
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

    fn port(&self, group: &Group<'_>, name: &str) -> Result<LibPort, ParseError> {
        let direction = match group.attr_text("direction") {
            Some("input") => PortDirection::Input,
            Some("output") => PortDirection::Output,
            Some("inout") => PortDirection::Inout,
            Some("internal") => PortDirection::Internal,
            Some(other) => {
                return Err(self.error(group.line, format!("pin '{name}' has direction '{other}'")));
            }
            None => return Err(self.error(group.line, format!("pin '{name}' has no direction"))),
        };
        let cap = self.units.capacitance;
        let default_cap = match direction {
            PortDirection::Input | PortDirection::Inout => self.defaults.input_pin_cap,
            PortDirection::Output | PortDirection::Internal => self.defaults.output_pin_cap,
        };
        let both = self.number_attr(group, "capacitance")?.map(|v| v * cap);
        let rise = self
            .number_attr(group, "rise_capacitance")?
            .map(|v| v * cap)
            .or(both)
            .unwrap_or(default_cap);
        let fall = self
            .number_attr(group, "fall_capacitance")?
            .map(|v| v * cap)
            .or(both)
            .unwrap_or(default_cap);
        let clock_gate = if Self::bool_attr(group, "clock_gate_clock_pin") {
            ClockGatePin::Clock
        } else if Self::bool_attr(group, "clock_gate_enable_pin") {
            ClockGatePin::Enable
        } else if Self::bool_attr(group, "clock_gate_test_pin") {
            ClockGatePin::Test
        } else if Self::bool_attr(group, "clock_gate_out_pin") {
            ClockGatePin::Out
        } else {
            ClockGatePin::None
        };
        Ok(LibPort {
            name: name.to_string(),
            direction,
            capacitance: EdgeValues { rise, fall },
            max_capacitance: self.number_attr(group, "max_capacitance")?.map(|v| v * cap),
            max_transition: self
                .number_attr(group, "max_transition")?
                .map(|v| v * self.units.time),
            fanout_load: self.number_attr(group, "fanout_load")?,
            function: self.function_attr(group, "function")?,
            three_state: self.function_attr(group, "three_state")?,
            is_clock: Self::bool_attr(group, "clock"),
            clock_gate,
        })
    }

    fn timing(&self, cell: &mut LibCell, to: usize, group: &Group<'_>) -> Result<(), ParseError> {
        let timing_type = group.attr_text("timing_type").unwrap_or("combinational");
        let (role, from_edge, to_edge) = arc_role(timing_type);
        let sense = match group.attr_text("timing_sense") {
            Some("positive_unate") => TimingSense::PositiveUnate,
            Some("negative_unate") => TimingSense::NegativeUnate,
            _ => TimingSense::NonUnate,
        };
        let when = self.function_attr(group, "when")?;
        let table_names: &[(&str, Edge, bool)] = if role.is_delay() {
            &[
                ("cell_rise", Edge::Rise, true),
                ("cell_fall", Edge::Fall, true),
                ("rise_transition", Edge::Rise, false),
                ("fall_transition", Edge::Fall, false),
            ]
        } else if role.check_side().is_some() {
            &[
                ("rise_constraint", Edge::Rise, true),
                ("fall_constraint", Edge::Fall, true),
            ]
        } else {
            &[]
        };
        let tables = table_names
            .iter()
            .map(|&(name, edge, is_delay)| {
                group
                    .groups_named(name)
                    .next()
                    .map(|g| self.table(g).map(|t| (edge, is_delay, t)))
                    .transpose()
            })
            .collect::<Result<Vec<_>, _>>()?;
        let table_for = |edge: Edge, is_delay: bool| -> Option<Table> {
            tables
                .iter()
                .flatten()
                .find(|(e, d, _)| *e == edge && *d == is_delay)
                .map(|(_, _, t)| t.clone())
        };
        let mut arcs = Vec::new();
        if role.is_delay() {
            let pairs: Vec<(Edge, Edge)> = match (role, from_edge, sense) {
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
                    (Edge::Rise, Edge::Rise),
                    (Edge::Fall, Edge::Rise),
                    (Edge::Rise, Edge::Fall),
                    (Edge::Fall, Edge::Fall),
                ],
            };
            for (from, to_e) in pairs {
                if to_edge.is_some_and(|restricted| restricted != to_e) {
                    continue;
                }
                let delay = table_for(to_e, true);
                let slew = table_for(to_e, false);
                if delay.is_none() && slew.is_none() {
                    continue;
                }
                arcs.push(TimingArc {
                    from_edge: from,
                    to_edge: to_e,
                    delay,
                    slew,
                });
            }
        } else if let (Some(clock_edge), Some(_)) = (from_edge, role.check_side()) {
            for to_e in Edge::BOTH {
                if let Some(constraint) = table_for(to_e, true) {
                    arcs.push(TimingArc {
                        from_edge: clock_edge,
                        to_edge: to_e,
                        delay: Some(constraint),
                        slew: None,
                    });
                }
            }
        }
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
            cell.arc_sets.push(TimingArcSet {
                from,
                to,
                role,
                sense,
                when: when.clone(),
                arcs: arcs.clone(),
            });
        }
        Ok(())
    }

    fn table(&self, group: &Group<'_>) -> Result<Table, ParseError> {
        let template_name = group.args.first().map_or("scalar", |v| v.0);
        let template = if template_name == "scalar" {
            Template {
                variables: Vec::new(),
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
        for (k, variable) in template.variables.iter().enumerate() {
            let values = match group.attr(&format!("index_{}", k + 1)) {
                Some(attr) => self.axis_values(attr, *variable)?,
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
            axes.push(Axis {
                variable: *variable,
                values,
            });
        }
        let values_attr = group
            .attr("values")
            .ok_or_else(|| self.error(group.line, format!("'{}' has no values", group.name)))?;
        let values: Vec<f64> = self
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

fn axis_variable(name: &str) -> AxisVariable {
    match name {
        "input_net_transition" | "input_transition_time" => AxisVariable::InputTransition,
        "total_output_net_capacitance" => AxisVariable::OutputCapacitance,
        "related_out_total_output_net_capacitance" => AxisVariable::RelatedOutputCapacitance,
        "related_pin_transition" => AxisVariable::RelatedPinTransition,
        "constrained_pin_transition" => AxisVariable::ConstrainedPinTransition,
        _ => AxisVariable::Other,
    }
}

/// The role a `timing_type` names, with the input edge it fixes for a clock
/// and the output edge it restricts to, where it does.
fn arc_role(timing_type: &str) -> (ArcRole, Option<Edge>, Option<Edge>) {
    match timing_type {
        "combinational" => (ArcRole::Combinational, None, None),
        "combinational_rise" => (ArcRole::Combinational, None, Some(Edge::Rise)),
        "combinational_fall" => (ArcRole::Combinational, None, Some(Edge::Fall)),
        "rising_edge" => (ArcRole::RegClkToQ, Some(Edge::Rise), None),
        "falling_edge" => (ArcRole::RegClkToQ, Some(Edge::Fall), None),
        "clear" | "preset" => (ArcRole::RegSetClr, None, None),
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
        _ => (ArcRole::Check, None, None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LIB: &str = r#"
/* a comment */
library (tiny) {
  delay_model : table_lookup;
  time_unit : "1ns";
  capacitive_load_unit (1,ff);
  pulling_resistance_unit : "1kohm";
  slew_lower_threshold_pct_rise : 30.00 ;
  slew_upper_threshold_pct_rise : 70.00 ;
  slew_lower_threshold_pct_fall : 30.00 ;
  slew_upper_threshold_pct_fall : 70.00 ;
  default_max_transition : 0.5;
  default_fanout_load : 1.0;
  define(process_corner, operating_conditions, string);
  lu_table_template (T2) {
    variable_1 : input_net_transition;
    variable_2 : total_output_net_capacitance;
    index_1 ("0.1, 0.2");
    index_2 ("1.0, 2.0");
  }
  lu_table_template (C2) {
    variable_1 : related_pin_transition;
    variable_2 : constrained_pin_transition;
    index_1 ("0.1, 0.2");
    index_2 ("0.1, 0.2");
  }
  cell (BUF_X1) {
    area : 0.798;
    pg_pin(VDD) { voltage_name : VDD; pg_type : primary_power; }
    pg_pin(VSS) { voltage_name : VSS; pg_type : primary_ground; }
    pin (A) {
      direction : input;
      capacitance : 0.9;
      rise_capacitance : 0.9;
      fall_capacitance : 0.8;
    }
    pin (Z) {
      direction : output;
      max_capacitance : 60.0;
      function : "A";
      timing () {
        related_pin : "A";
        timing_sense : positive_unate;
        cell_rise(T2) {
          index_1 ("0.1, 0.3");
          values ("0.010, 0.020", \
                  "0.011, 0.021");
        }
        rise_transition(T2) {
          values ("0.001, 0.002", "0.002, 0.003");
        }
        cell_fall(T2) { values ("1,2,3,4"); }
        fall_transition(T2) { values ("1,2,3,4"); }
      }
    }
  }
  cell (DFF_X1) {
    area : 4.5;
    dont_use : true;
    ff ("IQ" , "IQN") {
      next_state : "D";
      clocked_on : "CK";
    }
    pin (D) { direction : input; capacitance : 1.1;
      timing () { related_pin : "CK"; timing_type : setup_rising;
        rise_constraint(C2) { values ("1,2,3,4"); } } }
    pin (CK) { direction : input; clock : true; capacitance : 0.9; }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "CK"; timing_type : rising_edge;
        cell_rise(T2) { values ("1,2,3,4"); } cell_fall(T2) { values ("1,2,3,4"); } } }
  }
}
"#;

    fn tiny() -> Liberty {
        parse_str(LIB, Path::new("tiny.lib")).unwrap()
    }

    #[test]
    fn units_scale_to_si() {
        let lib = tiny();
        let buf = lib.cell(lib.cell_id("BUF_X1").unwrap());
        let a = &buf.ports[buf.port("A").unwrap()];
        assert!((a.capacitance.rise - 0.9e-15).abs() < 1e-30);
        assert!((a.capacitance.fall - 0.8e-15).abs() < 1e-30);
        let z = &buf.ports[buf.port("Z").unwrap()];
        assert!((z.max_capacitance.unwrap() - 60e-15).abs() < 1e-28);
        assert!((lib.defaults.max_transition.unwrap() - 0.5e-9).abs() < 1e-22);
        assert!((lib.thresholds.slew_lower.rise - 0.3).abs() < 1e-12);
    }

    #[test]
    fn tables_take_their_own_indices_over_the_templates() {
        let lib = tiny();
        let buf = lib.cell(lib.cell_id("BUF_X1").unwrap());
        let set = &buf.arc_sets[0];
        assert_eq!(buf.ports[set.from].name, "A");
        assert_eq!(set.role, ArcRole::Combinational);
        assert_eq!(set.arcs.len(), 2);
        let rise = set.arcs.iter().find(|a| a.to_edge == Edge::Rise).unwrap();
        let delay = rise.delay.as_ref().unwrap();
        let close = |got: &[f64], want: &[f64]| {
            got.len() == want.len()
                && got
                    .iter()
                    .zip(want)
                    .all(|(g, w)| (g - w).abs() < 1e-6 * w.abs())
        };
        assert!(close(&delay.axes[0].values, &[0.1e-9, 0.3e-9]));
        assert!(close(&delay.axes[1].values, &[1e-15, 2e-15]));
        assert!((delay.lookup(0.1e-9, 2e-15) - 0.020e-9).abs() < 1e-20);
        assert!(rise.slew.is_some());
    }

    #[test]
    fn sequential_cells_and_checks_parse() {
        let lib = tiny();
        let dff = lib.cell(lib.cell_id("DFF_X1").unwrap());
        assert!(dff.dont_use);
        assert_eq!(dff.sequentials.len(), 1);
        assert_eq!(dff.sequentials[0].output, "IQ");
        assert_eq!(dff.sequentials[0].output_inv.as_deref(), Some("IQN"));
        assert!(dff.ports[dff.port("CK").unwrap()].is_clock);
        let setup = dff
            .arc_sets
            .iter()
            .find(|s| s.role == ArcRole::Setup)
            .unwrap();
        assert_eq!(dff.ports[setup.from].name, "CK");
        assert_eq!(dff.ports[setup.to].name, "D");
        assert_eq!(setup.arcs.len(), 1);
        assert_eq!(setup.arcs[0].from_edge, Edge::Rise);
        assert_eq!(setup.arcs[0].to_edge, Edge::Rise);
        assert!((setup.arcs[0].check_value(0.1e-9, 0.2e-9) - 2e-9).abs() < 1e-20);
        assert!((lib.units.time - 1e-9).abs() < 1e-24);
        assert!((lib.units.capacitance - 1e-15).abs() < 1e-30);
        let q = dff
            .arc_sets
            .iter()
            .find(|s| s.role == ArcRole::RegClkToQ)
            .unwrap();
        assert!(q.arcs.iter().all(|a| a.from_edge == Edge::Rise));
        assert_eq!(q.arcs.len(), 2);
    }

    #[test]
    fn a_statetable_keeps_its_rows_for_comparison() {
        let src = "library (x) { cell (c) { statetable (\"CK E\", \"IQ\") { table : \"L L : - : L ,\\\n   L H : - : H \" ; } } }";
        let lib = parse_str(src, Path::new("x.lib")).unwrap();
        let table = lib.cells[0].statetable.as_ref().unwrap();
        assert_eq!(table.inputs, vec!["CK", "E"]);
        assert_eq!(table.outputs, vec!["IQ"]);
        assert_eq!(table.rows, vec!["L L : - : L", "L H : - : H"]);
    }

    #[test]
    fn buses_expand_to_bit_pins_with_the_bus_attributes_and_arcs() {
        let src = r#"
library (ram) {
  time_unit : "1ns";
  capacitive_load_unit (1,ff);
  type (DATA) { base_type : array ; data_type : bit ; bit_width : 2; bit_from : 1; bit_to : 0 ; }
  lu_table_template (slew_t) { variable_1 : total_output_net_capacitance; index_1 ("5.0, 500.0"); }
  cell(mem) {
    area : 10;
    pin(clk) { direction : input; capacitance : 25.0; clock : true;
      internal_power() { rise_power(scalar) { values ("1.0") } } }
    bus(rd_out) {
      bus_type : DATA;
      direction : output;
      max_capacitance : 500.0;
      timing() {
        related_pin : "clk" ;
        timing_type : rising_edge;
        cell_rise(scalar) { values ("0.204"); }
        cell_fall(scalar) { values ("0.204"); }
        rise_transition(slew_t) { index_1 ("5.0, 500.0"); values ("0.009, 0.227") }
        fall_transition(slew_t) { values ("0.009, 0.227") }
      }
    }
    bus(addr_in) {
      bus_type : DATA;
      direction : input;
      capacitance : 5.0;
      timing() { related_pin : clk; timing_type : setup_rising ; rise_constraint(scalar) { values ("0.05"); } }
    }
  }
}
"#;
        let lib = parse_str(src, Path::new("ram.lib")).unwrap();
        let mem = lib.cell(lib.cell_id("mem").unwrap());
        let names: Vec<&str> = mem.ports.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(
            names,
            vec!["clk", "rd_out[1]", "rd_out[0]", "addr_in[1]", "addr_in[0]"]
        );
        let out = &mem.ports[mem.port("rd_out[0]").unwrap()];
        assert_eq!(out.direction, PortDirection::Output);
        assert!((out.max_capacitance.unwrap() - 500e-15).abs() < 1e-27);
        let addr = &mem.ports[mem.port("addr_in[1]").unwrap()];
        assert!((addr.capacitance.rise - 5e-15).abs() < 1e-28);
        let clk_to_q: Vec<&TimingArcSet> = mem
            .arc_sets
            .iter()
            .filter(|s| s.role == ArcRole::RegClkToQ)
            .collect();
        assert_eq!(clk_to_q.len(), 2);
        let arc = &clk_to_q[0].arcs[0];
        assert!((arc.gate_delay(0.0, 500e-15).1 - 0.227e-9).abs() < 1e-14);
        assert!(mem.arc_sets.iter().any(|s| s.role == ArcRole::Setup));
    }

    #[test]
    fn a_bundle_group_is_refused() {
        let src = "library (x) { cell (c) { bundle (b) { } } }";
        let err = parse_str(src, Path::new("x.lib")).unwrap_err();
        assert!(err.to_string().contains("bundle"));
    }
}
