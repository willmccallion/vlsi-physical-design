//! Reads SDC constraints.
//!
//! SDC is Tcl, so this is a small Tcl interpreter: it splits commands and
//! words, substitutes variables and bracketed commands, evaluates `expr`,
//! and runs the constraint commands against the design, resolving port and
//! clock names as it goes. Commands outside the subset below, and options
//! a command does not take or support, are errors rather than silently
//! dropped. As `OpenSTA` reads them, times are in the library's time unit
//! whatever `set_units` says.

use super::lexer::{ParseError, read_file};
use crate::db::design::NetlistDB;
use crate::db::indices::{CellId, IoPinId};
use crate::db::liberty::{Edge, Liberty, LibraryUnits, MinMax, SideValues};
use crate::db::library::{PinDirection, SignalUse};
use crate::db::sdc::{
    Clock, ClockEdge, ClockId, FalsePath, MultiCyclePath, PathSpec, PortDelay, RiseFallMinMax, Sdc,
};
use crate::units::Seconds;
use std::collections::HashMap;
use std::path::Path;

/// Reads the constraints in `path` for `db`, with times in the units of
/// `liberty`.
pub fn parse_file(path: &Path, db: &NetlistDB, liberty: &Liberty) -> Result<Sdc, ParseError> {
    let text = read_file(path)?;
    parse_str(&text, path, db, liberty)
}

/// Reads the constraints in `text`, naming `path` in errors.
pub fn parse_str(
    text: &str,
    path: &Path,
    db: &NetlistDB,
    liberty: &Liberty,
) -> Result<Sdc, ParseError> {
    let mut interpreter = Interpreter {
        db,
        units: liberty.units,
        path,
        variables: HashMap::new(),
        sdc: Sdc::default(),
    };
    let _ = interpreter.eval_script(text, 1)?;
    Ok(interpreter.sdc)
}

/// A reference a command returned: `OpenSTA` hands objects between
/// commands, and a plain name is ambiguous where a port and a clock share
/// one.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Object {
    Clock(String),
    Port(String),
    Cell(String),
}

impl Object {
    const CLOCK_PREFIX: &'static str = "_sta_clock:";
    const PORT_PREFIX: &'static str = "_sta_port:";
    const CELL_PREFIX: &'static str = "_sta_cell:";

    fn encode(&self) -> String {
        match self {
            Self::Clock(name) => format!("{}{name}", Self::CLOCK_PREFIX),
            Self::Port(name) => format!("{}{name}", Self::PORT_PREFIX),
            Self::Cell(name) => format!("{}{name}", Self::CELL_PREFIX),
        }
    }

    fn decode(word: &str) -> Option<Self> {
        word.strip_prefix(Self::CLOCK_PREFIX)
            .map(|n| Self::Clock(n.to_string()))
            .or_else(|| {
                word.strip_prefix(Self::PORT_PREFIX)
                    .map(|n| Self::Port(n.to_string()))
            })
            .or_else(|| {
                word.strip_prefix(Self::CELL_PREFIX)
                    .map(|n| Self::Cell(n.to_string()))
            })
    }
}

/// A command's arguments taken apart: `-key value` pairs, `-flag`s and
/// the positional words, as `OpenSTA`'s `parse_key_args`.
#[derive(Debug, Default)]
struct Args {
    keys: HashMap<String, String>,
    flags: Vec<String>,
    positional: Vec<String>,
}

impl Args {
    fn flag(&self, name: &str) -> bool {
        self.flags.iter().any(|f| f == name)
    }

    fn key(&self, name: &str) -> Option<&str> {
        self.keys.get(name).map(String::as_str)
    }
}

/// A number as Tcl's `expr` holds it: an integer stays one until a
/// fraction enters, and integer division rounds down.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Number {
    Int(i64),
    Double(f64),
}

impl Number {
    fn parse(text: &str) -> Option<Self> {
        let trimmed = text.trim();
        trimmed
            .parse::<i64>()
            .map(Self::Int)
            .ok()
            .or_else(|| trimmed.parse::<f64>().ok().map(Self::Double))
    }

    const fn as_f64(self) -> f64 {
        match self {
            Self::Int(i) => i as f64,
            Self::Double(d) => d,
        }
    }

    fn format(self) -> String {
        match self {
            Self::Int(i) => i.to_string(),
            Self::Double(d) => format!("{d:?}"),
        }
    }

    fn apply(self, op: char, rhs: Self) -> Option<Self> {
        if let (Self::Int(a), Self::Int(b)) = (self, rhs) {
            return match op {
                '+' => a.checked_add(b).map(Self::Int),
                '-' => a.checked_sub(b).map(Self::Int),
                '*' => a.checked_mul(b).map(Self::Int),
                '/' => floor_div_rem(a, b).map(|(quotient, _)| Self::Int(quotient)),
                '%' => floor_div_rem(a, b).map(|(_, remainder)| Self::Int(remainder)),
                _ => None,
            };
        }
        let (a, b) = (self.as_f64(), rhs.as_f64());
        match op {
            '+' => Some(Self::Double(a + b)),
            '-' => Some(Self::Double(a - b)),
            '*' => Some(Self::Double(a * b)),
            '/' if b != 0.0 => Some(Self::Double(a / b)),
            _ => None,
        }
    }
}

/// `a / b` rounded down, and the remainder, which takes `b`'s sign, as
/// Tcl's integer `/` and `%` give them; none when `b` is zero or the
/// quotient overflows.
fn floor_div_rem(a: i64, b: i64) -> Option<(i64, i64)> {
    let (a, b, sign) = match b.signum() {
        0 => return None,
        1 => (a, b, 1),
        _ => (a.checked_neg()?, b.checked_neg()?, -1),
    };
    Some((a.div_euclid(b), sign * a.rem_euclid(b)))
}

struct Interpreter<'a> {
    db: &'a NetlistDB,
    units: LibraryUnits,
    path: &'a Path,
    variables: HashMap<String, String>,
    sdc: Sdc,
}

impl Interpreter<'_> {
    /// `args` split by the command's `options`.
    fn options(&self, args: &[String], options: &Options, line: usize) -> Result<Args, ParseError> {
        parse_args(args, options).map_err(|message| self.error(line, message))
    }

    fn error(&self, line: usize, message: impl Into<String>) -> ParseError {
        ParseError::Syntax {
            path: self.path.to_path_buf(),
            line,
            message: message.into(),
        }
    }

    /// Runs every command of `script`; the result is the last command's.
    fn eval_script(&mut self, script: &str, first_line: usize) -> Result<String, ParseError> {
        let mut result = String::new();
        for (text, line) in split_commands(script, first_line) {
            let words = split_words(&text).map_err(|m| self.error(line, m))?;
            if words.is_empty() {
                continue;
            }
            let mut args = Vec::with_capacity(words.len());
            for word in &words {
                args.push(self.substitute(word, line)?);
            }
            result = self.run(&args, line)?;
        }
        Ok(result)
    }

    /// The text of one word after substitution.
    fn substitute(&mut self, word: &Word, line: usize) -> Result<String, ParseError> {
        match word {
            Word::Braced(text) => Ok(text.clone()),
            Word::Plain(text) => self.substitute_text(text, line),
        }
    }

    fn substitute_text(&mut self, text: &str, line: usize) -> Result<String, ParseError> {
        let chars: Vec<char> = text.chars().collect();
        let mut out = String::new();
        let mut i = 0;
        while i < chars.len() {
            match chars[i] {
                '\\' if i + 1 < chars.len() => {
                    out.push(unescape(chars[i + 1]));
                    i += 2;
                }
                '$' => {
                    let (name, next) = variable_name(&chars, i + 1);
                    if name.is_empty() {
                        out.push('$');
                        i += 1;
                        continue;
                    }
                    let value = self
                        .variables
                        .get(&name)
                        .ok_or_else(|| self.error(line, format!("no such variable '{name}'")))?;
                    out.push_str(value);
                    i = next;
                }
                '[' => {
                    let end = matching_bracket(&chars, i)
                        .ok_or_else(|| self.error(line, "unbalanced '['"))?;
                    let inner: String = chars[i + 1..end].iter().collect();
                    out.push_str(&self.eval_script(&inner, line)?);
                    i = end + 1;
                }
                c => {
                    out.push(c);
                    i += 1;
                }
            }
        }
        Ok(out)
    }

    fn run(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        let command = args[0].as_str();
        let rest = &args[1..];
        match command {
            "set" => self.set(rest, line),
            "expr" => self.expr(&rest.join(" "), line),
            "list" => Ok(join_list(rest)),
            "llength" => Ok(split_list(rest.first().map_or("", String::as_str))
                .len()
                .to_string()),
            "lindex" => self.lindex(rest, line),
            "puts" | "current_design" | "group_path" | "set_units_checked" => Ok(String::new()),
            "set_units" => self.set_units(rest, line),
            "get_ports" => self.get_ports(rest, line),
            "get_cells" => self.get_cells(rest, line),
            "get_clocks" => self.get_clocks(rest, line),
            "all_clocks" => Ok(join_list(
                &self
                    .sdc
                    .clocks
                    .iter()
                    .map(|c| Object::Clock(c.name.clone()).encode())
                    .collect::<Vec<_>>(),
            )),
            "all_inputs" => self.all_ports(rest, line, PinDirection::Input),
            "all_outputs" => self.all_ports(rest, line, PinDirection::Output),
            "create_clock" => self.create_clock(rest, line),
            "set_input_delay" => self.set_port_delay(rest, line, PinDirection::Input),
            "set_output_delay" => self.set_port_delay(rest, line, PinDirection::Output),
            "set_clock_uncertainty" => self.set_clock_uncertainty(rest, line),
            "set_false_path" => self.set_false_path(rest, line),
            "set_multicycle_path" => self.set_multicycle_path(rest, line),
            "set_clock_groups" => self.set_clock_groups(rest, line),
            "set_timing_derate" => self.set_timing_derate(rest, line),
            "set_max_fanout" | "set_max_transition" | "set_max_capacitance" => {
                self.set_design_limit(command, rest, line)
            }
            "set_load" => self.set_load(rest, line),
            "set_input_transition" => self.set_input_transition(rest, line),
            "set_propagated_clock" => self.set_propagated_clock(rest, line),
            "set_driving_cell"
            | "set_clock_latency"
            | "set_clock_transition"
            | "set_disable_timing"
            | "set_case_analysis"
            | "create_generated_clock"
            | "set_max_delay"
            | "set_min_delay"
            | "set_clock_sense"
            | "set_ideal_network"
            | "set_data_check"
            | "set_min_pulse_width"
            | "set_max_area"
            | "set_wire_load_model"
            | "set_operating_conditions" => {
                Err(self.error(line, format!("'{command}' is not supported")))
            }
            _ => Err(self.error(line, format!("unknown command '{command}'"))),
        }
    }

    fn set(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        match args {
            [name] => self
                .variables
                .get(name)
                .cloned()
                .ok_or_else(|| self.error(line, format!("no such variable '{name}'"))),
            [name, value] => {
                let _ = self.variables.insert(name.clone(), value.clone());
                Ok(value.clone())
            }
            _ => Err(self.error(line, "set wants a name and at most one value")),
        }
    }

    fn lindex(&self, args: &[String], line: usize) -> Result<String, ParseError> {
        let [list, index] = args else {
            return Err(self.error(line, "lindex wants a list and an index"));
        };
        let items = split_list(list);
        let i: usize = match index.as_str() {
            "end" => items.len().saturating_sub(1),
            text => text
                .parse()
                .map_err(|_| self.error(line, format!("lindex index '{text}' is not a number")))?,
        };
        Ok(items.get(i).cloned().unwrap_or_default())
    }

    /// Tcl's `expr` over `+`, `-`, `*`, `/`, `%`, parentheses and unary
    /// signs.
    fn expr(&self, text: &str, line: usize) -> Result<String, ParseError> {
        let tokens = tokenize_expr(text).map_err(|m| self.error(line, m))?;
        let mut parser = ExprParser { tokens, at: 0 };
        let value = parser.sum().map_err(|m| self.error(line, m))?;
        if parser.at != parser.tokens.len() {
            return Err(self.error(line, format!("cannot evaluate expr '{text}'")));
        }
        Ok(value.format())
    }

    /// `set_units` only checks against the library's units, as `OpenSTA`
    /// does: values keep the library's scale either way.
    fn set_units(&self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &SET_UNITS, line)?;
        let checks: [(&str, f64, &str); 2] = [
            ("-time", self.units.time.get(), "s"),
            ("-capacitance", self.units.capacitance.get(), "f"),
        ];
        for (key, library_scale, suffix) in checks {
            if let Some(value) = parsed.key(key) {
                let scale = unit_scale(value, suffix)
                    .ok_or_else(|| self.error(line, format!("unknown unit '{value}'")))?;
                if (scale - library_scale).abs() > 1e-6 * library_scale {
                    log::warn!(
                        "{}:{line}: set_units {key} {value} does not match the library's \
                         scale of {library_scale:e}; values keep the library's units",
                        self.path.display()
                    );
                }
            }
        }
        Ok(String::new())
    }

    fn get_ports(&self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &GET_PORTS, line)?;
        let mut names = Vec::new();
        for pattern in parsed.positional.iter().flat_map(|p| split_list(p)) {
            let matched = self.ports_matching(&pattern);
            if matched.is_empty() && !parsed.flag("-quiet") {
                return Err(self.error(line, format!("no port matches '{pattern}'")));
            }
            names.extend(matched);
        }
        Ok(join_list(
            &names
                .into_iter()
                .map(|n| Object::Port(n).encode())
                .collect::<Vec<_>>(),
        ))
    }

    /// `get_cells`: the instances named, exactly or by glob; hierarchy is
    /// flat here, so `-hierarchical` changes nothing. A pattern matching
    /// nothing names nothing, with a warning unless `-quiet`, as `OpenSTA`
    /// treats it: constraints often name instances synthesis renamed.
    fn get_cells(&self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &GET_CELLS, line)?;
        let mut names = Vec::new();
        for pattern in parsed.positional.iter().flat_map(|p| split_list(p)) {
            let matched = self.cells_matching(&pattern);
            if matched.is_empty() && !parsed.flag("-quiet") {
                log::warn!(
                    "{}:{line}: no instance matches '{pattern}'",
                    self.path.display()
                );
            }
            names.extend(matched);
        }
        Ok(join_list(
            &names
                .into_iter()
                .map(|n| Object::Cell(n).encode())
                .collect::<Vec<_>>(),
        ))
    }

    fn cells_matching(&self, pattern: &str) -> Vec<String> {
        if self.db.cell_id(pattern).is_some() {
            return vec![pattern.to_string()];
        }
        self.db
            .cells
            .iter()
            .filter(|c| glob_matches(pattern, &c.name))
            .map(|c| c.name.clone())
            .collect()
    }

    /// The instances a word names: `get_cells` results, or names to match.
    fn cells_of(&self, word: &str, line: usize) -> Result<Vec<CellId>, ParseError> {
        let mut cells = Vec::new();
        for item in split_list(word) {
            let names = match Object::decode(&item) {
                Some(Object::Cell(name)) => vec![name],
                Some(Object::Port(name) | Object::Clock(name)) => {
                    return Err(self.error(line, format!("'{name}' is not an instance")));
                }
                None => {
                    let matched = self.cells_matching(&item);
                    if matched.is_empty() {
                        return Err(self.error(line, format!("no instance matches '{item}'")));
                    }
                    matched
                }
            };
            for name in names {
                let id = self
                    .db
                    .cell_id(&name)
                    .ok_or_else(|| self.error(line, format!("no instance '{name}'")))?;
                cells.push(id);
            }
        }
        Ok(cells)
    }

    fn ports_matching(&self, pattern: &str) -> Vec<String> {
        if self.db.io_pin_id(pattern).is_some() {
            return vec![pattern.to_string()];
        }
        self.db
            .io_pins
            .iter()
            .filter(|p| glob_matches(pattern, &p.name))
            .map(|p| p.name.clone())
            .collect()
    }

    fn get_clocks(&self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &GET_CLOCKS, line)?;
        let mut names = Vec::new();
        for pattern in parsed.positional.iter().flat_map(|p| split_list(p)) {
            let matched: Vec<String> = self
                .sdc
                .clocks
                .iter()
                .filter(|c| c.name == pattern || glob_matches(&pattern, &c.name))
                .map(|c| c.name.clone())
                .collect();
            if matched.is_empty() && !parsed.flag("-quiet") {
                return Err(self.error(line, format!("no clock matches '{pattern}'")));
            }
            names.extend(matched);
        }
        Ok(join_list(
            &names
                .into_iter()
                .map(|n| Object::Clock(n).encode())
                .collect::<Vec<_>>(),
        ))
    }

    fn all_ports(
        &self,
        args: &[String],
        line: usize,
        direction: PinDirection,
    ) -> Result<String, ParseError> {
        let parsed = self.options(
            args,
            match direction {
                PinDirection::Input => &ALL_INPUTS,
                _ => &ALL_OUTPUTS,
            },
            line,
        )?;
        let no_clocks = parsed.flag("-no_clocks");
        if !parsed.positional.is_empty() {
            return Err(self.error(line, "all_inputs and all_outputs take no arguments"));
        }
        let names: Vec<String> = self
            .db
            .io_pins
            .iter()
            .enumerate()
            .filter(|(i, p)| {
                let wanted = match direction {
                    PinDirection::Input => {
                        matches!(p.direction, PinDirection::Input | PinDirection::Inout)
                    }
                    _ => matches!(p.direction, PinDirection::Output | PinDirection::Inout),
                };
                wanted
                    && !matches!(p.signal_use, SignalUse::Power | SignalUse::Ground)
                    && !(no_clocks && self.sdc.clock_at(IoPinId::new(*i)).is_some())
            })
            .map(|(_, p)| Object::Port(p.name.clone()).encode())
            .collect();
        Ok(join_list(&names))
    }

    /// The ports a word names: objects from `get_ports`, or names and
    /// patterns resolved as `get_ports` would.
    fn ports_of(&self, word: &str, line: usize) -> Result<Vec<IoPinId>, ParseError> {
        let mut ports = Vec::new();
        for item in split_list(word) {
            let names = match Object::decode(&item) {
                Some(Object::Port(name)) => vec![name],
                Some(Object::Clock(name) | Object::Cell(name)) => {
                    return Err(self.error(line, format!("'{name}' is not a port")));
                }
                None => {
                    let matched = self.ports_matching(&item);
                    if matched.is_empty() {
                        return Err(self.error(line, format!("no port matches '{item}'")));
                    }
                    matched
                }
            };
            for name in names {
                let id = self
                    .db
                    .io_pin_id(&name)
                    .ok_or_else(|| self.error(line, format!("no port '{name}'")))?;
                ports.push(id);
            }
        }
        Ok(ports)
    }

    /// The clocks a word names: objects from `get_clocks`, or names.
    fn clocks_of(&self, word: &str, line: usize) -> Result<Vec<ClockId>, ParseError> {
        let mut clocks = Vec::new();
        for item in split_list(word) {
            let name = match Object::decode(&item) {
                Some(Object::Clock(name)) => name,
                Some(Object::Port(name) | Object::Cell(name)) => {
                    return Err(self.error(line, format!("'{name}' is not a clock")));
                }
                None => item,
            };
            let id = self
                .sdc
                .clock_id(&name)
                .ok_or_else(|| self.error(line, format!("no clock '{name}'")))?;
            clocks.push(id);
        }
        Ok(clocks)
    }

    /// The clocks and ports an exception's `-from` or `-to` names; a bare
    /// name is a clock if one has it, else a port.
    fn path_ends(
        &self,
        word: &str,
        line: usize,
    ) -> Result<(Vec<ClockId>, Vec<IoPinId>), ParseError> {
        let mut clocks = Vec::new();
        let mut ports = Vec::new();
        for item in split_list(word) {
            match Object::decode(&item) {
                Some(Object::Clock(name)) => clocks.extend(self.clocks_of(&name, line)?),
                Some(Object::Port(name)) => ports.extend(self.ports_of(&name, line)?),
                Some(Object::Cell(name)) => {
                    return Err(self.error(
                        line,
                        format!(
                            "'{name}' is an instance; paths through instances are not supported"
                        ),
                    ));
                }
                None => match self.sdc.clock_id(&item) {
                    Some(id) => clocks.push(id),
                    None => ports.extend(self.ports_of(&item, line)?),
                },
            }
        }
        Ok((clocks, ports))
    }

    fn time(&self, text: &str, line: usize, what: &str) -> Result<Seconds, ParseError> {
        let value: f64 = text
            .trim()
            .parse()
            .map_err(|_| self.error(line, format!("{what} '{text}' is not a number")))?;
        Ok(value * self.units.time)
    }

    fn create_clock(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &CREATE_CLOCK, line)?;
        let sources = match parsed.positional.as_slice() {
            [] => Vec::new(),
            [ports] => self.ports_of(ports, line)?,
            _ => return Err(self.error(line, "create_clock takes one port list")),
        };
        let period_text = parsed
            .key("-period")
            .ok_or_else(|| self.error(line, "create_clock without -period"))?;
        let period = self.time(period_text, line, "-period")?;
        if period <= Seconds::ZERO {
            return Err(self.error(line, "create_clock -period must be positive"));
        }
        let name = match parsed.key("-name") {
            Some(name) => name.to_string(),
            None => self.clock_name_of(&sources, line)?,
        };
        let (rise, fall) = match parsed.key("-waveform") {
            Some(text) => self.waveform(text, period, line)?,
            None => (Seconds::ZERO, period / 2.0),
        };
        let clock = Clock {
            name,
            period,
            rise,
            fall,
            sources,
            uncertainty: SideValues::both(None),
            propagated: false,
        };
        self.define_clock(clock, parsed.flag("-add"));
        Ok(String::new())
    }

    /// The name of a clock `create_clock` leaves unnamed: its first port's.
    fn clock_name_of(&self, sources: &[IoPinId], line: usize) -> Result<String, ParseError> {
        let first = sources
            .first()
            .ok_or_else(|| self.error(line, "create_clock needs -name or a port"))?;
        Ok(self.db.io_pins[*first].name.clone())
    }

    /// The rise and fall times `-waveform` gives as `text`, which must rise
    /// then fall within two periods of `period`.
    fn waveform(
        &self,
        text: &str,
        period: Seconds,
        line: usize,
    ) -> Result<(Seconds, Seconds), ParseError> {
        let edges = split_list(text);
        let [r, f] = edges.as_slice() else {
            return Err(self.error(line, "-waveform wants two edge times"));
        };
        let rise = self.time(r, line, "-waveform edge")?;
        let fall = self.time(f, line, "-waveform edge")?;
        if fall < rise || fall > 2.0 * period {
            return Err(self.error(
                line,
                "-waveform edges must rise then fall within two periods",
            ));
        }
        Ok((rise, fall))
    }

    /// Defines `clock`: in place of the clock of its name, unless `add`;
    /// else as a new clock, which takes its ports from the others unless
    /// `add`.
    fn define_clock(&mut self, clock: Clock, add: bool) {
        if !add && let Some(existing) = self.sdc.clock_id(&clock.name) {
            self.sdc.clocks[existing] = clock;
            return;
        }
        if !add {
            for other in &mut self.sdc.clocks {
                other.sources.retain(|s| !clock.sources.contains(s));
            }
        }
        let _ = self.sdc.clocks.push(clock);
    }

    fn set_port_delay(
        &mut self,
        args: &[String],
        line: usize,
        direction: PinDirection,
    ) -> Result<String, ParseError> {
        let parsed = self.options(args, &PORT_DELAY, line)?;
        let [delay_text, ports_text] = parsed.positional.as_slice() else {
            return Err(self.error(line, "a port delay wants a delay and a port list"));
        };
        let delay = self.time(delay_text, line, "delay")?;
        let clock_name = parsed
            .key("-clock")
            .ok_or_else(|| self.error(line, "a port delay without -clock is not supported"))?;
        let clocks = self.clocks_of(clock_name, line)?;
        let [clock] = clocks.as_slice() else {
            return Err(self.error(line, "-clock wants exactly one clock"));
        };
        let reference = ClockEdge {
            clock: *clock,
            edge: if parsed.flag("-clock_fall") {
                Edge::Fall
            } else {
                Edge::Rise
            },
        };
        let spec = PortDelaySpec {
            reference,
            edges: edges_of(&parsed),
            sides: sides_of(&parsed),
            delay,
            add: parsed.flag("-add_delay"),
        };
        for port in self.ports_of(ports_text, line)? {
            if self.port_takes_delay(port, direction, *clock, line) {
                self.record_port_delay(port, direction, &spec);
            }
        }
        Ok(String::new())
    }

    /// Whether a port delay in `direction` relative to `clock` may go on
    /// `port`, warning where it may not.
    fn port_takes_delay(
        &self,
        port: IoPinId,
        direction: PinDirection,
        clock: ClockId,
        line: usize,
    ) -> bool {
        let pin = &self.db.io_pins[port];
        let (command, allowed) = match direction {
            PinDirection::Input => (
                "set_input_delay",
                matches!(pin.direction, PinDirection::Input | PinDirection::Inout),
            ),
            _ => (
                "set_output_delay",
                matches!(pin.direction, PinDirection::Output | PinDirection::Inout),
            ),
        };
        if !allowed {
            log::warn!(
                "{}:{line}: {command} not allowed on port '{}'",
                self.path.display(),
                pin.name
            );
            return false;
        }
        if self.sdc.clock(clock).sources.contains(&port) {
            log::warn!(
                "{}:{line}: {command} relative to a clock defined on the same port '{}' \
                 not allowed",
                self.path.display(),
                pin.name
            );
            return false;
        }
        true
    }

    /// Sets or adds `spec` among `port`'s delays in `direction`; set alone,
    /// it clears the delays relative to other clock edges.
    fn record_port_delay(&mut self, port: IoPinId, direction: PinDirection, spec: &PortDelaySpec) {
        let delays = match direction {
            PinDirection::Input => &mut self.sdc.input_delays,
            _ => &mut self.sdc.output_delays,
        };
        let list = delays.entry(port).or_default();
        if !spec.add {
            list.retain(|d| d.reference == spec.reference);
        }
        let index = if let Some(index) = list.iter().position(|d| d.reference == spec.reference) {
            index
        } else {
            list.push(PortDelay {
                reference: spec.reference,
                delays: RiseFallMinMax::default(),
            });
            list.len() - 1
        };
        let entry = &mut list[index];
        if spec.add {
            entry.delays.merge(&spec.edges, &spec.sides, spec.delay);
        } else {
            entry.delays.set(&spec.edges, &spec.sides, spec.delay);
        }
    }

    /// `set_propagated_clock` on clocks; on ports or pins, which `OpenSTA`
    /// also takes, it is not supported.
    fn set_propagated_clock(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &SET_PROPAGATED_CLOCK, line)?;
        let [objects] = parsed.positional.as_slice() else {
            return Err(self.error(line, "set_propagated_clock wants one object list"));
        };
        for clock in self.clocks_of(objects, line)? {
            self.sdc.clocks[clock].propagated = true;
        }
        Ok(String::new())
    }

    fn set_clock_uncertainty(
        &mut self,
        args: &[String],
        line: usize,
    ) -> Result<String, ParseError> {
        let parsed = self.options(args, &SET_CLOCK_UNCERTAINTY, line)?;
        let [value_text, clocks_text] = parsed.positional.as_slice() else {
            return Err(self.error(line, "set_clock_uncertainty wants a value and a clock list"));
        };
        let value = self.time(value_text, line, "uncertainty")?;
        let sides = sides_of_setup_hold(&parsed);
        for clock in self.clocks_of(clocks_text, line)? {
            for side in &sides {
                self.sdc.clocks[clock].uncertainty[*side] = Some(value);
            }
        }
        Ok(String::new())
    }

    fn path_spec(&self, parsed: &Args, line: usize) -> Result<PathSpec, ParseError> {
        let mut spec = PathSpec::default();
        if let Some(from) = parsed.key("-from") {
            (spec.from_clocks, spec.from_ports) = self.path_ends(from, line)?;
        }
        if let Some(to) = parsed.key("-to") {
            (spec.to_clocks, spec.to_ports) = self.path_ends(to, line)?;
        }
        Ok(spec)
    }

    fn set_false_path(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &SET_FALSE_PATH, line)?;
        let paths = self.path_spec(&parsed, line)?;
        if paths == PathSpec::default() {
            return Err(self.error(line, "set_false_path wants -from or -to"));
        }
        self.sdc.false_paths.push(FalsePath {
            side: setup_hold_side(&parsed),
            paths,
        });
        Ok(String::new())
    }

    fn set_multicycle_path(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &SET_MULTICYCLE_PATH, line)?;
        let [multiplier_text] = parsed.positional.as_slice() else {
            return Err(self.error(line, "set_multicycle_path wants one path multiplier"));
        };
        let multiplier: i32 = multiplier_text.parse().map_err(|_| {
            self.error(
                line,
                format!("path multiplier '{multiplier_text}' is not an integer"),
            )
        })?;
        let paths = self.path_spec(&parsed, line)?;
        let side = setup_hold_side(&parsed);
        let mut use_end_clock = side != Some(MinMax::Min);
        if parsed.flag("-start") && parsed.flag("-end") {
            return Err(self.error(line, "cannot use -start with -end"));
        }
        if parsed.flag("-start") {
            use_end_clock = false;
        } else if parsed.flag("-end") {
            use_end_clock = true;
        }
        self.sdc.multicycle_paths.push(MultiCyclePath {
            side,
            multiplier,
            use_end_clock,
            paths,
        });
        Ok(String::new())
    }

    /// Exclusive or asynchronous clock groups: paths between groups are
    /// false, as `OpenSTA` times them, unless `-allow_paths` keeps them.
    fn set_clock_groups(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        let kinds = [
            "-asynchronous",
            "-logically_exclusive",
            "-physically_exclusive",
        ];
        let mut groups: Vec<Vec<ClockId>> = Vec::new();
        let mut allow_paths = false;
        let mut kind_count = 0;
        let mut i = 0;
        while i < args.len() {
            match args[i].as_str() {
                "-group" => {
                    let clocks = args
                        .get(i + 1)
                        .ok_or_else(|| self.error(line, "-group wants a clock list"))?;
                    groups.push(self.clocks_of(clocks, line)?);
                    i += 2;
                }
                "-name" | "-comment" => i += 2,
                "-allow_paths" => {
                    allow_paths = true;
                    i += 1;
                }
                k if kinds.contains(&k) => {
                    kind_count += 1;
                    i += 1;
                }
                other => return Err(self.error(line, format!("unexpected '{other}'"))),
            }
        }
        if kind_count != 1 {
            return Err(self.error(
                line,
                "set_clock_groups wants exactly one of -asynchronous, -logically_exclusive \
                 or -physically_exclusive",
            ));
        }
        if allow_paths {
            return Ok(String::new());
        }
        for (a, from) in groups.iter().enumerate() {
            for (b, to) in groups.iter().enumerate() {
                if a != b {
                    self.sdc.false_paths.push(FalsePath {
                        side: None,
                        paths: PathSpec {
                            from_clocks: from.clone(),
                            to_clocks: to.clone(),
                            ..PathSpec::default()
                        },
                    });
                }
            }
        }
        Ok(String::new())
    }

    fn set_timing_derate(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &SET_TIMING_DERATE, line)?;
        let (factor_text, objects) = match parsed.positional.as_slice() {
            [factor] => (factor, None),
            [factor, objects] => (factor, Some(objects)),
            _ => {
                return Err(self.error(
                    line,
                    "set_timing_derate wants a factor and at most one object list",
                ));
            }
        };
        let factor: f64 = factor_text
            .parse()
            .map_err(|_| self.error(line, format!("derate '{factor_text}' is not a number")))?;
        let sides: Vec<MinMax> = match (parsed.flag("-early"), parsed.flag("-late")) {
            (true, false) => vec![MinMax::Min],
            (false, true) => vec![MinMax::Max],
            _ => vec![MinMax::Min, MinMax::Max],
        };
        let (cell, net) = match (parsed.flag("-cell_delay"), parsed.flag("-net_delay")) {
            (false, false) => (true, true),
            picked => picked,
        };
        if let Some(objects) = objects {
            let cells = self.cells_of(objects, line)?;
            if net && !cell {
                return Err(self.error(
                    line,
                    "set_timing_derate -net_delay on instances is not supported",
                ));
            }
            for id in cells {
                let derate = self.sdc.cell_derates.entry(id).or_default();
                for &side in &sides {
                    derate.cell[side] = Some(factor);
                }
            }
            return Ok(String::new());
        }
        for side in sides {
            if cell {
                self.sdc.derating.cell[side] = factor;
            }
            if net {
                self.sdc.derating.net[side] = factor;
            }
        }
        Ok(String::new())
    }

    fn set_design_limit(
        &mut self,
        command: &str,
        args: &[String],
        line: usize,
    ) -> Result<String, ParseError> {
        let parsed = self.options(args, &SET_DESIGN_LIMIT, line)?;
        let [value_text, rest @ ..] = parsed.positional.as_slice() else {
            return Err(self.error(line, format!("{command} wants a value")));
        };
        if let [objects] = rest
            && objects != &self.db.name
            && Object::decode(objects).is_some()
        {
            return Err(self.error(line, format!("{command} on ports is not supported")));
        }
        let value: f64 = value_text
            .parse()
            .map_err(|_| self.error(line, format!("'{value_text}' is not a number")))?;
        match command {
            "set_max_fanout" => self.sdc.limits.max_fanout = Some(value),
            "set_max_transition" => {
                self.sdc.limits.max_transition = Some(value * self.units.time);
            }
            _ => self.sdc.limits.max_capacitance = Some(value * self.units.capacitance),
        }
        Ok(String::new())
    }

    fn set_load(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &SET_LOAD, line)?;
        let [value_text, ports_text] = parsed.positional.as_slice() else {
            return Err(self.error(line, "set_load wants a value and a port list"));
        };
        let value: f64 = value_text
            .parse()
            .map_err(|_| self.error(line, format!("load '{value_text}' is not a number")))?;
        for port in self.ports_of(ports_text, line)? {
            let _ = self
                .sdc
                .port_loads
                .insert(port, value * self.units.capacitance);
        }
        Ok(String::new())
    }

    fn set_input_transition(&mut self, args: &[String], line: usize) -> Result<String, ParseError> {
        let parsed = self.options(args, &SET_INPUT_TRANSITION, line)?;
        let [value_text, ports_text] = parsed.positional.as_slice() else {
            return Err(self.error(line, "set_input_transition wants a value and a port list"));
        };
        let value = self.time(value_text, line, "transition")?;
        let edges = edges_of(&parsed);
        let sides = sides_of(&parsed);
        for port in self.ports_of(ports_text, line)? {
            self.sdc
                .input_slews
                .entry(port)
                .or_default()
                .set(&edges, &sides, value);
        }
        Ok(String::new())
    }
}

/// The edges `-rise`/`-fall` flags pick; both without either.
fn edges_of(parsed: &Args) -> Vec<Edge> {
    match (parsed.flag("-rise"), parsed.flag("-fall")) {
        (true, false) => vec![Edge::Rise],
        (false, true) => vec![Edge::Fall],
        _ => vec![Edge::Rise, Edge::Fall],
    }
}

/// The sides `-min`/`-max` flags pick; both without either.
fn sides_of(parsed: &Args) -> Vec<MinMax> {
    match (parsed.flag("-min"), parsed.flag("-max")) {
        (true, false) => vec![MinMax::Min],
        (false, true) => vec![MinMax::Max],
        _ => vec![MinMax::Min, MinMax::Max],
    }
}

/// The sides `-setup`/`-hold` flags pick; both without either.
fn sides_of_setup_hold(parsed: &Args) -> Vec<MinMax> {
    match setup_hold_side(parsed) {
        Some(side) => vec![side],
        None => vec![MinMax::Min, MinMax::Max],
    }
}

/// The one side `-setup` or `-hold` picks; none for both.
fn setup_hold_side(parsed: &Args) -> Option<MinMax> {
    match (parsed.flag("-setup"), parsed.flag("-hold")) {
        (true, false) => Some(MinMax::Max),
        (false, true) => Some(MinMax::Min),
        _ => None,
    }
}

/// What one port delay command sets on each of its ports.
struct PortDelaySpec {
    reference: ClockEdge,
    edges: Vec<Edge>,
    sides: Vec<MinMax>,
    delay: Seconds,
    add: bool,
}

/// The options a command takes, as `OpenSTA` defines them: flags, keys
/// that take a value, and options this reader rejects as not supported
/// because ignoring them would change what the constraint means.
struct Options {
    flags: &'static [&'static str],
    keys: &'static [&'static str],
    unsupported: &'static [&'static str],
}

const PATH_UNSUPPORTED: &[&str] = &[
    "-rise",
    "-fall",
    "-reset_path",
    "-rise_from",
    "-fall_from",
    "-through",
    "-rise_through",
    "-fall_through",
    "-rise_to",
    "-fall_to",
];

const OBJECT_QUERY_UNSUPPORTED: &[&str] = &["-regexp", "-nocase", "-filter", "-of_objects"];

const SET_UNITS: Options = Options {
    flags: &[],
    keys: &[
        "-time",
        "-capacitance",
        "-resistance",
        "-voltage",
        "-current",
        "-power",
        "-distance",
    ],
    unsupported: &[],
};

const GET_PORTS: Options = Options {
    flags: &["-quiet"],
    keys: &[],
    unsupported: OBJECT_QUERY_UNSUPPORTED,
};

/// Hierarchy is flat here, so `-hierarchical` changes nothing.
const GET_CELLS: Options = Options {
    flags: &["-quiet", "-hierarchical"],
    keys: &[],
    unsupported: &["-regexp", "-nocase", "-filter", "-of_objects", "-hsc"],
};

const GET_CLOCKS: Options = Options {
    flags: &["-quiet"],
    keys: &[],
    unsupported: &["-regexp", "-nocase", "-filter"],
};

const ALL_INPUTS: Options = Options {
    flags: &["-no_clocks"],
    keys: &[],
    unsupported: &[],
};

const ALL_OUTPUTS: Options = Options {
    flags: &[],
    keys: &[],
    unsupported: &[],
};

const CREATE_CLOCK: Options = Options {
    flags: &["-add"],
    keys: &["-name", "-period", "-waveform", "-comment"],
    unsupported: &[],
};

const PORT_DELAY: Options = Options {
    flags: &[
        "-rise",
        "-fall",
        "-max",
        "-min",
        "-clock_fall",
        "-add_delay",
    ],
    keys: &["-clock"],
    unsupported: &[
        "-reference_pin",
        "-source_latency_included",
        "-network_latency_included",
    ],
};

const SET_PROPAGATED_CLOCK: Options = Options {
    flags: &[],
    keys: &[],
    unsupported: &[],
};

/// `-rise` and `-fall` only pick the edge of an inter-clock `-to`.
const SET_CLOCK_UNCERTAINTY: Options = Options {
    flags: &["-setup", "-hold", "-rise", "-fall"],
    keys: &[],
    unsupported: &[
        "-from",
        "-rise_from",
        "-fall_from",
        "-to",
        "-rise_to",
        "-fall_to",
    ],
};

const SET_FALSE_PATH: Options = Options {
    flags: &["-setup", "-hold"],
    keys: &["-from", "-to", "-comment"],
    unsupported: PATH_UNSUPPORTED,
};

const SET_MULTICYCLE_PATH: Options = Options {
    flags: &["-setup", "-hold", "-start", "-end"],
    keys: &["-from", "-to", "-comment"],
    unsupported: PATH_UNSUPPORTED,
};

const SET_TIMING_DERATE: Options = Options {
    flags: &["-early", "-late", "-cell_delay", "-net_delay"],
    keys: &[],
    unsupported: &["-rise", "-fall", "-clock", "-data", "-cell_check"],
};

const SET_DESIGN_LIMIT: Options = Options {
    flags: &[],
    keys: &[],
    unsupported: &["-clock_path", "-data_path", "-rise", "-fall"],
};

/// `-pin_load` is the default.
const SET_LOAD: Options = Options {
    flags: &["-pin_load"],
    keys: &[],
    unsupported: &[
        "-rise",
        "-fall",
        "-min",
        "-max",
        "-subtract_pin_load",
        "-wire_load",
    ],
};

const SET_INPUT_TRANSITION: Options = Options {
    flags: &["-rise", "-fall", "-min", "-max"],
    keys: &[],
    unsupported: &[],
};

/// Whether `word` is an option rather than a value; `-1.5` is a number.
fn is_option(word: &str) -> bool {
    word.len() > 1
        && word.starts_with('-')
        && !word[1..].starts_with(|c: char| c.is_ascii_digit() || c == '.')
}

/// Splits a command's arguments by its `options`, refusing any option it
/// does not take or that is not supported.
fn parse_args(args: &[String], options: &Options) -> Result<Args, String> {
    let mut parsed = Args::default();
    let mut words = args.iter();
    while let Some(word) = words.next() {
        if !is_option(word) {
            parsed.positional.push(word.clone());
            continue;
        }
        let name = word.as_str();
        if options.unsupported.contains(&name) {
            return Err(format!("{name} is not supported"));
        }
        if options.flags.contains(&name) {
            parsed.flags.push(word.clone());
        } else if options.keys.contains(&name) {
            let value = words
                .next()
                .ok_or_else(|| format!("{name} wants a value"))?;
            let _ = parsed.keys.insert(word.clone(), value.clone());
        } else {
            return Err(format!("unknown option {name}"));
        }
    }
    Ok(parsed)
}

/// The scale of a unit like `ps` or `1ns` with the given base suffix,
/// matched in any case as `check_unit` does; the prefix keeps its case, so
/// `M` is mega and `m` milli.
fn unit_scale(text: &str, suffix: &str) -> Option<f64> {
    let split = text.len().checked_sub(suffix.len())?;
    if !text.get(split..)?.eq_ignore_ascii_case(suffix) {
        return None;
    }
    let body = &text[..split];
    let (multiplier, prefix) = match body.chars().last() {
        Some(c) if c.is_ascii_alphabetic() => (&body[..body.len() - 1], Some(c)),
        _ => (body, None),
    };
    let multiplier: f64 = if multiplier.is_empty() {
        1.0
    } else {
        multiplier.parse().ok()?
    };
    let prefix_scale = match prefix {
        None => 1.0,
        Some('m') => 1e-3,
        Some('u') => 1e-6,
        Some('n') => 1e-9,
        Some('p') => 1e-12,
        Some('f') => 1e-15,
        Some('k') => 1e3,
        Some('M') => 1e6,
        Some(_) => return None,
    };
    Some(multiplier * prefix_scale)
}

/// Whether `name` matches a glob of `*` and `?`; other characters,
/// brackets included, match themselves.
fn glob_matches(pattern: &str, name: &str) -> bool {
    let p: Vec<char> = pattern.chars().collect();
    let n: Vec<char> = name.chars().collect();
    let mut dp = vec![vec![false; n.len() + 1]; p.len() + 1];
    dp[0][0] = true;
    for i in 1..=p.len() {
        if p[i - 1] == '*' {
            dp[i][0] = dp[i - 1][0];
        }
    }
    for i in 1..=p.len() {
        for j in 1..=n.len() {
            dp[i][j] = match p[i - 1] {
                '*' => dp[i - 1][j] || dp[i][j - 1],
                '?' => dp[i - 1][j - 1],
                c => dp[i - 1][j - 1] && c == n[j - 1],
            };
        }
    }
    dp[p.len()][n.len()]
}

/// One word of a command before substitution.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Word {
    /// Braces kept the text literal.
    Braced(String),
    /// Variables and bracketed commands are still to be substituted.
    Plain(String),
}

/// The commands of a script with the line each starts on: newlines and
/// semicolons end a command outside braces, brackets and quotes, a
/// backslash continues a line, and a `#` starting a command is a comment.
fn split_commands(script: &str, first_line: usize) -> Vec<(String, usize)> {
    let mut splitter = CommandSplitter {
        chars: script.chars().collect(),
        i: 0,
        line: first_line,
        start_line: first_line,
        current: String::new(),
        brace_depth: 0,
        bracket_depth: 0,
        in_quotes: false,
        at_command_start: true,
        commands: Vec::new(),
    };
    while splitter.i < splitter.chars.len() {
        let c = splitter.chars[splitter.i];
        if splitter.at_command_start && splitter.between_commands(c) {
            continue;
        }
        splitter.within_command(c);
    }
    if !splitter.current.trim().is_empty() {
        splitter
            .commands
            .push((splitter.current, splitter.start_line));
    }
    splitter.commands
}

/// Where [`split_commands`] is in a script, and the commands it has split.
struct CommandSplitter {
    chars: Vec<char>,
    i: usize,
    line: usize,
    start_line: usize,
    current: String,
    brace_depth: usize,
    bracket_depth: usize,
    in_quotes: bool,
    at_command_start: bool,
    commands: Vec<(String, usize)>,
}

impl CommandSplitter {
    /// Takes `c` between two commands, where blanks, empty commands and
    /// comments are skipped; false when `c` starts the next command.
    fn between_commands(&mut self, c: char) -> bool {
        if c == ' ' || c == '\t' {
            self.i += 1;
            return true;
        }
        if c == '\n' || c == ';' {
            if c == '\n' {
                self.line += 1;
            }
            self.start_line = self.line;
            self.i += 1;
            return true;
        }
        if c == '#' {
            self.skip_comment();
            return true;
        }
        self.at_command_start = false;
        self.start_line = self.line;
        false
    }

    /// Skips a comment up to the end of its line, which a backslash
    /// continues.
    fn skip_comment(&mut self) {
        let chars = &self.chars;
        while self.i < chars.len() && chars[self.i] != '\n' {
            if chars[self.i] == '\\' && self.i + 1 < chars.len() && chars[self.i + 1] == '\n' {
                self.line += 1;
                self.i += 1;
            }
            self.i += 1;
        }
    }

    /// Takes `c` inside a command, ending the command at a newline or
    /// semicolon outside braces, brackets and quotes.
    fn within_command(&mut self, c: char) {
        match c {
            '\\' if self.i + 1 < self.chars.len() => {
                self.take_backslash();
                return;
            }
            '{' if !self.in_quotes => self.brace_depth += 1,
            '}' if !self.in_quotes => self.brace_depth = self.brace_depth.saturating_sub(1),
            '[' if !self.in_quotes && self.brace_depth == 0 => self.bracket_depth += 1,
            ']' if !self.in_quotes && self.brace_depth == 0 => {
                self.bracket_depth = self.bracket_depth.saturating_sub(1);
            }
            '"' if self.brace_depth == 0 => self.in_quotes = !self.in_quotes,
            '\n' | ';' if self.brace_depth == 0 && self.bracket_depth == 0 && !self.in_quotes => {
                if c == '\n' {
                    self.line += 1;
                }
                self.commands
                    .push((std::mem::take(&mut self.current), self.start_line));
                self.at_command_start = true;
                self.i += 1;
                return;
            }
            '\n' => self.line += 1,
            _ => {}
        }
        self.current.push(c);
        self.i += 1;
    }

    /// Takes a backslash and what follows: a line continuation becomes one
    /// space, eating the next line's indent; any other pair is kept.
    fn take_backslash(&mut self) {
        let next = self.chars[self.i + 1];
        if next == '\n' {
            self.line += 1;
            self.current.push(' ');
            self.i += 2;
            while self.i < self.chars.len() && matches!(self.chars[self.i], ' ' | '\t') {
                self.i += 1;
            }
            return;
        }
        self.current.push('\\');
        self.current.push(next);
        self.i += 2;
    }
}

/// The words of one command.
fn split_words(text: &str) -> Result<Vec<Word>, String> {
    let chars: Vec<char> = text.chars().collect();
    let mut words = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        if chars[i].is_whitespace() {
            i += 1;
            continue;
        }
        let (word, next) = match chars[i] {
            '{' => braced_word(&chars, i)?,
            '"' => quoted_word(&chars, i)?,
            _ => bare_word(&chars, i)?,
        };
        words.push(word);
        i = next;
    }
    Ok(words)
}

/// The braced word opening at `open`, and the index after it, which must
/// end the word.
fn braced_word(chars: &[char], open: usize) -> Result<(Word, usize), String> {
    let end = matching_brace(chars, open).ok_or("unbalanced '{'")?;
    let word = Word::Braced(chars[open + 1..end].iter().collect());
    let next = end + 1;
    if next < chars.len() && !chars[next].is_whitespace() {
        return Err("extra characters after close-brace".to_string());
    }
    Ok((word, next))
}

/// The quoted word opening at `open`, and the index after its closing
/// quote.
fn quoted_word(chars: &[char], open: usize) -> Result<(Word, usize), String> {
    let mut j = open + 1;
    let mut word = String::new();
    while j < chars.len() && chars[j] != '"' {
        if let Some(next) = copy_escape_or_command(chars, j, &mut word)? {
            j = next;
            continue;
        }
        word.push(chars[j]);
        j += 1;
    }
    if j >= chars.len() {
        return Err("unbalanced '\"'".to_string());
    }
    Ok((Word::Plain(word), j + 1))
}

/// The bare word starting at `start`, and the index of the blank after it.
fn bare_word(chars: &[char], start: usize) -> Result<(Word, usize), String> {
    let mut j = start;
    let mut word = String::new();
    while j < chars.len() && !chars[j].is_whitespace() {
        if let Some(next) = copy_escape_or_command(chars, j, &mut word)? {
            j = next;
            continue;
        }
        if chars[j] == '{' {
            let end = matching_brace(chars, j).ok_or("unbalanced '{'")?;
            word.extend(&chars[j..=end]);
            j = end + 1;
            continue;
        }
        word.push(chars[j]);
        j += 1;
    }
    Ok((Word::Plain(word), j))
}

/// Copies onto `word` the backslash pair or bracketed command starting at
/// `j`, if one does, returning the index after it.
fn copy_escape_or_command(
    chars: &[char],
    j: usize,
    word: &mut String,
) -> Result<Option<usize>, String> {
    if chars[j] == '\\' && j + 1 < chars.len() {
        word.push(chars[j]);
        word.push(chars[j + 1]);
        return Ok(Some(j + 2));
    }
    if chars[j] == '[' {
        let end = matching_bracket(chars, j).ok_or("unbalanced '['")?;
        word.extend(&chars[j..=end]);
        return Ok(Some(end + 1));
    }
    Ok(None)
}

/// The index of the brace closing the one at `open`.
const fn matching_brace(chars: &[char], open: usize) -> Option<usize> {
    let mut depth = 0usize;
    let mut i = open;
    while i < chars.len() {
        match chars[i] {
            '\\' => i += 1,
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(i);
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

/// The index of the bracket closing the one at `open`, skipping braced
/// and quoted text.
fn matching_bracket(chars: &[char], open: usize) -> Option<usize> {
    let mut depth = 0usize;
    let mut i = open;
    while i < chars.len() {
        match chars[i] {
            '\\' => i += 1,
            '{' => i = matching_brace(chars, i)?,
            '"' => {
                i += 1;
                while i < chars.len() && chars[i] != '"' {
                    if chars[i] == '\\' {
                        i += 1;
                    }
                    i += 1;
                }
            }
            '[' => depth += 1,
            ']' => {
                depth -= 1;
                if depth == 0 {
                    return Some(i);
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

/// The variable name at `start` after a `$`, and where it ends.
fn variable_name(chars: &[char], start: usize) -> (String, usize) {
    if chars.get(start) == Some(&'{') {
        let end = chars[start..]
            .iter()
            .position(|&c| c == '}')
            .map_or(chars.len(), |p| start + p);
        return (
            chars[start + 1..end].iter().collect(),
            (end + 1).min(chars.len()),
        );
    }
    let mut end = start;
    while end < chars.len() && (chars[end].is_ascii_alphanumeric() || chars[end] == '_') {
        end += 1;
    }
    (chars[start..end].iter().collect(), end)
}

const fn unescape(c: char) -> char {
    match c {
        'n' => '\n',
        't' => '\t',
        other => other,
    }
}

/// The elements of a Tcl list: words, with braces and quotes grouping.
pub(crate) fn split_list(text: &str) -> Vec<String> {
    let chars: Vec<char> = text.chars().collect();
    let mut items = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        if chars[i].is_whitespace() {
            i += 1;
            continue;
        }
        if chars[i] == '{'
            && let Some(end) = matching_brace(&chars, i)
        {
            items.push(chars[i + 1..end].iter().collect());
            i = end + 1;
            continue;
        }
        if chars[i] == '"' {
            let mut j = i + 1;
            let mut item = String::new();
            while j < chars.len() && chars[j] != '"' {
                if chars[j] == '\\' && j + 1 < chars.len() {
                    item.push(unescape(chars[j + 1]));
                    j += 2;
                    continue;
                }
                item.push(chars[j]);
                j += 1;
            }
            items.push(item);
            i = j + 1;
            continue;
        }
        let mut j = i;
        let mut item = String::new();
        while j < chars.len() && !chars[j].is_whitespace() {
            if chars[j] == '\\' && j + 1 < chars.len() {
                item.push(unescape(chars[j + 1]));
                j += 2;
                continue;
            }
            item.push(chars[j]);
            j += 1;
        }
        items.push(item);
        i = j;
    }
    items
}

/// A Tcl list of `items`, bracing any that needs it.
fn join_list(items: &[String]) -> String {
    items
        .iter()
        .map(|item| {
            let special = item.is_empty()
                || item
                    .chars()
                    .any(|c| c.is_whitespace() || "{}[]$\"\\;".contains(c));
            if special {
                format!("{{{item}}}")
            } else {
                item.clone()
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

#[derive(Clone, Debug, PartialEq)]
enum ExprToken {
    Number(Number),
    Op(char),
    Open,
    Close,
}

fn tokenize_expr(text: &str) -> Result<Vec<ExprToken>, String> {
    let chars: Vec<char> = text.chars().collect();
    let mut tokens = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c.is_whitespace() {
            i += 1;
        } else if c.is_ascii_digit() || c == '.' {
            let start = i;
            while i < chars.len()
                && (chars[i].is_ascii_alphanumeric()
                    || chars[i] == '.'
                    || ((chars[i] == '+' || chars[i] == '-') && matches!(chars[i - 1], 'e' | 'E')))
            {
                i += 1;
            }
            let text: String = chars[start..i].iter().collect();
            let number = Number::parse(&text).ok_or_else(|| format!("'{text}' is not a number"))?;
            tokens.push(ExprToken::Number(number));
        } else if "+-*/%".contains(c) {
            tokens.push(ExprToken::Op(c));
            i += 1;
        } else if c == '(' {
            tokens.push(ExprToken::Open);
            i += 1;
        } else if c == ')' {
            tokens.push(ExprToken::Close);
            i += 1;
        } else {
            return Err(format!("cannot evaluate expr '{text}'"));
        }
    }
    Ok(tokens)
}

struct ExprParser {
    tokens: Vec<ExprToken>,
    at: usize,
}

impl ExprParser {
    fn peek_op(&self, ops: &str) -> Option<char> {
        match self.tokens.get(self.at) {
            Some(ExprToken::Op(c)) if ops.contains(*c) => Some(*c),
            _ => None,
        }
    }

    fn sum(&mut self) -> Result<Number, String> {
        let mut value = self.product()?;
        while let Some(op) = self.peek_op("+-") {
            self.at += 1;
            let rhs = self.product()?;
            value = value.apply(op, rhs).ok_or("division by zero in expr")?;
        }
        Ok(value)
    }

    fn product(&mut self) -> Result<Number, String> {
        let mut value = self.unary()?;
        while let Some(op) = self.peek_op("*/%") {
            self.at += 1;
            let rhs = self.unary()?;
            value = value.apply(op, rhs).ok_or("division by zero in expr")?;
        }
        Ok(value)
    }

    fn unary(&mut self) -> Result<Number, String> {
        if let Some(op) = self.peek_op("+-") {
            self.at += 1;
            let value = self.unary()?;
            return Ok(if op == '-' {
                match value {
                    Number::Int(i) => Number::Int(-i),
                    Number::Double(d) => Number::Double(-d),
                }
            } else {
                value
            });
        }
        match self.tokens.get(self.at).cloned() {
            Some(ExprToken::Number(n)) => {
                self.at += 1;
                Ok(n)
            }
            Some(ExprToken::Open) => {
                self.at += 1;
                let value = self.sum()?;
                if self.tokens.get(self.at) != Some(&ExprToken::Close) {
                    return Err("unbalanced '(' in expr".to_string());
                }
                self.at += 1;
                Ok(value)
            }
            _ => Err("malformed expr".to_string()),
        }
    }
}

#[cfg(test)]
mod tests;
