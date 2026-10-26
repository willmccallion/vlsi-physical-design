//! Clock-gating checks inferred from cell functions, as `OpenSTA`'s
//! `GatedClk`.
//!
//! An input of a gate whose output is in a clock network, whose function
//! is an AND or an OR of the clock with that input, is a gated clock's
//! enable: it must settle before the clock's leading edge and hold until
//! its trailing edge, so the gate passes whole pulses.

use crate::netlist::{ClockNetwork, Netlist};
use pare_common::db::design::PinOwner;
use pare_common::db::indices::PinId;
use pare_common::db::liberty::{ArcRole, Edge, FuncExpr, MinMax};
use rustc_hash::FxHashMap;

/// The clock value at which a gate passes its clock through.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActiveValue {
    /// The gate passes the clock while the enable is high, as an AND.
    One,
    /// The gate passes the clock while the enable is low, as an OR.
    Zero,
}

impl ActiveValue {
    const fn flipped(self) -> Self {
        match self {
            Self::One => Self::Zero,
            Self::Zero => Self::One,
        }
    }

    /// The clock edge a check on `side` is made against: the leading edge
    /// for setup, the trailing one for hold, as `gatedClkActiveTrans`.
    #[must_use]
    pub const fn check_edge(self, side: MinMax) -> Edge {
        let leading = match self {
            Self::One => Edge::Rise,
            Self::Zero => Edge::Fall,
        };
        match (side, leading) {
            (MinMax::Max, edge) => edge,
            (MinMax::Min, Edge::Rise) => Edge::Fall,
            (MinMax::Min, Edge::Fall) => Edge::Rise,
        }
    }
}

/// A gated clock's enable: the gate's clock input and its active value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GatedEnable {
    /// The gate's clock input.
    pub clock: PinId,
    /// The value at which the gate passes the clock.
    pub active: ActiveValue,
}

/// Whether `func` gates `clock` with `enable`, and at which clock value,
/// as `OpenSTA`'s `isClkGatingFunc`.
///
/// Under any outer negation the function must be an AND or an OR, one of
/// whose operands is the clock or its negation and another of which reads
/// the enable.
#[must_use]
pub fn gating(func: &FuncExpr, enable: &str, clock: &str) -> Option<ActiveValue> {
    let mut root = func;
    while let FuncExpr::Not(inner) = root {
        root = inner;
    }
    let (mut active, is_and) = match root {
        FuncExpr::And(..) => (ActiveValue::One, true),
        FuncExpr::Or(..) => (ActiveValue::Zero, false),
        _ => return None,
    };
    let mut operands = Vec::new();
    collect_operands(root, is_and, &mut operands);
    let mut gates_clock = false;
    for operand in &operands {
        match operand {
            FuncExpr::Not(inner) if matches!(&**inner, FuncExpr::Port(p) if p == clock) => {
                gates_clock = true;
                active = active.flipped();
            }
            FuncExpr::Port(p) if p == clock => gates_clock = true,
            _ => {}
        }
    }
    let reads_enable = operands.iter().any(|o| reads(o, enable));
    (gates_clock && reads_enable).then_some(active)
}

/// The operands of an AND (or an OR) tree, as `functionClkOperands`.
fn collect_operands<'f>(expr: &'f FuncExpr, is_and: bool, out: &mut Vec<&'f FuncExpr>) {
    match (expr, is_and) {
        (FuncExpr::And(a, b), true) | (FuncExpr::Or(a, b), false) => {
            collect_operands(a, is_and, out);
            collect_operands(b, is_and, out);
        }
        _ => out.push(expr),
    }
}

/// Whether `expr` reads `port`.
fn reads(expr: &FuncExpr, port: &str) -> bool {
    match expr {
        FuncExpr::Zero | FuncExpr::One => false,
        FuncExpr::Port(p) => p == port,
        FuncExpr::Not(a) => reads(a, port),
        FuncExpr::And(a, b) | FuncExpr::Or(a, b) | FuncExpr::Xor(a, b) => {
            reads(a, port) || reads(b, port)
        }
    }
}

/// The names of the ports `expr` reads, each once, in order of reading.
fn ports(expr: &FuncExpr, out: &mut Vec<String>) {
    match expr {
        FuncExpr::Zero | FuncExpr::One => {}
        FuncExpr::Port(p) => {
            if !out.contains(p) {
                out.push(p.clone());
            }
        }
        FuncExpr::Not(a) => ports(a, out),
        FuncExpr::And(a, b) | FuncExpr::Or(a, b) | FuncExpr::Xor(a, b) => {
            ports(a, out);
            ports(b, out);
        }
    }
}

/// Every gated clock enable of the design, as `OpenSTA`'s
/// `isGatedClkEnable` finds them.
///
/// An enable is an input pin outside the clock network with a
/// combinational arc to an output in it whose function gates a clock input
/// of the same cell with it.
#[must_use]
pub fn gated_enables(
    netlist: &Netlist<'_>,
    clocks: &ClockNetwork,
) -> FxHashMap<PinId, GatedEnable> {
    let db = netlist.db;
    let mut found = FxHashMap::default();
    for (i, pin) in db.pins.iter().enumerate() {
        let enable = PinId::new(i);
        let PinOwner::Cell { cell, .. } = pin.owner else {
            continue;
        };
        if clocks.is_clock(enable) || !netlist.is_load(enable) {
            continue;
        }
        let Some((lib_id, enable_port)) = netlist.link.pin_port(db, enable) else {
            continue;
        };
        let lib = netlist.link.liberty.cell(lib_id);
        if !lib.ports[enable_port].is_input() {
            continue;
        }
        let outputs = lib
            .arc_sets
            .iter()
            .filter(|s| s.role == ArcRole::Combinational && s.from == enable_port)
            .map(|s| s.to);
        'outputs: for output in outputs {
            let Some(out_pin) = netlist.pin_of_port(cell, output) else {
                continue;
            };
            let Some(func) = &lib.ports[output].function else {
                continue;
            };
            if !clocks.is_clock(out_pin) {
                continue;
            }
            let enable_name = &lib.ports[enable_port].name;
            let mut names = Vec::new();
            ports(func, &mut names);
            for clock_name in names.iter().filter(|n| *n != enable_name) {
                let Some(active) = gating(func, enable_name, clock_name) else {
                    continue;
                };
                let Some(clock) = lib
                    .port(clock_name)
                    .and_then(|p| netlist.pin_of_port(cell, p))
                else {
                    continue;
                };
                if clocks.is_clock(clock) {
                    let _ = found.insert(enable, GatedEnable { clock, active });
                    break 'outputs;
                }
            }
        }
    }
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(text: &str) -> FuncExpr {
        FuncExpr::parse(text).unwrap()
    }

    #[test]
    fn an_and_gates_its_clock_while_the_enable_is_high() {
        assert_eq!(gating(&f("(CK & EN)"), "EN", "CK"), Some(ActiveValue::One));
    }

    #[test]
    fn an_or_gates_its_clock_while_the_enable_is_low() {
        assert_eq!(gating(&f("(CK | EN)"), "EN", "CK"), Some(ActiveValue::Zero));
    }

    #[test]
    fn a_nand_still_gates_and_an_inverted_clock_flips_the_active_value() {
        assert_eq!(gating(&f("!(CK & EN)"), "EN", "CK"), Some(ActiveValue::One));
        assert_eq!(
            gating(&f("(!CK & EN)"), "EN", "CK"),
            Some(ActiveValue::Zero)
        );
    }

    #[test]
    fn a_wider_and_gates_with_any_of_its_other_inputs() {
        assert_eq!(
            gating(&f("((A1 & A2) & A3)"), "A3", "A1"),
            Some(ActiveValue::One)
        );
    }

    #[test]
    fn an_xor_or_a_mux_does_not_gate() {
        assert_eq!(gating(&f("(CK ^ EN)"), "EN", "CK"), None);
        assert_eq!(gating(&f("((S & B) | (!S & A))"), "S", "A"), None);
    }

    #[test]
    fn setup_checks_the_leading_edge_and_hold_the_trailing_one() {
        assert_eq!(ActiveValue::One.check_edge(MinMax::Max), Edge::Rise);
        assert_eq!(ActiveValue::One.check_edge(MinMax::Min), Edge::Fall);
        assert_eq!(ActiveValue::Zero.check_edge(MinMax::Max), Edge::Fall);
        assert_eq!(ActiveValue::Zero.check_edge(MinMax::Min), Edge::Rise);
    }
}
