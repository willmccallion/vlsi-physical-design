use super::*;
use crate::bnet::PinLoad;
use crate::resizer::GlobalRouting;
use crate::route_parasitics::RouteRc;
use crate::testing::{
    DefSections, cells, cells_with, clock_cells_source, def_text, design, design_with_copies, pin,
    with_timed_session, with_wired_session,
};
use crate::testing::{metal2_wires, with_session_inputs};
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::{NetRoute, RouteSource};
use pare_common::db::indices::{CutLayerId, LayerId};
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::liberty::parse_str;
use std::fmt::Write;
use std::path::Path as FilePath;

/// An AND gate whose inputs reach `ZN` with the given rise and fall
/// delays, in nanoseconds.
fn and2(a1: (f64, f64), a2: (f64, f64)) -> LibCell {
    let timing = |pin: &str, (rise, fall): (f64, f64)| {
        format!(
            "timing () {{ related_pin : \"{pin}\"; timing_sense : positive_unate;
                   cell_rise(scalar) {{ values (\"{rise}\"); }}
                   cell_fall(scalar) {{ values (\"{fall}\"); }} }}"
        )
    };
    let src = format!(
        "library (t) {{ time_unit : \"1ns\"; capacitive_load_unit (1,ff);
               cell (AND2_X1) {{
                 pin (A1) {{ direction : input; capacitance : 1.0; }}
                 pin (A2) {{ direction : input; capacitance : 1.0; }}
                 pin (ZN) {{ direction : output; function : \"(A1 & A2)\"; {} {} }}
               }} }}",
        timing("A1", a1),
        timing("A2", a2)
    );
    let lib = parse_str(&src, FilePath::new("t.lib")).unwrap();
    lib.cells[LibCellId(0)].clone()
}

fn candidate(cell: &LibCell) -> Option<LibPortId> {
    let slews = EdgeValues {
        rise: Seconds::ZERO,
        fall: Seconds::ZERO,
    };
    swap_candidate(
        cell,
        LibPortId::new(0),
        LibPortId::new(2),
        &[LibPortId::new(1)],
        Farads::ZERO,
        &PortSlews::default(),
        &slews,
    )
}

/// A buffer `name` on `BUF_X1`'s footprint that may drive `max_cap_ff`
/// femtofarads and slews to `slew` nanoseconds at 100 fF.
fn sized_buffer(name: &str, max_cap_ff: f64, slew: f64) -> String {
    format!(
        "cell ({name}) {{ area : 0.8 ;
               pin (A) {{ direction : input ; capacitance : 0.97 ; }}
               pin (Z) {{ direction : output ; function : \"A\" ; max_capacitance : {max_cap_ff} ;
                 timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
                   cell_rise (delay_t) {{ values (\"0.03, 0.30\", \"0.04, 0.31\") ; }}
                   rise_transition (delay_t) {{
                     values (\"0.0, {slew}\", \"0.0, {slew}\") ; }} }} }} }}\n"
    )
}

/// The size `half_driving_power_cell` picks for a `BUF_X1` under
/// `constraints`, among buffers `A` driving up to 30.6 fF, `B` up to
/// 30 fF and `C` up to 10 fF, `A` the weakest.
fn half_driver_under(constraints: &str) -> String {
    let sizes = [("A", 30.6, 0.9), ("B", 30.0, 0.5), ("C", 10.0, 0.45)];
    half_driver_among(&sizes, constraints)
}

/// The size `half_driving_power_cell` picks for a `BUF_X1` under
/// `constraints`, among buffers of the given names, load limits in
/// femtofarads and slews in nanoseconds at 100 fF.
fn half_driver_among(sizes: &[(&str, f64, f64)], constraints: &str) -> String {
    let copies: Vec<(&str, &str)> = sizes.iter().map(|&(n, _, _)| ("BUF_X1", n)).collect();
    let mut db = design_with_copies(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN sizes ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\nEND DESIGN\n",
        &copies,
    );
    let extra: String = sizes
        .iter()
        .map(|&(name, cap, slew)| sized_buffer(name, cap, slew))
        .collect();
    let lib = cells_with(&extra);
    let mut picked = String::new();
    with_timed_session(&mut db, &lib, constraints, |resizer| {
        let buf = resizer.link.liberty.cell_id("BUF_X1").unwrap();
        let half = half_driving_power_cell(resizer, buf).unwrap();
        picked.clone_from(&resizer.link.liberty.cell(half).name);
    });
    picked
}

#[test]
fn the_half_driving_size_matches_its_target_exactly_or_comes_closest() {
    assert_eq!(half_driver_under(""), "B");
}

#[test]
fn a_design_load_limit_bounds_every_size_the_half_driver_is_chosen_by() {
    assert_eq!(
        half_driver_under("set_max_capacitance 20 [current_design]\n"),
        "C"
    );
}

#[test]
fn an_isolation_cell_has_no_equivalent_inputs() {
    let gate = and2((0.05, 0.05), (0.05, 0.05));
    let src = "library (t) {
               cell (ISO) { is_isolation_cell : true ;
                 pin (A1) { direction : input; capacitance : 1.0; }
                 pin (A2) { direction : input; capacitance : 1.0; }
                 pin (ZN) { direction : output; function : \"(A1 & A2)\"; } } }";
    let isolation = parse_str(src, FilePath::new("t.lib")).unwrap().cells[LibCellId(0)].clone();

    assert_eq!(
        equivalent_pins(&gate, LibPortId::new(0)),
        [LibPortId::new(1)]
    );
    assert_eq!(equivalent_pins(&isolation, LibPortId::new(0)), []);
}

#[test]
fn another_input_is_judged_by_its_slowest_arc_not_its_first() {
    let cell = and2((0.05, 0.05), (0.03, 0.08));

    assert_eq!(candidate(&cell), None);
}

#[test]
fn another_input_faster_on_every_arc_is_taken() {
    let cell = and2((0.05, 0.05), (0.04, 0.03));

    assert_eq!(candidate(&cell), Some(LibPortId::new(1)));
}

/// A flop launching through an inverter into another flop.
fn inverted_pair() -> pare_common::db::design::NetlistDB {
    design(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN inv ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 3 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - i1 INV_X1 + PLACED ( 20000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 40000 2800 ) N ;\n\
             END COMPONENTS\nPINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 3 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- a ( f1 Q ) ( i1 A ) ;\n\
             - b ( i1 ZN ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n",
    )
}

/// The inverter's input slews unchanged and its inputs arriving
/// `adjust_ns` later, judged by `rule`.
fn later_by(adjust_ns: f64, rule: SlackRule) -> SlewImpact {
    let slew = EdgeValues::both(Seconds::new(0.02e-9));
    SlewImpact {
        old: slew,
        new: slew,
        adjust: Seconds::new(adjust_ns * 1e-9),
        margin: Seconds::ZERO,
        rule,
    }
}

#[test]
fn an_unchanged_arrival_keeps_the_no_worse_rule_under_a_margin() {
    let impact = SlewImpact {
        margin: Seconds::new(0.05e-9),
        ..later_by(0.0, SlackRule::NoWorse)
    };

    assert_eq!(inverter_keeps(&[impact]), [true]);
}

/// Whether the inverter's output keeps its slack under each impact,
/// with a clock of 1 ns.
fn inverter_keeps(impacts: &[SlewImpact]) -> Vec<bool> {
    let mut db = inverted_pair();
    let mut kept = Vec::new();
    let constraints = "create_clock -name c -period 1 [get_ports clk]\n";
    with_timed_session(
        &mut db,
        &cells(&clock_cells_source()),
        constraints,
        |resizer| {
            let inverter = resizer.db.cell_id("i1").unwrap();
            kept = impacts
                .iter()
                .map(|impact| input_slew_impact_ok(resizer, inverter, impact))
                .collect();
        },
    );
    kept
}

#[test]
fn a_later_arrival_breaks_the_no_worse_rule_but_not_a_positive_slack() {
    let impacts = [
        later_by(0.01, SlackRule::NoWorse),
        later_by(0.01, SlackRule::NonNegative),
    ];

    assert_eq!(inverter_keeps(&impacts), [false, true]);
}

#[test]
fn an_arrival_past_the_slack_breaks_the_non_negative_rule() {
    assert_eq!(
        inverter_keeps(&[later_by(10.0, SlackRule::NonNegative)]),
        [false]
    );
}

#[test]
fn an_earlier_arrival_keeps_both_rules() {
    let impacts = [
        later_by(-0.01, SlackRule::NoWorse),
        later_by(-0.01, SlackRule::NonNegative),
    ];

    assert_eq!(inverter_keeps(&impacts), [true, true]);
}

#[test]
fn the_critical_input_is_judged_by_its_slowest_arc() {
    let cell = and2((0.02, 0.06), (0.05, 0.05));

    assert_eq!(candidate(&cell), Some(LibPortId::new(1)));
}

/// A combinational cell with inputs `inputs` and one output `ZN` of
/// `function`, in that port order.
fn logic_cell(inputs: &[&str], function: &str) -> LibCell {
    multi_output_cell(inputs, &[("ZN", function)])
}

/// A combinational cell with inputs `inputs`, then the outputs `outputs`
/// of the given names and functions, in that port order.
fn multi_output_cell(inputs: &[&str], outputs: &[(&str, &str)]) -> LibCell {
    let mut input_pins = String::new();
    for name in inputs {
        writeln!(
            input_pins,
            "pin ({name}) {{ direction : input; capacitance : 1.0; }}"
        )
        .unwrap();
    }
    let mut output_pins = String::new();
    for (name, function) in outputs {
        writeln!(
            output_pins,
            "pin ({name}) {{ direction : output; function : \"{function}\"; }}"
        )
        .unwrap();
    }
    let src = format!(
        "library (t) {{ time_unit : \"1ns\"; capacitive_load_unit (1,ff);
           cell (GATE) {{ {input_pins} {output_pins} }} }}"
    );
    parse_str(&src, FilePath::new("t.lib")).unwrap().cells[LibCellId(0)].clone()
}

fn ports(indices: &[usize]) -> Vec<LibPortId> {
    indices.iter().map(|&i| LibPortId::new(i)).collect()
}

#[test]
fn every_input_of_an_and_gate_is_equivalent_to_the_others() {
    let gate = logic_cell(&["A", "B", "C"], "(A & B & C)");

    assert_eq!(equivalent_pins(&gate, LibPortId::new(0)), ports(&[1, 2]));
    assert_eq!(equivalent_pins(&gate, LibPortId::new(2)), ports(&[0, 1]));
}

#[test]
fn inputs_are_equivalent_only_where_swapping_them_never_changes_the_output() {
    let gate = logic_cell(&["A", "B", "C"], "(A & B & !C)");

    assert_eq!(equivalent_pins(&gate, LibPortId::new(0)), ports(&[1]));
    assert_eq!(equivalent_pins(&gate, LibPortId::new(1)), ports(&[0]));
    assert_eq!(equivalent_pins(&gate, LibPortId::new(2)), []);
}

#[test]
fn an_input_that_only_one_term_reads_has_no_equivalent() {
    let gate = logic_cell(&["A", "B"], "(A & !B)");
    let mixed = logic_cell(&["A", "B", "C"], "((A & B) | C)");

    assert_eq!(equivalent_pins(&gate, LibPortId::new(0)), []);
    assert_eq!(equivalent_pins(&mixed, LibPortId::new(0)), ports(&[1]));
    assert_eq!(equivalent_pins(&mixed, LibPortId::new(2)), []);
}

#[test]
fn inputs_must_be_equivalent_for_every_output_of_the_gate() {
    let gate = multi_output_cell(
        &["A", "B", "C"],
        &[("Z1", "(A & B & C)"), ("Z2", "(A & !B)")],
    );
    let reversed = multi_output_cell(
        &["A", "B", "C"],
        &[("Z1", "(A & !B)"), ("Z2", "(A & B & C)")],
    );

    assert_eq!(equivalent_pins(&gate, LibPortId::new(0)), []);
    assert_eq!(equivalent_pins(&reversed, LibPortId::new(0)), []);
}

#[test]
fn a_gate_of_one_input_or_none_to_speak_of_has_no_equivalent_pins() {
    let inverter = logic_cell(&["A"], "!A");
    let sink = multi_output_cell(&["A", "B"], &[]);

    assert_eq!(equivalent_pins(&inverter, LibPortId::new(0)), []);
    assert_eq!(equivalent_pins(&sink, LibPortId::new(0)), []);
}

/// An AND of `count` inputs `I0`, `I1`, ...
fn wide_and(count: usize) -> LibCell {
    let names: Vec<String> = (0..count).map(|i| format!("I{i}")).collect();
    let refs: Vec<&str> = names.iter().map(String::as_str).collect();
    logic_cell(&refs, &format!("({})", names.join(" & ")))
}

#[test]
fn pin_equivalence_is_simulated_for_up_to_sixteen_inputs() {
    assert_eq!(
        equivalent_pins(&wide_and(16), LibPortId::new(0)),
        ports(&(1..16).collect::<Vec<_>>())
    );
    assert_eq!(equivalent_pins(&wide_and(17), LibPortId::new(0)), []);
}

#[test]
fn an_input_that_is_as_fast_as_the_critical_one_is_not_a_swap() {
    let cell = and2((0.05, 0.05), (0.05, 0.05));

    assert_eq!(candidate(&cell), None);
}

#[test]
fn moves_are_counted_as_pending_until_committed_and_as_made_throughout() {
    let mut log = MoveLog::default();
    let cell = CellId::new(3);
    assert!(!log.has_pending(MoveKind::SizeUp, cell));

    log.count(MoveKind::SizeUp, cell, 3);

    assert!(log.has_pending(MoveKind::SizeUp, cell));
    assert!(!log.has_pending(MoveKind::SizeUp, CellId::new(4)));
    assert_eq!(
        (log.made(MoveKind::SizeUp), log.committed(MoveKind::SizeUp)),
        (3, 0)
    );

    log.commit();
    log.count(MoveKind::SizeUp, cell, 2);

    assert_eq!(
        (log.made(MoveKind::SizeUp), log.committed(MoveKind::SizeUp)),
        (5, 3)
    );
    log.undo();
    assert!(!log.has_pending(MoveKind::SizeUp, cell));
    assert!(log.has_moves(MoveKind::SizeUp, cell));
    assert_eq!(log.made(MoveKind::SizeUp), 3);
    assert_eq!(log.made(MoveKind::Clone), 0);
}

#[test]
fn the_intrinsic_delay_of_a_port_is_its_largest_arc_at_no_slew_and_no_load() {
    let cell = and2((0.05, 0.05), (0.03, 0.08));

    let intrinsic = intrinsic_delay(&cell, LibPortId::new(2));

    assert!(
        (intrinsic - Seconds::new(0.08e-9)).abs() < Seconds::new(1e-15),
        "{intrinsic:?}"
    );
}

#[test]
fn the_half_driving_size_is_the_one_nearest_half_the_load_limit_when_none_matches() {
    // Half of BUF_X1's 60 fF is 30: 40 fF is 10 away, 25 fF 5 and 10 fF 20.
    let sizes = [("A", 40.0, 0.9), ("B", 25.0, 0.5), ("C", 10.0, 0.45)];

    assert_eq!(half_driver_among(&sizes, ""), "B");
}

#[test]
fn of_two_sizes_equally_far_from_half_the_limit_the_first_is_taken() {
    let sizes = [("A", 40.0, 0.9), ("C", 20.0, 0.45)];

    assert_eq!(half_driver_among(&sizes, ""), "A");
}

/// A flop launching into buffer `g1`, which drives flops `f2` and `f3`
/// directly and, through buffer `h1`, flop `f4`, all along a row.
fn fork() -> NetlistDB {
    let components = "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
        - g1 BUF_X1 + PLACED ( 40000 2800 ) N ;\n\
        - f2 DFF_X1 + PLACED ( 90000 2800 ) N ;\n\
        - f3 DFF_X1 + PLACED ( 120000 14000 ) N ;\n\
        - h1 BUF_X1 + PLACED ( 70000 30000 ) N ;\n\
        - f4 DFF_X1 + PLACED ( 150000 30000 ) N ;\n";
    let nets = "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ( f3 CK ) ( f4 CK ) ;\n\
        - n0 ( f1 Q ) ( g1 A ) ;\n\
        - n1 ( g1 Z ) ( f2 D ) ( f3 D ) ( h1 A ) ;\n\
        - n2 ( h1 Z ) ( f4 D ) ;\n";
    design(&def_text(
        "fork",
        &DefSections {
            components,
            nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

const SLOW_CLOCK: &str = "create_clock -name clk -period 0.3 [get_ports clk]\n";

/// The pin `name` of an instance as `cell/pin`, or else of the design.
fn end_pin(db: &NetlistDB, name: &str) -> PinId {
    if name.contains('/') {
        return pin(db, name);
    }
    db.pins
        .iter_enumerated()
        .map(|(id, _)| id)
        .find(|&id| db.pin_name(id) == format!("PIN {name}"))
        .unwrap()
}

/// What the wires of a session are made of.
#[derive(Clone, Copy)]
enum Wires {
    /// Wires of no resistance or capacitance.
    Ideal,
    /// Wires of 3.5 ohms and 0.1 fF a micron.
    Metal2,
}

/// Runs `test` on the site of driver pin `driver` along the worst path
/// into `end` of `db` over `lib`, with metal wires.
fn on_site(
    db: &mut NetlistDB,
    lib: &Liberty,
    constraints: &str,
    pins: (&str, &str),
    test: impl FnOnce(&mut Resizer<'_, Sta>, &Path, usize),
) {
    on_site_with(Wires::Metal2, db, lib, constraints, pins, test);
}

/// [`on_site`] with `wires`.
fn on_site_with(
    wires: Wires,
    db: &mut NetlistDB,
    lib: &Liberty,
    constraints: &str,
    (end, driver): (&str, &str),
    test: impl FnOnce(&mut Resizer<'_, Sta>, &Path, usize),
) {
    let end = end_pin(db, end);
    let on_path = |resizer: &mut Resizer<'_, Sta>| {
        let netlist = resizer.netlist();
        let path = resizer
            .sta()
            .worst_slack_path(&netlist, end, MinMax::Max)
            .unwrap();
        let index = path
            .points
            .iter()
            .position(|p| resizer.db.pin_name(p.pin) == driver)
            .unwrap();
        test(resizer, &path, index);
    };
    match wires {
        Wires::Ideal => with_timed_session(db, lib, constraints, on_path),
        Wires::Metal2 => with_wired_session(db, lib, constraints, on_path),
    }
}

#[test]
fn loads_are_ranked_by_the_slack_they_have_beyond_the_driver_and_then_by_name() {
    let mut db = fork();

    on_site(
        &mut db,
        &cells(&clock_cells_source()),
        SLOW_CLOCK,
        ("f4/D", "g1/Z"),
        |resizer, path, index| {
            let site = site(resizer, path, index).unwrap();
            let driver_slack = resizer
                .sta()
                .slack(site.driver, MinMax::Max)
                .time()
                .unwrap();

            let ranked = loads_by_spare_slack(resizer, &site, driver_slack);

            let names: Vec<&str> = ranked.iter().map(|l| l.name.as_str()).collect();
            assert_eq!(names, ["f2/D", "f3/D", "h1/A"]);
            for load in &ranked {
                let slack = resizer
                    .sta()
                    .slack_edge(load.pin, site.edge, MinMax::Max)
                    .time()
                    .unwrap();
                assert!((load.spare - (slack - driver_slack)).abs() < Seconds::new(1e-15));
            }
            assert!(
                ranked[0].spare > Seconds::new(1e-12),
                "{:?}",
                ranked[0].spare
            );
        },
    );
}

#[test]
fn a_clone_is_placed_at_the_mean_of_the_driver_and_the_loads_it_takes_over() {
    let mut db = fork();

    on_site(
        &mut db,
        &cells(&clock_cells_source()),
        SLOW_CLOCK,
        ("f4/D", "g1/Z"),
        |resizer, path, index| {
            let site = site(resizer, path, index).unwrap();
            let loads = loads_by_spare_slack(resizer, &site, Seconds::ZERO);
            let taken = &loads[..2];

            let at = clone_location(resizer, site.driver, taken);

            let netlist = resizer.netlist();
            let places: Vec<Point<Dbu>> = [site.driver, taken[0].pin, taken[1].pin]
                .iter()
                .map(|&p| netlist.pin_location(p))
                .collect();
            assert_eq!(
                at.x.get(),
                places.iter().map(|p| p.x.get()).sum::<i64>() / 3
            );
            assert_eq!(
                at.y.get(),
                places.iter().map(|p| p.y.get()).sum::<i64>() / 3
            );
            assert_ne!(at, Point::new(Dbu::ZERO, Dbu::ZERO));
        },
    );
}

#[test]
fn only_a_gate_with_one_output_and_no_state_is_a_single_output_combinational_one() {
    let mut db = fork();
    let lib = cells(&clock_cells_source());

    with_timed_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let combinational = |name: &str| {
            let cell = resizer.db.cell_id(name).unwrap();
            let (lib, _) = resizer.link.cell(resizer.db, cell).unwrap();
            is_single_output_combinational(resizer, cell, lib)
        };

        assert!(combinational("g1"));
        assert!(!combinational("f1"));
    });
}

#[test]
fn a_gate_of_two_outputs_is_not_single_output() {
    let mut db = fork();
    let lib = cells_with(
        "cell (TWO) { area : 1.0 ;
           pin (A) { direction : input ; capacitance : 1.0 ; }
           pin (Y) { direction : output ; function : \"A\" ; }
           pin (Z) { direction : output ; function : \"!A\" ; } }",
    );

    with_timed_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let cell = resizer.db.cell_id("g1").unwrap();
        let two = resizer.link.liberty.cell_id("TWO").unwrap();

        assert!(!is_single_output_combinational(resizer, cell, two));
    });
}

/// A buffer `name` like `BUF_X1`, with a slew of `slew` nanoseconds at
/// 100 fF and `delay` nanoseconds at 100 fF and no input slew, which set
/// its drive resistance and its delay; a slow input adds 0.01 to the delay.
fn tuned_buffer(name: &str, slew: f64, delay: f64) -> String {
    let slow = format!("{:.2}", delay + 0.01);
    let blunt = format!("{:.2}", slew + 0.01);
    format!(
        "cell ({name}) {{ area : 0.8 ;
               pin (A) {{ direction : input ; capacitance : 0.97 ; }}
               pin (Z) {{ direction : output ; function : \"A\" ; max_capacitance : 60.0 ;
                 timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
                   cell_rise (delay_t) {{ values (\"0.03, {delay}\", \"0.04, {slow}\") ; }}
                   cell_fall (delay_t) {{ values (\"0.03, {delay}\", \"0.04, {slow}\") ; }}
                   rise_transition (delay_t) {{ values (\"0.01, {slew}\", \"0.02, {blunt}\") ; }}
                   fall_transition (delay_t) {{ values (\"0.01, {slew}\", \"0.02, {blunt}\") ; }} }} }} }}\n"
    )
}

/// The size `upsize_cell` picks for a `BUF_X1` driving 10 fF from an
/// ideal driver, among `BUF_X1`, the clock buffer and the `sizes`, each a
/// name, a slew and a delay at 100 fF in nanoseconds.
fn upsized_among(sizes: &[(&str, f64, f64)]) -> Option<String> {
    let copies: Vec<(&str, &str)> = sizes.iter().map(|&(n, _, _)| ("BUF_X1", n)).collect();
    let mut db = design_with_copies(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN sizes ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\nEND DESIGN\n",
        &copies,
    );
    let extra: String = sizes
        .iter()
        .map(|&(name, slew, delay)| tuned_buffer(name, slew, delay))
        .collect();
    let lib = cells_with(&extra);
    let mut picked = None;
    with_timed_session(&mut db, &lib, "", |resizer| {
        let buf = resizer.link.liberty.cell_id("BUF_X1").unwrap();
        let cell = resizer.link.liberty.cell(buf);
        let (input, output) = (cell.port("A").unwrap(), cell.port("Z").unwrap());
        let load = Farads::new(10e-15);

        let size = upsize_cell(resizer, buf, input, output, load, Ohms::ZERO);

        picked = size.map(|s| resizer.link.liberty.cell(s).name.clone());
    });
    picked
}

#[test]
fn the_size_up_is_the_weakest_that_drives_harder_and_is_faster() {
    assert_eq!(upsized_among(&[]), Some("CLKBUF_X3".to_string()));
}

#[test]
fn a_size_of_the_same_drive_is_not_a_size_up() {
    assert_eq!(
        upsized_among(&[("E", 0.40, 0.10)]),
        Some("CLKBUF_X3".to_string())
    );
}

#[test]
fn a_size_that_drives_harder_but_is_no_faster_is_not_a_size_up() {
    // Same delay table as BUF_X1, so the same delay at its input load.
    assert_eq!(
        upsized_among(&[("G", 0.30, 0.30)]),
        Some("CLKBUF_X3".to_string())
    );
    assert_eq!(
        upsized_among(&[("H", 0.30, 0.90)]),
        Some("CLKBUF_X3".to_string())
    );
}

#[test]
fn a_weaker_size_is_not_a_size_up_however_fast() {
    assert_eq!(
        upsized_among(&[("W", 0.60, 0.10)]),
        Some("CLKBUF_X3".to_string())
    );
}

/// A flop launching into buffer `g1` and flop `f5` alike; `g1` drives
/// inverter `i1`, which drives flop `f2`.
fn buffer_beside_a_flop() -> NetlistDB {
    let components = "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
        - g1 BUF_X1 + PLACED ( 40000 2800 ) N ;\n\
        - i1 INV_X1 + PLACED ( 80000 2800 ) N ;\n\
        - f2 DFF_X1 + PLACED ( 120000 2800 ) N ;\n\
        - f5 DFF_X1 + PLACED ( 40000 20000 ) N ;\n";
    let nets = "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ( f5 CK ) ;\n\
        - n0 ( f1 Q ) ( g1 A ) ( f5 D ) ;\n\
        - n1 ( g1 Z ) ( i1 A ) ;\n\
        - n2 ( i1 ZN ) ( f2 D ) ;\n";
    design(&def_text(
        "side",
        &DefSections {
            components,
            nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// Runs `test` on the removal of buffer `g1` of [`buffer_beside_a_flop`]
/// under a clock of `period` nanoseconds, with the slack the flop `f5`
/// beside it has.
fn on_buffer_removal(period: f64, test: impl FnOnce(&Resizer<'_, Sta>, &Removal, Seconds)) {
    let mut db = buffer_beside_a_flop();
    let f5_d = pin(&db, "f5/D");
    let clock = format!("create_clock -name clk -period {period} [get_ports clk]\n");
    on_site(
        &mut db,
        &cells(&clock_cells_source()),
        &clock,
        ("f2/D", "g1/Z"),
        |resizer, path, index| {
            let at = site(resizer, path, index).unwrap();
            let removal = Removal {
                buffer: at,
                input: at.input.unwrap(),
                prev_driver: at.prev_driver.unwrap(),
                prev_edge: at.prev_edge.unwrap(),
            };
            let side_slack = resizer.sta().slack(f5_d, MinMax::Max).time().unwrap();
            test(resizer, &removal, side_slack);
        },
    );
}

/// A change that slows the previous driver by `degrade`, leaving its slew
/// alone.
fn slowing_by(degrade: Seconds) -> DriverChange {
    let slew = EdgeValues::both(Seconds::new(0.02e-9));
    DriverChange {
        old_slew: slew,
        new_slew: slew,
        delay_degrade: degrade,
        delay_improve: Seconds::ZERO,
    }
}

#[test]
fn a_side_load_with_slack_to_spare_takes_the_later_arrival() {
    on_buffer_removal(1.0, |resizer, removal, slack| {
        assert!(slack > Seconds::new(0.1e-9));

        assert!(side_loads_ok(
            resizer,
            removal,
            &slowing_by(slack * 0.5),
            Seconds::ZERO
        ));
    });
}

#[test]
fn a_side_load_that_the_later_arrival_takes_below_zero_is_refused() {
    on_buffer_removal(1.0, |resizer, removal, slack| {
        assert!(!side_loads_ok(
            resizer,
            removal,
            &slowing_by(slack * 1.5),
            Seconds::ZERO
        ));
    });
}

#[test]
fn a_side_load_is_refused_when_the_margin_takes_it_below_zero() {
    on_buffer_removal(1.0, |resizer, removal, slack| {
        let no_delay = slowing_by(Seconds::ZERO);

        assert!(!side_loads_ok(resizer, removal, &no_delay, slack * 1.5));
        assert!(side_loads_ok(resizer, removal, &no_delay, slack * 0.5));
    });
}

#[test]
fn a_side_load_the_later_arrival_leaves_at_exactly_zero_is_kept() {
    on_buffer_removal(1.0, |resizer, removal, slack| {
        assert!(side_loads_ok(
            resizer,
            removal,
            &slowing_by(slack),
            Seconds::ZERO
        ));
    });
}

#[test]
fn a_side_load_already_failing_may_lose_a_tenth_of_its_violation_but_no_more() {
    on_buffer_removal(0.1, |resizer, removal, slack| {
        assert!(slack < Seconds::new(-0.01e-9), "{slack:?}");

        assert!(side_loads_ok(
            resizer,
            removal,
            &slowing_by(slack.abs() * 0.05),
            Seconds::ZERO
        ));
        assert!(!side_loads_ok(
            resizer,
            removal,
            &slowing_by(slack.abs() * 0.5),
            Seconds::ZERO
        ));
    });
}

#[test]
fn the_loads_of_a_buffer_taken_out_must_keep_their_outputs_slack() {
    on_buffer_removal(1.0, |resizer, removal, _| {
        let later = slowing_by(Seconds::new(0.5e-9));
        let earlier = DriverChange {
            delay_improve: Seconds::new(0.5e-9),
            ..slowing_by(Seconds::ZERO)
        };

        assert!(!buffer_loads_ok(resizer, removal, &later, Seconds::ZERO).unwrap());
        assert!(buffer_loads_ok(resizer, removal, &earlier, Seconds::ZERO).unwrap());
    });
}

/// Runs `test` on the inverter of [`inverted_pair`] with the slack its
/// output has under a clock of 1 ns.
fn with_inverter_slack(test: impl FnOnce(&Resizer<'_, Sta>, CellId, Seconds)) {
    let mut db = inverted_pair();
    let output = pin(&db, "i1/ZN");
    let constraints = "create_clock -name c -period 1 [get_ports clk]\n";
    with_timed_session(
        &mut db,
        &cells(&clock_cells_source()),
        constraints,
        |resizer| {
            let inverter = resizer.db.cell_id("i1").unwrap();
            let slack = resizer.sta().slack(output, MinMax::Max).time().unwrap();
            test(resizer, inverter, slack);
        },
    );
}

#[test]
fn the_margin_comes_off_the_slack_the_non_negative_rule_judges() {
    with_inverter_slack(|resizer, inverter, slack| {
        let impact = SlewImpact {
            margin: slack * 0.5,
            ..later_by(0.0, SlackRule::NonNegative)
        };
        let later = SlewImpact {
            adjust: slack * 0.75,
            ..impact
        };

        assert!(input_slew_impact_ok(resizer, inverter, &impact));
        assert!(!input_slew_impact_ok(resizer, inverter, &later));
    });
}

#[test]
fn an_arrival_that_leaves_exactly_no_slack_keeps_the_non_negative_rule() {
    with_inverter_slack(|resizer, inverter, slack| {
        let impact = SlewImpact {
            adjust: slack,
            ..later_by(0.0, SlackRule::NonNegative)
        };

        assert!(input_slew_impact_ok(resizer, inverter, &impact));
    });
}

#[test]
fn slower_input_slews_slow_the_output_and_break_the_no_worse_rule() {
    with_inverter_slack(|resizer, inverter, _| {
        let sharp = EdgeValues::both(Seconds::new(0.005e-9));
        let blunt = EdgeValues::both(Seconds::new(0.2e-9));
        let slowing = SlewImpact {
            old: sharp,
            new: blunt,
            ..later_by(0.0, SlackRule::NoWorse)
        };
        let speeding = SlewImpact {
            old: blunt,
            new: sharp,
            ..slowing
        };

        assert!(!input_slew_impact_ok(resizer, inverter, &slowing));
        assert!(input_slew_impact_ok(resizer, inverter, &speeding));
    });
}

fn micron_point(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(2000 * x), Dbu::new(2000 * y))
}

fn load_at(at: Point<Dbu>, pin: usize, cap_ff: f64) -> BnetPtr {
    let load = PinLoad {
        cap: Farads::new(cap_ff * 1e-15),
        fanout: 1.0,
        max_slew: Seconds::INFINITY,
    };
    Bnet::load(at, PinId::new(pin), load)
}

fn slew_of(slews: &[(PinId, Seconds)], pin: usize) -> Seconds {
    slews.iter().find(|(p, _)| *p == PinId::new(pin)).unwrap().1
}

#[test]
fn a_wire_degrades_the_slew_by_its_elmore_delay_times_the_slew_factor() {
    let mut db = fork();
    let lib = cells(&clock_cells_source());

    with_wired_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let rc = resizer.tree_rc();
        // 2 um and then 3 um of 3.5 ohm and 0.1 fF a micron.
        let far = load_at(micron_point(5, 0), 7, 2.0);
        let near = Bnet::wire(micron_point(2, 0), far, None, &rc);
        let tree = Bnet::wire(micron_point(0, 0), near, None, &rc);

        let slews = slews_in_tree(resizer, Seconds::new(20e-12), &tree);

        // The wire to the load: 10.5 ohm, 2 fF and half of 0.3 fF; the
        // wire before it: 7 ohm into that and half of its own 0.2 fF.
        let load_wire: f64 = 10.5 * (2.0e-15 + 0.15e-15);
        let outer_wire = 7.0 * (2.0e-15 + 0.3e-15 + 0.1e-15);
        let expected = (load_wire + outer_wire).mul_add(resizer.slew_rc_factor, 20e-12);
        assert_eq!(slews.len(), 1);
        assert!(
            close(slew_of(&slews, 7).get(), expected, 1e-9),
            "{slews:?} {expected}"
        );
    });
}

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * a.abs().max(b.abs())
}

#[test]
fn both_branches_of_a_junction_start_from_the_slew_at_it() {
    let mut db = fork();
    let lib = cells(&clock_cells_source());

    with_wired_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let rc = resizer.tree_rc();
        let left = Bnet::wire(
            micron_point(1, 0),
            load_at(micron_point(3, 0), 1, 1.0),
            None,
            &rc,
        );
        let right = load_at(micron_point(1, 4), 2, 1.0);
        let junction = Bnet::junction(micron_point(1, 0), left, right);

        let slews = slews_in_tree(resizer, Seconds::new(20e-12), &junction);

        let wire = 7.0 * (1.0e-15 + 0.1e-15);
        assert!(close(
            slew_of(&slews, 1).get(),
            20e-12 + wire * resizer.slew_rc_factor,
            1e-9
        ));
        assert_eq!(slew_of(&slews, 2), Seconds::new(20e-12));
    });
}

/// Global routes that have none to give.
struct NoRoutes;

impl RouteSource for NoRoutes {
    fn update(&mut self, _: &NetlistDB, _: &[NetId]) {}

    fn begin_trial(&mut self) {}

    fn keep_trial(&mut self) {}

    fn restore_trial(&mut self) {}

    fn route(&self, _: &NetlistDB, _: NetId) -> Option<NetRoute> {
        None
    }
}

#[test]
fn a_via_degrades_the_slew_by_its_resistance_into_the_capacitance_under_it() {
    let mut db = fork();
    let lib = cells(&clock_cells_source());
    let wires = metal2_wires(&db);
    let via = Ohms::new(20.0);
    let rc = RouteRc::new(&db.library, &[], &[(CutLayerId::new(1), via)], LayerId(0));

    with_session_inputs(&mut db, &lib, SLOW_CLOCK, wires, |inputs, sdc| {
        let mut source = NoRoutes;
        let routes = GlobalRouting {
            source: &mut source,
            rc,
            legalize: &|_, _| {},
        };
        let resizer = Resizer::on_global_routes(inputs, sdc, routes).unwrap();
        let tree_rc = resizer.tree_rc();
        let (below, above) = (LayerId::new(1), LayerId::new(2));
        let tree = Bnet::via(
            micron_point(0, 0),
            below,
            above,
            load_at(micron_point(0, 0), 4, 3.0),
            &tree_rc,
        );
        assert_eq!(tree_rc.via(below, above), via);

        let slews = slews_in_tree(&resizer, Seconds::new(20e-12), &tree);

        let expected = (20.0 * 3.0e-15_f64).mul_add(resizer.slew_rc_factor, 20e-12);
        assert!(
            close(slew_of(&slews, 4).get(), expected, 1e-9),
            "{slews:?} {expected}"
        );
    });
}

fn sorted_loads(tree: &Bnet) -> Vec<usize> {
    let mut loads: Vec<usize> = tree.loads().iter().map(|p| p.index()).collect();
    loads.sort_unstable();
    loads
}

#[test]
fn stitching_replaces_the_load_the_buffer_stood_at_with_what_the_buffer_drove() {
    let mut db = fork();
    let lib = cells(&clock_cells_source());

    with_wired_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let rc = resizer.tree_rc();
        let left = Bnet::wire(
            micron_point(0, 0),
            load_at(micron_point(4, 0), 10, 1.0),
            None,
            &rc,
        );
        let right = Bnet::wire(
            micron_point(0, 0),
            load_at(micron_point(0, 4), 30, 1.0),
            None,
            &rc,
        );
        let outer = Bnet::junction(micron_point(0, 0), left, right);
        let inner = load_at(micron_point(4, 0), 20, 1.0);

        let left_gone = stitch(resizer, &outer, PinId::new(10), &inner).unwrap();
        let right_gone = stitch(resizer, &outer, PinId::new(30), &inner).unwrap();

        assert_eq!(sorted_loads(&left_gone), [20, 30]);
        assert_eq!(sorted_loads(&right_gone), [10, 20]);
        assert!(stitch(resizer, &outer, PinId::new(99), &inner).is_none());
    });
}

#[test]
fn stitching_finds_the_load_below_a_wire_and_a_via() {
    let mut db = fork();
    let lib = cells(&clock_cells_source());

    with_wired_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let rc = resizer.tree_rc();
        let load = load_at(micron_point(4, 0), 10, 1.0);
        let via = Bnet::via(
            micron_point(4, 0),
            LayerId::new(1),
            LayerId::new(2),
            load,
            &rc,
        );
        let outer = Bnet::wire(micron_point(0, 0), via, None, &rc);
        let inner = load_at(micron_point(4, 0), 20, 1.0);

        let stitched = stitch(resizer, &outer, PinId::new(10), &inner).unwrap();

        assert_eq!(sorted_loads(&stitched), [20]);
        assert!(matches!(stitched.kind, Kind::Wire { .. }));
    });
}

#[test]
fn a_load_that_is_not_the_one_asked_for_is_not_replaced() {
    let mut db = fork();
    let lib = cells(&clock_cells_source());

    with_wired_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let outer = load_at(micron_point(4, 0), 10, 1.0);
        let inner = load_at(micron_point(4, 0), 20, 1.0);

        assert!(stitch(resizer, &outer, PinId::new(11), &inner).is_none());
        assert_eq!(
            sorted_loads(&stitch(resizer, &outer, PinId::new(10), &inner).unwrap()),
            [20]
        );
    });
}

#[test]
fn what_the_buffer_drove_is_joined_by_a_wire_only_where_it_started_elsewhere() {
    let mut db = fork();
    let lib = cells(&clock_cells_source());

    with_wired_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let rc = resizer.tree_rc();
        let at_the_load = load_at(micron_point(4, 0), 20, 1.0);
        let elsewhere = Bnet::wire(
            micron_point(4, 3),
            load_at(micron_point(8, 3), 20, 1.0),
            None,
            &rc,
        );
        let outer = load_at(micron_point(4, 0), 10, 1.0);

        let same_place = stitch(resizer, &outer, PinId::new(10), &at_the_load).unwrap();
        let apart = stitch(resizer, &outer, PinId::new(10), &elsewhere).unwrap();

        assert!(Rc::ptr_eq(&same_place, &at_the_load));
        assert_eq!(apart.at, micron_point(4, 0));
        assert_eq!(apart.length(), Dbu::new(2000 * 3));
        assert!(matches!(&apart.kind, Kind::Wire { to, .. } if Rc::ptr_eq(to, &elsewhere)));
    });
}

#[test]
fn what_the_buffer_drove_on_a_routing_layer_is_hung_as_it_is() {
    let mut db = fork();
    let lib = cells(&clock_cells_source());

    with_wired_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let rc = resizer.tree_rc();
        let routed = Bnet::wire(
            micron_point(4, 3),
            load_at(micron_point(8, 3), 20, 1.0),
            Some(LayerId::new(1)),
            &rc,
        );
        let outer = load_at(micron_point(4, 0), 10, 1.0);

        let stitched = stitch(resizer, &outer, PinId::new(10), &routed).unwrap();

        assert!(Rc::ptr_eq(&stitched, &routed));
    });
}

#[test]
fn the_slews_after_a_removal_are_scaled_by_how_far_the_estimate_is_from_timing() {
    let mut db = buffer_beside_a_flop();
    let lib = cells(&clock_cells_source());
    let clock = "create_clock -name clk -period 1 [get_ports clk]\n";
    let (q, buffer_in, buffer_out) = (pin(&db, "f1/Q"), pin(&db, "g1/A"), pin(&db, "g1/Z"));
    let (side, load) = (pin(&db, "f5/D"), pin(&db, "i1/A"));

    with_wired_session(&mut db, &lib, clock, |resizer| {
        let new_slew = Seconds::new(30e-12);

        let slews = slews_after_removal(resizer, q, buffer_out, buffer_in, new_slew)
            .unwrap()
            .unwrap();

        let tree1 = resizer.buffered_tree(q).unwrap().unwrap();
        let tree2 = resizer.buffered_tree(buffer_out).unwrap().unwrap();
        let stitched = stitch(resizer, &tree1, buffer_in, &tree2).unwrap();
        let estimate = |slew: Seconds, tree: &Bnet, pin: PinId| {
            slews_in_tree(resizer, slew, tree)
                .iter()
                .find(|(p, _)| *p == pin)
                .unwrap()
                .1
        };
        let up = estimate(resizer.graph.slew(q).max(), &tree1, buffer_in);
        let up_factor = resizer.graph.slew(buffer_in).max() / up;
        let down = estimate(resizer.graph.slew(buffer_out).max(), &tree2, load);
        let down_factor = resizer.graph.slew(load).max() / down;
        assert!((up_factor - 1.0).abs() > 1e-3 && (down_factor - 1.0).abs() > 1e-3);
        assert!(up_factor.mul_add(down_factor, -1.0).abs() > 1e-3);

        let found = |pin: PinId| slews.iter().find(|(p, _)| *p == pin).unwrap().1;
        assert_eq!(slews.len(), 2);
        assert!(close(
            found(side).get(),
            estimate(new_slew, &stitched, side).get(),
            1e-12
        ));
        let expected = estimate(new_slew, &stitched, load) * (up_factor * down_factor);
        assert!(
            close(found(load).get(), expected.get(), 1e-9),
            "{:?} {expected:?}",
            found(load)
        );
    });
}

/// What `try_move` of `kind` makes of the driver `driver` on the worst
/// path into `end` of `db`, and what `check` finds of the session after.
fn tried<R>(
    db: &mut NetlistDB,
    lib: &Liberty,
    constraints: &str,
    pins: (&str, &str),
    kind: MoveKind,
    check: impl FnOnce(&mut Resizer<'_, Sta>, bool) -> R,
) -> R {
    tried_with(Wires::Metal2, db, lib, constraints, pins, kind, check)
}

/// [`tried`] with `wires`.
fn tried_with<R>(
    wires: Wires,
    db: &mut NetlistDB,
    lib: &Liberty,
    constraints: &str,
    (end, driver): (&str, &str),
    kind: MoveKind,
    check: impl FnOnce(&mut Resizer<'_, Sta>, bool) -> R,
) -> R {
    let mut outcome = None;
    let mut check = Some(check);
    on_site_with(
        wires,
        db,
        lib,
        constraints,
        (end, driver),
        |resizer, path, index| {
            let mut moves = Moves::new(resizer).unwrap();
            let moved = moves
                .try_move(resizer, kind, path, index, Seconds::ZERO)
                .unwrap();
            outcome = Some((check.take().unwrap())(resizer, moved));
        },
    );
    outcome.unwrap()
}

fn master_of(resizer: &Resizer<'_, Sta>, cell: &str) -> String {
    let cell = resizer.db.cell_id(cell).unwrap();
    resizer.link.cell(resizer.db, cell).unwrap().1.name.clone()
}

/// A flop launching through `first` and then `second` buffer cells, with
/// `extra` flops loading the first's output, into another flop.
fn two_buffers(first: &str, second: &str, extra: usize) -> NetlistDB {
    let mut components = format!(
        "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n- c1 {first} + PLACED ( 82000 2800 ) N ;\n\
         - b1 {second} + PLACED ( 162000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 322000 2800 ) N ;\n"
    );
    let mut clock = String::from("- clk ( PIN clk ) ( f1 CK ) ( f2 CK )");
    let mut side = String::new();
    for i in 0..extra {
        writeln!(
            components,
            "- x{i} DFF_X1 + PLACED ( 162000 {} ) N ;",
            20_000 + 20_000 * i
        )
        .unwrap();
        write!(clock, " ( x{i} CK )").unwrap();
        write!(side, " ( x{i} D )").unwrap();
    }
    let nets = format!(
        "{clock} ;\n- n0 ( f1 Q ) ( c1 A ) ;\n- n1 ( c1 Z ) ( b1 A ){side} ;\n- n2 ( b1 Z ) ( f2 D ) ;\n"
    );
    design(&def_text(
        "two",
        &DefSections {
            components: &components,
            nets: &nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

const TIGHT_CLOCK: &str = "create_clock -name clk -period 0.2 [get_ports clk]\n";

#[test]
fn a_buffer_behind_a_lone_stronger_buffer_takes_its_size() {
    let mut db = two_buffers("CLKBUF_X3", "BUF_X1", 0);
    let lib = cells(&clock_cells_source());

    let (moved, master) = tried(
        &mut db,
        &lib,
        TIGHT_CLOCK,
        ("f2/D", "b1/Z"),
        MoveKind::SizeUpMatch,
        |resizer, moved| (moved, master_of(resizer, "b1")),
    );

    assert!(moved);
    assert_eq!(master, "CLKBUF_X3");
}

/// Whether `try_move` of `SizeUpMatch` on the second buffer of
/// [`two_buffers`] changes anything, and what it is then.
fn matched_buffer(first: &str, second: &str, extra: usize, constraints: &str) -> (bool, String) {
    let mut db = two_buffers(first, second, extra);
    let lib = cells(&clock_cells_source());

    tried(
        &mut db,
        &lib,
        constraints,
        ("f2/D", "b1/Z"),
        MoveKind::SizeUpMatch,
        |resizer, moved| (moved, master_of(resizer, "b1")),
    )
}

#[test]
fn a_buffer_behind_a_driver_with_other_loads_keeps_its_size() {
    assert_eq!(
        matched_buffer("CLKBUF_X3", "BUF_X1", 1, TIGHT_CLOCK),
        (false, "BUF_X1".to_string())
    );
}

#[test]
fn a_buffer_stronger_than_its_driver_keeps_its_size() {
    assert_eq!(
        matched_buffer("BUF_X1", "CLKBUF_X3", 0, TIGHT_CLOCK),
        (false, "CLKBUF_X3".to_string())
    );
}

#[test]
fn a_buffer_the_size_of_its_driver_keeps_its_size() {
    assert_eq!(
        matched_buffer("BUF_X1", "BUF_X1", 0, TIGHT_CLOCK),
        (false, "BUF_X1".to_string())
    );
}

#[test]
fn a_buffer_is_not_sized_up_to_a_size_that_breaks_its_driver_s_load_limit() {
    let constraints = format!("{TIGHT_CLOCK}set_max_capacitance 1.0 [current_design]\n");

    assert_eq!(
        matched_buffer("CLKBUF_X3", "BUF_X1", 0, &constraints),
        (false, "BUF_X1".to_string())
    );
}

/// An AND gate `name` whose `A` arc delays `slow` and `B` arc `fast`
/// nanoseconds, kept from swapping where `dont_touch`.
fn and_with_arcs(name: &str, slow: f64, fast: f64, dont_touch: bool) -> String {
    let arc = |pin: &str, delay: f64| {
        format!(
            "timing () {{ related_pin : \"{pin}\" ; timing_sense : positive_unate ;
               cell_rise (scalar) {{ values (\"{delay}\") ; }}
               cell_fall (scalar) {{ values (\"{delay}\") ; }}
               rise_transition (scalar) {{ values (\"0.02\") ; }}
               fall_transition (scalar) {{ values (\"0.02\") ; }} }}"
        )
    };
    format!(
        "cell ({name}) {{ area : 0.8 ; dont_touch : {dont_touch} ;
           pin (A) {{ direction : input ; capacitance : 0.97 ; }}
           pin (B) {{ direction : input ; capacitance : 0.97 ; }}
           pin (Z) {{ direction : output ; function : \"(A & B)\" ; max_capacitance : 60.0 ;
             {} {} }} }}\n",
        arc("A", slow),
        arc("B", fast)
    )
}

/// Flop `f1` into the slow pin `A` of gate `g1` and flop `f3` into its
/// fast pin `B`, `g1` into flop `f2`.
fn gate_with_a_slow_pin(master: &str) -> NetlistDB {
    let components = format!(
        "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n- f3 DFF_X1 + PLACED ( 2000 20000 ) N ;\n\
         - g1 {master} + PLACED ( 82000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 162000 2800 ) N ;\n"
    );
    let nets = "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ( f3 CK ) ;\n\
        - n0 ( f1 Q ) ( g1 A ) ;\n- n1 ( f3 Q ) ( g1 B ) ;\n- n2 ( g1 Z ) ( f2 D ) ;\n";
    design_with_copies(
        &def_text(
            "swap",
            &DefSections {
                components: &components,
                nets,
                inputs: &["clk"],
                outputs: &[],
            },
        ),
        &[("AND2_X1", master)],
    )
}

/// Tries `SwapPins` twice on gate `g1` of [`gate_with_a_slow_pin`], and
/// says whether each swapped and which of the inputs the flop `f1` ends on.
fn swapped_gate(dont_touch: bool) -> (bool, bool, String) {
    let mut db = gate_with_a_slow_pin("ANDS");
    let lib = cells_with(&and_with_arcs("ANDS", 0.30, 0.05, dont_touch));
    let b = pin(&db, "g1/B");
    let f1_net = db.pins[pin(&db, "f1/Q")].net;

    let (first, second, f1_on_b) = {
        let mut ran = None;
        on_site(
            &mut db,
            &lib,
            TIGHT_CLOCK,
            ("f2/D", "g1/Z"),
            |resizer, path, index| {
                let mut moves = Moves::new(resizer).unwrap();
                let first = moves
                    .try_move(resizer, MoveKind::SwapPins, path, index, Seconds::ZERO)
                    .unwrap();
                let second = moves
                    .try_move(resizer, MoveKind::SwapPins, path, index, Seconds::ZERO)
                    .unwrap();
                ran = Some((first, second, resizer.db.pins[b].net == f1_net));
            },
        );
        ran.unwrap()
    };
    (first, second, if f1_on_b { "B" } else { "A" }.to_string())
}

#[test]
fn the_critical_input_moves_to_a_faster_equivalent_pin_once() {
    assert_eq!(swapped_gate(false), (true, false, "B".to_string()));
}

#[test]
fn a_gate_the_library_protects_keeps_its_pins() {
    assert_eq!(swapped_gate(true), (false, false, "A".to_string()));
}

/// Gate `g1` of `master` fed by flops `f1` and `f3`, driving `count`
/// flops `x0`, `x1`, ... down a column.
fn gate_with_loads(master: &str, count: usize) -> NetlistDB {
    let mut components = format!(
        "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n- f3 DFF_X1 + PLACED ( 2000 20000 ) N ;\n\
         - g1 {master} + PLACED ( 82000 2800 ) N ;\n"
    );
    let mut clock = String::from("- clk ( PIN clk ) ( f1 CK ) ( f3 CK )");
    let mut fanout = String::from("- n2 ( g1 Z )");
    for i in 0..count {
        writeln!(
            components,
            "- x{i} DFF_X1 + PLACED ( 162000 {} ) N ;",
            2800 + 20_000 * i
        )
        .unwrap();
        write!(clock, " ( x{i} CK )").unwrap();
        write!(fanout, " ( x{i} D )").unwrap();
    }
    let nets =
        format!("{clock} ;\n- n0 ( f1 Q ) ( g1 A ) ;\n- n1 ( f3 Q ) ( g1 B ) ;\n{fanout} ;\n");
    design(&def_text(
        "loads",
        &DefSections {
            components: &components,
            nets: &nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// Flop `f1` driving `count` flops `x0`, `x1`, ... down a column.
fn flop_with_loads(count: usize) -> NetlistDB {
    let mut components = String::from("- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n");
    let mut clock = String::from("- clk ( PIN clk ) ( f1 CK )");
    let mut fanout = String::from("- n0 ( f1 Q )");
    for i in 0..count {
        writeln!(
            components,
            "- x{i} DFF_X1 + PLACED ( 162000 {} ) N ;",
            2800 + 20_000 * i
        )
        .unwrap();
        write!(clock, " ( x{i} CK )").unwrap();
        write!(fanout, " ( x{i} D )").unwrap();
    }
    let nets = format!("{clock} ;\n{fanout} ;\n");
    design(&def_text(
        "flop",
        &DefSections {
            components: &components,
            nets: &nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// The pin of `cell`'s port `port`.
fn pin_of(resizer: &Resizer<'_, Sta>, cell: CellId, port: &str) -> PinId {
    resizer.db.cells[cell]
        .pins
        .iter()
        .copied()
        .find(|&p| resizer.db.pin_name(p).ends_with(&format!("/{port}")))
        .unwrap()
}

#[test]
fn a_gate_of_many_loads_is_cloned_to_take_half_of_them() {
    let mut db = gate_with_loads("AND2_X1", 10);
    let lib = cells(&clock_cells_source());
    let (z, a, b) = (pin(&db, "g1/Z"), pin(&db, "g1/A"), pin(&db, "g1/B"));
    let (q, other_q) = (pin(&db, "f1/Q"), pin(&db, "f3/Q"));
    let cells_before = db.cells.len();

    on_site(
        &mut db,
        &lib,
        TIGHT_CLOCK,
        ("x0/D", "g1/Z"),
        |resizer, path, index| {
            let load_before = resizer.load_cap(q);
            let mut moves = Moves::new(resizer).unwrap();

            let moved = moves
                .try_move(resizer, MoveKind::Clone, path, index, Seconds::ZERO)
                .unwrap();

            let clone = CellId::new(cells_before);
            let (clone_a, clone_b, clone_z) = (
                pin_of(resizer, clone, "A"),
                pin_of(resizer, clone, "B"),
                pin_of(resizer, clone, "Z"),
            );
            assert!(moved);
            assert_eq!(resizer.db.cells.len(), cells_before + 1);
            assert_eq!((resizer.fanout(z), resizer.fanout(clone_z)), (5, 5));
            assert_eq!(resizer.db.pins[clone_a].net, resizer.db.pins[a].net);
            assert_eq!(resizer.db.pins[clone_b].net, resizer.db.pins[b].net);
            assert_eq!((resizer.fanout(q), resizer.fanout(other_q)), (2, 2));
            // The clone's input adds its capacitance to the net's driver, 0.97 fF.
            assert!(resizer.load_cap(q) - load_before >= Farads::new(0.97e-15));
        },
    );
}

#[test]
fn a_gate_of_eight_loads_is_left_alone() {
    let mut db = gate_with_loads("AND2_X1", 8);
    let lib = cells(&clock_cells_source());
    let cells_before = db.cells.len();

    tried(
        &mut db,
        &lib,
        TIGHT_CLOCK,
        ("x0/D", "g1/Z"),
        MoveKind::Clone,
        |resizer, moved| {
            assert!(!moved);
            assert_eq!(resizer.db.cells.len(), cells_before);
        },
    );
}

#[test]
fn a_flop_of_many_loads_is_not_cloned() {
    let mut db = flop_with_loads(10);
    let lib = cells(&clock_cells_source());
    let cells_before = db.cells.len();

    tried(
        &mut db,
        &lib,
        TIGHT_CLOCK,
        ("x0/D", "f1/Q"),
        MoveKind::Clone,
        |resizer, moved| {
            assert!(!moved);
            assert_eq!(resizer.db.cells.len(), cells_before);
        },
    );
}

/// Flop `f1` driving `count` flops `x0`, `x1`, ... `x` database units
/// along, spread down the die 30 000 units apart.
fn flop_with_spread_loads(count: usize, x: usize) -> NetlistDB {
    let mut components = String::from("- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n");
    let mut clock = String::from("- clk ( PIN clk ) ( f1 CK )");
    let mut fanout = String::from("- n0 ( f1 Q )");
    for i in 0..count {
        writeln!(
            components,
            "- x{i} DFF_X1 + PLACED ( {x} {} ) N ;",
            2800 + 30_000 * i % 700_000
        )
        .unwrap();
        write!(clock, " ( x{i} CK )").unwrap();
        write!(fanout, " ( x{i} D )").unwrap();
    }
    let nets = format!("{clock} ;\n{fanout} ;\n");
    design(&def_text(
        "spread",
        &DefSections {
            components: &components,
            nets: &nets,
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// What `Buffer` makes of flop `f1` driving `count` flops `x` database
/// units away: whether it moved, and the cells added.
fn buffered_flop(count: usize, x: usize) -> (bool, usize) {
    let mut db = flop_with_spread_loads(count, x);
    let lib = cells(&clock_cells_source());
    let before = db.cells.len();
    let moved = tried(
        &mut db,
        &lib,
        TIGHT_CLOCK,
        ("x0/D", "f1/Q"),
        MoveKind::Buffer,
        |_, moved| moved,
    );
    (moved, db.cells.len() - before)
}

#[test]
fn a_net_of_two_far_loads_is_rebuffered() {
    let (moved, added) = buffered_flop(2, 700_000);

    assert!(moved);
    assert!(added > 0);
}

#[test]
fn a_net_of_one_load_is_not_rebuffered_however_far_it_is() {
    assert_eq!(buffered_flop(1, 700_000), (false, 0));
}

#[test]
fn a_net_of_twenty_loads_is_not_rebuffered() {
    assert_eq!(buffered_flop(20, 400_000), (false, 0));
}

/// What the buffer `b1` of [`buffer_to_remove`] drives.
#[derive(Clone, Copy)]
enum BufferLoads {
    /// This many flops.
    Flops(usize),
    /// The design's output port `out`.
    Port,
}

/// Flop `f1` driving buffer `b1` and `siblings` flops `p0`, `p1`, ...;
/// the buffer drives `loads`.
fn buffer_to_remove(siblings: usize, loads: BufferLoads) -> NetlistDB {
    let mut components = String::from(
        "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n- b1 BUF_X1 + PLACED ( 82000 2800 ) N ;\n",
    );
    let mut clock = String::from("- clk ( PIN clk ) ( f1 CK )");
    let mut upstream = String::from("- n0 ( f1 Q ) ( b1 A )");
    for i in 0..siblings {
        writeln!(
            components,
            "- p{i} DFF_X1 + PLACED ( 82000 {} ) N ;",
            20_000 + 20_000 * i
        )
        .unwrap();
        write!(clock, " ( p{i} CK )").unwrap();
        write!(upstream, " ( p{i} D )").unwrap();
    }
    let mut downstream = String::from("- n1 ( b1 Z )");
    let outputs: &[&str] = match loads {
        BufferLoads::Flops(count) => {
            for i in 0..count {
                writeln!(
                    components,
                    "- s{i} DFF_X1 + PLACED ( 162000 {} ) N ;",
                    2800 + 20_000 * i
                )
                .unwrap();
                write!(clock, " ( s{i} CK )").unwrap();
                write!(downstream, " ( s{i} D )").unwrap();
            }
            &[]
        }
        BufferLoads::Port => {
            downstream = String::from("- out ( b1 Z )");
            &["out"]
        }
    };
    let nets = format!("{clock} ;\n{upstream} ;\n{downstream} ;\n");
    design(&def_text(
        "unbuffer",
        &DefSections {
            components: &components,
            nets: &nets,
            inputs: &["clk"],
            outputs,
        },
    ))
}

/// Whether `Unbuffer` takes `b1` out of `db` under `constraints`, the end
/// of the path being `end`.
fn unbuffered(db: &mut NetlistDB, constraints: &str, end: &str) -> bool {
    unbuffered_over(db, &cells(&clock_cells_source()), constraints, end)
}

/// [`unbuffered`] over the cells of `lib`, with ideal wires.
fn unbuffered_over(db: &mut NetlistDB, lib: &Liberty, constraints: &str, end: &str) -> bool {
    let pins = (end, "b1/Z");
    tried_with(
        Wires::Ideal,
        db,
        lib,
        constraints,
        pins,
        MoveKind::Unbuffer,
        |_, moved| moved,
    )
}

const ONE_NS: &str = "create_clock -name clk -period 1 [get_ports clk]\n";

/// Whether `Unbuffer` takes `b1` out of a design where it and `siblings`
/// flops load `f1` and it drives `loads` flops, under a clock of 1 ns and
/// `more`, over a library whose pins have a fanout load of 1.
fn unbuffered_among_flops(siblings: usize, loads: usize, more: &str) -> bool {
    let mut db = buffer_to_remove(siblings, BufferLoads::Flops(loads));
    let source = clock_cells_source().replacen(
        "default_max_transition : 0.2 ;",
        "default_max_transition : 0.2 ; default_fanout_load : 1.0 ;",
        1,
    );

    unbuffered_over(&mut db, &cells(&source), &format!("{ONE_NS}{more}"), "s0/D")
}

#[test]
fn a_buffer_is_taken_out_while_its_driver_stays_within_ten_loads() {
    assert!(unbuffered_among_flops(0, 1, ""));
    assert!(unbuffered_among_flops(5, 5, ""));
}

#[test]
fn a_buffer_is_kept_where_taking_it_out_gives_its_driver_eleven_loads() {
    assert!(!unbuffered_among_flops(6, 5, ""));
    assert!(!unbuffered_among_flops(5, 6, ""));
}

#[test]
fn a_set_fanout_limit_replaces_the_default_of_ten_for_the_buffer_s_driver() {
    let limit = |max: usize| format!("set_max_fanout {max} [current_design]\n");

    assert!(unbuffered_among_flops(5, 5, &limit(10)));
    assert!(!unbuffered_among_flops(5, 5, &limit(9)));
    assert!(unbuffered_among_flops(6, 5, &limit(11)));
    assert!(!unbuffered_among_flops(6, 5, &limit(10)));
}

const PORT_LOAD: &str =
    "set_load 0.5 [get_ports out]\nset_output_delay 0.1 -clock clk [get_ports out]\n";

/// Whether `Unbuffer` takes `b1` out of a design where it drives the port
/// `out` and `siblings` flops load `f1` with it, under a clock of 1 ns, a
/// port load of 0.5 fF and `more`.
fn unbuffered_before_a_port(siblings: usize, more: &str) -> bool {
    let mut db = buffer_to_remove(siblings, BufferLoads::Port);

    unbuffered(&mut db, &format!("{ONE_NS}{PORT_LOAD}{more}"), "out")
}

/// The loads of `f1` and of `b1` with the buffer in, and the capacitance
/// of the buffer's input.
fn loads_beside_a_port(siblings: usize) -> (Farads, Farads, Farads) {
    let mut db = buffer_to_remove(siblings, BufferLoads::Port);
    let lib = cells(&clock_cells_source());
    let (q, z) = (pin(&db, "f1/Q"), pin(&db, "b1/Z"));
    let mut loads = None;
    with_timed_session(&mut db, &lib, &format!("{ONE_NS}{PORT_LOAD}"), |resizer| {
        let buffer = resizer.link.liberty.cell_id("BUF_X1").unwrap();
        let cell = resizer.link.liberty.cell(buffer);
        let input = cell.ports[cell.port("A").unwrap()].capacitance();
        loads = Some((resizer.load_cap(q), resizer.load_cap(z), input));
    });
    loads.unwrap()
}

#[test]
fn a_buffer_stays_when_its_driver_would_end_over_its_load_limit() {
    let (driver, buffer, input) = loads_beside_a_port(0);
    let after = driver + buffer - input;

    let at_the_limit = |limit: Farads| {
        format!(
            "set_max_capacitance {} [current_design]\n",
            limit.get() * 1e15
        )
    };

    assert!(unbuffered_before_a_port(
        0,
        &at_the_limit(after + input * 0.5)
    ));
    assert!(!unbuffered_before_a_port(
        0,
        &at_the_limit(after - input * 0.5)
    ));
}

#[test]
fn a_buffer_stays_when_its_driver_already_over_its_limit_would_stay_over_it() {
    let (driver, _, _) = loads_beside_a_port(2);
    let limit = format!(
        "set_max_capacitance {} [current_design]\n",
        driver.get() * 1e15 * 0.5
    );

    assert!(unbuffered_before_a_port(2, ""));
    assert!(!unbuffered_before_a_port(2, &limit));
}

#[test]
fn a_load_limit_of_zero_limits_nothing() {
    assert!(unbuffered_before_a_port(
        2,
        "set_max_capacitance 0.0 [current_design]\n"
    ));
}

/// How much later the previous driver of buffer `b1` of `db` settles when
/// the buffer's load is added to it, by its own library's tables, and the
/// slack flop `p0` beside the buffer has under a clock of 1 ns.
fn slowing_beside_a_loaded_port(db: &mut NetlistDB, constraints: &str) -> (Seconds, Seconds) {
    let lib = cells(&clock_cells_source());
    let p0_d = pin(db, "p0/D");
    let mut found = None;
    on_site_with(
        Wires::Ideal,
        db,
        &lib,
        constraints,
        ("out", "b1/Z"),
        |resizer, path, index| {
            let at = site(resizer, path, index).unwrap();
            let prev = at.prev_driver.unwrap();
            let (prev_lib, prev_port) = resizer.link.pin_port(resizer.db, prev).unwrap();
            let PinOwner::Cell { cell, .. } = resizer.db.pins[prev].owner else {
                panic!("a flop drives the buffer");
            };
            let buffer = resizer.link.liberty.buffer_cell(at.lib).unwrap();
            let in_cap = resizer.link.liberty.cell(at.lib).ports[buffer.input].capacitance();
            let old_load = resizer.load_cap(prev);
            let new_load = old_load + resizer.load_cap(at.driver) - in_cap;
            let slews = resizer.input_slews(cell);
            let delay = |load: Farads| {
                let flop = resizer.link.liberty.cell(prev_lib);
                gate_delays(flop, prev_port, load, &resizer.targets.slews, Some(&slews))
                    .0
                    .get(at.prev_edge.unwrap())
            };
            let slack = resizer.sta().slack(p0_d, MinMax::Max).time().unwrap();
            found = Some((delay(new_load) - delay(old_load), slack));
        },
    );
    found.unwrap()
}

/// Whether `Unbuffer` takes `b1` out when the flop `p0` beside it has the
/// slack the removal costs it, `extra` more.
fn unbuffered_with_slack_to_spare(extra: Seconds) -> bool {
    let loaded = "set_load 30 [get_ports out]\nset_output_delay 0.1 -clock clk [get_ports out]\n";
    let (slowing, slack) = {
        let mut db = buffer_to_remove(1, BufferLoads::Port);
        slowing_beside_a_loaded_port(&mut db, &format!("{ONE_NS}{loaded}"))
    };
    assert!(slowing > Seconds::new(20e-12), "{slowing:?}");
    let period = Seconds::new(1e-9) - slack + slowing + extra;
    let clock = format!(
        "create_clock -name clk -period {} [get_ports clk]\n",
        period.get() * 1e9
    );
    let mut db = buffer_to_remove(1, BufferLoads::Port);

    unbuffered(&mut db, &format!("{clock}{loaded}"), "out")
}

#[test]
fn a_buffer_is_taken_out_when_the_slack_beside_it_covers_the_slowing_of_its_driver() {
    assert!(unbuffered_with_slack_to_spare(Seconds::new(2e-12)));
}

#[test]
fn a_buffer_is_kept_when_the_slack_beside_it_falls_short_of_the_slowing_of_its_driver() {
    assert!(!unbuffered_with_slack_to_spare(Seconds::new(-2e-12)));
}
