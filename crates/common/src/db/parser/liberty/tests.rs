use super::*;
use crate::db::liberty::LibCellId;
use crate::db::library::DuplicateName;
use crate::units::{Farads, Seconds, Watts};

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
    assert!((a.capacitance.rise.get() - 0.9e-15).abs() < 1e-30);
    assert!((a.capacitance.fall.get() - 0.8e-15).abs() < 1e-30);
    let z = &buf.ports[buf.port("Z").unwrap()];
    assert!((z.max_capacitance.unwrap().get() - 60e-15).abs() < 1e-28);
    assert!((lib.defaults.max_transition.unwrap().get() - 0.5e-9).abs() < 1e-22);
    assert!((lib.thresholds.slew_lower.rise - 0.3).abs() < 1e-12);
}

/// A one-cell library whose header adds `header` and whose cell `c`
/// has `body`.
fn leaky(header: &str, body: &str) -> Result<Liberty, ParseError> {
    let src = format!(
        r#"library (x) {{ time_unit : "1ns"; capacitive_load_unit (1,ff); {header}
  cell (c) {{ {body} pin (A) {{ direction : input; capacitance : 1.0; }} }} }}"#
    );
    parse_str(&src, Path::new("x.lib"))
}

#[test]
fn leakage_scales_by_the_leakage_power_unit() {
    let lib = leaky(
        r#"leakage_power_unit : "1nW";"#,
        r#"cell_leakage_power : 21.5;
               leakage_power () { when : "!A"; value : 20.0; }
               leakage_power () { when : "A"; value : 23.0; }"#,
    )
    .unwrap();

    let cell = &lib.cells[LibCellId(0)];
    let got: Vec<f64> = cell.state_leakages.iter().map(|w| w.get()).collect();

    assert!((cell.leakage.unwrap().get() - 21.5e-9).abs() < 1e-20);
    assert!((got[0] - 20.0e-9).abs() < 1e-20 && (got[1] - 23.0e-9).abs() < 1e-20);
}

#[test]
fn leakage_without_a_unit_is_in_watts_and_a_valueless_state_is_passed_over() {
    let lib = leaky("", r#"leakage_power () { when : "A"; }"#).unwrap();

    let cell = &lib.cells[LibCellId(0)];

    assert_eq!(cell.leakage, None);
    assert_eq!(cell.state_leakages, Vec::<Watts>::new());
    assert_eq!(lib.units.leakage_power, Watts::new(1.0));
}

#[test]
fn an_unknown_leakage_power_unit_is_refused() {
    let err = leaky(r#"leakage_power_unit : "1nJ";"#, "").unwrap_err();

    assert!(
        err.to_string().contains("unknown leakage power unit"),
        "{err}"
    );
}

#[test]
fn tables_take_their_own_indices_over_the_templates() {
    let lib = tiny();
    let buf = lib.cell(lib.cell_id("BUF_X1").unwrap());
    let set = &buf.arc_sets[crate::db::liberty::ArcSetId::new(0)];
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
    assert!(
        (delay.lookup(Seconds::new(0.1e-9), Farads::new(2e-15)).get() - 0.020e-9).abs() < 1e-20
    );
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
    assert!(
        (setup.arcs[0]
            .check_value(Seconds::new(0.1e-9), Seconds::new(0.2e-9))
            .get()
            - 2e-9)
            .abs()
            < 1e-20
    );
    assert!((lib.units.time.get() - 1e-9).abs() < 1e-24);
    assert!((lib.units.capacitance.get() - 1e-15).abs() < 1e-30);
    let q = dff
        .arc_sets
        .iter()
        .find(|s| s.role == ArcRole::RegClkToQ)
        .unwrap();
    assert!(q.arcs.iter().all(|a| a.from_edge == Edge::Rise));
    assert_eq!(q.arcs.len(), 2);
}

/// The tiny library with `old` replaced by `new` once.
fn tiny_with(old: &str, new: &str) -> Result<Liberty, ParseError> {
    assert!(LIB.contains(old), "no '{old}' in the tiny library");
    parse_str(&LIB.replacen(old, new, 1), Path::new("tiny.lib"))
}

#[test]
fn a_delay_table_along_an_axis_delays_are_not_looked_up_on_is_refused() {
    let unknown = tiny_with(
        "variable_2 : total_output_net_capacitance;",
        "variable_2 : equal_or_opposite_output_net_capacitance;",
    );
    let checks_only = tiny_with(
        "variable_2 : total_output_net_capacitance;",
        "variable_2 : constrained_pin_transition;",
    );

    for refused in [unknown, checks_only] {
        let message = refused.map(|_| ()).unwrap_err().to_string();
        assert!(message.contains("cannot vary along"), "{message}");
    }
}

#[test]
fn a_check_table_along_the_output_load_is_refused() {
    let refused = tiny_with(
        "variable_2 : constrained_pin_transition;",
        "variable_2 : total_output_net_capacitance;",
    );

    let message = refused.map(|_| ()).unwrap_err().to_string();
    assert!(message.contains("cannot vary along"), "{message}");
}

#[test]
fn a_template_no_timing_table_uses_may_vary_along_anything() {
    let lib = tiny_with(
        "  lu_table_template (C2) {",
        "  lu_table_template (P1) { variable_1 : input_voltage; index_1 (\"0.1\"); }\n  \
             lu_table_template (C2) {",
    );

    assert!(lib.is_ok());
}

#[test]
fn an_unknown_timing_type_is_refused() {
    let refused = tiny_with("timing_type : setup_rising;", "timing_type : setup_risng;");

    let message = refused.map(|_| ()).unwrap_err().to_string();
    assert!(
        message.contains("unknown timing_type 'setup_risng'"),
        "{message}"
    );
}

#[test]
fn every_timing_type_opensta_knows_is_read() {
    for timing_type in [
        "non_seq_setup_rising",
        "nochange_high_low",
        "skew_falling",
        "min_pulse_width",
        "retaining_time",
    ] {
        let lib = tiny_with(
            "timing_type : setup_rising;",
            &format!("timing_type : {timing_type};"),
        );

        assert!(lib.is_ok(), "{timing_type}: {lib:?}");
    }
}

#[test]
fn a_file_without_a_library_group_is_an_error_of_the_whole_file() {
    let error = parse_str("/* empty */", Path::new("x.lib")).unwrap_err();

    assert!(matches!(error, ParseError::File { .. }), "{error}");
}

/// A macro whose clock pin `CK` has the clock tree path timing groups
/// `paths`, in a library timed in nanoseconds.
fn macro_with_clock_tree(paths: &str) -> Liberty {
    let src = format!(
        "library (x) {{ time_unit : \"1ns\"; \
             lu_table_template (by_slew) {{ variable_1 : input_net_transition; \
             index_1 (\"0.0, 1.0\"); }} \
             cell (ram) {{ pin (CK) {{ direction : input; clock : true; {paths} }} }} }}"
    );
    parse_str(&src, Path::new("x.lib")).unwrap()
}

fn ns(value: f64) -> Seconds {
    Seconds::new(value * 1e-9)
}

fn clock_tree_delay(lib: &Liberty, slew: Seconds, edge: Edge, bound: MinMax) -> Seconds {
    lib.cells[LibCellId(0)].ports[LibPortId(0)].clock_tree_delay(slew, edge, bound)
}

#[test]
fn a_clock_tree_path_gives_each_edge_its_own_delay_on_its_side() {
    let lib = macro_with_clock_tree(
        "timing () { timing_type : max_clock_tree_path; timing_sense : positive_unate; \
             cell_rise (scalar) { values (\"0.3\"); } cell_fall (scalar) { values (\"0.4\"); } }",
    );

    assert_eq!(
        clock_tree_delay(&lib, ns(0.0), Edge::Rise, MinMax::Max),
        ns(0.3)
    );
    assert_eq!(
        clock_tree_delay(&lib, ns(0.0), Edge::Fall, MinMax::Max),
        ns(0.4)
    );
    assert_eq!(
        clock_tree_delay(&lib, ns(0.0), Edge::Rise, MinMax::Min),
        Seconds::ZERO
    );
}

#[test]
fn a_clock_tree_path_is_looked_up_at_the_pins_slew() {
    let lib = macro_with_clock_tree(
        "timing () { timing_type : min_clock_tree_path; timing_sense : positive_unate; \
             cell_rise (by_slew) { values (\"0.2, 0.6\"); } }",
    );

    let halfway = clock_tree_delay(&lib, ns(0.5), Edge::Rise, MinMax::Min);
    assert!((halfway - ns(0.4)).get().abs() < 1e-21, "{halfway:?}");
    assert_eq!(
        clock_tree_delay(&lib, ns(0.5), Edge::Fall, MinMax::Min),
        Seconds::ZERO
    );
}

#[test]
fn a_negative_unate_clock_tree_path_gives_no_same_edge_delay() {
    let lib = macro_with_clock_tree(
        "timing () { timing_type : max_clock_tree_path; timing_sense : negative_unate; \
             cell_rise (scalar) { values (\"0.3\"); } }",
    );

    assert_eq!(
        clock_tree_delay(&lib, ns(0.0), Edge::Rise, MinMax::Max),
        Seconds::ZERO
    );
}

#[test]
fn a_clock_tree_path_without_a_stated_sense_still_gives_its_delay() {
    let lib = macro_with_clock_tree(
        "timing () { timing_type : max_clock_tree_path; \
             cell_rise (scalar) { values (\"0.3\"); } }",
    );

    assert_eq!(
        clock_tree_delay(&lib, ns(0.0), Edge::Rise, MinMax::Max),
        ns(0.3)
    );
}

#[test]
fn a_library_defining_a_cell_twice_is_refused() {
    let src = "library (x) { cell (c) { } cell (c) { } }";

    let err = parse_str(src, Path::new("x.lib")).unwrap_err();

    assert!(
        err.to_string().contains("cell 'c' is defined twice"),
        "{err}"
    );
}

#[test]
fn an_absorbed_library_brings_its_cells_with_their_own_defaults() {
    let mut first = parse_str(
        "library (a) { default_max_fanout : 10; cell (x) { } }",
        Path::new("a.lib"),
    )
    .unwrap();
    let second = parse_str(
        "library (b) { default_max_fanout : 20; cell (y) { } }",
        Path::new("b.lib"),
    )
    .unwrap();

    first.absorb(second).unwrap();

    let fanout = |name: &str| {
        let cell = first.cell(first.cell_id(name).unwrap());
        first.cell_defaults(cell).max_fanout
    };
    assert_eq!(fanout("x"), Some(10.0));
    assert_eq!(fanout("y"), Some(20.0));
}

#[test]
fn a_refused_absorption_leaves_the_library_as_it_was() {
    let mut first = parse_str("library (a) { cell (x) { } }", Path::new("a.lib")).unwrap();
    let before = first.clone();
    let second = parse_str(
        "library (b) { cell (y) { } cell (x) { } }",
        Path::new("b.lib"),
    )
    .unwrap();

    let clash = first.absorb(second);

    assert!(clash.is_err());
    assert_eq!(first, before);
}

#[test]
fn absorbing_a_library_with_a_cell_already_there_names_the_cell() {
    let mut first = parse_str("library (a) { cell (x) { } }", Path::new("a.lib")).unwrap();
    let second = parse_str("library (b) { cell (x) { } }", Path::new("b.lib")).unwrap();

    let clash = first.absorb(second);

    assert_eq!(
        clash,
        Err(DuplicateName {
            kind: "cell",
            name: "x".to_string(),
        })
    );
}

#[test]
fn a_pad_cell_is_one_by_either_attribute_and_pad_cell_has_the_last_word() {
    let src = "library (x) { \
            cell (plain) { } \
            cell (by_pad_cell) { pad_cell : true; } \
            cell (by_is_pad) { is_pad : true; } \
            cell (overruled) { is_pad : true; pad_cell : false; } }";
    let lib = parse_str(src, Path::new("x.lib")).unwrap();
    let pads: Vec<bool> = lib.cells.iter().map(|c| c.is_pad).collect();
    assert_eq!(pads, vec![false, true, true, false]);
}

#[test]
fn a_statetable_keeps_its_rows_for_comparison() {
    let src = "library (x) { cell (c) { statetable (\"CK E\", \"IQ\") { table : \"L L : - : L ,\\\n   L H : - : H \" ; } } }";
    let lib = parse_str(src, Path::new("x.lib")).unwrap();
    let table = lib.cells[LibCellId(0)].statetable.as_ref().unwrap();
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
    assert!((out.max_capacitance.unwrap().get() - 500e-15).abs() < 1e-27);
    let addr = &mem.ports[mem.port("addr_in[1]").unwrap()];
    assert!((addr.capacitance.rise.get() - 5e-15).abs() < 1e-28);
    let clk_to_q: Vec<&TimingArcSet> = mem
        .arc_sets
        .iter()
        .filter(|s| s.role == ArcRole::RegClkToQ)
        .collect();
    assert_eq!(clk_to_q.len(), 2);
    let arc = &clk_to_q[0].arcs[0];
    assert!(
        (arc.gate_delay(Seconds::new(0.0), Farads::new(500e-15))
            .1
            .get()
            - 0.227e-9)
            .abs()
            < 1e-14
    );
    assert!(mem.arc_sets.iter().any(|s| s.role == ArcRole::Setup));
}

/// A library with cell `mem`: a clock `clk`, and the 2-bit buses `d` and
/// `q`, whose groups hold `d_body` and `q_body` after their type and
/// direction.
fn bus_library(d_body: &str, q_body: &str) -> Result<Liberty, ParseError> {
    let src = format!(
        r#"
library (ram) {{
  time_unit : "1ns";
  capacitive_load_unit (1,ff);
  type (DATA) {{ base_type : array ; data_type : bit ; bit_width : 2; bit_from : 1; bit_to : 0 ; }}
  cell(mem) {{
    pin(clk) {{ direction : input; capacitance : 1.0; clock : true; }}
    bus(d) {{ bus_type : DATA; direction : input; capacitance : 5.0; {d_body} }}
    bus(q) {{ bus_type : DATA; direction : output; {q_body} }}
  }}
}}
"#
    );
    parse_str(&src, Path::new("ram.lib"))
}

fn mem_port<'l>(lib: &'l Liberty, name: &str) -> &'l LibPort {
    let mem = lib.cell(lib.cell_id("mem").unwrap());
    &mem.ports[mem.port(name).unwrap()]
}

#[test]
fn a_bus_bit_keeps_the_bus_capacitance_its_own_pin_group_leaves_out() {
    let lib = bus_library("pin(d[1]) { direction : input; max_transition : 0.3; }", "").unwrap();

    let bit = mem_port(&lib, "d[1]");

    assert!(
        (bit.capacitance.rise.get() - 5e-15).abs() < 1e-28,
        "{bit:?}"
    );
    assert!(
        (bit.max_transition.unwrap().get() - 0.3e-9).abs() < 1e-20,
        "{bit:?}"
    );
}

#[test]
fn a_bus_bit_pin_group_without_a_direction_takes_the_bus_direction() {
    let lib = bus_library("pin(d[0]) { capacitance : 7.0; }", "").unwrap();

    let bit = mem_port(&lib, "d[0]");

    assert_eq!(bit.direction, PortDirection::Input);
    assert!(
        (bit.capacitance.rise.get() - 7e-15).abs() < 1e-28,
        "{bit:?}"
    );
}

#[test]
fn a_pin_group_over_a_range_of_bus_bits_gives_each_bit_its_arcs() {
    let timing = r#"pin("q[1:0]") { timing() { related_pin : "clk"; timing_type : rising_edge;
            cell_rise(scalar) { values ("0.2"); } cell_fall(scalar) { values ("0.2"); } } }"#;
    let lib = bus_library("", timing).unwrap();
    let mem = lib.cell(lib.cell_id("mem").unwrap());

    let driven: Vec<&str> = mem
        .arc_sets
        .iter()
        .filter(|s| s.role == ArcRole::RegClkToQ)
        .map(|s| mem.ports[s.to].name.as_str())
        .collect();

    assert_eq!(driven, ["q[1]", "q[0]"]);
}

#[test]
fn a_pin_group_naming_a_bit_outside_its_bus_is_refused() {
    let err = bus_library("pin(d[2]) { capacitance : 7.0; }", "").unwrap_err();

    assert!(err.to_string().contains("not a bit of bus 'd'"), "{err}");
}

/// The arc sets of cell `c`, whose output `Z` has `output_body` and
/// is timed from inputs `A`, `B` and `EN` by `timing_body`.
fn sensed_cell(output_body: &str, timing_body: &str) -> LibCell {
    let src = format!(
        r#"
library (x) {{
  time_unit : "1ns";
  capacitive_load_unit (1,ff);
  cell (c) {{
    pin (A) {{ direction : input; capacitance : 1.0; }}
    pin (B) {{ direction : input; capacitance : 1.0; }}
    pin (EN) {{ direction : input; capacitance : 1.0; }}
    pin (Z) {{ direction : output; {output_body}
      timing () {{ {timing_body}
        cell_rise(scalar) {{ values ("0.1"); }} cell_fall(scalar) {{ values ("0.1"); }} }} }}
  }}
}}
"#
    );
    let lib = parse_str(&src, Path::new("x.lib")).unwrap();
    lib.cells[LibCellId(0)].clone()
}

#[test]
fn a_missing_timing_sense_is_inferred_from_the_output_function() {
    let cell = sensed_cell(r#"function : "!(A & B)";"#, r#"related_pin : "A";"#);

    let set = &cell.arc_sets[crate::db::liberty::ArcSetId::new(0)];
    let edges: Vec<(Edge, Edge)> = set.arcs.iter().map(|a| (a.from_edge, a.to_edge)).collect();

    assert_eq!(set.sense, TimingSense::NegativeUnate);
    assert_eq!(edges, [(Edge::Rise, Edge::Fall), (Edge::Fall, Edge::Rise)]);
}

#[test]
fn non_unate_arcs_come_falling_outputs_first_as_opensta_makes_them() {
    let cell = sensed_cell(r#"function : "A ^ B";"#, r#"related_pin : "A";"#);

    let edges: Vec<(Edge, Edge)> = cell.arc_sets[crate::db::liberty::ArcSetId::new(0)]
        .arcs
        .iter()
        .map(|a| (a.from_edge, a.to_edge))
        .collect();

    assert_eq!(
        edges,
        [
            (Edge::Fall, Edge::Fall),
            (Edge::Rise, Edge::Fall),
            (Edge::Rise, Edge::Rise),
            (Edge::Fall, Edge::Rise),
        ]
    );
}

#[test]
fn a_missing_tristate_enable_sense_is_the_opposite_of_the_three_state_condition() {
    let cell = sensed_cell(
        r#"function : "A"; three_state : "!EN";"#,
        r#"related_pin : "EN"; timing_type : three_state_enable;"#,
    );

    assert_eq!(
        cell.arc_sets[crate::db::liberty::ArcSetId::new(0)].sense,
        TimingSense::PositiveUnate
    );
}

#[test]
fn a_clear_arc_only_ever_drives_its_output_low() {
    let cell = sensed_cell(
        r#"function : "A";"#,
        r#"related_pin : "B"; timing_type : clear; timing_sense : positive_unate;"#,
    );

    let edges: Vec<(Edge, Edge)> = cell.arc_sets[crate::db::liberty::ArcSetId::new(0)]
        .arcs
        .iter()
        .map(|a| (a.from_edge, a.to_edge))
        .collect();

    assert_eq!(edges, [(Edge::Fall, Edge::Fall)]);
}

#[test]
fn a_preset_arc_only_ever_drives_its_output_high() {
    let cell = sensed_cell(
        r#"function : "A";"#,
        r#"related_pin : "B"; timing_type : preset; timing_sense : negative_unate;"#,
    );

    let edges: Vec<(Edge, Edge)> = cell.arc_sets[crate::db::liberty::ArcSetId::new(0)]
        .arcs
        .iter()
        .map(|a| (a.from_edge, a.to_edge))
        .collect();

    assert_eq!(edges, [(Edge::Fall, Edge::Rise)]);
}

#[test]
fn a_bundle_group_is_refused() {
    let src = "library (x) { cell (c) { bundle (b) { } } }";
    let err = parse_str(src, Path::new("x.lib")).unwrap_err();
    assert!(err.to_string().contains("bundle"));
}
