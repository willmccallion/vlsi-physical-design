use super::*;
use crate::fixtures;
use pare_common::db::parser::liberty;
use std::path::Path;

/// The buffer `B` whose timing arc has the unateness `sense` and the
/// tables `tables`.
fn buffer_with(sense: &str, tables: &str) -> (Liberty, BufferCell) {
    let src = format!(
        "library (x) {{
               capacitive_load_unit (1,ff) ;
               lu_table_template (delay_t) {{
                 variable_1 : input_net_transition ;
                 variable_2 : total_output_net_capacitance ;
                 index_1 (\"0.005, 0.2\") ;
                 index_2 (\"1.0, 100.0\") ;
               }}
               cell (B) {{
                 pin (A) {{ direction : input ; capacitance : 1.0 ; }}
                 pin (Z) {{
                   direction : output ; function : \"A\" ;
                   timing () {{ related_pin : \"A\" ; timing_sense : {sense} ; {tables} }}
                 }}
               }}
             }}"
    );
    let lib = liberty::parse_str(&src, Path::new("x.lib")).unwrap();
    let buffer = lib.buffer_cell(lib.cell_id("B").unwrap()).unwrap();
    (lib, buffer)
}

const TABLE: &str = "(delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }";

#[test]
fn a_buffers_rising_delay_grows_with_the_load_beyond_its_own_input() {
    let (lib, buffer) = buffer_with(
        "positive_unate",
        &format!("cell_rise {TABLE} rise_transition {TABLE}"),
    );

    let alone = rising_delay(&lib, buffer, Farads::ZERO).unwrap();
    let loaded = rising_delay(&lib, buffer, Farads::new(50e-15)).unwrap();

    assert!(0.0 < alone && alone < loaded, "{alone} {loaded}");
}

#[test]
fn a_buffer_without_a_rising_arc_has_no_rising_delay() {
    let (lib, buffer) = buffer_with(
        "positive_unate",
        &format!("cell_fall {TABLE} fall_transition {TABLE}"),
    );

    assert_eq!(rising_delay(&lib, buffer, Farads::ZERO), None);
}

#[test]
fn only_a_rising_input_rising_the_output_times_a_buffer() {
    let rising = format!("cell_rise {TABLE} rise_transition {TABLE}");
    let slow_fall = "cell_fall (delay_t) { values (\"0.5, 0.9\", \"0.6, 0.9\") ; }";
    let (both, buffer) = buffer_with(
        "non_unate",
        &format!("{rising} {slow_fall} fall_transition {TABLE}"),
    );
    let (rise_only, _) = buffer_with("positive_unate", &rising);

    let delays = [&both, &rise_only].map(|lib| rising_delay(lib, buffer, Farads::ZERO));

    assert_eq!(delays[0], delays[1]);
}

#[test]
fn a_buffer_of_no_rising_delay_has_none() {
    let zero = "(delay_t) { values (\"0.0, 0.0\", \"0.0, 0.0\") ; }";
    let (lib, buffer) = buffer_with(
        "positive_unate",
        &format!("cell_rise {zero} rise_transition {TABLE}"),
    );

    assert_eq!(rising_delay(&lib, buffer, Farads::ZERO), None);
}

/// The fixture cells as the clock sees them: `CLKBUF_X3` a buffer, `INV_X1`
/// an inverter, `AND2_X1` a gate, `DFF_X1` a flop, `DFF_SLOW` a clock gate
/// clocked at `CK`, `DFF_SLOWER` a latch of data `D` and `BUF_X1` a buffer
/// that its library also gives a register.
const CLOCK_ROLES: &str = r#"library (roles) {
  cell (CLKBUF_X3) { pin (A) { direction : input ; }
    pin (Z) { direction : output ; function : "A" ; } }
  cell (INV_X1) { pin (A) { direction : input ; }
    pin (ZN) { direction : output ; function : "!A" ; } }
  cell (AND2_X1) { pin (A) { direction : input ; } pin (B) { direction : input ; }
    pin (Z) { direction : output ; function : "(A & B)" ; } }
  cell (DFF_X1) { ff ("IQ", "IQN") { next_state : "D" ; clocked_on : "CK" ; }
    pin (D) { direction : input ; } pin (CK) { direction : input ; clock : true ; }
    pin (Q) { direction : output ; function : "IQ" ; } }
  cell (DFF_SLOW) { ff ("IQ", "IQN") { next_state : "D" ; clocked_on : "CK" ; }
    pin (D) { direction : input ; clock_gate_enable_pin : true ; }
    pin (CK) { direction : input ; clock_gate_clock_pin : true ; }
    pin (Q) { direction : output ; clock_gate_out_pin : true ; } }
  cell (DFF_SLOWER) { latch ("IQ", "IQN") { enable : "CK" ; data_in : "D" ; }
    pin (D) { direction : input ; } pin (CK) { direction : input ; }
    pin (Q) { direction : output ; function : "IQ" ; } }
  cell (BUF_X1) { ff ("IQ", "IQN") { next_state : "A" ; clocked_on : "A" ; }
    pin (A) { direction : input ; }
    pin (Z) { direction : output ; function : "A" ; } }
}"#;

#[test]
fn the_clock_passes_buffers_gates_a_gates_clock_and_a_latchs_data() {
    let db = fixtures::design(
        "COMPONENTS 7 ;\n- buf CLKBUF_X3 ;\n- inv INV_X1 ;\n- and AND2_X1 ;\n- ff DFF_X1 ;\n\
         - icg DFF_SLOW ;\n- lat DFF_SLOWER ;\n- regbuf BUF_X1 ;\nEND COMPONENTS\n\
         NETS 1 ;\n- n ( buf A ) ( inv A ) ( and A ) ( ff CK ) ( ff D ) ( icg CK ) ( icg D ) \
         ( lat CK ) ( lat D ) ( regbuf A ) ;\nEND NETS\n",
    );
    let lib = liberty::parse_str(CLOCK_ROLES, Path::new("roles.lib")).unwrap();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let net = db.net_id("n").unwrap();

    let mut passing: Vec<(String, bool)> = db.nets[net]
        .pins
        .iter()
        .map(|&pin| (db.pin_name(pin), propagates_clock(&link, &db, pin)))
        .collect();
    passing.sort();

    let expected = [
        ("and/A", true),
        ("buf/A", true),
        ("ff/CK", false),
        ("ff/D", false),
        ("icg/CK", true),
        ("icg/D", false),
        ("inv/A", true),
        ("lat/CK", false),
        ("lat/D", true),
        ("regbuf/A", true),
    ]
    .map(|(name, passes)| (name.to_string(), passes));
    assert_eq!(passing, expected);
}
