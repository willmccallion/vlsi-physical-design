use super::*;
use pare_common::db::parser::liberty;
use std::path::Path;

/// The buffer `B` whose timing arc has the tables `tables`.
fn buffer_with(tables: &str) -> (Liberty, BufferCell) {
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
                   timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ; {tables} }}
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
    let (lib, buffer) = buffer_with(&format!("cell_rise {TABLE} rise_transition {TABLE}"));

    let alone = rising_delay(&lib, buffer, Farads::ZERO).unwrap();
    let loaded = rising_delay(&lib, buffer, Farads::new(50e-15)).unwrap();

    assert!(0.0 < alone && alone < loaded, "{alone} {loaded}");
}

#[test]
fn a_buffer_without_a_rising_arc_has_no_rising_delay() {
    let (lib, buffer) = buffer_with(&format!("cell_fall {TABLE} fall_transition {TABLE}"));

    assert_eq!(rising_delay(&lib, buffer, Farads::ZERO), None);
}
