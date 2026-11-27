use super::*;
use crate::db::design::{NewIoPin, PlacementStatus};
use crate::db::library::{Library, Macro, MacroClass, Symmetry};
use crate::geom::dbu::{Dbu, DbuPerMicron};
use crate::geom::orient::Orientation;
use crate::geom::point::Point;
use crate::units::Farads;

fn design() -> NetlistDB {
    let mut library = Library::new(DbuPerMicron::new(2000).unwrap());
    let sram = library
        .add_macro(Macro::new(
            "fakeram".into(),
            MacroClass::Block,
            Point::new(Dbu::new(2000), Dbu::new(2000)),
            Symmetry::default(),
            None,
        ))
        .unwrap();
    let mut db = NetlistDB::new("top".into(), library);
    let _ = db
        .add_cell(
            "core/mem/macro_mem".into(),
            sram,
            Point::new(Dbu::new(0), Dbu::new(0)),
            Orientation::N,
            PlacementStatus::Placed,
        )
        .unwrap();
    for (name, direction) in [
        ("clk", PinDirection::Input),
        ("rst_n", PinDirection::Input),
        ("key[0]", PinDirection::Input),
        ("key[1]", PinDirection::Input),
        ("out[0]", PinDirection::Output),
        ("io", PinDirection::Inout),
        ("VDD", PinDirection::Inout),
    ] {
        let net = db.add_net(name).unwrap();
        let _ = db
            .add_io_pin(NewIoPin {
                name: name.into(),
                net,
                direction,
                signal_use: if name == "VDD" {
                    SignalUse::Power
                } else {
                    SignalUse::Signal
                },
                shapes: Vec::new(),
                location: Point::new(Dbu::new(0), Dbu::new(0)),
                orient: Orientation::N,
                status: PlacementStatus::Placed,
            })
            .unwrap();
    }
    db
}

fn liberty() -> Liberty {
    let mut lib = Liberty::new("lib".into());
    lib.units.time = Seconds::new(1e-9);
    lib.units.capacitance = Farads::new(1e-15);
    lib
}

fn parse(text: &str) -> Sdc {
    parse_str(text, Path::new("test.sdc"), &design(), &liberty()).unwrap()
}

fn port(db: &NetlistDB, name: &str) -> IoPinId {
    db.io_pin_id(name).unwrap()
}

const ORFS: &str = r"
current_design top
set clk_name core_clock
set clk_port_name clk
set clk_period 2.2
set clk_io_pct 0.2
set clk_port [get_ports $clk_port_name]
create_clock -name $clk_name -period $clk_period $clk_port
set non_clock_inputs [all_inputs -no_clocks]
set_input_delay [expr $clk_period * $clk_io_pct] -clock $clk_name $non_clock_inputs
set_output_delay [expr $clk_period * $clk_io_pct] -clock $clk_name [all_outputs]
";

/// A script that leans on every rule [`split_commands`] keeps.
const TRICKY_SCRIPT: &str = "  # a comment \\\n still the comment\nset a 1; set b {x;\ny}\n\
        \n\tcreate_clock [get_ports \"clk;\"] \\\n    -period 2\nputs \"q;\\\"\" ; # after\nlast [a\nb]";

#[test]
fn a_script_splits_into_commands_at_their_lines() {
    let commands = split_commands(TRICKY_SCRIPT, 1);

    assert_eq!(
        format!("{commands:?}"),
        "[(\"set a 1\", 3), (\"set b {x;\\ny}\", 3), (\"create_clock [get_ports \\\"clk;\\\"]  -period 2\", 6), (\"puts \\\"q;\\\\\\\"\\\" \", 8), (\"last [a\\nb]\", 9)]"
    );
}

#[test]
fn a_command_splits_into_braced_and_plain_words() {
    let words = split_words("a {b {c} d} \"e [f \"g\"] \\\" h\" i[j k]{l m} n\\ o");

    assert_eq!(
        format!("{words:?}"),
        "Ok([Plain(\"a\"), Braced(\"b {c} d\"), Plain(\"e [f \\\"g\\\"] \\\\\\\" h\"), Plain(\"i[j k]{l m}\"), Plain(\"n\\\\ o\")])"
    );
}

#[test]
fn words_report_what_they_leave_unbalanced() {
    let errors: Vec<_> = ["{a", "{a}b", "\"a", "a[b", "\"a[b\""]
        .into_iter()
        .map(split_words)
        .collect();

    assert_eq!(
        format!("{errors:?}"),
        "[Err(\"unbalanced '{'\"), Err(\"extra characters after close-brace\"), Err(\"unbalanced '\\\"'\"), Err(\"unbalanced '['\"), Err(\"unbalanced '['\")]"
    );
}

#[test]
fn an_orfs_constraint_file_defines_the_clock_and_port_delays() {
    let db = design();
    let sdc = parse(ORFS);

    assert_eq!(sdc.clocks.len(), 1);
    let clock = &sdc.clocks[ClockId(0)];
    assert_eq!(clock.name, "core_clock");
    assert!((clock.period.get() - 2.2e-9).abs() < 1e-20);
    assert!((clock.fall.get() - 1.1e-9).abs() < 1e-20);
    assert_eq!(clock.sources, vec![port(&db, "clk")]);
    assert!(!sdc.input_delays.contains_key(&port(&db, "clk")));
    let delay = &sdc.input_delays[&port(&db, "key[1]")];
    assert_eq!(delay.len(), 1);
    assert_eq!(delay[0].reference.edge, Edge::Rise);
    let value = delay[0].delays.get(Edge::Fall, MinMax::Min).unwrap().get();
    assert!((value - 0.44e-9).abs() < 1e-18);
    assert!(sdc.input_delays.contains_key(&port(&db, "io")));
    assert!(sdc.output_delays.contains_key(&port(&db, "io")));
    assert!(sdc.output_delays.contains_key(&port(&db, "out[0]")));
    assert!(!sdc.input_delays.contains_key(&port(&db, "VDD")));
    assert!(!sdc.output_delays.contains_key(&port(&db, "VDD")));
}

#[test]
fn expr_follows_tcl_integer_and_float_rules() {
    let sdc = parse(
        "set p 3\n\
             create_clock -name a -period [expr $p/2] [get_ports clk]\n\
             create_clock -name b -period [expr ($p + 1.0) / 2] -waveform [list 0 [expr $p/2]] [get_ports rst_n]\n",
    );

    assert!((sdc.clocks[ClockId(0)].period.get() - 1e-9).abs() < 1e-20);
    assert!((sdc.clocks[ClockId(1)].period.get() - 2e-9).abs() < 1e-20);
    assert!((sdc.clocks[ClockId(1)].fall.get() - 1e-9).abs() < 1e-20);
}

#[test]
fn a_clock_redefined_by_name_is_replaced_in_place() {
    let sdc = parse(
        "create_clock -name c -period 2 [get_ports clk]\n\
             create_clock -name d -period 3 [get_ports rst_n]\n\
             create_clock -name c -period 5 -waveform {1 2} [get_ports clk]\n",
    );

    assert_eq!(sdc.clocks.len(), 2);
    let c = &sdc.clocks[ClockId(0)];
    assert_eq!(c.name, "c");
    assert!((c.period.get() - 5e-9).abs() < 1e-20);
    assert!((c.rise.get() - 1e-9).abs() < 1e-20);
}

#[test]
fn a_new_clock_takes_its_ports_from_the_others_unless_added() {
    let db = design();
    let taken = parse(
        "create_clock -name c -period 2 [get_ports clk]\n\
             create_clock -name d -period 3 [get_ports clk]\n",
    );
    let added = parse(
        "create_clock -name c -period 2 [get_ports clk]\n\
             create_clock -name d -period 3 -add [get_ports clk]\n",
    );

    let clk = port(&db, "clk");
    assert!(!taken.clocks[ClockId(0)].sources.contains(&clk));
    assert!(taken.clocks[ClockId(1)].sources.contains(&clk));
    assert!(added.clocks[ClockId(0)].sources.contains(&clk));
    assert!(added.clocks[ClockId(1)].sources.contains(&clk));
}

#[test]
fn added_delays_merge_and_replaced_delays_clear() {
    let db = design();
    let sdc = parse(
        "create_clock -name c -period 2 -waveform {0 1} [get_ports clk]\n\
             set_input_delay -clock c -max 0.5 [get_ports {key[0]}]\n\
             set_input_delay -clock c -min 0.1 [get_ports {key[0]}]\n\
             set_input_delay -clock c -clock_fall -max 0.7 -add_delay [get_ports {key[0]}]\n\
             set_input_delay -clock c -clock_fall -max 0.6 -add_delay [get_ports {key[0]}]\n",
    );

    let delays = &sdc.input_delays[&port(&db, "key[0]")];
    assert_eq!(delays.len(), 2);
    let rise = delays
        .iter()
        .find(|d| d.reference.edge == Edge::Rise)
        .unwrap();
    assert!((rise.delays.get(Edge::Rise, MinMax::Max).unwrap().get() - 0.5e-9).abs() < 1e-20);
    assert!((rise.delays.get(Edge::Rise, MinMax::Min).unwrap().get() - 0.1e-9).abs() < 1e-20);
    let fall = delays
        .iter()
        .find(|d| d.reference.edge == Edge::Fall)
        .unwrap();
    assert!((fall.delays.get(Edge::Rise, MinMax::Max).unwrap().get() - 0.7e-9).abs() < 1e-20);
}

#[test]
fn exceptions_name_clocks_and_ports() {
    let db = design();
    let sdc = parse(
        "create_clock -name a -period 2 [get_ports clk]\n\
             create_clock -name b -period 4 [get_ports rst_n]\n\
             set_clock_uncertainty 0.15 [get_clocks a]\n\
             set_clock_uncertainty -hold 0.05 [get_clocks b]\n\
             set_false_path -from [get_clocks a] -to [get_clocks b]\n\
             set_multicycle_path 2 -setup -to [list [get_ports {out[0]}]]\n\
             set_multicycle_path 1 -hold -to [get_ports {out[0]}]\n\
             set_timing_derate -early 0.95\n\
             set_max_fanout 20 [current_design]\n\
             set_load 3 [get_ports {out[0]}]\n",
    );

    let close =
        |got: Option<Seconds>, want: f64| got.is_some_and(|g| (g.get() - want).abs() < 1e-20);
    assert!(close(
        sdc.clocks[ClockId(0)].uncertainty[MinMax::Min],
        0.15e-9
    ));
    assert!(close(
        sdc.clocks[ClockId(0)].uncertainty[MinMax::Max],
        0.15e-9
    ));
    assert!(close(
        sdc.clocks[ClockId(1)].uncertainty[MinMax::Min],
        0.05e-9
    ));
    assert_eq!(sdc.clocks[ClockId(1)].uncertainty[MinMax::Max], None);
    assert_eq!(sdc.false_paths.len(), 1);
    assert_eq!(sdc.false_paths[0].paths.from_clocks, vec![ClockId(0)]);
    assert_eq!(sdc.false_paths[0].paths.to_clocks, vec![ClockId(1)]);
    assert_eq!(sdc.multicycle_paths.len(), 2);
    assert_eq!(sdc.multicycle_paths[0].multiplier, 2);
    assert_eq!(sdc.multicycle_paths[0].side, Some(MinMax::Max));
    assert!(sdc.multicycle_paths[0].use_end_clock);
    assert_eq!(
        sdc.multicycle_paths[0].paths.to_ports,
        vec![port(&db, "out[0]")]
    );
    assert!(!sdc.multicycle_paths[1].use_end_clock);
    assert!((sdc.derating.cell[MinMax::Min] - 0.95).abs() < 1e-12);
    assert!((sdc.derating.net[MinMax::Max] - 1.0).abs() < 1e-12);
    assert_eq!(sdc.limits.max_fanout, Some(20.0));
    assert!((sdc.port_loads[&port(&db, "out[0]")].get() - 3e-15).abs() < 1e-30);
}

#[test]
fn asynchronous_clock_groups_are_false_paths_both_ways() {
    let sdc = parse(
        "create_clock -name a -period 2 [get_ports clk]\n\
             create_clock -name b -period 4 [get_ports rst_n]\n\
             set_clock_groups -asynchronous -group [get_clocks a] -group [get_clocks b]\n",
    );

    assert_eq!(sdc.false_paths.len(), 2);
    assert_eq!(sdc.false_paths[0].paths.from_clocks, vec![ClockId(0)]);
    assert_eq!(sdc.false_paths[1].paths.from_clocks, vec![ClockId(1)]);
}

#[test]
fn comments_continuations_and_quotes_parse() {
    let sdc = parse(
        "# a comment; with a semicolon\n\
             create_clock -name \"my clk\" \\\n    -period 2 \\\n    [get_ports clk]; set x 1\n",
    );

    assert_eq!(sdc.clocks[ClockId(0)].name, "my clk");
}

#[test]
fn unknown_commands_are_errors() {
    let error = parse_str(
            "create_clock -name a -period 2 [get_ports clk]\nset_driving_cell -lib_cell BUF_X1 [get_ports rst_n]\n",
            Path::new("t.sdc"),
            &design(),
            &liberty(),
        )
        .unwrap_err();

    assert!(error.to_string().contains("t.sdc:2"));
    assert!(error.to_string().contains("not supported"));
}

#[test]
fn globs_match_bus_bits_literally() {
    assert!(glob_matches("key[*]", "key[13]"));
    assert!(glob_matches("key*", "key[0]"));
    assert!(!glob_matches("key?", "key[0]"));
    assert!(glob_matches("k?y[0]", "key[0]"));
}

#[test]
fn set_propagated_clock_marks_the_clocks_named() {
    let sdc = parse(
        "create_clock -name clk -period 1 [get_ports clk]\n\
             set_propagated_clock [get_clocks {clk}]\n",
    );

    assert!(sdc.clocks[ClockId(0)].propagated);
}

#[test]
fn get_cells_names_instances_exactly_or_by_glob() {
    let sdc = parse(
        "create_clock -name clk -period 1 [get_ports clk]\n\
             set_timing_derate -early -cell_delay 0.97 [get_cells {core/mem/macro_mem}]\n",
    );
    let db = design();
    let cell = db.cell_id("core/mem/macro_mem").unwrap();

    assert!((sdc.cell_derate(Some(cell), MinMax::Min) - 0.97).abs() < 1e-12);
    assert!((sdc.cell_derate(Some(cell), MinMax::Max) - 1.0).abs() < 1e-12);
    assert!((sdc.cell_derate(None, MinMax::Min) - 1.0).abs() < 1e-12);

    let globbed = parse(
        "create_clock -name clk -period 1 [get_ports clk]\n\
             set_timing_derate -late 1.05 [get_cells core/*]\n",
    );
    assert!((globbed.cell_derate(Some(cell), MinMax::Max) - 1.05).abs() < 1e-12);
    assert!((globbed.derating.cell[MinMax::Max] - 1.0).abs() < 1e-12);
}

#[test]
fn get_cells_of_nothing_names_nothing() {
    let sdc = parse(
        "create_clock -name clk -period 1 [get_ports clk]\n\
             set_timing_derate -early 0.9 [get_cells nothing]\n",
    );
    let db = design();
    let cell = db.cell_id("core/mem/macro_mem").unwrap();

    assert!(sdc.cell_derates.is_empty());
    assert!((sdc.cell_derate(Some(cell), MinMax::Min) - 1.0).abs() < 1e-12);
    assert!((sdc.derating.cell[MinMax::Min] - 1.0).abs() < 1e-12);
}

fn parse_error(text: &str) -> String {
    parse_str(text, Path::new("t.sdc"), &design(), &liberty())
        .unwrap_err()
        .to_string()
}

#[test]
fn a_derate_of_clock_paths_is_refused_rather_than_read_as_a_global_one() {
    let error = parse_error("set_timing_derate -clock -early 0.95\n");

    assert!(error.contains("-clock is not supported"), "{error}");
}

#[test]
fn a_false_path_through_an_edge_is_refused_rather_than_widened() {
    let error = parse_error(
        "create_clock -name a -period 2 [get_ports clk]\n\
             set_false_path -rise_through [get_ports rst_n] -to [get_clocks a]\n",
    );

    assert!(error.contains("-rise_through is not supported"), "{error}");
}

#[test]
fn an_option_a_command_does_not_take_is_an_error() {
    let error = parse_error("create_clock -nme a -period 2 [get_ports clk]\n");

    assert!(error.contains("unknown option -nme"), "{error}");
}

#[test]
fn a_key_without_its_value_is_an_error() {
    let error = parse_error("create_clock -name a [get_ports clk] -period\n");

    assert!(error.contains("-period wants a value"), "{error}");
}

#[test]
fn a_negative_number_is_a_value_not_an_option() {
    let sdc = parse(
        "create_clock -name a -period 2 [get_ports clk]\n\
             set_input_delay -0.5 -clock a [get_ports rst_n]\n",
    );

    assert_eq!(sdc.input_delays.len(), 1);
}

#[test]
fn unit_prefixes_keep_their_case_and_suffixes_do_not() {
    assert_eq!(unit_scale("1Ms", "s"), Some(1e6));
    assert_eq!(unit_scale("1ms", "s"), Some(1e-3));
    assert_eq!(unit_scale("ps", "s"), Some(1e-12));
    assert_eq!(unit_scale("fF", "f"), Some(1e-15));
    assert_eq!(unit_scale("NS", "s"), None);
}

/// What `script` evaluates to over the test design: its last command's
/// result.
fn eval(script: &str) -> Result<String, ParseError> {
    let db = design();
    let mut interpreter = Interpreter {
        db: &db,
        units: liberty().units,
        path: Path::new("t.sdc"),
        variables: HashMap::new(),
        sdc: Sdc::default(),
    };
    interpreter.eval_script(script, 1)
}

/// The value `expr` gives for `text`.
fn expr(text: &str) -> Result<String, ParseError> {
    eval(&format!("expr {{{text}}}"))
}

#[test]
fn integer_arithmetic_stays_integer() {
    for (text, value) in [
        ("7 + 2", "9"),
        ("7 - 2", "5"),
        ("7 * 2", "14"),
        ("7 / 2", "3"),
        ("7 % 3", "1"),
        ("5-2", "3"),
        ("2 + 3 * 4", "14"),
        ("1 - 2", "-1"),
    ] {
        assert_eq!(expr(text).unwrap(), value, "{text}");
    }
}

#[test]
fn integer_division_rounds_down_and_the_remainder_takes_the_sign_of_the_divisor() {
    for (text, value) in [
        ("-7 / 2", "-4"),
        ("7 / -2", "-4"),
        ("-7 / -2", "3"),
        ("-8 / 2", "-4"),
        ("-7 % 2", "1"),
        ("7 % -2", "-1"),
        ("-7 % -2", "-1"),
        ("-6 % 3", "0"),
    ] {
        assert_eq!(expr(text).unwrap(), value, "{text}");
    }
}

#[test]
fn an_integer_quotient_that_overflows_is_an_error() {
    assert_eq!(
        expr("(-9223372036854775807 - 1) / 1").unwrap(),
        "-9223372036854775808"
    );
    assert!(expr("(-9223372036854775807 - 1) / -1").is_err());
}

#[test]
fn dividing_by_zero_is_an_error() {
    for text in ["7 / 0", "7 % 0", "1.5 / 0"] {
        assert!(expr(text).is_err(), "{text}");
    }
}

#[test]
fn a_fraction_makes_the_arithmetic_floating() {
    for (text, value) in [
        ("7.5 - 2", "5.5"),
        ("5 / 2.0", "2.5"),
        ("1e-3 * 1000", "1.0"),
        ("2e+1", "20.0"),
        ("-1.5", "-1.5"),
    ] {
        assert_eq!(expr(text).unwrap(), value, "{text}");
    }
}

#[test]
fn unary_signs_apply_to_what_follows() {
    for (text, value) in [
        ("-3", "-3"),
        ("+3", "3"),
        ("-3 + 1", "-2"),
        ("2 * -3", "-6"),
    ] {
        assert_eq!(expr(text).unwrap(), value, "{text}");
    }
}

#[test]
fn an_operator_where_a_number_belongs_is_an_error() {
    assert!(expr("* 3").is_err());
}

#[test]
fn a_backslash_escapes_the_character_after_it() {
    assert_eq!(eval("set v x\\ty").unwrap(), "x\ty");
    assert_eq!(eval("set v \"a\\\"b\"").unwrap(), "a\"b");
}

#[test]
fn a_backslash_ending_a_word_stays_a_backslash() {
    assert_eq!(eval("set v x\\").unwrap(), "x\\");
}

#[test]
fn a_dollar_without_a_name_stays_a_dollar() {
    assert_eq!(eval("set v \"cost $ 5\"").unwrap(), "cost $ 5");
}

#[test]
fn a_braced_variable_name_may_hold_any_character() {
    assert_eq!(eval("set a.b 1; set v x${a.b}y").unwrap(), "x1y");
}

#[test]
fn set_with_only_a_name_gives_the_value() {
    assert_eq!(eval("set x 5; set x").unwrap(), "5");
}

#[test]
fn llength_counts_the_elements_of_a_list() {
    assert_eq!(eval("llength {a b {c d}}").unwrap(), "3");
}

#[test]
fn lindex_picks_an_element_by_position_or_the_last() {
    assert_eq!(eval("lindex {a b c} 1").unwrap(), "b");
    assert_eq!(eval("lindex {a b c} end").unwrap(), "c");
    assert_eq!(eval("lindex {a b} 5").unwrap(), "");
}

#[test]
fn a_bracketed_command_skips_escaped_braced_and_quoted_brackets() {
    for (script, value) in [
        ("set v [list x\\]]", "{x]}"),
        ("set v [list {a]b}]", "{a]b}"),
        ("set v [list \"a]b\"]", "{a]b}"),
        ("set v [list \"a\\\"]b\"]", "{a\"]b}"),
    ] {
        assert_eq!(eval(script).unwrap(), value, "{script}");
    }
}

#[test]
fn set_units_checks_the_units_without_a_result() {
    assert_eq!(eval("set_units -time ns -capacitance fF").unwrap(), "");
    let error = eval("set_units -time parsecs").unwrap_err().to_string();
    assert!(error.contains("unknown unit 'parsecs'"), "{error}");
}

#[test]
fn set_units_warns_only_where_the_library_differs() {
    let matching = crate::testing::capture_logs(|| {
        let _ = eval("set_units -time 1000ps -capacitance fF").unwrap();
    });
    let differing = crate::testing::capture_logs(|| {
        let _ = eval("set_units -time ps").unwrap();
    });

    assert_eq!(matching, Vec::<String>::new());
    assert_eq!(differing.len(), 1, "{differing:?}");
    assert!(differing[0].contains("set_units -time ps"), "{differing:?}");
}

#[test]
fn a_port_matching_nothing_is_an_error_unless_quiet() {
    assert!(eval("get_ports nosuch").is_err());
    assert_eq!(eval("get_ports -quiet nosuch").unwrap(), "");
}

#[test]
fn an_instance_matching_nothing_warns_unless_quiet() {
    let unmatched = crate::testing::capture_logs(|| {
        let _ = eval("get_cells nosuch").unwrap();
    });
    let quiet = crate::testing::capture_logs(|| {
        let _ = eval("get_cells -quiet nosuch").unwrap();
    });
    let matched = crate::testing::capture_logs(|| {
        let _ = eval("get_cells core/*").unwrap();
    });

    assert_eq!(unmatched.len(), 1, "{unmatched:?}");
    assert!(
        unmatched[0].contains("no instance matches 'nosuch'"),
        "{unmatched:?}"
    );
    assert_eq!(quiet, Vec::<String>::new());
    assert_eq!(matched, Vec::<String>::new());
}

#[test]
fn clocks_match_by_glob_and_a_clock_matching_nothing_is_an_error_unless_quiet() {
    let clocks = "create_clock -name core_clock -period 2 [get_ports clk]\n";

    assert_eq!(
        eval(&format!("{clocks}get_clocks core*")).unwrap(),
        "_sta_clock:core_clock"
    );
    assert!(eval(&format!("{clocks}get_clocks nosuch")).is_err());
    assert_eq!(
        eval(&format!("{clocks}get_clocks -quiet nosuch")).unwrap(),
        ""
    );
}

#[test]
fn a_clock_without_ports_is_a_virtual_clock() {
    let sdc = parse("create_clock -name v -period 2\n");

    assert_eq!(sdc.clocks[ClockId(0)].name, "v");
    assert!(sdc.clocks[ClockId(0)].sources.is_empty());
}

#[test]
fn an_unnamed_clock_takes_its_first_port_name() {
    let sdc = parse("create_clock -period 2 [get_ports clk]\n");

    assert_eq!(sdc.clocks[ClockId(0)].name, "clk");
}

/// The error a clock of period 2 with `waveform` gives, if any.
fn waveform_error(waveform: &str) -> Option<String> {
    parse_str(
        &format!("create_clock -name c -period 2 -waveform {{{waveform}}} [get_ports clk]\n"),
        Path::new("t.sdc"),
        &design(),
        &liberty(),
    )
    .err()
    .map(|error| error.to_string())
}

#[test]
fn a_waveform_rises_then_falls_within_two_periods_as_opensta_checks() {
    assert_eq!(waveform_error("0 1"), None);
    assert_eq!(waveform_error("1 1"), None);
    assert_eq!(waveform_error("0 4"), None);
    assert!(waveform_error("1 0").is_some());
    assert!(waveform_error("0 5").is_some());
}

#[test]
fn a_port_delay_goes_only_on_ports_of_its_direction() {
    let db = design();
    let mut sdc = Sdc::default();
    let warnings = crate::testing::capture_logs(|| {
        sdc = parse(
            "create_clock -name a -period 2 [get_ports clk]\n\
             set_input_delay 1 -clock a [get_ports {out[0]}]\n",
        );
    });

    assert!(!sdc.input_delays.contains_key(&port(&db, "out[0]")));
    assert_eq!(warnings.len(), 1, "{warnings:?}");
    assert!(
        warnings[0].contains("set_input_delay not allowed on port 'out[0]'"),
        "{warnings:?}"
    );
}

#[test]
fn a_multicycle_path_counts_from_the_start_clock_with_start() {
    let sdc = parse(
        "create_clock -name a -period 2 [get_ports clk]\n\
         set_multicycle_path 2 -start -from [get_clocks a]\n",
    );

    assert!(!sdc.multicycle_paths[0].use_end_clock);
}

#[test]
fn a_multicycle_path_cannot_count_from_both_clocks() {
    let error = parse_error(
        "create_clock -name a -period 2 [get_ports clk]\n\
         set_multicycle_path 2 -start -end -from [get_clocks a]\n",
    );

    assert!(error.contains("cannot use -start with -end"), "{error}");
}

const TWO_CLOCKS: &str = "create_clock -name a -period 2 [get_ports clk]\n\
     create_clock -name b -period 4 [get_ports rst_n]\n";

#[test]
fn clock_groups_make_paths_between_different_groups_false() {
    let sdc = parse(&format!(
        "{TWO_CLOCKS}set_clock_groups -name g -comment {{two}} -asynchronous \
         -group [get_clocks a] -group [get_clocks b]\n"
    ));

    let paths: Vec<_> = sdc
        .false_paths
        .iter()
        .map(|f| (f.paths.from_clocks.clone(), f.paths.to_clocks.clone()))
        .collect();
    assert_eq!(
        paths,
        [
            (vec![ClockId(0)], vec![ClockId(1)]),
            (vec![ClockId(1)], vec![ClockId(0)])
        ]
    );
}

#[test]
fn clock_groups_that_allow_paths_add_no_false_path() {
    let sdc = parse(&format!(
        "{TWO_CLOCKS}set_clock_groups -allow_paths -asynchronous \
         -group [get_clocks a] -group [get_clocks b]\n"
    ));

    assert!(sdc.false_paths.is_empty());
}

#[test]
fn an_unknown_word_of_clock_groups_is_named() {
    let error = parse_error(&format!(
        "{TWO_CLOCKS}set_clock_groups -asynchronous -bogus -group [get_clocks a]\n"
    ));

    assert!(error.contains("unexpected '-bogus'"), "{error}");
}

#[test]
fn a_late_derate_leaves_the_early_side() {
    let sdc = parse("set_timing_derate -late 1.05\n");

    assert!((sdc.derating.cell[MinMax::Max] - 1.05).abs() < 1e-12);
    assert!((sdc.derating.cell[MinMax::Min] - 1.0).abs() < 1e-12);
}

#[test]
fn a_design_limit_on_ports_is_refused_and_on_the_design_is_kept() {
    let error = parse_error("set_max_fanout 20 [get_ports clk]\n");
    let sdc = parse("set_max_fanout 20 top\n");

    assert!(
        error.contains("set_max_fanout on ports is not supported"),
        "{error}"
    );
    assert_eq!(sdc.limits.max_fanout, Some(20.0));
}

#[test]
fn a_lone_dash_is_a_value_not_an_option() {
    assert!(!is_option("-"));
    assert!(is_option("-name"));
}

#[test]
fn a_unit_without_a_prefix_has_its_number_for_scale() {
    assert_eq!(unit_scale("1s", "s"), Some(1.0));
}

#[test]
fn leading_stars_match_an_empty_name() {
    assert!(glob_matches("*", ""));
    assert!(glob_matches("**", ""));
    assert!(!glob_matches("*a", ""));
}

#[test]
fn a_list_splits_at_blanks_keeping_braced_and_quoted_elements_whole() {
    for (text, items) in [
        ("a {b c} d", vec!["a", "b c", "d"]),
        ("x y \"b c\" z", vec!["x", "y", "b c", "z"]),
        ("\"a\\\"b\" c", vec!["a\"b", "c"]),
        ("a\\ b c", vec!["a b", "c"]),
        ("a\\nb", vec!["a\nb"]),
        ("{a\\}b} c", vec!["a\\}b", "c"]),
        ("a\\", vec!["a\\"]),
        ("\"a\\", vec!["a\\"]),
    ] {
        assert_eq!(split_list(text), items, "{text:?}");
    }
}

#[test]
fn a_list_braces_the_elements_that_need_it() {
    let items = |words: &[&str]| words.iter().map(|w| (*w).to_string()).collect::<Vec<_>>();

    assert_eq!(join_list(&items(&["", "a"])), "{} a");
    assert_eq!(join_list(&items(&["a b", "a$b", "c"])), "{a b} {a$b} c");
}

#[test]
fn braces_and_brackets_inside_quotes_and_quotes_inside_braces_are_plain_text() {
    for script in [
        "set v \"a{b\"\nset w 2",
        "set v \"a[b\"\nset w 2",
        "set v {a\"b}\nset w 2",
        "set v [list \"a]b\"\n c]\nset w 2",
        "set v [list {a]b}\n c]\nset w 2",
    ] {
        assert_eq!(split_commands(script, 1).len(), 2, "{script:?}");
    }
}

#[test]
fn a_script_may_end_in_a_comment_or_a_line_continuation() {
    assert_eq!(split_commands("set a 1\n# the end", 1).len(), 1);
    assert_eq!(split_commands("set a 1\n# the end \\", 1).len(), 1);
    assert_eq!(
        split_commands("set a 1 \\\n", 1),
        [("set a 1  ".to_string(), 1)]
    );
}

#[test]
fn an_escape_inside_a_quoted_element_is_kept_wherever_it_falls() {
    assert_eq!(split_list("\"ab\\\"c\" d"), ["ab\"c", "d"]);
}

#[test]
fn a_constraint_file_is_read_from_disk() {
    let path = std::env::temp_dir().join(format!("pare-sdc-{}.sdc", std::process::id()));
    std::fs::write(&path, "create_clock -name a -period 2 [get_ports clk]\n").unwrap();

    let sdc = parse_file(&path, &design(), &liberty());

    std::fs::remove_file(&path).unwrap();
    assert_eq!(sdc.unwrap().clocks.len(), 1);
}

#[test]
fn all_clocks_names_every_clock() {
    let script = format!("{TWO_CLOCKS}all_clocks");

    assert_eq!(eval(&script).unwrap(), "_sta_clock:a _sta_clock:b");
}

#[test]
fn a_transition_limit_takes_the_library_time_unit() {
    let sdc = parse("set_max_transition 0.5 top\n");

    let limit = sdc.limits.max_transition.unwrap().get();
    assert!((limit - 0.5e-9).abs() < 1e-18, "{limit}");
}

#[test]
fn an_input_transition_goes_on_the_edges_and_sides_its_flags_pick() {
    let db = design();
    let sdc = parse(
        "set_input_transition -rise -max 0.1 [get_ports rst_n]\n\
         set_input_transition -fall 0.2 [get_ports clk]\n",
    );

    let rst = &sdc.input_slews[&port(&db, "rst_n")];
    assert!(rst.get(Edge::Rise, MinMax::Max).is_some());
    assert_eq!(rst.get(Edge::Rise, MinMax::Min), None);
    assert_eq!(rst.get(Edge::Fall, MinMax::Max), None);
    let clk = &sdc.input_slews[&port(&db, "clk")];
    assert!(clk.get(Edge::Fall, MinMax::Min).is_some());
    assert_eq!(clk.get(Edge::Rise, MinMax::Min), None);
    assert_eq!(
        eval("set_input_transition 0.1 [get_ports clk]").unwrap(),
        ""
    );
}
