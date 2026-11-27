use super::*;

#[test]
fn logic_delays_are_the_delays_besides_tristate_enables() {
    let roles = [
        ArcRole::Combinational,
        ArcRole::RegClkToQ,
        ArcRole::RegSetClr,
        ArcRole::LatchEnToQ,
        ArcRole::TristateEnable,
        ArcRole::TristateDisable,
        ArcRole::Setup,
        ArcRole::Hold,
        ArcRole::Recovery,
        ArcRole::Removal,
        ArcRole::Check,
    ];

    let logic: Vec<ArcRole> = roles.into_iter().filter(|r| r.is_logic_delay()).collect();

    assert_eq!(
        logic,
        [
            ArcRole::Combinational,
            ArcRole::RegClkToQ,
            ArcRole::RegSetClr,
            ArcRole::LatchEnToQ
        ]
    );
    assert!(logic.iter().all(|r| r.is_delay()));
}

fn times(values: &[f64]) -> Vec<Seconds> {
    values.iter().copied().map(Seconds::new).collect()
}

fn table_7x7_like() -> Table {
    Table {
        axes: vec![
            Axis {
                variable: AxisVariable::InputTransition,
                values: vec![1.0, 2.0, 4.0],
            },
            Axis {
                variable: AxisVariable::OutputCapacitance,
                values: vec![10.0, 20.0],
            },
        ],
        values: times(&[0.0, 10.0, 1.0, 11.0, 3.0, 13.0]),
    }
}

#[test]
fn lookup_interpolates_bilinearly_inside_the_axes() {
    let t = table_7x7_like();
    assert!((t.lookup(Seconds::new(1.5), Farads::new(15.0)).get() - 5.5).abs() < 1e-12);
    assert!((t.lookup(Seconds::new(3.0), Farads::new(10.0)).get() - 2.0).abs() < 1e-12);
}

#[test]
fn lookup_extrapolates_from_the_end_pairs_outside_the_axes() {
    let t = table_7x7_like();
    assert!((t.lookup(Seconds::new(0.0), Farads::new(10.0)).get() - (-1.0)).abs() < 1e-12);
    assert!((t.lookup(Seconds::new(6.0), Farads::new(10.0)).get() - 5.0).abs() < 1e-12);
    assert!((t.lookup(Seconds::new(1.0), Farads::new(30.0)).get() - 20.0).abs() < 1e-12);
}

#[test]
fn lookup_returns_the_point_of_a_single_point_axis() {
    let t = Table {
        axes: vec![Axis {
            variable: AxisVariable::OutputCapacitance,
            values: vec![5.0],
        }],
        values: times(&[7.0]),
    };
    assert!((t.lookup(Seconds::new(0.0), Farads::new(99.0)).get() - 7.0).abs() < 1e-12);
    let scalar = Table {
        axes: Vec::new(),
        values: times(&[2.5]),
    };
    assert!((scalar.lookup(Seconds::new(1.0), Farads::new(1.0)).get() - 2.5).abs() < 1e-12);
}

#[test]
fn functions_parse_with_liberty_precedence() {
    let f = FuncExpr::parse("!(A1 & A2) | B ^ C").unwrap();
    assert_eq!(
        f,
        FuncExpr::Or(
            Box::new(FuncExpr::Not(Box::new(FuncExpr::And(
                Box::new(FuncExpr::Port("A1".into())),
                Box::new(FuncExpr::Port("A2".into()))
            )))),
            Box::new(FuncExpr::Xor(
                Box::new(FuncExpr::Port("B".into())),
                Box::new(FuncExpr::Port("C".into()))
            ))
        )
    );
    assert_eq!(
        FuncExpr::parse("A B'").unwrap(),
        FuncExpr::And(
            Box::new(FuncExpr::Port("A".into())),
            Box::new(FuncExpr::Not(Box::new(FuncExpr::Port("B".into()))))
        )
    );
    assert_eq!(FuncExpr::parse("0").unwrap(), FuncExpr::Zero);
    assert!(FuncExpr::parse("(A").is_err());
}

#[test]
fn evaluation_follows_three_valued_logic() {
    let f = FuncExpr::parse("(S & B) | (A & !S)").unwrap();
    let s_one = |p: &str| match p {
        "S" => Logic::One,
        _ => Logic::Unknown,
    };
    assert_eq!(f.eval(&s_one), Logic::Unknown);
    assert!(f.depends_on("B", &s_one));
    assert!(!f.depends_on("A", &s_one));
    let and = FuncExpr::parse("A & B").unwrap();
    let a_zero = |p: &str| {
        if p == "A" {
            Logic::Zero
        } else {
            Logic::Unknown
        }
    };
    assert_eq!(and.eval(&a_zero), Logic::Zero);
    assert!(!and.depends_on("B", &a_zero));
    assert!(and.depends_on("B", &|_| Logic::Unknown));
}

fn port(name: &str, direction: PortDirection, function: Option<&str>) -> LibPort {
    LibPort {
        name: name.into(),
        direction,
        capacitance: EdgeValues::both(Farads::new(1e-15)),
        max_capacitance: None,
        max_transition: None,
        max_fanout: None,
        fanout_load: None,
        function: function.map(|f| FuncExpr::parse(f).unwrap()),
        three_state: None,
        is_clock: false,
        clock_gate: ClockGatePin::None,
        clock_tree_paths: Vec::new(),
    }
}

fn cell(name: &str, ports: Vec<LibPort>) -> LibCell {
    LibCell {
        name: name.into(),
        area: CellArea::new(1.0),
        dont_use: false,
        dont_touch: false,
        is_clock_gate: false,
        is_clock_cell: false,
        is_power_cell: false,
        is_isolation_cell: false,
        is_pad: false,
        library: 0,
        footprint: None,
        user_function_class: None,
        ports: ports.into(),
        pg_pins: vec!["VDD".into(), "VSS".into()],
        arc_sets: IdVec::new(),
        sequentials: Vec::new(),
        statetable: None,
        leakage: None,
        state_leakages: Vec::new(),
    }
}

#[test]
fn a_buffer_has_one_input_passed_to_one_output() {
    let buf = cell(
        "BUF",
        vec![
            port("A", PortDirection::Input, None),
            port("Z", PortDirection::Output, Some("A")),
        ],
    );
    let inv = cell(
        "INV",
        vec![
            port("A", PortDirection::Input, None),
            port("ZN", PortDirection::Output, Some("!A")),
        ],
    );
    let and = cell(
        "AND",
        vec![
            port("A1", PortDirection::Input, None),
            port("A2", PortDirection::Input, None),
            port("ZN", PortDirection::Output, Some("A1 & A2")),
        ],
    );
    assert!(buf.is_buffer() && !buf.is_inverter());
    assert!(inv.is_inverter() && !inv.is_buffer());
    assert!(!and.is_buffer() && and.buffer_ports().is_none());
}

#[test]
fn cells_with_the_same_pins_and_functions_are_equivalent() {
    let x1 = cell(
        "BUF_X1",
        vec![
            port("A", PortDirection::Input, None),
            port("Z", PortDirection::Output, Some("A")),
        ],
    );
    let mut x2 = x1.clone();
    x2.name = "BUF_X2".into();
    x2.area = CellArea::new(2.0);
    let mut clkbuf = x1.clone();
    clkbuf.name = "CLKBUF_X1".into();
    let inv = cell(
        "INV_X1",
        vec![
            port("A", PortDirection::Input, None),
            port("ZN", PortDirection::Output, Some("!A")),
        ],
    );
    assert!(x1.equivalent(&x2));
    assert!(x1.equivalent(&clkbuf));
    assert!(!x1.equivalent(&inv));
}

#[test]
fn drive_resistance_is_the_slew_over_the_largest_table_load() {
    let arc = TimingArc {
        from_edge: Edge::Rise,
        to_edge: Edge::Rise,
        delay: None,
        slew: Some(Table {
            axes: vec![
                Axis {
                    variable: AxisVariable::InputTransition,
                    values: vec![0.0, 1.0],
                },
                Axis {
                    variable: AxisVariable::OutputCapacitance,
                    values: vec![1.0, 4.0],
                },
            ],
            values: times(&[1.0, 8.0, 2.0, 9.0]),
        }),
    };
    assert!((arc.drive_resistance().get() - 2.0).abs() < 1e-12);
}

fn sense(function: &str, port: &str) -> Option<TimingSense> {
    FuncExpr::parse(function).unwrap().port_timing_sense(port)
}

#[test]
fn an_inverted_conjunction_is_negative_unate_in_each_input() {
    assert_eq!(sense("!(A & B)", "A"), Some(TimingSense::NegativeUnate));
    assert_eq!(sense("!(A & B)", "B"), Some(TimingSense::NegativeUnate));
}

#[test]
fn a_pin_read_both_ways_or_through_an_xor_is_non_unate() {
    assert_eq!(
        sense("(S & B) | (A & !S)", "S"),
        Some(TimingSense::NonUnate)
    );
    assert_eq!(sense("A ^ B", "B"), Some(TimingSense::NonUnate));
}

#[test]
fn a_pin_the_function_does_not_read_has_no_sense() {
    assert_eq!(sense("A & B", "C"), None);
    assert_eq!(sense("A | (B & 1)", "B"), Some(TimingSense::PositiveUnate));
}

#[test]
fn the_min_side_prefers_the_smaller_and_the_max_side_the_larger() {
    assert!(MinMax::Min.beats(1, 2));
    assert!(!MinMax::Min.beats(2, 1));
    assert!(!MinMax::Min.beats(1, 1));
    assert!(MinMax::Max.beats(2, 1));
    assert!(!MinMax::Max.beats(1, 2));
    assert!(!MinMax::Max.beats(1, 1));
}

fn axis(values: &[f64]) -> Axis {
    Axis {
        variable: AxisVariable::InputTransition,
        values: values.to_vec(),
    }
}

#[test]
fn a_lookup_interpolates_between_the_points_around_the_value() {
    let a = axis(&[1.0, 2.0, 4.0, 8.0]);

    assert_eq!(a.lower_index(3.0), 1);
    assert_eq!(a.lower_index(4.0), 2);
    assert_eq!(a.lower_index(0.5), 0);
    assert_eq!(a.lower_index(9.0), 2);
    assert_eq!(axis(&[5.0]).lower_index(9.0), 0);
}

#[test]
fn a_table_is_consistent_only_with_a_value_per_point_of_at_most_three_axes() {
    let mut short = table_7x7_like();
    short.values.truncate(5);
    let empty = Table {
        axes: Vec::new(),
        values: Vec::new(),
    };
    let four_axes = Table {
        axes: vec![axis(&[1.0]); 4],
        values: times(&[1.0]),
    };

    assert!(table_7x7_like().is_consistent());
    assert!(!short.is_consistent());
    assert!(!empty.is_consistent());
    assert!(!four_axes.is_consistent());
}

#[test]
fn only_the_output_capacitance_axes_give_capacitance_points() {
    let points: Vec<f64> = table_7x7_like()
        .capacitance_points()
        .map(Farads::get)
        .collect();

    assert_eq!(points, [10.0, 20.0]);
}

#[test]
fn a_lookup_reads_the_related_capacitance_along_its_axis() {
    let t = Table {
        axes: vec![Axis {
            variable: AxisVariable::RelatedOutputCapacitance,
            values: vec![0.0, 10.0],
        }],
        values: times(&[1.0, 3.0]),
    };

    let value = t.lookup_with_related(Seconds::ZERO, Farads::ZERO, Farads::new(5.0));

    assert!((value.get() - 2.0).abs() < 1e-12, "{value:?}");
}

#[test]
fn timing_checks_and_set_or_clear_arcs_are_not_searched() {
    for role in [
        ArcRole::Setup,
        ArcRole::Hold,
        ArcRole::RegSetClr,
        ArcRole::TristateDisable,
    ] {
        assert!(!role.is_searched(), "{role:?}");
    }
    assert!(ArcRole::Combinational.is_searched());
}

#[test]
fn a_pin_read_the_same_way_twice_keeps_its_sense() {
    assert_eq!(sense("A & A", "A"), Some(TimingSense::PositiveUnate));
}

fn flop() -> Sequential {
    Sequential {
        kind: SequentialKind::FlipFlop,
        output: "IQ".into(),
        output_inv: Some("IQN".into()),
        clock: Some(FuncExpr::parse("CK").unwrap()),
        data: Some(FuncExpr::parse("D").unwrap()),
        clear: Some(FuncExpr::parse("!RN").unwrap()),
        preset: None,
    }
}

#[test]
fn storage_is_equivalent_only_when_every_part_matches() {
    let same = flop();
    let variants = [
        Sequential {
            kind: SequentialKind::Latch,
            ..flop()
        },
        Sequential {
            output: "Q".into(),
            ..flop()
        },
        Sequential {
            output_inv: None,
            ..flop()
        },
        Sequential {
            clock: Some(FuncExpr::parse("!CK").unwrap()),
            ..flop()
        },
        Sequential {
            data: None,
            ..flop()
        },
        Sequential {
            clear: None,
            ..flop()
        },
        Sequential {
            preset: Some(FuncExpr::parse("!SN").unwrap()),
            ..flop()
        },
    ];

    assert!(flop().equiv(&same));
    for variant in &variants {
        assert!(!flop().equiv(variant), "{variant:?}");
    }
}

fn buffer(name: &str) -> LibCell {
    cell(
        name,
        vec![
            port("A", PortDirection::Input, None),
            port("Z", PortDirection::Output, Some("A")),
        ],
    )
}

#[test]
fn a_cell_holds_state_when_it_has_storage() {
    let mut reg = buffer("REG");
    reg.sequentials.push(flop());

    assert!(reg.is_sequential());
    assert!(!buffer("BUF").is_sequential());
}

#[test]
fn a_cell_has_functions_when_any_pin_has_one() {
    let blank = cell("BLANK", vec![port("A", PortDirection::Input, None)]);

    assert!(buffer("BUF").has_functions());
    assert!(!blank.has_functions());
}

#[test]
fn cells_differing_in_storage_power_pins_or_a_pin_are_not_equivalent() {
    let mut stateful = buffer("REG");
    stateful.sequentials.push(flop());
    let mut other_power = buffer("BUF_P");
    other_power.pg_pins = vec!["VDDX".into(), "VSS".into()];
    let mut fewer_power = buffer("BUF_1P");
    fewer_power.pg_pins.truncate(1);
    let mut tristate = buffer("TBUF");
    tristate.ports[LibPortId::new(1)].three_state = Some(FuncExpr::parse("!EN").unwrap());
    let mut flipped = buffer("BUF_IN");
    flipped.ports[LibPortId::new(0)].direction = PortDirection::Output;

    for other in [&stateful, &other_power, &fewer_power, &tristate, &flipped] {
        assert!(!buffer("BUF").equivalent(other), "{}", other.name);
    }
    let mut also_stateful = buffer("REG_X2");
    also_stateful.sequentials.push(flop());
    assert!(stateful.equivalent(&also_stateful));
}

/// A cell of pins `A` and `Z` without functions, its arcs from `A` to `Z`
/// of the given roles, senses and conditions.
fn functionless(arcs: &[(ArcRole, TimingSense, Option<&str>)]) -> LibCell {
    let mut c = cell(
        "MACRO",
        vec![
            port("A", PortDirection::Input, None),
            port("Z", PortDirection::Output, None),
        ],
    );
    for &(role, sense, when) in arcs {
        let _ = c.arc_sets.push(TimingArcSet {
            from: LibPortId::new(0),
            to: LibPortId::new(1),
            role,
            sense,
            when: when.map(|w| FuncExpr::parse(w).unwrap()),
            arcs: Vec::new(),
        });
    }
    c
}

#[test]
fn cells_without_functions_are_equivalent_only_with_the_same_arcs() {
    let base = [(
        ArcRole::Combinational,
        TimingSense::PositiveUnate,
        Some("B"),
    )];
    let variants: [&[(ArcRole, TimingSense, Option<&str>)]; 5] = [
        &[(ArcRole::RegClkToQ, TimingSense::PositiveUnate, Some("B"))],
        &[(
            ArcRole::Combinational,
            TimingSense::NegativeUnate,
            Some("B"),
        )],
        &[(ArcRole::Combinational, TimingSense::PositiveUnate, None)],
        &[],
        &[
            (
                ArcRole::Combinational,
                TimingSense::PositiveUnate,
                Some("B"),
            ),
            (ArcRole::Setup, TimingSense::NonUnate, None),
        ],
    ];

    assert!(functionless(&base).equivalent(&functionless(&base)));
    for arcs in variants {
        assert!(
            !functionless(&base).equivalent(&functionless(arcs)),
            "{arcs:?}"
        );
    }
}

#[test]
fn arcs_match_by_their_pins_names_whatever_their_order() {
    let mut reordered = functionless(&[(ArcRole::Combinational, TimingSense::PositiveUnate, None)]);
    reordered.ports = vec![
        port("Z", PortDirection::Output, None),
        port("A", PortDirection::Input, None),
    ]
    .into();
    reordered.arc_sets[ArcSetId::new(0)].from = LibPortId::new(1);
    reordered.arc_sets[ArcSetId::new(0)].to = LibPortId::new(0);
    let base = functionless(&[(ArcRole::Combinational, TimingSense::PositiveUnate, None)]);
    let mut backwards = base.clone();
    backwards.arc_sets[ArcSetId::new(0)].from = LibPortId::new(1);
    backwards.arc_sets[ArcSetId::new(0)].to = LibPortId::new(0);

    assert!(base.equivalent(&reordered));
    assert!(!base.equivalent(&backwards));
}

#[test]
fn the_delay_arcs_into_a_pin_leave_out_its_checks_and_other_pins() {
    let c = functionless(&[
        (ArcRole::Combinational, TimingSense::PositiveUnate, None),
        (ArcRole::Setup, TimingSense::NonUnate, None),
    ]);

    let into_z: Vec<ArcRole> = c
        .delay_arc_sets_to(LibPortId::new(1))
        .map(|s| s.role)
        .collect();

    assert_eq!(into_z, [ArcRole::Combinational]);
    assert_eq!(c.delay_arc_sets_to(LibPortId::new(0)).count(), 0);
}

const LIMITS_LIB: &str = r#"
library (limits) {
  time_unit : "1ns" ;
  capacitive_load_unit (1, ff) ;
  default_max_transition : 0.5 ;
  default_max_capacitance : 40 ;
  cell (BUF) {
    area : 1 ;
    pin (A) { direction : input ; capacitance : 1 ; }
    pin (Z) { direction : output ; function : "A" ; }
  }
}
"#;

#[test]
fn library_limits_apply_to_outputs_only() {
    let lib =
        crate::db::parser::liberty::parse_str(LIMITS_LIB, std::path::Path::new("l.lib")).unwrap();
    let buf = lib.cell(lib.cell_id("BUF").unwrap());
    let pin = |name: &str| &buf.ports[buf.port(name).unwrap()];

    assert_eq!(lib.slew_limit(buf, pin("A")), None);
    assert_eq!(lib.capacitance_limit(buf, pin("A")), None);
    let slew = lib.slew_limit(buf, pin("Z")).unwrap().get();
    let cap = lib.capacitance_limit(buf, pin("Z")).unwrap().get();
    assert!((slew - 0.5e-9).abs() < 1e-18, "{slew}");
    assert!((cap - 40e-15).abs() < 1e-24, "{cap}");
}

#[test]
fn three_valued_logic_follows_its_truth_tables() {
    use Logic::{One, Unknown, Zero};
    let levels = [Zero, One, Unknown];
    let and = [
        [Zero, Zero, Zero],
        [Zero, One, Unknown],
        [Zero, Unknown, Unknown],
    ];
    let or = [
        [Zero, One, Unknown],
        [One, One, One],
        [Unknown, One, Unknown],
    ];
    let xor = [
        [Zero, One, Unknown],
        [One, Zero, Unknown],
        [Unknown, Unknown, Unknown],
    ];

    for (i, a) in levels.into_iter().enumerate() {
        for (j, b) in levels.into_iter().enumerate() {
            assert_eq!(a.and(b), and[i][j], "{a:?} and {b:?}");
            assert_eq!(a.or(b), or[i][j], "{a:?} or {b:?}");
            assert_eq!(a.xor(b), xor[i][j], "{a:?} xor {b:?}");
        }
    }
}

#[test]
fn expressions_are_equivalent_only_with_the_same_shape() {
    let a = FuncExpr::parse("A").unwrap();
    let b = FuncExpr::parse("B").unwrap();

    assert!(!a.equiv(&b));
    assert!(FuncExpr::equiv_opt(None, None));
    assert!(!FuncExpr::equiv_opt(Some(&a), None));
    assert!(!FuncExpr::equiv_opt(Some(&a), Some(&b)));
}

#[test]
fn only_zero_and_one_are_constant_expressions() {
    assert!(FuncExpr::Zero.is_constant());
    assert!(FuncExpr::One.is_constant());
    assert!(!FuncExpr::parse("A").unwrap().is_constant());
}

#[test]
fn a_pin_counts_when_some_levels_of_the_unknown_pins_let_it_through() {
    let unknown = |_: &str| Logic::Unknown;

    assert!(
        FuncExpr::parse("A & B & C")
            .unwrap()
            .depends_on("C", &unknown)
    );
    assert!(
        FuncExpr::parse("A & !B & C")
            .unwrap()
            .depends_on("C", &unknown)
    );
    assert!(
        !FuncExpr::parse("A & !A & C")
            .unwrap()
            .depends_on("C", &unknown)
    );
}

#[test]
fn a_known_pin_stays_at_its_level() {
    let b_zero = |p: &str| {
        if p == "B" {
            Logic::Zero
        } else {
            Logic::Unknown
        }
    };

    assert!(!FuncExpr::parse("B & C").unwrap().depends_on("C", &b_zero));
}

/// The conjunction of `n` pins `P0`, `P1`, ... and a pin `Q`: `n + 1` pins.
fn wide_and(n: usize) -> FuncExpr {
    let pins: Vec<String> = (0..n).map(|i| format!("P{i}")).collect();
    FuncExpr::parse(&format!("{} & Q", pins.join(" & "))).unwrap()
}

#[test]
fn too_many_unknown_pins_count_as_a_dependence_without_trying_them() {
    let unknown = |_: &str| Logic::Unknown;

    assert!(!wide_and(MAX_FREE_PINS - 1).depends_on("R", &unknown));
    assert!(wide_and(MAX_FREE_PINS).depends_on("R", &unknown));
}

#[test]
fn an_expression_prints_with_its_grouping() {
    let f = FuncExpr::parse("!A & (B | C)").unwrap();

    assert_eq!(f.to_string(), "(!A & (B | C))");
}

#[test]
fn a_parenthesis_or_negation_after_a_term_ands_it() {
    assert_eq!(
        FuncExpr::parse("A (B | C)").unwrap(),
        FuncExpr::parse("A & (B | C)").unwrap()
    );
    assert_eq!(
        FuncExpr::parse("A !B").unwrap(),
        FuncExpr::parse("A & !B").unwrap()
    );
}

#[test]
fn one_is_a_constant_and_an_operator_cannot_start_a_term() {
    assert_eq!(FuncExpr::parse("1").unwrap(), FuncExpr::One);
    assert!(FuncExpr::parse("& A").is_err());
}

#[test]
fn names_start_with_letters_digits_or_bus_and_path_characters() {
    for c in ['a', 'Z', '0', '_', '[', ']', '.', '\\'] {
        assert!(FuncParser::starts_name(c), "{c:?}");
    }
    for c in ['&', '(', '!', ' ', '^'] {
        assert!(!FuncParser::starts_name(c), "{c:?}");
    }
}
