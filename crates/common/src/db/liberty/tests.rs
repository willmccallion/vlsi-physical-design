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
