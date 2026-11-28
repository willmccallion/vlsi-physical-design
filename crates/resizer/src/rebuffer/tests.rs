use super::*;
use crate::bnet::{Kind, PinLoad};
use crate::resizer::GlobalRouting;
use crate::route_parasitics::RouteRc;
use crate::testing::{
    cells, clock_cells_source, design, metal2_wires, pin, with_session_inputs, with_timed_session,
    with_wired_session,
};
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::{NetRoute, RouteSource};
use pare_common::db::indices::NetId;
use pare_common::units::{CellArea, FaradsPerMeter, OhmsPerMeter};
use std::rc::Rc;

/// A flop launching into another, both clocked through a clock buffer
/// whose output slew reaches their clock pins.
fn buffered_clock_pair() -> NetlistDB {
    design(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN pair ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 3 ;\n- cb CLKBUF_X3 + PLACED ( 2000 2800 ) N ;\n\
             - f1 DFF_X1 + PLACED ( 12000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 40000 2800 ) N ;\n\
             END COMPONENTS\nPINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 3 ;\n- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( f1 CK ) ( f2 CK ) ;\n\
             - q ( f1 Q ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n",
    )
}

#[test]
fn rebuffering_sees_every_load_of_a_net_when_two_share_a_place() {
    let mut db = design(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN fork ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 4 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - b1 BUF_X1 + PLACED ( 30000 2800 ) N ;\n- b2 BUF_X1 + PLACED ( 30000 2800 ) N ;\n\
             - b3 BUF_X1 + PLACED ( 50000 2800 ) N ;\nEND COMPONENTS\n\
             PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 2 ;\n- clk ( PIN clk ) ( f1 CK ) ;\n\
             - q ( f1 Q ) ( b1 A ) ( b2 A ) ( b3 A ) ;\nEND NETS\nEND DESIGN\n",
    );
    let q = pin(&db, "f1/Q");
    let clock = "create_clock -name clk -period 0.5 [get_ports clk]\n";

    with_timed_session(&mut db, &cells(&clock_cells_source()), clock, |resizer| {
        let tree = resizer.buffered_tree(q).unwrap().unwrap();
        let mut sinks = Vec::new();
        collect_sinks(&tree, &mut sinks);
        let mut names: Vec<String> = sinks
            .iter()
            .filter_map(|s| match s.kind {
                Kind::Load { pin } => Some(resizer.db.pin_name(pin)),
                _ => None,
            })
            .collect();
        names.sort();

        assert_eq!(names, ["b1/A", "b2/A", "b3/A"]);
    });
}

/// A flop launching through buffer `bf`, placed with `status`, into
/// another.
fn buffer_between_flops(status: &str) -> NetlistDB {
    design(&format!(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN between ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 3 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - bf BUF_X1 + {status} ( 30000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 50000 2800 ) N ;\n\
             END COMPONENTS\nPINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 3 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n\
             - q ( f1 Q ) ( bf A ) ;\n- d ( bf Z ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n"
    ))
}

/// Whether setup repair may take buffer `bf` out of `db` under
/// `constraints`.
fn removable_under(mut db: NetlistDB, constraints: &str) -> bool {
    let mut removable = false;
    with_timed_session(&mut db, &cells(&clock_cells_source()), constraints, |r| {
        removable = can_remove_buffer(r, r.db.cell_id("bf").unwrap());
    });
    removable
}

const CLOCK: &str = "create_clock -name clk -period 0.5 [get_ports clk]\n";

#[test]
fn a_buffer_that_cannot_drive_its_own_input_within_its_slew_limit_is_refused() {
    let mut db = buffer_between_flops("PLACED");
    let tight = clock_cells_source().replacen(
        "capacitance : 0.97 ;",
        "capacitance : 0.97 ; max_transition : 0.000001 ;",
        1,
    );

    with_wired_session(&mut db, &cells(&tight), CLOCK, |resizer| {
        let error = Rebuffer::new(resizer).err();

        assert_eq!(
            error,
            Some(RepairError::WireLengthLimit("BUF_X1".to_string()))
        );
    });
}

#[test]
fn rebuffering_a_whole_design_replaces_a_buffer_chain_and_keeps_it_timed() {
    let mut db = design(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN chain ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 800000 40000 ) ;\n\
             COMPONENTS 4 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - b1 BUF_X1 + PLACED ( 4000 2800 ) N ;\n- b2 BUF_X1 + PLACED ( 6000 2800 ) N ;\n\
             - f2 DFF_X1 + PLACED ( 602000 2800 ) N ;\nEND COMPONENTS\n\
             PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 4 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( b1 A ) ;\n\
             - m ( b1 Z ) ( b2 A ) ;\n- d ( b2 Z ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n",
    );
    let lib = cells(&clock_cells_source());
    let d = pin(&db, "f2/D");

    with_wired_session(&mut db, &lib, CLOCK, |resizer| {
        let mut rebuffer = Rebuffer::new(resizer).unwrap();

        rebuffer.fully_rebuffer(resizer).unwrap();

        assert!(rebuffer.removed > 0, "{rebuffer:?}");
        assert!(resizer.sta().slack(d, MinMax::Max).time().is_some());
        let netlist = resizer.netlist();
        let q = pin(resizer.db, "f1/Q");
        assert!(netlist.loads(resizer.db.pins[q].net).count() > 0);
    });
}

#[test]
fn a_placed_buffer_may_be_taken_out_and_a_fixed_one_may_not() {
    assert!(removable_under(buffer_between_flops("PLACED"), CLOCK));
    assert!(!removable_under(buffer_between_flops("FIXED"), CLOCK));
}

#[test]
fn a_buffer_between_constrained_ports_may_be_taken_out() {
    let db = design(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN ports ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 1 ;\n- bf BUF_X1 + PLACED ( 30000 2800 ) N ;\nEND COMPONENTS\n\
             PINS 3 ;\n- clk + NET clk + DIRECTION INPUT ;\n- in + NET in + DIRECTION INPUT ;\n\
             - out + NET out + DIRECTION OUTPUT ;\nEND PINS\n\
             NETS 3 ;\n- clk ( PIN clk ) ;\n- in ( PIN in ) ( bf A ) ;\n\
             - out ( bf Z ) ( PIN out ) ;\nEND NETS\nEND DESIGN\n",
    );
    let delays = format!(
        "{CLOCK}set_input_delay 0.1 -clock clk [get_ports in]\n\
             set_output_delay 0.1 -clock clk [get_ports out]\n"
    );

    assert!(removable_under(db, &delays));
}

#[test]
fn a_buffer_with_its_own_derate_may_not_be_taken_out() {
    let derated = format!("{CLOCK}set_timing_derate -late 1.1 [get_cells bf]\n");

    assert!(!removable_under(buffer_between_flops("PLACED"), &derated));
}

/// A load of `cap_ff` femtofarads at the origin with `slack_ns`
/// nanoseconds of slack.
fn load_with(pin: PinId, cap_ff: f64, slack_ns: f64) -> BnetPtr {
    let load = PinLoad {
        cap: Farads::new(cap_ff * 1e-15),
        fanout: 1.0,
        max_slew: Seconds::INFINITY,
    };
    let slack = Fixed::from_seconds(Seconds::new(slack_ns * 1e-9));
    Bnet::load(Point::new(Dbu::ZERO, Dbu::ZERO), pin, load).with_slack(slack, None)
}

/// Whether a junction pulls a side under a buffer of its own making.
fn is_rewritten(junction: &Bnet) -> bool {
    matches!(&junction.kind, Kind::Junction { left, .. }
            if matches!(&left.kind, Kind::Buffer { to, .. }
                if matches!(to.under_wires().kind, Kind::Junction { .. })))
}

#[test]
fn a_junction_rewrites_its_topology_only_where_allowed() {
    let mut db = buffer_between_flops("PLACED");
    let pin = pin(&db, "f2/D");

    with_timed_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let rebuffer = Rebuffer::new(resizer).unwrap();
        let critical = Bnet::junction(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            load_with(pin, 1.0, 1.0),
            load_with(pin, 5.0, 5.0),
        );
        let buffered = Bnet::buffer(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            &rebuffer.sizes[0],
            load_with(pin, 5.0, 10.0),
            Fixed::ZERO,
        );
        let node = Bnet::junction(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            Rc::clone(&critical),
            Rc::clone(&buffered),
        );
        let (left, right) = ([critical], [buffered]);
        let sides = JunctionSides {
            left: &left,
            right: &right,
        };

        let rewritable = rebuffer.junction_options(resizer, &node, sides, Topology::Rewritable);
        let kept = rebuffer.junction_options(resizer, &node, sides, Topology::Kept);

        assert!(rewritable.iter().any(|o| is_rewritten(o)));
        assert!(!kept.iter().any(|o| is_rewritten(o)));
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
fn rebuffering_rewrites_topology_on_placement_parasitics_only() {
    let mut db = buffer_between_flops("PLACED");
    let lib = cells(&clock_cells_source());
    let rc = RouteRc::new(&db.library, &[], &[], LayerId(0));
    let wires = metal2_wires(&db);

    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        assert_eq!(Topology::of_session(resizer), Topology::Rewritable);
    });
    with_session_inputs(&mut db, &lib, CLOCK, wires, |inputs, sdc| {
        let mut source = NoRoutes;
        let routes = GlobalRouting {
            source: &mut source,
            rc,
            legalize: &|_, _| {},
        };
        let resizer = Resizer::on_global_routes(inputs, sdc, routes).unwrap();

        assert_eq!(Topology::of_session(&resizer), Topology::Kept);
    });
}

#[test]
fn a_flop_on_a_propagated_clock_is_rebuffered_from_its_clock_pin_slew() {
    let mut db = buffered_clock_pair();
    let q = pin(&db, "f1/Q");
    let constraints = "create_clock -name clk -period 0.5 [get_ports clk]\n\
                           set_propagated_clock [all_clocks]\n";

    with_timed_session(
        &mut db,
        &cells(&clock_cells_source()),
        constraints,
        |resizer| {
            let mut rebuffer = Rebuffer::new(resizer).unwrap();
            assert!(rebuffer.set_driver(resizer, q));
            let tree = resizer.buffered_tree(q).unwrap().unwrap();
            let tree = rebuffer.annotate_load_slacks(resizer, &tree, q);

            let seen = rebuffer.slack_at_driver(resizer, &tree).to_seconds();
            let timed = resizer.sta().slack(q, MinMax::Max).time().unwrap();

            assert!(
                (seen - timed).abs() < Seconds::new(1e-14),
                "{seen:?} vs {timed:?}"
            );
        },
    );
}

/// `femtoseconds` as a [`Fixed`] time.
fn fs(femtoseconds: i64) -> Fixed {
    let bias = if femtoseconds < 0 { -0.25 } else { 0.25 };
    Fixed::from_seconds(Seconds::new((femtoseconds as f64 + bias) * 1e-15))
}

/// A load node of `cap_ff` femtofarads and `slack` femtoseconds of slack,
/// at the origin.
fn raw_load(pin: PinId, cap_ff: f64, slack: i64) -> BnetPtr {
    Rc::new(Bnet {
        kind: Kind::Load { pin },
        at: Point::new(Dbu::ZERO, Dbu::ZERO),
        cap: Farads::new(cap_ff * 1e-15),
        fanout: 1.0,
        max_load_slew: Seconds::INFINITY,
        area: CellArea::ZERO,
        slack: fs(slack),
        slack_edges: None,
        delay: Fixed::ZERO,
        arrival_delay: Fixed::ZERO,
    })
}

/// A node of `kind` whose own delay is `delay` femtoseconds.
fn raw_node(kind: Kind, delay: i64) -> BnetPtr {
    Rc::new(Bnet {
        kind,
        at: Point::new(Dbu::ZERO, Dbu::ZERO),
        cap: Farads::ZERO,
        fanout: 0.0,
        max_load_slew: Seconds::INFINITY,
        area: CellArea::ZERO,
        slack: Fixed::ZERO,
        slack_edges: None,
        delay: fs(delay),
        arrival_delay: Fixed::ZERO,
    })
}

/// A rebuffering search over the buffers of a session on
/// [`buffer_between_flops`] with wires of [`metal2_wires`].
fn with_search(test: impl FnOnce(&mut Resizer<'_, Sta>, &mut Rebuffer)) {
    let mut db = buffer_between_flops("PLACED");
    with_wired_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let mut rebuffer = Rebuffer::new(resizer).unwrap();
        test(resizer, &mut rebuffer);
    });
}

/// Whether `strong` can drive `tree` and `extra` of wire more, by whether a
/// timing frontier keeps it.
fn is_driven(resizer: &Resizer<'_, Sta>, strong: &BufferSize, tree: &BnetPtr, extra: Dbu) -> bool {
    let mut frontier = Frontier::new(None, 1);
    frontier.offer(resizer, strong, tree, extra);
    !frontier.kept.is_empty()
}

#[test]
fn a_buffer_drives_a_tree_and_extra_wire_only_within_its_slew_limit() {
    with_search(|resizer, rebuffer| {
        // A wire of 10 um to a load of 10 fF and 1 ns of slew limit, over
        // wires of 3.5 ohms and 0.1 fF a micron at 2000 units a micron.
        let load = Bnet::load(
            Point::new(Dbu::new(20_000), Dbu::ZERO),
            pin(resizer.db, "f2/D"),
            PinLoad {
                cap: Farads::new(10e-15),
                fanout: 1.0,
                max_slew: Seconds::new(1e-9),
            },
        );
        let tree = Bnet::wire(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            load,
            None,
            &resizer.tree_rc(),
        );
        let extra = Dbu::new(40_000);

        // Adding 20 um of extra wire: 30 um of wire is 105 ohms and the
        // load is the tree's 11 fF, the extra wire's 2 fF and the buffer's
        // 2 fF, 15 fF. The limit is 0.8 of 1 ns, so the drive resistance
        // that reaches it is 0.8 ns / (15 fF * 1.39) less the wire's 105.
        let reaching = 0.8e-9 / (15e-15 * 1.39) - 105.0;
        let size = |drive: f64| BufferSize {
            drive_resistance: Ohms::new(drive),
            output_cap: Farads::new(2e-15),
            margined_max_cap: Farads::new(1e-12),
            ..rebuffer.sizes[0]
        };

        assert!(is_driven(resizer, &size(reaching * 0.999), &tree, extra));
        assert!(!is_driven(resizer, &size(reaching * 1.001), &tree, extra));
    });
}

#[test]
fn a_buffer_drives_a_tree_and_extra_wire_only_within_its_load_limit() {
    with_search(|resizer, rebuffer| {
        let load = Bnet::load(
            Point::new(Dbu::new(20_000), Dbu::ZERO),
            pin(resizer.db, "f2/D"),
            PinLoad {
                cap: Farads::new(10e-15),
                fanout: 1.0,
                max_slew: Seconds::new(1e-9),
            },
        );
        let tree = Bnet::wire(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            load,
            None,
            &resizer.tree_rc(),
        );
        let size = |max_cap_ff: f64| BufferSize {
            drive_resistance: Ohms::new(1.0),
            output_cap: Farads::new(2e-15),
            margined_max_cap: Farads::new(max_cap_ff * 1e-15),
            ..rebuffer.sizes[0]
        };

        // The tree's 11 fF, the extra 20 um wire's 2 fF and the buffer's
        // 2 fF come to 15 fF.
        assert!(is_driven(resizer, &size(15.01), &tree, Dbu::new(40_000)));
        assert!(!is_driven(resizer, &size(14.99), &tree, Dbu::new(40_000)));
    });
}

#[test]
fn an_assured_option_goes_in_before_the_first_that_is_as_heavy() {
    let options: Vec<BnetPtr> = [1.0, 3.0, 5.0]
        .iter()
        .map(|&cap| raw_load(PinId::new(0), cap, 0))
        .collect();
    let position_of_assured = |cap: f64| {
        let mut kept = options.clone();
        let assured = raw_load(PinId::new(1), cap, 0);
        insert_assured(&mut kept, Rc::clone(&assured));
        assert_eq!(kept.len(), 4);
        kept.iter().position(|o| Rc::ptr_eq(o, &assured)).unwrap()
    };

    assert_eq!(position_of_assured(4.0), 2);
    assert_eq!(position_of_assured(3.0), 1);
    assert_eq!(position_of_assured(0.5), 0);
    assert_eq!(position_of_assured(9.0), 3);
}

#[test]
fn each_node_below_a_wire_via_or_buffer_arrives_after_the_delays_above_it() {
    with_search(|_, rebuffer| {
        let load = raw_load(PinId::new(0), 1.0, 0);
        let buffer = raw_node(
            Kind::Buffer {
                buffer: rebuffer.sizes[0].buffer,
                to: load,
            },
            30,
        );
        let via = raw_node(
            Kind::Via {
                to: buffer,
                layer: LayerId::new(0),
                to_layer: LayerId::new(1),
            },
            20,
        );
        let wire = raw_node(
            Kind::Wire {
                to: via,
                layer: None,
            },
            10,
        );
        let junction = raw_node(
            Kind::Junction {
                left: wire,
                right: raw_load(PinId::new(1), 1.0, 0),
            },
            0,
        );

        let timed = with_arrivals(&junction, fs(100));

        let mut arrivals = vec![timed.arrival_delay];
        let Kind::Junction { left, right } = &timed.kind else {
            unreachable!()
        };
        assert_eq!(right.arrival_delay, fs(100));
        let mut node = Rc::clone(left);
        while let Some(inner) = node.inner().map(Rc::clone) {
            arrivals.push(node.arrival_delay);
            node = inner;
        }
        arrivals.push(node.arrival_delay);
        assert_eq!(arrivals, [fs(100), fs(100), fs(110), fs(130), fs(160)]);
    });
}

#[test]
fn the_next_cheaper_pair_steps_the_side_whose_next_option_has_more_slack() {
    let side = |options: [(f64, i64); 3]| -> Vec<BnetPtr> {
        options
            .iter()
            .map(|&(cap, slack)| raw_load(PinId::new(0), cap, slack))
            .collect()
    };
    let left = side([(1.0, 1), (4.0, 6), (9.0, 9)]);
    let right = side([(1.0, 2), (3.0, 6), (8.0, 7)]);
    let sides = JunctionSides {
        left: &left,
        right: &right,
    };
    let next = |best_cap_ff: f64| sides.next_cheaper_pair((2, 2), Farads::new(best_cap_ff * 1e-15));

    // Equal slack steps the right side; a pair as heavy as the best is
    // passed over; sums are of the two sides' capacitances.
    assert_eq!(next(17.0), Some((2, 1)));
    assert_eq!(next(10.0), Some((1, 1)));
    assert_eq!(next(7.0), Some((1, 0)));
    assert_eq!(next(1.0), None);
}

/// Runs `test` on a session over [`buffer_between_flops`] whose global
/// routes give the layers of `layers` their wire RC.
fn with_layered_session(
    layers: &[(LayerId, OhmsPerMeter, FaradsPerMeter)],
    test: impl FnOnce(&Resizer<'_, Sta>),
) {
    let mut db = buffer_between_flops("PLACED");
    let lib = cells(&clock_cells_source());
    let rc = RouteRc::new(&db.library, layers, &[], LayerId(0));
    let wires = metal2_wires(&db);
    with_session_inputs(&mut db, &lib, CLOCK, wires, |inputs, sdc| {
        let mut source = NoRoutes;
        let routes = GlobalRouting {
            source: &mut source,
            rc,
            legalize: &|_, _| {},
        };
        test(&Resizer::on_global_routes(inputs, sdc, routes).unwrap());
    });
}

const STEP_SENTINEL: Dbu = Dbu::new(7);

/// A search whose smallest buffer has a 2 fF input, a 1 kilohm drive and
/// `input_max_slew`, with `max_wire_length` and the sentinel step.
fn search_with_smallest(
    resizer: &Resizer<'_, Sta>,
    input_max_slew: Seconds,
    max_wire_length: Option<Dbu>,
) -> Rebuffer {
    let size = BufferSize {
        input_cap: Farads::new(2e-15),
        drive_resistance: Ohms::new(1000.0),
        input_max_slew,
        ..buffer_size(resizer, resizer.buffers.cells[0])
    };
    Rebuffer {
        sizes: vec![size],
        max_wire_length,
        wire_length_step: Some(STEP_SENTINEL),
        driver: None,
        removed: 0,
        inserted: 0,
    }
}

const METAL2_RC: (LayerId, OhmsPerMeter, FaradsPerMeter) = (
    LayerId(1),
    OhmsPerMeter::new(2.0e6),
    FaradsPerMeter::new(1.0e-10),
);

#[test]
fn the_wire_step_on_a_layer_is_the_least_of_the_wire_a_buffer_drives_by_slew_and_by_load() {
    with_layered_session(&[METAL2_RC], |resizer| {
        let layer = Some(LayerId(1));
        let step = |slew: f64, max_wire_length: Option<Dbu>| {
            search_with_smallest(resizer, Seconds::new(slew), max_wire_length)
                .wire_length_step_on(resizer, layer)
        };

        // The buffer's 60 fF load limit less 20 percent and its own 2 fF
        // input leaves 46 fF, which 0.1 fF a micron of wire reaches 460
        // microns along: 920 000 units.
        assert_eq!(step(1e-6, None), Some(Dbu::new(920_000)));
        // The slew limit of 30 ps, 24 ps with the margin, reaches 119.38
        // microns of wire by the Elmore quadratic: 238 765 units.
        assert_eq!(step(3e-11, None), Some(Dbu::new(238_765)));
        // A negative root is no length, and the load limit stands.
        assert_eq!(step(2e-12, None), Some(Dbu::new(920_000)));
        // A slew limit that leaves no wire at all, to the last bit.
        assert_eq!(
            step(3.474_999_999_999_999_6e-12, None),
            Some(Dbu::new(920_000))
        );
        assert_eq!(step(3e-11, Some(Dbu::new(5_000))), Some(Dbu::new(5_000)));
    });
}

#[test]
fn the_wire_step_is_the_signal_wire_step_without_a_layer_or_a_wire_rc_to_use() {
    let dead = |layer: u8| {
        (
            LayerId(layer),
            OhmsPerMeter::new(2.0e6),
            FaradsPerMeter::ZERO,
        )
    };
    let resistless = (LayerId(2), OhmsPerMeter::ZERO, FaradsPerMeter::new(1.0e-10));
    with_layered_session(&[METAL2_RC, dead(0), resistless], |resizer| {
        let search = search_with_smallest(resizer, Seconds::new(3e-11), None);

        assert_eq!(
            search.wire_length_step_on(resizer, None),
            Some(STEP_SENTINEL)
        );
        assert_eq!(
            search.wire_length_step_on(resizer, Some(LayerId(0))),
            Some(STEP_SENTINEL)
        );
        assert_eq!(
            search.wire_length_step_on(resizer, Some(LayerId(2))),
            Some(STEP_SENTINEL)
        );
        assert_ne!(
            search.wire_length_step_on(resizer, Some(LayerId(1))),
            Some(STEP_SENTINEL)
        );
    });
}
