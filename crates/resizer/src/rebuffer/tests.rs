use super::*;
use crate::bnet::{Kind, PinLoad};
use crate::library::{BufferSet, Link};
use crate::parasitics::WireRc;
use crate::resizer::GlobalRouting;
use crate::route_parasitics::RouteRc;
use crate::testing::{
    cells, cells_with, clock_cells_source, design, design_with_copies, metal2_wires, pin,
    with_session_inputs, with_timed_session, with_wired_session,
};
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::{NetRoute, RouteSource};
use pare_common::db::indices::NetId;
use pare_common::db::liberty::{BufferCell, Edge};
use pare_common::db::library::SignalUse;
use pare_common::units::{CellArea, FaradsPerMeter, OhmsPerMeter};
use std::fmt::Write;
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
    buffer_between_flops_with(status, &[])
}

/// [`buffer_between_flops`] over the clock cells and copies of `BUF_X1` as
/// `copies` name them.
fn buffer_between_flops_with(status: &str, copies: &[&str]) -> NetlistDB {
    let copies: Vec<(&str, &str)> = copies.iter().map(|&name| ("BUF_X1", name)).collect();
    design_with_copies(
        &format!(
            "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN between ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 3 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - bf BUF_X1 + {status} ( 30000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 50000 2800 ) N ;\n\
             END COMPONENTS\nPINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 3 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n\
             - q ( f1 Q ) ( bf A ) ;\n- d ( bf Z ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n"
        ),
        &copies,
    )
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

/// A linear congruential generator, for seeded inputs.
struct Lcg(u64);

impl Lcg {
    /// A value in `[low, high)`.
    fn between(&mut self, low: f64, high: f64) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let unit = (self.0 >> 11) as f64 / (1u64 << 53) as f64;
        low + unit * (high - low)
    }
}

/// A buffer cell `name` whose tables rise with slew and load, from `rng`.
fn random_buffer(name: &str, rng: &mut Lcg) -> String {
    let table = |rng: &mut Lcg, base: (f64, f64), step: (f64, f64)| {
        let first = rng.between(base.0, base.1);
        let load = first + rng.between(step.0, step.1);
        let slew = first + rng.between(step.0, step.1) / 4.0;
        let both = load + slew - first + rng.between(0.0, step.0);
        format!("\"{first:.4}, {load:.4}\", \"{slew:.4}, {both:.4}\"")
    };
    let delay = table(rng, (0.01, 0.05), (0.05, 0.4));
    let transition = table(rng, (0.005, 0.02), (0.05, 0.5));
    let area = rng.between(0.5, 3.0);
    let input_cap = rng.between(0.5, 5.0);
    let max_cap = rng.between(20.0, 200.0);
    let max_transition = rng.between(0.05, 0.5);
    format!(
        "cell ({name}) {{ area : {area:.3} ; \
         pin (A) {{ direction : input ; capacitance : {input_cap:.3} ; }} \
         pin (Z) {{ direction : output ; function : \"A\" ; max_capacitance : {max_cap:.1} ; \
         max_transition : {max_transition:.3} ; \
         timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ; \
         cell_rise (delay_t) {{ values ({delay}) ; }} cell_fall (delay_t) {{ values ({delay}) ; }} \
         rise_transition (delay_t) {{ values ({transition}) ; }} \
         fall_transition (delay_t) {{ values ({transition}) ; }} }} }} }}\n"
    )
}

#[test]
fn the_limits_of_seeded_buffers_are_as_they_were() {
    const COUNT: usize = 60;
    let mut rng = Lcg(11);
    let names: Vec<String> = (0..COUNT).map(|i| format!("RB{i}")).collect();
    let extra: String = names.iter().map(|n| random_buffer(n, &mut rng)).collect();
    let mut db = buffer_between_flops("PLACED");
    let lib = cells_with(&extra);
    let wires = |resistance: f64, capacitance: f64| WireRc {
        resistance: OhmsPerMeter::from_ohms_per_micron(resistance),
        capacitance: FaradsPerMeter::from_femtofarads_per_micron(capacitance),
        layer: LayerId(1),
    };
    let mut text = String::new();

    for (label, rc) in [
        ("metal2", wires(3.5, 0.1)),
        ("light", wires(0.000_01, 0.000_01)),
        ("ideal", wires(0.0, 0.0)),
    ] {
        with_session_inputs(&mut db, &lib, CLOCK, rc, |inputs, sdc| {
            let resizer = Resizer::timed(inputs, sdc).unwrap();
            for name in &names {
                let buffer = lib.buffer_cell(lib.cell_id(name).unwrap()).unwrap();
                let size = buffer_size(&resizer, buffer);

                assert!(size.intrinsic_delay >= Fixed::ZERO, "{name}");
                assert!(size.margined_max_cap >= Farads::ZERO, "{name}");
                let _ = writeln!(
                    text,
                    "{label} {name} in {:.6e} out {:.6e} area {:.4} intrinsic {:?} \
                     max_cap {:.6e} drive {:.6e} slew_length {:?} cap_length {:?}",
                    size.input_cap.get(),
                    size.output_cap.get(),
                    size.area.get(),
                    size.intrinsic_delay,
                    size.margined_max_cap.get(),
                    size.drive_resistance.get(),
                    wire_length_limit_by_load_slew(&resizer, &size),
                    wire_length_limit_by_max_cap(&resizer, &size),
                );
            }
        });
    }

    pare_common::testing::assert_golden(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden"),
        "c_buffer_limits.txt",
        &text,
    );
}

#[test]
fn the_longest_wire_worth_no_buffer_for_seeded_buffer_sets_is_as_it_was() {
    const COUNT: usize = 24;
    let mut rng = Lcg(23);
    let names: Vec<String> = (0..COUNT).map(|i| format!("RB{i}")).collect();
    let extra: String = names.iter().map(|n| random_buffer(n, &mut rng)).collect();
    let db = buffer_between_flops("PLACED");
    let lib = cells_with(&extra);
    let link = Link::new(&db, &lib, &[]).unwrap();
    let buffer = |name: &String| lib.buffer_cell(lib.cell_id(name).unwrap()).unwrap();
    let wires = |resistance: f64, capacitance: f64| WireRc {
        resistance: OhmsPerMeter::from_ohms_per_micron(resistance),
        capacitance: FaradsPerMeter::from_femtofarads_per_micron(capacitance),
        layer: LayerId(1),
    };
    let mut text = String::new();

    for sets in names.chunks(3) {
        let cells: Vec<BufferCell> = sets.iter().map(buffer).collect();
        let set = BufferSet {
            lowest_drive: cells[cells.len() - 1],
            cells,
        };
        for rc in [
            wires(3.5, 0.1),
            wires(1.0, 0.2),
            wires(10.0, 0.05),
            wires(0.0, 0.1),
        ] {
            let limit = max_wire_length_of(&link, &set, rc, db.dbu());
            let _ = writeln!(
                text,
                "{sets:?} r {} c {} limit {limit:?}",
                rc.resistance.get(),
                rc.capacitance.get()
            );
        }
    }

    pare_common::testing::assert_golden(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden"),
        "c_max_wire_lengths.txt",
        &text,
    );
}

/// A buffer cell `name` of `input_cap` femtofarads whose delay is the
/// `delays` nanoseconds at loads of 1, 20 and 100 femtofarads whatever the
/// input slew, and whose output slews to 0.5 ns at 100 femtofarads.
fn flat_buffer(name: &str, input_cap: f64, delays: [f64; 3]) -> String {
    let [d1, d20, d100] = delays;
    let delay = format!("\"{d1}, {d20}, {d100}\", \"{d1}, {d20}, {d100}\"");
    let slew = "\"0.01, 0.5, 0.5\", \"0.01, 0.5, 0.5\"";
    format!(
        "cell ({name}) {{ area : 1.0 ; \
         pin (A) {{ direction : input ; capacitance : {input_cap} ; }} \
         pin (Z) {{ direction : output ; function : \"A\" ; max_capacitance : 100.0 ; \
         timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ; \
         cell_rise (loads_t) {{ values ({delay}) ; }} cell_fall (loads_t) {{ values ({delay}) ; }} \
         rise_transition (loads_t) {{ values ({slew}) ; }} \
         fall_transition (loads_t) {{ values ({slew}) ; }} }} }} }}\n"
    )
}

const LOADS_TEMPLATE: &str = "lu_table_template (loads_t) { \
    variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ; \
    index_1 (\"0.005, 0.2\") ; index_2 (\"1.0, 20.0, 100.0\") ; } \
    lu_table_template (few_t) { \
    variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ; \
    index_1 (\"0.005, 0.2\") ; index_2 (\"1.0, 5.0\") ; }\n";

/// The clock cells, `BUF_X1` made unusable, with the Liberty `extra`.
fn cells_with_buf_unusable(extra: &str) -> pare_common::db::liberty::Liberty {
    let source =
        clock_cells_source().replacen("cell (BUF_X1) {", "cell (BUF_X1) { dont_use : true ;", 1);
    let end = source.rfind('}').unwrap();
    cells(&format!("{}{LOADS_TEMPLATE}{extra}\n}}\n", &source[..end]))
}

/// The names of the buffers worth inserting for speed among `cells` of the
/// Liberty text `extra`, a copy of `BUF_X1` each.
fn fast_buffers_among(names: &[&str], extra: &str) -> Vec<String> {
    let mut db = buffer_between_flops_with("PLACED", names);
    let lib = cells_with_buf_unusable(extra);
    let mut fast = Vec::new();
    with_wired_session(&mut db, &lib, CLOCK, |resizer| {
        fast = find_fast_buffers(resizer)
            .into_iter()
            .map(|b| lib.cell(b.id).name.clone())
            .collect();
    });
    fast
}

#[test]
fn a_buffer_another_is_faster_than_at_every_load_despite_its_input_cap_is_not_fast() {
    // `FB` outpaces `FA` at 1 fF by 8 ps, 5 ps over its extra fF of input
    // load at the weakest drive's 5 kilohms; it is slower from 20 fF on,
    // which the cut-off of twenty times `FA`'s input capacitance leaves
    // out. `FC` is faster than `FB` by 1 ps, not worth the 5 ps.
    // `FB` also has an arc by a clock edge, whose table's loads of 5 fF are
    // no concern of a buffer's delay.
    let clocked = "timing () { related_pin : \"A\" ; timing_type : rising_edge ; \
        cell_rise (few_t) { values (\"0.001, 0.001\", \"0.001, 0.001\") ; } \
        cell_fall (few_t) { values (\"0.001, 0.001\", \"0.001, 0.001\") ; } \
        rise_transition (few_t) { values (\"0.01, 0.01\", \"0.01, 0.01\") ; } \
        fall_transition (few_t) { values (\"0.01, 0.01\", \"0.01, 0.01\") ; } }";
    let fb = flat_buffer("FB", 2.0, [0.092, 0.5, 0.5]).replacen(
        "timing () {",
        &format!("{clocked} timing () {{"),
        1,
    );
    let extra = [
        flat_buffer("FA", 1.0, [0.100, 0.100, 0.100]),
        fb,
        flat_buffer("FC", 3.0, [0.091, 0.091, 0.091]),
    ]
    .concat();

    assert_eq!(
        fast_buffers_among(&["FA", "FB", "FC"], &extra),
        ["FB", "FC"]
    );
}

/// A flop `f1` driving net `q` into a buffer `bp` toward `f2`, a fixed
/// buffer `bf` toward `f3`, and a buffer `pb` onto the port `out`; and a
/// buffer `bx` on the input port `in2`'s net, which `f4` reads as well.
fn buffer_tree_lab() -> NetlistDB {
    design(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN lab ;\n\
         UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
         COMPONENTS 9 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
         - bp BUF_X1 + PLACED ( 20000 2800 ) N ;\n- bf BUF_X1 + FIXED ( 20000 8400 ) N ;\n\
         - pb BUF_X1 + PLACED ( 20000 14000 ) N ;\n- bx BUF_X1 + PLACED ( 40000 2800 ) N ;\n\
         - f2 DFF_X1 + PLACED ( 50000 2800 ) N ;\n- f3 DFF_X1 + PLACED ( 50000 8400 ) N ;\n\
         - f4 DFF_X1 + PLACED ( 50000 14000 ) N ;\n- f5 DFF_X1 + PLACED ( 60000 2800 ) N ;\n\
         END COMPONENTS\nPINS 4 ;\n- clk + NET clk + DIRECTION INPUT ;\n\
         - in2 + NET in2 + DIRECTION INPUT ;\n- out + NET out + DIRECTION OUTPUT ;\n\
         - unused + NET unused + DIRECTION INPUT ;\nEND PINS\n\
         NETS 9 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ( f3 CK ) ( f4 CK ) ( f5 CK ) ;\n\
         - q ( f1 Q ) ( bp A ) ( bf A ) ( pb A ) ;\n- d2 ( bp Z ) ( f2 D ) ;\n\
         - d3 ( bf Z ) ( f3 D ) ;\n- out ( pb Z ) ( PIN out ) ;\n\
         - in2 ( PIN in2 ) ( bx A ) ( f4 D ) ;\n- xz ( bx Z ) ( f5 D ) ;\n\
         - unused ( PIN unused ) ;\n- bpq ( f2 Q ) ;\nEND NETS\nEND DESIGN\n",
    )
}

/// Runs `test` on a timed session over [`buffer_tree_lab`] and a search
/// with `f1/Q` as its driver.
fn with_tree_lab(test: impl FnOnce(&mut Resizer<'_, Sta>, &mut Rebuffer, PinId)) {
    let mut db = buffer_tree_lab();
    let q = pin(&db, "f1/Q");
    with_wired_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let mut rebuffer = Rebuffer::new(resizer).unwrap();
        assert!(rebuffer.set_driver(resizer, q));
        test(resizer, &mut rebuffer, q);
    });
}

fn pin_names(resizer: &Resizer<'_, Sta>, pins: Vec<PinId>) -> Vec<String> {
    let mut names: Vec<String> = pins.into_iter().map(|p| resizer.db.pin_name(p)).collect();
    names.sort();
    names
}

#[test]
fn importing_a_net_folds_in_the_buffers_that_may_go_and_keeps_the_rest_as_loads() {
    with_tree_lab(|resizer, rebuffer, q| {
        let imported = rebuffer.import_buffer_tree(resizer, q).unwrap().unwrap();

        // `bp` is folded in: what it drives is the load. The fixed `bf`
        // and the port buffer `pb` stay loads themselves.
        assert_eq!(
            pin_names(resizer, imported.loads()),
            ["bf/A", "f2/D", "pb/A"]
        );
    });
}

#[test]
fn the_buffers_to_remove_after_rebuffering_are_those_the_import_folded_in() {
    with_tree_lab(|resizer, rebuffer, q| {
        let imported = rebuffer.import_buffer_tree(resizer, q).unwrap().unwrap();

        let old = collect_imported_buffers(resizer, q, &imported);

        let names: Vec<&str> = old
            .iter()
            .map(|&c| resizer.db.cells[c].name.as_str())
            .collect();
        assert_eq!(names, ["bp"]);
    });
}

#[test]
fn a_buffer_alone_on_a_port_net_is_a_port_buffer_and_one_sharing_the_net_is_not() {
    with_tree_lab(|resizer, _, _| {
        let is_port_buffer_named =
            |name: &str| is_port_buffer(resizer, resizer.db.cell_id(name).unwrap());

        assert!(is_port_buffer_named("pb"));
        assert!(!is_port_buffer_named("bp"));
        assert!(!is_port_buffer_named("bx"));
    });
}

#[test]
fn only_a_net_with_an_output_port_has_a_top_level_output() {
    with_tree_lab(|resizer, _, _| {
        let net = |name: &str| resizer.db.net_id(name).unwrap();

        assert!(has_top_level_output(resizer, net("out")));
        assert!(!has_top_level_output(resizer, net("in2")));
        assert!(!has_top_level_output(resizer, net("q")));
    });
}

/// The cells of the candidate lab: a tie-low cell and a tristate buffer.
const CANDIDATE_CELLS: &str = "\
    cell (TIELO) { area : 0.53 ; pin (ZN) { direction : output ; function : \"0\" ; } } \
    cell (TBUF) { area : 0.8 ; \
      pin (A) { direction : input ; capacitance : 1.0 ; } \
      pin (B) { direction : input ; capacitance : 1.0 ; } \
      pin (Z) { direction : output ; function : \"A\" ; three_state : \"B\" ; \
        timing () { related_pin : \"A\" ; timing_sense : positive_unate ; \
          cell_rise (delay_t) { values (\"0.03, 0.30\", \"0.04, 0.31\") ; } } } }";

/// The drivers `rebuffer_candidates` picks in a design with a driver of
/// each kind: an inverter `ok`; a buffer `buf` between logic and logic; a
/// buffer `pbuf` onto the port `out`; an inverter `vss` on a ground net;
/// an inverter `ck` on the clock; a tie cell `lo`; and a tristate `tri`.
fn candidate_names() -> Vec<String> {
    let mut db = design_with_copies(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN cand ;\n\
         UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
         COMPONENTS 9 ;\n- ok INV_X1 + PLACED ( 2000 2800 ) N ;\n\
         - buf BUF_X1 + PLACED ( 4000 2800 ) N ;\n- pbuf BUF_X1 + PLACED ( 6000 2800 ) N ;\n\
         - vss INV_X1 + PLACED ( 8000 2800 ) N ;\n- ck INV_X1 + PLACED ( 10000 2800 ) N ;\n\
         - lo TIELO + PLACED ( 12000 2800 ) N ;\n- tri TBUF + PLACED ( 14000 2800 ) N ;\n\
         - f1 DFF_X1 + PLACED ( 20000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 30000 2800 ) N ;\n\
         END COMPONENTS\nPINS 3 ;\n- clk + NET clk + DIRECTION INPUT ;\n\
         - a + NET a + DIRECTION INPUT ;\n- out + NET out + DIRECTION OUTPUT ;\nEND PINS\n\
         NETS 10 ;\n- clk ( PIN clk ) ( ck A ) ( f1 CK ) ;\n- ckn ( ck ZN ) ( f2 CK ) ;\n\
         - a ( PIN a ) ( ok A ) ( vss A ) ( tri A ) ( tri B ) ;\n\
         - n1 ( ok ZN ) ( buf A ) ( pbuf A ) ;\n- n2 ( buf Z ) ( f1 D ) ;\n\
         - out ( pbuf Z ) ( PIN out ) ;\n- vssn ( vss ZN ) ( f2 D ) ;\n\
         - low ( lo ZN ) ;\n- trn ( tri Z ) ;\n- q ( f1 Q ) ;\nEND NETS\nEND DESIGN\n",
        &[("INV_X1", "TIELO"), ("AND2_X1", "TBUF")],
    );
    let ground = db.net_id("vssn").unwrap();
    db.nets[ground].signal_use = SignalUse::Ground;
    let lib = cells_with(CANDIDATE_CELLS);
    let mut names = Vec::new();
    with_timed_session(&mut db, &lib, CLOCK, |resizer| {
        names = pin_names(resizer, rebuffer_candidates(resizer));
    });
    names
}

#[test]
fn full_rebuffering_leaves_alone_clocks_constants_tristates_special_nets_and_logic_buffers() {
    assert_eq!(candidate_names(), ["f1/Q", "ok/ZN", "pbuf/Z"]);
}

/// A flop like `DFF_X1` whose output falls 100 ps after its clock rises
/// more slowly than it rises.
fn slow_falling_flop() -> String {
    let source = clock_cells_source();
    let start = source.find("cell (DFF_X1)").unwrap();
    let end = source.find("cell (DFF_SLOW)").unwrap();
    source[start..end].replace("DFF_X1", "DFFA").replace(
        "cell_fall (delay_t) { values (\"0.08, 0.3\", \"0.09, 0.31\") ; }",
        "cell_fall (delay_t) { values (\"0.18, 0.4\", \"0.19, 0.41\") ; }",
    )
}

/// A clock buffer driving two `DFFA` flops, whose `f1/Q` drives `f2/D`.
fn buffered_slow_flops() -> NetlistDB {
    design_with_copies(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN pair ;\n\
         UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
         COMPONENTS 3 ;\n- cb CLKBUF_X3 + PLACED ( 2000 2800 ) N ;\n\
         - f1 DFFA + PLACED ( 12000 2800 ) N ;\n- f2 DFFA + PLACED ( 40000 2800 ) N ;\n\
         END COMPONENTS\nPINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
         NETS 3 ;\n- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( f1 CK ) ( f2 CK ) ;\n\
         - q ( f1 Q ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n",
        &[("DFF_X1", "DFFA")],
    )
}

/// What `timing_through` gives for the driver `f1/Q` of a net it loads
/// with 10 fF, entering by a rising clock and leaving by `edge`, under
/// `constraints`, with the arrival at the driver `arrival`.
fn driver_timing_of_slow_flop(
    constraints: &str,
    edge: Edge,
    arrival: Seconds,
) -> (DriverTiming, Seconds) {
    let mut db = buffered_slow_flops();
    let lib = cells_with(&slow_falling_flop());
    let ck = pin(&db, "f1/CK");
    let mut result = None;
    with_timed_session(&mut db, &lib, constraints, |resizer| {
        let (lib_id, cell) = resizer
            .link
            .cell(resizer.db, resizer.db.cell_id("f1").unwrap())
            .unwrap();
        let set = cell
            .arc_sets
            .iter_enumerated()
            .find(|(_, s)| s.role == ArcRole::RegClkToQ)
            .map(|(id, _)| id)
            .unwrap();
        assert_eq!(resizer.link.liberty.cell(lib_id).name, "DFFA");
        let point = DriverPoint {
            arrival,
            from: Some((ck, Edge::Rise, set)),
        };
        let timing = timing_through(resizer, cell, &point, edge, Farads::new(10e-15)).unwrap();
        let prev = resizer
            .sta()
            .arrival(ck, Edge::Rise, MinMax::Max)
            .unwrap_or(Seconds::ZERO);
        result = Some((timing, prev));
    });
    result.unwrap()
}

/// The delay of the clock-to-output arc of `DFFA` from a rising clock to
/// `edge` at `slew` into a lumped load of 10 fF.
fn flop_arc_delay(edge: Edge, slew: Seconds) -> Seconds {
    let lib = cells_with(&slow_falling_flop());
    let cell = lib.cell(lib.cell_id("DFFA").unwrap());
    cell.arc_sets
        .iter()
        .find(|s| s.role == ArcRole::RegClkToQ)
        .unwrap()
        .arcs
        .iter()
        .find(|a| a.from_edge == Edge::Rise && a.to_edge == edge)
        .unwrap()
        .gate_delay(slew, Farads::new(10e-15))
        .0
}

#[test]
fn a_flop_on_an_ideal_clock_is_timed_from_a_clock_of_no_slew_and_its_own_edge_arcs() {
    let (rise, prev) = driver_timing_of_slow_flop(CLOCK, Edge::Rise, Seconds::new(1e-9));
    let (fall, _) = driver_timing_of_slow_flop(CLOCK, Edge::Fall, Seconds::new(1e-9));

    let near = |a: Seconds, b: Seconds| (a - b).abs() < Seconds::new(1e-15);
    assert!(near(rise.delay, flop_arc_delay(Edge::Rise, Seconds::ZERO)));
    assert!(near(fall.delay, flop_arc_delay(Edge::Fall, Seconds::ZERO)));
    assert!(fall.delay > rise.delay);
    assert!(near(
        rise.correction,
        Seconds::new(1e-9) - (prev + rise.delay)
    ));
}

#[test]
fn a_flop_on_a_propagated_clock_is_timed_from_the_slew_the_clock_arrives_with() {
    let propagated = format!("{CLOCK}set_propagated_clock [all_clocks]\n");

    let (rise, _) = driver_timing_of_slow_flop(&propagated, Edge::Rise, Seconds::new(1e-9));

    let (ideal, _) = driver_timing_of_slow_flop(CLOCK, Edge::Rise, Seconds::new(1e-9));
    assert!(
        rise.delay > ideal.delay,
        "{:?} {:?}",
        rise.delay,
        ideal.delay
    );
}

#[test]
fn the_area_target_gives_back_a_quarter_of_the_slack_and_a_hundredth_of_the_path() {
    let mut db = buffered_clock_pair();
    let q = pin(&db, "f1/Q");
    with_timed_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let mut rebuffer = Rebuffer::new(resizer).unwrap();
        assert!(rebuffer.set_driver(resizer, q));
        let plain = resizer.buffered_tree(q).unwrap().unwrap();
        let tree = rebuffer.annotate_load_slacks(resizer, &plain, q);
        let slack = rebuffer.slack_at_driver(resizer, &tree);
        let delay = rebuffer.driver_timing(resizer, &tree).delay;
        let path = critical_path_delay(&tree).to_seconds();
        assert!(slack.to_seconds() > Seconds::ZERO && delay > Seconds::ZERO);

        for error in [0.0, -4e-12, 5e-12] {
            let error = Seconds::new(error);
            let spare = (slack.to_seconds() - error.min(Seconds::ZERO)).max(Seconds::ZERO);
            let relaxation = spare / 4.0 + (delay + path) * 0.01;

            assert_eq!(
                rebuffer.area_target(resizer, &tree, error),
                slack - Fixed::from_seconds(relaxation),
                "{error:?}"
            );
        }
        assert_eq!(
            rebuffer.area_target(resizer, &plain, Seconds::ZERO),
            -Fixed::INF
        );
    });
}

#[test]
fn full_rebuffering_counts_the_buffers_it_puts_in_and_takes_out() {
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
    with_wired_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let mut rebuffer = Rebuffer::new(resizer).unwrap();

        rebuffer.fully_rebuffer(resizer).unwrap();

        let placed = resizer
            .db
            .cells
            .iter()
            .filter(|c| !c.retired && c.name.starts_with("place"))
            .count();
        assert_eq!(rebuffer.removed, 2);
        assert!(placed > 0);
        assert_eq!(rebuffer.inserted, placed);
    });
}
