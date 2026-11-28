use super::*;
use crate::bnet::{Kind, PinLoad, SlackEdges};
use crate::library::{BufferSet, Link};
use crate::parasitics::WireRc;
use crate::resizer::GlobalRouting;
use crate::route_parasitics::RouteRc;
use crate::testing::{
    cells, cells_with, clock_cells_source, design, design_with_copies, metal2_wires, pin,
    with_session_inputs, with_timed_session, with_wired_session,
};
use pare_common::convert::SaturatingFrom;
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

/// How a buffer's output slew follows its load.
#[derive(Clone, Copy)]
enum SlewShape {
    /// Rising steeply with load.
    Rising,
    /// Starting high and rising slowly, so that the slew limit is only
    /// reached at many times the first guess of the load.
    Gentle,
    /// Staying under any slew limit.
    Never,
    /// Not tabulated: the buffer has no drive resistance.
    Missing,
}

impl SlewShape {
    const ALL: [Self; 4] = [Self::Rising, Self::Gentle, Self::Never, Self::Missing];

    /// The `index`th shape in turn.
    fn nth(index: usize) -> Self {
        Self::ALL[index % Self::ALL.len()]
    }
}

/// A buffer cell `name` whose delay rises with slew and load, and whose
/// output slew follows `shape`, from `rng`.
fn random_buffer(name: &str, rng: &mut Lcg, shape: SlewShape) -> String {
    let table = |rng: &mut Lcg, base: (f64, f64), step: (f64, f64)| {
        let first = rng.between(base.0, base.1);
        let load = first + rng.between(step.0, step.1);
        let slew = first + rng.between(step.0, step.1) / 4.0;
        let both = load + slew - first + rng.between(0.0, step.0);
        format!("\"{first:.4}, {load:.4}\", \"{slew:.4}, {both:.4}\"")
    };
    let delay = table(rng, (0.01, 0.05), (0.05, 0.4));
    let transition = match shape {
        SlewShape::Rising => table(rng, (0.005, 0.02), (0.05, 0.5)),
        SlewShape::Gentle => table(rng, (0.12, 0.2), (0.01, 0.03)),
        SlewShape::Never => table(rng, (0.02, 0.03), (0.0, 0.002)),
        SlewShape::Missing => String::new(),
    };
    let transitions = if transition.is_empty() {
        String::new()
    } else {
        format!(
            "rise_transition (delay_t) {{ values ({transition}) ; }} \
             fall_transition (delay_t) {{ values ({transition}) ; }}"
        )
    };
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
         {transitions} }} }} }}\n"
    )
}

#[test]
fn the_limits_of_seeded_buffers_are_as_they_were() {
    const COUNT: usize = 60;
    let mut rng = Lcg(11);
    let names: Vec<String> = (0..COUNT).map(|i| format!("RB{i}")).collect();
    let extra: String = names
        .iter()
        .enumerate()
        .map(|(i, n)| random_buffer(n, &mut rng, SlewShape::nth(i)))
        .collect();
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
    let extra: String = names
        .iter()
        .enumerate()
        .map(|(i, n)| random_buffer(n, &mut rng, SlewShape::nth(i)))
        .collect();
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

#[test]
fn the_session_wire_length_limit_is_the_one_its_buffers_and_wire_give() {
    with_search(|resizer, rebuffer| {
        let expected = max_wire_length_of(
            resizer.link,
            &resizer.buffers,
            resizer.graph.wire_rc(),
            resizer.db.dbu(),
        );

        assert!(expected.is_some());
        assert_eq!(max_wire_length(resizer), expected);
        assert_eq!(rebuffer.max_wire_length, expected);
    });
}

/// The DEF of a driver flop `f1` whose net `q` reaches `loads` flops
/// placed by `rng` over up to 1.6 million units (800 um) of die, some
/// through a buffer `b1`.
fn seeded_fanout(rng: &mut Lcg, loads: usize, buffered: bool) -> String {
    let mut components = String::from("- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n");
    let mut q = String::from("- q ( f1 Q )");
    let mut clock = String::from("- clk ( PIN clk ) ( f1 CK )");
    for i in 0..loads {
        let x = i64::saturating_from(rng.between(20_000.0, 1_600_000.0)) / 2 * 2;
        let y = i64::saturating_from(rng.between(2_800.0, 300_000.0)) / 2800 * 2800;
        let _ = writeln!(components, "- l{i} DFF_X1 + PLACED ( {x} {y} ) N ;");
        let _ = write!(clock, " ( l{i} CK )");
        if buffered && i == 0 {
            let _ = writeln!(components, "- b1 BUF_X1 + PLACED ( {} {y} ) N ;", x / 2);
            q += " ( b1 A )";
        } else {
            let _ = write!(q, " ( l{i} D )");
        }
    }
    let buffer_net = if buffered {
        "- bz ( b1 Z ) ( l0 D ) ;\n"
    } else {
        ""
    };
    format!(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN seeded ;\n\
         UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 1700000 400000 ) ;\n\
         COMPONENTS {} ;\n{components}END COMPONENTS\n\
         PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
         NETS {} ;\n{clock} ;\n{q} ;\n{buffer_net}END NETS\nEND DESIGN\n",
        loads + 1 + usize::from(buffered),
        2 + usize::from(buffered),
    )
}

/// The buffers added to `resizer`'s design since it was made, as
/// `name master x y`.
fn added_buffers(resizer: &Resizer<'_, Sta>, before: usize) -> String {
    resizer
        .db
        .cells
        .iter()
        .skip(before)
        .fold(String::new(), |mut text, cell| {
            let master = &resizer.db.library.macros[cell.macro_id].name;
            let _ = write!(
                text,
                " [{} {master} {} {}{}]",
                cell.name,
                cell.location.x.get(),
                cell.location.y.get(),
                if cell.retired { " retired" } else { "" }
            );
            text
        })
}

#[test]
fn rebuffering_seeded_nets_gives_the_buffers_it_did() {
    const CASES: usize = 36;
    let mut rng = Lcg(5);
    let extra: String = (0..3)
        .map(|i| random_buffer(&format!("RB{i}"), &mut rng, SlewShape::Rising))
        .collect();
    let lib = cells_with(&extra);
    let mut text = String::new();
    for case in 0..CASES {
        let loads = 2 + case % 5;
        let period = rng.between(0.25, 0.8);
        let constraints = format!("create_clock -name clk -period {period:.3} [get_ports clk]\n");
        for whole_design in [false, true] {
            let def = seeded_fanout(&mut Lcg(100 + case as u64), loads, case % 3 == 0);
            let mut db = design_with_copies(
                &def,
                &[("BUF_X1", "RB0"), ("BUF_X1", "RB1"), ("BUF_X1", "RB2")],
            );
            let q = pin(&db, "f1/Q");
            let before = db.cells.len();
            with_wired_session(&mut db, &lib, &constraints, |resizer| {
                let mut rebuffer = Rebuffer::new(resizer).unwrap();
                let outcome = if whole_design {
                    rebuffer
                        .fully_rebuffer(resizer)
                        .map(|()| (rebuffer.removed, rebuffer.inserted))
                } else {
                    rebuffer.rebuffer_pin(resizer, q).map(|count| (0, count))
                };
                let _ = writeln!(
                    text,
                    "case {case} loads {loads} period {period:.3} whole {whole_design}: {outcome:?}{}",
                    added_buffers(resizer, before)
                );
            });
        }
    }

    pare_common::testing::assert_golden(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden"),
        "c_rebuffered_nets.txt",
        &text,
    );
}

/// A load-like node of `cap_ff` femtofarads, `slack` femtoseconds, `area`
/// and `edges` for the slack, at the origin.
fn option_node(cap_ff: f64, slack: i64, area: f64, edges: Option<SlackEdges>) -> BnetPtr {
    let load = raw_load(PinId::new(0), cap_ff, slack);
    Rc::new(Bnet {
        kind: load.kind.clone(),
        area: CellArea::new(area),
        slack_edges: edges,
        ..*load
    })
}

/// Runs `test` with a search whose only buffer has `size_of` the first
/// buffer's size, driver `f1/Q` set.
fn with_sized_search(
    size_of: impl FnOnce(BufferSize) -> BufferSize,
    test: impl FnOnce(&Resizer<'_, Sta>, &mut Rebuffer, &BufferSize),
) {
    let mut db = buffer_between_flops("PLACED");
    let q = pin(&db, "f1/Q");
    with_wired_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let mut rebuffer = Rebuffer::new(resizer).unwrap();
        let size = size_of(rebuffer.sizes[0]);
        rebuffer.sizes = vec![size];
        assert!(rebuffer.set_driver(resizer, q));
        test(resizer, &mut rebuffer, &size);
    });
}

/// A buffer of 2 fF input, 5 fF output, 1 square micron, no intrinsic
/// delay and a 50 fF load limit, which can drive what is light.
fn plain_size(base: BufferSize) -> BufferSize {
    BufferSize {
        input_cap: Farads::new(2e-15),
        output_cap: Farads::new(5e-15),
        area: CellArea::new(1.0),
        intrinsic_delay: Fixed::ZERO,
        margined_max_cap: Farads::new(50e-15),
        drive_resistance: Ohms::new(100.0),
        input_max_slew: Seconds::INFINITY,
        ..base
    }
}

#[test]
fn a_timing_frontier_keeps_only_options_with_more_slack_than_those_kept() {
    with_sized_search(plain_size, |resizer, _, size| {
        let mut frontier = Frontier::new(None, 4);

        for slack in [100, 100, 50, 150] {
            frontier.offer(
                resizer,
                size,
                &option_node(10.0, slack, 0.0, None),
                Dbu::ZERO,
            );
        }

        let kept: Vec<Fixed> = frontier.kept.iter().map(|o| o.slack).collect();
        assert_eq!(kept, [fs(100), fs(150)]);
        assert_eq!(frontier.best_slack, fs(150));
    });
}

#[test]
fn an_area_frontier_keeps_only_options_with_less_area_and_enough_slack() {
    with_sized_search(plain_size, |resizer, _, size| {
        let exemplar = option_node(10.0, 500, 0.0, None);
        let offer_all = |options: &[(f64, i64)]| {
            let mut frontier = Frontier::new(Some((fs(100), &exemplar)), 4);
            for &(area, slack) in options {
                frontier.offer(
                    resizer,
                    size,
                    &option_node(10.0, slack, area, None),
                    Dbu::ZERO,
                );
            }
            frontier
                .kept
                .iter()
                .map(|o| o.area.get())
                .collect::<Vec<f64>>()
        };

        // Enough slack and less area than any kept: kept. The same area
        // again, or more, or too little slack, not.
        assert_eq!(offer_all(&[(3.0, 200), (3.0, 200), (5.0, 200)]), [3.0]);
        assert_eq!(offer_all(&[(3.0, 50), (2.0, 200)]), [2.0]);
        assert_eq!(offer_all(&[(3.0, 200), (2.0, 50), (1.0, 100)]), [3.0, 1.0]);
    });
}

#[test]
fn a_buffer_is_put_on_the_option_that_it_buffers_best_and_can_drive() {
    with_sized_search(plain_size, |resizer, _, size| {
        let sized = BufferSize {
            intrinsic_delay: fs(10),
            ..*size
        };
        let options = [
            option_node(10.0, 112, 0.0, None),
            option_node(10.0, 120, 0.0, None),
        ];
        let mut frontier = Frontier::new(None, 4);
        frontier.best_slack = fs(105);

        // The first has 102 left once the intrinsic delay is taken off,
        // too little against 105; the buffer delays nothing without edges
        // to time.
        let best = frontier.best_buffered(resizer, &sized, &options);

        let (option, delay) = best.unwrap();
        assert!(Rc::ptr_eq(&option, &options[1]));
        assert_eq!(delay, Fixed::ZERO);
        assert_eq!(frontier.best_slack, fs(120));
        assert_eq!(frontier.best_area, CellArea::new(1.0));
        let mut alone = Frontier::new(None, 4);
        alone.best_slack = fs(105);
        assert!(
            alone
                .best_buffered(resizer, &sized, &options[..1])
                .is_none()
        );
    });
}

#[test]
fn a_buffer_that_cannot_drive_an_option_is_not_put_on_it() {
    with_sized_search(plain_size, |resizer, _, size| {
        let weak = BufferSize {
            margined_max_cap: Farads::new(8e-15),
            ..*size
        };
        let options = [option_node(10.0, 100, 0.0, None)];
        let mut frontier = Frontier::new(None, 4);

        assert!(frontier.best_buffered(resizer, &weak, &options).is_none());
        assert!(frontier.best_buffered(resizer, size, &options).is_some());
    });
}

#[test]
fn an_area_frontier_charges_a_buffer_its_area_before_taking_it() {
    with_sized_search(plain_size, |resizer, _, size| {
        let exemplar = option_node(10.0, 500, 0.0, None);
        let options = [option_node(10.0, 200, 0.0, None)];
        let mut frontier = Frontier::new(Some((fs(100), &exemplar)), 4);
        frontier.best_area = CellArea::new(0.5);

        // 0 square microns and the buffer's 1 is not less than 0.5.
        assert!(frontier.best_buffered(resizer, size, &options).is_none());
        frontier.best_area = CellArea::new(1.5);
        assert!(frontier.best_buffered(resizer, size, &options).is_some());
    });
}

#[test]
fn a_buffer_delays_an_option_by_its_delay_into_the_option_and_its_own_output() {
    with_sized_search(plain_size, |resizer, _, size| {
        let edges = Some(SlackEdges::Both);
        let option = option_node(10.0, 100_000, 0.0, edges);
        let expected = timing::buffer_delay(resizer, size, edges, Farads::new(15e-15));
        assert!(expected > Fixed::ZERO);
        let mut frontier = Frontier::new(None, 4);

        let (_, delay) = frontier
            .best_buffered(resizer, size, &[Rc::clone(&option)])
            .unwrap();

        assert_eq!(delay, expected);
        // The buffered option must beat the best slack by more than the
        // delay: one exactly at it is not taken.
        let mut tied = Frontier::new(None, 4);
        tied.best_slack = option.slack - expected;
        assert!(tied.best_buffered(resizer, size, &[option]).is_none());
    });
}

#[test]
fn only_the_option_a_buffer_cannot_beat_stays_when_none_was_kept_before_it() {
    with_sized_search(plain_size, |resizer, rebuffer, size| {
        let small = |slack| option_node(1.0, slack, 0.0, None);
        // Everything is lighter than the buffer's input, but the extra
        // wire is too much for it to drive as it is.
        let heavy_wire = Dbu::new(4_000_000);
        let options = [small(100), small(300), small(200)];

        let frontier = rebuffer.buffered_frontier(resizer, &options, heavy_wire, None);

        // The last option alone is buffered: the near ones are not tried.
        assert_eq!(frontier.kept.len(), 1);
        assert!(matches!(frontier.kept[0].kind, Kind::Buffer { .. }));
        assert_eq!(frontier.kept[0].slack, fs(200));
        assert!(Rc::ptr_eq(frontier.kept[0].inner().unwrap(), &options[2]));
        let _ = size;
    });
}

#[test]
fn a_frontier_over_no_options_is_empty() {
    with_sized_search(plain_size, |resizer, rebuffer, _| {
        assert!(
            rebuffer
                .buffered_frontier(resizer, &[], Dbu::ZERO, None)
                .kept
                .is_empty()
        );
    });
}

#[test]
fn options_no_heavier_than_a_buffers_input_are_offered_and_heavier_ones_buffered() {
    with_sized_search(plain_size, |resizer, rebuffer, _| {
        let light = option_node(1.0, 100, 0.0, None);
        let heavy = option_node(10.0, 200, 0.0, None);

        let frontier = rebuffer.buffered_frontier(
            resizer,
            &[Rc::clone(&light), Rc::clone(&heavy)],
            Dbu::ZERO,
            None,
        );

        // The light one stays as it is; the buffer goes on the heavy one,
        // which as it is would add no slack to the buffered one's.
        assert_eq!(frontier.kept.len(), 2);
        assert!(Rc::ptr_eq(&frontier.kept[0], &light));
        assert!(matches!(frontier.kept[1].kind, Kind::Buffer { .. }));
        assert!(Rc::ptr_eq(frontier.kept[1].inner().unwrap(), &heavy));
    });
}

#[test]
fn area_options_that_already_meet_the_target_get_no_extra_option() {
    with_sized_search(plain_size, |resizer, rebuffer, _| {
        let exemplar = option_node(10.0, 500, 0.0, None);
        let mut options = vec![option_node(10.0, 450, 0.0, None)];

        rebuffer
            .insert_area_options(resizer, &mut options, Dbu::ZERO, (fs(400), &exemplar))
            .unwrap();

        // The buffered option and the option as it is: no third to make
        // the target certain.
        assert_eq!(options.len(), 2);
    });
}

#[test]
fn area_options_none_of_which_meets_the_target_fail() {
    with_sized_search(plain_size, |resizer, rebuffer, _| {
        let exemplar = option_node(10.0, 500, 0.0, None);
        let mut options = vec![option_node(10.0, 300, 0.0, None)];

        let outcome =
            rebuffer.insert_area_options(resizer, &mut options, Dbu::ZERO, (fs(400), &exemplar));

        assert_eq!(outcome, Err(AreaRecoveryFailed));
    });
}

/// Runs `test` with a search whose buffer takes 1 ps before it does
/// anything, so that only an assured fallback can put it on an option,
/// and a buffered `exemplar` over a load at 500 fs of slack.
fn with_fallback_only_search(
    test: impl FnOnce(&Resizer<'_, Sta>, &mut Rebuffer, BufferSize, BnetPtr),
) {
    with_sized_search(
        |base| BufferSize {
            intrinsic_delay: fs(1000),
            ..plain_size(base)
        },
        |resizer, rebuffer, size| {
            let exemplar = Bnet::buffer(
                Point::new(Dbu::ZERO, Dbu::ZERO),
                size,
                option_node(1.0, 500, 0.0, None),
                Fixed::ZERO,
            );
            test(resizer, rebuffer, *size, exemplar);
        },
    );
}

/// What `insert_area_options` makes of `options` for a 400 fs target and
/// a wire of over a thousand microns upstream, which the buffer cannot
/// drive so that no option stands as it is.
fn assured_options(
    resizer: &Resizer<'_, Sta>,
    rebuffer: &Rebuffer,
    exemplar: &Bnet,
    options: &[BnetPtr],
) -> Result<Vec<BnetPtr>, AreaRecoveryFailed> {
    let mut options = options.to_vec();
    rebuffer
        .insert_area_options(
            resizer,
            &mut options,
            Dbu::new(4_000_000),
            (fs(400), exemplar),
        )
        .map(|()| options)
}

#[test]
fn the_assured_option_is_the_cheapest_the_exemplars_buffer_can_drive_to_the_target() {
    with_fallback_only_search(|resizer, rebuffer, _, exemplar| {
        let unreachable = option_node(60.0, 450, 0.5, None);
        let costly = option_node(10.0, 450, 2.0, None);
        let cheap = option_node(10.0, 450, 1.0, None);
        let late = option_node(10.0, 350, 0.1, None);

        let assured = assured_options(
            resizer,
            rebuffer,
            &exemplar,
            &[unreachable, costly, Rc::clone(&cheap), late],
        )
        .unwrap();

        assert_eq!(assured.len(), 1);
        assert!(matches!(assured[0].kind, Kind::Buffer { .. }));
        assert!(Rc::ptr_eq(assured[0].inner().unwrap(), &cheap));
    });
}

#[test]
fn no_option_a_buffer_can_drive_to_the_target_leaves_no_assured_option() {
    with_fallback_only_search(|resizer, rebuffer, _, exemplar| {
        let options = [
            option_node(60.0, 450, 0.5, None),
            option_node(10.0, 350, 0.1, None),
        ];

        let outcome = assured_options(resizer, rebuffer, &exemplar, &options);

        assert_eq!(outcome.err(), Some(AreaRecoveryFailed));
    });
}

#[test]
fn the_assured_buffer_is_timed_into_the_option_and_its_own_output() {
    with_fallback_only_search(|resizer, rebuffer, size, exemplar| {
        let edges = Some(SlackEdges::Both);
        let right = timing::buffer_delay(resizer, &size, edges, Farads::new(15e-15));
        let wrong = timing::buffer_delay(resizer, &size, edges, Farads::new(5e-15));
        let gap = |a: Fixed, b: Fixed| a.to_seconds() - b.to_seconds();
        assert!(gap(right, wrong) > Seconds::new(2e-15));
        // Slack between the two delays' reach of the target: met only if
        // the load is taken as the buffer sees it, which is not met.
        let midpoint = (right.to_seconds() + wrong.to_seconds()) / 2.0;
        let slack = Fixed::from_seconds(Seconds::new(400e-15) + midpoint);
        let late = Rc::new(Bnet {
            slack,
            ..(*option_node(10.0, 0, 0.0, edges)).clone()
        });

        let assured = assured_options(resizer, rebuffer, &exemplar, &[late]);

        assert_eq!(assured.err(), Some(AreaRecoveryFailed));
    });
}

/// A constrained load of `cap_ff` femtofarads and `slack` femtoseconds at
/// `x` database units along the axis.
fn constrained_load(resizer: &Resizer<'_, Sta>, x: i64, cap_ff: f64, slack: i64) -> BnetPtr {
    Bnet::load(
        Point::new(Dbu::new(x), Dbu::ZERO),
        pin(resizer.db, "f2/D"),
        PinLoad {
            cap: Farads::new(cap_ff * 1e-15),
            fanout: 1.0,
            max_slew: Seconds::INFINITY,
        },
    )
    .with_slack(fs(slack), Some(SlackEdges::One(Edge::Rise)))
}

#[test]
fn area_recovery_on_a_tree_short_of_its_target_keeps_the_slackest_option_it_has() {
    with_sized_search(plain_size, |resizer, rebuffer, _| {
        let load = constrained_load(resizer, 20_000, 10.0, 100_000);
        let root = Bnet::wire(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            load,
            None,
            &resizer.tree_rc(),
        );

        let recovered = rebuffer.recover_area(resizer, &root, fs(1_000_000), 0.0);

        assert!(recovered.is_ok());
    });
}

#[test]
fn area_recovery_asks_each_node_for_its_target_plus_the_delay_above_it() {
    with_sized_search(plain_size, |resizer, rebuffer, size| {
        // Nothing but the load below a buffer and a wire, whose delay the
        // buffer's target must make room for: at the target exactly, a
        // load falls short of target plus that delay.
        let target = 100_000;
        let load = constrained_load(resizer, 20_000, 10.0, target);
        let buffer = Bnet::buffer(
            Point::new(Dbu::new(20_000), Dbu::ZERO),
            size,
            load,
            Fixed::ZERO,
        );
        let root = Bnet::wire(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            buffer,
            None,
            &resizer.tree_rc(),
        );
        assert!(root.delay > Fixed::ZERO);

        let recovered = rebuffer.recover_area(resizer, &root, fs(target), 1.0);

        assert_eq!(recovered.err(), Some(AreaRecoveryFailed));
    });
}

#[test]
fn a_buffer_node_takes_the_size_of_its_own_buffer_and_times_it_into_its_load() {
    let mut db = buffer_between_flops("PLACED");
    let lib = cells(&clock_cells_source());
    with_wired_session(&mut db, &lib, CLOCK, |resizer| {
        let mut rebuffer = Rebuffer::new(resizer).unwrap();
        let own = lib.buffer_cell(lib.cell_id("BUF_X1").unwrap()).unwrap();
        let other = lib.buffer_cell(lib.cell_id("CLKBUF_X3").unwrap()).unwrap();
        let size = |buffer, input_ff: f64| BufferSize {
            buffer,
            input_cap: Farads::new(input_ff * 1e-15),
            output_cap: Farads::new(5e-15),
            ..rebuffer.sizes[0]
        };
        rebuffer.sizes = vec![size(own, 2.0), size(other, 3.0)];
        let edges = Some(SlackEdges::Both);
        let load = option_node(10.0, 100_000, 0.0, edges);
        let origin = Point::new(Dbu::ZERO, Dbu::ZERO);

        let first = rebuffer.buffer_node(resizer, origin, own, Rc::clone(&load));
        let second = rebuffer.buffer_node(resizer, origin, other, Rc::clone(&load));

        assert_eq!(first.cap, rebuffer.sizes[0].input_cap);
        assert_eq!(second.cap, rebuffer.sizes[1].input_cap);
        assert_ne!(first.cap, second.cap);
        let delay =
            |size: &BufferSize| timing::buffer_delay(resizer, size, edges, Farads::new(15e-15));
        assert_eq!(first.delay, delay(&rebuffer.sizes[0]));
        assert!(first.delay > Fixed::ZERO);
    });
}

#[test]
fn a_buffer_node_of_a_buffer_the_search_does_not_use_is_sized_from_the_library() {
    with_search(|resizer, rebuffer| {
        let lib = resizer.link.liberty;
        let unused = lib.buffer_cell(lib.cell_id("CLKBUF_X3").unwrap()).unwrap();
        let load = option_node(10.0, 100_000, 0.0, None);

        let node = rebuffer.buffer_node(resizer, Point::new(Dbu::ZERO, Dbu::ZERO), unused, load);

        assert_eq!(
            node.cap,
            lib.cell(unused.id).ports[unused.input].capacitance()
        );
    });
}

/// `DFF_X1` with a 4 fF output pin.
fn flop_with_output_cap() -> pare_common::db::liberty::Liberty {
    let source = clock_cells_source();
    let at = source.find("cell (DFF_X1)").unwrap();
    let q = at + source[at..].find("pin (Q) {").unwrap();
    let mut text = source;
    text.insert_str(q + "pin (Q) {".len(), " capacitance : 4.0 ;");
    cells(&text)
}

#[test]
fn the_driver_is_timed_into_the_tree_and_its_own_output_pin() {
    let mut db = buffered_clock_pair();
    let q = pin(&db, "f1/Q");
    with_timed_session(&mut db, &flop_with_output_cap(), CLOCK, |resizer| {
        let mut rebuffer = Rebuffer::new(resizer).unwrap();
        assert!(rebuffer.set_driver(resizer, q));
        let plain = resizer.buffered_tree(q).unwrap().unwrap();
        let tree = rebuffer.annotate_load_slacks(resizer, &plain, q);
        let SlackEdges::One(edge) = tree.slack_edges.unwrap() else {
            panic!("one edge expected")
        };
        let driver = rebuffer.driver.clone().unwrap();
        let cell = resizer.link.liberty.cell(driver.cell);
        let point = driver.arrival_paths[edge].unwrap();

        let timing = rebuffer.driver_timing(resizer, &tree);

        let load = tree.cap + Farads::new(4e-15);
        let expected = timing_through(resizer, cell, &point, edge, load).unwrap();
        assert_eq!(timing.delay, expected.delay);
        assert!(expected.delay > Seconds::ZERO);
    });
}

/// Runs `test` on a wired session over [`buffered_clock_pair`] with a
/// search driven by `f1/Q` and the tree of its net timed.
fn with_timed_tree(test: impl FnOnce(&Resizer<'_, Sta>, &mut Rebuffer, &Bnet)) {
    let mut db = buffered_clock_pair();
    let q = pin(&db, "f1/Q");
    with_wired_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
        let mut rebuffer = Rebuffer::new(resizer).unwrap();
        assert!(rebuffer.set_driver(resizer, q));
        let plain = resizer.buffered_tree(q).unwrap().unwrap();
        let tree = rebuffer.annotate_load_slacks(resizer, &plain, q);
        test(resizer, &mut rebuffer, &tree);
    });
}

/// `tree` with another slew limit among its loads.
fn with_slew_limit(tree: &Bnet, limit: Seconds) -> Bnet {
    Bnet {
        kind: tree.kind.clone(),
        max_load_slew: limit,
        ..*tree
    }
}

#[test]
fn a_driver_must_reach_every_load_within_its_slew_limit_by_the_elmore_estimate() {
    with_timed_tree(|resizer, rebuffer, tree| {
        let driver = rebuffer.driver.clone().unwrap();
        let resistance = resizer
            .link
            .liberty
            .cell(driver.cell)
            .drive_resistance(driver.port);
        let wire = Meters::from_dbu(tree.max_load_wire_length(), resizer.db.dbu());
        assert!(wire > Meters::ZERO);
        // Driver and wire in series into the whole capacitance, at the
        // Elmore factor for 20 to 80 percent, against 80 percent of the
        // limit.
        let slew = (resistance + wire * resizer.graph.wire_rc().resistance) * tree.cap * 1.39;
        let limit = |scale: f64| with_slew_limit(tree, slew / 0.8 * scale);

        assert!(rebuffer.evaluate_option(resizer, &limit(1.001)).is_some());
        assert!(rebuffer.evaluate_option(resizer, &limit(0.999)).is_none());
    });
}

#[test]
fn a_driver_over_its_slew_limit_may_only_drive_what_it_has_driven_before() {
    with_timed_tree(|resizer, rebuffer, tree| {
        let slew = rebuffer.driver_timing(resizer, tree).slew;
        assert!(slew > Seconds::ZERO);
        let drive = |rebuffer: &mut Rebuffer, limit: Seconds, seen: Farads| {
            let driver = rebuffer.driver.as_mut().unwrap();
            driver.max_slew = limit;
            driver.load_high_water_mark = seen;
            rebuffer.evaluate_option(resizer, tree).is_some()
        };

        // Over the limit and heavier than any seen: refused. Not heavier,
        // or not over: driven.
        assert!(!drive(rebuffer, slew / 2.0, Farads::ZERO));
        assert!(drive(rebuffer, slew / 2.0, tree.cap));
        assert!(drive(rebuffer, slew / 2.0, tree.cap * 2.0));
        assert!(drive(rebuffer, slew, Farads::ZERO));
        assert!(drive(rebuffer, slew * 2.0, Farads::ZERO));
        // A drive raises the mark to the load's, from none seen.
        let driver = rebuffer.driver.as_ref().unwrap();
        assert_eq!(driver.load_high_water_mark, tree.cap);
    });
}

#[test]
fn a_junction_pairs_its_sides_options_from_the_most_slack_down_by_falling_capacitance() {
    with_search(|resizer, rebuffer| {
        let side = |options: &[(f64, i64)]| -> Vec<BnetPtr> {
            options
                .iter()
                .map(|&(cap, slack)| option_node(cap, slack, 0.0, None))
                .collect()
        };
        let left = side(&[(3.0, 30), (6.0, 40)]);
        let right = side(&[(2.0, 30), (5.0, 40), (7.0, 70)]);
        let node = Bnet::junction(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            Rc::clone(&left[1]),
            Rc::clone(&right[2]),
        );
        let sides = JunctionSides {
            left: &left,
            right: &right,
        };

        let options = rebuffer.junction_options(resizer, &node, sides, Topology::Kept);

        // Each side steps down while the other has no more slack, so the
        // pairs are those two, and the one lighter is listed first.
        let pairs: Vec<(Farads, Fixed)> = options.iter().map(|o| (o.cap, o.slack)).collect();
        assert_eq!(
            pairs,
            [
                (left[0].cap + right[0].cap, fs(30)),
                (left[1].cap + right[1].cap, fs(40))
            ]
        );
    });
}

#[test]
fn a_junction_with_a_side_without_options_has_none() {
    with_search(|resizer, rebuffer| {
        let load = [option_node(3.0, 30, 0.0, None)];
        let node = Bnet::junction(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            Rc::clone(&load[0]),
            Rc::clone(&load[0]),
        );
        for (left, right) in [
            (&load[..], &[][..]),
            (&[][..], &load[..]),
            (&[][..], &[][..]),
        ] {
            let sides = JunctionSides { left, right };

            assert!(
                rebuffer
                    .junction_options(resizer, &node, sides, Topology::Kept)
                    .is_empty()
            );
        }
    });
}

#[test]
fn the_loads_of_a_net_an_input_port_drives_are_given_the_slack_of_their_worst_path() {
    let mut db = design(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN ports ;\n\
         UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
         COMPONENTS 1 ;\n- bf BUF_X1 + PLACED ( 30000 2800 ) N ;\nEND COMPONENTS\n\
         PINS 3 ;\n- clk + NET clk + DIRECTION INPUT ;\n- in + NET in + DIRECTION INPUT ;\n\
         - out + NET out + DIRECTION OUTPUT ;\nEND PINS\n\
         NETS 3 ;\n- clk ( PIN clk ) ;\n- in ( PIN in ) ( bf A ) ;\n\
         - out ( bf Z ) ( PIN out ) ;\nEND NETS\nEND DESIGN\n",
    );
    let port = db.io_pins[db.io_pin_id("in").unwrap()].pin;
    let constraints = format!(
        "{CLOCK}set_input_delay 0.1 -clock clk [get_ports in]\n\
         set_output_delay 0.1 -clock clk [get_ports out]\n"
    );
    with_wired_session(
        &mut db,
        &cells(&clock_cells_source()),
        &constraints,
        |resizer| {
            let mut rebuffer = Rebuffer::new(resizer).unwrap();
            let plain = resizer.buffered_tree(port).unwrap().unwrap();

            let tree = rebuffer.annotate_load_slacks(resizer, &plain, port);

            assert!(tree.slack_edges.is_some());
            assert!(tree.slack < Fixed::INF);
        },
    );
}

#[test]
fn a_load_limit_found_by_the_slew_growing_slowly_expands_the_guess_and_then_bisects() {
    // Output slew of 0.15 ns at 1 fF and 0.17 ns at 100 fF, growing along
    // that line beyond, against 80 percent of a 0.3 ns limit: 0.24 ns, at
    // 446.5 fF. The first guess, 0.24 ns over the 1.7 kilohm drive twice,
    // is 282.4 fF, short of it, so it doubles to 564.7 fF; the bisection
    // between them stops at 445.588 fF, within a hundredth.
    let cell = "cell (GENTLE) { area : 1.0 ; \
        pin (A) { direction : input ; capacitance : 1.0 ; } \
        pin (Z) { direction : output ; function : \"A\" ; max_capacitance : 1000.0 ; \
        max_transition : 0.3 ; \
        timing () { related_pin : \"A\" ; timing_sense : positive_unate ; \
        cell_rise (delay_t) { values (\"0.03, 0.30\", \"0.04, 0.31\") ; } \
        cell_fall (delay_t) { values (\"0.03, 0.30\", \"0.04, 0.31\") ; } \
        rise_transition (delay_t) { values (\"0.15, 0.17\", \"0.15, 0.17\") ; } \
        fall_transition (delay_t) { values (\"0.15, 0.17\", \"0.15, 0.17\") ; } } } }\n";
    let mut db = buffer_between_flops("PLACED");
    let lib = cells_with(cell);
    with_wired_session(&mut db, &lib, CLOCK, |resizer| {
        let buffer = lib.buffer_cell(lib.cell_id("GENTLE").unwrap()).unwrap();

        let limit = buffer_size(resizer, buffer).margined_max_cap;

        assert!(((limit.get() - 4.455_882_352_941_176e-13) / limit.get()).abs() < 1e-6);
    });
}
