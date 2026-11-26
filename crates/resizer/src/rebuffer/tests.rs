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
