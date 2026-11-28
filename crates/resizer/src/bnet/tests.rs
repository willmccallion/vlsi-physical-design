use super::*;
use crate::library::Link;
use crate::testing::{DefSections, cells, clock_cells_source, def_text, design};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::CutLayerId;

fn pin_load(cap: f64, max_slew: f64) -> PinLoad {
    PinLoad {
        cap: Farads::new(cap),
        fanout: 1.0,
        max_slew: Seconds::new(max_slew),
    }
}

#[test]
fn fixed_delays_round_to_femtoseconds_and_saturate() {
    assert_eq!(Fixed::from_seconds(Seconds::new(1.5e-12)), Fixed(1500));
    assert_eq!(Fixed::from_seconds(Seconds::new(1e3)), Fixed::INF);
    assert_eq!(Fixed::from_seconds(Seconds::new(-1e3)), -Fixed::INF);
    assert_eq!(Fixed::lerp(Fixed(0), Fixed(100), 0.25), Fixed(25));
    assert!((Fixed(2000).to_seconds() - Seconds::new(2e-12)).abs() < Seconds::new(1e-24));
}

#[test]
fn a_wire_takes_its_elmore_delay_off_the_slack() {
    let rc = WireRc {
        resistance: OhmsPerMeter::from_ohms_per_micron(1.0),
        capacitance: FaradsPerMeter::from_femtofarads_per_micron(1.0),
        layer: pare_common::db::indices::LayerId::new(0),
    };
    let load = Bnet::load(
        Point::new(Dbu::new(2000), Dbu::ZERO),
        PinId::new(0),
        pin_load(1e-15, 1e-9),
    );
    let load = load.with_slack(
        Fixed::from_seconds(Seconds::new(1e-9)),
        Some(SlackEdges::One(Edge::Rise)),
    );

    let tree_rc = TreeRc {
        signal: rc,
        layers: None,
        dbu: DbuPerMicron::new(2000).unwrap(),
    };

    let wire = Bnet::wire(Point::new(Dbu::ZERO, Dbu::ZERO), load, None, &tree_rc);

    // One micron of wire: 1 ohm and 1 fF; delay = 1 * (0.5 fF + 1 fF).
    assert_eq!(wire.length(), Dbu::new(2000));
    assert!((wire.cap - Farads::new(2e-15)).abs() < Farads::new(1e-30));
    assert_eq!(wire.delay, Fixed(1));
    assert_eq!(wire.slack, Fixed(1_000_000 - 1));
    assert_eq!(wire.max_load_wire_length(), Dbu::new(2000));
}

#[test]
fn a_junction_combines_its_sides() {
    let a = Bnet::load(
        Point::new(Dbu::ZERO, Dbu::ZERO),
        PinId::new(0),
        pin_load(1e-15, 2e-9),
    )
    .with_slack(Fixed(10), Some(SlackEdges::One(Edge::Rise)));
    let b = Bnet::load(
        Point::new(Dbu::ZERO, Dbu::ZERO),
        PinId::new(1),
        pin_load(2e-15, 1e-9),
    )
    .with_slack(Fixed(5), Some(SlackEdges::One(Edge::Fall)));

    let junction = Bnet::junction(Point::new(Dbu::ZERO, Dbu::ZERO), a, b);

    assert!((junction.cap - Farads::new(3e-15)).abs() < Farads::new(1e-30));
    assert!((junction.fanout - 2.0).abs() < 1e-12);
    assert!((junction.max_load_slew - Seconds::new(1e-9)).abs() < Seconds::new(1e-24));
    assert_eq!(junction.slack, Fixed(5));
    assert_eq!(junction.slack_edges, Some(SlackEdges::Both));
    assert_eq!(junction.loads(), vec![PinId::new(0), PinId::new(1)]);
}

const DBU: u32 = 2000;

fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

fn load_at(x: i64, y: i64, pin: u32, cap: f64) -> BnetPtr {
    Bnet::load(pt(x, y), PinId::new(pin as usize), pin_load(cap, 1e-9))
}

fn signal_rc(layers: Option<&RouteRc>) -> TreeRc<'_> {
    TreeRc {
        signal: WireRc {
            resistance: OhmsPerMeter::from_ohms_per_micron(1.0),
            capacitance: FaradsPerMeter::from_femtofarads_per_micron(1.0),
            layer: LayerId::new(0),
        },
        layers,
        dbu: DbuPerMicron::new(DBU).unwrap(),
    }
}

fn empty_design() -> NetlistDB {
    design(&def_text(
        "empty",
        &DefSections {
            components: "",
            nets: "",
            inputs: &[],
            outputs: &[],
        },
    ))
}

/// Cut layers 0 and 1 of 10 and 20 ohms per cut.
fn via_rc(db: &NetlistDB) -> RouteRc {
    RouteRc::new(
        &db.library,
        &[],
        &[
            (CutLayerId::new(0), Ohms::new(10.0)),
            (CutLayerId::new(1), Ohms::new(20.0)),
        ],
        LayerId::new(0),
    )
}

#[test]
fn slack_edges_combine_to_the_one_they_share_or_to_both() {
    let rise = Some(SlackEdges::One(Edge::Rise));
    let fall = Some(SlackEdges::One(Edge::Fall));

    assert_eq!(SlackEdges::combine(None, rise), rise);
    assert_eq!(SlackEdges::combine(fall, None), fall);
    assert_eq!(SlackEdges::combine(None, None), None);
    assert_eq!(SlackEdges::combine(rise, rise), rise);
    assert_eq!(SlackEdges::combine(rise, fall), Some(SlackEdges::Both));
    assert_eq!(
        SlackEdges::combine(Some(SlackEdges::Both), rise),
        Some(SlackEdges::Both)
    );
}

#[test]
fn a_via_takes_the_resistance_of_every_cut_it_crosses_off_the_slack() {
    let db = empty_design();
    let layers = via_rc(&db);
    let rc = signal_rc(Some(&layers));
    let below = load_at(0, 0, 0, 2e-15).with_slack(Fixed(1_000_000), None);

    let up = Bnet::via(
        pt(0, 0),
        LayerId::new(0),
        LayerId::new(2),
        below.clone(),
        &rc,
    );
    let down = Bnet::via(pt(0, 0), LayerId::new(2), LayerId::new(0), below, &rc);

    // Cuts 0 and 1 in series: 30 ohms into 2 fF is 60 fs.
    for via in [up, down] {
        assert_eq!(via.delay, Fixed(60));
        assert_eq!(via.slack, Fixed(1_000_000 - 60));
    }
}

#[test]
fn tree_rc_finds_a_via_by_the_cuts_between_its_layers() {
    let db = empty_design();
    let layers = via_rc(&db);
    let rc = signal_rc(Some(&layers));

    assert_eq!(rc.via(LayerId::new(1), LayerId::new(2)), Ohms::new(20.0));
    assert_eq!(rc.via(LayerId::new(2), LayerId::new(0)), Ohms::new(30.0));
    assert_eq!(rc.via(LayerId::new(1), LayerId::new(1)), Ohms::ZERO);
    assert_eq!(
        signal_rc(None).via(LayerId::new(0), LayerId::new(2)),
        Ohms::ZERO
    );
}

#[test]
fn a_wire_and_a_via_report_their_layers_and_what_they_reach() {
    let db = empty_design();
    let layers = via_rc(&db);
    let rc = signal_rc(Some(&layers));
    let leaf = load_at(2000, 0, 0, 1e-15);
    let wire = Bnet::wire(pt(0, 0), leaf.clone(), Some(LayerId::new(1)), &rc);
    let unlayered = Bnet::wire(pt(0, 0), leaf.clone(), None, &rc);
    let via = Bnet::via(
        pt(0, 0),
        LayerId::new(0),
        LayerId::new(1),
        wire.clone(),
        &rc,
    );

    assert_eq!(wire.layer(), Some(LayerId::new(1)));
    assert_eq!(unlayered.layer(), None);
    assert_eq!(via.layer(), Some(LayerId::new(0)));
    assert_eq!(leaf.layer(), None);
    assert!(Rc::ptr_eq(wire.inner().unwrap(), &leaf));
    assert!(Rc::ptr_eq(via.inner().unwrap(), &wire));
    assert!(leaf.inner().is_none());
}

#[test]
fn the_wire_layer_below_is_found_through_vias_but_not_past_a_load() {
    let db = empty_design();
    let layers = via_rc(&db);
    let rc = signal_rc(Some(&layers));
    let leaf = load_at(2000, 0, 0, 1e-15);
    let wire = Bnet::wire(pt(0, 0), leaf.clone(), Some(LayerId::new(1)), &rc);
    let via = Bnet::via(
        pt(0, 0),
        LayerId::new(0),
        LayerId::new(1),
        wire.clone(),
        &rc,
    );
    let unlayered = Bnet::wire(pt(0, 0), leaf.clone(), None, &rc);
    let junction = Bnet::junction(pt(0, 0), wire.clone(), leaf.clone());

    assert_eq!(wire.wire_layer_below(), Some(LayerId::new(1)));
    assert_eq!(via.wire_layer_below(), Some(LayerId::new(1)));
    assert_eq!(unlayered.wire_layer_below(), None);
    assert_eq!(leaf.wire_layer_below(), None);
    assert_eq!(junction.wire_layer_below(), None);
}

#[test]
fn a_subtree_fits_only_when_no_worse_than_the_target_in_every_measure() {
    let rc = signal_rc(None);
    let node = Bnet::wire(pt(0, 0), load_at(2000, 0, 0, 1e-15), None, &rc)
        .with_slack(Fixed(500), Some(SlackEdges::Both));
    let target = node.metrics();
    assert!(node.fits(&target));

    let tighter = [
        Metrics {
            max_load_wire_length: Dbu::new(1999),
            ..target
        },
        target.with_slack(Fixed(501)),
        Metrics {
            cap: target.cap - Farads::new(1e-15),
            ..target
        },
        Metrics {
            max_load_slew: target.max_load_slew * 2.0,
            ..target
        },
        Metrics {
            fanout: target.fanout - 1.0,
            ..target
        },
    ];
    for metrics in tighter {
        assert!(!node.fits(&metrics), "{metrics:?}");
    }
}

#[test]
fn a_subtree_with_more_slack_and_less_of_everything_else_fits_a_looser_target() {
    let rc = signal_rc(None);
    let node = Bnet::wire(pt(0, 0), load_at(2000, 0, 0, 1e-15), None, &rc)
        .with_slack(Fixed(500), Some(SlackEdges::Both));
    let target = node.metrics();
    let looser = Metrics {
        max_load_wire_length: Dbu::new(3000),
        slack: Fixed(100),
        cap: target.cap * 2.0,
        max_load_slew: target.max_load_slew / 2.0,
        fanout: target.fanout + 1.0,
    };

    assert!(node.fits(&looser));
}

#[test]
fn metrics_with_another_slack_keep_every_other_measure() {
    let rc = signal_rc(None);
    let node = Bnet::wire(pt(0, 0), load_at(2000, 0, 0, 1e-15), None, &rc);
    let metrics = node.metrics();

    let moved = metrics.with_slack(Fixed(77));

    assert_eq!(moved.slack, Fixed(77));
    assert_ne!(metrics.slack, moved.slack);
    assert_eq!(
        Metrics {
            slack: metrics.slack,
            ..moved
        },
        metrics
    );
}

#[test]
fn a_subtree_over_scattered_sinks_reaches_each_of_them_once() {
    let rc = signal_rc(None);
    let sinks = [
        load_at(8000, 0, 0, 1e-15),
        load_at(0, 8000, 1, 1e-15),
        load_at(8000, 8000, 2, 1e-15),
    ];

    let tree = over_subtrees(pt(0, 0), &sinks, &rc).unwrap();

    let mut loads = tree.loads();
    loads.sort_unstable();
    assert_eq!(loads, [PinId::new(0), PinId::new(1), PinId::new(2)]);
    assert!(over_subtrees(pt(0, 0), &[], &rc).is_none());
}

struct UnitLoads;

impl LoadModel for UnitLoads {
    fn load(&self, _: PinId) -> PinLoad {
        pin_load(1e-15, 1e-9)
    }
}

#[test]
fn a_steiner_tree_hangs_loads_but_neither_the_driver_nor_another_driver_on_the_net() {
    let db = design(&def_text(
        "twodrivers",
        &DefSections {
            components: "- d1 INV_X1 + PLACED ( 2000 2800 ) N ;\n\
                         - d2 INV_X1 + PLACED ( 20000 2800 ) N ;\n\
                         - a INV_X1 + PLACED ( 2000 22400 ) N ;\n",
            nets: "- n ( d1 ZN ) ( d2 ZN ) ( a A ) ;\n",
            inputs: &[],
            outputs: &[],
        },
    ));
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let pins = ["d1/ZN", "d2/ZN", "a/A"].map(|name| crate::testing::pin(&db, name));
    let points: Vec<Point<Dbu>> = pins.iter().map(|&p| netlist.pin_location(p)).collect();
    let tree = steiner::prim_dijkstra(&points, 0, steiner::DEFAULT_ALPHA);
    let terminal_pins: Vec<Vec<PinId>> = pins.iter().map(|&p| vec![p]).collect();

    let buffered = from_steiner(
        &netlist,
        &tree,
        &terminal_pins,
        pins[0],
        &signal_rc(None),
        &UnitLoads,
    )
    .unwrap();

    assert_eq!(buffered.loads(), [pins[2]]);
}
