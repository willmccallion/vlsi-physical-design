use super::*;

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
