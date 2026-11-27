use super::*;

fn ns(value: f64) -> Seconds {
    Seconds::new(value * 1e-9)
}

#[test]
fn a_merged_value_keeps_the_later_on_the_late_side_and_the_earlier_on_the_early_side() {
    let mut values = RiseFallMinMax::default();
    assert!(values.is_empty());

    values.merge(&[Edge::Rise], &[MinMax::Max], ns(1.0));
    values.merge(&[Edge::Rise], &[MinMax::Max], ns(2.0));
    values.merge(&[Edge::Rise], &[MinMax::Min], ns(2.0));
    values.merge(&[Edge::Rise], &[MinMax::Min], ns(1.0));

    assert!(!values.is_empty());
    assert_eq!(values.get(Edge::Rise, MinMax::Max), Some(ns(2.0)));
    assert_eq!(values.get(Edge::Rise, MinMax::Min), Some(ns(1.0)));
    assert_eq!(values.get(Edge::Fall, MinMax::Max), None);
}

fn clock(port: usize) -> Clock {
    let mut uncertainty = SideValues::both(None);
    uncertainty[MinMax::Max] = Some(ns(0.1));
    Clock {
        name: format!("c{port}"),
        period: ns(2.0),
        rise: Seconds::ZERO,
        fall: ns(1.0),
        sources: vec![IoPinId::new(port)],
        uncertainty,
        propagated: false,
    }
}

fn two_clocks() -> Sdc {
    let mut sdc = Sdc::default();
    let _ = sdc.clocks.push(clock(0));
    let _ = sdc.clocks.push(clock(1));
    sdc
}

#[test]
fn a_clock_has_the_uncertainty_set_on_a_side_and_none_on_the_other() {
    let c = clock(0);

    assert_eq!(c.uncertainty(MinMax::Max), ns(0.1));
    assert_eq!(c.uncertainty(MinMax::Min), Seconds::ZERO);
}

#[test]
fn an_edge_happens_at_its_clock_s_rise_or_fall() {
    let sdc = two_clocks();
    let fall = ClockEdge {
        clock: ClockId(1),
        edge: Edge::Fall,
    };

    assert_eq!(sdc.edge_time(fall), ns(1.0));
}

#[test]
fn a_clock_is_found_by_the_port_it_enters_at() {
    let sdc = two_clocks();

    assert_eq!(sdc.clock_at(IoPinId::new(1)), Some(ClockId(1)));
    assert_eq!(sdc.clock_at(IoPinId::new(5)), None);
    assert_eq!(
        sdc.clock_ports().collect::<Vec<_>>(),
        [IoPinId::new(0), IoPinId::new(1)]
    );
}

fn spec(from_clocks: &[u32], to_clocks: &[u32], to_ports: &[usize]) -> PathSpec {
    PathSpec {
        from_clocks: from_clocks.iter().map(|&c| ClockId(c)).collect(),
        to_clocks: to_clocks.iter().map(|&c| ClockId(c)).collect(),
        to_ports: to_ports.iter().map(|&p| IoPinId::new(p)).collect(),
        ..PathSpec::default()
    }
}

#[test]
fn a_path_spec_matches_only_the_ends_it_names() {
    let from = |c| PathEnd::Clock(ClockId(c));
    let port = |p| PathEnd::Port(IoPinId::new(p));

    assert!(spec(&[], &[], &[]).matches(from(0), port(3), None));
    assert!(spec(&[0], &[], &[]).matches(from(0), port(3), None));
    assert!(!spec(&[0], &[], &[]).matches(from(1), port(3), None));
    assert!(spec(&[], &[], &[3]).matches(from(1), port(3), None));
    assert!(!spec(&[], &[], &[3]).matches(from(1), port(4), None));
    assert!(spec(&[], &[1], &[]).matches(from(0), port(4), Some(ClockId(1))));
    assert!(!spec(&[], &[1], &[]).matches(from(0), port(4), Some(ClockId(0))));
    assert!(!spec(&[0], &[], &[3]).matches(from(0), port(4), None));
}

#[test]
fn a_multicycle_path_without_a_side_holds_at_zero_on_the_early_side() {
    let path = |side| MultiCyclePath {
        side,
        multiplier: 3,
        use_end_clock: true,
        paths: PathSpec::default(),
    };

    assert_eq!(path(None).multiplier(MinMax::Min), 0);
    assert_eq!(path(None).multiplier(MinMax::Max), 3);
    assert_eq!(path(Some(MinMax::Max)).multiplier(MinMax::Min), 3);
}

#[test]
fn a_false_path_on_one_side_leaves_the_other_side_checked() {
    let mut sdc = two_clocks();
    sdc.false_paths.push(FalsePath {
        side: Some(MinMax::Max),
        paths: spec(&[0], &[], &[]),
    });
    let (from, to) = (PathEnd::Clock(ClockId(0)), PathEnd::Port(IoPinId::new(3)));

    assert!(sdc.is_false_path(from, to, None, MinMax::Max));
    assert!(!sdc.is_false_path(from, to, None, MinMax::Min));
    assert!(!sdc.is_false_path(PathEnd::Clock(ClockId(1)), to, None, MinMax::Max));
}
