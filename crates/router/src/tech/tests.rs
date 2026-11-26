use super::*;

/// The tiny fixture's tech on all its layers, M1 to M3.
fn tiny_tech() -> RouteTech {
    let config = RoutingConfig {
        min_layer: Some("M1".to_string()),
        ..RoutingConfig::default()
    };
    RouteTech::new(&crate::tests::tiny(), &RouteSetup::from(&config)).unwrap()
}

/// The first crossing of `layer` that `wanted` accepts, on its first
/// track.
fn first_crossing(tech: &RouteTech, layer: LayerId, wanted: impl Fn(&Cross) -> bool) -> Node {
    let cross = tech
        .layer(layer)
        .cross
        .iter_enumerated()
        .find(|(_, c)| wanted(c))
        .unwrap()
        .0;
    Node {
        layer,
        track: TrackIdx(0),
        cross,
    }
}

#[test]
fn a_jog_may_leave_a_crossing_under_a_track_of_the_layer_above() {
    let tech = tiny_tech();

    let n = first_crossing(&tech, LayerId(1), |c| c.up.is_some());

    assert!(tech.jog_allowed(n));
}

#[test]
fn no_jog_leaves_a_crossing_that_only_the_layer_below_crosses() {
    let tech = tiny_tech();

    let n = first_crossing(&tech, LayerId(1), |c| c.up.is_none());

    assert!(!tech.jog_allowed(n));
}

#[test]
fn on_the_top_layer_a_jog_runs_along_a_track_of_the_layer_below() {
    let tech = tiny_tech();

    let n = first_crossing(&tech, tech.max_layer, |c| c.down.is_some());

    assert!(tech.jog_allowed(n));
}

#[test]
fn no_jog_leaves_a_layer_s_last_track() {
    let tech = tiny_tech();
    let lt = tech.layer(LayerId(1));
    let up = first_crossing(&tech, LayerId(1), |c| c.up.is_some());

    let last = Node {
        track: TrackIdx::new(lt.tracks.len() - 1),
        ..up
    };

    assert!(!tech.jog_allowed(last));
}

fn die(width: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(0), Dbu::new(0)),
        Point::new(Dbu::new(width), Dbu::new(10)),
    )
}

#[test]
fn a_die_of_as_many_gcells_as_the_grid_holds_is_routable() {
    assert_eq!(
        check_grid_span(&die(65_535 * 100), Dbu::new(100), 2),
        Ok(())
    );
}

#[test]
fn a_die_past_the_grid_s_span_is_refused_with_its_gcells() {
    assert_eq!(
        check_grid_span(&die(65_535 * 100 + 1), Dbu::new(100), 2),
        Err(RouteError::GridTooLarge(Gcells(65_536)))
    );
}

#[test]
fn a_grid_with_more_nodes_than_32_bits_index_is_refused() {
    let square = Rect::new(
        Point::new(Dbu::new(0), Dbu::new(0)),
        Point::new(Dbu::new(40_000), Dbu::new(40_000)),
    );

    assert_eq!(
        check_grid_span(&square, Dbu::new(1), 3),
        Err(RouteError::GridTooManyNodes(4_800_000_000))
    );
}

fn spacings(of: &[i64]) -> impl Fn(LayerId) -> Dbu + '_ {
    let of: IdVec<LayerId, i64> = of.to_vec().into();
    move |l| Dbu::new(of[l])
}

#[test]
fn gcells_are_sized_by_the_middle_spacing_of_the_second_to_fourth_layers() {
    let pitch = gcell_pitch(spacings(&[280, 380, 280, 560, 560]), LayerId(4));

    assert_eq!(pitch, Dbu::new(380));
}

#[test]
fn an_odd_track_spacing_is_rounded_down_to_an_even_one() {
    let pitch = gcell_pitch(spacings(&[200, 401, 401, 401]), LayerId(3));

    assert_eq!(pitch, Dbu::new(400));
}

#[test]
fn below_four_routing_layers_the_highest_layer_sizes_the_gcells() {
    let pitch = gcell_pitch(spacings(&[200, 300, 500]), LayerId(2));

    assert_eq!(pitch, Dbu::new(500));
}
