use super::reservation::LowPads;
use super::whole_net::waypoints;
use super::*;
use crate::tech::RouteSetup;

fn cell(layer: u8, x: u16, y: u16) -> GCell {
    GCell {
        layer: LayerId::new(usize::from(layer)),
        x,
        y,
    }
}

#[test]
fn waypoints_split_a_corridor_every_few_gcells_and_ignore_layer_changes() {
    let mut cells = vec![cell(1, 0, 0), cell(2, 0, 0)];
    cells.extend((1..=10).map(|x| cell(2, x, 0)));

    let stops = waypoints(&cells, 4);

    assert_eq!(stops, [5, 9]);
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

#[test]
fn a_pad_closer_than_the_spacing_to_another_nets_pad_is_not_clear() {
    let mut pads = LowPads::new(Dbu::new(1000));
    pads.add(LayerId::new(0), rect(0, 0, 140, 70), NetId(1));

    let near = rect(260, 0, 400, 70);
    let far = rect(270, 0, 410, 70);

    assert!(!pads.clear(LayerId::new(0), &near, NetId(2), Dbu::new(130)));
    assert!(pads.clear(LayerId::new(0), &far, NetId(2), Dbu::new(130)));
}

#[test]
fn a_pad_is_clear_of_its_own_nets_pads_and_other_layers() {
    let mut pads = LowPads::new(Dbu::new(1000));
    pads.add(LayerId::new(0), rect(0, 0, 140, 70), NetId(1));

    let touching = rect(140, 0, 280, 70);

    assert!(pads.clear(LayerId::new(0), &touching, NetId(1), Dbu::new(130)));
    assert!(pads.clear(LayerId::new(1), &touching, NetId(2), Dbu::new(130)));
}

#[test]
fn pads_are_found_across_bucket_edges() {
    let mut pads = LowPads::new(Dbu::new(100));
    pads.add(LayerId::new(0), rect(150, 150, 190, 190), NetId(1));

    let across = rect(220, 150, 260, 190);

    assert!(!pads.clear(LayerId::new(0), &across, NetId(2), Dbu::new(65)));
}

#[test]
fn speculative_routes_do_not_depend_on_the_thread_count() {
    let (db, config) = crate::tests::placed_tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
    let gcell = crate::gcell_size(&tech, &config);
    let costs = round_costs(0, crate::VIA_COST_PITCHES * tech.min_layer_pitch.to_f64());
    let route_with = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        pool.install(|| {
            let mut dr = DetailedRouter::new(&db, &tech, gcell);
            dr.route_speculatively(routable_nets(&db), costs);
            dr.routes
        })
    };

    let one = route_with(1);
    let eight = route_with(8);

    assert_eq!(one, eight);
}

#[test]
fn a_short_corridor_has_no_waypoints() {
    let cells: Vec<GCell> = (0..=5).map(|x| cell(2, x, 0)).collect();

    assert_eq!(waypoints(&cells, 4), []);
}
