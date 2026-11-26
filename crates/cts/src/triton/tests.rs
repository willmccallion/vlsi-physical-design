use super::*;
use crate::fixtures;
use pare_common::db::rules::WidthRangeSpacing;

/// A tree rooted at node 0 over nodes at `xs` along the x axis.
fn tree(xs: &[i64], parent: &[Option<usize>]) -> SteinerTree {
    SteinerTree {
        nodes: xs
            .iter()
            .map(|&x| Point::new(Dbu::new(x), Dbu::new(0)))
            .collect(),
        parent: parent.to_vec(),
        terminals: xs.len(),
        root: 0,
    }
}

#[test]
fn short_edges_join_their_nodes_and_long_edges_part_them() {
    let tree = tree(&[0, 10, 1000, 1010], &[None, Some(0), Some(1), Some(2)]);

    let clusters = long_edge_clusters(&tree, Dbu::new(100));

    assert_eq!(clusters, vec![vec![1, 0], vec![2, 3]]);
}

#[test]
fn an_edge_as_long_as_the_limit_is_long() {
    let tree = tree(&[0, 100], &[None, Some(0)]);

    let clusters = long_edge_clusters(&tree, Dbu::new(100));

    assert_eq!(clusters, vec![vec![1], vec![0]]);
}

#[test]
fn a_short_edge_between_clusters_moves_the_smaller_into_the_larger() {
    // Node 4's edge to the root joins {1, 0} into {2, 3, 4}, leaving
    // the smaller cluster empty in its place.
    let tree = tree(
        &[0, 1, 2, 3, 4],
        &[None, Some(0), Some(3), Some(4), Some(0)],
    );

    let clusters = long_edge_clusters(&tree, Dbu::new(100));

    assert_eq!(clusters, vec![vec![], vec![2, 3, 4, 1, 0]]);
}

#[test]
fn blocks_closer_than_a_buffer_merge_into_one_blockage() {
    let db = fixtures::design(
        "COMPONENTS 3 ;\n\
             - a RAM + PLACED ( 0 0 ) N ;\n\
             - b RAM + PLACED ( 40100 0 ) N ;\n\
             - c RAM + PLACED ( 200000 200000 ) N ;\n\
             END COMPONENTS\n",
    );

    let (blockages, diameter) = merge_blockages(&db, Point::new(Dbu::new(100), Dbu::new(100)));

    let corners = |x0, y0, x1, y1| {
        Rect::new(
            Point::new(Dbu::new(x0), Dbu::new(y0)),
            Point::new(Dbu::new(x1), Dbu::new(y1)),
        )
    };
    assert_eq!(
        blockages,
        [
            corners(0, 0, 80_100, 40_000),
            corners(200_000, 200_000, 240_000, 240_000)
        ]
    );
    let diameter = diameter.map(Microns::get).unwrap();
    assert!((diameter - 40.0).abs() < 1e-12, "{diameter}");
}

#[test]
fn without_blocks_macro_clusters_get_no_diameter() {
    let db = fixtures::design("");

    let (blockages, diameter) = merge_blockages(&db, Point::new(Dbu::new(100), Dbu::new(100)));

    assert!(blockages.is_empty());
    assert_eq!(diameter, None);
}

#[test]
fn a_pin_without_shapes_sits_at_its_reference_point() {
    let mut db = fixtures::design(
        "COMPONENTS 1 ;\n- ram RAM + PLACED ( 100000 100000 ) N ;\nEND COMPONENTS\n",
    );
    let ram = db.cell_id("ram").unwrap();
    let bare = db.cell_macro(ram).pin_id("CLKBARE").unwrap();
    let net = db.add_net("n").unwrap();
    let pin = db.connect_cell_pin(ram, bare, net);

    assert_eq!(
        pin_position(&db, pin),
        Point::new(Dbu::new(120_000), Dbu::new(120_000))
    );
}

fn metal1() -> pare_common::db::library::RoutingLayer {
    let db = fixtures::design("");
    db.library.routing_layers.iter().next().unwrap().clone()
}

#[test]
fn a_layers_net_spacing_is_its_min_spacing_without_other_rules() {
    let layer = metal1();

    assert_eq!(net_spacing(&layer, layer.width), Dbu::new(130));
}

#[test]
fn a_width_range_rule_covering_the_width_sets_the_net_spacing() {
    let mut layer = metal1();
    layer.rules.width_range_spacing = vec![
        WidthRangeSpacing {
            spacing: Dbu::new(300),
            min_width: Dbu::new(500),
            max_width: Dbu::new(1000),
        },
        WidthRangeSpacing {
            spacing: Dbu::new(200),
            min_width: Dbu::new(0),
            max_width: Dbu::new(1000),
        },
    ];

    assert_eq!(net_spacing(&layer, layer.width), Dbu::new(200));
}

#[test]
fn without_any_spacing_the_net_spacing_is_the_gap_between_tracks() {
    let mut layer = metal1();
    layer.min_spacing = Dbu::ZERO;

    assert_eq!(net_spacing(&layer, layer.width), Dbu::new(140));
}
