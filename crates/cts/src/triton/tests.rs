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

/// The fanout limit the settings take from the root and sink buffers'.
fn max_fanout(root: Option<u32>, sink: Option<u32>) -> Result<Option<u32>, CtsError> {
    fanout_settings(CtsOptions::default(), BufferFanouts { root, sink }).map(|s| s.max_fanout)
}

#[test]
fn a_root_fanout_limit_below_the_leaf_sinks_limits_the_trees() {
    assert_eq!(
        max_fanout(Some(MAX_LEAF_SINKS - 1), None),
        Ok(Some(MAX_LEAF_SINKS - 1))
    );
}

#[test]
fn a_root_fanout_limit_of_the_leaf_sinks_or_more_does_not() {
    assert_eq!(max_fanout(Some(MAX_LEAF_SINKS), None), Ok(None));
}

#[test]
fn the_lower_of_the_root_and_sink_fanout_limits_is_taken() {
    let limits = [(10, 8), (8, 10)].map(|(root, sink)| max_fanout(Some(root), Some(sink)));

    assert_eq!(limits, [Ok(Some(8)), Ok(Some(8))]);
}

#[test]
fn a_fanout_limit_of_one_branch_per_level_is_enough() {
    assert_eq!(
        max_fanout(Some(TREE_BRANCHES), None),
        Ok(Some(TREE_BRANCHES))
    );
}

#[test]
fn a_fanout_limit_below_the_branches_of_a_level_is_refused() {
    let limit = TREE_BRANCHES - 1;

    assert_eq!(
        max_fanout(Some(limit), None),
        Err(CtsError::FanoutLimitTooLow(limit))
    );
}

#[test]
fn a_pin_of_several_shapes_sits_at_the_mean_of_their_centers() {
    let mut db = fixtures::design(
        "COMPONENTS 1 ;\n- ram RAM + PLACED ( 100000 100000 ) N ;\nEND COMPONENTS\n",
    );
    let ram = db.cell_id("ram").unwrap();
    let master = db.cells[ram].macro_id;
    let clk = db.cell_macro(ram).pin_id("CLK").unwrap();
    let shapes = &mut db.library.macros[master].pins[clk].shapes;
    let mut moved = shapes[0];
    moved.rect = moved
        .rect
        .translate(Point::new(Dbu::new(2000), Dbu::new(4000)));
    shapes.push(moved);
    let net = db.add_net("n").unwrap();
    let pin = db.connect_cell_pin(ram, clk, net);

    // The shapes' centers are (100200, 120000) and (102200, 124000).
    assert_eq!(
        pin_position(&db, pin),
        Point::new(Dbu::new(101_200), Dbu::new(122_000))
    );
}

/// The fixture flops with clock trees inside: `DFF_X1` rising in 0.2 ns
/// and falling in 0.4, `DFF_SLOW` only rising in 0.2 and `DFF_SLOWER`
/// only falling in 0.6.
const INSIDE_CLOCK_TREES: &str = r#"library (inside) {
  time_unit : "1ns" ;
  cell (DFF_X1) { ff ("IQ", "IQN") { next_state : "D" ; clocked_on : "CK" ; }
    pin (D) { direction : input ; }
    pin (CK) { direction : input ; clock : true ;
      timing () { timing_type : max_clock_tree_path ;
        cell_rise (scalar) { values ("0.2") ; } cell_fall (scalar) { values ("0.4") ; } } }
    pin (Q) { direction : output ; function : "IQ" ; } }
  cell (DFF_SLOW) { ff ("IQ", "IQN") { next_state : "D" ; clocked_on : "CK" ; }
    pin (D) { direction : input ; }
    pin (CK) { direction : input ; clock : true ;
      timing () { timing_type : max_clock_tree_path ;
        cell_rise (scalar) { values ("0.2") ; } } }
    pin (Q) { direction : output ; function : "IQ" ; } }
  cell (DFF_SLOWER) { ff ("IQ", "IQN") { next_state : "D" ; clocked_on : "CK" ; }
    pin (D) { direction : input ; }
    pin (CK) { direction : input ; clock : true ;
      timing () { timing_type : max_clock_tree_path ;
        cell_fall (scalar) { values ("0.6") ; } } }
    pin (Q) { direction : output ; function : "IQ" ; } }
}"#;

#[test]
fn a_clock_tree_inside_a_cell_delays_by_its_mean_edge_or_its_one_edge() {
    let db = fixtures::design(
        "COMPONENTS 3 ;\n- both DFF_X1 ;\n- rise DFF_SLOW ;\n- fall DFF_SLOWER ;\nEND COMPONENTS\n\
         NETS 1 ;\n- clk ( both CK ) ( rise CK ) ( fall CK ) ;\nEND NETS\n",
    );
    let lib = pare_common::db::parser::liberty::parse_str(
        INSIDE_CLOCK_TREES,
        std::path::Path::new("inside.lib"),
    )
    .unwrap();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let clk = db.net_id("clk").unwrap();

    let mut delays: Vec<(String, f64)> = db.nets[clk]
        .pins
        .iter()
        .map(|&p| {
            (
                db.pin_name(p),
                liberty_insertion_delay(&link, &db, p).get() * 1e9,
            )
        })
        .collect();
    delays.sort_by(|a, b| a.0.cmp(&b.0));

    let expected = [("both/CK", 0.3), ("fall/CK", 0.6), ("rise/CK", 0.2)];
    assert!(
        delays
            .iter()
            .zip(expected)
            .all(|((name, ns), (want, at))| name == want && (ns - at).abs() < 1e-9),
        "{delays:?}"
    );
}

#[test]
fn a_cells_first_output_is_found_after_its_inputs() {
    let db = fixtures::design(
        "COMPONENTS 1 ;\n- gate AND2_X1 ;\nEND COMPONENTS\n\
         NETS 3 ;\n- a ( gate A ) ;\n- b ( gate B ) ;\n- z ( gate Z ) ;\nEND NETS\n",
    );
    let gate = db.cell_id("gate").unwrap();

    let output = first_output(&db, gate).map(|p| db.pin_name(p));

    assert_eq!(output.as_deref(), Some("gate/Z"));
}
