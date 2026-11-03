//! Parasitics and buffering trees along global routes, on small fixture
//! designs.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, NonDefaultLayer, NonDefaultRule, PinOwner};
use pare_common::db::global_route::{NetRoute, PinGridLocation, RoutePoint, RouteSegment};
use pare_common::db::indices::{LayerId, NetId, NonDefaultRuleId, PinId};
use pare_common::db::parser::{def, lef, liberty};
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::geom::point::Point;
use pare_resizer::bnet::{self, Kind, LoadModel, TreeRc};
use pare_resizer::parasitics::{NetParasitics, WireRc};
use pare_resizer::route_parasitics::RouteRc;
use pare_resizer::{Link, Netlist};
use std::path::{Path, PathBuf};

fn fixture(crate_name: &str, name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join(crate_name)
        .join("tests/fixtures")
        .join(name)
}

fn tiny_design() -> NetlistDB {
    let units = def::read_units(&fixture("common", "tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("common", "tiny.lef")], units).unwrap();
    def::parse(&fixture("common", "tiny.def"), library).unwrap()
}

const fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu(x), Dbu(y))
}

const fn at(x: i64, y: i64, layer: u8) -> RoutePoint {
    RoutePoint {
        at: pt(x, y),
        layer: LayerId(layer),
    }
}

const fn segment(from: RoutePoint, to: RoutePoint) -> RouteSegment {
    RouteSegment { from, to }
}

fn cell_pin(db: &NetlistDB, cell: &str, pin: &str) -> PinId {
    let cell = db.cell_id(cell).unwrap();
    db.cells[cell.index()]
        .pins
        .iter()
        .copied()
        .find(|&p| match db.pins[p.index()].owner {
            PinOwner::Cell { pin: m, .. } => db.cell_macro(cell).pin(m).name == pin,
            PinOwner::Io(_) => false,
        })
        .unwrap()
}

const fn pin_at(pin: PinId, at: Point<Dbu>, grid: Point<Dbu>, layer: u8) -> PinGridLocation {
    PinGridLocation {
        pin,
        at,
        grid,
        layer: LayerId(layer),
    }
}

fn sorted(mut values: Vec<f64>) -> Vec<f64> {
    values.sort_by(f64::total_cmp);
    values
}

fn close(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len()
        && a.iter()
            .zip(b)
            .all(|(x, y)| (x - y).abs() <= 1e-6 * x.abs().max(y.abs()))
}

/// Net `n1` of the tiny design routed from `u1/Y` up to M2, along M2 to
/// `u2/A`, with a piece on M1 the router does not use.
fn n1_route(db: &NetlistDB) -> NetRoute {
    NetRoute {
        segments: vec![
            segment(at(1000, 1000, 0), at(1000, 1000, 1)),
            segment(at(1000, 1000, 1), at(1000, 3000, 1)),
            segment(at(1000, 3000, 0), at(3000, 3000, 0)),
        ],
        pins: vec![
            pin_at(cell_pin(db, "u1", "Y"), pt(900, 1000), pt(1000, 1000), 0),
            pin_at(cell_pin(db, "u2", "A"), pt(1000, 3000), pt(1000, 3000), 1),
            pin_at(cell_pin(db, "u3", "A"), pt(3000, 3000), pt(3000, 3000), 0),
        ],
    }
}

#[test]
fn route_segments_become_resistors_with_half_their_capacitance_at_each_end() {
    let db = tiny_design();
    let net = db.net_id("n1").unwrap();
    let rc = RouteRc::new(
        &db.library,
        &[(LayerId(1), 2.0e6, 1.0e-10)],
        &[],
        LayerId(1),
    );

    let parasitics = NetParasitics::from_route(&db, net, n1_route(&db), &rc);

    // The via takes the LEF's 5 ohms; 2 um of M2 takes 4 ohms and 0.2 fF;
    // u1/Y joins through the via and 0.1 um of M2; u2/A sits on the route;
    // u3/A is on the unused M1 piece and stays apart.
    let resistors = sorted(parasitics.resistors().iter().map(|r| r.2).collect());
    assert!(close(&resistors, &[0.0, 4.0, 5.0, 5.2]), "{resistors:?}");
    let total: f64 = parasitics.node_caps().iter().sum();
    assert!((total - 2.1e-16).abs() < 1e-6 * 2.1e-16, "{total}");
    let u3 = cell_pin(&db, "u3", "A");
    let (_, u3_node) = parasitics
        .pin_nodes()
        .iter()
        .copied()
        .find(|&(p, _)| p == u3)
        .unwrap();
    assert!(
        parasitics
            .resistors()
            .iter()
            .all(|&(a, b, _)| a != u3_node && b != u3_node)
    );
}

#[test]
fn layers_without_set_values_take_their_lef_resistance_and_capacitance() {
    let db = tiny_design();
    let net = db.net_id("n1").unwrap();
    let rc = RouteRc::new(&db.library, &[], &[], LayerId(0));
    let route = NetRoute {
        segments: vec![segment(at(0, 1000, 0), at(1000, 1000, 0))],
        pins: Vec::new(),
    };

    let parasitics = NetParasitics::from_route(&db, net, route, &rc);

    // M1 is 0.1 um wide at 0.38 ohm per square, 7.7161e-5 pF per square
    // micron and 2.7365e-5 pF per micron of edge.
    let resistors: Vec<f64> = parasitics.resistors().iter().map(|r| r.2).collect();
    assert!(close(&resistors, &[3.8]), "{resistors:?}");
    let total: f64 = parasitics.node_caps().iter().sum();
    let expected = 2.0f64.mul_add(2.7365e-5, 0.1 * 7.7161e-5) * 1e-12;
    assert!((total - expected).abs() < 1e-6 * expected, "{total}");
}

#[test]
fn a_wider_non_default_rule_lowers_the_wire_resistance() {
    let mut db = tiny_design();
    let net = db.net_id("n1").unwrap();
    db.non_default_rules.push(NonDefaultRule {
        name: "wide".into(),
        hard_spacing: false,
        layers: vec![NonDefaultLayer {
            layer: LayerId(1),
            width: Dbu(200),
            spacing: None,
            wire_extension: None,
        }],
        vias: Vec::new(),
        via_rules: Vec::new(),
        min_cuts: Vec::new(),
    });
    db.nets[net.index()].non_default_rule = Some(NonDefaultRuleId::new(0));
    let rc = RouteRc::new(
        &db.library,
        &[(LayerId(1), 2.0e6, 1.0e-10)],
        &[],
        LayerId(1),
    );
    let route = NetRoute {
        segments: vec![segment(at(1000, 1000, 1), at(1000, 3000, 1))],
        pins: Vec::new(),
    };

    let parasitics = NetParasitics::from_route(&db, net, route, &rc);

    let resistors: Vec<f64> = parasitics.resistors().iter().map(|r| r.2).collect();
    assert!(close(&resistors, &[2.0]), "{resistors:?}");
}

struct UnitLoads;

impl LoadModel for UnitLoads {
    fn load(&self, _: PinId) -> (f64, f64, f64) {
        (1e-15, 1.0, 1e-9)
    }
}

/// An inverter driving two others, on the clock fixture's cells.
fn fanout_design() -> NetlistDB {
    let text = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN fanout ;\n\
        UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\n\
        COMPONENTS 3 ;\n- d INV_X1 + PLACED ( 2000 2800 ) N ;\n\
        - a INV_X1 + PLACED ( 20000 2800 ) N ;\n- b INV_X1 + PLACED ( 2000 22400 ) N ;\n\
        END COMPONENTS\nNETS 1 ;\n- n ( d ZN ) ( a A ) ( b A ) ;\nEND NETS\nEND DESIGN\n";
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("cts", "clock.lef")], units).unwrap();
    def::parse_str(text, Path::new("fanout.def"), library).unwrap()
}

#[test]
fn a_route_tree_climbs_by_vias_and_runs_wires_on_the_route_layers() {
    let db = fanout_design();
    let lib = liberty::parse_file(&fixture("cts", "clock.lib")).unwrap();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let net = NetId::new(0);
    let (d, a, b) = (
        cell_pin(&db, "d", "ZN"),
        cell_pin(&db, "a", "A"),
        cell_pin(&db, "b", "A"),
    );
    let route = NetRoute {
        segments: vec![
            segment(at(3000, 3000, 0), at(3000, 3000, 1)),
            segment(at(3000, 3000, 1), at(3000, 3000, 2)),
            segment(at(3000, 3000, 2), at(21000, 3000, 2)),
            segment(at(21000, 3000, 2), at(21000, 3000, 1)),
            segment(at(21000, 3000, 1), at(21000, 3000, 0)),
            segment(at(3000, 3000, 1), at(3000, 23000, 1)),
            segment(at(3000, 23000, 1), at(3000, 23000, 0)),
        ],
        pins: vec![
            pin_at(d, pt(2500, 3000), pt(3000, 3000), 0),
            pin_at(a, pt(20500, 3000), pt(21000, 3000), 0),
            pin_at(b, pt(2500, 23000), pt(3000, 23000), 0),
        ],
    };
    let route_rc = RouteRc::new(
        &db.library,
        &[(LayerId(1), 2.0e6, 1.0e-10), (LayerId(2), 1.0e6, 1.0e-10)],
        &[],
        LayerId(1),
    );
    let rc = TreeRc {
        signal: WireRc::per_micron(3.0, 0.1, LayerId(2)),
        layers: Some(&route_rc),
        dbu: 2000.0,
    };
    let parasitics = NetParasitics::from_route(&db, net, route, &route_rc);

    let tree = bnet::from_parasitics(&netlist, &parasitics, d, &rc, &UnitLoads).unwrap();

    let mut loads = tree.loads();
    loads.sort_unstable();
    assert_eq!(loads, vec![a, b]);
    assert!(matches!(tree.kind, Kind::Via { .. }));
    assert_eq!(tree.layer(), Some(LayerId(0)));
    assert_eq!(tree.max_load_wire_length(), 20000);
    let mut wire_layers = Vec::new();
    collect_wire_layers(&tree, &mut wire_layers);
    wire_layers.sort_unstable();
    assert_eq!(wire_layers, vec![Some(LayerId(1)), Some(LayerId(2))]);
}

fn collect_wire_layers(node: &bnet::Bnet, into: &mut Vec<Option<LayerId>>) {
    match &node.kind {
        Kind::Wire { to, layer } => {
            into.push(*layer);
            collect_wire_layers(to, into);
        }
        Kind::Via { to, .. } | Kind::Buffer { to, .. } => collect_wire_layers(to, into),
        Kind::Junction { left, right } => {
            collect_wire_layers(left, into);
            collect_wire_layers(right, into);
        }
        Kind::Load { .. } => {}
    }
}
