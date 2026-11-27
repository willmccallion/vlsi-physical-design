//! What every design rule check keeps, whatever the layout: reordering
//! its shapes, moving or mirroring it or cutting a rectangle in two changes
//! its markers only as it moves them; checking it in tiles finds the same
//! violations, and the same markers in one tile; metal beyond the rules'
//! reach leaves them alone.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{CutLayerId, LayerId, NetId};
use pare_common::db::library::LayerRef;
use pare_common::db::parser::lef;
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_verify::drc::{
    DrcReport, Layout, Marker, Origin, Owner, OwnerIndex, Rules, check, check_tiles_lazily,
};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};
use std::collections::BTreeSet;
use std::path::Path;

/// The manufacturing grid of the fixture library, in database units.
const GRID: i64 = 5;

/// The rules of the common crate's fixture library at 1000 units a micron:
/// metal 0.1 microns wide spaced 0.1 apart, a spacing table and a width
/// range above, cuts spaced 0.12 apart.
fn fixture_rules() -> Rules {
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("../common/tests/fixtures");
    let units = DbuPerMicron::new(1000).unwrap();
    let library = lef::parse_files(&[fixtures.join("tiny.lef")], units).unwrap();
    Rules::from_design(&NetlistDB::new("t".into(), library))
}

/// One rectangle of a generated layout.
#[derive(Clone, Copy, Debug)]
struct Piece {
    layer: LayerRef,
    owner: u32,
    rect: Rect<Dbu>,
    origin: Origin,
}

const fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

/// A rectangle of one of three nets on either of the two lower metal
/// layers or the cut between them, on the grid within 1.5 microns, its
/// sides near the rules' widths and spacings so that they meet, nearly
/// meet and run alongside each other.
fn piece() -> impl Strategy<Value = Piece> {
    let layer = prop_oneof![
        3 => Just(LayerRef::Routing(LayerId(0))),
        2 => Just(LayerRef::Routing(LayerId(1))),
        1 => Just(LayerRef::Cut(CutLayerId(0))),
    ];
    let side = prop::sample::select(vec![100, 105, 150, 200, 400, 1000]);
    let origin = prop_oneof![4 => Just(Origin::Routing), 1 => Just(Origin::Input)];
    (
        layer,
        0..3_u32,
        0..300_i64,
        0..300_i64,
        side.clone(),
        side,
        origin,
    )
        .prop_map(|(layer, owner, x, y, w, h, origin)| {
            let (x, y) = (x * GRID, y * GRID);
            Piece {
                layer,
                owner,
                rect: rect(x, y, x + w, y + h),
                origin,
            }
        })
}

fn pieces() -> impl Strategy<Value = Vec<Piece>> {
    prop::collection::vec(piece(), 1..10)
}

/// The layout of `pieces` under `rules`' layers, its three nets in order.
fn layout_of(pieces: &[Piece], rules: &Rules) -> Layout {
    let mut layout = Layout::new(rules.metal.len(), rules.cut.len());
    let owners: Vec<_> = (0..3)
        .map(|n| layout.owner(Owner::Net(NetId::new(n)), false))
        .collect();
    for p in pieces {
        layout.add(p.layer, owners[p.owner as usize], p.rect, p.origin);
    }
    layout
}

/// A marker's kind, rule, layer, box and owners, its owners in order.
type Canonical = (String, u16, LayerRef, [i64; 4], (OwnerIndex, OwnerIndex));

/// A violation found: its category, kind, rule, layer and owners.
type Found = (usize, String, u16, LayerRef, (OwnerIndex, OwnerIndex));

/// A marker with its owners in order, so that markers compare whichever
/// shape of a pair was met first.
fn canonical(m: &Marker) -> Canonical {
    let (a, b) = m.owners;
    let r = m.bbox;
    (
        format!("{:?}", m.violation),
        m.rule,
        m.layer,
        [r.min.x.get(), r.min.y.get(), r.max.x.get(), r.max.y.get()],
        (a.min(b), a.max(b)),
    )
}

/// Every category of `report`, each a sorted list of canonical markers.
fn markers(report: &DrcReport) -> Vec<Vec<Canonical>> {
    [
        &report.violations,
        &report.advisory,
        &report.pin_obstruction,
        &report.placement,
        &report.input,
    ]
    .iter()
    .map(|list| {
        let mut sorted: Vec<_> = list.iter().map(canonical).collect();
        sorted.sort();
        sorted
    })
    .collect()
}

/// `pieces` and every marker's box under `move_rect`.
fn moved(pieces: &[Piece], move_rect: impl Fn(Rect<Dbu>) -> Rect<Dbu>) -> Vec<Piece> {
    pieces
        .iter()
        .map(|p| Piece {
            rect: move_rect(p.rect),
            ..*p
        })
        .collect()
}

/// The canonical markers of `report` with their boxes under `move_rect`.
fn moved_markers(
    report: &DrcReport,
    move_rect: impl Fn(Rect<Dbu>) -> Rect<Dbu>,
) -> Vec<Vec<Canonical>> {
    let mut moved = report.clone();
    for list in [
        &mut moved.violations,
        &mut moved.advisory,
        &mut moved.pin_obstruction,
        &mut moved.placement,
        &mut moved.input,
    ] {
        for m in list.iter_mut() {
            m.bbox = move_rect(m.bbox);
        }
    }
    markers(&moved)
}

/// The report of checking the layout of `pieces` whole, and of checking
/// it in tiles of `core` a side seeing the rules' reach beyond their edges.
fn whole_and_tiled(pieces: &[Piece], rules: &Rules, core: Dbu) -> (DrcReport, DrcReport) {
    let whole = layout_of(pieces, rules);
    let bounds = pieces
        .iter()
        .map(|p| p.rect)
        .reduce(|a, b| a.union(&b))
        .unwrap();
    let window_of = |window: &Rect<Dbu>| {
        let mut part = Layout::with_owners_of(&whole);
        for layer in whole.metal_layer_ids() {
            for shape in whole.metal(layer).iter().filter(|s| s.rect.touches(window)) {
                part.add_shape(LayerRef::Routing(layer), shape);
            }
        }
        for layer in whole.cut_layer_ids() {
            for shape in whole.cut(layer).iter().filter(|s| s.rect.touches(window)) {
                part.add_shape(LayerRef::Cut(layer), shape);
            }
        }
        part
    };
    let tiled = check_tiles_lazily(
        bounds,
        whole.owners().clone(),
        rules,
        (core, rules.max_reach()),
        window_of,
    );
    (check(&whole, rules), tiled)
}

/// The violations `report` finds: each category's kinds, rules, layers
/// and pairs of owners, whatever markers show them.
fn violations(report: &DrcReport) -> BTreeSet<Found> {
    [
        &report.violations,
        &report.advisory,
        &report.pin_obstruction,
        &report.placement,
        &report.input,
    ]
    .iter()
    .enumerate()
    .flat_map(|(category, list)| {
        list.iter().map(move |m| {
            let (violation, rule, layer, _, owners) = canonical(m);
            (category, violation, rule, layer, owners)
        })
    })
    .collect()
}

fn config() -> Config {
    Config {
        cases: 256,
        rng_seed: RngSeed::Fixed(71),
        failure_persistence: None,
        ..Config::default()
    }
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn markers_do_not_depend_on_the_order_of_the_shapes(
        (pieces, shuffled) in pieces().prop_flat_map(|p| (Just(p.clone()), Just(p).prop_shuffle()))
    ) {
        let rules = fixture_rules();

        let report = check(&layout_of(&pieces, &rules), &rules);

        prop_assert_eq!(markers(&report), markers(&check(&layout_of(&shuffled, &rules), &rules)));
    }

    #[test]
    fn moving_a_layout_by_whole_grid_steps_moves_its_markers(
        pieces in pieces(),
        dx in -400..400_i64,
        dy in -400..400_i64,
    ) {
        let rules = fixture_rules();
        let shift = Point::new(Dbu::new(dx * GRID), Dbu::new(dy * GRID));
        let shifted = |r: Rect<Dbu>| r.translate(shift);

        let report = check(&layout_of(&pieces, &rules), &rules);

        let after = check(&layout_of(&moved(&pieces, shifted), &rules), &rules);
        prop_assert_eq!(moved_markers(&report, shifted), markers(&after));
    }

    #[test]
    fn mirroring_a_layout_mirrors_its_markers(pieces in pieces()) {
        let rules = fixture_rules();
        let mirror = |r: Rect<Dbu>| rect(-r.max.x.get(), r.min.y.get(), -r.min.x.get(), r.max.y.get());

        let report = check(&layout_of(&pieces, &rules), &rules);

        let after = check(&layout_of(&moved(&pieces, mirror), &rules), &rules);
        prop_assert_eq!(moved_markers(&report, mirror), markers(&after));
    }

    #[test]
    fn checking_in_tiles_finds_the_violations_checking_the_whole_finds(
        pieces in pieces(),
        core in prop::sample::select(vec![150_i64, 300, 500, 1000]),
    ) {
        let rules = fixture_rules();

        let (whole, tiled) = whole_and_tiled(&pieces, &rules, Dbu::new(core));

        prop_assert_eq!(violations(&whole), violations(&tiled));
    }

    #[test]
    fn checking_in_one_tile_finds_what_checking_the_whole_finds(pieces in pieces()) {
        let rules = fixture_rules();

        let (whole, tiled) = whole_and_tiled(&pieces, &rules, Dbu::new(10_000));

        prop_assert_eq!(markers(&whole), markers(&tiled));
    }

    #[test]
    fn cutting_a_metal_rectangle_in_two_changes_no_marker(
        pieces in pieces(),
        which in any::<prop::sample::Index>(),
        at in 1..1000_i64,
    ) {
        let rules = fixture_rules();
        let k = which.index(pieces.len());
        let p = pieces[k];
        let r = p.rect;
        let steps = (r.width().get() / GRID).max(1);
        prop_assume!(matches!(p.layer, LayerRef::Routing(_)) && steps >= 2);
        let x = r.min.x.get() + GRID * (1 + at % (steps - 1));
        let mut cut = pieces.clone();
        cut[k].rect = rect(r.min.x.get(), r.min.y.get(), x, r.max.y.get());
        cut.push(Piece { rect: rect(x, r.min.y.get(), r.max.x.get(), r.max.y.get()), ..p });

        let report = check(&layout_of(&pieces, &rules), &rules);

        prop_assert_eq!(markers(&report), markers(&check(&layout_of(&cut, &rules), &rules)));
    }

    #[test]
    fn metal_beyond_the_rules_reach_leaves_the_markers_alone(
        pieces in pieces(),
        far in piece(),
    ) {
        let rules = fixture_rules();
        let away = Point::new(Dbu::new(100_000), Dbu::ZERO);
        let far = Piece { rect: far.rect.translate(away), ..far };
        let mut with_far = pieces.clone();
        with_far.push(far);

        let before = markers(&check(&layout_of(&pieces, &rules), &rules));
        let after = markers(&check(&layout_of(&with_far, &rules), &rules));

        for (old, new) in before.iter().zip(&after) {
            let near: Vec<_> = new.iter().filter(|m| m.3[0] < 50_000).cloned().collect();
            prop_assert_eq!(old, &near);
        }
    }
}
