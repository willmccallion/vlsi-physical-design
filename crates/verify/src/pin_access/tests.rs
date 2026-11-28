use super::*;
use pare_common::db::design::{PlacementStatus, Row, SpecialNet, SpecialWire, TrackAxis, TrackDef};
use pare_common::db::indices::{CutLayerId, IdVec, MacroId, MacroPinId, SiteId};
use pare_common::db::library::{
    CoreKind, CutLayer, Library, Macro, MacroClass, MacroPin, PinDirection, RoutingLayer, Shape,
    SignalUse, Site, SiteClass, Symmetry, ViaDef, ViaSource,
};
use pare_common::db::rules::{CutLayerRules, ParallelRunLengthTable, RoutingLayerRules};
use pare_common::geom::dbu::DbuPerMicron;

const fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(pt(x0, y0), pt(x1, y1))
}

fn layer(name: &str, direction: Direction) -> RoutingLayer {
    RoutingLayer {
        name: name.into(),
        direction,
        pitch_x: Dbu::new(200),
        pitch_y: Dbu::new(200),
        offset_x: None,
        offset_y: None,
        width: Dbu::new(100),
        min_spacing: Dbu::new(100),
        spacing_table: None,
        rules: RoutingLayerRules::default(),
        electrical: pare_common::db::library::LayerElectrical::default(),
    }
}

/// Horizontal M1 under vertical M2 with 0.1 cut vias whose M2 pad is
/// 0.1 by 0.18 and whose M1 pad is 0.18 by 0.1 or 0.1 by 0.18; one 0.4
/// by 2.0 cell master with a 0.1-wide M1 pin centred 0.1 in from its
/// left edge; a 0.58-wide M2 ground stripe at x = 3.9 over a 10 by 4
/// die.
fn design() -> (NetlistDB, pare_common::db::indices::MacroId) {
    design_with(3900, None, None)
}

/// [`design`] with the stripe at `stripe_x`, M2 under `table`, and an
/// M1 ground rail 0.1 wide along y = `rail_y` when given.
fn design_with(
    stripe_x: i64,
    table: Option<SpacingTable>,
    rail_y: Option<i64>,
) -> (NetlistDB, pare_common::db::indices::MacroId) {
    design_full(stripe_x, table, rail_y, None)
}

fn cut_layer(name: &str) -> CutLayer {
    CutLayer {
        name: name.into(),
        width: Some(Dbu::new(100)),
        spacing: Some(Dbu::new(120)),
        rules: CutLayerRules::default(),
        resistance: None,
    }
}

/// A default via through `cut` with a 0.1 cut and the given pads below
/// and above.
fn via(name: &str, cut: CutLayerId, below: Shape, above: Shape) -> ViaDef {
    ViaDef {
        name: name.into(),
        cut,
        shapes: vec![
            below,
            Shape {
                layer: LayerRef::Cut(cut),
                rect: rect(-50, -50, 50, 50),
            },
            above,
        ],
        source: ViaSource::LefDefault,
    }
}

const fn metal(layer: LayerId, rect: Rect<Dbu>) -> Shape {
    Shape {
        layer: LayerRef::Routing(layer),
        rect,
    }
}

/// A 0.4 by 2.0 core master on `site` with one pin of `shape`.
fn one_pin_master(name: &str, site: SiteId, pin: MacroPin) -> Macro {
    let symmetry = Symmetry {
        x: true,
        y: true,
        r90: false,
    };
    let mut m = Macro::new(
        name.into(),
        MacroClass::Core(CoreKind::Regular),
        pt(400, 2000),
        symmetry,
        Some(site),
    );
    let _ = m.add_pin(pin);
    m
}

fn pin(name: &str, direction: PinDirection, shape: Shape) -> MacroPin {
    MacroPin {
        name: name.into(),
        direction,
        signal_use: SignalUse::Signal,
        shapes: vec![shape],
    }
}

/// A ground wire of `width` on `layer` from `from` to `to`.
fn ground_wire(
    layer: LayerId,
    width: i64,
    from: Point<Dbu>,
    to: Point<Dbu>,
    shape: &str,
) -> SpecialWire {
    SpecialWire {
        layer,
        width: Dbu::new(width),
        from,
        to,
        extensions: (Dbu::ZERO, Dbu::ZERO),
        shape: Some(shape.into()),
    }
}

/// [`design_with`] plus a horizontal M3 over M2, joined by a 0.1 cut via
/// with 0.1 by 0.18 pads, and a 6.0-wide M3 ground stripe along
/// y = `m3_stripe_y` when given.
fn design_full(
    stripe_x: i64,
    table: Option<SpacingTable>,
    rail_y: Option<i64>,
    m3_stripe_y: Option<i64>,
) -> (NetlistDB, pare_common::db::indices::MacroId) {
    let (mut lib, [m1, m2, m3], site) = three_layer_library(table);
    let m = add_masters(&mut lib, site, m1, m2);
    let mut db = NetlistDB::new("t".into(), lib);
    db.die_area = rect(0, 0, 10_000, 4000);
    db.tracks.push(TrackDef {
        axis: TrackAxis::X,
        start: Dbu::new(100),
        count: 50,
        step: Dbu::new(200),
        layers: vec![m2],
    });
    db.tracks.push(TrackDef {
        axis: TrackAxis::Y,
        start: Dbu::new(0),
        count: 21,
        step: Dbu::new(200),
        layers: vec![m1, m3],
    });
    let mut wires = vec![ground_wire(
        m2,
        580,
        pt(stripe_x, 0),
        pt(stripe_x, 4000),
        "STRIPE",
    )];
    wires.extend(rail_y.map(|y| ground_wire(m1, 100, pt(0, y), pt(10_000, y), "FOLLOWPIN")));
    wires.extend(m3_stripe_y.map(|y| ground_wire(m3, 6000, pt(0, y), pt(10_000, y), "STRIPE")));
    let _ = db.special_nets.push(SpecialNet {
        name: "vss".into(),
        signal_use: SignalUse::Ground,
        connections: Vec::new(),
        wires,
        rects: Vec::new(),
        vias: Vec::new(),
    });
    (db, m)
}

/// M1, M2 with `table`, and M3, the vias between them, and a core site
/// 0.2 by 2.0.
fn three_layer_library(table: Option<SpacingTable>) -> (Library, [LayerId; 3], SiteId) {
    let mut lib = Library::new(DbuPerMicron::new(1000).unwrap());
    let m1 = lib
        .add_routing_layer(layer("M1", Direction::Horizontal))
        .unwrap();
    let v1 = lib.add_cut_layer(cut_layer("V1")).unwrap();
    let m2 = lib
        .add_routing_layer(RoutingLayer {
            spacing_table: table,
            ..layer("M2", Direction::Vertical)
        })
        .unwrap();
    let v2 = lib.add_cut_layer(cut_layer("V2")).unwrap();
    let m3 = lib
        .add_routing_layer(layer("M3", Direction::Horizontal))
        .unwrap();
    let tall = rect(-50, -90, 50, 90);
    let wide = rect(-90, -50, 90, 50);
    for v in [
        via("V23", v2, metal(m2, tall), metal(m3, wide)),
        via("V12", v1, metal(m1, wide), metal(m2, tall)),
        ViaDef {
            source: ViaSource::Lef,
            ..via("V12V", v1, metal(m1, tall), metal(m2, tall))
        },
    ] {
        let _ = lib.add_via(v).unwrap();
    }
    let site = lib
        .add_site(Site {
            name: "s".into(),
            class: SiteClass::Core,
            size: pt(200, 2000),
            symmetry: Symmetry {
                x: true,
                y: true,
                r90: false,
            },
        })
        .unwrap();
    (lib, [m1, m2, m3], site)
}

/// Adds the masters `C`, an M1 input it returns, `U`, an M2 input, `R`,
/// an M1 input off centre, and `E`, an M2 output, on `site`.
fn add_masters(
    lib: &mut Library,
    site: SiteId,
    m1: LayerId,
    m2: LayerId,
) -> pare_common::db::indices::MacroId {
    let input = |layer, rect| pin("a", PinDirection::Input, metal(layer, rect));
    let m = lib
        .add_macro(one_pin_master(
            "C",
            site,
            input(m1, rect(50, 500, 150, 1500)),
        ))
        .unwrap();
    for master in [
        one_pin_master("U", site, input(m2, rect(50, 500, 150, 1500))),
        one_pin_master("R", site, input(m1, rect(250, 500, 350, 1500))),
        one_pin_master(
            "E",
            site,
            pin(
                "o",
                PinDirection::Output,
                metal(m2, rect(0, 500, 150, 1500)),
            ),
        ),
    ] {
        let _ = lib.add_macro(master).unwrap();
    }
    m
}

/// Spacing 0.10 up to width 0.46 and 0.15 from there, at any run.
fn wide_metal_table() -> SpacingTable {
    SpacingTable::ParallelRunLength(ParallelRunLengthTable {
        run_lengths: vec![Dbu::new(0)],
        rows: vec![
            (Dbu::new(0), vec![Dbu::new(100)]),
            (Dbu::new(460), vec![Dbu::new(150)]),
        ],
    })
}

fn blocked_in(db: &mut NetlistDB, m: pare_common::db::indices::MacroId, x: i64) -> usize {
    let cell = cell_at(db, m, x);
    let access = PinAccess::new(db, LayerId(1)).unwrap();
    access.blocked_pins(db, cell, pt(x, 0), Orientation::N)
}

fn of_net(net: usize, r: Rect<Dbu>) -> Fixed {
    Fixed {
        rect: r,
        holder: Holder::of_net(Some(NetId::new(net))),
        obstruction: false,
    }
}

#[test]
fn the_gap_between_side_by_side_rectangles_spans_their_shared_stretch() {
    let gap = gap_between(&rect(100, 600, 240, 880), &rect(0, 400, 240, 540));

    assert_eq!(gap, rect(100, 540, 240, 600));
}

#[test]
fn the_gap_between_diagonal_rectangles_is_the_box_between_their_corners() {
    let gap = gap_between(&rect(0, 0, 100, 100), &rect(150, 130, 300, 300));

    assert_eq!(gap, rect(100, 100, 150, 130));
}

#[test]
fn a_pad_inside_its_pin_has_no_gap_to_the_pins_other_bar() {
    // XOR2_X1's pin A: a vertical bar and a horizontal bar meeting it,
    // with a via pad on the vertical bar 0.03 um above the horizontal.
    let net = Holder::of_net(Some(NetId::new(7)));
    let vertical = of_net(7, rect(100, 400, 240, 950));
    let horizontal = of_net(7, rect(-950, 400, 240, 540));
    let pad = rect(100, 600, 240, 880);

    assert!(gap_filled(
        &pad,
        &horizontal.rect,
        net,
        &[vertical, horizontal.clone()]
    ));
}

#[test]
fn a_gap_to_the_same_net_that_nothing_fills_stays_a_gap() {
    let net = Holder::of_net(Some(NetId::new(7)));
    let other = of_net(7, rect(0, 400, 240, 540));
    let pad = rect(100, 600, 240, 880);

    assert!(!gap_filled(
        &pad,
        &other.rect,
        net,
        std::slice::from_ref(&other)
    ));
}

#[test]
fn another_nets_metal_does_not_fill_a_gap() {
    let net = Holder::of_net(Some(NetId::new(7)));
    let mine = of_net(7, rect(0, 400, 240, 540));
    let theirs = of_net(8, rect(100, 400, 240, 950));
    let pad = rect(100, 600, 240, 880);

    assert!(!gap_filled(&pad, &mine.rect, net, &[mine.clone(), theirs]));
}

#[test]
fn a_wide_stripe_demands_the_tables_spacing_of_the_via_pad() {
    let (mut plain, m) = design_with(3950, None, None);
    let (mut tabled, _) = design_with(3950, Some(wide_metal_table()), None);

    assert_eq!(blocked_in(&mut plain, m, 3400), 0, "0.11 um clears 0.10");
    assert_eq!(blocked_in(&mut tabled, m, 3400), 1, "0.11 um breaks 0.15");
}

#[test]
fn a_pin_within_spacing_of_fixed_metal_on_its_own_layer_is_blocked() {
    let (mut db, m) = design_with(3900, None, Some(1600));

    assert_eq!(blocked_in(&mut db, m, 1000), 1);
}

#[test]
fn a_pin_clear_of_fixed_metal_on_its_own_layer_is_free() {
    let (mut db, m) = design_with(3900, None, Some(1700));

    assert_eq!(blocked_in(&mut db, m, 1000), 0);
}

#[test]
fn a_via_whose_lower_metal_sticks_out_of_the_pin_still_reaches_it() {
    let (mut db, m) = design();
    db.library.vias = std::mem::take(&mut db.library.vias)
        .into_iter()
        .filter(|v| v.name == "V12" || v.name == "V23")
        .collect();

    assert_eq!(blocked_in(&mut db, m, 1000), 0);
}

/// Master P, whose pin `a` is master C's and whose pin `b`, 0.1 wide,
/// stands 0.1 to its right, at (1.0, 0) with only the via `keep` up from
/// M1; how many of its pins are blocked.
fn blocked_pair(keep: &str) -> usize {
    let (mut db, _) = design();
    let symmetry = Symmetry {
        x: true,
        y: true,
        r90: false,
    };
    let mut pair = Macro::new(
        "P".into(),
        MacroClass::Core(CoreKind::Regular),
        pt(400, 2000),
        symmetry,
        Some(pare_common::db::indices::SiteId(0)),
    );
    for (name, x0) in [("a", 50), ("b", 250)] {
        let _ = pair.add_pin(MacroPin {
            name: name.into(),
            direction: PinDirection::Input,
            signal_use: SignalUse::Signal,
            shapes: vec![Shape {
                layer: LayerRef::Routing(LayerId(0)),
                rect: rect(x0, 500, x0 + 100, 1500),
            }],
        });
    }
    let pair = db.library.add_macro(pair).unwrap();
    db.library.vias = std::mem::take(&mut db.library.vias)
        .into_iter()
        .filter(|v| v.name == keep || v.name == "V23")
        .collect();
    let cell = db
        .add_cell(
            "p".into(),
            pair,
            pt(1000, 0),
            Orientation::N,
            PlacementStatus::Placed,
        )
        .unwrap();
    for (k, name) in [(0, "na"), (1, "nb")] {
        let net = db.net_or_insert(name);
        let _ = db.connect_cell_pin(cell, MacroPinId(k), net);
    }
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    access.blocked_pins(&db, cell, pt(1000, 0), Orientation::N)
}

#[test]
fn a_via_pad_too_near_another_pin_of_its_cell_does_not_reach_the_pin() {
    assert_eq!(blocked_pair("V12"), 2);
}

#[test]
fn a_via_pad_that_keeps_clear_of_the_cells_other_pins_reaches_the_pin() {
    assert_eq!(blocked_pair("V12V"), 0);
}

fn cell_at(db: &mut NetlistDB, m: pare_common::db::indices::MacroId, x: i64) -> CellId {
    let cell = db
        .add_cell(
            format!("c{x}"),
            m,
            pt(x, 0),
            Orientation::N,
            PlacementStatus::Placed,
        )
        .unwrap();
    let net = db.net_or_insert(&format!("n{x}"));
    let _ = db.connect_cell_pin(cell, MacroPinId(0), net);
    cell
}

fn blocked(x: i64) -> usize {
    let (mut db, m) = design();
    let cell = cell_at(&mut db, m, x);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    access.blocked_pins(&db, cell, pt(x, 0), Orientation::N)
}

#[test]
fn a_pin_under_the_stripe_is_blocked() {
    assert_eq!(blocked(3800), 1);
}

#[test]
fn a_pin_whose_via_pad_would_come_within_spacing_of_the_stripe_is_blocked() {
    assert_eq!(blocked(3400), 1);
}

#[test]
fn a_pin_whose_via_pad_keeps_the_spacing_is_free() {
    assert_eq!(blocked(3200), 0);
}

#[test]
fn a_pin_far_from_the_stripe_is_free() {
    assert_eq!(blocked(1000), 0);
}

fn blocked_upper(x: i64) -> usize {
    let (mut db, _) = design();
    let cell = cell_at(&mut db, pare_common::db::indices::MacroId(1), x);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    access.blocked_pins(&db, cell, pt(x, 0), Orientation::N)
}

#[test]
fn a_routing_layer_pin_within_spacing_of_the_stripe_is_blocked() {
    assert_eq!(blocked_upper(3400), 1);
}

#[test]
fn a_routing_layer_pin_clear_of_the_stripe_is_free() {
    assert_eq!(blocked_upper(3200), 0);
}

/// Master R at 1.0 with master E placed at `x` to its right, on the same
/// net when `same_net`; how many of R's pins are blocked.
fn blocked_beside(x: i64, same_net: bool) -> usize {
    let (mut db, _) = design();
    let right = cell_at(&mut db, MacroId(2), 1000);
    let edge = db
        .add_cell(
            "e".into(),
            MacroId(3),
            pt(x, 0),
            Orientation::N,
            PlacementStatus::Placed,
        )
        .unwrap();
    let net = db.net_or_insert(if same_net { "n1000" } else { "ne" });
    let _ = db.connect_cell_pin(edge, MacroPinId(0), net);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    let beside = [Neighbour {
        cell: edge,
        at: pt(x, 0),
        orient: Orientation::N,
    }];
    access.blocked_pins_among(&db, right, pt(1000, 0), Orientation::N, &beside)
}

#[test]
fn a_pin_beside_an_abutting_neighbours_routing_layer_pin_is_blocked() {
    assert_eq!(blocked_beside(1400, false), 1);
}

#[test]
fn a_neighbours_routing_layer_pin_one_site_away_does_not_block() {
    assert_eq!(blocked_beside(1600, false), 0);
}

#[test]
fn a_pad_ending_short_of_a_neighbours_pin_on_its_net_blocks() {
    assert_eq!(blocked_beside(1400, true), 1);
}

#[test]
fn the_fixed_metal_alone_leaves_the_pin_free() {
    let (mut db, _) = design();
    let right = cell_at(&mut db, MacroId(2), 1000);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    assert_eq!(
        access.blocked_pins(&db, right, pt(1000, 0), Orientation::N),
        0
    );
    assert_eq!(access.reach(), Dbu::new(190));
}

/// Master U placed at `x` with its M2 pin on no net.
fn blocked_unconnected(x: i64) -> usize {
    let (mut db, _) = design();
    let cell = db
        .add_cell(
            "u".into(),
            MacroId(1),
            pt(x, 0),
            Orientation::N,
            PlacementStatus::Placed,
        )
        .unwrap();
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    access.blocked_pins(&db, cell, pt(x, 0), Orientation::N)
}

#[test]
fn an_unconnected_pin_under_the_stripe_is_blocked() {
    assert_eq!(blocked_unconnected(3800), 1);
}

#[test]
fn an_unconnected_pin_clear_of_the_stripe_is_free() {
    assert_eq!(blocked_unconnected(1000), 0);
}

/// Master `m` at (1.0, 0) with an M3 stripe band centered on `stripe_y`.
fn blocked_under_m3_stripe(m: MacroId, stripe_y: i64) -> usize {
    let (mut db, _) = design_full(3900, None, None, Some(stripe_y));
    let cell = cell_at(&mut db, m, 1000);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    access.blocked_pins(&db, cell, pt(1000, 0), Orientation::N)
}

#[test]
fn a_routing_layer_pin_with_no_via_up_within_a_row_is_blocked() {
    // The band over y -1.0..5.0 leaves no M3 landing within 2.0 of the
    // pin's 0.5..1.5 run.
    assert_eq!(blocked_under_m3_stripe(MacroId(1), 2000), 1);
}

#[test]
fn a_routing_layer_pin_escapes_past_a_band_it_can_run_out_of() {
    // The band over y 1.0..7.0 leaves landings at y 0.0..0.8.
    assert_eq!(blocked_under_m3_stripe(MacroId(1), 4000), 0);
}

#[test]
fn a_pin_layer_pin_needs_its_via_pad_to_escape_too() {
    assert_eq!(blocked_under_m3_stripe(MacroId(0), 2000), 1);
    assert_eq!(blocked_under_m3_stripe(MacroId(0), 4000), 0);
}

#[test]
fn a_design_without_a_via_up_from_the_pins_has_no_check() {
    let (mut db, _) = design();
    db.library.vias = IdVec::new();

    assert!(PinAccess::new(&db, LayerId(1)).is_none());
}

fn stretch(lo: i64, hi: i64, block: PadBlock) -> PadStretch {
    PadStretch {
        lo: Dbu::new(lo),
        hi: Dbu::new(hi),
        block,
    }
}

#[test]
fn overlapping_pad_stretches_of_two_nets_block_every_pad() {
    let (a, b) = (PadBlock::Net(NetId::new(1)), PadBlock::Net(NetId::new(2)));

    let merged = PadStretches::merged(&[stretch(0, 10, a), stretch(5, 15, a), stretch(12, 20, b)]);

    assert_eq!(
        merged.0,
        [
            stretch(0, 11, a),
            stretch(12, 15, PadBlock::All),
            stretch(16, 20, b)
        ]
    );
    assert_eq!(merged.at(Dbu::new(21)), PadBlock::Clear);
}

/// What fixed metal on `metal` blocks `pad`, shape by shape.
fn pad_block_directly(metal: &LayerMetal, pad: &Rect<Dbu>) -> PadBlock {
    metal
        .fixed
        .iter()
        .filter(|f| too_close(pad, &f.rect, metal.required(pad, f)))
        .fold(PadBlock::Clear, |b, f| b.and(PadBlock::of(f.holder)))
}

/// Checks that the stretches the check works out for the pads of the vias
/// up from M2 agree, at every `step`th origin along each M3 track, with
/// measuring the pad against all of M3's fixed metal.
fn assert_pad_blocks_match_direct_checks(db: &NetlistDB, step: usize) {
    let access = PinAccess::new(db, LayerId(1)).unwrap();
    let up = access.up.as_ref().unwrap();
    let metal = LayerMetal::new(db, LayerId(2));
    let mut seen = std::collections::HashSet::new();

    for (t, &track) in up.tracks.iter().enumerate() {
        for (k, via) in up.vias.iter().enumerate() {
            for x in (-1500..11_500).step_by(step) {
                let at = pt(x, track.get());
                let direct = pad_block_directly(&metal, &via.pad.translate(at));
                assert_eq!(
                    up.pad_block(t, k, Dbu::new(x)),
                    direct,
                    "track {t} via {k} x {x}"
                );
                let _ = seen.insert(direct == PadBlock::Clear);
            }
        }
    }
    assert_eq!(seen.len(), 2, "both blocked and clear pads are compared");
}

#[test]
fn pad_stretches_agree_with_checking_every_via_position() {
    let (db, _) = design_full(3900, Some(wide_metal_table()), None, Some(2000));

    assert_pad_blocks_match_direct_checks(&db, 7);
}

#[test]
fn rectangles_overlapping_in_one_axis_run_side_by_side_by_that_overlap() {
    let run = parallel_run(&rect(0, 0, 100, 100), &rect(50, 200, 150, 300));

    assert_eq!(run, Dbu::new(50));
}

#[test]
fn rectangles_apart_in_both_axes_have_a_negative_run() {
    let run = parallel_run(&rect(0, 0, 100, 100), &rect(200, 250, 300, 300));

    assert_eq!(run, Dbu::new(-100));
}

#[test]
fn touching_rectangles_are_too_close_at_any_spacing() {
    assert!(too_close(
        &rect(0, 0, 100, 100),
        &rect(100, 0, 200, 100),
        Dbu::ZERO
    ));
}

#[test]
fn side_by_side_rectangles_are_too_close_only_inside_the_spacing() {
    let (a, b) = (rect(0, 0, 100, 100), rect(150, 20, 250, 80));

    assert!(too_close(&a, &b, Dbu::new(51)));
    assert!(!too_close(&a, &b, Dbu::new(50)));
}

#[test]
fn diagonal_rectangles_are_too_close_inside_the_corner_distance() {
    let (a, b) = (rect(0, 0, 100, 100), rect(160, 180, 300, 300));

    assert!(
        too_close(&a, &b, Dbu::new(101)),
        "60 by 80 is 100 corner to corner"
    );
    assert!(!too_close(&a, &b, Dbu::new(100)));
}

fn metal_with_ranges(ranges: Vec<WidthRangeSpacing>) -> LayerMetal {
    LayerMetal {
        fixed: RTree::new(),
        width: Dbu::new(100),
        min_spacing: Dbu::new(100),
        table: None,
        ranges,
        reach: Dbu::new(150),
        default_width_obstructions: false,
    }
}

fn range_rule(min_width: i64, max_width: i64, spacing: i64) -> WidthRangeSpacing {
    WidthRangeSpacing {
        spacing: Dbu::new(spacing),
        min_width: Dbu::new(min_width),
        max_width: Dbu::new(max_width),
    }
}

/// The spacing between two parallel bars `w1` and `w2` wide.
fn required_between(metal: &LayerMetal, w1: i64, w2: i64) -> Dbu {
    let shape = rect(0, 0, w1, 1000);
    let other = of_net(1, rect(2000, 0, 2000 + w2, 1000));
    metal.required(&shape, &other)
}

#[test]
fn a_width_range_rule_raises_the_spacing_of_a_shape_in_the_range() {
    let metal = metal_with_ranges(vec![range_rule(200, 300, 150)]);

    assert_eq!(required_between(&metal, 250, 100), Dbu::new(150));
    assert_eq!(required_between(&metal, 100, 250), Dbu::new(150));
    assert_eq!(required_between(&metal, 200, 300), Dbu::new(150));
}

#[test]
fn a_width_range_rule_leaves_shapes_outside_the_range_at_the_minimum() {
    let metal = metal_with_ranges(vec![range_rule(200, 300, 150)]);

    assert_eq!(required_between(&metal, 100, 100), Dbu::new(100));
    assert_eq!(required_between(&metal, 50, 50), Dbu::new(100));
    assert_eq!(required_between(&metal, 400, 400), Dbu::new(100));
}

#[test]
fn a_width_range_rule_never_lowers_the_spacing() {
    let metal = metal_with_ranges(vec![range_rule(200, 300, 80), range_rule(200, 300, 100)]);

    assert_eq!(required_between(&metal, 250, 250), Dbu::new(100));
}

#[test]
fn an_added_shape_may_sit_apart_from_metal_of_its_net_that_other_metal_of_the_net_bridges() {
    let metal = metal_with_ranges(Vec::new());
    let net = Holder::of_net(Some(NetId::new(7)));
    let near = [
        of_net(7, rect(0, 400, 240, 540)),
        of_net(7, rect(100, 400, 240, 950)),
    ];

    assert!(metal.clear_added(
        &rect(100, 600, 240, 880),
        net,
        &NeighbourMetal::default(),
        &near
    ));
}

#[test]
fn an_added_shape_may_not_sit_apart_from_metal_of_its_net_across_an_unfilled_gap() {
    let metal = metal_with_ranges(Vec::new());
    let net = Holder::of_net(Some(NetId::new(7)));
    let near = [of_net(7, rect(0, 400, 240, 540))];

    assert!(!metal.clear_added(
        &rect(100, 600, 240, 880),
        net,
        &NeighbourMetal::default(),
        &near
    ));
}

#[test]
fn the_vias_up_from_a_layer_leave_out_those_a_def_defines() {
    let (mut db, _) = design();
    let def_via = ViaDef {
        source: ViaSource::Def,
        name: "FROM_DEF".into(),
        ..via(
            "",
            CutLayerId(0),
            metal(LayerId(0), rect(-50, -50, 50, 50)),
            metal(LayerId(1), rect(-50, -50, 50, 50)),
        )
    };
    let _ = db.library.add_via(def_via).unwrap();

    let vias = ViaUp::all_between(&db.library, LayerId(0), LayerId(1));

    assert_eq!(vias.len(), 2);
}

#[test]
fn a_stretch_covers_its_last_origin() {
    let stretches = PadStretches(vec![stretch(0, 10, PadBlock::All)]);

    assert_eq!(stretches.at(Dbu::new(10)), PadBlock::All);
    assert_eq!(stretches.at(Dbu::new(11)), PadBlock::Clear);
}

/// `design` with M2's pitches set to `pitch_x` and `pitch_y`.
fn design_with_pitches(pitch_x: i64, pitch_y: i64) -> NetlistDB {
    let (mut db, _) = design();
    let m2 = &mut db.library.routing_layers[LayerId(1)];
    m2.pitch_x = Dbu::new(pitch_x);
    m2.pitch_y = Dbu::new(pitch_y);
    db
}

fn add_row(db: &mut NetlistDB) {
    db.rows.push(Row {
        name: "r".into(),
        site: SiteId(0),
        origin: pt(0, 0),
        orient: Orientation::N,
        num_x: 10,
        num_y: 1,
        step: pt(200, 0),
    });
}

#[test]
fn a_design_with_rows_lets_metal_escape_a_row_height() {
    let mut db = design_with_pitches(250, 300);
    add_row(&mut db);

    assert_eq!(escape_reach(&db, LayerId(1)), Dbu::new(2000));
}

#[test]
fn without_rows_metal_escapes_ten_pitches_across_a_vertical_layer() {
    let db = design_with_pitches(250, 300);

    assert_eq!(escape_reach(&db, LayerId(1)), Dbu::new(2500));
}

#[test]
fn without_rows_metal_escapes_ten_pitches_across_a_horizontal_layer() {
    let db = design_with_pitches(250, 300);

    assert_eq!(escape_reach(&db, LayerId(0)), Dbu::new(2000));
    assert_eq!(escape_reach(&db, LayerId(2)), Dbu::new(2000));
}

#[test]
fn a_horizontal_layers_escape_follows_its_own_pitch() {
    let mut db = design_with_pitches(250, 300);
    db.library.routing_layers[LayerId(2)].pitch_y = Dbu::new(330);

    assert_eq!(escape_reach(&db, LayerId(2)), Dbu::new(3300));
}

#[test]
fn rows_of_zero_height_sites_give_no_escape_reach_of_their_own() {
    let mut db = design_with_pitches(250, 300);
    add_row(&mut db);
    db.library.sites[SiteId(0)].size = pt(200, 0);

    assert_eq!(escape_reach(&db, LayerId(1)), Dbu::new(2500));
}

#[test]
fn the_check_reports_the_reach_of_its_escape() {
    let db = design_with_pitches(250, 300);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    assert_eq!(access.escape_reach(), Dbu::new(2500));
}

fn relied_on(
    access: &PinAccess,
    db: &NetlistDB,
    cell: CellId,
    at: Point<Dbu>,
) -> Option<Vec<(LayerId, Rect<Dbu>)>> {
    access.reliance(db, cell, at, Orientation::N, &[])
}

#[test]
fn a_free_pins_cell_relies_on_the_pins_own_metal() {
    let (mut db, m) = design();
    let cell = cell_at(&mut db, m, 1000);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    let relied = relied_on(&access, &db, cell, pt(1000, 0)).unwrap();

    assert!(relied.contains(&(LayerId(0), rect(1050, 500, 1150, 1500))));
}

#[test]
fn a_free_pins_cell_relies_on_the_pad_and_run_its_via_escapes_by() {
    let (mut db, m) = design();
    let cell = cell_at(&mut db, m, 1000);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    let relied = relied_on(&access, &db, cell, pt(1000, 0)).unwrap();

    let above: Vec<Rect<Dbu>> = relied
        .iter()
        .filter(|(layer, _)| *layer == LayerId(1))
        .map(|&(_, r)| r)
        .collect();
    assert_eq!(above.len(), 2, "a via pad and the run it escapes by");
    assert!(above[0].width() > Dbu::ZERO && above[1].width() > Dbu::ZERO);
}

#[test]
fn a_cell_with_a_blocked_pin_relies_on_nothing() {
    let (mut db, m) = design();
    let cell = cell_at(&mut db, m, 3800);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    assert_eq!(relied_on(&access, &db, cell, pt(3800, 0)), None);
}

#[test]
fn a_cell_with_only_a_supply_pin_relies_on_nothing_and_is_not_blocked() {
    let (mut db, m) = design();
    let cell = cell_at(&mut db, m, 3800);
    let supply = db.cells[cell].pins[0];
    let net = db.pins[supply].net;
    db.nets[net].signal_use = SignalUse::Power;
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    assert_eq!(relied_on(&access, &db, cell, pt(3800, 0)), Some(Vec::new()));
}

#[test]
fn a_pin_on_a_supply_net_is_not_counted_as_blocked() {
    let (mut db, m) = design();
    let cell = cell_at(&mut db, m, 3800);
    let supply = db.cells[cell].pins[0];
    let net = db.pins[supply].net;
    db.nets[net].signal_use = SignalUse::Ground;
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    assert_eq!(
        access.blocked_pins(&db, cell, pt(3800, 0), Orientation::N),
        0
    );
}

#[test]
fn a_pin_on_no_net_needs_no_via_to_reach_it() {
    let (mut db, m) = design();
    let cell = db
        .add_cell(
            "c".into(),
            m,
            pt(3400, 0),
            Orientation::N,
            PlacementStatus::Placed,
        )
        .unwrap();
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    assert_eq!(
        access.blocked_pins(&db, cell, pt(3400, 0), Orientation::N),
        0
    );
    assert_eq!(blocked(3400), 1, "on a net, the same pin has no via");
}

#[test]
fn a_cells_metal_on_the_checked_layers_is_its_pins_and_obstructions_placed() {
    let (mut db, _) = design();
    let site = SiteId(0);
    let mut master = one_pin_master(
        "O",
        site,
        pin(
            "a",
            PinDirection::Input,
            metal(LayerId(0), rect(50, 500, 150, 1500)),
        ),
    );
    master.obstructions = vec![
        metal(LayerId(1), rect(200, 0, 300, 100)),
        metal(LayerId(2), rect(0, 0, 100, 100)),
    ];
    let master = db.library.add_macro(master).unwrap();
    let cell = db
        .add_cell(
            "o".into(),
            master,
            pt(1000, 0),
            Orientation::N,
            PlacementStatus::Placed,
        )
        .unwrap();
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    let metal = access.cell_metal(&db, cell, pt(1000, 0), Orientation::N);

    assert_eq!(
        metal,
        [
            (LayerId(0), rect(1050, 500, 1150, 1500)),
            (LayerId(1), rect(1200, 0, 1300, 100)),
        ]
    );
}

fn wide_table_access() -> PinAccess {
    let (db, _) = design_with(3900, Some(wide_metal_table()), None);
    PinAccess::new(&db, LayerId(1)).unwrap()
}

#[test]
fn metal_disturbs_reliance_within_its_layers_spacing() {
    let access = wide_table_access();
    let relied = [(LayerId(1), rect(0, 0, 100, 100))];

    assert!(access.disturbs(&[(LayerId(1), rect(220, 0, 320, 100))], &relied));
    assert!(!access.disturbs(&[(LayerId(1), rect(250, 0, 350, 100))], &relied));
}

#[test]
fn the_pin_layers_own_spacing_decides_whether_its_metal_disturbs() {
    let access = wide_table_access();
    let relied = [(LayerId(0), rect(0, 0, 100, 100))];

    assert!(access.disturbs(&[(LayerId(0), rect(150, 0, 250, 100))], &relied));
    assert!(!access.disturbs(&[(LayerId(0), rect(200, 0, 300, 100))], &relied));
}

#[test]
fn metal_on_another_layer_does_not_disturb_reliance() {
    let access = wide_table_access();
    let relied = [(LayerId(1), rect(0, 0, 100, 100))];

    assert!(!access.disturbs(&[(LayerId(0), rect(0, 0, 100, 100))], &relied));
    assert!(!access.disturbs(&[], &relied));
    assert!(!access.disturbs(&[(LayerId(1), rect(0, 0, 100, 100))], &[]));
}

/// The check on a design whose first via's M2 pad is `pad`.
fn access_with_pad(pad: Rect<Dbu>) -> PinAccess {
    let (mut db, _) = design();
    for v in &mut db.library.vias {
        if v.cut == CutLayerId(0) {
            v.shapes[2].rect = pad;
        }
    }
    PinAccess::new(&db, LayerId(1)).unwrap()
}

#[test]
fn the_reach_of_a_via_pad_counts_its_far_side_in_x() {
    let access = access_with_pad(rect(-200, -50, 100, 50));

    assert_eq!(access.reach(), Dbu::new(300));
}

#[test]
fn the_reach_of_a_via_pad_counts_its_far_side_in_y() {
    let access = access_with_pad(rect(-50, -200, 50, 100));

    assert_eq!(access.reach(), Dbu::new(300));
}

#[test]
fn the_check_margin_adds_access_escape_and_spacing() {
    let (db, _) = design();
    let access = PinAccess::new(&db, LayerId(1)).unwrap();

    assert_eq!(access.check_margin(), Dbu::new(90 + 2000 + 90 + 100));
}

#[test]
fn the_check_margin_counts_the_far_side_of_a_pad_in_x() {
    let access = access_with_pad(rect(-200, -50, 100, 50));

    assert_eq!(access.check_margin(), Dbu::new(200 + 2000 + 90 + 100));
}

#[test]
fn the_check_margin_counts_the_far_side_of_a_pad_in_y() {
    let access = access_with_pad(rect(-50, -200, 50, 100));

    assert_eq!(access.check_margin(), Dbu::new(200 + 2000 + 90 + 100));
}

/// How many pins are blocked of a cell at the origin on a net, whose one
/// master has its M1 pin at `pin_rect`.
fn blocked_with_pin_at(pin_rect: Rect<Dbu>) -> usize {
    let (mut db, _) = design();
    let master = one_pin_master(
        "N",
        SiteId(0),
        pin("a", PinDirection::Input, metal(LayerId(0), pin_rect)),
    );
    let master = db.library.add_macro(master).unwrap();
    let cell = cell_at(&mut db, master, 0);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    access.blocked_pins(&db, cell, pt(0, 0), Orientation::N)
}

#[test]
fn a_via_whose_cut_would_stick_out_of_the_pin_sideways_does_not_reach_it() {
    assert_eq!(blocked_with_pin_at(rect(150, 500, 250, 1500)), 1);
}

#[test]
fn a_via_whose_cut_would_stick_out_of_the_pin_lengthways_does_not_reach_it() {
    assert_eq!(blocked_with_pin_at(rect(50, 250, 150, 350)), 1);
}

fn m3(rect: Rect<Dbu>) -> Shape {
    metal(LayerId(2), rect)
}

/// A design whose escape works in chunks of 1.6 um along M3, with M3
/// metal in short pieces across the die, beside its chunk boundaries and
/// past its edges, at the edge of the spacing from the tracks, and the M3
/// pad of its vias `pad`.
fn chunked_design_with_pad(pad: Rect<Dbu>) -> NetlistDB {
    let (mut db, _) = design();
    db.library.routing_layers[LayerId(1)].pitch_x = Dbu::new(20);
    for v in &mut db.library.vias {
        if v.cut == CutLayerId(1) {
            v.shapes[2].rect = pad;
        }
    }
    for blockage in [
        rect(-800, 1000, -400, 1100),
        rect(10_400, 1500, 10_800, 1600),
        rect(11_500, 1700, 11_600, 1800),
        rect(1440, 2100, 1560, 2200),
        rect(1650, 3000, 1750, 3100),
        rect(3100, 800, 4800, 900),
        rect(4790, 2600, 4900, 2700),
        rect(6000, 1149, 6100, 1249),
        rect(7000, 751, 7100, 851),
        rect(8000, 1950, 8030, 2050),
        rect(9500, 3500, 9700, 3600),
    ]
    .map(m3)
    {
        let _ = db.routing_blockages.push(blockage);
    }
    db
}

#[test]
fn pad_stretches_of_chunks_agree_with_checking_every_via_position() {
    let db = chunked_design_with_pad(rect(-90, -50, 90, 50));

    assert_pad_blocks_match_direct_checks(&db, 1);
}

#[test]
fn pad_stretches_agree_for_a_pad_reaching_far_to_the_left() {
    let db = chunked_design_with_pad(rect(-700, -50, 100, 50));

    assert_pad_blocks_match_direct_checks(&db, 1);
}

#[test]
fn pad_stretches_agree_for_a_pad_reaching_far_to_the_right() {
    let db = chunked_design_with_pad(rect(-100, -50, 700, 50));

    assert_pad_blocks_match_direct_checks(&db, 1);
}

fn chunk_of_origin(die_min_x: i64, along: i64) -> usize {
    let mut db = chunked_design_with_pad(rect(-90, -50, 90, 50));
    db.die_area = rect(die_min_x, 0, die_min_x + 10_000, 4000);
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    access.up.as_ref().unwrap().chunk_of(Dbu::new(along))
}

#[test]
fn a_via_origin_falls_in_the_chunk_counted_from_the_dies_edge() {
    assert_eq!(chunk_of_origin(0, 0), 0);
    assert_eq!(chunk_of_origin(0, 1599), 0);
    assert_eq!(chunk_of_origin(0, 1600), 1);
    assert_eq!(chunk_of_origin(0, 4800), 3);
    assert_eq!(chunk_of_origin(300, 1899), 0);
    assert_eq!(chunk_of_origin(300, 1900), 1);
}

#[test]
fn via_origins_beyond_the_die_fall_in_its_first_and_last_chunks() {
    assert_eq!(chunk_of_origin(0, -1), 0);
    assert_eq!(chunk_of_origin(0, -50_000), 0);
    assert_eq!(chunk_of_origin(0, 9999), 6);
    assert_eq!(chunk_of_origin(0, 50_000), 6);
}

fn free_m3_tracks(free: &[i64]) -> Vec<Shape> {
    (200..=3800)
        .step_by(200)
        .filter(|y| !free.contains(y))
        .map(|y| m3(rect(0, y - 50, 10_000, y + 50)))
        .collect()
}

fn design_blocked_by(blockages: Vec<Shape>) -> NetlistDB {
    let (mut db, _) = design();
    for blockage in blockages {
        let _ = db.routing_blockages.push(blockage);
    }
    db
}

/// The run `start` escapes by in `db`, for the net of no cell.
fn escape_of(db: &NetlistDB, start: Rect<Dbu>) -> Option<Rect<Dbu>> {
    let access = PinAccess::new(db, LayerId(1)).unwrap();
    let everything = rect(-100_000, -100_000, 100_000, 100_000);
    let mut gathered = NeighbourMetal::default();
    gathered.above.extend(access.metal_above.near(&everything));
    gathered.below.extend(access.metal_below.near(&everything));
    gathered.zone = Some(everything);
    let holder = Holder::of_net(Some(NetId::new(0)));
    access.escapes(&start, holder, &gathered)
}

#[test]
fn metal_escapes_by_the_nearer_track_above_it() {
    let db = design();

    assert_eq!(
        escape_of(&db.0, rect(250, 1060, 350, 1160)),
        Some(rect(250, 1060, 350, 1290))
    );
}

#[test]
fn metal_escapes_by_the_nearer_track_below_it() {
    let db = design();

    assert_eq!(
        escape_of(&db.0, rect(250, 1010, 350, 1110)),
        Some(rect(250, 910, 350, 1110))
    );
}

#[test]
fn metal_between_two_equally_near_tracks_escapes_by_the_one_above() {
    let db = design();

    assert_eq!(
        escape_of(&db.0, rect(250, 1050, 350, 1150)),
        Some(rect(250, 1050, 350, 1290))
    );
}

#[test]
fn metal_over_tracks_escapes_by_the_lowest_of_them() {
    let db = design();

    assert_eq!(
        escape_of(&db.0, rect(250, 1000, 350, 1400)),
        Some(rect(250, 910, 350, 1400))
    );
}

#[test]
fn metal_escapes_by_a_track_exactly_an_escape_reach_below_it() {
    let db = design_blocked_by(free_m3_tracks(&[200]));

    assert_eq!(
        escape_of(&db, rect(250, 2200, 350, 2300)),
        Some(rect(250, 110, 350, 2300))
    );
}

#[test]
fn metal_escapes_by_a_track_within_the_escape_reach_below_it() {
    let db = design_blocked_by(free_m3_tracks(&[1200]));

    assert_eq!(
        escape_of(&db, rect(250, 2200, 350, 2300)),
        Some(rect(250, 1110, 350, 2300))
    );
}

#[test]
fn metal_does_not_escape_by_a_track_beyond_the_escape_reach_below_it() {
    let db = design_blocked_by(free_m3_tracks(&[200]));

    assert_eq!(escape_of(&db, rect(250, 2400, 350, 2500)), None);
}

#[test]
fn metal_escapes_by_a_track_exactly_an_escape_reach_above_it() {
    let db = design_blocked_by(free_m3_tracks(&[3200]));

    assert_eq!(
        escape_of(&db, rect(250, 1000, 350, 1200)),
        Some(rect(250, 1000, 350, 3290))
    );
}

#[test]
fn metal_escapes_by_a_track_within_the_escape_reach_above_it() {
    let db = design_blocked_by(free_m3_tracks(&[2000]));

    assert_eq!(
        escape_of(&db, rect(250, 1000, 350, 1200)),
        Some(rect(250, 1000, 350, 2090))
    );
}

#[test]
fn metal_does_not_escape_by_a_track_beyond_the_escape_reach_above_it() {
    let db = design_blocked_by(free_m3_tracks(&[3400]));

    assert_eq!(escape_of(&db, rect(250, 1000, 350, 1200)), None);
}

fn transposed(rect: Rect<Dbu>) -> Rect<Dbu> {
    Rect::new(
        pt(rect.min.y.get(), rect.min.x.get()),
        pt(rect.max.y.get(), rect.max.x.get()),
    )
}

/// `db` with every x and y exchanged, so that M2 runs horizontally.
fn turned(mut db: NetlistDB) -> NetlistDB {
    for l in &mut db.library.routing_layers {
        l.direction = match l.direction {
            Direction::Horizontal => Direction::Vertical,
            Direction::Vertical => Direction::Horizontal,
        };
    }
    for v in &mut db.library.vias {
        for s in &mut v.shapes {
            s.rect = transposed(s.rect);
        }
    }
    for t in &mut db.tracks {
        t.axis = match t.axis {
            TrackAxis::X => TrackAxis::Y,
            TrackAxis::Y => TrackAxis::X,
        };
    }
    for n in &mut db.special_nets {
        for w in &mut n.wires {
            w.from = pt(w.from.y.get(), w.from.x.get());
            w.to = pt(w.to.y.get(), w.to.x.get());
        }
    }
    for s in &mut db.routing_blockages {
        s.rect = transposed(s.rect);
    }
    db.die_area = transposed(db.die_area);
    db
}

/// Metal on M2's track at 300 running 1.0 um, escaping past M2 metal
/// `beside` it, with the layout turned a quarter when `run_across`.
fn escape_past_obstacle(beside: Rect<Dbu>, run_across: bool) -> Option<Rect<Dbu>> {
    let db = design_blocked_by(vec![metal(LayerId(1), beside)]);
    let start = rect(250, 1050, 350, 2050);
    if run_across {
        escape_of(&turned(db), transposed(start))
    } else {
        escape_of(&db, start)
    }
}

#[test]
fn a_run_may_not_come_within_spacing_of_metal_on_its_right() {
    assert_eq!(
        escape_past_obstacle(rect(420, 1700, 520, 1900), false),
        None
    );
}

#[test]
fn a_run_may_not_come_within_spacing_of_metal_on_its_left() {
    assert_eq!(
        escape_past_obstacle(rect(130, 1700, 180, 1900), false),
        None
    );
}

#[test]
fn a_horizontal_run_may_not_come_within_spacing_of_metal_above_it() {
    assert_eq!(escape_past_obstacle(rect(420, 1700, 520, 1900), true), None);
}

#[test]
fn a_horizontal_run_may_not_come_within_spacing_of_metal_below_it() {
    assert_eq!(escape_past_obstacle(rect(130, 1700, 180, 1900), true), None);
}

#[test]
fn a_pad_blocked_only_by_metal_of_its_own_net_is_clear_for_that_net() {
    let db = chunked_design_with_pad(rect(-90, -50, 90, 50));
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    let up = access.up.as_ref().unwrap();
    up.pads[0]
        .set(vec![PadStretches(vec![stretch(
            0,
            1599,
            PadBlock::Net(NetId::new(7)),
        )])])
        .unwrap();
    let at = pt(100, 0);
    let on_net = |net| Holder::of_net(Some(NetId::new(net)));

    assert!(up.pad_clear(0, 0, at, on_net(7)));
    assert!(!up.pad_clear(0, 0, at, on_net(8)));
    assert!(!up.pad_clear(0, 0, at, Holder::of_net(None)));
}
