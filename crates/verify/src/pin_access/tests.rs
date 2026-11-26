use super::*;
use pare_common::db::design::{PlacementStatus, SpecialNet, SpecialWire, TrackAxis, TrackDef};
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

#[test]
fn pad_stretches_agree_with_checking_every_via_position() {
    let (db, _) = design_full(3900, Some(wide_metal_table()), None, Some(2000));
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    let up = access.up.as_ref().unwrap();
    let metal = LayerMetal::new(&db, LayerId(2));
    let mut seen = std::collections::HashSet::new();

    for (t, &track) in up.tracks.iter().enumerate() {
        for (k, via) in up.vias.iter().enumerate() {
            for x in (-1000..11000).step_by(7) {
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
