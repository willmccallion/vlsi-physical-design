use super::*;
use pare_common::db::indices::CutLayerId;

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

/// A 1000 by 1000 die in tiles of 100 with metal at two corners, one
/// across four tiles and past the die's upper right corner, and a cut.
fn cornered() -> (Layout, ShapeTiles) {
    let mut layout = Layout::new(1, 1);
    let owner = layout.owner(Owner::Net(NetId::new(0)), false);
    let metal = LayerRef::Routing(LayerId::new(0));
    layout.add(metal, owner, rect(850, 850, 1100, 1100), Origin::Input);
    layout.add(metal, owner, rect(0, 0, 50, 50), Origin::Input);
    let cut = LayerRef::Cut(CutLayerId::new(0));
    layout.add(cut, owner, rect(960, 960, 990, 990), Origin::Input);
    let tiles = ShapeTiles::new(&layout, &rect(0, 0, 1000, 1000), Dbu::new(100));
    (layout, tiles)
}

#[test]
fn shapes_past_the_die_edge_are_found_from_its_edge_tiles_in_order_once() {
    let (_, tiles) = cornered();

    let near = tiles.near(&rect(900, 900, 2000, 2000));

    let metal = LayerRef::Routing(LayerId::new(0));
    assert_eq!(near, [(metal, 0), (LayerRef::Cut(CutLayerId::new(0)), 0)]);
}

#[test]
fn a_window_beyond_the_die_finds_the_shapes_of_the_nearest_tiles() {
    let (_, tiles) = cornered();

    let near = tiles.near(&rect(-500, -500, -400, -400));

    assert_eq!(near, [(LayerRef::Routing(LayerId::new(0)), 1)]);
}
