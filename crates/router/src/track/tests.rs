use super::*;

fn cell(layer: u8, x: u16, y: u16) -> GCell {
    GCell {
        layer: LayerId(layer),
        x,
        y,
    }
}

fn alternating(l: LayerId) -> Direction {
    if l.index().is_multiple_of(2) {
        Direction::Vertical
    } else {
        Direction::Horizontal
    }
}

#[test]
fn runs_merge_paths_along_a_panel_and_skip_layer_changes() {
    let paths = vec![
        vec![cell(1, 0, 2), cell(1, 1, 2), cell(1, 2, 2), cell(2, 2, 2)],
        vec![cell(1, 2, 2), cell(1, 3, 2)],
        vec![cell(1, 5, 2), cell(1, 6, 2)],
        vec![cell(2, 2, 2), cell(2, 2, 3)],
    ];

    let found = runs(alternating, &paths);

    let row = PanelId {
        layer: LayerId(1),
        index: 2,
    };
    let column = PanelId {
        layer: LayerId(2),
        index: 2,
    };
    assert_eq!(found, [(row, 0, 3), (row, 5, 6), (column, 2, 3)]);
}

#[test]
fn a_wire_is_cut_around_its_blocked_edges() {
    let pieces = clear_runs(CrossIdx(0), CrossIdx(8), |c| [3, 4, 7].contains(&c.0));

    assert_eq!(
        pieces,
        [(CrossIdx(0), CrossIdx(3)), (CrossIdx(5), CrossIdx(7))]
    );
}

#[test]
fn overlap_counts_the_spacing_and_is_zero_when_apart() {
    assert_eq!(
        overlap(
            Dbu::new(0),
            Dbu::new(100),
            Dbu::new(50),
            Dbu::new(300),
            Dbu::new(10)
        ),
        Dbu::new(60)
    );
    assert_eq!(
        overlap(
            Dbu::new(0),
            Dbu::new(100),
            Dbu::new(110),
            Dbu::new(300),
            Dbu::new(10)
        ),
        Dbu::new(0)
    );
}
