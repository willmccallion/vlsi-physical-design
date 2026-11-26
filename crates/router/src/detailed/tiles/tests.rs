use super::*;

const fn cell(x: i64, y: i64) -> CellBox {
    CellBox {
        x0: Gcells(x),
        y0: Gcells(y),
        x1: Gcells(x),
        y1: Gcells(y),
    }
}

const DIE: CellBox = CellBox {
    x0: Gcells(0),
    y0: Gcells(0),
    x1: Gcells(99),
    y1: Gcells(99),
};

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

#[test]
fn violations_a_few_gcells_apart_form_one_cluster() {
    let clusters = merge_boxes(vec![cell(10, 10), cell(14, 12)]);

    assert_eq!(
        clusters,
        [CellBox {
            x0: Gcells(10),
            y0: Gcells(10),
            x1: Gcells(14),
            y1: Gcells(12)
        }]
    );
}

#[test]
fn violations_farther_apart_stay_separate_clusters() {
    let clusters = merge_boxes(vec![cell(10, 10), cell(15, 10)]);

    assert_eq!(clusters, [cell(10, 10), cell(15, 10)]);
}

#[test]
fn a_lone_violation_gets_a_centred_tile_and_four_shifted_ones() {
    let tiles = expand_boxes(&[cell(50, 50)], DIE);

    let expected = BTreeSet::from([
        CellBox {
            x0: Gcells(47),
            y0: Gcells(47),
            x1: Gcells(53),
            y1: Gcells(53),
        },
        CellBox {
            x0: Gcells(44),
            y0: Gcells(47),
            x1: Gcells(50),
            y1: Gcells(53),
        },
        CellBox {
            x0: Gcells(50),
            y0: Gcells(47),
            x1: Gcells(56),
            y1: Gcells(53),
        },
        CellBox {
            x0: Gcells(47),
            y0: Gcells(44),
            x1: Gcells(53),
            y1: Gcells(50),
        },
        CellBox {
            x0: Gcells(47),
            y0: Gcells(50),
            x1: Gcells(53),
            y1: Gcells(56),
        },
    ]);
    assert_eq!(tiles, [expected]);
}

#[test]
fn tiles_stop_at_the_die_edge() {
    let tiles = expand_boxes(&[cell(1, 50)], DIE);

    assert!(tiles[0].iter().all(|t| t.x0 >= Gcells(0)));
    assert!(tiles[0].contains(&CellBox {
        x0: Gcells(0),
        y0: Gcells(47),
        x1: Gcells(4),
        y1: Gcells(53)
    }));
}

#[test]
fn tiles_of_one_cluster_never_reach_onto_another_clusters_tiles() {
    let tiles = expand_boxes(&[cell(50, 50), cell(56, 50)], DIE);

    for a in &tiles[0] {
        for b in &tiles[1] {
            assert!(a.x1 < b.x0 || b.x1 < a.x0 || a.y1 < b.y0 || b.y1 < a.y0);
        }
    }
}

#[test]
fn workers_whose_reaches_touch_go_into_different_batches() {
    let reaches = [rect(0, 0, 10, 10), rect(10, 0, 20, 10), rect(30, 0, 40, 10)];

    let split = batches(&reaches);

    assert_eq!(split, [vec![0, 2], vec![1]]);
}

#[test]
fn touching_guides_running_the_same_way_merge() {
    let merged = merge_guides(vec![rect(0, 0, 30, 10), rect(30, 0, 60, 10)]);

    assert_eq!(merged, [rect(0, 0, 60, 10)]);
}

#[test]
fn touching_guides_running_across_each_other_stay_apart() {
    let merged = merge_guides(vec![rect(0, 0, 30, 10), rect(20, 0, 30, 40)]);

    assert_eq!(merged.len(), 2);
}
