use super::*;

fn node(layer: u8, track: u32, cross: u32) -> Node {
    Node {
        layer: LayerId(layer),
        track: TrackIdx(track),
        cross: CrossIdx(cross),
    }
}

fn wire(layer: u8, track: u32, cross: u32) -> Edge {
    Edge::Wire(node(layer, track, cross))
}

fn jog(track: u32, cross: u32) -> Edge {
    Edge::Jog(node(1, track, cross))
}

fn via(track: u32, cross: u32) -> Edge {
    Edge::Via {
        lower: node(1, track, cross),
        upper: node(2, cross, track),
    }
}

/// The edges in a sorted list without repeats, as routes held them.
fn sorted(edges: &[Edge]) -> Vec<Edge> {
    let mut list = edges.to_vec();
    list.sort_unstable();
    list.dedup();
    list
}

fn mixed() -> Vec<Edge> {
    vec![
        wire(1, 4, 7),
        via(4, 9),
        wire(1, 4, 5),
        wire(1, 4, 6),
        wire(2, 3, 1),
        wire(1, 4, 9),
        via(2, 2),
        wire(1, 4, 6),
        jog(4, 6),
        wire(1, 5, 0),
        jog(2, 3),
        jog(4, 6),
    ]
}

#[test]
fn a_set_reads_back_its_edges_as_a_sorted_list_without_repeats() {
    let set: EdgeSet = mixed().into_iter().collect();

    assert_eq!(set.iter().collect::<Vec<_>>(), sorted(&mixed()));
}

#[test]
fn consecutive_wire_edges_on_a_track_become_one_run() {
    let set: EdgeSet = mixed().into_iter().collect();

    assert_eq!(
        set.runs(),
        [
            Run {
                layer: LayerId(1),
                track: TrackIdx(4),
                from: CrossIdx(5),
                to: CrossIdx(8)
            },
            Run {
                layer: LayerId(1),
                track: TrackIdx(4),
                from: CrossIdx(9),
                to: CrossIdx(10)
            },
            Run {
                layer: LayerId(1),
                track: TrackIdx(5),
                from: CrossIdx(0),
                to: CrossIdx(1)
            },
            Run {
                layer: LayerId(2),
                track: TrackIdx(3),
                from: CrossIdx(1),
                to: CrossIdx(2)
            },
        ]
    );
}

#[test]
fn subtracting_leaves_what_removing_each_edge_leaves() {
    let mut set: EdgeSet = mixed().into_iter().collect();
    let gone = [
        wire(1, 4, 6),
        via(4, 9),
        wire(1, 5, 0),
        jog(2, 3),
        wire(3, 1, 1),
    ];

    set.subtract(&gone.into_iter().collect());

    let expected: Vec<Edge> = sorted(&mixed())
        .into_iter()
        .filter(|e| !gone.contains(e))
        .collect();
    assert_eq!(set.iter().collect::<Vec<_>>(), expected);
}

#[test]
fn subtracting_a_stretch_longer_than_a_run_removes_all_of_it() {
    let mut set = EdgeSet::from_runs([
        Run {
            layer: LayerId(1),
            track: TrackIdx(4),
            from: CrossIdx(5),
            to: CrossIdx(8),
        },
        Run {
            layer: LayerId(1),
            track: TrackIdx(4),
            from: CrossIdx(12),
            to: CrossIdx(20),
        },
    ]);

    set.subtract(&EdgeSet::from_runs([Run {
        layer: LayerId(1),
        track: TrackIdx(4),
        from: CrossIdx(0),
        to: CrossIdx(14),
    }]));

    assert_eq!(
        set.runs(),
        [Run {
            layer: LayerId(1),
            track: TrackIdx(4),
            from: CrossIdx(14),
            to: CrossIdx(20)
        }]
    );
}

#[test]
fn retaining_splits_a_run_where_an_edge_is_dropped() {
    let mut set = EdgeSet::from_runs([Run {
        layer: LayerId(1),
        track: TrackIdx(4),
        from: CrossIdx(0),
        to: CrossIdx(6),
    }]);

    set.retain(|e| e != wire(1, 4, 2) && e != wire(1, 4, 3));

    assert_eq!(
        set.runs(),
        [
            Run {
                layer: LayerId(1),
                track: TrackIdx(4),
                from: CrossIdx(0),
                to: CrossIdx(2)
            },
            Run {
                layer: LayerId(1),
                track: TrackIdx(4),
                from: CrossIdx(4),
                to: CrossIdx(6)
            },
        ]
    );
}

#[test]
fn a_union_merges_runs_that_meet() {
    let mut set = EdgeSet::from_runs([Run {
        layer: LayerId(1),
        track: TrackIdx(4),
        from: CrossIdx(0),
        to: CrossIdx(3),
    }]);

    set.union(&EdgeSet::from_runs([Run {
        layer: LayerId(1),
        track: TrackIdx(4),
        from: CrossIdx(3),
        to: CrossIdx(5),
    }]));

    assert_eq!(
        set.runs(),
        [Run {
            layer: LayerId(1),
            track: TrackIdx(4),
            from: CrossIdx(0),
            to: CrossIdx(5)
        }]
    );
}

#[test]
fn clipping_keeps_the_jogs_whose_ends_it_accepts() {
    let set: EdgeSet = mixed().into_iter().collect();

    let clipped = set.clipped(|_| None, |(_, b)| b.track == TrackIdx(5));

    assert_eq!(clipped.iter().collect::<Vec<_>>(), [jog(4, 6)]);
}

#[test]
fn retaining_drops_the_jogs_it_refuses() {
    let mut set: EdgeSet = mixed().into_iter().collect();

    set.retain(|e| e != jog(2, 3));

    assert_eq!(set.jogs(), [node(1, 4, 6)]);
}

#[test]
fn sets_of_the_same_edges_are_equal_however_they_were_built() {
    let built: EdgeSet = mixed().into_iter().collect();
    let mut grown = EdgeSet::default();
    for e in mixed().into_iter().rev() {
        grown.extend([e]);
    }

    assert_eq!(built, grown);
}
