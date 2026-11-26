use super::*;

#[test]
fn fixed_tracks_return_the_stretches_overlapping_a_range() {
    let fixed = FixedTracks {
        stretches: vec![vec![vec![
            Interval::new(Dbu::new(0), Dbu::new(30)),
            Interval::new(Dbu::new(50), Dbu::new(60)),
            Interval::new(Dbu::new(80), Dbu::new(90)),
        ]]]
        .into(),
    };
    let track = TrackRef {
        layer: LayerId(0),
        track: TrackIdx(0),
    };

    let got: Vec<Interval> = fixed
        .fixed_stretches(track, Interval::new(Dbu::new(30), Dbu::new(80)))
        .collect();

    assert_eq!(got, vec![Interval::new(Dbu::new(50), Dbu::new(60))]);
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::from_corners(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

/// Via cuts are 100 wide with spacing 100, so fine buckets are 800 wide
/// and coarse ones 51200.
fn layer() -> CutLayer {
    CutLayer::new(CutTech {
        spacing: Dbu::new(100),
        max_half: Dbu::new(50),
    })
}

fn cut(rect: Rect<Dbu>, owner: Owner, origin: Origin) -> CutOccupant {
    CutOccupant::new(&rect, owner, origin)
}

#[test]
fn a_cut_beside_the_far_edge_of_a_large_obstruction_conflicts_with_it() {
    let mut cuts = layer();
    cuts.add(cut(
        rect(0, 0, 300_000, 40_000),
        Owner::Obstacle,
        Origin::Fixed,
    ));

    let near_edge = cuts
        .conflicts(rect(300_050, 100, 300_150, 200), Owner::Net(NetId(1)))
        .count();
    let clear = cuts
        .conflicts(rect(300_100, 100, 300_200, 200), Owner::Net(NetId(1)))
        .count();

    assert_eq!(near_edge, 1);
    assert_eq!(clear, 0);
}

#[test]
fn a_large_shape_is_reported_once_however_many_buckets_a_query_spans() {
    let mut cuts = layer();
    cuts.add(cut(
        rect(0, 0, 300_000, 40_000),
        Owner::Obstacle,
        Origin::Fixed,
    ));

    let spanning = cuts
        .conflicts(rect(51_100, 39_950, 51_300, 40_050), Owner::Net(NetId(1)))
        .count();

    assert_eq!(spanning, 1);
}

#[test]
fn removing_wiring_keeps_fixed_cuts_in_both_tiers() {
    let mut cuts = layer();
    let shapes = [
        cut(rect(0, 0, 100, 100), Owner::Net(NetId(1)), Origin::Wiring),
        cut(rect(0, 0, 100, 100), Owner::Net(NetId(1)), Origin::Fixed),
        cut(
            rect(1000, 0, 60_000, 500),
            Owner::Net(NetId(1)),
            Origin::Wiring,
        ),
    ];
    let buckets: Vec<CutBucket> = shapes.iter().map(|c| cuts.bucket_of(&c.rect())).collect();
    for shape in shapes {
        cuts.add(shape);
    }

    for bucket in buckets {
        cuts.retain_in(bucket, |o| {
            o.owner != Owner::Net(NetId(1)) || o.origin == Origin::Fixed
        });
    }

    assert_eq!(cuts.all().count(), 1);
    assert_eq!(cuts.all().next().map(|c| c.origin), Some(Origin::Fixed));
}

#[test]
fn occupancy_records_hold_coordinates_in_32_bits() {
    assert_eq!(size_of::<Occupant>(), 44);
    assert_eq!(size_of::<CutOccupant>(), 28);
}

#[test]
fn a_cut_keeps_its_rectangle() {
    let r = rect(-1_000, 2_000, 3_000, 4_000);

    let kept = cut(r, Owner::Net(NetId(1)), Origin::Wiring).rect();

    assert_eq!(kept, r);
}

#[test]
fn every_cut_is_listed_once() {
    let mut cuts = layer();
    cuts.add(cut(
        rect(0, 0, 100, 100),
        Owner::Net(NetId(1)),
        Origin::Wiring,
    ));
    cuts.add(cut(
        rect(-60_000, -60_000, 60_000, 60_000),
        Owner::Obstacle,
        Origin::Fixed,
    ));

    assert_eq!(cuts.all().count(), 2);
}
