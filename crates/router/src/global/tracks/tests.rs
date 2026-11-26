use super::*;

#[test]
fn adding_and_removing_a_fraction_cancels_exactly() {
    let loss = Tracks::from_f64(0.175);
    let mut usage = Tracks::whole(3);

    for _ in 0..1000 {
        usage += loss;
    }
    for _ in 0..1000 {
        usage -= loss;
    }

    assert_eq!(usage, Tracks::whole(3));
}

#[test]
fn rounding_reports_the_nearest_whole_track() {
    assert_eq!(Tracks::from_f64(2.49).rounded(), 2);
    assert_eq!(Tracks::from_f64(2.5).rounded(), 3);
    assert_eq!(Tracks::ZERO.rounded(), 0);
}

#[test]
fn saturating_sub_stops_at_zero() {
    assert_eq!(
        Tracks::whole(2).saturating_sub(Tracks::whole(5)),
        Tracks::ZERO
    );
    assert_eq!(
        Tracks::whole(5).saturating_sub(Tracks::whole(2)),
        Tracks::whole(3)
    );
}
