use super::*;
use pare_common::units::Farads;

const FEMTOFARAD: f64 = 1e-15;

/// Sinks of `cap_ff` femtofarads at the corners of a square 100 length
/// units a side, added counterclockwise from the origin.
fn square_corners(use_max_cap: bool, cap_ff: f64) -> SinkClustering {
    let sink_buffer_input_cap = Farads32::of(Farads::new(FEMTOFARAD));
    let mut clustering =
        SinkClustering::new(use_max_cap, FaradsPerDbu::new(0.0), sink_buffer_input_cap);
    for (x, y) in [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)] {
        clustering.add_point(
            x,
            y,
            Farads32::of(Farads::new(cap_ff * FEMTOFARAD)),
            SegmentLength::default(),
        );
    }
    clustering
}

#[test]
fn the_curve_visits_the_unit_squares_corners_up_across_and_down() {
    let corners = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)];

    let thetas = corners.map(|(x, y)| theta(x, y));

    let expected = [0.0, 0.25, 0.5, 0.75];
    assert!(
        thetas
            .iter()
            .zip(expected)
            .all(|(t, e)| (t - e).abs() < f64::EPSILON),
        "{thetas:?}"
    );
}

#[test]
fn sinks_are_cut_along_the_curve_into_groups_of_the_size() {
    let mut clustering = square_corners(false, 1.0);

    clustering.run(2, 1000.0, 1);

    assert_eq!(clustering.solution(), [vec![0, 3], vec![2, 1]]);
}

#[test]
fn sinks_farther_apart_than_the_diameter_stand_alone() {
    let mut clustering = square_corners(false, 1.0);

    clustering.run(4, 50.0, 1);

    assert_eq!(clustering.solution(), [vec![0], vec![3], vec![2], vec![1]]);
}

#[test]
fn under_a_load_limit_the_load_cuts_clusters_rather_than_the_size() {
    // Ten times the sink buffer's input capacitance holds two sinks of
    // four times it, not three.
    let mut clustering = square_corners(true, 4.0);

    clustering.run(4, 1000.0, 1);

    assert_eq!(clustering.solution(), [vec![0, 3], vec![2, 1]]);
}

#[test]
fn a_group_size_of_one_puts_each_sink_alone() {
    let mut clustering = square_corners(false, 1.0);

    clustering.run(1, 1000.0, 1);

    assert_eq!(clustering.solution(), [vec![0], vec![3], vec![2], vec![1]]);
}

#[test]
fn the_cut_at_the_curves_start_wins_when_it_is_tightest() {
    // Along the curve the sinks run 0, 1, 2, 3, and only the cut from 0
    // pairs each with its near neighbour.
    let sink_buffer_input_cap = Farads32::of(Farads::new(FEMTOFARAD));
    let mut clustering = SinkClustering::new(false, FaradsPerDbu::new(0.0), sink_buffer_input_cap);
    for (x, y) in [(0.0, 0.0), (0.0, 10.0), (100.0, 100.0), (100.0, 90.0)] {
        clustering.add_point(
            x,
            y,
            Farads32::of(Farads::new(FEMTOFARAD)),
            SegmentLength::default(),
        );
    }

    clustering.run(2, 1000.0, 1);

    assert_eq!(clustering.solution(), [vec![0, 1], vec![2, 3]]);
}

#[test]
fn a_later_run_keeps_the_earlier_clustering_unless_it_costs_less() {
    let mut clustering = square_corners(false, 1.0);
    clustering.run(2, 1000.0, 1);

    clustering.run(4, 50.0, 1);

    assert_eq!(clustering.solution(), [vec![0, 3], vec![2, 1]]);
}
