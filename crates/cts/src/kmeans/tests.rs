use super::*;
use pare_common::testing::{SeededRng, assert_golden};

#[test]
fn sinks_split_toward_the_nearer_mean_within_capacity() {
    let sinks = [
        SinkPoint::new(0.0, 0.0),
        SinkPoint::new(1.0, 0.0),
        SinkPoint::new(9.0, 0.0),
        SinkPoint::new(10.0, 0.0),
    ];
    let mut clustering = Clustering::new(&sinks, SinkPoint::new(5.0, 0.0));
    let mut means = [SinkPoint::new(0.0, 0.0), SinkPoint::new(10.0, 0.0)];

    clustering.run(2, 5, 4, &mut means);

    assert_eq!(clustering.clusters(), &[vec![0, 1], vec![2, 3]]);
}

#[test]
fn capacity_forces_the_cheapest_sinks_to_move() {
    let sinks = [
        SinkPoint::new(0.0, 0.0),
        SinkPoint::new(1.0, 0.0),
        SinkPoint::new(2.0, 0.0),
        SinkPoint::new(10.0, 0.0),
    ];
    let mut clustering = Clustering::new(&sinks, SinkPoint::new(5.0, 0.0));
    let mut means = [SinkPoint::new(0.0, 0.0), SinkPoint::new(10.0, 0.0)];

    clustering.run(2, 0, 4, &mut means);

    assert_eq!(clustering.clusters()[0].len(), 2);
    assert_eq!(clustering.clusters()[1].len(), 2);
}

/// A point on a grid `step` length units apart, eight steps a side.
fn grid_point(rng: &mut SeededRng, step: f32) -> SinkPoint {
    let at = |rng: &mut SeededRng| step * f32::from(u8::try_from(rng.below(9)).unwrap());
    SinkPoint::new(at(rng), at(rng))
}

/// Points as `(x y)` pairs.
fn points(points: &[SinkPoint]) -> String {
    let pairs: Vec<String> = points
        .iter()
        .map(|p| format!("({} {})", p.x, p.y))
        .collect();
    pairs.join(" ")
}

/// A seeded k-means run on a grid whose step puts distances on the reach
/// limits: sinks, branching point and means, a capacity that sometimes
/// binds, a few rounds and a power; what the run leaves.
fn kmeans_case(rng: &mut SeededRng) -> Vec<String> {
    let step = rng.pick(&[50.0, 650.0, 1300.0]);
    let sinks: Vec<SinkPoint> = (0..=rng.below(11)).map(|_| grid_point(rng, step)).collect();
    let branching_point = grid_point(rng, step);
    let mut means = [grid_point(rng, step), grid_point(rng, step)];
    let cap = usize::try_from(1 + rng.below(sinks.len() as u64)).unwrap();
    let rounds = u32::try_from(rng.below(5)).unwrap();
    let power = rng.pick(&[1, 2, 4]);
    let header = format!(
        "sinks {} branching point {} means {} cap {cap} rounds {rounds} power {power}",
        points(&sinks),
        points(&[branching_point]),
        points(&means),
    );
    let mut clustering = Clustering::new(&sinks, branching_point);

    clustering.run(cap, rounds, power, &mut means);

    let silhouette = clustering.silhouette(&means);
    vec![
        header,
        format!(
            "  clusters {:?} means {} silhouette {silhouette}",
            clustering.clusters(),
            points(&means)
        ),
    ]
}

#[test]
fn seeded_branches_split_as_they_always_have() {
    let mut rng = SeededRng::new(43);
    let lines: Vec<String> = (0..400).flat_map(|_| kmeans_case(&mut rng)).collect();

    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden");
    assert_golden(&dir, "kmeans_splits.txt", &(lines.join("\n") + "\n"));
}

/// The clusters a run of `rounds` rounds at power one leaves.
fn clusters_after(
    sinks: &[(f32, f32)],
    branching_point: (f32, f32),
    means: [(f32, f32); 2],
    (cap, rounds): (usize, u32),
) -> Vec<Vec<usize>> {
    let sinks: Vec<SinkPoint> = sinks.iter().map(|&(x, y)| SinkPoint::new(x, y)).collect();
    let mut means = means.map(|(x, y)| SinkPoint::new(x, y));
    let mut clustering =
        Clustering::new(&sinks, SinkPoint::new(branching_point.0, branching_point.1));

    clustering.run(cap, rounds, 1, &mut means);

    clustering.clusters().to_vec()
}

#[test]
fn means_moving_exactly_half_a_unit_have_not_settled() {
    // The first round moves the means half a unit; a second one splits the sinks.
    let clusters = clusters_after(
        &[(4.0, 6.0), (4.0, 0.0)],
        (6.0, 3.0),
        [(0.0, 4.0), (4.0, 3.0)],
        (2, 1),
    );

    assert_eq!(clusters, [vec![1], vec![0]]);
}

#[test]
fn means_moving_less_than_half_a_unit_have_settled() {
    let sinks = [
        (1.5, 4.0),
        (1.5, 1.0),
        (1.5, 3.0),
        (1.5, 1.5),
        (1.5, 1.5),
        (1.0, 2.0),
    ];

    let clusters = clusters_after(&sinks, (1.0, 0.5), [(2.0, 2.0), (0.0, 0.0)], (2, 2));

    assert_eq!(clusters, [vec![0, 3, 4], vec![1, 2, 5]]);
}
