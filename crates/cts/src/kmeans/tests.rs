use super::*;

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
