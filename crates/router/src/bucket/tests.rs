use super::*;

#[test]
fn values_are_found_in_the_buckets_they_were_filed_in() {
    let mut hash = SpatialHash::new(Point::new(Dbu::new(100), Dbu::new(0)), Dbu::new(10));
    let key = hash.bucket(Point::new(Dbu::new(125), Dbu::new(5)));
    hash.insert(key, 'a');
    hash.insert(hash.bucket(Point::new(Dbu::new(145), Dbu::new(5))), 'b');

    assert_eq!(key, Point::new(Bucket(2), Bucket(0)));
    assert_eq!(hash.in_bucket(key), ['a']);
    assert_eq!(
        hash.between(key, Point::new(Bucket(4), Bucket(0))).count(),
        2
    );
    assert!(hash.in_bucket(Point::new(Bucket(3), Bucket(0))).is_empty());
}

#[test]
fn retaining_keeps_only_the_accepted_values_of_one_bucket() {
    let mut hash = SpatialHash::new(Point::new(Dbu::new(0), Dbu::new(0)), Dbu::new(10));
    let key = Point::new(Bucket(0), Bucket(0));
    hash.insert(key, 1);
    hash.insert(key, 2);

    hash.retain_in(key, |&v| v == 2);

    assert_eq!(hash.in_bucket(key), [2]);
}

#[test]
fn a_point_below_the_origin_lies_in_a_bucket_before_the_first() {
    let p = Point::new(Dbu::new(-1), Dbu::new(100));

    assert_eq!(
        bucket_at(p, Dbu::new(100)),
        Point::new(Bucket(-1), Bucket(1))
    );
}

#[test]
fn buckets_between_two_corners_come_column_by_column() {
    let all: Vec<Point<Bucket>> = buckets_between(
        Point::new(Bucket(0), Bucket(0)),
        Point::new(Bucket(1), Bucket(1)),
    )
    .collect();

    let at = |x, y| Point::new(Bucket(x), Bucket(y));
    assert_eq!(all, [at(0, 0), at(0, 1), at(1, 0), at(1, 1)]);
}
