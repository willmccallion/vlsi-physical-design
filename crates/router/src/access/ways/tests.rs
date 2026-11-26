use super::*;

#[test]
fn ways_round_trip_through_their_bits() {
    let ways = Ways::NONE.with(Way::AlongLower).with(Way::Up);

    assert_eq!(Ways::from_bits(ways.bits()), Some(ways));
    assert!(ways.contains(Way::Up) && !ways.contains(Way::Down));
}

#[test]
fn bits_beyond_the_six_ways_are_refused() {
    assert_eq!(Ways::from_bits(64), None);
}
