use super::*;

#[test]
fn truncating_division_rounds_toward_zero_and_floor_division_down() {
    assert_eq!(Dbu(7).div_trunc(Dbu(2)), 3);
    assert_eq!(Dbu(-7).div_trunc(Dbu(2)), -3);
    assert_eq!(Dbu(7).div_floor(Dbu(2)), 3);
    assert_eq!(Dbu(-7).div_floor(Dbu(2)), -4);
}

#[test]
fn ceiling_division_rounds_up_unless_exact() {
    assert_eq!(Dbu(7).div_ceil(Dbu(2)), 4);
    assert_eq!(Dbu(8).div_ceil(Dbu(2)), 4);
    assert_eq!(Dbu(0).div_ceil(Dbu(2)), 0);
    assert_eq!(Dbu(-7).div_ceil(Dbu(2)), -3);
}

#[test]
fn a_multiple_of_a_step_has_no_remainder_on_either_side_of_zero() {
    assert!(Dbu(-6).is_multiple_of(Dbu(3)));
    assert!(!Dbu(7).is_multiple_of(Dbu(3)));
}

#[test]
fn a_strip_runs_long_enough_to_cover_the_area_rounded_up() {
    assert_eq!(DbuArea(1400).div_ceil(Dbu(70)), Dbu(20));
    assert_eq!(DbuArea(1401).div_ceil(Dbu(70)), Dbu(21));
    assert_eq!(DbuArea(0).div_ceil(Dbu(70)), Dbu(0));
}

const fn on_grid<T>(value: T) -> Snapped<T> {
    Snapped {
        value,
        off_grid: false,
    }
}

const fn off_grid<T>(value: T) -> Snapped<T> {
    Snapped {
        value,
        off_grid: true,
    }
}

#[test]
fn decimal_micron_values_on_the_grid_convert_exactly() {
    let units = DbuPerMicron::new(2000).unwrap();

    assert_eq!(units.snap(0.095), on_grid(Dbu(190)));
    assert_eq!(units.snap(-0.035), on_grid(Dbu(-70)));
    assert_eq!(units.snap(296.2965), on_grid(Dbu(592_593)));
}

#[test]
fn small_and_large_values_off_the_grid_round_to_the_nearest_unit() {
    let units = DbuPerMicron::new(1000).unwrap();

    assert_eq!(units.snap(0.0004), off_grid(Dbu(0)));
    assert_eq!(units.snap(0.0006), off_grid(Dbu(1)));
    assert_eq!(units.snap(296.2961), off_grid(Dbu(296_296)));
    assert_eq!(units.snap(1000.0004), off_grid(Dbu(1_000_000)));
}

#[test]
fn a_value_half_way_between_units_rounds_away_from_zero() {
    let units = DbuPerMicron::new(2).unwrap();

    assert_eq!(units.snap(0.25), off_grid(Dbu(1)));
    assert_eq!(units.snap(-0.25), off_grid(Dbu(-1)));
}

#[test]
fn square_microns_round_to_the_nearest_square_unit() {
    let units = DbuPerMicron::new(2000).unwrap();

    assert_eq!(units.snap_area(0.0115), on_grid(DbuArea(46_000)));
    assert_eq!(units.snap_area(0.125_000_01), off_grid(DbuArea(500_000)));
}

#[test]
fn zero_resolution_is_rejected() {
    assert!(DbuPerMicron::new(0).is_none());
}

#[test]
fn remainder_pairs_with_truncating_division() {
    let (quotient, remainder) = (Dbu(-3) / 2, Dbu(-3) % 2);

    assert_eq!((quotient, remainder), (Dbu(-1), Dbu(-1)));
    assert_eq!(quotient * 2 + remainder, Dbu(-3));
}

#[test]
fn midpoint_rounds_down() {
    assert_eq!(Dbu(-3).midpoint(Dbu(0)), Dbu(-2));
    assert_eq!(Dbu(3).midpoint(Dbu(4)), Dbu(3));
}

#[test]
fn an_area_prints_as_its_number() {
    assert_eq!(DbuArea(42).to_string(), "42");
}

#[test]
fn the_remainder_keeps_the_sign_of_the_dividend_as_cs_does() {
    assert_eq!((Dbu(7) % 3, Dbu(-7) % 3), (Dbu(1), Dbu(-1)));
}

#[test]
fn an_area_and_a_length_convert_to_floats_and_microns() {
    let units = DbuPerMicron::new(1000).unwrap();

    assert!((DbuArea(12).to_f64() - 12.0).abs() < f64::EPSILON);
    assert!((units.to_microns(Dbu(250)) - 0.25).abs() < f64::EPSILON);
    assert_eq!(units.to_dbu_rounded(0.1234), Dbu(123));
    assert_eq!(units.to_dbu_rounded(-0.0005), Dbu(-1));
}
