use super::*;

fn microns(value: Dbu, dbu: DbuPerMicron) -> String {
    MicronFormat::new(dbu).unwrap().microns(value)
}

#[test]
fn microns_are_exact_and_trimmed() {
    let dbu = DbuPerMicron::new(100).unwrap();
    assert_eq!(microns(Dbu::new(12_350), dbu), "123.5");
    assert_eq!(microns(Dbu::new(-5), dbu), "-0.05");
    assert_eq!(microns(Dbu::new(400), dbu), "4");
}

#[test]
fn microns_are_exact_at_a_resolution_other_than_a_power_of_ten() {
    let dbu = DbuPerMicron::new(2000).unwrap();
    assert_eq!(microns(Dbu::new(1000), dbu), "0.5");
    assert_eq!(microns(Dbu::new(-2001), dbu), "-1.0005");
}

#[test]
fn microns_are_exact_at_the_finest_binary_resolution() {
    let dbu = DbuPerMicron::new(1 << 31).unwrap();
    assert_eq!(
        microns(Dbu::new(1), dbu),
        "0.0000000004656612873077392578125"
    );
}

#[test]
fn a_resolution_without_exact_decimals_has_no_format() {
    assert!(MicronFormat::new(DbuPerMicron::new(3).unwrap()).is_none());
}

#[test]
fn zero_microns_have_no_sign() {
    assert_eq!(microns(Dbu::ZERO, DbuPerMicron::new(100).unwrap()), "0");
}
