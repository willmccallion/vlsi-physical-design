use super::*;

#[test]
fn a_resistance_driving_a_capacitance_gives_a_time() {
    let tau = Ohms::new(1e3) * Farads::new(2e-15);

    assert!((tau.nanos() - 2e-3).abs() < 1e-15);
}

#[test]
fn a_time_over_a_capacitance_gives_a_resistance() {
    let resistance = Seconds::new(1e-9) / Farads::new(10e-15);

    assert!((resistance.get() - 1e5).abs() < 1e-6);
}

#[test]
fn a_wire_per_meter_converts_to_per_database_unit() {
    let dbu = DbuPerMicron::new(2000).unwrap();

    let res = OhmsPerMeter::new(4e6).per_dbu(dbu);
    let cap = FaradsPerMeter::new(2e-10).per_dbu(dbu);

    assert!((res.get() - 2e-3).abs() < 1e-15);
    assert!((cap.get() - 1e-19).abs() < 1e-30);
}

#[test]
fn a_length_in_database_units_converts_to_meters() {
    let length = Meters::from_dbu(Dbu::new(4000), DbuPerMicron::new(2000).unwrap());

    assert!((length.get() - 2e-6).abs() < 1e-18);
}

#[test]
fn a_length_converts_to_meters_as_openroad_rounds_it() {
    let length = Meters::from_dbu(Dbu::new(13), DbuPerMicron::new(2000).unwrap());

    assert_eq!(length.get().to_bits(), 6.5e-9_f64.to_bits());
}

#[test]
fn two_quantities_of_one_kind_divide_to_a_plain_ratio() {
    let ratio = Seconds::new(3e-9) / Seconds::new(2e-9);

    assert!((ratio - 1.5).abs() < 1e-12);
}

#[test]
fn a_capacitance_per_meter_reads_per_micron() {
    let per_micron = FaradsPerMeter::from_femtofarads_per_micron(0.5).per_micron();

    assert!((per_micron - 0.5e-15).abs() < 1e-27, "{per_micron}");
}

#[test]
fn a_bare_number_is_its_own_base_value_and_a_micron_rate_scales_to_meters() {
    assert!((2.5_f64.base() - 2.5).abs() < f64::EPSILON);
    assert!((OhmsPerMeter::from_ohms_per_micron(1.5).get() - 1.5e6).abs() < 1e-6);
}
