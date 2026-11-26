crate::keyword_enum! {
    /// A test enum.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    enum Mode {
        /// The first.
        #[default]
        Fast = "FAST",
        Slow = "SLOW",
    }
}

#[test]
fn every_value_prints_as_the_keyword_it_parses_from() {
    let round_trip = Mode::ALL.map(|m| Mode::from_keyword(m.as_str()));

    assert_eq!(round_trip, [Some(Mode::Fast), Some(Mode::Slow)]);
}

#[test]
fn a_word_parses_only_as_written() {
    assert_eq!(
        [Mode::from_keyword("fast"), Mode::from_keyword("MEDIUM")],
        [None, None]
    );
}

#[test]
fn variant_attributes_carry_over() {
    assert_eq!(Mode::default(), Mode::Fast);
}
