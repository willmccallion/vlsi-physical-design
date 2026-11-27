use super::*;

#[test]
fn a_phase_shows_its_title_as_it_starts() {
    assert_eq!(Phase::DetailedPlacement.title(), Some("Detailed Placement"));
    assert_eq!(
        Phase::LegalizationAfterTimingRepair.title(),
        Some("Legalization")
    );
}

#[test]
fn a_phase_its_section_names_shows_no_title() {
    assert_eq!(Phase::GlobalPlacement.title(), None);
    assert_eq!(Phase::Routing.title(), None);
}

#[test]
fn a_finished_phase_is_recorded_under_its_name() {
    let mut report = Report::default();

    PhaseTimer::start(Phase::ClockTreeSynthesis).finish(&mut report);

    assert_eq!(report.phases.len(), 1);
    assert_eq!(report.phases[0].name, "clock tree synthesis");
}
