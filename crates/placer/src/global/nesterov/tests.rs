use super::*;

#[test]
fn gamma_shrinks_from_forty_bins_to_under_one_as_overflow_falls() {
    let bin = Point::new(2.0, 2.0);

    assert!((gamma_for(1.0, bin) - 80.0).abs() < 1e-9);
    assert!((gamma_for(0.1, bin) - 0.8).abs() < 1e-9);
    assert!((gamma_for(0.01, bin) - 0.8).abs() < 1e-9);
}

#[test]
fn the_penalty_grows_fastest_while_hpwl_falls() {
    assert!((penalty_growth(-1.0, 100.0, 1.05) - 1.05).abs() < 1e-12);
    assert!((penalty_growth(100.0, 100.0, 1.05) - 1.0).abs() < 1e-12);
    assert!((penalty_growth(1e9, 100.0, 1.05) - 0.95).abs() < 1e-12);
}

#[test]
fn a_step_kept_whole_takes_one_trial() {
    let mut trials = 0;

    let step = backtrack_steps(1.0, |step| {
        trials += 1;
        step
    });

    assert_eq!((trials, step), (1, 1.0));
}

#[test]
fn a_step_that_keeps_shrinking_stops_after_gpl_s_ten_trials() {
    let mut trials = 0;

    let step = backtrack_steps(1.0, |step| {
        trials += 1;
        step / 2.0
    });

    assert_eq!((trials, step), (MAX_BACKTRACKS, 0.5_f64.powi(10)));
}

#[test]
fn a_round_gives_back_iterations_up_to_its_cap() {
    let mut refund = Refund::new(2);

    let taken = [refund.take(), refund.take(), refund.take()];

    assert_eq!(taken, [true, true, false]);
}

#[test]
fn each_routability_round_gives_back_its_own_cap() {
    let mut refund = Refund::new(2);
    let _ = (refund.take(), refund.take());

    refund.start_round();

    assert!(refund.take());
}

#[test]
fn the_growth_limit_is_lowered_once_as_overflow_falls() {
    let mut limit = GrowthLimit::new();

    limit.observe(0.5);
    let before = limit.highest;
    limit.observe(0.3);
    let after = limit.highest;
    limit.observe(0.2);

    assert!((before - 1.05).abs() < 1e-12);
    assert!((after - 1.0395).abs() < 1e-12);
    assert!((limit.highest - after).abs() < 1e-12);
}

fn evaluation(field_energy: Vec<f64>) -> Evaluation {
    Evaluation {
        wirelength: Wirelength::default(),
        overflow: 0.5,
        field_overflow: vec![0.5; field_energy.len()].into(),
        field_energy: field_energy.into(),
    }
}

#[test]
fn eplace_weights_grow_by_the_given_factor_except_when_frozen() {
    let mut weights = Weights::eplace(vec![2.0, 4.0].into());
    weights.frozen[FieldId(1)] = true;

    weights.update(&evaluation(vec![1.0, 1.0]), 1.05);

    assert!((weights.penalties[FieldId(0)] - 2.1).abs() < 1e-12);
    assert!((weights.penalties[FieldId(1)] - 4.0).abs() < 1e-12);
}

#[test]
fn a_scheduled_weight_steps_along_its_normalized_energy() {
    let mut weights = Weights {
        penalties: vec![1.0, 1.0].into(),
        frozen: vec![false, false].into(),
        schedule: Some(Schedule {
            precondition: vec![0.1, 0.05].into(),
            u: vec![1.0, 1.0].into(),
            step: 0.1,
            cap: f64::MAX,
            site: 1.0,
            kinds: vec![Precondition::Weighted; 2].into(),
            damping: 1.0,
        }),
    };

    weights.update(&evaluation(vec![10.0, 20.0]), 1.05);

    let u = 0.1f64.mul_add(std::f64::consts::FRAC_1_SQRT_2, 1.0);
    let penalty = u * 2001.0;
    assert!((weights.penalties[FieldId(0)] - penalty).abs() < 1e-9);
    assert!((weights.penalties[FieldId(1)] - penalty).abs() < 1e-9);
    let rate = (2000.0 * std::f64::consts::SQRT_2).ln();
    let growth = (rate / (1.0 + rate)).mul_add(0.01, 1.03);
    let expected_step = 0.1 * growth;
    let step = weights.schedule.map_or(0.0, |s| s.step);
    assert!((step - expected_step).abs() < 1e-12);
}

#[test]
fn the_quadratic_term_scales_a_scheduled_weight_by_the_energy_left() {
    let weights = Weights {
        penalties: vec![3.0].into(),
        frozen: vec![false].into(),
        schedule: Some(Schedule {
            precondition: vec![0.5].into(),
            u: vec![1.0].into(),
            step: 0.1,
            cap: f64::MAX,
            site: 1.0,
            kinds: vec![Precondition::Weighted].into(),
            damping: 1.0,
        }),
    };

    let expected = 3.0 * 1001.0;
    assert!((weights.multiplier(FieldId(0), 1.0) - expected).abs() < 1e-9);
    assert!((weights.multiplier(FieldId(0), 0.0) - 3.0).abs() < 1e-12);
}

fn scheduled(frozen: Vec<bool>) -> Weights {
    Weights {
        penalties: vec![5.0, 7.0].into(),
        frozen: frozen.into(),
        schedule: Some(Schedule {
            precondition: vec![0.5, 0.25].into(),
            u: vec![0.01, 0.02].into(),
            step: 0.1,
            cap: f64::MAX,
            site: 1.0,
            kinds: vec![Precondition::Weighted; 2].into(),
            damping: 1.0,
        }),
    }
}

#[test]
fn a_rescaled_schedule_reads_grown_energies_as_the_ungrown_ones() {
    let mut plain = scheduled(vec![false, false]);
    let mut grown = scheduled(vec![false, false]);

    plain.update(&evaluation(vec![2.0, 4.0]), 1.05);
    grown.rescale_energy(&vec![2.0, 4.0].into(), &vec![6.0, 10.0].into());
    grown.update(&evaluation(vec![6.0, 10.0]), 1.05);

    for (a, b) in plain.penalties.iter().zip(&grown.penalties) {
        assert!((a - b).abs() < 1e-9 * a.abs(), "{plain:?} vs {grown:?}");
    }
}

#[test]
fn rescaling_to_unchanged_energies_changes_nothing() {
    let mut weights = scheduled(vec![false, false]);
    let before = weights.schedule.as_ref().unwrap().precondition.clone();

    weights.rescale_energy(&vec![2.0, 4.0].into(), &vec![2.0, 4.0].into());

    assert_eq!(weights.schedule.as_ref().unwrap().precondition, before);
}

#[test]
fn a_frozen_field_keeps_its_energy_reference() {
    let mut weights = scheduled(vec![false, true]);

    weights.rescale_energy(&vec![2.0, 4.0].into(), &vec![6.0, 10.0].into());

    let s = weights.schedule.as_ref().unwrap();
    assert!((s.precondition[FieldId(0)] - 0.5 / 3.0).abs() < 1e-12);
    assert!((s.precondition[FieldId(1)] - 0.25).abs() < 1e-12);
}

fn shares(values: [f64; 2]) -> [Share; 2] {
    values.map(|v| Share::new(v).unwrap())
}

#[test]
fn timing_thresholds_fire_once_each_from_the_largest_down() {
    let thresholds = shares([0.64, 0.20]);
    let mut fired = [false, false];

    assert!(!timing_due(&thresholds, &mut fired, 0.70));
    assert!(timing_due(&thresholds, &mut fired, 0.63));
    assert_eq!(fired, [true, false]);
    assert!(!timing_due(&thresholds, &mut fired, 0.50));
    assert!(timing_due(&thresholds, &mut fired, 0.19));
    assert_eq!(fired, [true, true]);
    assert!(!timing_due(&thresholds, &mut fired, 0.05));
}

#[test]
fn an_overflow_under_every_threshold_fires_them_all_at_once() {
    let thresholds = shares([0.20, 0.64]);
    let mut fired = [false, false];

    assert!(timing_due(&thresholds, &mut fired, 0.10));
    assert_eq!(fired, [true, true]);
}

#[test]
fn remapped_weights_carry_fields_by_their_old_index() {
    let mut weights = Weights::eplace(vec![2.0, 4.0].into());
    weights.frozen[FieldId(1)] = true;

    weights.remap(&vec![Some(FieldId(1)), None, Some(FieldId(0))].into());

    assert_eq!(weights.penalties, IdVec::from(vec![4.0, 3.0, 2.0]));
    assert_eq!(weights.frozen, IdVec::from(vec![true, false, false]));
}

#[test]
fn a_large_object_is_kept_centered_on_a_small_die() {
    let die = Rect::new(Point::new(0.0, 0.0), Point::new(10.0, 10.0));

    let b = center_bounds(&die, Point::new(12.0, 4.0));

    assert_eq!((b.min.x, b.max.x), (5.0, 5.0));
    assert_eq!((b.min.y, b.max.y), (2.0, 8.0));
}
