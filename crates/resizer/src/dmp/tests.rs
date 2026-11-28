use super::*;
use pare_common::db::liberty::{Axis, AxisVariable, Edge, Table};
use pare_common::testing::assert_golden;
use std::cell::Cell;
use std::fmt::Write;
use std::path::{Path, PathBuf};

fn buffer_arc() -> TimingArc {
    let axes = vec![
        Axis {
            variable: AxisVariable::InputTransition,
            values: vec![1e-12, 50e-12, 200e-12],
        },
        Axis {
            variable: AxisVariable::OutputCapacitance,
            values: vec![1e-15, 10e-15, 60e-15],
        },
    ];
    TimingArc {
        from_edge: Edge::Rise,
        to_edge: Edge::Rise,
        delay: Some(Table {
            axes: axes.clone(),
            values: [
                15e-12, 30e-12, 120e-12, 20e-12, 35e-12, 125e-12, 40e-12, 55e-12, 145e-12,
            ]
            .map(Seconds::new)
            .to_vec(),
        }),
        slew: Some(Table {
            axes,
            values: [
                4e-12, 15e-12, 100e-12, 5e-12, 16e-12, 101e-12, 10e-12, 20e-12, 105e-12,
            ]
            .map(Seconds::new)
            .to_vec(),
        }),
    }
}

#[test]
fn fast_exponential_tracks_the_real_one() {
    for x in [-10.0, -3.0, -0.5, 0.0] {
        assert!((exp2(x) - x.exp()).abs() < 2e-3 * x.exp().max(1e-3));
    }
    assert!(exp2(-13.0) <= 0.0);
}

#[test]
fn a_lumped_load_is_the_table_value() {
    let arc = buffer_arc();
    let pi = PiModel {
        c2: Farads::new(20e-15),
        rpi: Ohms::ZERO,
        c1: Farads::ZERO,
    };

    let result = gate_delay(&arc, &Thresholds::default(), Seconds::new(10e-12), pi);

    let (delay, slew) = arc.gate_delay(Seconds::new(10e-12), Farads::new(20e-15));
    assert!((result.delay - delay).abs() < Seconds::new(1e-18));
    assert!((result.slew - slew).abs() < Seconds::new(1e-18));
    assert!((result.ceff - Farads::new(20e-15)).abs() < Farads::new(1e-30));
    assert_eq!(
        result.load_delay_slew(Seconds::new(3e-12)),
        (Seconds::new(3e-12), slew)
    );
}

#[test]
fn a_resistive_wire_lowers_the_effective_capacitance() {
    let arc = buffer_arc();
    let pi = PiModel {
        c2: Farads::new(5e-15),
        rpi: Ohms::new(2000.0),
        c1: Farads::new(40e-15),
    };

    let result = gate_delay(&arc, &Thresholds::default(), Seconds::new(10e-12), pi);

    assert!(result.ceff < Farads::new(45e-15));
    assert!(result.ceff > Farads::new(5e-15));
    let lumped_delay = arc.gate_delay(Seconds::new(10e-12), Farads::new(45e-15)).0;
    assert!(result.delay < lumped_delay);
    let (wire_delay, load_slew) = result.load_delay_slew(Seconds::new(40e-12));
    assert!(wire_delay > Seconds::ZERO);
    assert!(load_slew >= result.slew);
}

#[test]
fn newton_solves_a_linear_system() {
    let mut x = [0.0, 0.0];
    newton_raphson::<2>(&mut x, &|x, f, j| {
        f[0] = x[0] + x[1] - 5.0;
        f[1] = x[0] - x[1] - 1.0;
        *j = [[1.0, 1.0], [1.0, -1.0]];
        Ok(())
    })
    .unwrap();
    assert!((x[0] - 3.0).abs() < 1e-9 && (x[1] - 2.0).abs() < 1e-9);
}

#[test]
fn newton_steps_from_where_the_equations_move_the_iterate() {
    let mut x = [-1.0e6];
    let solved = newton_raphson::<1>(&mut x, &|x, f, j| {
        if x[0] <= 0.0 {
            x[0] = 1.0;
        }
        f[0] = x[0] - 2.0;
        j[0][0] = 1.0;
        Ok(())
    });

    assert_eq!(solved, Ok(()));
    assert!((x[0] - 2.0).abs() < 1e-9, "{x:?}");
}

#[test]
fn root_finding_brackets_and_converges() {
    let root = find_root(|t| (t * t - 2.0, 2.0 * t), 0.0, 2.0).unwrap();
    assert!((root - 2f64.sqrt()).abs() < 0.02);
    assert!(find_root(|t| (t + 1.0, 1.0), 0.0, 2.0).is_none());
}

fn golden_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}

/// A seeded generator of values in `[0, 1)`.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        f64::from(u32::try_from(self.0 >> 32).unwrap()) / 4_294_967_296.0
    }

    fn between(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.next()
    }
}

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * a.abs().max(b.abs())
}

fn common_for(arc: &TimingArc, in_slew: f64) -> Common<'_> {
    let thresholds = Thresholds::default();
    let edge = arc.to_edge;
    Common {
        arc,
        in_slew,
        vth: thresholds.output.get(edge),
        vl: thresholds.slew_lower.get(edge),
        vh: thresholds.slew_upper.get(edge),
        slew_derate: thresholds.slew_derate,
    }
}

fn pi_of(c2: f64, rpi: f64, c1: f64) -> Pi {
    Pi { c2, rpi, c1 }
}

/// `pi` driven through the buffer arc at a 10 ps input slew.
fn gate_of(pi: Pi) -> GateDelay {
    let model = PiModel {
        c2: Farads::new(pi.c2),
        rpi: Ohms::new(pi.rpi),
        c1: Farads::new(pi.c1),
    };
    gate_delay(
        &buffer_arc(),
        &Thresholds::default(),
        Seconds::new(10e-12),
        model,
    )
}

fn fit_of(delay: &GateDelay) -> &Fit {
    match &delay.model {
        LoadModel::Fitted(fit) => fit,
        LoadModel::Lumped => panic!("the driver was not fitted"),
    }
}

/// `value` in the golden files' fixed notation.
fn sci(value: f64) -> String {
    format!("{value:.9e}")
}

#[test]
fn the_fast_exponential_still_counts_at_minus_twelve() {
    assert!(exp2(-12.0) > 0.0);
    assert!(exp2(-12.001) <= 0.0);
}

#[test]
fn a_root_at_a_bound_is_returned_as_it_is() {
    assert_eq!(find_root(|t| (t, 1.0), 0.0, 2.0), Some(0.0));
    assert_eq!(find_root(|t| (-t, -1.0), 0.0, 2.0), Some(0.0));
    assert_eq!(find_root(|t| (t - 2.0, 1.0), 0.0, 2.0), Some(2.0));
    assert_eq!(find_root(|t| (2.0 - t, -1.0), 0.0, 2.0), Some(2.0));
}

#[test]
fn bounds_with_the_same_sign_bracket_no_root() {
    assert!(find_root(|t| (t + 1.0, 1.0), 0.0, 2.0).is_none());
    assert!(find_root(|t| (t - 3.0, 1.0), 0.0, 2.0).is_none());
}

#[test]
fn a_falling_function_has_its_root_found_too() {
    let root = find_root(|t| (2.0 - t * t, -2.0 * t), 0.0, 2.0).unwrap();

    assert!((root - 2f64.sqrt()).abs() < 0.02, "{root}");
}

#[test]
fn a_root_search_that_never_settles_gives_up() {
    let evaluations = Cell::new(0);

    let root = find_root(
        |t| {
            evaluations.set(evaluations.get() + 1);
            (t - 1.0, 1.0e-300)
        },
        0.0,
        1.0e9,
    );

    assert!(root.is_none());
    assert_eq!(evaluations.get(), 2 + 1 + FIND_ROOT_MAX_ITER);
}

/// A seeded function with a root in `[0, 10)` and a bracket around it.
fn seeded_root_case(random: &mut Lcg) -> (u32, f64, f64, f64, f64) {
    let kind = u32::try_from(random.0 >> 62).unwrap();
    let root = random.between(1.0, 9.0);
    let scale = random.between(0.2, 5.0);
    let low = root - random.between(0.5, 3.0);
    let high = root + random.between(0.5, 3.0);
    (kind, root, scale, low, high)
}

fn seeded_function(kind: u32, root: f64, scale: f64) -> impl Fn(f64) -> (f64, f64) {
    move |t| match kind {
        0 => (scale * (t - root), scale),
        1 => (
            scale * (t - root).powi(3) + 0.01 * (t - root),
            3.0 * scale * (t - root).powi(2) + 0.01,
        ),
        2 => (scale * (t - root).exp_m1(), scale * (t - root).exp()),
        _ => (
            scale * (root - t) * (1.0 + 0.1 * t),
            scale * (0.1 * root - 0.2 * t - 1.0),
        ),
    }
}

#[test]
fn root_finding_over_seeded_functions_is_as_it_was() {
    let mut random = Lcg(7);
    let mut text = String::new();
    for _ in 0..200 {
        let (kind, root, scale, low, high) = seeded_root_case(&mut random);
        let evaluations = Cell::new(0);
        let function = seeded_function(kind, root, scale);
        let found = find_root(
            |t| {
                evaluations.set(evaluations.get() + 1);
                function(t)
            },
            low,
            high,
        );
        let found = found.map_or_else(|| "none".to_string(), sci);
        writeln!(
            text,
            "{kind} {root:.4} {low:.4} {high:.4}: {found} in {}",
            evaluations.get()
        )
        .unwrap();
    }

    assert_golden(&golden_dir(), "a_dmp_find_root.txt", &text);
}

#[test]
fn seeded_roots_are_found_within_their_tolerance() {
    let mut random = Lcg(11);
    for _ in 0..200 {
        let (kind, root, scale, low, high) = seeded_root_case(&mut random);
        let function = seeded_function(kind, root, scale);
        if let Some(found) = find_root(&function, low, high) {
            assert!((found - root).abs() <= 0.02 * root, "{kind} {root} {found}");
        }
    }
}

#[test]
fn a_pivot_is_found_below_a_zero_on_the_diagonal() {
    let mut a = [[0.0, 1.0], [1.0, 0.0]];
    let mut b = [2.0, 3.0];

    let index = lu_decompose(&mut a).unwrap();
    lu_solve(&a, &index, &mut b);

    assert!(close(b[0], 3.0, 1e-12) && close(b[1], 2.0, 1e-12), "{b:?}");
}

#[test]
fn a_matrix_with_an_empty_row_has_no_decomposition() {
    let mut a = [[1.0, 2.0], [0.0, 0.0]];

    assert_eq!(lu_decompose(&mut a), Err(DmpError));
}

#[test]
fn seeded_systems_are_solved_to_their_residual() {
    let mut random = Lcg(3);
    for _ in 0..100 {
        let a: [[f64; 3]; 3] =
            std::array::from_fn(|_| std::array::from_fn(|_| random.between(-5.0, 5.0)));
        let wanted: [f64; 3] = std::array::from_fn(|_| random.between(-5.0, 5.0));
        let mut b: [f64; 3] = std::array::from_fn(|i| (0..3).map(|j| a[i][j] * wanted[j]).sum());
        let mut lu = a;

        let index = lu_decompose(&mut lu).unwrap();
        lu_solve(&lu, &index, &mut b);

        for i in 0..3 {
            assert!(
                (b[i] - wanted[i]).abs() < 1e-6 * (1.0 + wanted[i].abs()),
                "{a:?} {b:?}"
            );
        }
    }
}

#[test]
fn lu_decomposition_of_seeded_matrices_is_as_it_was() {
    let mut random = Lcg(5);
    let mut text = String::new();
    for _ in 0..60 {
        let mut a: [[f64; 3]; 3] =
            std::array::from_fn(|_| std::array::from_fn(|_| random.between(-4.0, 4.0)));
        a[1][0] = 0.0;
        let index = lu_decompose(&mut a).unwrap();
        let rows: Vec<String> = a.iter().flatten().map(|&v| sci(v)).collect();
        writeln!(text, "{index:?} {}", rows.join(" ")).unwrap();
    }

    assert_golden(&golden_dir(), "a_dmp_lu.txt", &text);
}

#[test]
fn newton_stops_on_the_first_step_inside_its_tolerance() {
    let evaluations = Cell::new(0);
    let mut x = [100.0];

    let solved = newton_raphson::<1>(&mut x, &|x, f, j| {
        evaluations.set(evaluations.get() + 1);
        f[0] = x[0] - 99.0;
        j[0][0] = 1.0;
        Ok(())
    });

    assert_eq!(solved, Ok(()));
    assert_eq!(evaluations.get(), 2);
}

#[test]
fn gate_delays_scale_the_table_slew_by_the_derate() {
    let arc = buffer_arc();
    let mut common = common_for(&arc, 10e-12);
    common.slew_derate = 0.5;

    let (t_vth, t_vl, slew) = common.gate_delays(20e-15);

    let (delay, table_slew) = arc.gate_delay(Seconds::new(10e-12), Farads::new(20e-15));
    assert!(close(t_vth, delay.get(), 1e-12));
    assert!(close(slew, 0.5 * table_slew.get(), 1e-12));
    // The slew spans 0.2 to 0.8 of the supply, the crossing at 0.5.
    assert!(
        close(t_vth - t_vl, slew * 0.5, 1e-9),
        "{t_vth} {t_vl} {slew}"
    );
}

#[test]
fn the_ramp_response_of_a_capacitive_load_follows_its_time_constant() {
    let (rd, cl, tau) = (1.0e3, 1.0e-14, 1.0e-11);
    let (t0, dt) = (1.0e-11, 2.0e-11);
    let y0 = |t: f64| t - tau * (1.0 - (-t / tau).exp());

    assert!(y(t0 - 1e-12, t0, dt, rd, cl).abs() < f64::EPSILON);
    assert!(close(y(t0 + 1e-11, t0, dt, rd, cl), y0(1e-11) / dt, 3e-3));
    assert!(close(
        y(t0 + 3e-11, t0, dt, rd, cl),
        (y0(3e-11) - y0(1e-11)) / dt,
        3e-3
    ));
    assert!(close(y(t0 + 1e-9, t0, dt, rd, cl), 1.0, 3e-3));
}

#[test]
fn the_partials_of_the_ramp_response_match_its_differences() {
    let (rd, cl, t0, dt) = (1.0e3, 1.0e-14, 1.0e-11, 2.0e-11);
    for t in [1.5e-11, 2.5e-11, 3.5e-11, 6.0e-11] {
        let partials = dy(t, t0, dt, rd, cl);
        let step = |f: &dyn Fn(f64) -> f64, at: f64| {
            let h = at * 1e-4;
            (f(at + h) - f(at - h)) / (2.0 * h)
        };
        let by_t0 = step(&|v| y(t, v, dt, rd, cl), t0);
        let by_cl = step(&|v| y(t, t0, dt, rd, v), cl);

        assert!(
            close(partials[0], by_t0, 1e-2),
            "t0 at {t}: {} {by_t0}",
            partials[0]
        );
        assert!(
            close(partials[2], by_cl, 1e-2),
            "cl at {t}: {} {by_cl}",
            partials[2]
        );
        let by_dt = step(&|v| y(t, t0, v, rd, cl), dt);
        assert!(
            close(partials[1], by_dt, 1e-2),
            "dt at {t}: {} {by_dt}",
            partials[1]
        );
    }
}

#[test]
fn nothing_has_moved_before_the_ramp_starts() {
    let partials = dy(1.0e-11, 2.0e-11, 1.0e-11, 1.0e3, 1.0e-14);

    assert!(partials.iter().all(|p| p.abs() < f64::MIN_POSITIVE));
}

/// A driver over a pi with almost no near capacitance, and its resistive
/// and capacitive load.
const ZERO_C2: (f64, f64, f64) = (1.0e-18, 2000.0, 40.0e-15);

#[test]
fn the_response_of_a_driver_without_near_capacitance_is_a_charging_capacitor() {
    let (c2, rpi, c1) = ZERO_C2;
    let delay = gate_of(pi_of(c2, rpi, c1));
    let Response::ZeroC2 { p1, .. } = fit_of(&delay).response else {
        panic!("not the zero-c2 response");
    };
    let rd = common_for(&buffer_arc(), 10e-12).gate_model_rd(c2, c1);
    let charge = rd * c1;
    assert!(close(p1, 1.0 / (c1 * (rd + rpi)), 1e-12));

    for t in [0.0, 1e-11, 5e-11, 2e-10] {
        let (v, dv) = fit_of(&delay).response.v0(t);

        let decay = (-p1 * t).exp();
        assert!(
            (v - (t - charge * (1.0 - decay))).abs() < 2e-3 * (t + charge),
            "{t}: {v}"
        );
        assert!((dv - (1.0 - charge * p1 * decay)).abs() < 2e-3, "{t}: {dv}");
    }
}

#[test]
fn the_load_of_a_driver_without_near_capacitance_follows_through_its_elmore_pole() {
    let (c2, rpi, c1) = ZERO_C2;
    let delay = gate_of(pi_of(c2, rpi, c1));
    let Response::ZeroC2 { p1, .. } = fit_of(&delay).response else {
        panic!("not the zero-c2 response");
    };
    let rd = common_for(&buffer_arc(), 10e-12).gate_model_rd(c2, c1);
    let charge = rd * c1;
    let p3 = 1.0 / 30e-12;

    for t in [0.0, 1e-11, 5e-11, 2e-10] {
        let (v, dv) = fit_of(&delay).response.vl0(t, p3);

        let (e1, e3) = ((-p1 * t).exp(), (-p3 * t).exp());
        let mix = charge * p3 / (p3 - p1);
        let expected = t - (1.0 - e3) / p3 - charge * (1.0 - e3) + mix * (e1 - e3);
        let slope = 1.0 - e3 - charge * p3 * e3 + mix * (p3 * e3 - p1 * e1);
        assert!(
            (v - expected).abs() < 2e-3 * (t + 1.0 / p3),
            "{t}: {v} {expected}"
        );
        assert!((dv - slope).abs() < 2e-3, "{t}: {dv} {slope}");
    }
}

#[test]
fn a_fitted_driver_reaches_its_thresholds_at_the_times_its_table_gives() {
    let rd = common_for(&buffer_arc(), 10e-12).gate_model_rd(5e-15, 40e-15);
    for pi in [
        pi_of(5e-15, 2000.0, 40e-15),
        pi_of(ZERO_C2.0, ZERO_C2.1, ZERO_C2.2),
    ] {
        let delay = gate_of(pi);
        let fit = fit_of(&delay);
        let arc = buffer_arc();
        let common = common_for(&arc, 10e-12);
        let (t_vth, t_vl, _) = common.gate_delays(delay.ceff.get());

        let at_vth = y(t_vth, fit.t0, fit.dt, rd, delay.ceff.get());
        let at_vl = y(t_vl, fit.t0, fit.dt, rd, delay.ceff.get());

        assert!((at_vth - 0.5).abs() < 0.02, "{at_vth}");
        assert!((at_vl - 0.2).abs() < 0.02, "{at_vl}");
    }
}

#[test]
fn the_driver_slew_is_measured_between_the_slew_thresholds_and_derated() {
    let delay = gate_of(pi_of(5e-15, 2000.0, 40e-15));
    let mut fit = fit_of(&delay).clone();
    let (_, slew) = fit.driver_delay_slew().unwrap();

    fit.slew_derate = 0.5;
    let (_, derated) = fit.driver_delay_slew().unwrap();

    assert!(close(derated, 2.0 * slew, 1e-12), "{slew} {derated}");
}

#[test]
fn the_driver_ramp_is_searched_out_to_the_load_time_constants() {
    let rd = common_for(&buffer_arc(), 10e-12).gate_model_rd(5e-15, 40e-15);
    let (c2, rpi, c1) = (5e-15, 2000.0, 40e-15);
    let bound = |fit: &Fit, near: f64, far: f64| fit.t0 + fit.dt + (far + near) * (rd + rpi) * 2.0;

    let delay = gate_of(pi_of(c2, rpi, c1));
    assert!(close(
        fit_of(&delay).vo_upper_bound,
        bound(fit_of(&delay), c2, c1),
        1e-9
    ));

    let (c2, rpi, c1) = ZERO_C2;
    let rd = common_for(&buffer_arc(), 10e-12).gate_model_rd(c2, c1);
    let delay = gate_of(pi_of(c2, rpi, c1));
    let fit = fit_of(&delay);
    assert!(close(
        fit.vo_upper_bound,
        fit.t0 + fit.dt + c1 * (rd + rpi) * 2.0,
        1e-9
    ));
}

#[test]
fn a_small_near_capacitance_behind_a_larger_far_one_is_still_fitted() {
    let delay = gate_of(pi_of(1e-15, 2000.0, 40e-15));

    assert!(matches!(delay.model, LoadModel::Fitted(_)));
    let delay = gate_of(pi_of(40e-15, 2000.0, 1e-15));
    assert!(matches!(delay.model, LoadModel::Fitted(_)));
}

#[test]
fn a_resistive_load_is_fitted_not_taken_as_lumped() {
    let delay = gate_of(pi_of(5e-15, 2000.0, 40e-15));

    assert!(matches!(delay.model, LoadModel::Fitted(_)));
    assert!(delay.slew > Seconds::new(80e-12));
}

/// What the fit makes of `pi` where it is neither lumped nor of vanishing
/// near capacitance.
fn pi_fit_of(pi: Pi) -> RawDelay {
    let arc = buffer_arc();
    let common = common_for(&arc, 10e-12);
    pi_model(&common, common.gate_model_rd(pi.c2, pi.c1), pi)
}

fn lumped_of(pi: Pi) -> RawDelay {
    let arc = buffer_arc();
    lumped(&common_for(&arc, 10e-12), pi.c1 + pi.c2)
}

fn ceff_of(pi: Pi) -> f64 {
    gate_of(pi).ceff.get()
}

#[test]
fn a_wire_as_resistive_as_a_thousandth_of_the_driver_is_fitted() {
    let arc = buffer_arc();
    let rd = common_for(&arc, 10e-12).gate_model_rd(5e-15, 40e-15);
    let pi = pi_of(5e-15, rd * 1e-3, 40e-15);

    assert!(close(ceff_of(pi), pi_fit_of(pi).ceff, 1e-12));
    assert!(!close(ceff_of(pi), lumped_of(pi).ceff, 1e-9));
}

#[test]
fn a_wire_below_a_thousandth_of_the_driver_is_lumped() {
    let arc = buffer_arc();
    let rd = common_for(&arc, 10e-12).gate_model_rd(5e-15, 40e-15);
    let pi = pi_of(5e-15, rd * 1e-4, 40e-15);

    assert!(close(ceff_of(pi), 45e-15, 1e-12));
    assert!(matches!(gate_of(pi).model, LoadModel::Lumped));
}

#[test]
fn a_far_capacitance_of_a_thousandth_of_the_near_one_is_fitted() {
    let c2 = 5e-15;
    let pi = pi_of(c2, 2000.0, c2 * 1e-3);

    assert!(close(ceff_of(pi), pi_fit_of(pi).ceff, 1e-12));
    assert!(!close(ceff_of(pi), lumped_of(pi).ceff, 1e-9));
}

#[test]
fn a_far_capacitance_below_a_thousandth_of_the_near_one_is_lumped() {
    let c2 = 5e-15;
    let pi = pi_of(c2, 2000.0, c2 * 1e-4);

    assert!(close(ceff_of(pi), pi.c1 + pi.c2, 1e-12));
    assert!(matches!(gate_of(pi).model, LoadModel::Lumped));
}

#[test]
fn a_near_capacitance_of_a_thousandth_of_the_far_one_is_fitted_as_a_pi() {
    let c1 = 40e-15;
    let pi = pi_of(c1 * 1e-3, 2000.0, c1);

    assert!(close(ceff_of(pi), pi_fit_of(pi).ceff, 1e-12));
    assert!(!close(ceff_of(pi), c1, 1e-9));
}

#[test]
fn a_near_capacitance_below_a_thousandth_of_the_far_one_takes_the_far_one_as_effective() {
    let pi = pi_of(ZERO_C2.0, ZERO_C2.1, ZERO_C2.2);

    assert!(close(ceff_of(pi), pi.c1, 1e-15));
    assert!(matches!(
        gate_of(pi).model,
        LoadModel::Fitted(Fit {
            response: Response::ZeroC2 { .. },
            ..
        })
    ));
}

#[test]
fn a_driver_of_almost_no_resistance_sees_the_load_lumped() {
    let arc = linear_arc(30e-12, 7.2e-16, 10e-12, 10e-12);
    let common = common_for(&arc, 10e-12);
    assert!(common.gate_model_rd(5e-15, 40e-15) < 1e-2);
    let pi = PiModel {
        c2: Farads::new(5e-15),
        rpi: Ohms::new(2000.0),
        c1: Farads::new(40e-15),
    };

    let delay = gate_delay(&arc, &Thresholds::default(), Seconds::new(10e-12), pi);

    assert_eq!(delay.ceff, Farads::new(45e-15));
    assert!(matches!(delay.model, LoadModel::Lumped));
}

/// An arc whose delay and slew each start at their first value, grow by
/// their range over the table's loads of 1 to 100 fF, and are half as big
/// again at the table's slowest input.
fn linear_arc(delay: f64, delay_range: f64, slew: f64, slew_range: f64) -> TimingArc {
    let axes = vec![
        Axis {
            variable: AxisVariable::InputTransition,
            values: vec![1e-12, 200e-12],
        },
        Axis {
            variable: AxisVariable::OutputCapacitance,
            values: vec![1e-15, 100e-15],
        },
    ];
    let table = |first: f64, range: f64| Table {
        axes: axes.clone(),
        values: [first, first + range, first * 1.5, first * 1.5 + range]
            .map(Seconds::new)
            .to_vec(),
    };
    TimingArc {
        from_edge: Edge::Rise,
        to_edge: Edge::Rise,
        delay: Some(table(delay, delay_range)),
        slew: Some(table(slew, slew_range)),
    }
}

/// A seeded input slew, pi model and point to evaluate the fit equations at.
fn seeded_equations(random: &mut Lcg) -> (f64, Pi, [f64; 3]) {
    let in_slew = random.between(2e-12, 100e-12);
    let pi = pi_of(
        random.between(1e-15, 20e-15),
        random.between(300.0, 5000.0),
        random.between(5e-15, 60e-15),
    );
    let arc = buffer_arc();
    let common = common_for(&arc, in_slew);
    let rd = common.gate_model_rd(pi.c2, pi.c1);
    let start = initial_params(&common, rd, pi.c1 + pi.c2);
    let x = [
        start.t0 * random.between(0.7, 1.3),
        start.dt * random.between(0.7, 1.3),
        (pi.c1 + pi.c2) * random.between(0.2, 1.0),
    ];
    (in_slew, pi, x)
}

fn evaluated_equations(
    pi: Pi,
    in_slew: f64,
    x: [f64; 3],
) -> (Result<(), DmpError>, [f64; 3], [[f64; 3]; 3]) {
    let arc = buffer_arc();
    let common = common_for(&arc, in_slew);
    let rd = common.gate_model_rd(pi.c2, pi.c1);
    let poles = PiPoles::new(rd, pi);
    let (mut fvec, mut fjac) = ([0.0; 3], [[0.0; 3]; 3]);
    let outcome = poles.equations(&common, rd, pi, &x, &mut fvec, &mut fjac);
    (outcome, fvec, fjac)
}

#[test]
fn the_fit_equations_of_seeded_drivers_are_as_they_were() {
    let mut random = Lcg(13);
    let mut text = String::new();
    for _ in 0..40 {
        let (in_slew, pi, x) = seeded_equations(&mut random);
        let (outcome, fvec, fjac) = evaluated_equations(pi, in_slew, x);
        let numbers: Vec<String> = fvec
            .iter()
            .chain(fjac.iter().flatten())
            .map(|&v| sci(v))
            .collect();
        writeln!(text, "{outcome:?} {}", numbers.join(" ")).unwrap();
    }

    assert_golden(&golden_dir(), "a_dmp_equations.txt", &text);
}

#[test]
fn the_fit_equations_leave_the_domain_only_on_a_negative_capacitance_a_surplus_or_no_ramp() {
    let pi = pi_of(5e-15, 2000.0, 40e-15);
    let (t0, dt) = (1.0e-11, 1.5e-11);

    assert_eq!(
        evaluated_equations(pi, 10e-12, [t0, dt, -1e-18]).0,
        Err(DmpError)
    );
    assert_eq!(
        evaluated_equations(pi, 10e-12, [t0, dt, 45.1e-15]).0,
        Err(DmpError)
    );
    assert_eq!(
        evaluated_equations(pi, 10e-12, [t0, 0.0, 20e-15]).0,
        Err(DmpError)
    );
    assert_eq!(evaluated_equations(pi, 10e-12, [t0, dt, 20e-15]).0, Ok(()));
    assert_eq!(evaluated_equations(pi, 10e-12, [t0, dt, 0.0]).0, Ok(()));
}

#[test]
fn gate_and_load_delays_of_seeded_loads_are_as_they_were() {
    let mut random = Lcg(17);
    let mut text = String::new();
    for _ in 0..80 {
        let near = if random.next() < 0.3 {
            random.between(1e-18, 1e-16)
        } else {
            random.between(1e-15, 30e-15)
        };
        let pi = pi_of(
            near,
            random.between(100.0, 6000.0),
            random.between(2e-15, 70e-15),
        );
        let delay = gate_of(pi);
        let wire: Vec<String> = [5e-12, 40e-12, 150e-12]
            .iter()
            .map(|&elmore| {
                let (d, s) = delay.load_delay_slew(Seconds::new(elmore));
                format!("{} {}", sci(d.get()), sci(s.get()))
            })
            .collect();
        writeln!(
            text,
            "{} {} {} | {} {} {} | {}",
            sci(pi.c2),
            sci(pi.rpi),
            sci(pi.c1),
            sci(delay.delay.get()),
            sci(delay.slew.get()),
            sci(delay.ceff.get()),
            wire.join(" | ")
        )
        .unwrap();
    }

    assert_golden(&golden_dir(), "a_dmp_gate_delays.txt", &text);
}

fn resistive_fit() -> (GateDelay, Fit) {
    let delay = gate_of(pi_of(5e-15, 2000.0, 40e-15));
    let fit = fit_of(&delay).clone();
    (delay, fit)
}

#[test]
fn a_load_with_no_wire_delay_to_speak_of_gets_the_driver_slew() {
    let (delay, _) = resistive_fit();
    let slew = delay.slew;

    let at_the_limit = Seconds::new(slew.get() * 1e-3);
    let (_, below) = delay.load_delay_slew(Seconds::new(at_the_limit.get() * 0.99));
    let (wire, _) = delay.load_delay_slew(at_the_limit);

    assert_eq!(below, slew);
    assert_ne!(wire, at_the_limit, "an Elmore delay at the limit is fitted");
}

#[test]
fn a_load_that_crosses_before_the_driver_has_the_elmore_delay() {
    let (delay, mut fit) = resistive_fit();
    let elmore = 40e-12;
    fit.vo_delay = 1.0e-6;

    let (wire, _) = fit.load_delay_slew(elmore, delay.slew.get());

    assert!(close(wire, elmore, 1e-12));
}

#[test]
fn a_load_that_crosses_with_the_driver_has_no_delay_at_all() {
    let (delay, mut fit) = resistive_fit();
    let elmore = 40e-12;
    fit.vo_delay = 0.0;
    let (crossing, _) = fit.load_delay_slew(elmore, delay.slew.get());

    fit.vo_delay = crossing;
    let (wire, _) = fit.load_delay_slew(elmore, delay.slew.get());

    assert!(
        crossing > 0.0 && wire.abs() < f64::EPSILON,
        "{crossing} {wire}"
    );
}

#[test]
fn a_load_is_no_sharper_than_its_driver() {
    let (_, fit) = resistive_fit();
    let driver_slew = 1.0e-9;

    let (_, slew) = fit.load_delay_slew(40e-12, driver_slew);

    assert!(close(slew, driver_slew, 1e-12));
}

#[test]
fn the_load_slew_is_derated_like_the_driver_slew() {
    let (_, mut fit) = resistive_fit();
    let (_, slew) = fit.load_delay_slew(100e-12, 1e-12);

    fit.slew_derate = 0.5;
    let (_, derated) = fit.load_delay_slew(100e-12, 1e-12);

    assert!(slew > 1e-12);
    assert!(close(derated, 2.0 * slew, 1e-12), "{slew} {derated}");
}

#[test]
fn a_zero_c2_fit_the_search_drives_to_no_width_falls_back_to_the_far_capacitance() {
    let arc = linear_arc(85e-12, 257.4e-12, 39e-12, 49.5e-12);
    let (in_slew, c1) = (149e-12, 184e-15);
    let pi = PiModel {
        c2: Farads::new(1e-18),
        rpi: Ohms::new(500.0),
        c1: Farads::new(c1),
    };

    let delay = gate_delay(&arc, &Thresholds::default(), Seconds::new(in_slew), pi);

    let (table_delay, table_slew) = common_for(&arc, in_slew).table(c1);
    assert!(matches!(delay.model, LoadModel::Lumped));
    assert_eq!(
        (delay.delay.get(), delay.slew.get(), delay.ceff.get()),
        (table_delay, table_slew, c1)
    );
}
