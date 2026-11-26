use super::*;
use pare_common::db::liberty::{Axis, AxisVariable, Edge, Table};

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
