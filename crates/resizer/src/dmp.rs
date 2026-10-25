//! Gate and wire delays by the Dartu, Menezes and Pileggi effective
//! capacitance method, as `OpenSTA`'s default `dmp_ceff_elmore` calculator.
//!
//! The driver is fitted as a ramp of start `t0` and width `dt` behind a
//! resistance `rd` driving the pi model of its wire; the table lookup then
//! takes the effective capacitance the fit finds, and each load's slew comes
//! from the ramp's response through the wire's Elmore delay. The cases a
//! pi element vanishes in are handled apart, as the original does.

// The formulas keep OpenSTA's grouping so they can be checked against it.
#![allow(clippy::suboptimal_flops, clippy::suspicious_operation_groupings)]

use pare_common::db::liberty::{Thresholds, TimingArc};

const DRIVER_PARAM_TOL: f64 = 0.01;
const VTH_TIME_TOL: f64 = 0.01;
const TINY: f64 = 1.0e-20;
const FIND_ROOT_MAX_ITER: usize = 20;
const NEWTON_MAX_ITER: usize = 100;

/// The pi model of a driver's load.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PiModel {
    /// Capacitance at the driver, in farads.
    pub c2: f64,
    /// Resistance to the far capacitance, in ohms.
    pub rpi: f64,
    /// Capacitance beyond the resistance, in farads.
    pub c1: f64,
}

/// What the calculator found for a driver.
#[derive(Clone, Debug, PartialEq)]
pub struct GateDelay {
    /// Delay through the gate, in seconds.
    pub delay: f64,
    /// Slew at the driver, in seconds.
    pub slew: f64,
    /// The effective capacitance the lookup used, in farads.
    pub ceff: f64,
    model: LoadModel,
}

impl GateDelay {
    /// Wire delay and slew at a load with Elmore delay `elmore` from the
    /// driver, in seconds.
    #[must_use]
    pub fn load_delay_slew(&self, elmore: f64) -> (f64, f64) {
        self.model.load_delay_slew(elmore, self.slew)
    }
}

/// Fits the driver and looks the arc up, as `DmpCeffDelayCalc::gateDelay`.
#[must_use]
pub fn gate_delay(
    arc: &TimingArc,
    thresholds: &Thresholds,
    in_slew: f64,
    pi: PiModel,
) -> GateDelay {
    let edge = arc.to_edge;
    let vth = thresholds.output.get(edge);
    let vl = thresholds.slew_lower.get(edge);
    let vh = thresholds.slew_upper.get(edge);
    let common = Common {
        arc,
        in_slew,
        vth,
        vl,
        vh,
        slew_derate: thresholds.slew_derate,
    };
    let PiModel { c2, rpi, c1 } = pi;
    let rd = common.gate_model_rd(c2, c1);
    let lumped = rd < 1e-2 || rpi < rd * 1e-3 || c1 == 0.0 || c1 < c2 * 1e-3 || rpi == 0.0;
    if lumped {
        let ceff = c1 + c2;
        let (delay, slew) = common.table(ceff);
        return GateDelay {
            delay,
            slew,
            ceff,
            model: LoadModel::Lumped,
        };
    }
    if c2 < c1 * 1e-3 {
        return zero_c2(&common, rd, pi);
    }
    pi_model(&common, rd, pi)
}

/// What every DMP variant shares.
struct Common<'a> {
    arc: &'a TimingArc,
    in_slew: f64,
    vth: f64,
    vl: f64,
    vh: f64,
    slew_derate: f64,
}

impl Common<'_> {
    /// Table delay and slew at a capacitive load.
    fn table(&self, load: f64) -> (f64, f64) {
        self.arc.gate_delay(self.in_slew, load)
    }

    /// The driver's resistance implied by the delay tables' slope with
    /// load, as `gateModelRd`.
    fn gate_model_rd(&self, c2: f64, c1: f64) -> f64 {
        let cap1 = c1 + c2;
        let cap2 = cap1 + 1e-15;
        let (d1, _) = self.table(cap1);
        let (d2, _) = self.table(cap2);
        -self.vth.ln() * (d1 - d2).abs() / (cap2 - cap1)
    }

    /// Threshold crossing time, lower slew threshold time and measured slew
    /// at `ceff`, as `DmpAlg::gateDelays`.
    fn gate_delays(&self, ceff: f64) -> (f64, f64, f64) {
        let (t_vth, table_slew) = self.table(ceff);
        let slew = table_slew * self.slew_derate;
        let t_vl = t_vth - slew * (self.vth - self.vl) / (self.vh - self.vl);
        (t_vth, t_vl, slew)
    }
}

/// How a load's delay and slew follow from the driver fit.
#[derive(Clone, Debug, PartialEq)]
enum LoadModel {
    /// The driver saw a lumped load: wire delay is the Elmore delay and the
    /// load slew the driver's.
    Lumped,
    /// A fitted ramp through a pi model.
    Fitted(Fit),
}

impl LoadModel {
    fn load_delay_slew(&self, elmore: f64, drvr_slew: f64) -> (f64, f64) {
        match self {
            Self::Lumped => (elmore, drvr_slew),
            Self::Fitted(fit) => fit.load_delay_slew(elmore, drvr_slew),
        }
    }
}

/// The response of a pi model (or its degenerate forms) to a unit ramp.
#[derive(Clone, Debug, PartialEq)]
enum Response {
    Pi {
        p1: f64,
        p2: f64,
        k0: f64,
        k1: f64,
        k2: f64,
        k3: f64,
        k4: f64,
    },
    ZeroC2 {
        p1: f64,
        k0: f64,
        k1: f64,
        k2: f64,
        k3: f64,
    },
}

impl Response {
    /// Output response to a unit ramp and its derivative, as `V0`.
    fn v0(&self, t: f64) -> (f64, f64) {
        match *self {
            Self::Pi {
                p1,
                p2,
                k0,
                k1,
                k2,
                k3,
                k4,
            } => {
                let exp_p1 = exp2(-p1 * t);
                let exp_p2 = exp2(-p2 * t);
                (
                    k0 * (k1 + k2 * t + k3 * exp_p1 + k4 * exp_p2),
                    k0 * (k2 - k3 * p1 * exp_p1 - k4 * p2 * exp_p2),
                )
            }
            Self::ZeroC2 { p1, k0, k1, k2, k3 } => {
                let exp_p1 = exp2(-p1 * t);
                (
                    k0 * (k1 + k2 * t + k3 * exp_p1),
                    k0 * (k2 - k3 * p1 * exp_p1),
                )
            }
        }
    }

    /// Load response to a unit ramp through an Elmore pole `p3` and its
    /// derivative, as `Vl0`.
    fn vl0(&self, t: f64, p3: f64) -> (f64, f64) {
        match *self {
            Self::Pi {
                p1,
                p2,
                k0,
                k1,
                k2,
                k3,
                k4,
            } => {
                let d1 = k0 * (k1 - k2 / p3);
                let d3 = -p3 * k0 * k3 / (p1 - p3);
                let d4 = -p3 * k0 * k4 / (p2 - p3);
                let d5 = k0 * (k2 / p3 - k1 + p3 * k3 / (p1 - p3) + p3 * k4 / (p2 - p3));
                let exp_p1 = exp2(-p1 * t);
                let exp_p2 = exp2(-p2 * t);
                let exp_p3 = exp2(-p3 * t);
                (
                    d1 + t + d3 * exp_p1 + d4 * exp_p2 + d5 * exp_p3,
                    1.0 - d3 * p1 * exp_p1 - d4 * p2 * exp_p2 - d5 * p3 * exp_p3,
                )
            }
            Self::ZeroC2 { p1, k0, k1, k2, k3 } => {
                let d1 = k0 * (k1 - k2 / p3);
                let d3 = -p3 * k0 * k3 / (p1 - p3);
                let d5 = k0 * (k2 / p3 - k1 + p3 * k3 / (p1 - p3));
                let exp_p1 = exp2(-p1 * t);
                let exp_p3 = exp2(-p3 * t);
                (
                    d1 + t + d3 * exp_p1 + d5 * exp_p3,
                    1.0 - d3 * p1 * exp_p1 - d5 * p3 * exp_p3,
                )
            }
        }
    }
}

/// A fitted driver: ramp start and width, the thresholds it is measured
/// against, and the time the output crossed its threshold.
#[derive(Clone, Debug, PartialEq)]
struct Fit {
    t0: f64,
    dt: f64,
    vth: f64,
    vl: f64,
    vh: f64,
    slew_derate: f64,
    vo_delay: f64,
    vo_upper_bound: f64,
    response: Response,
}

impl Fit {
    /// The driver output at `t` and its derivative, as `Vo`.
    fn vo(&self, t: f64) -> (f64, f64) {
        let t1 = t - self.t0;
        if t1 <= 0.0 {
            return (0.0, 0.0);
        }
        if t1 <= self.dt {
            let (v, dv) = self.response.v0(t1);
            return (v / self.dt, dv / self.dt);
        }
        let (v, dv) = self.response.v0(t1);
        let (v_dt, dv_dt) = self.response.v0(t1 - self.dt);
        ((v - v_dt) / self.dt, (dv - dv_dt) / self.dt)
    }

    /// The load voltage at `t` and its derivative, as `Vl`.
    fn vl(&self, t: f64, p3: f64) -> (f64, f64) {
        let t1 = t - self.t0;
        if t1 <= 0.0 {
            return (0.0, 0.0);
        }
        if t1 <= self.dt {
            let (v, dv) = self.response.vl0(t1, p3);
            return (v / self.dt, dv / self.dt);
        }
        let (v, dv) = self.response.vl0(t1, p3);
        let (v_dt, dv_dt) = self.response.vl0(t1 - self.dt, p3);
        ((v - v_dt) / self.dt, (dv - dv_dt) / self.dt)
    }

    fn find_vo_crossing(&self, vth: f64, lower: f64, upper: f64) -> Result<f64, DmpError> {
        find_root(
            |t| {
                let (vo, dvo) = self.vo(t);
                (vo - vth, dvo)
            },
            lower,
            upper,
        )
        .ok_or(DmpError)
    }

    fn find_vl_crossing(&self, vth: f64, p3: f64, lower: f64, upper: f64) -> Result<f64, DmpError> {
        find_root(
            |t| {
                let (vl, dvl) = self.vl(t, p3);
                (vl - vth, dvl)
            },
            lower,
            upper,
        )
        .ok_or(DmpError)
    }

    /// Delay and slew of the driver's fitted waveform, as
    /// `findDriverDelaySlew`.
    fn driver_delay_slew(&self) -> Result<(f64, f64), DmpError> {
        let upper = self.vo_upper_bound;
        let delay = self.find_vo_crossing(self.vth, self.t0, upper)?;
        let tl = self.find_vo_crossing(self.vl, self.t0, delay)?;
        let th = self.find_vo_crossing(self.vh, delay, upper)?;
        Ok((delay, (th - tl) / self.slew_derate))
    }

    /// Wire delay and load slew through an Elmore delay, as
    /// `DmpAlg::loadDelaySlew`.
    fn load_delay_slew(&self, elmore: f64, drvr_slew: f64) -> (f64, f64) {
        if elmore == 0.0 || elmore < drvr_slew * 1e-3 {
            return (elmore, drvr_slew);
        }
        let p3 = 1.0 / elmore;
        let fitted = (|| -> Result<(f64, f64), DmpError> {
            let upper = self.vo_upper_bound + elmore * 2.0;
            let load_delay = self.find_vl_crossing(self.vth, p3, self.t0, upper)?;
            let tl = self.find_vl_crossing(self.vl, p3, self.t0, load_delay)?;
            let th = self.find_vl_crossing(self.vh, p3, load_delay, upper)?;
            let mut delay = load_delay - self.vo_delay;
            let mut slew = (th - tl) / self.slew_derate;
            if delay < 0.0 {
                delay = elmore;
            }
            if slew < drvr_slew {
                slew = drvr_slew;
            }
            Ok((delay, slew))
        })();
        fitted.unwrap_or((elmore, drvr_slew))
    }
}

/// The fit did not converge; the caller falls back as `OpenSTA` does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct DmpError;

/// The ramp response of a capacitive load, as `DmpAlg::y`.
fn y(t: f64, t0: f64, dt: f64, rd: f64, cl: f64) -> f64 {
    let t1 = t - t0;
    if t1 <= 0.0 {
        0.0
    } else if t1 <= dt {
        y0(t1, rd, cl) / dt
    } else {
        (y0(t1, rd, cl) - y0(t1 - dt, rd, cl)) / dt
    }
}

fn y0(t: f64, rd: f64, cl: f64) -> f64 {
    t - rd * cl * (1.0 - exp2(-t / (rd * cl)))
}

fn y0dt(t: f64, rd: f64, cl: f64) -> f64 {
    1.0 - exp2(-t / (rd * cl))
}

fn y0dcl(t: f64, rd: f64, cl: f64) -> f64 {
    rd * ((1.0 + t / (rd * cl)) * exp2(-t / (rd * cl)) - 1.0)
}

/// Partial derivatives of [`y`] by `t0`, `dt` and `cl`, as `DmpAlg::dy`.
fn dy(t: f64, t0: f64, dt: f64, rd: f64, cl: f64) -> [f64; 3] {
    let t1 = t - t0;
    if t1 <= 0.0 {
        [0.0, 0.0, 0.0]
    } else if t1 <= dt {
        [
            -y0dt(t1, rd, cl) / dt,
            -y0(t1, rd, cl) / (dt * dt),
            y0dcl(t1, rd, cl) / dt,
        ]
    } else {
        [
            -(y0dt(t1, rd, cl) - y0dt(t1 - dt, rd, cl)) / dt,
            -(y0(t1, rd, cl) + y0(t1 - dt, rd, cl)) / (dt * dt) + y0dt(t1 - dt, rd, cl) / dt,
            (y0dcl(t1, rd, cl) - y0dcl(t1 - dt, rd, cl)) / dt,
        ]
    }
}

/// The driver parameters the Newton iteration solves for.
#[derive(Clone, Copy, Debug)]
struct Params {
    t0: f64,
    dt: f64,
    ceff: f64,
}

/// Starting point for the driver fit at `ceff`, as the first lines of
/// `findDriverParams`.
fn initial_params(common: &Common<'_>, rd: f64, ceff: f64) -> Params {
    let (t_vth, _, slew) = common.gate_delays(ceff);
    let dt = slew / (common.vh - common.vl);
    let t0 = t_vth + (1.0 - common.vth).ln() * rd * ceff - common.vth * dt;
    Params { t0, dt, ceff }
}

/// The full pi model fit, as `DmpPi`.
fn pi_model(common: &Common<'_>, rd: f64, pi: PiModel) -> GateDelay {
    let PiModel { c2, rpi, c1 } = pi;
    let z1 = 1.0 / (rpi * c1);
    let k0 = 1.0 / (rd * c2);
    let a = rpi * rd * c1 * c2;
    let b = rd * (c1 + c2) + rpi * c1;
    let sqrt = (b * b - 4.0 * a).sqrt();
    let p1 = (b + sqrt) / (2.0 * a);
    let p2 = (b - sqrt) / (2.0 * a);
    let p1p2 = p1 * p2;
    let k2 = z1 / p1p2;
    let k1 = (1.0 - k2 * (p1 + p2)) / p1p2;
    let k4 = (k1 * p1 + k2) / (p2 - p1);
    let k3 = -k1 - k4;
    let z = (c1 + c2) / (rpi * c1 * c2);
    let big_a = z / p1p2;
    let big_b = (z - p1) / (p1 * (p1 - p2));
    let big_d = (z - p2) / (p2 * (p2 - p1));
    let response = Response::Pi {
        p1,
        p2,
        k0,
        k1,
        k2,
        k3,
        k4,
    };

    let eqns = |x: &[f64; 3],
                fvec: &mut [f64; 3],
                fjac: &mut [[f64; 3]; 3]|
     -> Result<(), DmpError> {
        let [t0, dt, ceff] = *x;
        if ceff < 0.0 || ceff > c1 + c2 || dt <= 0.0 {
            return Err(DmpError);
        }
        let (t_vth, t_vl, slew) = common.gate_delays(ceff);
        if slew == 0.0 {
            return Err(DmpError);
        }
        let ceff_time = (slew / (common.vh - common.vl)).min(1.4 * dt);
        let exp_p1_dt = exp2(-p1 * dt);
        let exp_p2_dt = exp2(-p2 * dt);
        let exp_dt_rd_ceff = exp2(-dt / (rd * ceff));
        let y50 = y(t_vth, t0, dt, rd, ceff);
        let y20 = y(t_vl, t0, dt, rd, ceff);
        let exp_p1_ct = exp2(-p1 * ceff_time);
        let exp_p2_ct = exp2(-p2 * ceff_time);
        let exp_ct_rd_ceff = exp2(-ceff_time / (rd * ceff));
        let ipi = (big_a * ceff_time
            + (big_b / p1) * (1.0 - exp_p1_ct)
            + (big_d / p2) * (1.0 - exp_p2_ct))
            / (rd * ceff_time * dt);
        let iceff = (rd * ceff * ceff_time - (rd * ceff) * (rd * ceff) * (1.0 - exp_ct_rd_ceff))
            / (rd * ceff_time * dt);
        fvec[2] = ipi - iceff;
        fvec[1] = y50 - common.vth;
        fvec[0] = y20 - common.vl;
        fjac[2][0] = 0.0;
        fjac[2][1] = (-big_a * dt + big_b * dt * exp_p1_dt
            - (2.0 * big_b / p1) * (1.0 - exp_p1_dt)
            + big_d * dt * exp_p2_dt
            - (2.0 * big_d / p2) * (1.0 - exp_p2_dt)
            + rd * ceff * (dt + dt * exp_dt_rd_ceff - 2.0 * rd * ceff * (1.0 - exp_dt_rd_ceff)))
            / (rd * dt * dt * dt);
        fjac[2][2] =
            (2.0 * rd * ceff - dt - (2.0 * rd * ceff + dt) * exp2(-dt / (rd * ceff))) / (dt * dt);
        fjac[0] = dy(t_vl, t0, dt, rd, ceff);
        fjac[1] = dy(t_vth, t0, dt, rd, ceff);
        Ok(())
    };
    let solve = |start: f64| -> Result<Params, DmpError> {
        let init = initial_params(common, rd, start);
        let mut x = [init.t0, init.dt, start];
        newton_raphson::<3>(&mut x, &eqns)?;
        Ok(Params {
            t0: x[0],
            dt: x[1],
            ceff: x[2],
        })
    };

    let Ok(params) = solve(c2 + c1).or_else(|_| solve(c2)) else {
        let ceff = c1 + c2;
        let (delay, slew) = common.table(ceff);
        return GateDelay {
            delay,
            slew,
            ceff,
            model: LoadModel::Lumped,
        };
    };
    let ceff = params.ceff;
    let (table_delay, table_slew) = common.table(ceff);
    let mut fit = Fit {
        t0: params.t0,
        dt: params.dt,
        vth: common.vth,
        vl: common.vl,
        vh: common.vh,
        slew_derate: common.slew_derate,
        vo_delay: 0.0,
        vo_upper_bound: params.t0 + params.dt + (c1 + c2) * (rd + rpi) * 2.0,
        response,
    };
    match fit.driver_delay_slew() {
        Ok((vo_delay, vo_slew)) => {
            fit.vo_delay = vo_delay;
            GateDelay {
                delay: table_delay,
                slew: vo_slew,
                ceff,
                model: LoadModel::Fitted(fit),
            }
        }
        Err(DmpError) => GateDelay {
            delay: table_delay,
            slew: table_slew,
            ceff,
            model: LoadModel::Lumped,
        },
    }
}

/// The fit when the near capacitance vanishes, as `DmpZeroC2`.
fn zero_c2(common: &Common<'_>, rd: f64, pi: PiModel) -> GateDelay {
    let PiModel { rpi, c1, .. } = pi;
    let ceff = c1;
    let z1 = 1.0 / (rpi * c1);
    let p1 = 1.0 / (c1 * (rd + rpi));
    let k0 = p1 / z1;
    let k2 = 1.0 / k0;
    let k1 = (p1 - z1) / (p1 * p1);
    let k3 = -k1;
    let response = Response::ZeroC2 { p1, k0, k1, k2, k3 };

    let eqns =
        |x: &[f64; 2], fvec: &mut [f64; 2], fjac: &mut [[f64; 2]; 2]| -> Result<(), DmpError> {
            let t0 = x[0];
            let mut dt = x[1];
            let (t_vth, t_vl, _) = common.gate_delays(ceff);
            if dt <= 0.0 {
                dt = (t_vl - t_vth) / 100.0;
            }
            fvec[1] = y(t_vth, t0, dt, rd, ceff) - common.vth;
            fvec[0] = y(t_vl, t0, dt, rd, ceff) - common.vl;
            let [a0, a1, _] = dy(t_vl, t0, dt, rd, ceff);
            fjac[0] = [a0, a1];
            let [b0, b1, _] = dy(t_vth, t0, dt, rd, ceff);
            fjac[1] = [b0, b1];
            Ok(())
        };
    let init = initial_params(common, rd, ceff);
    let mut x = [init.t0, init.dt];
    let fitted = newton_raphson::<2>(&mut x, &eqns).and_then(|()| {
        let mut fit = Fit {
            t0: x[0],
            dt: x[1],
            vth: common.vth,
            vl: common.vl,
            vh: common.vh,
            slew_derate: common.slew_derate,
            vo_delay: 0.0,
            vo_upper_bound: x[0] + x[1] + c1 * (rd + rpi) * 2.0,
            response,
        };
        let (delay, slew) = fit.driver_delay_slew()?;
        fit.vo_delay = delay;
        Ok((delay, slew, fit))
    });
    if let Ok((delay, slew, fit)) = fitted {
        return GateDelay {
            delay,
            slew,
            ceff,
            model: LoadModel::Fitted(fit),
        };
    }
    let (delay, slew) = common.table(ceff);
    GateDelay {
        delay,
        slew,
        ceff,
        model: LoadModel::Lumped,
    }
}

/// A system of `N` equations: fills the residuals and the Jacobian at `x`.
type Equations<'a, const N: usize> =
    dyn Fn(&[f64; N], &mut [f64; N], &mut [[f64; N]; N]) -> Result<(), DmpError> + 'a;

/// Newton-Raphson over `N` unknowns with LU-decomposed steps, as `OpenSTA`'s
/// `newtonRaphson`.
fn newton_raphson<const N: usize>(
    x: &mut [f64; N],
    eval: &Equations<'_, N>,
) -> Result<(), DmpError> {
    let mut fvec = [0.0; N];
    let mut fjac = [[0.0; N]; N];
    for _ in 0..NEWTON_MAX_ITER {
        eval(x, &mut fvec, &mut fjac)?;
        let mut p = [0.0; N];
        for i in 0..N {
            p[i] = -fvec[i];
        }
        let index = lu_decompose(&mut fjac)?;
        lu_solve(&fjac, &index, &mut p);
        let mut all_under_tol = true;
        for i in 0..N {
            if p[i].abs() > x[i].abs() * DRIVER_PARAM_TOL {
                all_under_tol = false;
            }
            x[i] += p[i];
        }
        if all_under_tol {
            eval(x, &mut fvec, &mut fjac)?;
            return Ok(());
        }
    }
    Err(DmpError)
}

/// Crout LU decomposition with partial pivoting; returns the row
/// permutation.
fn lu_decompose<const N: usize>(a: &mut [[f64; N]; N]) -> Result<[usize; N], DmpError> {
    let mut scale = [0.0; N];
    let mut index = [0usize; N];
    for i in 0..N {
        let big = a[i].iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        if big == 0.0 {
            return Err(DmpError);
        }
        scale[i] = 1.0 / big;
    }
    for j in 0..N {
        for i in 0..j {
            let sum: f64 = (0..i).map(|k| a[i][k] * a[k][j]).sum();
            a[i][j] -= sum;
        }
        let mut big = 0.0;
        let mut imax = j;
        for i in j..N {
            let sum: f64 = (0..j).map(|k| a[i][k] * a[k][j]).sum();
            a[i][j] -= sum;
            let dum = scale[i] * a[i][j].abs();
            if dum >= big {
                big = dum;
                imax = i;
            }
        }
        if j != imax {
            a.swap(imax, j);
            scale[imax] = scale[j];
        }
        index[j] = imax;
        if a[j][j] == 0.0 {
            a[j][j] = TINY;
        }
        if j + 1 != N {
            let pivot = 1.0 / a[j][j];
            for row in a.iter_mut().skip(j + 1) {
                row[j] *= pivot;
            }
        }
    }
    Ok(index)
}

fn lu_solve<const N: usize>(a: &[[f64; N]; N], index: &[usize; N], b: &mut [f64; N]) {
    let mut non_zero: Option<usize> = None;
    for i in 0..N {
        let perm = index[i];
        let mut sum = b[perm];
        b[perm] = b[i];
        if let Some(first) = non_zero {
            for j in first..i {
                sum -= a[i][j] * b[j];
            }
        } else if sum != 0.0 {
            non_zero = Some(i);
        }
        b[i] = sum;
    }
    for i in (0..N).rev() {
        let sum: f64 = ((i + 1)..N).map(|j| a[i][j] * b[j]).sum();
        b[i] = (b[i] - sum) / a[i][i];
    }
}

/// A root of `func` between `x1` and `x2` by Newton steps guarded with
/// bisection, as `OpenSTA`'s `findRoot`; `None` when the bounds do not
/// bracket one or the iteration runs out.
fn find_root(func: impl Fn(f64) -> (f64, f64), x1: f64, x2: f64) -> Option<f64> {
    let (y1, _) = func(x1);
    let (y2, _) = func(x2);
    if (y1 > 0.0 && y2 > 0.0) || (y1 < 0.0 && y2 < 0.0) {
        return None;
    }
    if y1 == 0.0 {
        return Some(x1);
    }
    if y2 == 0.0 {
        return Some(x2);
    }
    let (mut x1, mut x2) = if y1 > 0.0 { (x2, x1) } else { (x1, x2) };
    let mut root = f64::midpoint(x1, x2);
    let mut dx_prev = (x2 - x1).abs();
    let mut dx = dx_prev;
    let (mut yv, mut dyv) = func(root);
    for _ in 0..FIND_ROOT_MAX_ITER {
        if ((root - x2) * dyv - yv) * ((root - x1) * dyv - yv) > 0.0
            || (2.0 * yv).abs() > (dx_prev * dyv).abs()
        {
            dx_prev = dx;
            dx = (x2 - x1) * 0.5;
            root = x1 + dx;
        } else {
            dx_prev = dx;
            dx = yv / dyv;
            root -= dx;
        }
        if dx.abs() <= VTH_TIME_TOL * root.abs() {
            return Some(root);
        }
        (yv, dyv) = func(root);
        if yv < 0.0 {
            x1 = root;
        } else {
            x2 = root;
        }
    }
    None
}

/// `OpenSTA`'s fast exponential: twelve squarings of `1 + x / 4096`, and
/// zero below `-12`.
fn exp2(x: f64) -> f64 {
    if x < -12.0 {
        return 0.0;
    }
    let mut y = 1.0 + x / 4096.0;
    for _ in 0..12 {
        y *= y;
    }
    y
}

#[cfg(test)]
mod tests {
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
                values: vec![
                    15e-12, 30e-12, 120e-12, 20e-12, 35e-12, 125e-12, 40e-12, 55e-12, 145e-12,
                ],
            }),
            slew: Some(Table {
                axes,
                values: vec![
                    4e-12, 15e-12, 100e-12, 5e-12, 16e-12, 101e-12, 10e-12, 20e-12, 105e-12,
                ],
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
            c2: 20e-15,
            rpi: 0.0,
            c1: 0.0,
        };
        let result = gate_delay(&arc, &Thresholds::default(), 10e-12, pi);
        let (delay, slew) = arc.gate_delay(10e-12, 20e-15);
        assert!((result.delay - delay).abs() < 1e-18);
        assert!((result.slew - slew).abs() < 1e-18);
        assert!((result.ceff - 20e-15).abs() < 1e-30);
        assert_eq!(result.load_delay_slew(3e-12), (3e-12, slew));
    }

    #[test]
    fn a_resistive_wire_lowers_the_effective_capacitance() {
        let arc = buffer_arc();
        let pi = PiModel {
            c2: 5e-15,
            rpi: 2000.0,
            c1: 40e-15,
        };
        let result = gate_delay(&arc, &Thresholds::default(), 10e-12, pi);
        assert!(result.ceff < 45e-15);
        assert!(result.ceff > 5e-15);
        let (lumped_delay, _) = arc.gate_delay(10e-12, 45e-15);
        assert!(result.delay < lumped_delay);
        let (wire_delay, load_slew) = result.load_delay_slew(40e-12);
        assert!(wire_delay > 0.0);
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
    fn root_finding_brackets_and_converges() {
        let root = find_root(|t| (t * t - 2.0, 2.0 * t), 0.0, 2.0).unwrap();
        assert!((root - 2f64.sqrt()).abs() < 0.02);
        assert!(find_root(|t| (t + 1.0, 1.0), 0.0, 2.0).is_none());
    }
}
