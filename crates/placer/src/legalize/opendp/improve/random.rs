//! Random improvement, as `DetailedRandom` running `default -gen rng -obj
//! hpwl -cost (hpwl)`: random cells try moves and swaps to random spots
//! within ten sites and rows, each kept unless it lengthens the nets.

use super::swap::Hpwl;
use super::{ImprovementError, Improver, Result};

/// How far, in sites and rows, a random move may reach.
const REACH: usize = 10;
/// Spots a random move tries before giving up.
const TRIES: usize = 5;

/// `default -p passes -t tolerance -f moves_per_candidate`.
pub(super) fn run(
    imp: &mut Improver,
    passes: usize,
    tolerance: f64,
    moves_per_candidate: f64,
) -> Result<()> {
    let tolerance = tolerance.max(0.01);
    let mut objective = Hpwl::new(imp);
    let _ = objective.current(imp);
    for _ in 0..passes.max(1) {
        imp.resort_segments();
        if go(imp, &mut objective, moves_per_candidate)? < tolerance {
            break;
        }
    }
    imp.resort_segments();
    Ok(())
}

/// One pass of random trials, returning the share of the cost it saved,
/// as `DetailedRandom::go`.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)] // attempt counts as OpenDP's
fn go(imp: &mut Improver, objective: &mut Hpwl, moves_per_candidate: f64) -> Result<f64> {
    let mut candidates = imp.movable_cells();
    if candidates.is_empty() {
        return Ok(0.0);
    }
    let attempts = (moves_per_candidate * candidates.len() as f64).ceil() as usize;
    imp.shuffle(&mut candidates);
    let initial = objective.current(imp);
    let mut current = initial;
    for _ in 0..attempts {
        let _generator = imp.random(1);
        if !random_trial(imp, &candidates)? {
            continue;
        }
        let next = current - objective.delta(imp);
        if next <= current {
            imp.accept_move();
            objective.accept(imp);
            current = next;
        } else {
            imp.reject_move()?;
        }
    }
    let _ = objective.current(imp);
    Ok((initial - current) as f64 / initial as f64)
}

/// A move or else a swap of a random cell to a random spot nearby, as
/// `RandomGenerator::generate`.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)] // as OpenDP's int casts
fn random_trial(imp: &mut Improver, candidates: &[usize]) -> Result<bool> {
    let arch = &imp.arch;
    let ydim = arch.rows.len();
    let span_x = (arch.max_x - arch.min_x) as f64;
    let xdim = ((span_x / arch.rows[0].site_spacing as f64) as i32).max(0);
    let xwid = span_x / f64::from(xdim);
    let ywid = (arch.max_y - arch.min_y) as f64 / ydim as f64;
    let (min_x, min_y) = (arch.min_x as f64, arch.min_y as f64);
    let row_height = arch.rows[0].height as f64;
    let n = candidates[imp.random(candidates.len())];
    if imp.rows_spanned(n) != 1 {
        return Ok(false);
    }
    let [si] = imp.cell_segs[n][..] else {
        return Err(ImprovementError::Internal(
            "Only working with single height cells currently.",
        ));
    };
    let ydim = i32::try_from(ydim).unwrap_or(i32::MAX);
    let reach = REACH as i32;
    for _ in 0..TRIES {
        let yi = imp.bottom(n) as f64 + 0.5 * imp.height(n) as f64;
        let xi = imp.left(n) as f64 + 0.5 * imp.width(n) as f64;
        let grid_xi = (((xi - min_x) / xwid) as i32).max(0).min(xdim - 1);
        let grid_yi = (((yi - min_y) / ywid) as i32).max(0).min(ydim - 1);
        let rel_x = imp.random(2 * REACH + 1) as i32;
        let rel_y = imp.random(2 * REACH + 1) as i32;
        let grid_xj = (grid_xi - reach + rel_x).max(0).min(xdim - 1);
        let grid_yj = (grid_yi - reach + rel_y).max(0).min(ydim - 1);
        let xj = min_x + f64::from(grid_xj) * xwid;
        let yj = min_y + f64::from(grid_yj) * ywid;
        let rj = (((yj - min_y) / row_height) as i32).max(0).min(ydim - 1);
        let rj = usize::try_from(rj).unwrap_or(0);
        let x_truncated = i64::from(xj as i32);
        let found = imp.segs_in_row[rj].iter().copied().find(|&s| {
            let seg = &imp.segments[s];
            x_truncated >= seg.min_x && x_truncated <= seg.max_x
        });
        let Some(sj) = found else {
            continue;
        };
        if imp.group(n) != imp.segments[sj].region {
            continue;
        }
        let x = xj.round() as i64;
        if imp.try_move(n, si, x, sj)? || imp.try_swap(n, si, x, sj)? {
            return Ok(true);
        }
    }
    Ok(false)
}
