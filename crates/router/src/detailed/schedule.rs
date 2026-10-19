//! The strategy of each round of detailed routing, as `TritonRoute`'s
//! iteration strategy sets it (`strategy()` in `OpenROAD`'s
//! `src/drt/src/dr/FlexDR.cpp`, with its `ROUTESHAPECOST` of 8 and
//! `MARKERCOST` of 32). Round `r` takes iteration `r`: the conflict and
//! marker costs per unit of length, kept comparable as both grow, the factor
//! marker counters decay by, and which nets it rips up. The iterations in
//! which `TritonRoute` rips up every net or the nets near violations go
//! back to the base costs, and so do those rounds here. The size and offset
//! of `TritonRoute`'s worker tiling are left out, since regions here have
//! their own.

use crate::search::Costs;

/// Which nets an iteration rips up, as `TritonRoute`'s `RipUpMode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RipUp {
    /// Every net.
    All,
    /// The nets of each violation.
    Drc,
    /// The nets of each violation and those near it.
    NearDrc,
}

/// One iteration of `TritonRoute`'s strategy, without its worker tiling.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Strategy {
    /// Rip-up and reroute iterations within one worker.
    maze_end_iter: u32,
    /// Cost per unit of length of a conflict with other wiring.
    conflict: f64,
    /// Cost per unit of length through a marked violation.
    marker: f64,
    /// Cost per unit of length of a conflict with fixed metal.
    fixed_shape: f64,
    /// Factor marker counters decay by after each net routed.
    decay: f64,
    /// Which nets the iteration rips up.
    pub ripup: RipUp,
    /// Whether routes keep to their global route.
    follow_guide: bool,
}

const fn row(
    maze_end_iter: u32,
    conflict: f64,
    marker: f64,
    fixed_shape: f64,
    decay: f64,
    ripup: RipUp,
    follow_guide: bool,
) -> Strategy {
    Strategy {
        maze_end_iter,
        conflict,
        marker,
        fixed_shape,
        decay,
        ripup,
        follow_guide,
    }
}

const SCHEDULE: [Strategy; 65] = [
    row(3, 8.0, 0.0, 8.0, 0.950, RipUp::All, true),    // 0
    row(3, 8.0, 8.0, 8.0, 0.950, RipUp::All, true),    // 1
    row(3, 8.0, 8.0, 8.0, 0.950, RipUp::All, true),    // 2
    row(8, 8.0, 32.0, 16.0, 0.950, RipUp::Drc, false), // 3
    row(8, 8.0, 32.0, 16.0, 0.950, RipUp::Drc, false), // 4
    row(8, 8.0, 32.0, 16.0, 0.950, RipUp::Drc, false), // 5
    row(8, 8.0, 32.0, 16.0, 0.950, RipUp::Drc, false), // 6
    row(8, 8.0, 32.0, 16.0, 0.950, RipUp::Drc, false), // 7
    row(8, 8.0, 32.0, 16.0, 0.950, RipUp::Drc, false), // 8
    row(8, 8.0, 32.0, 16.0, 0.950, RipUp::Drc, false), // 9
    row(8, 16.0, 32.0, 24.0, 0.950, RipUp::Drc, false), // 10
    row(8, 16.0, 32.0, 24.0, 0.950, RipUp::Drc, false), // 11
    row(8, 16.0, 32.0, 24.0, 0.950, RipUp::Drc, false), // 12
    row(8, 16.0, 32.0, 24.0, 0.950, RipUp::Drc, false), // 13
    row(8, 16.0, 32.0, 24.0, 0.950, RipUp::Drc, false), // 14
    row(8, 16.0, 32.0, 32.0, 0.950, RipUp::Drc, false), // 15
    row(8, 16.0, 32.0, 32.0, 0.950, RipUp::Drc, false), // 16
    row(8, 8.0, 32.0, 32.0, 0.950, RipUp::All, false), // 17
    row(8, 32.0, 32.0, 32.0, 0.950, RipUp::Drc, false), // 18
    row(8, 32.0, 32.0, 32.0, 0.950, RipUp::Drc, false), // 19
    row(8, 32.0, 32.0, 80.0, 0.950, RipUp::Drc, false), // 20
    row(8, 32.0, 32.0, 80.0, 0.950, RipUp::Drc, false), // 21
    row(8, 32.0, 32.0, 80.0, 0.950, RipUp::Drc, false), // 22
    row(8, 8.0, 32.0, 80.0, 0.950, RipUp::NearDrc, false), // 23
    row(8, 32.0, 32.0, 80.0, 0.950, RipUp::Drc, false), // 24
    row(8, 8.0, 32.0, 80.0, 0.950, RipUp::All, false), // 25
    row(8, 64.0, 64.0, 80.0, 0.950, RipUp::Drc, false), // 26
    row(8, 64.0, 64.0, 80.0, 0.950, RipUp::Drc, false), // 27
    row(8, 64.0, 64.0, 80.0, 0.950, RipUp::Drc, false), // 28
    row(8, 64.0, 64.0, 80.0, 0.950, RipUp::Drc, false), // 29
    row(8, 8.0, 32.0, 400.0, 0.950, RipUp::NearDrc, false), // 30
    row(8, 64.0, 64.0, 400.0, 0.950, RipUp::Drc, false), // 31
    row(8, 64.0, 64.0, 400.0, 0.950, RipUp::Drc, false), // 32
    row(8, 8.0, 32.0, 400.0, 0.950, RipUp::All, false), // 33
    row(8, 128.0, 128.0, 400.0, 0.950, RipUp::Drc, false), // 34
    row(8, 128.0, 128.0, 400.0, 0.950, RipUp::Drc, false), // 35
    row(8, 128.0, 128.0, 400.0, 0.950, RipUp::Drc, false), // 36
    row(8, 8.0, 32.0, 400.0, 0.950, RipUp::NearDrc, false), // 37
    row(8, 128.0, 128.0, 400.0, 0.950, RipUp::Drc, false), // 38
    row(8, 128.0, 128.0, 400.0, 0.950, RipUp::Drc, false), // 39
    row(8, 128.0, 128.0, 800.0, 0.990, RipUp::Drc, false), // 40
    row(8, 8.0, 32.0, 800.0, 0.990, RipUp::All, false), // 41
    row(16, 128.0, 128.0, 800.0, 0.990, RipUp::Drc, false), // 42
    row(16, 128.0, 128.0, 800.0, 0.990, RipUp::Drc, false), // 43
    row(16, 8.0, 32.0, 800.0, 0.990, RipUp::NearDrc, false), // 44
    row(16, 128.0, 128.0, 800.0, 0.990, RipUp::Drc, false), // 45
    row(16, 128.0, 128.0, 800.0, 0.990, RipUp::Drc, false), // 46
    row(16, 128.0, 128.0, 800.0, 0.990, RipUp::Drc, false), // 47
    row(16, 128.0, 128.0, 800.0, 0.990, RipUp::Drc, false), // 48
    row(8, 8.0, 32.0, 800.0, 0.990, RipUp::All, false), // 49
    row(32, 256.0, 256.0, 800.0, 0.999, RipUp::Drc, false), // 50
    row(32, 8.0, 32.0, 800.0, 0.999, RipUp::NearDrc, false), // 51
    row(32, 256.0, 256.0, 800.0, 0.999, RipUp::Drc, false), // 52
    row(32, 256.0, 256.0, 800.0, 0.999, RipUp::Drc, false), // 53
    row(32, 256.0, 256.0, 800.0, 0.999, RipUp::Drc, false), // 54
    row(32, 256.0, 256.0, 800.0, 0.999, RipUp::Drc, false), // 55
    row(32, 256.0, 256.0, 800.0, 0.999, RipUp::Drc, false), // 56
    row(8, 8.0, 32.0, 800.0, 0.999, RipUp::All, false), // 57
    row(64, 8.0, 32.0, 800.0, 0.999, RipUp::NearDrc, false), // 58
    row(64, 512.0, 512.0, 800.0, 0.999, RipUp::Drc, false), // 59
    row(64, 512.0, 512.0, 800.0, 0.999, RipUp::Drc, false), // 60
    row(64, 512.0, 512.0, 800.0, 0.999, RipUp::Drc, false), // 61
    row(64, 512.0, 512.0, 800.0, 0.999, RipUp::Drc, false), // 62
    row(64, 512.0, 512.0, 800.0, 0.999, RipUp::Drc, false), // 63
    row(64, 512.0, 512.0, 800.0, 0.999, RipUp::Drc, false), // 64
];

/// The strategy of round `round`, 0 being the first pass. Rounds past
/// the schedule keep its last strategy.
pub fn round_strategy(round: usize) -> Strategy {
    SCHEDULE[round.min(SCHEDULE.len() - 1)]
}

/// The costs of round `round`, 0 being the first pass, with a via costing
/// `via`. Rounds past the schedule keep its last costs.
pub fn round_costs(round: usize, via: f64) -> Costs {
    let strategy = round_strategy(round);
    Costs {
        via,
        conflict: strategy.conflict,
        marker: strategy.marker,
        decay: strategy.decay,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_first_pass_has_no_marker_cost() {
        let costs = round_costs(0, 1.0);

        assert!((costs.conflict - 8.0).abs() < f64::EPSILON);
        assert!(costs.marker.abs() < f64::EPSILON);
    }

    #[test]
    fn conflict_and_marker_costs_grow_together() {
        let costs = round_costs(34, 1.0);

        assert!((costs.conflict - 128.0).abs() < f64::EPSILON);
        assert!((costs.marker - 128.0).abs() < f64::EPSILON);
    }

    #[test]
    fn rounds_past_the_schedule_keep_its_last_costs() {
        assert_eq!(round_costs(200, 1.0), round_costs(64, 1.0));
    }

    #[test]
    fn rounds_that_rip_up_every_net_go_back_to_the_base_costs() {
        let strategy = round_strategy(49);

        assert_eq!(strategy.ripup, RipUp::All);
        assert!((strategy.conflict - 8.0).abs() < f64::EPSILON);
    }

    #[test]
    fn rounds_with_equal_costs_but_a_new_fixed_shape_cost_differ() {
        let (fourteenth, fifteenth) = (round_strategy(14), round_strategy(15));

        assert_ne!(fourteenth, fifteenth);
    }

    #[test]
    fn rounds_differing_only_in_tiling_are_the_same_strategy() {
        assert_eq!(round_strategy(4), round_strategy(5));
    }
}
