//! The costs of each round of detailed routing, as `TritonRoute`'s
//! iteration strategy sets them (`strategy()` in `OpenROAD`'s
//! `src/drt/src/dr/FlexDR.cpp`, with its `ROUTESHAPECOST` of 8 and
//! `MARKERCOST` of 32). Round `r` takes the costs of iteration `r`: the
//! conflict and marker costs per unit of length, kept comparable as both
//! grow, and the factor marker counters decay by. The iterations in which
//! `TritonRoute` rips up every net or the nets near violations go back to
//! the base costs, and so do those rounds here.

use crate::search::Costs;

/// Conflict cost, marker cost and marker decay of each iteration, with the
/// rip-up mode `TritonRoute` gives it.
const SCHEDULE: [(f64, f64, f64); 65] = [
    (8.0, 0.0, 0.950),     // 0 ALL
    (8.0, 8.0, 0.950),     // 1 ALL
    (8.0, 8.0, 0.950),     // 2 ALL
    (8.0, 32.0, 0.950),    // 3 DRC
    (8.0, 32.0, 0.950),    // 4 DRC
    (8.0, 32.0, 0.950),    // 5 DRC
    (8.0, 32.0, 0.950),    // 6 DRC
    (8.0, 32.0, 0.950),    // 7 DRC
    (8.0, 32.0, 0.950),    // 8 DRC
    (8.0, 32.0, 0.950),    // 9 DRC
    (16.0, 32.0, 0.950),   // 10 DRC
    (16.0, 32.0, 0.950),   // 11 DRC
    (16.0, 32.0, 0.950),   // 12 DRC
    (16.0, 32.0, 0.950),   // 13 DRC
    (16.0, 32.0, 0.950),   // 14 DRC
    (16.0, 32.0, 0.950),   // 15 DRC
    (16.0, 32.0, 0.950),   // 16 DRC
    (8.0, 32.0, 0.950),    // 17 ALL
    (32.0, 32.0, 0.950),   // 18 DRC
    (32.0, 32.0, 0.950),   // 19 DRC
    (32.0, 32.0, 0.950),   // 20 DRC
    (32.0, 32.0, 0.950),   // 21 DRC
    (32.0, 32.0, 0.950),   // 22 DRC
    (8.0, 32.0, 0.950),    // 23 NEARDRC
    (32.0, 32.0, 0.950),   // 24 DRC
    (8.0, 32.0, 0.950),    // 25 ALL
    (64.0, 64.0, 0.950),   // 26 DRC
    (64.0, 64.0, 0.950),   // 27 DRC
    (64.0, 64.0, 0.950),   // 28 DRC
    (64.0, 64.0, 0.950),   // 29 DRC
    (8.0, 32.0, 0.950),    // 30 NEARDRC
    (64.0, 64.0, 0.950),   // 31 DRC
    (64.0, 64.0, 0.950),   // 32 DRC
    (8.0, 32.0, 0.950),    // 33 ALL
    (128.0, 128.0, 0.950), // 34 DRC
    (128.0, 128.0, 0.950), // 35 DRC
    (128.0, 128.0, 0.950), // 36 DRC
    (8.0, 32.0, 0.950),    // 37 NEARDRC
    (128.0, 128.0, 0.950), // 38 DRC
    (128.0, 128.0, 0.950), // 39 DRC
    (128.0, 128.0, 0.990), // 40 DRC
    (8.0, 32.0, 0.990),    // 41 ALL
    (128.0, 128.0, 0.990), // 42 DRC
    (128.0, 128.0, 0.990), // 43 DRC
    (8.0, 32.0, 0.990),    // 44 NEARDRC
    (128.0, 128.0, 0.990), // 45 DRC
    (128.0, 128.0, 0.990), // 46 DRC
    (128.0, 128.0, 0.990), // 47 DRC
    (128.0, 128.0, 0.990), // 48 DRC
    (8.0, 32.0, 0.990),    // 49 ALL
    (256.0, 256.0, 0.999), // 50 DRC
    (8.0, 32.0, 0.999),    // 51 NEARDRC
    (256.0, 256.0, 0.999), // 52 DRC
    (256.0, 256.0, 0.999), // 53 DRC
    (256.0, 256.0, 0.999), // 54 DRC
    (256.0, 256.0, 0.999), // 55 DRC
    (256.0, 256.0, 0.999), // 56 DRC
    (8.0, 32.0, 0.999),    // 57 ALL
    (8.0, 32.0, 0.999),    // 58 NEARDRC
    (512.0, 512.0, 0.999), // 59 DRC
    (512.0, 512.0, 0.999), // 60 DRC
    (512.0, 512.0, 0.999), // 61 DRC
    (512.0, 512.0, 0.999), // 62 DRC
    (512.0, 512.0, 0.999), // 63 DRC
    (512.0, 512.0, 0.999), // 64 DRC
];

/// The costs of round `round`, 0 being the first pass, with a via costing
/// `via`. Rounds past the schedule keep its last costs.
pub fn round_costs(round: usize, via: f64) -> Costs {
    let (conflict, marker, decay) = SCHEDULE[round.min(SCHEDULE.len() - 1)];
    Costs {
        via,
        conflict,
        marker,
        decay,
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
}
