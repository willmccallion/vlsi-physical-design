use super::*;
use crate::plan::FlowPlan;
use crate::repair::repair_netlist;
use crate::testing::{scratch_dir, timed, timed_config};
use std::collections::BTreeSet;

#[test]
fn every_flop_clock_pin_has_a_clock_arrival() {
    let config = timed_config("", &scratch_dir("clock-arrivals"));
    let mut db = timed();
    let plan = FlowPlan::resolve(&config, &db).unwrap();
    let repair_plan = plan.repair.as_ref().unwrap();
    let mut repair = repair_netlist(&config, repair_plan, &mut db, &mut Report::default()).unwrap();
    let mut timed_repair = repair.timed().unwrap();
    propagate_clocks(timed_repair.sdc);

    let arrivals = clock_arrivals(&mut db, &mut timed_repair).unwrap();

    let pins: BTreeSet<String> = arrivals.keys().map(|&pin| db.pin_name(pin)).collect();
    for flop in ["ff0", "ff1", "ff2", "ff3"] {
        assert!(pins.contains(&format!("{flop}/CK")), "{pins:?}");
    }
}
