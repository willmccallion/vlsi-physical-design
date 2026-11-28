use super::*;
use crate::netlist::{Constants, Netlist};
use crate::testing::{DefSections, cells, clock_cells_source, def_text, design, pin};

/// The clock `clk` inverted by `i` for flops `f1` and `f2`, and by the
/// fixed `fixed` for `f3`.
fn inverted_clocks() -> NetlistDB {
    design(&def_text(
        "inverted",
        &DefSections {
            components: "- i INV_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - fixed INV_X1 + FIXED ( 2000 8400 ) N ;\n\
                             - f1 DFF_X1 + PLACED ( 20000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 40000 2800 ) N ;\n\
                             - f3 DFF_X1 + PLACED ( 20000 8400 ) N ;\n",
            nets: "- clk ( PIN clk ) ( i A ) ( fixed A ) ;\n\
                       - clkn ( i ZN ) ( f1 CK ) ( f2 CK ) ;\n\
                       - clkf ( fixed ZN ) ( f3 CK ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

#[test]
fn each_load_of_a_clock_inverter_gets_its_own_clone_and_a_fixed_one_stays() {
    let mut db = inverted_clocks();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let clocks = {
        let netlist = Netlist::new(&db, &link);
        let constants = Constants::find(&netlist);
        ClockNetwork::from_ports(&netlist, &["clk".to_string()], &constants).unwrap()
    };

    let done = repair_clock_inverters(&mut db, &link, &clocks, &mut Namer::new()).unwrap();

    assert_eq!((done.inverters, done.clones), (1, 2));
    assert!(db.cell_id("i").is_none() && db.cell_id("fixed").is_some());
    let driver_of = |name: &str| {
        let net = db.pins[pin(&db, name)].net;
        let drivers: Vec<PinId> = db.nets[net]
            .pins
            .iter()
            .copied()
            .filter(|&p| db.pin_name(p).ends_with("/ZN"))
            .collect();
        assert_eq!(drivers.len(), 1, "{name}");
        db.pin_name(drivers[0])
    };
    assert_ne!(driver_of("f1/CK"), driver_of("f2/CK"));
    assert_eq!(driver_of("f3/CK"), "fixed/ZN");
}

#[test]
fn the_net_a_replaced_inverter_drove_goes_with_it() {
    let mut db = inverted_clocks();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let clocks = {
        let netlist = Netlist::new(&db, &link);
        let constants = Constants::find(&netlist);
        ClockNetwork::from_ports(&netlist, &["clk".to_string()], &constants).unwrap()
    };

    let done = repair_clock_inverters(&mut db, &link, &clocks, &mut Namer::new()).unwrap();

    assert!(db.net_id("clkn").is_none());
    assert!(db.net_id("clkf").is_some());
    assert!(done.renumbering.is_some());
}
