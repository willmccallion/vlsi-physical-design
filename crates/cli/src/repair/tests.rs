use super::*;
use crate::testing::{cts_fixture, scratch_dir, tiny};
use pare_common::db::design::PinOwner;
use pare_common::db::indices::PinId;

fn clock_lib() -> PathBuf {
    cts_fixture("clock.lib")
}

#[test]
fn one_liberty_file_gives_its_cells() {
    let liberty = load_liberty(&[clock_lib()]).unwrap();

    assert!(liberty.cell_id("DFF_X1").is_some());
}

#[test]
fn liberty_files_merge_their_cells() {
    let renamed = scratch_dir("liberty-merge").join("renamed.lib");
    let text = std::fs::read_to_string(clock_lib()).unwrap();
    std::fs::write(&renamed, text.replace("  cell (", "  cell (B_")).unwrap();

    let liberty = load_liberty(&[clock_lib(), renamed]).unwrap();

    assert!(liberty.cell_id("DFF_X1").is_some());
    assert!(liberty.cell_id("B_DFF_X1").is_some());
}

#[test]
fn a_cell_in_two_liberty_files_names_both() {
    let dir = scratch_dir("liberty-twice");
    let text = std::fs::read_to_string(clock_lib()).unwrap();
    let renamed = text.replace("  cell (", "  cell (B_");
    let (second, third) = (dir.join("b.lib"), dir.join("c.lib"));
    std::fs::write(&second, &renamed).unwrap();
    std::fs::write(&third, &renamed).unwrap();

    let error = load_liberty(&[clock_lib(), second.clone(), third.clone()]).unwrap_err();

    assert_eq!(
        error.to_string(),
        format!(
            "cell 'B_CLKBUF_X3' of {} is already in {}",
            third.display(),
            second.display()
        )
    );
}

#[test]
fn a_cell_of_the_first_liberty_file_again_names_the_first() {
    let error = load_liberty(&[clock_lib(), clock_lib()]).unwrap_err();

    assert_eq!(
        error.to_string(),
        format!(
            "cell 'CLKBUF_X3' of {} is already in {}",
            clock_lib().display(),
            clock_lib().display()
        )
    );
}

/// The clock network `clock_network` finds in the tiny design, linked to
/// the clock library, for the SDC `constraints` when given, else the
/// clock `ports`.
fn tiny_clock_network(
    ports: &[String],
    constraints: Option<&str>,
) -> (NetlistDB, anyhow::Result<ClockNetwork>) {
    let db = tiny();
    let liberty = load_liberty(&[clock_lib()]).unwrap();
    let sdc =
        constraints.map(|text| sdc::parse_str(text, Path::new("tiny.sdc"), &db, &liberty).unwrap());
    let clocks = sdc.as_ref().map_or(Clocks::Ports(ports), Clocks::Sdc);
    let link = Link::new(&db, &liberty, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let constants = Constants::find(&netlist);
    let network = clock_network(&netlist, clocks, &constants);
    (db, network)
}

/// The input pin of `u1`, which the net of port `in` drives.
fn u1_input(db: &NetlistDB) -> PinId {
    let u1 = db.cell_id("u1").unwrap();
    let net = db.net_id("in").unwrap();
    db.nets[net]
        .pins
        .iter()
        .copied()
        .find(|&p| matches!(db.pins[p].owner, PinOwner::Cell { cell, .. } if cell == u1))
        .unwrap()
}

const CLOCK_ON_IN: &str = "create_clock -name clk -period 10 [get_ports in]";

#[test]
fn without_an_sdc_file_the_clock_ports_are_the_clocks() {
    let (db, network) = tiny_clock_network(&["in".to_string()], None);

    assert!(network.unwrap().is_clock(u1_input(&db)));
}

#[test]
fn without_clock_ports_or_an_sdc_file_nothing_is_a_clock() {
    let (db, network) = tiny_clock_network(&[], None);

    assert!(!network.unwrap().is_clock(u1_input(&db)));
}

#[test]
fn an_unknown_clock_port_is_an_error() {
    let (_, network) = tiny_clock_network(&["clk".to_string()], None);

    assert_eq!(
        network.unwrap_err().to_string(),
        "clock port 'clk' is not an I/O pin of the design"
    );
}

#[test]
fn with_an_sdc_file_its_clocks_are_the_clocks() {
    let (db, network) = tiny_clock_network(&[], Some(CLOCK_ON_IN));

    assert!(network.unwrap().is_clock(u1_input(&db)));
}

#[test]
fn no_liberty_files_is_an_error() {
    let error = load_liberty(&[]).unwrap_err();

    assert_eq!(error.to_string(), "no Liberty files");
}
