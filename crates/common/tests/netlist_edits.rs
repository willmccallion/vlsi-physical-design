//! Editing the netlist: adding, moving, renaming and removing, against the
//! fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{EditError, NetlistDB, PinOwner};
use pare_common::db::indices::{CellId, NetId, PinId};
use pare_common::db::parser::{def, lef};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

fn tiny_design() -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse(&fixture("tiny.def"), library).unwrap()
}

fn net_pins(db: &NetlistDB, net: &str) -> Vec<String> {
    let id = db.net_id(net).unwrap();
    let mut names: Vec<String> = db.nets[id].pins.iter().map(|&p| db.pin_name(p)).collect();
    names.sort();
    names
}

/// The names of a net's pins in the order the net lists them.
fn net_pin_order(db: &NetlistDB, net: &str) -> Vec<String> {
    let id = db.net_id(net).unwrap();
    db.nets[id].pins.iter().map(|&p| db.pin_name(p)).collect()
}

/// Every connection is listed by its net and its owner, and every listing
/// points back to it.
fn assert_consistent(db: &NetlistDB) {
    for (i, pin) in db.pins.iter().enumerate() {
        let id = PinId::new(i);
        assert!(db.nets[pin.net].pins.contains(&id));
        match pin.owner {
            PinOwner::Cell { cell, .. } => assert!(db.cells[cell].pins.contains(&id)),
            PinOwner::Io(io) => {
                assert_eq!(db.io_pins[io].pin, id);
                assert_eq!(db.io_pins[io].net, pin.net);
            }
        }
    }
    for (i, cell) in db.cells.iter().enumerate() {
        assert_eq!(db.cell_id(&cell.name), Some(CellId::new(i)));
        for &p in &cell.pins {
            assert!(matches!(db.pins[p].owner, PinOwner::Cell { cell, .. } if cell.index() == i));
        }
    }
    for (i, net) in db.nets.iter().enumerate() {
        assert_eq!(db.net_id(&net.name), Some(NetId::new(i)));
        for &p in &net.pins {
            assert_eq!(db.pins[p].net.index(), i);
        }
    }
}

#[test]
fn a_pin_moves_between_nets() {
    let mut db = tiny_design();
    let u2_a = db.cells[db.cell_id("u2").unwrap()].pins[0];
    assert_eq!(db.pin_name(u2_a), "u2/A");
    let n2 = db.net_id("n2").unwrap();

    db.move_pin(u2_a, n2);

    assert_eq!(net_pins(&db, "n1"), vec!["u1/Y", "u3/A"]);
    assert_eq!(net_pins(&db, "n2"), vec!["u2/A", "u2/Y", "u3/B", "u4/A"]);
    assert_consistent(&db);
}

#[test]
fn a_moved_pin_put_back_takes_its_old_place() {
    let mut db = tiny_design();
    let u1_y = db.cells[db.cell_id("u1").unwrap()].pins[1];
    let before = (net_pin_order(&db, "n1"), net_pin_order(&db, "n2"));
    assert_eq!(before.0.first().map(String::as_str), Some("u1/Y"));
    let place = db.pin_place(u1_y);
    db.move_pin(u1_y, db.net_id("n2").unwrap());

    db.put_pin_back(u1_y, place);

    assert_eq!((net_pin_order(&db, "n1"), net_pin_order(&db, "n2")), before);
    assert_consistent(&db);
}

#[test]
fn an_io_pin_moving_nets_updates_its_record() {
    let mut db = tiny_design();
    let io = db.io_pin_id("in").unwrap();
    let pin = db.io_pins[io].pin;
    let fresh = db.add_net("buffered").unwrap();

    db.move_pin(pin, fresh);

    assert_eq!(db.io_pins[io].net, fresh);
    assert_eq!(net_pins(&db, "buffered"), vec!["PIN in"]);
    assert_eq!(net_pins(&db, "in"), vec!["u1/A"]);
    assert_consistent(&db);
}

#[test]
fn adding_or_renaming_to_an_existing_net_name_is_refused() {
    let mut db = tiny_design();
    assert!(db.add_net("n1").is_err());
    let n2 = db.net_id("n2").unwrap();
    assert!(db.rename_net(n2, "n3".into()).is_err());

    db.rename_net(n2, "renamed".into()).unwrap();

    assert_eq!(db.net_id("n2"), None);
    assert_eq!(db.net_id("renamed"), Some(n2));
    assert_eq!(db.nets[n2].name, "renamed");
}

#[test]
fn removing_cells_drops_their_connections_and_renumbers() {
    let mut db = tiny_design();
    let u1 = db.cell_id("u1").unwrap();
    let u5 = db.cell_id("u5").unwrap();
    let cells_before = db.cells.len();
    let pins_before = db.pins.len();
    let removed_pins = db.cells[u1].pins.len() + db.cells[u5].pins.len();

    let _ = db.remove(&[u1, u5], &[]).unwrap();

    assert_eq!(db.cells.len(), cells_before - 2);
    assert_eq!(db.pins.len(), pins_before - removed_pins);
    assert_eq!(db.cell_id("u1"), None);
    assert_eq!(db.cell_id("u5"), None);
    assert_eq!(net_pins(&db, "in"), vec!["PIN in"]);
    assert_eq!(net_pins(&db, "n1"), vec!["u2/A", "u3/A"]);
    assert_eq!(
        net_pins(&db, "n3"),
        vec!["ram0/D", "u3/Y", "u4/B", "u6/A", "u7/A"]
    );
    assert_eq!(db.cells[db.cell_id("u7").unwrap()].name, "u7");
    assert_consistent(&db);
}

#[test]
fn a_net_is_removed_only_once_nothing_connects_to_it() {
    let mut db = tiny_design();
    let n1 = db.net_id("n1").unwrap();
    assert_eq!(db.remove(&[], &[n1]), Err(EditError::NetInUse("n1".into())));

    let u1 = db.cell_id("u1").unwrap();
    let n2 = db.net_id("n2").unwrap();
    for pin in db.nets[n1].pins.clone() {
        db.move_pin(pin, n2);
    }
    let nets_before = db.nets.len();
    let _ = db.remove(&[u1], &[n1]).unwrap();

    assert_eq!(db.nets.len(), nets_before - 1);
    assert_eq!(db.net_id("n1"), None);
    assert_eq!(
        net_pins(&db, "n2"),
        vec!["u2/A", "u2/Y", "u3/A", "u3/B", "u4/A"]
    );
    assert_consistent(&db);
}

#[test]
fn a_retired_cell_leaves_its_nets_and_comes_back() {
    let mut db = tiny_design();
    let u1 = db.cell_id("u1").unwrap();
    let before = (net_pin_order(&db, "in"), net_pin_order(&db, "n1"));
    assert_eq!(before.1.first().map(String::as_str), Some("u1/Y"));

    let places = db.retire_cell(u1);

    assert!(db.cells[u1].retired);
    assert_eq!(net_pins(&db, "in"), vec!["PIN in"]);
    assert_eq!(net_pins(&db, "n1"), vec!["u2/A", "u3/A"]);
    assert_eq!(db.cells[u1].pins.len(), 2);

    db.unretire_cell(u1, places);

    assert!(!db.cells[u1].retired);
    assert_eq!((net_pin_order(&db, "in"), net_pin_order(&db, "n1")), before);
    assert_consistent(&db);
}

#[test]
fn removal_drops_retired_cells_and_maps_the_survivors() {
    let mut db = tiny_design();
    let u1 = db.cell_id("u1").unwrap();
    let u5 = db.cell_id("u5").unwrap();
    let u7 = db.cell_id("u7").unwrap();
    let u7_pin = db.cells[u7].pins[0];
    let n3 = db.net_id("n3").unwrap();
    let _ = db.retire_cell(u5);

    let map = db.remove(&[u1], &[]).unwrap();

    assert_eq!(db.cell_id("u5"), None);
    assert_eq!(map.cells[u1], None);
    assert_eq!(map.cells[u5], None);
    let new_u7 = map.cells[u7].unwrap();
    assert_eq!(db.cells[new_u7].name, "u7");
    let new_pin = map.pins[u7_pin].unwrap();
    assert!(db.cells[new_u7].pins.contains(&new_pin));
    assert_eq!(db.nets[map.nets[n3].unwrap()].name, "n3");
    assert_eq!(
        net_pins(&db, "n3"),
        vec!["ram0/D", "u3/Y", "u4/B", "u6/A", "u7/A"]
    );
    assert_consistent(&db);
}

#[test]
fn a_new_master_with_every_pin_keeps_the_connections() {
    let mut db = tiny_design();
    let u1 = db.cell_id("u1").unwrap();
    let nand2 = db.library.macro_id("NAND2").unwrap();

    db.replace_master(u1, nand2).unwrap();

    assert_eq!(db.cells[u1].macro_id, nand2);
    assert_eq!(net_pins(&db, "in"), vec!["PIN in", "u1/A"]);
    assert_eq!(net_pins(&db, "n1"), vec!["u1/Y", "u2/A", "u3/A"]);
    assert_consistent(&db);
}

#[test]
fn a_new_master_lacking_a_connected_pin_is_refused_by_its_name() {
    let mut db = tiny_design();
    let u3 = db.cell_id("u3").unwrap();
    let inv = db.library.macro_id("INV").unwrap();

    let refused = db.replace_master(u3, inv);

    assert_eq!(
        refused,
        Err(EditError::MissingPin {
            cell: "u3".into(),
            master: "INV".into(),
            pin: "B".into(),
        })
    );
}
