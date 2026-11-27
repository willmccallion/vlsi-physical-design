use super::*;
use crate::db::design::PinOwner;

#[test]
fn synthesized_stack_alternates_and_has_a_via_per_cut() {
    let mut lib = Library::new(BOOKSHELF_DBU);

    synthesize_layer_stack(&mut lib, Dbu::new(6400), 4).unwrap();

    let dirs: Vec<_> = lib.routing_layers.iter().map(|l| l.direction).collect();
    assert_eq!(
        dirs,
        [
            Direction::Horizontal,
            Direction::Vertical,
            Direction::Horizontal,
            Direction::Vertical
        ]
    );
    assert_eq!(lib.routing_layers[LayerId(0)].track_pitch(), Dbu::new(400));
    assert_eq!(lib.cut_layers.len(), 3);
    assert!((0..3).all(|c| lib.default_via(crate::db::indices::CutLayerId(c)).is_some()));
}

fn scratch(name: &str, text: &str) -> std::path::PathBuf {
    let path = std::env::temp_dir().join(format!("pare-bookshelf-{}-{name}", std::process::id()));
    std::fs::write(&path, text).unwrap();
    path
}

fn tiny() -> NetlistDB {
    let aux =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/bookshelf/tiny.aux");
    parse(&aux, 6).unwrap()
}

#[test]
fn a_stack_needs_two_routing_layers() {
    assert!(synthesize_layer_stack(&mut Library::new(BOOKSHELF_DBU), Dbu::new(6400), 1).is_err());
    assert!(synthesize_layer_stack(&mut Library::new(BOOKSHELF_DBU), Dbu::new(6400), 2).is_ok());
}

#[test]
fn a_core_row_takes_its_site_width_and_spacing() {
    let path = scratch(
        "rows.scl",
        "UCLA scl 1.0\nNumRows : 1\nCoreRow Horizontal\n Coordinate : 0\n Height : 16\n \
         Sitewidth : 2\n Sitespacing : 3\n SubrowOrigin : 0 NumSites : 10\nEnd\n",
    );

    let scl = read_scl(&path, BOOKSHELF_DBU);

    std::fs::remove_file(&path).unwrap();
    let row = &scl.unwrap().rows[0];
    assert_eq!(
        (row.site_width, row.site_spacing),
        (Dbu::new(200), Dbu::new(300))
    );
}

#[test]
fn a_placement_marked_fixed_fixes_a_movable_node() {
    let mut db = tiny();
    let path = scratch("fixed.pl", "UCLA pl 1.0\n\na2\t10\t0\t: N /FIXED\n");

    let read = read_placement(&mut db, &path);

    std::fs::remove_file(&path).unwrap();
    read.unwrap();
    let a2 = db.cell_id("a2").unwrap();
    assert!(!db.cells[a2].status.is_movable());
}

#[test]
fn a_node_on_two_nets_connects_a_pin_of_its_own_to_each() {
    let db = tiny();

    let pins = |net: &str| db.nets[db.net_id(net).unwrap()].pins.len();
    assert_eq!(pins("clk"), 2);
    assert_eq!(db.nets.iter().map(|n| n.pins.len()).sum::<usize>(), 5);
}

#[test]
fn each_layer_gets_tracks_across_the_die_from_half_a_pitch_in() {
    let db = tiny();
    let die = db.die_area;

    assert_eq!(db.tracks.len(), db.library.routing_layers.len());
    let first = &db.tracks[0];
    let pitch = db.library.routing_layers[LayerId(0)].track_pitch();
    assert_eq!(first.start, die.min.y + pitch / 2);
    let last = first.start + pitch * i64::from(first.count - 1);
    assert!(last + pitch / 2 <= die.max.y && die.max.y < last + pitch + pitch / 2);
}

#[test]
fn a_node_takes_its_pins_in_the_order_its_nets_list_it() {
    let db = tiny();
    let a1 = db.cell_id("a1").unwrap();

    let pins: Vec<MacroPinId> = db.cells[a1]
        .pins
        .iter()
        .filter_map(|&p| match db.pins[p].owner {
            PinOwner::Cell { pin, .. } => Some(pin),
            PinOwner::Io(_) => None,
        })
        .collect();

    assert_eq!(pins, [MacroPinId(0), MacroPinId(1)]);
}
