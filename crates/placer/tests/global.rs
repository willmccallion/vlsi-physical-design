//! Global placement of a small generated design, pinned down whole:
//! routability-driven, and timing-driven through a repair that resizes,
//! adds and removes cells.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::indices::CellId;
use pare_common::db::parser::{def, lef};
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::testing;
use pare_placer::global::{
    self, GlobalParams, NoTiming, Reweighted, RoutabilityParams, RoutingTiles, TimingDriver,
    TimingParams,
};
use pare_placer::model::PlacementModel;
use pare_placer::zones::Zones;
use std::convert::Infallible;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

/// Gates in a ring, every third an inverter and the rest NAND gates, each
/// NAND's second input fed by the gate halfway round, unplaced in a core
/// of thirty rows of the fixture's cells.
fn ring(gates: usize) -> NetlistDB {
    let mut text = String::from(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN ring ;\n\
         UNITS DISTANCE MICRONS 1000 ;\nDIEAREA ( 0 0 ) ( 60000 60000 ) ;\n",
    );
    for r in 0..30 {
        let orient = if r % 2 == 0 { "N" } else { "FS" };
        let _ = writeln!(
            text,
            "ROW ROW_{r} core 0 {} {orient} DO 300 BY 1 STEP 200 0 ;",
            r * 2000
        );
    }
    let is_inverter = |g: usize| g.is_multiple_of(3);
    let _ = writeln!(text, "COMPONENTS {gates} ;");
    for g in 0..gates {
        let master = if is_inverter(g) { "INV" } else { "NAND2" };
        let _ = writeln!(text, "- g{g} {master} ;");
    }
    let _ = writeln!(text, "END COMPONENTS\nNETS {gates} ;");
    for g in 0..gates {
        let _ = write!(text, "- n{g} ( g{g} Y ) ( g{} A )", (g + 1) % gates);
        let fed = (g + gates / 2) % gates;
        if !is_inverter(fed) {
            let _ = write!(text, " ( g{fed} B )");
        }
        text.push_str(" ;\n");
    }
    text.push_str("END NETS\nEND DESIGN\n");
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse_str(&text, Path::new("ring.def"), library).unwrap()
}

/// A repair that, once, turns inverter `g3` into a NAND gate, adds an
/// inverter `extra` on `n0` where `g0` stands, and removes `g10`.
struct ScriptedRepair<'a> {
    db: &'a mut NetlistDB,
    repairs: usize,
}

impl TimingDriver for ScriptedRepair<'_> {
    type Error = Infallible;

    fn repair(&mut self, model: &PlacementModel) -> Result<Option<Reweighted>, Infallible> {
        self.repairs += 1;
        if self.repairs > 1 {
            return Ok(None);
        }
        let db = &mut *self.db;
        model.write_back(db);
        let cells = db.cells.len();
        let nand = db.library.macro_id("NAND2").unwrap();
        let inverter = db.library.macro_id("INV").unwrap();
        db.replace_master(db.cell_id("g3").unwrap(), nand).unwrap();
        let at = db.cells[db.cell_id("g0").unwrap().index()].location;
        let extra = db
            .add_cell(
                "extra".to_string(),
                inverter,
                at,
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        let input = db.library.macros[inverter.index()].pin_id("A").unwrap();
        let _ = db.connect_cell_pin(extra, input, db.net_id("n0").unwrap());
        let renumbering = db.remove(&[db.cell_id("g10").unwrap()], &[]).unwrap();
        let model = PlacementModel::from_db(db, &Zones::new(db).unwrap());
        Ok(Some(Reweighted {
            model,
            cell_map: renumbering.cells[..cells]
                .iter()
                .map(|c| c.map(CellId::index))
                .collect(),
        }))
    }
}

/// Settings for placing the ring at a 0.7 target density.
const fn params() -> GlobalParams {
    GlobalParams {
        target_density: 0.7,
        target_overflow: 0.1,
        max_iterations: 400,
        initial_rounds: global::INITIAL_PLACE_ROUNDS,
        bins: 32,
        seed: 1,
        routability: None,
        timing: None,
    }
}

/// The result and every cell's position, as text to pin a run down.
fn placement_text(db: &NetlistDB, result: &global::GlobalResult) -> String {
    let mut text = format!(
        "{:?} after {} iterations, overflow {:.6}, hpwl {:.3}\n",
        result.outcome, result.iterations, result.overflow, result.hpwl
    );
    for round in &result.congestion {
        let _ = writeln!(text, "{round:?}");
    }
    for cell in &db.cells {
        let _ = writeln!(
            text,
            "{} {} {}",
            cell.name,
            cell.location.x.get(),
            cell.location.y.get()
        );
    }
    text
}

fn assert_golden(name: &str, actual: &str) {
    testing::assert_golden(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden"),
        name,
        actual,
    );
}

#[test]
fn a_congested_placement_inflates_its_cells_as_it_always_does() {
    let mut db = ring(240);
    let mut model = PlacementModel::from_db(&db, &Zones::new(&db).unwrap());
    let tiles = RoutingTiles {
        origin: Point::new(0.0, 0.0),
        size: 6.0,
        nx: 10,
        ny: 10,
        capacity: vec![12.0; 100],
        reduction: vec![0.0; 100],
    };
    let params = GlobalParams {
        routability: Some(RoutabilityParams {
            tiles,
            wire_width: 2.0,
            target_congestion: 1.0,
        }),
        ..params()
    };

    let result = global::place::<NoTiming>(&mut model, &params, None, None, |_| {}).unwrap();

    assert!(
        result.congestion.iter().any(|r| r.inflated > 0),
        "{:?}",
        result.congestion
    );
    model.write_back(&mut db);
    assert_golden("ring_240_routability.txt", &placement_text(&db, &result));
}

#[test]
fn a_repair_midway_through_placement_gives_the_placement_it_always_has() {
    let mut db = ring(240);
    let mut model = PlacementModel::from_db(&db, &Zones::new(&db).unwrap());
    let params = GlobalParams {
        timing: Some(TimingParams {
            overflows: vec![0.5],
        }),
        ..params()
    };
    let mut repair = ScriptedRepair {
        db: &mut db,
        repairs: 0,
    };

    let result = global::place(&mut model, &params, None, Some(&mut repair), |_| {}).unwrap();

    assert_eq!(repair.repairs, 1);
    model.write_back(&mut db);
    assert_golden("ring_240_repaired.txt", &placement_text(&db, &result));
}
