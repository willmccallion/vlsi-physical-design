//! Whole routing results on the placed fixture, pinned in golden files.

use super::placed_tiny;
use crate::{Progress, RouteState, RunOptions, Save, SavePoint, route_with};
use pare_common::db::design::{NetlistDB, RouteElem};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::testing::assert_golden;
use std::fmt::Write;
use std::path::Path;

fn golden_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}

/// Routes the placed fixture, returning the design, the state saved after
/// global routing and every progress report but the timings, as text.
fn routed_tiny() -> (NetlistDB, RouteState, String) {
    let (mut db, config) = placed_tiny();
    let mut saved = None;
    let mut write = |state: &RouteState| {
        saved = Some(state.clone());
        Ok(())
    };
    let mut progress = String::new();
    let mut report = |p: &Progress<'_>| progress.push_str(&progress_text(p));
    let run = RunOptions {
        save: Some(Save {
            after: SavePoint::Global,
            write: &mut write,
        }),
        report: Some(&mut report),
        ..RunOptions::default()
    };
    let _ = route_with(&mut db, &config, run).unwrap();
    (db, saved.unwrap(), progress)
}

fn progress_text(p: &Progress<'_>) -> String {
    match p {
        Progress::Stage { .. } => String::new(),
        Progress::GlobalRouted {
            gcells,
            overflow,
            layers,
        } => {
            let mut text = format!("gcells {} {} overflow {overflow}\n", gcells.0, gcells.1);
            for l in *layers {
                let _ = writeln!(
                    text,
                    "layer {} {:?} capacity {} usage {} overflow {}",
                    l.name, l.direction, l.capacity, l.usage, l.overflow
                );
            }
            text
        }
        Progress::GlobalSettled { kept, rerouted } => {
            format!("settled kept {kept} rerouted {rerouted}\n")
        }
        Progress::DetailedRound(r) => format!(
            "round {} {} markers {} conflicted {} repaired {}\n",
            r.round, r.flow, r.markers, r.conflicted, r.repaired
        ),
    }
}

fn point_text(p: Point<Dbu>) -> String {
    format!("{} {}", p.x.get(), p.y.get())
}

fn elem_text(db: &NetlistDB, elem: &RouteElem) -> String {
    match elem {
        RouteElem::Wire {
            layer,
            from,
            to,
            extensions,
        } => format!(
            "wire {} ( {} ) ( {} ) {:?}",
            db.library.layer(*layer).name,
            point_text(*from),
            point_text(*to),
            extensions
        ),
        RouteElem::Via { via, at } => {
            format!("via {} ( {} )", db.library.via(*via).name, point_text(*at))
        }
        RouteElem::Patch { layer, rect } => format!(
            "patch {} ( {} ) ( {} )",
            db.library.layer(*layer).name,
            point_text(rect.min),
            point_text(rect.max)
        ),
    }
}

fn wiring_text(db: &NetlistDB) -> String {
    let mut text = String::new();
    for net in &db.nets {
        let _ = writeln!(text, "net {} {}", net.name, net.routing.len());
        for elem in &net.routing {
            let _ = writeln!(text, "  {}", elem_text(db, elem));
        }
    }
    text
}

fn is_round(line: &str) -> bool {
    line.starts_with("round ")
}

/// The lines of `text` that `keep` accepts.
fn lines_where(text: &str, keep: impl Fn(&str) -> bool) -> String {
    let mut kept = String::new();
    for line in text.lines().filter(|l| keep(l)) {
        kept.push_str(line);
        kept.push('\n');
    }
    kept
}

#[test]
fn global_routes_of_the_placed_fixture_match_the_golden() {
    let (_, global, progress) = routed_tiny();
    let mut text = Vec::new();
    global.write(&mut text).unwrap();
    let loads = lines_where(&progress, |l| !is_round(l));

    let actual = String::from_utf8(text).unwrap() + &loads;

    assert_golden(&golden_dir(), "tiny_global.txt", &actual);
}

#[test]
fn detailed_routes_of_the_placed_fixture_match_the_golden() {
    let (db, _, progress) = routed_tiny();
    let rounds = lines_where(&progress, is_round);

    let actual = rounds + &wiring_text(&db);

    assert_golden(&golden_dir(), "tiny_detailed.txt", &actual);
}
