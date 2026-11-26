use super::*;
use crate::access::Way;

fn node(layer: usize, track: u32, cross: u32) -> Node {
    Node {
        layer: LayerId::new(layer),
        track: TrackIdx(track),
        cross: CrossIdx(cross),
    }
}

fn state() -> RouteState {
    let route = NetRoute {
        edges: [
            Edge::Wire(node(2, 4, 7)),
            Edge::Wire(node(2, 4, 8)),
            Edge::Via {
                lower: node(2, 4, 8),
                upper: node(3, 9, 1),
            },
        ]
        .into_iter()
        .collect(),
        patches: vec![Patch {
            layer: LayerId::new(2),
            rect: Rect::new(
                Point::new(Dbu::new(-5), Dbu::new(10)),
                Point::new(Dbu::new(15), Dbu::new(40)),
            ),
        }],
        access: vec![AccessPoint {
            node: node(2, 4, 7),
            at: Point::new(Dbu::new(100), Dbu::new(-3)),
            pin_layer: LayerId::new(1),
            vias: vec![ViaId::new(3)],
            ways: Ways::NONE.with(Way::AlongLower).with(Way::Up),
        }],
        complete: true,
    };
    RouteState {
        design: "tiny design".to_string(),
        grid: GridShape {
            gcell: Dbu::new(5700),
            nx: 12,
            ny: 9,
            min_layer: LayerId::new(2),
            max_layer: LayerId::new(9),
        },
        global: vec![(
            "a".to_string(),
            vec![vec![
                GCell {
                    layer: LayerId::new(2),
                    x: 1,
                    y: 2,
                },
                GCell {
                    layer: LayerId::new(2),
                    x: 2,
                    y: 2,
                },
            ]],
        )],
        detailed: Some(DetailedState {
            round: 7,
            scope: Scope::NearConflicts,
            flow: Flow::Stubborn,
            effective: false,
            routes: vec![("a".to_string(), route)],
        }),
    }
}

#[test]
fn a_state_reads_back_exactly_as_written() {
    let saved = state();
    let mut text = Vec::new();

    saved.write(&mut text).unwrap();
    let read = RouteState::parse(std::str::from_utf8(&text).unwrap()).unwrap();

    assert_eq!(read, saved);
}

#[test]
fn a_route_s_jogs_read_back_as_written() {
    let mut saved = state();
    let detailed = saved.detailed.as_mut().unwrap();
    detailed.routes[0]
        .1
        .edges
        .extend([Edge::Jog(node(2, 4, 7))]);
    let mut text = Vec::new();
    saved.write(&mut text).unwrap();

    let read = RouteState::parse(&String::from_utf8(text).unwrap()).unwrap();

    assert_eq!(read, saved);
}

#[test]
fn a_state_saved_after_global_routing_has_no_detailed_part() {
    let saved = RouteState {
        detailed: None,
        ..state()
    };
    let mut text = Vec::new();

    saved.write(&mut text).unwrap();
    let read = RouteState::parse(std::str::from_utf8(&text).unwrap()).unwrap();

    assert_eq!(read, saved);
    assert!(!read.has_detailed());
}

#[test]
fn a_truncated_state_names_the_line_it_ends_at() {
    let mut text = Vec::new();
    state().write(&mut text).unwrap();
    let text = String::from_utf8(text).unwrap();
    let cut: String = text.lines().take(6).collect::<Vec<_>>().join("\n");

    let error = RouteState::parse(&cut).unwrap_err();

    assert_eq!(error.line, 7);
}

/// The lines of [`state`] as written: line 3 is the grid, 7 the
/// detailed header, 8 the net's `r` line and 9 its run.
fn state_lines() -> Vec<String> {
    let mut text = Vec::new();
    state().write(&mut text).unwrap();
    String::from_utf8(text)
        .unwrap()
        .lines()
        .map(str::to_string)
        .collect()
}

/// The error of reading [`state`] with line `number` (from one) as
/// `replacement`.
fn error_with_line(number: usize, replacement: &str) -> StateParseError {
    let mut lines = state_lines();
    lines[number - 1] = replacement.to_string();
    RouteState::parse(&(lines.join("\n") + "\n")).unwrap_err()
}

#[test]
fn single_wire_lines_of_older_states_read_as_the_run_they_make() {
    let mut lines = state_lines();
    lines[7] = "r a 1 3 1 1".to_string();
    lines[8] = "w 2 4 7".to_string();
    lines.insert(9, "w 2 4 8".to_string());

    let read = RouteState::parse(&(lines.join("\n") + "\n")).unwrap();

    assert_eq!(read, state());
}

#[test]
fn an_unknown_scope_is_refused_at_its_line() {
    let error = error_with_line(7, "detailed 7 sideways stubborn 0 1");

    assert_eq!(error.line, 7);
    assert!(error.reason.contains("unknown scope"), "{error}");
}

#[test]
fn an_unknown_flow_is_refused_at_its_line() {
    let error = error_with_line(7, "detailed 7 near drifting 0 1");

    assert_eq!(error.line, 7);
    assert!(error.reason.contains("unknown flow"), "{error}");
}

#[test]
fn a_flag_other_than_0_or_1_is_refused_at_its_line() {
    let error = error_with_line(8, "r a 2 2 1 1");

    assert_eq!(error.line, 8);
    assert!(error.reason.contains("must be 0 or 1"), "{error}");
}

#[test]
fn a_line_with_extra_fields_is_refused_at_its_line() {
    let error = error_with_line(3, "grid 5700 12 9 2 9 4");

    assert_eq!(error.line, 3);
    assert!(error.reason.contains("extra fields"), "{error}");
}

#[test]
fn a_run_that_ends_where_it_starts_is_refused_at_its_line() {
    let error = error_with_line(9, "s 2 4 7 7");

    assert_eq!(error.line, 9);
    assert!(error.reason.contains("must end after"), "{error}");
}

#[test]
fn an_unknown_edge_is_refused_at_its_line() {
    let error = error_with_line(9, "x 2 4 7");

    assert_eq!(error.line, 9);
    assert!(error.reason.contains("unknown edge"), "{error}");
}
