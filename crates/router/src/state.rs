//! A routing run saved part way, to resume from.
//!
//! The state holds every net's global route and, once detailed routing has
//! begun, every net's detailed route and how far the repair rounds had got,
//! so that a resumed run goes on exactly as the saved one would have: the
//! next round's costs follow from its number, its flow from the last
//! round's, and its marks from the violations the routes have.
//!
//! The file is plain text, one record a line.

use crate::access::{AccessPoint, Node};
use crate::detailed::{Flow, Scope};
use crate::global::GCell;
use crate::runs::{EdgeSet, Run};
use crate::wiring::{Edge, NetRoute, Patch};
use pare_common::db::indices::{LayerId, ViaId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::io::{self, Write};

/// The first line of every state file, naming the format and its version.
const MAGIC: &str = "pare-route-state 1";

/// A routing run saved after global routing or after a detailed round.
#[derive(Clone, Debug, PartialEq)]
pub struct RouteState {
    pub(crate) design: String,
    pub(crate) grid: GridShape,
    /// Each globally routed net's name and the paths of its tree.
    pub(crate) global: Vec<(String, Vec<Vec<GCell>>)>,
    pub(crate) detailed: Option<DetailedState>,
}

/// What a saved global route must match to be reused: the gcell grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct GridShape {
    pub gcell: Dbu,
    pub nx: u16,
    pub ny: u16,
    pub min_layer: LayerId,
    pub max_layer: LayerId,
}

/// Detailed routing as it stood after one of its rounds.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct DetailedState {
    /// The round just finished; 0 is the first pass.
    pub round: usize,
    /// The scope the next round takes.
    pub scope: Scope,
    /// The flow the round took.
    pub flow: Flow,
    /// Whether the last guides or stubborn round reduced any worker's
    /// violations.
    pub effective: bool,
    /// Each routed net's name and route.
    pub routes: Vec<(String, NetRoute)>,
}

/// Why a state file could not be read.
#[derive(Debug, thiserror::Error)]
#[error("route state line {line}: {reason}")]
pub struct StateParseError {
    line: usize,
    reason: String,
}

impl RouteState {
    /// Whether the state holds detailed routes, not only global ones.
    pub const fn has_detailed(&self) -> bool {
        self.detailed.is_some()
    }

    /// The detailed round the state was saved after, if any.
    pub fn round(&self) -> Option<usize> {
        self.detailed.as_ref().map(|d| d.round)
    }

    /// Writes the state in its text format.
    pub fn write(&self, out: &mut impl Write) -> io::Result<()> {
        writeln!(out, "{MAGIC}")?;
        writeln!(out, "design {}", self.design)?;
        let g = self.grid;
        writeln!(
            out,
            "grid {} {} {} {} {}",
            g.gcell.get(),
            g.nx,
            g.ny,
            g.min_layer.index(),
            g.max_layer.index()
        )?;
        writeln!(out, "global {}", self.global.len())?;
        for (name, paths) in &self.global {
            writeln!(out, "n {name} {}", paths.len())?;
            for path in paths {
                write!(out, "c")?;
                for c in path {
                    write!(out, " {} {} {}", c.layer.index(), c.x, c.y)?;
                }
                writeln!(out)?;
            }
        }
        if let Some(d) = &self.detailed {
            write_detailed(out, d)?;
        }
        writeln!(out, "end")
    }

    /// Reads a state from its text format.
    pub fn parse(text: &str) -> Result<Self, StateParseError> {
        let mut lines = Lines::new(text);
        lines.expect_exact(MAGIC)?;
        let design = lines.keyword("design")?.rest();
        let mut grid_line = lines.keyword("grid")?;
        let grid = GridShape {
            gcell: Dbu(grid_line.int()?),
            nx: grid_line.int()?,
            ny: grid_line.int()?,
            min_layer: LayerId::new(grid_line.int()?),
            max_layer: LayerId::new(grid_line.int()?),
        };
        grid_line.done()?;
        let count: usize = lines.keyword("global")?.int()?;
        let mut global = Vec::with_capacity(count);
        for _ in 0..count {
            let mut net = lines.keyword("n")?;
            let name = net.word()?;
            let paths: usize = net.int()?;
            net.done()?;
            let mut tree = Vec::with_capacity(paths);
            for _ in 0..paths {
                let mut path_line = lines.keyword("c")?;
                let mut path = Vec::new();
                while !path_line.is_done() {
                    path.push(GCell {
                        layer: LayerId::new(path_line.int()?),
                        x: path_line.int()?,
                        y: path_line.int()?,
                    });
                }
                tree.push(path);
            }
            global.push((name, tree));
        }
        let next = lines.next_line()?;
        let detailed = if next.starts_with("detailed ") {
            Some(parse_detailed(&mut lines, next)?)
        } else {
            lines.back();
            None
        };
        lines.expect_exact("end")?;
        Ok(Self {
            design,
            grid,
            global,
            detailed,
        })
    }
}

fn write_detailed(out: &mut impl Write, d: &DetailedState) -> io::Result<()> {
    let scope = match d.scope {
        Scope::Conflicts => "conflicts",
        Scope::NearConflicts => "near",
    };
    writeln!(
        out,
        "detailed {} {scope} {} {} {}",
        d.round,
        d.flow.name(),
        u8::from(d.effective),
        d.routes.len()
    )?;
    for (name, route) in &d.routes {
        writeln!(
            out,
            "r {name} {} {} {} {}",
            u8::from(route.complete),
            route.edges.runs().len() + route.edges.vias().len(),
            route.patches.len(),
            route.access.len()
        )?;
        for run in route.edges.runs() {
            writeln!(out, "s {} {}", node_text(run.node(run.from)), run.to)?;
        }
        for &(lower, upper) in route.edges.vias() {
            writeln!(out, "v {} {}", node_text(lower), node_text(upper))?;
        }
        for p in &route.patches {
            let r = p.rect;
            writeln!(
                out,
                "p {} {} {} {} {}",
                p.layer.index(),
                r.min.x.get(),
                r.min.y.get(),
                r.max.x.get(),
                r.max.y.get()
            )?;
        }
        for a in &route.access {
            write!(
                out,
                "a {} {} {} {}",
                node_text(a.node),
                a.at.x.get(),
                a.at.y.get(),
                a.pin_layer.index()
            )?;
            for v in &a.vias {
                write!(out, " {}", v.index())?;
            }
            writeln!(out)?;
        }
    }
    Ok(())
}

fn node_text(n: Node) -> String {
    format!("{} {} {}", n.layer.index(), n.track, n.cross)
}

fn parse_detailed(lines: &mut Lines<'_>, header: &str) -> Result<DetailedState, StateParseError> {
    let mut head = lines.fields(header);
    if head.word()? != "detailed" {
        return Err(head.error("expected a `detailed` line"));
    }
    let round: usize = head.int()?;
    let scope = match head.word()?.as_str() {
        "conflicts" => Scope::Conflicts,
        "near" => Scope::NearConflicts,
        other => return Err(head.error(&format!("unknown scope `{other}`"))),
    };
    let flow_name = head.word()?;
    let flow = Flow::from_name(&flow_name)
        .ok_or_else(|| head.error(&format!("unknown flow `{flow_name}`")))?;
    let effective = match head.int::<u8>()? {
        0 => false,
        1 => true,
        _ => return Err(head.error("effectiveness must be 0 or 1")),
    };
    let count: usize = head.int()?;
    head.done()?;
    let mut routes = Vec::with_capacity(count);
    for _ in 0..count {
        let mut r = lines.keyword("r")?;
        let name = r.word()?;
        let complete = match r.int::<u8>()? {
            0 => false,
            1 => true,
            _ => return Err(r.error("completeness must be 0 or 1")),
        };
        let (edges, patches, access): (usize, usize, usize) = (r.int()?, r.int()?, r.int()?);
        r.done()?;
        let mut route = NetRoute {
            complete,
            ..NetRoute::default()
        };
        let mut runs = Vec::new();
        let mut steps = Vec::new();
        for _ in 0..edges {
            let line = lines.next_line()?;
            let mut e = lines.fields(line);
            match e.word()?.as_str() {
                "s" => {
                    let first = e.node()?;
                    let to: u32 = e.int()?;
                    if to <= first.cross {
                        return Err(e.error("a run must end after its first crossing"));
                    }
                    runs.push(Run {
                        layer: first.layer,
                        track: first.track,
                        from: first.cross,
                        to,
                    });
                }
                "w" => steps.push(Edge::Wire(e.node()?)),
                "v" => steps.push(Edge::Via {
                    lower: e.node()?,
                    upper: e.node()?,
                }),
                other => return Err(e.error(&format!("unknown edge `{other}`"))),
            }
            e.done()?;
        }
        route.edges = EdgeSet::from_runs(runs);
        route.edges.extend(steps);
        for _ in 0..patches {
            let mut p = lines.keyword("p")?;
            let layer = LayerId::new(p.int()?);
            let (x0, y0, x1, y1) = (p.int()?, p.int()?, p.int()?, p.int()?);
            p.done()?;
            route.patches.push(Patch {
                layer,
                rect: Rect::new(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1))),
            });
        }
        for _ in 0..access {
            let mut a = lines.keyword("a")?;
            let node = a.node()?;
            let at = Point::new(Dbu(a.int()?), Dbu(a.int()?));
            let pin_layer = LayerId::new(a.int()?);
            let mut vias = Vec::new();
            while !a.is_done() {
                vias.push(ViaId::new(a.int()?));
            }
            route.access.push(AccessPoint {
                node,
                at,
                pin_layer,
                vias,
            });
        }
        routes.push((name, route));
    }
    Ok(DetailedState {
        round,
        scope,
        flow,
        effective,
        routes,
    })
}

/// The lines of a state file, read one at a time with their numbers.
struct Lines<'a> {
    lines: Vec<&'a str>,
    at: usize,
}

impl<'a> Lines<'a> {
    fn new(text: &'a str) -> Self {
        Self {
            lines: text.lines().collect(),
            at: 0,
        }
    }

    fn next_line(&mut self) -> Result<&'a str, StateParseError> {
        let line = self
            .lines
            .get(self.at)
            .copied()
            .ok_or_else(|| StateParseError {
                line: self.at + 1,
                reason: "the file ends early".to_string(),
            })?;
        self.at += 1;
        Ok(line)
    }

    const fn back(&mut self) {
        self.at -= 1;
    }

    fn expect_exact(&mut self, want: &str) -> Result<(), StateParseError> {
        let line = self.next_line()?;
        if line == want {
            Ok(())
        } else {
            Err(StateParseError {
                line: self.at,
                reason: format!("expected `{want}`"),
            })
        }
    }

    /// The next line, which must start with `keyword`, with its fields after
    /// the keyword.
    fn keyword(&mut self, keyword: &str) -> Result<Fields<'a>, StateParseError> {
        let line = self.next_line()?;
        let mut fields = self.fields(line);
        if fields.word()? != keyword {
            return Err(fields.error(&format!("expected a `{keyword}` line")));
        }
        Ok(fields)
    }

    fn fields(&self, line: &'a str) -> Fields<'a> {
        Fields {
            words: line.split_ascii_whitespace(),
            line: self.at,
        }
    }
}

/// The fields of one line.
struct Fields<'a> {
    words: std::str::SplitAsciiWhitespace<'a>,
    line: usize,
}

impl Fields<'_> {
    fn error(&self, reason: &str) -> StateParseError {
        StateParseError {
            line: self.line,
            reason: reason.to_string(),
        }
    }

    fn word(&mut self) -> Result<String, StateParseError> {
        self.words
            .next()
            .map(str::to_string)
            .ok_or_else(|| self.error("a field is missing"))
    }

    fn int<T: std::str::FromStr>(&mut self) -> Result<T, StateParseError> {
        let word = self.word()?;
        word.parse()
            .map_err(|_| self.error(&format!("`{word}` is not a valid number here")))
    }

    fn node(&mut self) -> Result<Node, StateParseError> {
        Ok(Node {
            layer: LayerId::new(self.int()?),
            track: self.int()?,
            cross: self.int()?,
        })
    }

    fn rest(mut self) -> String {
        self.words.by_ref().collect::<Vec<_>>().join(" ")
    }

    fn is_done(&self) -> bool {
        self.words.clone().next().is_none()
    }

    fn done(&mut self) -> Result<(), StateParseError> {
        if self.words.next().is_some() {
            return Err(self.error("the line has extra fields"));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(layer: usize, track: u32, cross: u32) -> Node {
        Node {
            layer: LayerId::new(layer),
            track,
            cross,
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
                rect: Rect::new(Point::new(Dbu(-5), Dbu(10)), Point::new(Dbu(15), Dbu(40))),
            }],
            access: vec![AccessPoint {
                node: node(2, 4, 7),
                at: Point::new(Dbu(100), Dbu(-3)),
                pin_layer: LayerId::new(1),
                vias: vec![ViaId::new(3)],
            }],
            complete: true,
        };
        RouteState {
            design: "tiny design".to_string(),
            grid: GridShape {
                gcell: Dbu(5700),
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
}
