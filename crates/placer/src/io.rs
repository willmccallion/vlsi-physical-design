//! Placement of top-level I/O pins on the die boundary, as `OpenROAD`'s
//! `place_pins` does it.
//!
//! Every edge offers slots on one layer whose wires cross it: the vertical
//! layer on the top and bottom edges, the horizontal one on the left and
//! right. Slots lie every other track, clear of the corners and of fixed
//! metal there. Each pin goes to the section of slots whose middle makes
//! its net shortest, or the next best one with room, and the pins of each
//! section are then matched to its slots by the Hungarian method, so that
//! together their nets' half-perimeter wirelength is least. Fixed pins stay
//! where they are and keep their slots.

use pare_common::db::design::{IoPin, NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{IoPinId, LayerId};
use pare_common::db::library::{Direction, LayerRef, Library, Shape};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::util::config::IoPlacementConfig;
use pare_common::verify::shapes::fixed_shapes_keeping_io;
use rayon::prelude::*;

/// Slots in one section of an edge.
const SLOTS_PER_SECTION: usize = 200;

/// Fewest tracks between neighbouring slots.
const SLOT_TRACKS: i64 = 2;

/// Tracks kept free at each corner, up to [`MAX_CORNER_MICRONS`].
const CORNER_TRACKS: i64 = 15;

/// Largest distance kept free at each corner.
const MAX_CORNER_MICRONS: f64 = 1.0;

/// Why pins could not be placed.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum IoPlacementError {
    /// No layer is configured for one direction.
    #[error(
        "io_placement.{key} is not set; it names the routing layer for pins on the {edges} edges"
    )]
    LayerNotSet {
        /// The missing configuration key.
        key: &'static str,
        /// The edges that layer serves.
        edges: &'static str,
    },
    /// A configured layer is not a routing layer.
    #[error("unknown routing layer '{0}'")]
    UnknownLayer(String),
    /// A configured layer runs the wrong way.
    #[error("layer '{layer}' is not {direction}")]
    WrongDirection {
        /// The layer.
        layer: String,
        /// The direction it should have.
        direction: &'static str,
    },
    /// The design has no tracks on a pin layer.
    #[error("the design defines no tracks for layer '{0}'")]
    NoTracks(String),
    /// The boundary has fewer free slots than pins to place.
    #[error("{pins} I/O pins to place but only {slots} free slots on the die boundary")]
    TooFewSlots {
        /// Pins to place.
        pins: usize,
        /// Free slots.
        slots: usize,
    },
}

/// The layers pins are placed on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IoLayers {
    /// Layer of pins on the left and right edges.
    pub horizontal: LayerId,
    /// Layer of pins on the top and bottom edges.
    pub vertical: LayerId,
}

impl IoLayers {
    /// The configured layers, checked against the library.
    pub fn from_config(
        config: &IoPlacementConfig,
        lib: &Library,
    ) -> Result<Self, IoPlacementError> {
        let resolve = |name: &Option<String>, key, edges, direction| {
            let name = name
                .as_deref()
                .ok_or(IoPlacementError::LayerNotSet { key, edges })?;
            let id = lib
                .routing_layer_id(name)
                .ok_or_else(|| IoPlacementError::UnknownLayer(name.to_string()))?;
            if lib.layer(id).direction != direction {
                return Err(IoPlacementError::WrongDirection {
                    layer: name.to_string(),
                    direction: match direction {
                        Direction::Horizontal => "horizontal",
                        Direction::Vertical => "vertical",
                    },
                });
            }
            Ok(id)
        };
        Ok(Self {
            horizontal: resolve(
                &config.horizontal_layer,
                "horizontal_layer",
                "left and right",
                Direction::Horizontal,
            )?,
            vertical: resolve(
                &config.vertical_layer,
                "vertical_layer",
                "top and bottom",
                Direction::Vertical,
            )?,
        })
    }
}

/// What pin placement did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IoPlacement {
    /// Pins placed.
    pub pins: usize,
    /// Free slots they were chosen from.
    pub slots: usize,
}

/// One side of the die.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Edge {
    Bottom,
    Right,
    Top,
    Left,
}

impl Edge {
    const ALL: [Self; 4] = [Self::Bottom, Self::Right, Self::Top, Self::Left];

    const fn runs_horizontally(self) -> bool {
        matches!(self, Self::Bottom | Self::Top)
    }
}

/// The size of the pin shape on a layer: its half width along the edge and
/// its length into the die.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PinSize {
    half_width: Dbu,
    length: Dbu,
}

impl PinSize {
    /// The layer's wire width, as long as it is wide; the library holds no
    /// minimum area that would make it longer.
    fn of(lib: &Library, layer: LayerId) -> Self {
        let width = lib.layer(layer).width;
        let half_width = Dbu((width.get() + 1) / 2);
        Self {
            half_width,
            length: half_width * 2,
        }
    }

    /// The pin rectangle at `at` on `edge`, pointing into the die.
    fn rect(self, edge: Edge, at: Point<Dbu>) -> Rect<Dbu> {
        let (w, l) = (self.half_width, self.length);
        let (lo, hi) = match edge {
            Edge::Bottom => (Point::new(-w, Dbu::ZERO), Point::new(w, l)),
            Edge::Top => (Point::new(-w, -l), Point::new(w, Dbu::ZERO)),
            Edge::Left => (Point::new(Dbu::ZERO, -w), Point::new(l, w)),
            Edge::Right => (Point::new(-l, -w), Point::new(Dbu::ZERO, w)),
        };
        Rect::new(at + lo, at + hi)
    }
}

/// A place on the boundary a pin may take.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Slot {
    edge: Edge,
    layer: LayerId,
    at: Point<Dbu>,
}

/// Neighbouring slots of one edge that pins are first assigned to.
#[derive(Debug)]
struct Section {
    slots: Vec<Slot>,
    pins: Vec<IoPinId>,
}

impl Section {
    fn middle(&self) -> Point<Dbu> {
        self.slots[self.slots.len() / 2].at
    }

    const fn has_room(&self) -> bool {
        self.pins.len() < self.slots.len()
    }
}

/// The I/O pins placement may move.
pub fn movable_io_pins(db: &NetlistDB) -> Vec<IoPinId> {
    (0..db.io_pins.len())
        .filter(|&i| db.io_pins[i].status.is_movable())
        .map(IoPinId::new)
        .collect()
}

/// Places every movable I/O pin on a free slot of the die boundary,
/// minimizing the wirelength of their nets for the current cell positions.
pub fn place_io_pins(
    db: &mut NetlistDB,
    layers: IoLayers,
) -> Result<IoPlacement, IoPlacementError> {
    let pins = movable_io_pins(db);
    if pins.is_empty() {
        return Ok(IoPlacement { pins: 0, slots: 0 });
    }
    let slots = free_slots(db, layers)?;
    if slots.len() < pins.len() {
        return Err(IoPlacementError::TooFewSlots {
            pins: pins.len(),
            slots: slots.len(),
        });
    }
    let boxes: Vec<Option<Rect<Dbu>>> = pins.iter().map(|&p| net_box(db, p)).collect();
    let mut sections = sections(&slots);
    for (k, &pin) in pins.iter().enumerate() {
        let mut order: Vec<usize> = (0..sections.len()).collect();
        order.sort_by_key(|&s| (hpwl_with(boxes[k], sections[s].middle()), s));
        let chosen = order.into_iter().find(|&s| sections[s].has_room()).ok_or(
            IoPlacementError::TooFewSlots {
                pins: pins.len(),
                slots: slots.len(),
            },
        )?;
        sections[chosen].pins.push(pin);
    }

    let box_of = |pin: IoPinId| pins.binary_search(&pin).ok().and_then(|k| boxes[k]);
    let placed: Vec<(IoPinId, Slot)> = sections
        .par_iter()
        .flat_map_iter(|section| {
            let costs: Vec<Vec<i64>> = section
                .pins
                .iter()
                .map(|&pin| {
                    let net = box_of(pin);
                    section.slots.iter().map(|s| hpwl_with(net, s.at)).collect()
                })
                .collect();
            let assignment = min_cost_assignment(&costs);
            section
                .pins
                .iter()
                .zip(assignment)
                .map(|(&pin, slot)| (pin, section.slots[slot]))
                .collect::<Vec<_>>()
        })
        .collect();

    for (pin, slot) in placed {
        let size = PinSize::of(&db.library, slot.layer);
        let io = &mut db.io_pins[pin.index()];
        io.location = slot.at;
        io.orient = Orientation::N;
        io.shapes = vec![Shape {
            layer: LayerRef::Routing(slot.layer),
            rect: size.rect(slot.edge, Point::default()),
        }];
        io.status = PlacementStatus::Placed;
    }
    Ok(IoPlacement {
        pins: pins.len(),
        slots: slots.len(),
    })
}

/// Half-perimeter of `net` grown to take in `p`; a net with nothing else
/// to reach costs nothing.
fn hpwl_with(net: Option<Rect<Dbu>>, p: Point<Dbu>) -> i64 {
    net.map_or(0, |r| {
        let b = r.union(&Rect::new(p, p));
        (b.width() + b.height()).get()
    })
}

/// The box around the other connections of a pin's net that do not move
/// with it: its cells' pins and its fixed I/O pins.
fn net_box(db: &NetlistDB, pin: IoPinId) -> Option<Rect<Dbu>> {
    let net = db.io_pins[pin.index()].net;
    db.nets[net.index()]
        .pins
        .iter()
        .filter(|&&p| match db.pins[p.index()].owner {
            PinOwner::Cell { .. } => true,
            PinOwner::Io(io) => !db.io_pins[io.index()].status.is_movable(),
        })
        .map(|&p| {
            let at = db.pin_position(p);
            Rect::new(at, at)
        })
        .reduce(|a, b| a.union(&b))
}

/// Splits each edge's slots, in order along it, into sections.
fn sections(slots: &[Slot]) -> Vec<Section> {
    let mut out = Vec::new();
    for edge in Edge::ALL {
        let on_edge: Vec<Slot> = slots.iter().copied().filter(|s| s.edge == edge).collect();
        out.extend(on_edge.chunks(SLOTS_PER_SECTION).map(|chunk| Section {
            slots: chunk.to_vec(),
            pins: Vec::new(),
        }));
    }
    out
}

/// Every slot of every edge that fixed metal leaves free.
fn free_slots(db: &NetlistDB, layers: IoLayers) -> Result<Vec<Slot>, IoPlacementError> {
    let blockages = fixed_shapes_keeping_io(db, |p: &IoPin| !p.status.is_movable());
    let mut out = Vec::new();
    for edge in Edge::ALL {
        let layer = if edge.runs_horizontally() {
            layers.vertical
        } else {
            layers.horizontal
        };
        let size = PinSize::of(&db.library, layer);
        let rules = &db.library.layer(layer);
        let on_layer: Vec<Rect<Dbu>> = blockages
            .iter()
            .filter(|s| s.shape.layer == LayerRef::Routing(layer))
            .map(|s| s.shape.rect)
            .collect();
        for at in edge_positions(db, layer, edge, size)? {
            let pin = size.rect(edge, at);
            let blocked = on_layer.iter().any(|r| {
                let width = r.width().min(r.height()).max(size.half_width * 2);
                let spacing = rules
                    .spacing_table
                    .as_ref()
                    .map_or(rules.min_spacing, |t| t.spacing(width, size.length))
                    .max(rules.min_spacing);
                too_close(&pin, r, spacing)
            });
            if !blocked {
                out.push(Slot { edge, layer, at });
            }
        }
    }
    Ok(out)
}

/// Whether two rectangles overlap or come closer than `spacing`.
fn too_close(a: &Rect<Dbu>, b: &Rect<Dbu>, spacing: Dbu) -> bool {
    let gap = |lo_a: Dbu, hi_a: Dbu, lo_b: Dbu, hi_b: Dbu| {
        i128::from((lo_a - hi_b).max(lo_b - hi_a).max(Dbu::ZERO).get())
    };
    let dx = gap(a.min.x, a.max.x, b.min.x, b.max.x);
    let dy = gap(a.min.y, a.max.y, b.min.y, b.max.y);
    let s = i128::from(spacing.get());
    dx * dx + dy * dy < s * s
}

/// Slot positions on `edge`: every [`SLOT_TRACKS`]th track of `layer`, or
/// further apart if pins that close would break the spacing, leaving the
/// pin inside the die and the corners free.
fn edge_positions(
    db: &NetlistDB,
    layer: LayerId,
    edge: Edge,
    size: PinSize,
) -> Result<Vec<Point<Dbu>>, IoPlacementError> {
    let die = db.die_area;
    let axis = if edge.runs_horizontally() {
        pare_common::db::design::TrackAxis::X
    } else {
        pare_common::db::design::TrackAxis::Y
    };
    let tracks = db
        .tracks
        .iter()
        .find(|t| t.axis == axis && t.layers.contains(&layer))
        .ok_or_else(|| IoPlacementError::NoTracks(db.library.layer(layer).name.clone()))?;
    let (lo, hi, fixed) = match edge {
        Edge::Bottom => (die.min.x, die.max.x, die.min.y),
        Edge::Top => (die.min.x, die.max.x, die.max.y),
        Edge::Left => (die.min.y, die.max.y, die.min.x),
        Edge::Right => (die.min.y, die.max.y, die.max.x),
    };
    let max_corner = db.dbu().to_dbu(MAX_CORNER_MICRONS).unwrap_or(Dbu::ZERO);
    let corner = (tracks.step * CORNER_TRACKS).min(max_corner);
    let from = lo + size.half_width + corner;
    let to = hi - size.half_width - corner;
    let apart = (size.half_width * 2 + db.library.layer(layer).min_spacing).get();
    let step = tracks.step.get().max(1);
    let slot_tracks = SLOT_TRACKS.max((apart + step - 1) / step);
    let out = tracks
        .coords()
        .step_by(usize::try_from(slot_tracks).unwrap_or(1))
        .filter(|&c| from <= c && c <= to)
        .map(|c| {
            if edge.runs_horizontally() {
                Point::new(c, fixed)
            } else {
                Point::new(fixed, c)
            }
        })
        .collect();
    Ok(out)
}

/// The column assigned to each row of `costs` so that the total is least,
/// by the Hungarian method with potentials; every row gets a distinct
/// column, so there must be at least as many columns as rows.
fn min_cost_assignment(costs: &[Vec<i64>]) -> Vec<usize> {
    const INFINITE: i64 = i64::MAX / 4;
    let rows = costs.len();
    let cols = costs.first().map_or(0, Vec::len);
    if rows == 0 {
        return Vec::new();
    }
    let mut row_potential = vec![0_i64; rows + 1];
    let mut col_potential = vec![0_i64; cols + 1];
    let mut matched_row = vec![0_usize; cols + 1];
    let mut previous = vec![0_usize; cols + 1];
    for row in 1..=rows {
        matched_row[0] = row;
        let mut col = 0;
        let mut slack = vec![INFINITE; cols + 1];
        let mut done = vec![false; cols + 1];
        loop {
            done[col] = true;
            let r = matched_row[col];
            let mut delta = INFINITE;
            let mut next = 0;
            for c in 1..=cols {
                if done[c] {
                    continue;
                }
                let reduced = costs[r - 1][c - 1] - row_potential[r] - col_potential[c];
                if reduced < slack[c] {
                    slack[c] = reduced;
                    previous[c] = col;
                }
                if slack[c] < delta {
                    delta = slack[c];
                    next = c;
                }
            }
            for c in 0..=cols {
                if done[c] {
                    row_potential[matched_row[c]] += delta;
                    col_potential[c] -= delta;
                } else {
                    slack[c] -= delta;
                }
            }
            col = next;
            if matched_row[col] == 0 {
                break;
            }
        }
        while col != 0 {
            let before = previous[col];
            matched_row[col] = matched_row[before];
            col = before;
        }
    }
    let mut out = vec![0; rows];
    for c in 1..=cols {
        if matched_row[c] != 0 {
            out[matched_row[c] - 1] = c - 1;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn total(costs: &[Vec<i64>], assignment: &[usize]) -> i64 {
        assignment
            .iter()
            .enumerate()
            .map(|(r, &c)| costs[r][c])
            .sum()
    }

    #[test]
    fn assignment_finds_the_cheapest_matching() {
        let costs = vec![vec![4, 1, 3], vec![2, 0, 5], vec![3, 2, 2]];

        let assignment = min_cost_assignment(&costs);

        assert_eq!(total(&costs, &assignment), 5);
    }

    #[test]
    fn assignment_gives_fewer_rows_distinct_columns() {
        let costs = vec![vec![7, 1, 1, 9], vec![7, 1, 2, 9]];

        let assignment = min_cost_assignment(&costs);

        assert_ne!(assignment[0], assignment[1]);
        assert_eq!(total(&costs, &assignment), 2);
    }

    #[test]
    fn assignment_matches_brute_force_on_small_matrices() {
        let mut seed = 12_345_u64;
        let mut next = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            i64::try_from(seed >> 54).unwrap()
        };
        for _ in 0..50 {
            let costs: Vec<Vec<i64>> = (0..4).map(|_| (0..5).map(|_| next()).collect()).collect();

            let best = brute_force(&costs, 0, &mut [false; 5]);

            assert_eq!(total(&costs, &min_cost_assignment(&costs)), best);
        }
    }

    fn brute_force(costs: &[Vec<i64>], row: usize, used: &mut [bool; 5]) -> i64 {
        if row == costs.len() {
            return 0;
        }
        let mut best = i64::MAX;
        for c in 0..5 {
            if !used[c] {
                used[c] = true;
                best = best.min(costs[row][c] + brute_force(costs, row + 1, used));
                used[c] = false;
            }
        }
        best
    }

    #[test]
    fn pin_rectangles_point_into_the_die() {
        let size = PinSize {
            half_width: Dbu(70),
            length: Dbu(140),
        };
        let at = Point::new(Dbu(1000), Dbu(0));

        let r = size.rect(Edge::Bottom, at);

        assert_eq!(
            r,
            Rect::new(
                Point::new(Dbu(930), Dbu(0)),
                Point::new(Dbu(1070), Dbu(140))
            )
        );
    }
}
