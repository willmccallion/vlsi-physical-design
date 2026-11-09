//! Placement of top-level I/O pins on the die boundary, as `OpenROAD`'s
//! `place_pins` does it.
//!
//! Every edge offers slots on one layer whose wires cross it: the vertical
//! layer on the top and bottom edges, the horizontal one on the left and
//! right. Slots lie every other track, clear of the corners, and run around
//! the die from the bottom left, counterclockwise; those on fixed metal or
//! a macro at the edge count as blocked. Each pin goes to the section of
//! slots whose middle makes its net shortest, or the next best one with
//! room, and the pins of each section are then matched to its free slots by
//! the Hungarian method, so that together their nets' half-perimeter
//! wirelength is least. Fixed pins stay where they are and keep their slots.

use crate::munkres::min_cost_assignment;
use pare_common::config::IoLayerNames;
use pare_common::db::design::{IoPin, NetlistDB, PinOwner, PlacementStatus, TrackAxis, TrackDef};
use pare_common::db::indices::{CellId, IoPinId, LayerId};
use pare_common::db::library::{Direction, LayerRef, Library, MacroClass, Shape};
use pare_common::db::shapes::fixed_shapes_keeping_io;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
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
    /// A configured layer is not a routing layer.
    #[error("unknown routing layer '{0}'")]
    UnknownLayer(String),
    /// A configured layer runs the wrong way.
    #[error("layer '{layer}' is not {direction}")]
    WrongDirection {
        /// The configured layer's name.
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
    /// The layers `names` names, checked against the library.
    pub fn from_names(names: &IoLayerNames, lib: &Library) -> Result<Self, IoPlacementError> {
        let resolve = |name: &str, direction| {
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
            horizontal: resolve(&names.horizontal, Direction::Horizontal)?,
            vertical: resolve(&names.vertical, Direction::Vertical)?,
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
    blocked: bool,
}

/// Neighbouring slots of one edge that pins are first assigned to.
#[derive(Debug)]
struct Section {
    slots: Vec<Slot>,
    free_slots: usize,
    pins: Vec<IoPinId>,
}

impl Section {
    fn new(slots: &[Slot]) -> Self {
        Self {
            slots: slots.to_vec(),
            free_slots: slots.iter().filter(|s| !s.blocked).count(),
            pins: Vec::new(),
        }
    }

    fn middle(&self) -> Point<Dbu> {
        self.slots[(self.slots.len() - 1) / 2].at
    }

    const fn has_room(&self) -> bool {
        self.pins.len() < self.free_slots
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
    let slots = boundary_slots(db, layers)?;
    let free_slots = slots.iter().filter(|s| !s.blocked).count();
    if free_slots < pins.len() {
        return Err(IoPlacementError::TooFewSlots {
            pins: pins.len(),
            slots: free_slots,
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
                slots: free_slots,
            },
        )?;
        sections[chosen].pins.push(pin);
    }

    let box_of = |pin: IoPinId| pins.binary_search(&pin).ok().and_then(|k| boxes[k]);
    let placed: Vec<(IoPinId, Slot)> = sections
        .par_iter()
        .flat_map_iter(|section| match_to_slots(section, &box_of))
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
        slots: free_slots,
    })
}

/// The section's pins matched to its free slots, as `HungarianMatching`.
fn match_to_slots(
    section: &Section,
    box_of: &impl Fn(IoPinId) -> Option<Rect<Dbu>>,
) -> Vec<(IoPinId, Slot)> {
    let free: Vec<Slot> = section
        .slots
        .iter()
        .filter(|s| !s.blocked)
        .copied()
        .collect();
    let costs: Vec<Vec<i64>> = free
        .iter()
        .map(|s| {
            section
                .pins
                .iter()
                .map(|&pin| hpwl_with(box_of(pin), s.at))
                .collect()
        })
        .collect();
    min_cost_assignment(&costs)
        .into_iter()
        .zip(free)
        .filter_map(|(pin, slot)| Some((section.pins[pin?], slot)))
        .collect()
}

/// Half-perimeter of `net` grown to take in `p`; a net with nothing else
/// to reach costs nothing.
fn hpwl_with(net: Option<Rect<Dbu>>, p: Point<Dbu>) -> i64 {
    net.map_or(0, |r| {
        let b = r.union(&Rect::new(p, p));
        (b.width() + b.height()).get()
    })
}

/// The box around the cell pins on a pin's net, where `IOPlacer::initNetlist`
/// takes them.
fn net_box(db: &NetlistDB, pin: IoPinId) -> Option<Rect<Dbu>> {
    let net = db.io_pins[pin.index()].net;
    db.nets[net.index()]
        .pins
        .iter()
        .filter(|&&p| matches!(db.pins[p.index()].owner, PinOwner::Cell { .. }))
        .map(|&p| {
            let at = db.average_pin_position(p);
            Rect::new(at, at)
        })
        .reduce(|a, b| a.union(&b))
}

/// Splits each edge's slots, in order along it, into sections, as
/// `IOPlacer::findSections`, which leaves out the last slot of an edge where
/// it would make a section alone.
fn sections(slots: &[Slot]) -> Vec<Section> {
    let mut out = Vec::new();
    for edge in Edge::ALL {
        let on_edge: Vec<Slot> = slots.iter().copied().filter(|s| s.edge == edge).collect();
        let Some(last) = on_edge.len().checked_sub(1) else {
            continue;
        };
        let mut begin = 0;
        loop {
            let end = (begin + SLOTS_PER_SECTION - 1).min(last);
            out.push(Section::new(&on_edge[begin..=end]));
            begin = end + 1;
            if begin >= last {
                break;
            }
        }
    }
    out
}

/// Every slot of every edge, blocked where fixed metal or a macro at the
/// edge is in the way.
fn boundary_slots(db: &NetlistDB, layers: IoLayers) -> Result<Vec<Slot>, IoPlacementError> {
    let blockages = fixed_shapes_keeping_io(db, |p: &IoPin| !p.status.is_movable());
    let macros = macro_stretches(db);
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
            let on_metal = on_layer.iter().any(|r| {
                let width = r.width().min(r.height()).max(size.half_width * 2);
                let spacing = rules
                    .spacing_table
                    .as_ref()
                    .map_or(rules.min_spacing, |t| {
                        t.spacing(width, size.half_width * 2, size.length)
                    })
                    .max(rules.min_spacing);
                too_close(&pin, r, spacing)
            });
            let along = if edge.runs_horizontally() { at.x } else { at.y };
            let on_macro = macros
                .iter()
                .any(|&(e, lo, hi)| e == edge && lo < along && along < hi);
            out.push(Slot {
                edge,
                layer,
                at,
                blocked: on_metal || on_macro,
            });
        }
    }
    Ok(out)
}

/// The stretches of each edge that placed macros reach, as
/// `IOPlacer::getBlockedRegionsFromMacros`.
fn macro_stretches(db: &NetlistDB) -> Vec<(Edge, Dbu, Dbu)> {
    (0..db.cells.len())
        .map(CellId::new)
        .filter(|&id| {
            db.cell_macro(id).class == MacroClass::Block
                && db.cells[id.index()].status != PlacementStatus::Unplaced
        })
        .filter_map(|id| db.die_area.intersection(&db.cell_bbox(id)))
        .flat_map(|reached| stretches_on_die_edges(db.die_area, reached))
        .collect()
}

/// The stretches of the die's edges that `b`, inside it, touches.
fn stretches_on_die_edges(die: Rect<Dbu>, b: Rect<Dbu>) -> Vec<(Edge, Dbu, Dbu)> {
    let mut out = Vec::new();
    if b.min.y == die.min.y {
        out.push((Edge::Bottom, b.min.x, b.max.x));
    }
    if b.max.y == die.max.y {
        out.push((Edge::Top, b.min.x, b.max.x));
    }
    if b.min.x == die.min.x {
        out.push((Edge::Left, b.min.y, b.max.y));
    }
    if b.max.x == die.max.x {
        out.push((Edge::Right, b.min.y, b.max.y));
    }
    out
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

/// Slot positions on `edge`, in `OpenROAD`'s order along it: rightward on
/// the bottom, up the right, leftward on the top and down the left.
fn edge_positions(
    db: &NetlistDB,
    layer: LayerId,
    edge: Edge,
    size: PinSize,
) -> Result<Vec<Point<Dbu>>, IoPlacementError> {
    let die = db.die_area;
    let axis = if edge.runs_horizontally() {
        TrackAxis::X
    } else {
        TrackAxis::Y
    };
    let (lo, hi, fixed) = match edge {
        Edge::Bottom => (die.min.x, die.max.x, die.min.y),
        Edge::Top => (die.min.x, die.max.x, die.max.y),
        Edge::Left => (die.min.y, die.max.y, die.min.x),
        Edge::Right => (die.min.y, die.max.y, die.max.x),
    };
    let reach = SlotReach {
        lo: lo + size.half_width,
        hi: hi - size.half_width,
        apart: (size.half_width * 2 + db.library.layer(layer).min_spacing).get(),
        max_corner: db.dbu().to_dbu(MAX_CORNER_MICRONS).unwrap_or(Dbu::ZERO),
    };
    let patterns: Vec<&TrackDef> = db
        .tracks
        .iter()
        .filter(|t| t.axis == axis && t.layers.contains(&layer))
        .collect();
    if patterns.is_empty() {
        return Err(IoPlacementError::NoTracks(
            db.library.layer(layer).name.clone(),
        ));
    }
    let mut coords: Vec<Dbu> = patterns.iter().flat_map(|t| reach.slots_of(t)).collect();
    coords.sort_unstable();
    if matches!(edge, Edge::Top | Edge::Left) {
        coords.reverse();
    }
    Ok(coords
        .into_iter()
        .map(|c| {
            if edge.runs_horizontally() {
                Point::new(c, fixed)
            } else {
                Point::new(fixed, c)
            }
        })
        .collect())
}

/// Where along an edge a layer's pins may sit.
#[derive(Clone, Copy, Debug)]
struct SlotReach {
    lo: Dbu,
    hi: Dbu,
    apart: i64,
    max_corner: Dbu,
}

impl SlotReach {
    /// One track pattern's slots: every [`SLOT_TRACKS`]th track, or further
    /// apart if pins that close would break the spacing, counted out from
    /// the pattern's start and clear of the corners, as
    /// `IOPlacer::findLayerSlots`.
    fn slots_of(self, tracks: &TrackDef) -> Vec<Dbu> {
        let step = tracks.step.get().max(1);
        let slot_tracks = SLOT_TRACKS.max((self.apart + step - 1) / step);
        let between = slot_tracks * step;
        let corner = (tracks.step * CORNER_TRACKS).min(self.max_corner).get();
        let clear = ceil_div(corner, between);
        let start = tracks.start.get();
        let first = ceil_div(self.lo.get() - start, between).max(0) + clear;
        let last = (i64::from(tracks.count) - 1).min((self.hi.get() - start) / between) - clear;
        (first..=last).map(|i| Dbu(start + i * between)).collect()
    }
}

const fn ceil_div(value: i64, by: i64) -> i64 {
    -(-value).div_euclid(by)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn slot_row(count: usize) -> Vec<Slot> {
        (0..count)
            .map(|i| Slot {
                edge: Edge::Bottom,
                layer: LayerId::new(0),
                at: Point::new(Dbu(i64::try_from(i).unwrap()), Dbu::ZERO),
                blocked: false,
            })
            .collect()
    }

    fn section_lengths(count: usize) -> Vec<usize> {
        sections(&slot_row(count))
            .iter()
            .map(|s| s.slots.len())
            .collect()
    }

    #[test]
    fn sections_split_an_edge_every_two_hundred_slots() {
        assert_eq!(section_lengths(428), vec![200, 200, 28]);
    }

    #[test]
    fn sections_leave_out_a_last_slot_that_would_be_alone() {
        assert_eq!(section_lengths(201), vec![200]);
    }

    #[test]
    fn a_blocked_slot_takes_up_a_section_but_offers_no_room() {
        let mut slots = slot_row(3);
        slots[1].blocked = true;

        let section = Section::new(&slots);

        assert_eq!(section.slots.len(), 3);
        assert_eq!(section.free_slots, 2);
        assert_eq!(section.middle().x, Dbu(1));
    }

    #[test]
    fn slots_count_from_the_track_start_past_the_die_edge_and_corner() {
        let tracks = TrackDef {
            axis: TrackAxis::X,
            start: Dbu(70),
            count: 100,
            step: Dbu(100),
            layers: Vec::new(),
        };
        let reach = SlotReach {
            lo: Dbu(70),
            hi: Dbu(9930),
            apart: 180,
            max_corner: Dbu(2000),
        };

        let slots = reach.slots_of(&tracks);

        assert_eq!(slots.first(), Some(&Dbu(1670)));
        assert_eq!(slots.last(), Some(&Dbu(8270)));
        assert_eq!(slots[1] - slots[0], Dbu(200));
    }

    #[test]
    fn a_box_on_two_edges_reaches_both_stretches() {
        let die = Rect::new(
            Point::new(Dbu::ZERO, Dbu::ZERO),
            Point::new(Dbu(1000), Dbu(1000)),
        );
        let corner = Rect::new(
            Point::new(Dbu(600), Dbu::ZERO),
            Point::new(Dbu(1000), Dbu(300)),
        );

        let stretches = stretches_on_die_edges(die, corner);

        assert_eq!(
            stretches,
            vec![
                (Edge::Bottom, Dbu(600), Dbu(1000)),
                (Edge::Right, Dbu::ZERO, Dbu(300)),
            ]
        );
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
