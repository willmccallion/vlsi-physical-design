//! Whether the metal around a cell's pins leaves each pin a via to reach it by.
//!
//! A pin below the routing layers is reached by a via whose cut lies inside
//! the pin at a crossing of the pin layer's and the first routing layer's
//! tracks, and whose metal on both layers keeps that layer's spacing from
//! the metal of every other owner there: power stripes, blockages, macro
//! obstructions, other fixed pins, the other pins and obstructions of its
//! own cell, and the pins and obstructions of the cells placed beside it.
//! The via's metal may stick out of the pin. A pin with no such via is
//! blocked, as is a pin whose own metal on either layer comes within
//! spacing of such metal, and placement must not leave a cell where one of
//! its pins is. Spacing follows the layer's tables, as the design rule check
//! reads them.
//!
//! Metal on the routing layer, the pin's own or the via's, must also escape:
//! run along its track, within a row height, to a clear via up to the next
//! routing layer, since on these libraries the first routing layer carries
//! only short stubs and a stub boxed in by stripes, I/O pins and the pins of
//! the cells above and below can never be connected.

use crate::db::design::{NetlistDB, PinOwner};
use crate::db::indices::{CellId, CutLayerId, LayerId, MacroPinId, NetId};
use crate::db::library::{Direction, LayerRef, Macro};
use crate::db::rules::{SpacingTable, WidthRangeSpacing};
use crate::geom::dbu::Dbu;
use crate::geom::orient::Orientation;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use crate::verify::shapes::{Owner, obstruction_shapes};
use rstar::{AABB, RTree, RTreeObject};
use rustc_hash::FxHashMap;
use std::cell::RefCell;
use std::sync::OnceLock;

/// A shape on one layer that a pin's via must keep clear of, and whose
/// metal it is.
#[derive(Clone, Debug)]
struct Fixed {
    rect: Rect<Dbu>,
    holder: Holder,
    /// An obstruction or blockage, whose spacing may follow the layer's
    /// default width rather than its own.
    obstruction: bool,
}

/// Whose metal a shape is: its net, if on one, and for a pin of a cell
/// placement can move, which pin, so that a pin on no net still knows its
/// own metal.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Holder {
    net: Option<NetId>,
    pin: Option<(CellId, MacroPinId)>,
}

impl Holder {
    /// Fixed metal of `net`, or of no net.
    const fn of_net(net: Option<NetId>) -> Self {
        Self { net, pin: None }
    }

    /// Whether metal of `self` may touch metal of `other`: both are on one
    /// net, or both are the same pin.
    fn joins(self, other: Self) -> bool {
        (self.net.is_some() && self.net == other.net)
            || (self.pin.is_some() && self.pin == other.pin)
    }
}

impl RTreeObject for Fixed {
    type Envelope = AABB<[i64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        envelope(&self.rect)
    }
}

fn envelope(r: &Rect<Dbu>) -> AABB<[i64; 2]> {
    AABB::from_corners(
        [r.min.x.get(), r.min.y.get()],
        [r.max.x.get(), r.max.y.get()],
    )
}

/// A via from the pin layer up: its cut and its metal on both layers,
/// centered on the origin.
#[derive(Clone, Debug)]
struct ViaUp {
    cut: Rect<Dbu>,
    bottom: Rect<Dbu>,
    pad: Rect<Dbu>,
}

/// A cell standing beside the one being checked, whose metal its vias must
/// keep clear of.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Neighbour {
    /// The cell.
    pub cell: CellId,
    /// Its lower-left corner.
    pub at: Point<Dbu>,
    /// Its orientation.
    pub orient: Orientation,
}

/// The metal one check of a cell's pins measures against, by layer: the
/// fixed metal around the cell and the metal of the cells beside it, and
/// what the check has found of the landings of vias up among it.
#[derive(Debug, Default)]
struct NeighbourMetal {
    above: Vec<Fixed>,
    below: Vec<Fixed>,
    /// The area whose fixed metal the lists hold; every shape checked
    /// against them, with its layer's spacing around it, lies inside.
    zone: Option<Rect<Dbu>>,
    /// For each pin, routing-layer track and landing on the next layer, the
    /// lower pad of the first via up there whose pads both keep clear, or
    /// `None` when none does.
    landings: RefCell<FxHashMap<LandingKey, Option<Rect<Dbu>>>>,
}

/// A landing of a via up for one pin: whose metal it carries, the
/// routing-layer track the via sits on, and the index of the next layer's
/// track it lands on.
type LandingKey = (Holder, Dbu, usize);

/// The fixed metal of one layer and the spacing it demands.
#[derive(Debug)]
struct LayerMetal {
    fixed: RTree<Fixed>,
    /// The layer's default wire width, which obstructions are measured at.
    width: Dbu,
    min_spacing: Dbu,
    table: Option<SpacingTable>,
    ranges: Vec<WidthRangeSpacing>,
    /// The largest spacing any shape of the layer can need.
    reach: Dbu,
    /// Whether obstructions are spaced as default-width metal.
    default_width_obstructions: bool,
}

impl LayerMetal {
    fn new(db: &NetlistDB, layer: LayerId) -> Self {
        let lib = &db.library;
        let l = lib.layer(layer);
        let fixed: Vec<Fixed> = obstruction_shapes(db)
            .into_iter()
            .filter(|s| s.shape.layer == LayerRef::Routing(layer))
            .map(|s| Fixed {
                rect: s.shape.rect,
                holder: Holder::of_net(match s.owner {
                    Owner::Net(n) => Some(n),
                    Owner::Obstacle => None,
                }),
                obstruction: s.obstruction,
            })
            .collect();
        let reach = l
            .spacing_table
            .as_ref()
            .map_or(Dbu::ZERO, SpacingTable::max_spacing)
            .max(l.min_spacing)
            .max(
                l.rules
                    .width_range_spacing
                    .iter()
                    .map(|r| r.spacing)
                    .max()
                    .unwrap_or(Dbu::ZERO),
            );
        Self {
            fixed: RTree::bulk_load(fixed),
            width: l.width,
            min_spacing: l.min_spacing,
            table: l.spacing_table.clone(),
            ranges: l.rules.width_range_spacing.clone(),
            reach,
            default_width_obstructions: lib.rules.min_spacing_for_obstructions,
        }
    }

    /// Whether `shape`, metal of `holder`, neither touches nor comes within
    /// the required spacing of any shape of `metal`, which holds every
    /// shape of the layer near it, that it may not join.
    fn clear(
        &self,
        shape: &Rect<Dbu>,
        holder: Holder,
        metal: &NeighbourMetal,
        near: &[Fixed],
    ) -> bool {
        debug_assert!(
            metal
                .zone
                .is_none_or(|z| z.encloses(&shape.expand(self.reach))),
            "{shape:?} reaches beyond the fixed metal gathered for the check"
        );
        !near
            .iter()
            .any(|f| !holder.joins(f.holder) && too_close(shape, &f.rect, self.required(shape, f)))
    }

    /// The fixed shapes of the layer meeting `zone`.
    fn near(&self, zone: &Rect<Dbu>) -> impl Iterator<Item = Fixed> + '_ {
        self.fixed
            .locate_in_envelope_intersecting(envelope(zone))
            .cloned()
    }

    /// The spacing `shape` and `other` need between them: the layer's
    /// table at their widths and parallel run, raised by any width-range
    /// rule either width falls in, and never below the minimum.
    fn required(&self, shape: &Rect<Dbu>, other: &Fixed) -> Dbu {
        let narrow = |r: &Rect<Dbu>| r.width().min(r.height());
        let w1 = narrow(shape);
        let w2 = if other.obstruction && self.default_width_obstructions {
            self.width
        } else {
            narrow(&other.rect)
        };
        let run = parallel_run(shape, &other.rect);
        let mut required = self
            .table
            .as_ref()
            .map_or(self.min_spacing, |t| t.spacing(w1, w2, run))
            .max(self.min_spacing);
        for rule in &self.ranges {
            let applies = |w: Dbu| rule.min_width <= w && w <= rule.max_width;
            if (applies(w1) || applies(w2)) && rule.spacing > required {
                required = rule.spacing;
            }
        }
        required
    }
}

/// How far two rectangles run side by side: their overlap along the axis
/// they are not apart in, negative when they are apart in both.
fn parallel_run(a: &Rect<Dbu>, b: &Rect<Dbu>) -> Dbu {
    let overlap = |lo_a: Dbu, hi_a: Dbu, lo_b: Dbu, hi_b: Dbu| hi_a.min(hi_b) - lo_a.max(lo_b);
    overlap(a.min.x, a.max.x, b.min.x, b.max.x).max(overlap(a.min.y, a.max.y, b.min.y, b.max.y))
}

/// Whether `a` and `b` touch or lie closer than `spacing`: along the axis
/// they are apart in when they run side by side, corner to corner otherwise.
fn too_close(a: &Rect<Dbu>, b: &Rect<Dbu>, spacing: Dbu) -> bool {
    if a.touches(b) {
        return true;
    }
    let gap = |lo_a: Dbu, hi_a: Dbu, lo_b: Dbu, hi_b: Dbu| {
        i128::from((lo_a - hi_b).max(lo_b - hi_a).max(Dbu::ZERO).get())
    };
    let dx = gap(a.min.x, a.max.x, b.min.x, b.max.x);
    let dy = gap(a.min.y, a.max.y, b.min.y, b.max.y);
    let s = i128::from(spacing.get());
    if parallel_run(a, b) > Dbu::ZERO {
        dx.max(dy) < s
    } else {
        dx * dx + dy * dy < s * s
    }
}

/// The check of pins on the layer below the lowest routing layer against
/// the metal on that routing layer and on their own.
#[derive(Debug)]
pub struct PinAccess {
    pin_layer: LayerId,
    above: LayerId,
    /// Coordinates across the routing layer of its tracks.
    tracks_above: Vec<Dbu>,
    /// Coordinates across the pin layer of its tracks.
    tracks_pin: Vec<Dbu>,
    above_vertical: bool,
    vias: Vec<ViaUp>,
    metal_above: LayerMetal,
    metal_below: LayerMetal,
    /// The way up from the routing layer, when there is a layer above it.
    up: Option<Escape>,
}

/// The vias from the routing layer to the next, which runs across it, and
/// where that layer's fixed metal blocks their pads, which metal on the
/// routing layer must reach within [`Self::reach`].
#[derive(Debug)]
struct Escape {
    /// Coordinates across the next layer of its tracks, which are positions
    /// along the routing layer's.
    tracks: Vec<Dbu>,
    vias: Vec<ViaUp>,
    metal: LayerMetal,
    /// How far along its track metal on the routing layer may run to a via.
    reach: Dbu,
    /// Whether the next layer's tracks run vertically.
    vertical: bool,
    /// Where the first chunk of via origins along every track starts.
    chunk_origin: Dbu,
    /// Length of a chunk of via origins along a track.
    chunk: Dbu,
    /// Chunks per track.
    chunks: usize,
    /// For each track and each chunk of it, once asked, where fixed metal
    /// blocks each via's upper pad among the chunk's origins.
    pads: Vec<OnceLock<Vec<PadStretches>>>,
}

/// Escape reaches of via origins along a track whose blocked pads are
/// worked out together, the first time a check asks about one of them.
const PAD_CHUNK_REACHES: i64 = 8;

/// What fixed metal keeps a via's upper pad from a landing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PadBlock {
    /// Nothing.
    Clear,
    /// Only metal of this net, which a pad of the same net may meet.
    Net(NetId),
    /// Metal of no net or of several nets.
    All,
}

/// The disjoint stretches, in order, of the via origins along one track at
/// which fixed metal blocks the via's upper pad, each with what blocks it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct PadStretches(Vec<PadStretch>);

/// Via origins `lo..=hi` along a track whose pads `block` stops.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PadStretch {
    lo: Dbu,
    hi: Dbu,
    block: PadBlock,
}

impl PadBlock {
    /// What blocks a pad that both `self` and `other` block.
    const fn and(self, other: Self) -> Self {
        match (self, other) {
            (Self::Clear, b) | (b, Self::Clear) => b,
            (Self::Net(a), Self::Net(b)) if a.index() == b.index() => Self::Net(a),
            _ => Self::All,
        }
    }

    /// What blocks a pad that fixed metal of `holder` comes too close to.
    const fn of(holder: Holder) -> Self {
        match holder.net {
            Some(n) => Self::Net(n),
            None => Self::All,
        }
    }
}

impl PadStretches {
    /// The stretches `pieces` cover, overlapping ones split where what
    /// blocks them changes.
    fn merged(pieces: &[PadStretch]) -> Self {
        let mut bounds: Vec<Dbu> = pieces.iter().flat_map(|p| [p.lo, p.hi + Dbu(1)]).collect();
        bounds.sort_unstable();
        bounds.dedup();
        let mut out: Vec<PadStretch> = Vec::new();
        for w in bounds.windows(2) {
            let (lo, hi) = (w[0], w[1] - Dbu(1));
            let block = pieces
                .iter()
                .filter(|p| p.lo <= lo && hi <= p.hi)
                .fold(PadBlock::Clear, |b, p| b.and(p.block));
            if block == PadBlock::Clear {
                continue;
            }
            match out.last_mut() {
                Some(last) if last.block == block && last.hi + Dbu(1) == lo => last.hi = hi,
                _ => out.push(PadStretch { lo, hi, block }),
            }
        }
        Self(out)
    }

    /// What blocks a pad whose via stands at `along`.
    fn at(&self, along: Dbu) -> PadBlock {
        let i = self.0.partition_point(|s| s.hi < along);
        match self.0.get(i) {
            Some(s) if s.lo <= along => s.block,
            _ => PadBlock::Clear,
        }
    }
}

impl Escape {
    fn new(db: &NetlistDB, next: LayerId, vias: Vec<ViaUp>, reach: Dbu) -> Self {
        let tracks = db.layer_tracks(next);
        let metal = LayerMetal::new(db, next);
        let vertical = db.library.layer(next).direction == Direction::Vertical;
        let (chunk_origin, extent) = if vertical {
            (db.die_area.min.y, db.die_area.height())
        } else {
            (db.die_area.min.x, db.die_area.width())
        };
        let chunk = (reach * PAD_CHUNK_REACHES).max(Dbu(1));
        let chunks = usize::try_from((extent.get() + chunk.get() - 1) / chunk.get())
            .unwrap_or(1)
            .max(1);
        Self {
            pads: (0..tracks.len() * chunks)
                .map(|_| OnceLock::new())
                .collect(),
            tracks,
            vias,
            metal,
            reach,
            vertical,
            chunk_origin,
            chunk,
            chunks,
        }
    }

    /// The chunk of via origins along a track that `along` falls in; the
    /// first and last chunks also take the origins beyond the die.
    fn chunk_of(&self, along: Dbu) -> usize {
        let i = (along - self.chunk_origin)
            .get()
            .div_euclid(self.chunk.get());
        usize::try_from(i).unwrap_or(0).min(self.chunks - 1)
    }

    /// What blocks via `k`'s upper pad with the via on track `t` at
    /// `along`.
    fn pad_block(&self, t: usize, k: usize, along: Dbu) -> PadBlock {
        let c = self.chunk_of(along);
        let pads = self.pads[t * self.chunks + c].get_or_init(|| self.chunk_stretches(t, c));
        pads[k].at(along)
    }

    /// Where along track `t` fixed metal blocks each via's upper pad, for
    /// every shape that can block a pad whose via stands in chunk `c`.
    fn chunk_stretches(&self, t: usize, c: usize) -> Vec<PadStretches> {
        let track = self.tracks[t];
        let start = self.chunk_origin + self.chunk * i64::try_from(c).unwrap_or(i64::MAX);
        let open = Dbu(i64::MAX / 4);
        let lo = if c == 0 { -open } else { start };
        let hi = if c + 1 == self.chunks {
            open
        } else {
            start + self.chunk
        };
        let extent = |f: fn(&Rect<Dbu>) -> (Dbu, Dbu)| {
            self.vias
                .iter()
                .map(|v| f(&v.pad))
                .fold((Dbu::ZERO, Dbu::ZERO), |(a, b), (c, d)| {
                    (a.min(c), b.max(d))
                })
        };
        let (along_pad, across_pad) = if self.vertical {
            (
                extent(|r| (r.min.y, r.max.y)),
                extent(|r| (r.min.x, r.max.x)),
            )
        } else {
            (
                extent(|r| (r.min.x, r.max.x)),
                extent(|r| (r.min.y, r.max.y)),
            )
        };
        let reach = self.metal.reach;
        let (a0, a1) = (lo + along_pad.0 - reach, hi + along_pad.1 + reach);
        let (b0, b1) = (track + across_pad.0 - reach, track + across_pad.1 + reach);
        let zone = if self.vertical {
            Rect::new(Point::new(b0, a0), Point::new(b1, a1))
        } else {
            Rect::new(Point::new(a0, b0), Point::new(a1, b1))
        };
        let near: Vec<&Fixed> = self
            .metal
            .fixed
            .locate_in_envelope_intersecting(envelope(&zone))
            .collect();
        self.vias
            .iter()
            .map(|via| {
                let pieces: Vec<PadStretch> = near
                    .iter()
                    .filter_map(|f| {
                        let (lo, hi) =
                            Self::blocked_origins(&self.metal, f, &via.pad, track, self.vertical)?;
                        Some(PadStretch {
                            lo,
                            hi,
                            block: PadBlock::of(f.holder),
                        })
                    })
                    .collect();
                PadStretches::merged(&pieces)
            })
            .collect()
    }

    /// The via origins along the track at `track` whose `pad` comes too
    /// close to fixed shape `f`, if any. They form one stretch: moving the
    /// pad toward the shape only shrinks the gap and lengthens the run
    /// beside it, which only raises the spacing needed, so the stretch is
    /// found by bisecting each side from the pad centred on the shape.
    fn blocked_origins(
        metal: &LayerMetal,
        f: &Fixed,
        pad: &Rect<Dbu>,
        track: Dbu,
        vertical: bool,
    ) -> Option<(Dbu, Dbu)> {
        let (along_shape, along_pad) = if vertical {
            ((f.rect.min.y, f.rect.max.y), (pad.min.y, pad.max.y))
        } else {
            ((f.rect.min.x, f.rect.max.x), (pad.min.x, pad.max.x))
        };
        let blocks = |origin: Dbu| {
            let at = if vertical {
                Point::new(track, origin)
            } else {
                Point::new(origin, track)
            };
            let placed = pad.translate(at);
            too_close(&placed, &f.rect, metal.required(&placed, f))
        };
        let centre = (along_shape.0 + along_shape.1 - along_pad.0 - along_pad.1) / 2;
        if !blocks(centre) {
            return None;
        }
        let below = along_shape.0 - along_pad.1 - metal.reach - Dbu(1);
        let above = along_shape.1 - along_pad.0 + metal.reach + Dbu(1);
        let first_blocked = |mut clear: Dbu, mut blocked: Dbu| {
            while (blocked - clear).abs() > Dbu(1) {
                let mid = clear + (blocked - clear) / 2;
                if blocks(mid) {
                    blocked = mid;
                } else {
                    clear = mid;
                }
            }
            blocked
        };
        Some((first_blocked(below, centre), first_blocked(above, centre)))
    }

    /// Whether via `k` landing on the next layer's track `t` at `at` keeps
    /// its upper pad clear of fixed metal `holder` may not join.
    fn pad_clear(&self, t: usize, k: usize, at: Point<Dbu>, holder: Holder) -> bool {
        let along = if self.vertical { at.y } else { at.x };
        match self.pad_block(t, k, along) {
            PadBlock::Clear => true,
            PadBlock::Net(by) => holder.net == Some(by),
            PadBlock::All => false,
        }
    }
}

impl PinAccess {
    /// The check for pins on the layer below `above`, the lowest layer
    /// signal wires use, or `None` when no library via joins the two.
    pub fn new(db: &NetlistDB, above: LayerId) -> Option<Self> {
        let pin_layer = LayerId::new(above.index().checked_sub(1)?);
        let cut = CutLayerId::new(pin_layer.index());
        let lib = &db.library;
        let metal_on = |v: &crate::db::library::ViaDef, layer: LayerId| {
            v.shapes
                .iter()
                .filter(|s| s.layer == LayerRef::Routing(layer))
                .map(|s| s.rect)
                .reduce(|a, b| a.union(&b))
        };
        let vias: Vec<ViaUp> = lib
            .vias
            .iter()
            .filter(|v| v.cut == cut && !v.from_def)
            .filter_map(|v| {
                let cut_rect = v
                    .shapes
                    .iter()
                    .find(|s| s.layer == LayerRef::Cut(cut))?
                    .rect;
                Some(ViaUp {
                    cut: cut_rect,
                    bottom: metal_on(v, pin_layer)?,
                    pad: metal_on(v, above)?,
                })
            })
            .collect();
        if vias.is_empty() {
            return None;
        }
        let next = LayerId::new(above.index() + 1);
        let crosses = |l: LayerId| lib.layer(l).direction != lib.layer(above).direction;
        let up = (next.index() < lib.routing_layers.len() && crosses(next))
            .then(|| {
                let cut_up = CutLayerId::new(above.index());
                let vias: Vec<ViaUp> = lib
                    .vias
                    .iter()
                    .filter(|v| v.cut == cut_up && !v.from_def)
                    .filter_map(|v| {
                        let cut_rect = v
                            .shapes
                            .iter()
                            .find(|s| s.layer == LayerRef::Cut(cut_up))?
                            .rect;
                        Some(ViaUp {
                            cut: cut_rect,
                            bottom: metal_on(v, above)?,
                            pad: metal_on(v, next)?,
                        })
                    })
                    .collect();
                Escape::new(db, next, vias, escape_reach(db, above))
            })
            .filter(|up| !up.vias.is_empty());
        Some(Self {
            pin_layer,
            above,
            tracks_above: db.layer_tracks(above),
            tracks_pin: db.layer_tracks(pin_layer),
            above_vertical: lib.layer(above).direction == Direction::Vertical,
            vias,
            metal_above: LayerMetal::new(db, above),
            metal_below: LayerMetal::new(db, pin_layer),
            up,
        })
    }

    /// How many signal pins of `cell`, placed at `at` with `orient`, no via
    /// can reach for the fixed metal alone.
    pub fn blocked_pins(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        orient: Orientation,
    ) -> usize {
        self.blocked_pins_among(db, cell, at, orient, &[])
    }

    /// How many signal pins of `cell`, placed at `at` with `orient`, are
    /// blocked while `neighbours` stand where they are, their pins and
    /// obstructions counting like fixed metal: a pin on a net that no via
    /// can reach, or any signal pin whose own metal conflicts with other
    /// metal.
    pub fn blocked_pins_among(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        orient: Orientation,
        neighbours: &[Neighbour],
    ) -> usize {
        let extra = self.cell_and_neighbour_metal(db, cell, at, orient, neighbours);
        let c = &db.cells[cell.index()];
        let m = db.library.cell_macro(c.macro_id);
        let net_of = pin_nets(db, cell);
        m.pins
            .iter()
            .enumerate()
            .filter(|(_, pin)| !pin.signal_use.is_supply())
            .filter(|&(k, _)| match net_of[k] {
                Some(net) if db.nets[net.index()].signal_use.is_supply() => false,
                net => {
                    let mp = MacroPinId::new(k);
                    let holder = Holder {
                        net,
                        pin: Some((cell, mp)),
                    };
                    self.pin_access(m, mp, holder, at, orient, &extra).is_none()
                }
            })
            .count()
    }

    /// The metal of `neighbours` and of `cell` itself standing at `at` with
    /// `orient`, whose other pins and obstructions a pin's vias must keep
    /// clear of too.
    fn cell_and_neighbour_metal(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        orient: Orientation,
        neighbours: &[Neighbour],
    ) -> NeighbourMetal {
        let mut all = Vec::with_capacity(neighbours.len() + 1);
        all.extend_from_slice(neighbours);
        all.push(Neighbour { cell, at, orient });
        let mut metal = self.neighbour_metal(db, &all);
        let size = db.library.cell_macro(db.cells[cell.index()].macro_id).size;
        let outline =
            orient.place_rect(&Rect::new(Point::new(Dbu::ZERO, Dbu::ZERO), size), size, at);
        let zone = outline.expand(self.check_margin());
        metal.above.extend(self.metal_above.near(&zone));
        metal.below.extend(self.metal_below.near(&zone));
        metal.zone = Some(zone);
        metal
    }

    /// How far beyond a cell's outline the shapes one check of its pins
    /// measures can reach: the escape along a track, the farthest any via's
    /// metal reaches from its cut, and the largest spacing of either layer.
    fn check_margin(&self) -> Dbu {
        let farthest = |pads: &mut dyn Iterator<Item = &Rect<Dbu>>| {
            pads.flat_map(|p| [-p.min.x, p.max.x, -p.min.y, p.max.y])
                .max()
                .unwrap_or(Dbu::ZERO)
        };
        let access = farthest(&mut self.vias.iter().flat_map(|v| [&v.pad, &v.bottom]));
        let escape = self.up.as_ref().map_or(Dbu::ZERO, |up| {
            up.reach + farthest(&mut up.vias.iter().map(|v| &v.bottom))
        });
        access + escape + self.metal_above.reach.max(self.metal_below.reach)
    }

    /// The metal every signal pin of `cell`, placed at `at` with `orient`
    /// among `neighbours`, relies on for its access: its own shapes and the
    /// via pads and runs on the routing layer its verdict used, by layer;
    /// `None` when some pin is blocked. Another cell's metal that keeps
    /// clear of all of it cannot block the cell, see [`Self::disturbs`].
    pub fn reliance(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        orient: Orientation,
        neighbours: &[Neighbour],
    ) -> Option<Vec<(LayerId, Rect<Dbu>)>> {
        let extra = self.cell_and_neighbour_metal(db, cell, at, orient, neighbours);
        let c = &db.cells[cell.index()];
        let m = db.library.cell_macro(c.macro_id);
        let net_of = pin_nets(db, cell);
        let mut relied = Vec::new();
        for (k, pin) in m.pins.iter().enumerate() {
            if pin.signal_use.is_supply() {
                continue;
            }
            if let Some(net) = net_of[k]
                && db.nets[net.index()].signal_use.is_supply()
            {
                continue;
            }
            let mp = MacroPinId::new(k);
            let holder = Holder {
                net: net_of[k],
                pin: Some((cell, mp)),
            };
            relied.extend(self.pin_access(m, mp, holder, at, orient, &extra)?);
        }
        Some(relied)
    }

    /// The metal of `cell` at `at` with `orient` on the two layers checked:
    /// its pin shapes and obstructions, by layer.
    pub fn cell_metal(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        orient: Orientation,
    ) -> Vec<(LayerId, Rect<Dbu>)> {
        let c = &db.cells[cell.index()];
        let m = db.library.cell_macro(c.macro_id);
        m.pins
            .iter()
            .flat_map(|pin| pin.shapes.iter())
            .chain(m.obstructions.iter())
            .filter_map(|s| match s.layer {
                LayerRef::Routing(l) if l == self.above || l == self.pin_layer => {
                    Some((l, orient.place_rect(&s.rect, m.size, at)))
                }
                _ => None,
            })
            .collect()
    }

    /// Whether `metal`, another cell's shapes by layer, comes within the
    /// largest spacing of its layer of any of `relied`, so that the cell
    /// relying on it must be checked again.
    pub fn disturbs(
        &self,
        metal: &[(LayerId, Rect<Dbu>)],
        relied: &[(LayerId, Rect<Dbu>)],
    ) -> bool {
        metal.iter().any(|&(layer, rect)| {
            let reach = if layer == self.above {
                self.metal_above.reach
            } else {
                self.metal_below.reach
            };
            relied
                .iter()
                .any(|&(l, r)| l == layer && too_close(&rect, &r, reach))
        })
    }

    /// How far beyond a cell's edge the metal of another cell can still
    /// block one of its pins: the largest spacing of either layer plus the
    /// farthest a via's metal reaches from a cut inside the cell.
    pub fn reach(&self) -> Dbu {
        let farthest = self
            .vias
            .iter()
            .flat_map(|v| [-v.pad.min.x, v.pad.max.x, -v.pad.min.y, v.pad.max.y])
            .max()
            .unwrap_or(Dbu::ZERO);
        (self.metal_above.reach + farthest).max(self.metal_below.reach)
    }

    /// How far along the routing layer's tracks a pin's metal may run to
    /// its via up, so how far away a cell can still block that escape.
    pub fn escape_reach(&self) -> Dbu {
        self.up.as_ref().map_or(Dbu::ZERO, |up| up.reach)
    }

    /// The metal of `neighbours` on both layers: every pin shape with its
    /// net, when the pin is on one, and every obstruction.
    fn neighbour_metal(&self, db: &NetlistDB, neighbours: &[Neighbour]) -> NeighbourMetal {
        let mut metal = NeighbourMetal::default();
        for n in neighbours {
            let c = &db.cells[n.cell.index()];
            let m = db.library.cell_macro(c.macro_id);
            let place = |rect: &Rect<Dbu>| n.orient.place_rect(rect, m.size, n.at);
            let net_of = |mp: MacroPinId| {
                c.pins.iter().find_map(|&p| {
                    let pin = &db.pins[p.index()];
                    matches!(pin.owner, PinOwner::Cell { pin: own, .. } if own == mp)
                        .then_some(pin.net)
                })
            };
            let mut add = |layer: LayerRef, rect: Rect<Dbu>, holder: Holder, obstruction: bool| {
                let list = if layer == LayerRef::Routing(self.above) {
                    &mut metal.above
                } else if layer == LayerRef::Routing(self.pin_layer) {
                    &mut metal.below
                } else {
                    return;
                };
                list.push(Fixed {
                    rect,
                    holder,
                    obstruction,
                });
            };
            for (k, pin) in m.pins.iter().enumerate() {
                let mp = MacroPinId::new(k);
                let holder = Holder {
                    net: net_of(mp),
                    pin: Some((n.cell, mp)),
                };
                for s in &pin.shapes {
                    add(s.layer, place(&s.rect), holder, false);
                }
            }
            for s in &m.obstructions {
                add(s.layer, place(&s.rect), Holder::of_net(None), true);
            }
        }
        metal
    }

    /// The metal the pin's access relies on, by layer, or `None` when the
    /// pin is blocked: when its metal on either layer conflicts with other
    /// metal, or, for a pin on a net, none of its shapes on the pin layer
    /// can be reached while it has no clear metal on the routing layer that
    /// escapes. A pin on no net needs no via. A pin with shapes on neither
    /// layer is never blocked here. `holder` is the pin's own.
    fn pin_access(
        &self,
        m: &Macro,
        mp: MacroPinId,
        holder: Holder,
        at: Point<Dbu>,
        orient: Orientation,
        extra: &NeighbourMetal,
    ) -> Option<Vec<(LayerId, Rect<Dbu>)>> {
        let mut relied = Vec::new();
        let mut clear_above = false;
        let mut on_pin_layer = false;
        let mut way_below: Option<Vec<Rect<Dbu>>> = None;
        for s in &m.pins[mp.index()].shapes {
            let rect = orient.place_rect(&s.rect, m.size, at);
            if s.layer == LayerRef::Routing(self.above) {
                if !self.metal_above.clear(&rect, holder, extra, &extra.above) {
                    log::trace!(
                        "pin {} at {rect:?} meets other metal on its routing layer",
                        m.pins[mp.index()].name
                    );
                    return None;
                }
                relied.push((self.above, rect));
                if holder.net.is_none() {
                    clear_above = true;
                } else if let Some(run) = self.escapes(&rect, holder, extra) {
                    relied.push((self.above, run));
                    clear_above = true;
                }
            } else if s.layer == LayerRef::Routing(self.pin_layer) {
                on_pin_layer = true;
                if !self.metal_below.clear(&rect, holder, extra, &extra.below) {
                    log::trace!(
                        "pin {} at {rect:?} meets other metal on the pin layer",
                        m.pins[mp.index()].name
                    );
                    return None;
                }
                relied.push((self.pin_layer, rect));
                if way_below.is_none() {
                    way_below = self.reachable(&rect, holder, extra);
                }
            }
        }
        let needs_way_out = holder.net.is_some() && (on_pin_layer || !clear_above);
        if needs_way_out && !clear_above {
            let Some(way) = way_below else {
                log::trace!(
                    "pin {} of a {} at {at:?} has no via up that escapes",
                    m.pins[mp.index()].name,
                    m.name
                );
                return None;
            };
            relied.extend(way.into_iter().map(|r| (self.above, r)));
        }
        Some(relied)
    }

    /// The metal on the routing layer by which metal `start` of `holder` reaches
    /// a via up, within the escape reach along its track, whose metal on the
    /// next layer is clear of fixed metal and whose wire and lower pad are
    /// clear of other metal on the routing layer, as the box around them;
    /// `start` itself when there is no layer above, `None` when no landing
    /// works.
    ///
    /// Landings are tried nearest first. The wire to a farther landing in one
    /// direction holds the wire to every nearer one, so once a wire runs into
    /// other metal no landing beyond it in that direction is tried.
    fn escapes(
        &self,
        start: &Rect<Dbu>,
        holder: Holder,
        extra: &NeighbourMetal,
    ) -> Option<Rect<Dbu>> {
        let Some(up) = &self.up else {
            return Some(*start);
        };
        let (lo, hi) = if self.above_vertical {
            (start.min.y, start.max.y)
        } else {
            (start.min.x, start.max.x)
        };
        let (across_lo, across_hi) = if self.above_vertical {
            (start.min.x, start.max.x)
        } else {
            (start.min.y, start.max.y)
        };
        let t = &up.tracks;
        let first = t.partition_point(|&c| c < lo - up.reach);
        let inside_lo = t.partition_point(|&c| c < lo);
        let inside_hi = t.partition_point(|&c| c <= hi);
        let last = t.partition_point(|&c| c <= hi + up.reach);
        for track in within(&self.tracks_above, across_lo, across_hi) {
            let landing = |i: usize| self.land(start, track, i, holder, extra);
            for i in inside_lo..inside_hi {
                if let Ok(Some(run)) = landing(i) {
                    return Some(run);
                }
            }
            let mut above = inside_hi..last;
            let mut below = (first..inside_lo).rev();
            let (mut next_above, mut next_below) = (above.next(), below.next());
            loop {
                let pick_above = match (next_above, next_below) {
                    (None, None) => break,
                    (Some(_), None) => true,
                    (None, Some(_)) => false,
                    (Some(a), Some(b)) => t[a] - hi <= lo - t[b],
                };
                let (i, cursor_next): (usize, &mut Option<usize>) = if pick_above {
                    (next_above.unwrap_or_default(), &mut next_above)
                } else {
                    (next_below.unwrap_or_default(), &mut next_below)
                };
                match landing(i) {
                    Ok(Some(run)) => return Some(run),
                    Ok(None) => {
                        *cursor_next = if pick_above {
                            above.next()
                        } else {
                            below.next()
                        };
                    }
                    Err(WireBlocked) => *cursor_next = None,
                }
            }
        }
        None
    }

    /// Whether a via up landing on the next layer's track `i`, where it
    /// crosses the routing layer's track at `track`, carries `start` up: the
    /// box around the wire from `start` to it and its lower pad when one of
    /// the vias works, `Ok(None)` when none does, and [`WireBlocked`] when
    /// some via's pads would fit but the wire itself runs into other metal.
    /// The wire is only checked when the pads fit, so a landing blocked at
    /// the pads never stops the search; what the pads meet is remembered in
    /// `extra` for the rest of the check.
    fn land(
        &self,
        start: &Rect<Dbu>,
        track: Dbu,
        i: usize,
        holder: Holder,
        extra: &NeighbourMetal,
    ) -> Result<Option<Rect<Dbu>>, WireBlocked> {
        let Some(up) = &self.up else {
            return Ok(Some(*start));
        };
        let landing = up.tracks[i];
        let half = self.metal_above.width / 2;
        let (at, wire) = if self.above_vertical {
            let (lo, hi) = (start.min.y.min(landing), start.max.y.max(landing));
            (
                Point::new(track, landing),
                Rect::new(Point::new(track - half, lo), Point::new(track + half, hi)),
            )
        } else {
            let (lo, hi) = (start.min.x.min(landing), start.max.x.max(landing));
            (
                Point::new(landing, track),
                Rect::new(Point::new(lo, track - half), Point::new(hi, track + half)),
            )
        };
        let key = (holder, track, i);
        let known = extra.landings.borrow().get(&key).copied();
        let bottom = known.unwrap_or_else(|| {
            let bottom = (0..up.vias.len()).find_map(|k| {
                let bottom = up.vias[k].bottom.translate(at);
                (up.pad_clear(i, k, at, holder)
                    && self.metal_above.clear(&bottom, holder, extra, &extra.above))
                .then_some(bottom)
            });
            let _ = extra.landings.borrow_mut().insert(key, bottom);
            bottom
        });
        let Some(bottom) = bottom else {
            return Ok(None);
        };
        if !self.metal_above.clear(&wire, holder, extra, &extra.above) {
            return Err(WireBlocked);
        }
        Ok(Some(wire.union(&bottom)))
    }

    /// The routing-layer metal of some via with its cut inside `rect`, at a
    /// crossing of the two layers' tracks, whose lower metal keeps clear of
    /// metal the pin may not join on the pin layer, whose upper metal does
    /// the same on the routing layer and escapes: the pad and the run it
    /// escapes by. The lower metal may stick out of the pin.
    fn reachable(
        &self,
        rect: &Rect<Dbu>,
        holder: Holder,
        extra: &NeighbourMetal,
    ) -> Option<Vec<Rect<Dbu>>> {
        let (xs, ys) = if self.above_vertical {
            (&self.tracks_above, &self.tracks_pin)
        } else {
            (&self.tracks_pin, &self.tracks_above)
        };
        self.vias.iter().find_map(|via| {
            within(xs, rect.min.x - via.cut.min.x, rect.max.x - via.cut.max.x).find_map(|x| {
                within(ys, rect.min.y - via.cut.min.y, rect.max.y - via.cut.max.y).find_map(|y| {
                    let at = Point::new(x, y);
                    let pad = via.pad.translate(at);
                    let bottom = via.bottom.translate(at);
                    if !self.metal_below.clear(&bottom, holder, extra, &extra.below)
                        || !self.metal_above.clear(&pad, holder, extra, &extra.above)
                    {
                        return None;
                    }
                    let run = self.escapes(&pad, holder, extra)?;
                    Some(vec![pad, run])
                })
            })
        })
    }
}

/// The net of each pin of `cell`'s master, by pin index, where it is on one.
fn pin_nets(db: &NetlistDB, cell: CellId) -> Vec<Option<NetId>> {
    let c = &db.cells[cell.index()];
    let m = db.library.cell_macro(c.macro_id);
    let mut net_of = vec![None; m.pins.len()];
    for &p in &c.pins {
        let pin = &db.pins[p.index()];
        if let PinOwner::Cell { pin: mp, .. } = pin.owner {
            net_of[mp.index()] = Some(pin.net);
        }
    }
    net_of
}

/// A wire on the routing layer that runs into other metal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct WireBlocked;

/// How far along the routing layer a pin's metal may run to reach its via
/// up: one row height, or ten of the layer's pitches for a design without
/// rows.
fn escape_reach(db: &NetlistDB, above: LayerId) -> Dbu {
    let lib = &db.library;
    db.rows
        .first()
        .map(|row| lib.site(row.site).size.y)
        .filter(|&h| h > Dbu::ZERO)
        .unwrap_or_else(|| {
            let l = lib.layer(above);
            let pitch = if l.direction == Direction::Vertical {
                l.pitch_x
            } else {
                l.pitch_y
            };
            pitch * 10
        })
}

/// The track coordinates in `[lo, hi]`.
fn within(tracks: &[Dbu], lo: Dbu, hi: Dbu) -> impl Iterator<Item = Dbu> + Clone + '_ {
    let a = tracks.partition_point(|&t| t < lo);
    let b = tracks.partition_point(|&t| t <= hi).max(a);
    tracks[a..b].iter().copied()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::db::design::{PlacementStatus, SpecialNet, SpecialWire, TrackAxis, TrackDef};
    use crate::db::indices::{MacroId, MacroPinId};
    use crate::db::library::{
        CoreKind, CutLayer, Library, MacroClass, MacroPin, PinDirection, RoutingLayer, Shape,
        SignalUse, Site, SiteClass, Symmetry, ViaDef,
    };
    use crate::db::rules::{CutLayerRules, ParallelRunLengthTable, RoutingLayerRules};
    use crate::geom::dbu::DbuPerMicron;

    const fn pt(x: i64, y: i64) -> Point<Dbu> {
        Point::new(Dbu(x), Dbu(y))
    }

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(pt(x0, y0), pt(x1, y1))
    }

    fn layer(name: &str, direction: Direction) -> RoutingLayer {
        RoutingLayer {
            name: name.into(),
            direction,
            pitch_x: Dbu(200),
            pitch_y: Dbu(200),
            offset_x: None,
            offset_y: None,
            width: Dbu(100),
            min_spacing: Dbu(100),
            spacing_table: None,
            rules: RoutingLayerRules::default(),
        }
    }

    /// Horizontal M1 under vertical M2 with 0.1 cut vias whose M2 pad is
    /// 0.1 by 0.18 and whose M1 pad is 0.18 by 0.1 or 0.1 by 0.18; one 0.4
    /// by 2.0 cell master with a 0.1-wide M1 pin centred 0.1 in from its
    /// left edge; a 0.58-wide M2 ground stripe at x = 3.9 over a 10 by 4
    /// die.
    fn design() -> (NetlistDB, crate::db::indices::MacroId) {
        design_with(3900, None, None)
    }

    /// [`design`] with the stripe at `stripe_x`, M2 under `table`, and an
    /// M1 ground rail 0.1 wide along y = `rail_y` when given.
    fn design_with(
        stripe_x: i64,
        table: Option<SpacingTable>,
        rail_y: Option<i64>,
    ) -> (NetlistDB, crate::db::indices::MacroId) {
        design_full(stripe_x, table, rail_y, None)
    }

    /// [`design_with`] plus a horizontal M3 over M2, joined by a 0.1 cut via
    /// with 0.1 by 0.18 pads, and a 6.0-wide M3 ground stripe along
    /// y = `m3_stripe_y` when given.
    fn design_full(
        stripe_x: i64,
        table: Option<SpacingTable>,
        rail_y: Option<i64>,
        m3_stripe_y: Option<i64>,
    ) -> (NetlistDB, crate::db::indices::MacroId) {
        let mut lib = Library::new(DbuPerMicron::new(1000).unwrap());
        let m1 = lib
            .add_routing_layer(layer("M1", Direction::Horizontal))
            .unwrap();
        let v1 = lib
            .add_cut_layer(CutLayer {
                name: "V1".into(),
                width: Some(Dbu(100)),
                spacing: Some(Dbu(120)),
                rules: CutLayerRules::default(),
            })
            .unwrap();
        let m2 = lib
            .add_routing_layer(RoutingLayer {
                spacing_table: table,
                ..layer("M2", Direction::Vertical)
            })
            .unwrap();
        let v2 = lib
            .add_cut_layer(CutLayer {
                name: "V2".into(),
                width: Some(Dbu(100)),
                spacing: Some(Dbu(120)),
                rules: CutLayerRules::default(),
            })
            .unwrap();
        let m3 = lib
            .add_routing_layer(layer("M3", Direction::Horizontal))
            .unwrap();
        let _ = lib
            .add_via(ViaDef {
                name: "V23".into(),
                is_default: true,
                cut: v2,
                shapes: vec![
                    Shape {
                        layer: LayerRef::Routing(m2),
                        rect: rect(-50, -90, 50, 90),
                    },
                    Shape {
                        layer: LayerRef::Cut(v2),
                        rect: rect(-50, -50, 50, 50),
                    },
                    Shape {
                        layer: LayerRef::Routing(m3),
                        rect: rect(-90, -50, 90, 50),
                    },
                ],
                from_def: false,
            })
            .unwrap();
        for (name, bottom) in [
            ("V12", rect(-90, -50, 90, 50)),
            ("V12V", rect(-50, -90, 50, 90)),
        ] {
            let _ = lib
                .add_via(ViaDef {
                    name: name.into(),
                    is_default: name == "V12",
                    cut: v1,
                    shapes: vec![
                        Shape {
                            layer: LayerRef::Routing(m1),
                            rect: bottom,
                        },
                        Shape {
                            layer: LayerRef::Cut(v1),
                            rect: rect(-50, -50, 50, 50),
                        },
                        Shape {
                            layer: LayerRef::Routing(m2),
                            rect: rect(-50, -90, 50, 90),
                        },
                    ],
                    from_def: false,
                })
                .unwrap();
        }
        let symmetry = Symmetry {
            x: true,
            y: true,
            r90: false,
        };
        let site = lib
            .add_site(Site {
                name: "s".into(),
                class: SiteClass::Core,
                size: pt(200, 2000),
                symmetry,
            })
            .unwrap();
        let mut m = Macro::new(
            "C".into(),
            MacroClass::Core(CoreKind::Regular),
            pt(400, 2000),
            symmetry,
            Some(site),
        );
        let _ = m.add_pin(MacroPin {
            name: "a".into(),
            direction: PinDirection::Input,
            signal_use: SignalUse::Signal,
            shapes: vec![Shape {
                layer: LayerRef::Routing(m1),
                rect: rect(50, 500, 150, 1500),
            }],
        });
        let m = lib.add_macro(m).unwrap();
        let mut upper = Macro::new(
            "U".into(),
            MacroClass::Core(CoreKind::Regular),
            pt(400, 2000),
            symmetry,
            Some(site),
        );
        let _ = upper.add_pin(MacroPin {
            name: "a".into(),
            direction: PinDirection::Input,
            signal_use: SignalUse::Signal,
            shapes: vec![Shape {
                layer: LayerRef::Routing(m2),
                rect: rect(50, 500, 150, 1500),
            }],
        });
        let _ = lib.add_macro(upper).unwrap();
        let mut right = Macro::new(
            "R".into(),
            MacroClass::Core(CoreKind::Regular),
            pt(400, 2000),
            symmetry,
            Some(site),
        );
        let _ = right.add_pin(MacroPin {
            name: "a".into(),
            direction: PinDirection::Input,
            signal_use: SignalUse::Signal,
            shapes: vec![Shape {
                layer: LayerRef::Routing(m1),
                rect: rect(250, 500, 350, 1500),
            }],
        });
        let _ = lib.add_macro(right).unwrap();
        let mut edge = Macro::new(
            "E".into(),
            MacroClass::Core(CoreKind::Regular),
            pt(400, 2000),
            symmetry,
            Some(site),
        );
        let _ = edge.add_pin(MacroPin {
            name: "o".into(),
            direction: PinDirection::Output,
            signal_use: SignalUse::Signal,
            shapes: vec![Shape {
                layer: LayerRef::Routing(m2),
                rect: rect(0, 500, 150, 1500),
            }],
        });
        let _ = lib.add_macro(edge).unwrap();
        let mut db = NetlistDB::new("t".into(), lib);
        db.die_area = rect(0, 0, 10_000, 4000);
        db.tracks.push(TrackDef {
            axis: TrackAxis::X,
            start: Dbu(100),
            count: 50,
            step: Dbu(200),
            layers: vec![m2],
        });
        db.tracks.push(TrackDef {
            axis: TrackAxis::Y,
            start: Dbu(0),
            count: 21,
            step: Dbu(200),
            layers: vec![m1, m3],
        });
        let mut wires = vec![SpecialWire {
            layer: m2,
            width: Dbu(580),
            from: pt(stripe_x, 0),
            to: pt(stripe_x, 4000),
            extensions: (Dbu::ZERO, Dbu::ZERO),
            shape: Some("STRIPE".into()),
        }];
        if let Some(y) = rail_y {
            wires.push(SpecialWire {
                layer: m1,
                width: Dbu(100),
                from: pt(0, y),
                to: pt(10_000, y),
                extensions: (Dbu::ZERO, Dbu::ZERO),
                shape: Some("FOLLOWPIN".into()),
            });
        }
        if let Some(y) = m3_stripe_y {
            wires.push(SpecialWire {
                layer: m3,
                width: Dbu(6000),
                from: pt(0, y),
                to: pt(10_000, y),
                extensions: (Dbu::ZERO, Dbu::ZERO),
                shape: Some("STRIPE".into()),
            });
        }
        db.special_nets.push(SpecialNet {
            name: "vss".into(),
            signal_use: SignalUse::Ground,
            connections: Vec::new(),
            wires,
            rects: Vec::new(),
            vias: Vec::new(),
        });
        (db, m)
    }

    /// Spacing 0.10 up to width 0.46 and 0.15 from there, at any run.
    fn wide_metal_table() -> SpacingTable {
        SpacingTable::ParallelRunLength(ParallelRunLengthTable {
            run_lengths: vec![Dbu(0)],
            rows: vec![(Dbu(0), vec![Dbu(100)]), (Dbu(460), vec![Dbu(150)])],
        })
    }

    fn blocked_in(db: &mut NetlistDB, m: crate::db::indices::MacroId, x: i64) -> usize {
        let cell = cell_at(db, m, x);
        let access = PinAccess::new(db, LayerId(1)).unwrap();
        access.blocked_pins(db, cell, pt(x, 0), Orientation::N)
    }

    #[test]
    fn a_wide_stripe_demands_the_tables_spacing_of_the_via_pad() {
        let (mut plain, m) = design_with(3950, None, None);
        let (mut tabled, _) = design_with(3950, Some(wide_metal_table()), None);

        assert_eq!(blocked_in(&mut plain, m, 3400), 0, "0.11 um clears 0.10");
        assert_eq!(blocked_in(&mut tabled, m, 3400), 1, "0.11 um breaks 0.15");
    }

    #[test]
    fn a_pin_within_spacing_of_fixed_metal_on_its_own_layer_is_blocked() {
        let (mut db, m) = design_with(3900, None, Some(1600));

        assert_eq!(blocked_in(&mut db, m, 1000), 1);
    }

    #[test]
    fn a_pin_clear_of_fixed_metal_on_its_own_layer_is_free() {
        let (mut db, m) = design_with(3900, None, Some(1700));

        assert_eq!(blocked_in(&mut db, m, 1000), 0);
    }

    #[test]
    fn a_via_whose_lower_metal_sticks_out_of_the_pin_still_reaches_it() {
        let (mut db, m) = design();
        db.library
            .vias
            .retain(|v| v.name == "V12" || v.name == "V23");

        assert_eq!(blocked_in(&mut db, m, 1000), 0);
    }

    /// Master P, whose pin `a` is master C's and whose pin `b`, 0.1 wide,
    /// stands 0.1 to its right, at (1.0, 0) with only the via `keep` up from
    /// M1; how many of its pins are blocked.
    fn blocked_pair(keep: &str) -> usize {
        let (mut db, _) = design();
        let symmetry = Symmetry {
            x: true,
            y: true,
            r90: false,
        };
        let mut pair = Macro::new(
            "P".into(),
            MacroClass::Core(CoreKind::Regular),
            pt(400, 2000),
            symmetry,
            Some(crate::db::indices::SiteId(0)),
        );
        for (name, x0) in [("a", 50), ("b", 250)] {
            let _ = pair.add_pin(MacroPin {
                name: name.into(),
                direction: PinDirection::Input,
                signal_use: SignalUse::Signal,
                shapes: vec![Shape {
                    layer: LayerRef::Routing(LayerId(0)),
                    rect: rect(x0, 500, x0 + 100, 1500),
                }],
            });
        }
        let pair = db.library.add_macro(pair).unwrap();
        db.library
            .vias
            .retain(|v| v.name == keep || v.name == "V23");
        let cell = db
            .add_cell(
                "p".into(),
                pair,
                pt(1000, 0),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        for (k, name) in [(0, "na"), (1, "nb")] {
            let net = db.net_or_insert(name);
            let _ = db.connect_cell_pin(cell, MacroPinId(k), net);
        }
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        access.blocked_pins(&db, cell, pt(1000, 0), Orientation::N)
    }

    #[test]
    fn a_via_pad_too_near_another_pin_of_its_cell_does_not_reach_the_pin() {
        assert_eq!(blocked_pair("V12"), 2);
    }

    #[test]
    fn a_via_pad_that_keeps_clear_of_the_cells_other_pins_reaches_the_pin() {
        assert_eq!(blocked_pair("V12V"), 0);
    }

    fn cell_at(db: &mut NetlistDB, m: crate::db::indices::MacroId, x: i64) -> CellId {
        let cell = db
            .add_cell(
                format!("c{x}"),
                m,
                pt(x, 0),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        let net = db.net_or_insert(&format!("n{x}"));
        let _ = db.connect_cell_pin(cell, MacroPinId(0), net);
        cell
    }

    fn blocked(x: i64) -> usize {
        let (mut db, m) = design();
        let cell = cell_at(&mut db, m, x);
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        access.blocked_pins(&db, cell, pt(x, 0), Orientation::N)
    }

    #[test]
    fn a_pin_under_the_stripe_is_blocked() {
        assert_eq!(blocked(3800), 1);
    }

    #[test]
    fn a_pin_whose_via_pad_would_come_within_spacing_of_the_stripe_is_blocked() {
        assert_eq!(blocked(3400), 1);
    }

    #[test]
    fn a_pin_whose_via_pad_keeps_the_spacing_is_free() {
        assert_eq!(blocked(3200), 0);
    }

    #[test]
    fn a_pin_far_from_the_stripe_is_free() {
        assert_eq!(blocked(1000), 0);
    }

    fn blocked_upper(x: i64) -> usize {
        let (mut db, _) = design();
        let cell = cell_at(&mut db, crate::db::indices::MacroId(1), x);
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        access.blocked_pins(&db, cell, pt(x, 0), Orientation::N)
    }

    #[test]
    fn a_routing_layer_pin_within_spacing_of_the_stripe_is_blocked() {
        assert_eq!(blocked_upper(3400), 1);
    }

    #[test]
    fn a_routing_layer_pin_clear_of_the_stripe_is_free() {
        assert_eq!(blocked_upper(3200), 0);
    }

    /// Master R at 1.0 with master E placed at `x` to its right, on the same
    /// net when `same_net`; how many of R's pins are blocked.
    fn blocked_beside(x: i64, same_net: bool) -> usize {
        let (mut db, _) = design();
        let right = cell_at(&mut db, MacroId(2), 1000);
        let edge = db
            .add_cell(
                "e".into(),
                MacroId(3),
                pt(x, 0),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        let net = db.net_or_insert(if same_net { "n1000" } else { "ne" });
        let _ = db.connect_cell_pin(edge, MacroPinId(0), net);
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        let beside = [Neighbour {
            cell: edge,
            at: pt(x, 0),
            orient: Orientation::N,
        }];
        access.blocked_pins_among(&db, right, pt(1000, 0), Orientation::N, &beside)
    }

    #[test]
    fn a_pin_beside_an_abutting_neighbours_routing_layer_pin_is_blocked() {
        assert_eq!(blocked_beside(1400, false), 1);
    }

    #[test]
    fn a_neighbours_routing_layer_pin_one_site_away_does_not_block() {
        assert_eq!(blocked_beside(1600, false), 0);
    }

    #[test]
    fn a_neighbours_pin_on_the_same_net_does_not_block() {
        assert_eq!(blocked_beside(1400, true), 0);
    }

    #[test]
    fn the_fixed_metal_alone_leaves_the_pin_free() {
        let (mut db, _) = design();
        let right = cell_at(&mut db, MacroId(2), 1000);
        let access = PinAccess::new(&db, LayerId(1)).unwrap();

        assert_eq!(
            access.blocked_pins(&db, right, pt(1000, 0), Orientation::N),
            0
        );
        assert_eq!(access.reach(), Dbu(190));
    }

    /// Master U placed at `x` with its M2 pin on no net.
    fn blocked_unconnected(x: i64) -> usize {
        let (mut db, _) = design();
        let cell = db
            .add_cell(
                "u".into(),
                MacroId(1),
                pt(x, 0),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        access.blocked_pins(&db, cell, pt(x, 0), Orientation::N)
    }

    #[test]
    fn an_unconnected_pin_under_the_stripe_is_blocked() {
        assert_eq!(blocked_unconnected(3800), 1);
    }

    #[test]
    fn an_unconnected_pin_clear_of_the_stripe_is_free() {
        assert_eq!(blocked_unconnected(1000), 0);
    }

    /// Master `m` at (1.0, 0) with an M3 stripe band centered on `stripe_y`.
    fn blocked_under_m3_stripe(m: MacroId, stripe_y: i64) -> usize {
        let (mut db, _) = design_full(3900, None, None, Some(stripe_y));
        let cell = cell_at(&mut db, m, 1000);
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        access.blocked_pins(&db, cell, pt(1000, 0), Orientation::N)
    }

    #[test]
    fn a_routing_layer_pin_with_no_via_up_within_a_row_is_blocked() {
        // The band over y -1.0..5.0 leaves no M3 landing within 2.0 of the
        // pin's 0.5..1.5 run.
        assert_eq!(blocked_under_m3_stripe(MacroId(1), 2000), 1);
    }

    #[test]
    fn a_routing_layer_pin_escapes_past_a_band_it_can_run_out_of() {
        // The band over y 1.0..7.0 leaves landings at y 0.0..0.8.
        assert_eq!(blocked_under_m3_stripe(MacroId(1), 4000), 0);
    }

    #[test]
    fn a_pin_layer_pin_needs_its_via_pad_to_escape_too() {
        assert_eq!(blocked_under_m3_stripe(MacroId(0), 2000), 1);
        assert_eq!(blocked_under_m3_stripe(MacroId(0), 4000), 0);
    }

    #[test]
    fn a_design_without_a_via_up_from_the_pins_has_no_check() {
        let (mut db, _) = design();
        db.library.vias.clear();

        assert!(PinAccess::new(&db, LayerId(1)).is_none());
    }

    fn stretch(lo: i64, hi: i64, block: PadBlock) -> PadStretch {
        PadStretch {
            lo: Dbu(lo),
            hi: Dbu(hi),
            block,
        }
    }

    #[test]
    fn overlapping_pad_stretches_of_two_nets_block_every_pad() {
        let (a, b) = (PadBlock::Net(NetId::new(1)), PadBlock::Net(NetId::new(2)));

        let merged =
            PadStretches::merged(&[stretch(0, 10, a), stretch(5, 15, a), stretch(12, 20, b)]);

        assert_eq!(
            merged.0,
            [
                stretch(0, 11, a),
                stretch(12, 15, PadBlock::All),
                stretch(16, 20, b)
            ]
        );
        assert_eq!(merged.at(Dbu(21)), PadBlock::Clear);
    }

    /// What fixed metal on `metal` blocks `pad`, shape by shape.
    fn pad_block_directly(metal: &LayerMetal, pad: &Rect<Dbu>) -> PadBlock {
        metal
            .fixed
            .iter()
            .filter(|f| too_close(pad, &f.rect, metal.required(pad, f)))
            .fold(PadBlock::Clear, |b, f| b.and(PadBlock::of(f.holder)))
    }

    #[test]
    fn pad_stretches_agree_with_checking_every_via_position() {
        let (db, _) = design_full(3900, Some(wide_metal_table()), None, Some(2000));
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        let up = access.up.as_ref().unwrap();
        let metal = LayerMetal::new(&db, LayerId(2));
        let mut seen = std::collections::HashSet::new();

        for (t, &track) in up.tracks.iter().enumerate() {
            for (k, via) in up.vias.iter().enumerate() {
                for x in (-1000..11000).step_by(7) {
                    let at = pt(x, track.get());
                    let direct = pad_block_directly(&metal, &via.pad.translate(at));
                    assert_eq!(
                        up.pad_block(t, k, Dbu(x)),
                        direct,
                        "track {t} via {k} x {x}"
                    );
                    let _ = seen.insert(direct == PadBlock::Clear);
                }
            }
        }
        assert_eq!(seen.len(), 2, "both blocked and clear pads are compared");
    }
}
