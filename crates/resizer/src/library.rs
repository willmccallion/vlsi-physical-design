//! The timing library joined to the design's masters: which Liberty cell
//! each master is, which cells may be used, which buffers are inserted and
//! which sizes a cell can swap to.

use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{CellId, IdVec, MacroId, MacroPinId, PinId, SiteId};
use pare_common::db::liberty::{BufferCell, LibCell, LibCellId, LibPortId, Liberty};
use pare_common::db::library::{CoreKind, MacroClass, PinDirection, SignalUse};
use pare_common::geom::dbu::DbuArea;
use pare_common::units::{Farads, Ohms, Watts};
use std::collections::HashMap;

/// Bands of drive resistance `findBuffers` splits the buffers into.
const DRIVE_BANDS: usize = 5;

/// Buffers `findBuffers` keeps from each band when they stand on one site.
const BUFFERS_PER_BAND: usize = 2;

/// The most used sites `findBuffers` keeps one buffer of each band for.
const BUFFER_SITES: usize = 2;

/// The share of the buffers above which `findBuffers` keeps only those of
/// one footprint.
const DOMINANT_FOOTPRINT_SHARE: f64 = 0.5;

/// Why the library could not be joined to the design.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum LinkError {
    /// A cell named as not to be used is in no library.
    #[error("dont_use names unknown cell '{0}'")]
    UnknownDontUse(String),
    /// No usable buffer has both a Liberty cell and a master.
    #[error("the libraries have no usable buffer")]
    NoBuffers,
}

/// The buffers repair may insert, as `OpenROAD`'s `findBuffers` picks them:
/// a few per band of drive resistance, the weakest last.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BufferSet {
    /// The chosen buffers, by increasing drive resistance.
    pub cells: Vec<BufferCell>,
    /// The chosen buffer with the largest drive resistance.
    pub lowest_drive: BufferCell,
}

/// A Liberty library joined to the masters of a design.
#[derive(Debug)]
pub struct Link<'a> {
    /// The timing library.
    pub liberty: &'a Liberty,
    cell_of_macro: IdVec<MacroId, Option<LibCellId>>,
    macro_of_cell: IdVec<LibCellId, Option<MacroId>>,
    port_of_pin: IdVec<MacroId, IdVec<MacroPinId, Option<LibPortId>>>,
    dont_use: IdVec<LibCellId, bool>,
    class_of: IdVec<LibCellId, Option<usize>>,
    classes: Vec<Vec<LibCellId>>,
    master_area: IdVec<LibCellId, Option<DbuArea>>,
    /// Whether each cell's master is of class `CORE`, as `dbMaster::isCore`.
    core_master: IdVec<LibCellId, bool>,
}

/// How much one sizing step may grow a cell, as `set_opt_config`'s
/// `-sizing_area_limit` and `-sizing_leakage_limit`: a size more than the
/// ratio times the present one's area or leakage is not offered.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SizingLimits {
    area: Option<f64>,
    leakage: Option<f64>,
}

/// A cell's measures the sizing limits compare.
#[derive(Clone, Copy, Debug, PartialEq)]
struct CellMeasures {
    /// Its master's area.
    area: Option<DbuArea>,
    /// The power it leaks while idle, where the library gives it.
    leakage: Option<Watts>,
}

impl SizingLimits {
    /// `OpenROAD`'s defaults: at most four times the area and the leakage.
    pub(crate) const OPENROAD: Self = Self {
        area: Some(4.0),
        leakage: Some(4.0),
    };

    /// No limits, as `repairClkNets` lifts them for clock buffers.
    pub(crate) const NONE: Self = Self {
        area: None,
        leakage: None,
    };

    /// Whether a cell measuring `from` may be sized to one measuring `to`,
    /// as `Resizer::getSwappableCells` filters; a missing measure passes.
    fn allows(self, from: CellMeasures, to: CellMeasures) -> bool {
        let area_ok = match (self.area, from.area, to.area) {
            (Some(limit), Some(from), Some(to)) if from.get() != 0 => {
                to.get() as f64 / from.get() as f64 <= limit
            }
            _ => true,
        };
        let leakage_ok = match (self.leakage, from.leakage, to.leakage) {
            (Some(limit), Some(from), Some(to)) => {
                let ratio = to / from;
                ratio.is_nan() || ratio <= limit
            }
            _ => true,
        };
        area_ok && leakage_ok
    }
}

impl<'a> Link<'a> {
    /// Joins `liberty` to the masters of `db`, by name, and marks the cells
    /// in `dont_use` and those the library marks as not to be used.
    pub fn new(
        db: &NetlistDB,
        liberty: &'a Liberty,
        dont_use: &[String],
    ) -> Result<Self, LinkError> {
        let mut dont_use_flags: IdVec<LibCellId, bool> =
            liberty.cells.iter().map(|c| c.dont_use).collect();
        for name in dont_use {
            let id = liberty
                .cell_id(name)
                .ok_or_else(|| LinkError::UnknownDontUse(name.clone()))?;
            dont_use_flags[id] = true;
        }
        let mut macro_of_cell = IdVec::filled(None, liberty.cells.len());
        let mut cell_of_macro = IdVec::with_capacity(db.library.macros.len());
        let mut port_of_pin = IdVec::with_capacity(db.library.macros.len());
        for (m, master) in db.library.macros.iter_enumerated() {
            let cell = liberty.cell_id(&master.name);
            let _ = cell_of_macro.push(cell);
            if let Some(cell) = cell {
                macro_of_cell[cell] = Some(m);
            }
            let lib_cell = cell.map(|c| liberty.cell(c));
            let _ = port_of_pin.push(
                master
                    .pins
                    .iter()
                    .map(|p| lib_cell.and_then(|c| c.port(&p.name)))
                    .collect(),
            );
        }
        let (class_of, classes) = equivalence_classes(liberty, &dont_use_flags);
        let master_area = macro_of_cell
            .iter()
            .map(|m| {
                m.map(|m| {
                    let size = db.library.macros[m].size;
                    size.x * size.y
                })
            })
            .collect();
        let core_master = macro_of_cell
            .iter()
            .map(|m| m.is_some_and(|m| matches!(db.library.macros[m].class, MacroClass::Core(_))))
            .collect();
        Ok(Self {
            liberty,
            cell_of_macro,
            macro_of_cell,
            port_of_pin,
            dont_use: dont_use_flags,
            class_of,
            classes,
            master_area,
            core_master,
        })
    }

    /// The Liberty cell of a master.
    #[must_use]
    pub(crate) fn cell_of_macro(&self, master: MacroId) -> Option<LibCellId> {
        self.cell_of_macro[master]
    }

    /// The master of a Liberty cell.
    #[must_use]
    pub fn macro_of(&self, cell: LibCellId) -> Option<MacroId> {
        self.macro_of_cell[cell]
    }

    /// Whether a Liberty cell has a master: `OpenROAD`'s link cell test.
    #[must_use]
    pub fn is_link_cell(&self, cell: LibCellId) -> bool {
        self.macro_of_cell[cell].is_some()
    }

    /// Whether the cell is not to be used.
    #[must_use]
    pub fn dont_use(&self, cell: LibCellId) -> bool {
        self.dont_use[cell]
    }

    /// The Liberty cell of an instance.
    #[must_use]
    pub fn cell(&self, db: &NetlistDB, instance: CellId) -> Option<(LibCellId, &'a LibCell)> {
        let id = self.cell_of_macro(db.cells[instance].macro_id)?;
        Some((id, self.liberty.cell(id)))
    }

    /// The Liberty cell and port index of an instance pin.
    #[must_use]
    pub fn pin_port(&self, db: &NetlistDB, pin: PinId) -> Option<(LibCellId, LibPortId)> {
        let PinOwner::Cell {
            cell,
            pin: macro_pin,
        } = db.pins[pin].owner
        else {
            return None;
        };
        let master = db.cells[cell].macro_id;
        let lib = self.cell_of_macro(master)?;
        let port = self.port_of_pin[master][macro_pin]?;
        Some((lib, port))
    }

    /// The index in the master's pins of the Liberty port `port` of `cell`.
    #[must_use]
    pub fn macro_pin_of_port(&self, cell: LibCellId, port: LibPortId) -> Option<MacroPinId> {
        let master = self.macro_of(cell)?;
        self.port_of_pin[master]
            .iter_enumerated()
            .find(|&(_, &p)| p == Some(port))
            .map(|(pin, _)| pin)
    }

    /// The cells an instance of `cell` may be swapped for, itself included,
    /// as `OpenROAD`'s `getSwappableCells`: none for a cell whose master is
    /// not a core cell; else the equivalent cells that have a master, may
    /// be used, are within `limits` of `cell` and share its user function
    /// class when it has one, or just the cell when it has no class.
    #[must_use]
    pub(crate) fn swappable_cells(&self, cell: LibCellId, limits: SizingLimits) -> Vec<LibCellId> {
        if !self.core_master[cell] {
            return Vec::new();
        }
        let Some(class) = self.class_of[cell] else {
            return vec![cell];
        };
        let from = self.measures(cell);
        let function_class = self.liberty.cell(cell).user_function_class.as_deref();
        self.classes[class]
            .iter()
            .copied()
            .filter(|&c| !self.dont_use(c) && self.is_link_cell(c))
            .filter(|&c| limits.allows(from, self.measures(c)))
            .filter(|&c| {
                function_class
                    .is_none_or(|f| self.liberty.cell(c).user_function_class.as_deref() == Some(f))
            })
            .collect()
    }

    fn measures(&self, cell: LibCellId) -> CellMeasures {
        CellMeasures {
            area: self.master_area[cell],
            leakage: cell_leakage(self.liberty.cell(cell)),
        }
    }

    /// Drive resistance of a buffer's output.
    #[must_use]
    pub(crate) fn buffer_drive_resistance(&self, buffer: BufferCell) -> Ohms {
        self.liberty.cell(buffer.id).drive_resistance(buffer.output)
    }

    /// Capacitance of a buffer's input.
    #[must_use]
    pub(crate) fn buffer_input_capacitance(&self, buffer: BufferCell) -> Farads {
        self.liberty.cell(buffer.id).ports[buffer.input].capacitance()
    }

    /// The buffers to insert, as `OpenROAD`'s `findBuffers` chooses them:
    /// the usable buffers that are not clock buffers, sorted by drive
    /// resistance, narrowed to the dominant footprint and sites, split into
    /// five bands of drive resistance, and the best two of each band by
    /// drive resistance times input capacitance kept.
    pub fn find_buffers(&self, db: &NetlistDB) -> Result<BufferSet, LinkError> {
        let list = self.usable_buffers();
        if list.is_empty() {
            return Err(LinkError::NoBuffers);
        }
        // NOTE: Resizer::findBuffers also keeps only the second-leakiest
        // threshold voltage (cellVTType); PARE has no VT model, which is the
        // same on single-VT libraries such as Nangate45.
        let footprint = self.dominant_footprint(&list);
        let filtered: Vec<BufferCell> = list
            .iter()
            .copied()
            .filter(|b| {
                footprint
                    .is_none_or(|best| self.liberty.cell(b.id).footprint.as_deref() == Some(best))
            })
            .collect();
        let site_of = |b: BufferCell| {
            self.macro_of(b.id)
                .and_then(|m| db.library.cell_macro(m).site)
        };
        let sites = most_used_sites(list.iter().map(|&b| site_of(b)));
        let mut chosen = Vec::new();
        for band in drive_bands(&filtered) {
            let mut band = band.to_vec();
            band.sort_by(|&a, &b| {
                let metric = |buffer: BufferCell| {
                    self.buffer_drive_resistance(buffer) * self.buffer_input_capacitance(buffer)
                };
                metric(a).total_cmp(&metric(b))
            });
            if sites.len() <= 1 {
                chosen.extend(band.iter().take(BUFFERS_PER_BAND));
            } else {
                for site in &sites {
                    if let Some(&b) = band.iter().find(|&&b| site_of(b) == *site) {
                        chosen.push(b);
                    }
                }
            }
        }
        let lowest_drive = *chosen.last().ok_or(LinkError::NoBuffers)?;
        Ok(BufferSet {
            cells: chosen,
            lowest_drive,
        })
    }

    /// The usable buffers that are neither clock buffers nor
    /// power-management cells, by rising drive resistance and then name, as
    /// `Resizer::getBufferList` filters them.
    fn usable_buffers(&self) -> Vec<BufferCell> {
        let mut list: Vec<BufferCell> = self
            .liberty
            .cells
            .ids()
            .filter(|&id| {
                let cell = self.liberty.cell(id);
                cell.is_buffer()
                    && !self.dont_use(id)
                    && self.is_link_cell(id)
                    && !cell.is_power_cell
                    && !is_clock_buffer(cell)
            })
            .filter_map(|id| self.liberty.buffer_cell(id))
            .collect();
        list.sort_by(|&a, &b| {
            self.buffer_drive_resistance(a)
                .total_cmp(&self.buffer_drive_resistance(b))
                .then_with(|| {
                    self.liberty
                        .cell(a.id)
                        .name
                        .cmp(&self.liberty.cell(b.id).name)
                })
        });
        list
    }

    /// The footprint more than [`DOMINANT_FOOTPRINT_SHARE`] of `buffers`
    /// have, when they have more than one.
    fn dominant_footprint<'l>(&'l self, buffers: &[BufferCell]) -> Option<&'l str> {
        let mut counts: Vec<(&str, usize)> = Vec::new();
        for footprint in buffers
            .iter()
            .filter_map(|b| self.liberty.cell(b.id).footprint.as_deref())
        {
            match counts.iter_mut().find(|(f, _)| *f == footprint) {
                Some((_, n)) => *n += 1,
                None => counts.push((footprint, 1)),
            }
        }
        if counts.len() <= 1 {
            return None;
        }
        counts
            .iter()
            .find(|(_, n)| *n as f64 / buffers.len() as f64 > DOMINANT_FOOTPRINT_SHARE)
            .map(|(f, _)| *f)
    }
}

/// The [`BUFFER_SITES`] sites the most of `sites` stand on, the most used
/// first; buffers without a site count as one.
fn most_used_sites(sites: impl Iterator<Item = Option<SiteId>>) -> Vec<Option<SiteId>> {
    let mut counts: Vec<(Option<SiteId>, usize)> = Vec::new();
    for site in sites {
        match counts.iter_mut().find(|(s, _)| *s == site) {
            Some((_, n)) => *n += 1,
            None => counts.push((site, 1)),
        }
    }
    counts.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
    counts
        .into_iter()
        .take(BUFFER_SITES)
        .map(|(s, _)| s)
        .collect()
}

/// `buffers` split into [`DRIVE_BANDS`] consecutive bands as evenly as they
/// go, the first bands taking one more where they do not divide.
fn drive_bands(buffers: &[BufferCell]) -> impl Iterator<Item = &[BufferCell]> {
    let n = buffers.len();
    let size = n / DRIVE_BANDS;
    let remainder = n % DRIVE_BANDS;
    (0..DRIVE_BANDS).map(move |band| {
        let start = band * size + band.min(remainder);
        let end = (start + size + usize::from(band < remainder)).min(n);
        &buffers[start.min(n)..end]
    })
}

impl Link<'_> {
    /// Whether a cell may go into a clock tree, as `OpenROAD`'s
    /// `isClockCellCandidate`: one that may be used and is no
    /// power-management cell.
    #[must_use]
    pub fn is_clock_cell_candidate(&self, cell: LibCellId) -> bool {
        let lib = self.liberty.cell(cell);
        !self.dont_use(cell) && !lib.is_power_cell
    }

    /// The buffers clock trees are built from, as `OpenROAD`'s
    /// `inferClockBufferList`: of the candidate buffers with a master, in
    /// library order, those the library marks as clock cells, else those
    /// with a LEF clock input, else those named `CLKBUF`, else those named
    /// `BUF`, else all of them.
    #[must_use]
    pub fn infer_clock_buffers(&self, db: &NetlistDB) -> Vec<BufferCell> {
        let candidates: Vec<BufferCell> = (0..self.liberty.cells.len())
            .map(|i| LibCellId(i as u32))
            .filter(|&id| {
                self.liberty.cell(id).is_buffer()
                    && self.is_link_cell(id)
                    && self.is_clock_cell_candidate(id)
            })
            .filter_map(|id| self.liberty.buffer_cell(id))
            .collect();
        let has_clock_input =
            |b: BufferCell| {
                self.macro_of(b.id).is_some_and(|m| {
                    db.library.cell_macro(m).pins.iter().any(|p| {
                        p.direction == PinDirection::Input && p.signal_use == SignalUse::Clock
                    })
                })
            };
        let named = |b: BufferCell, pattern: &str| {
            self.liberty
                .cell(b.id)
                .name
                .to_ascii_uppercase()
                .contains(pattern)
        };
        let tiers: [&dyn Fn(BufferCell) -> bool; 4] = [
            &|b| self.liberty.cell(b.id).is_clock_cell,
            &has_clock_input,
            &|b| named(b, "CLKBUF"),
            &|b| named(b, "BUF"),
        ];
        for tier in tiers {
            let chosen: Vec<BufferCell> = candidates.iter().copied().filter(|&b| tier(b)).collect();
            if !chosen.is_empty() {
                return chosen;
            }
        }
        candidates
    }
}

/// A cell's leakage, as `Resizer::cellLeakage`: its `cell_leakage_power`,
/// else the mean of its per-state leakages.
fn cell_leakage(cell: &LibCell) -> Option<Watts> {
    if cell.leakage.is_some() || cell.state_leakages.is_empty() {
        return cell.leakage;
    }
    let total: Watts = cell.state_leakages.iter().copied().sum();
    Some(total / cell.state_leakages.len() as f64)
}

/// Whether a buffer is meant for clock trees, as `Resizer::getBufferUse`
/// tells by default: the library marks it a clock cell or `CLKBUF` is in
/// its name.
fn is_clock_buffer(cell: &LibCell) -> bool {
    cell.is_clock_cell || cell.name.to_ascii_lowercase().contains("clkbuf")
}

/// Groups the cells that may be used into classes of mutually equivalent
/// cells, as `OpenSTA`'s `EquivCells`, each sorted by falling drive
/// resistance; a cell in no class maps to `None`.
fn equivalence_classes(
    liberty: &Liberty,
    dont_use: &IdVec<LibCellId, bool>,
) -> (IdVec<LibCellId, Option<usize>>, Vec<Vec<LibCellId>>) {
    let mut class_of: IdVec<LibCellId, Option<usize>> = IdVec::filled(None, liberty.cells.len());
    let mut classes: Vec<Vec<LibCellId>> = Vec::new();
    let mut by_ports: HashMap<Vec<&str>, Vec<LibCellId>> = HashMap::new();
    for (i, cell) in liberty.cells.iter_enumerated() {
        if dont_use[i] {
            continue;
        }
        let mut names: Vec<&str> = cell.ports.iter().map(|p| p.name.as_str()).collect();
        names.sort_unstable();
        let candidates = by_ports.entry(names).or_default();
        let found = candidates
            .iter()
            .find(|&&other| liberty.cells[other].equivalent(cell))
            .and_then(|&other| class_of[other]);
        if let Some(class) = found {
            classes[class].push(i);
            class_of[i] = Some(class);
        } else {
            class_of[i] = Some(classes.len());
            classes.push(vec![i]);
        }
        candidates.push(i);
    }
    // NOTE: EquivCells uses std::sort, which over a class this small is an
    // insertion sort, so cells of equal drive keep library order.
    for class in &mut classes {
        class.sort_by(|&a, &b| {
            output_drive_resistance(liberty.cell(b))
                .total_cmp(&output_drive_resistance(liberty.cell(a)))
        });
    }
    (class_of, classes)
}

/// The drive resistance of a cell's first output, as `EquivCells` sorts
/// by; zero for a cell without one.
fn output_drive_resistance(cell: &LibCell) -> Ohms {
    cell.port_ids()
        .find(|&p| cell.ports[p].is_output())
        .map_or(Ohms::ZERO, |port| cell.drive_resistance(port))
}

/// Whether an instance is a plain standard cell, as `OpenROAD`'s
/// `isLogicStdCell`: its master is of class `CORE` with no subclass.
pub(crate) fn is_logic_std_cell(db: &NetlistDB, instance: CellId) -> bool {
    matches!(
        db.cell_macro(instance).class,
        MacroClass::Core(CoreKind::Regular)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{DefSections, cells_with, def_text, design_with_copies};
    use pare_common::db::library::Site;
    use pare_common::db::parser::liberty::parse_str;
    use std::fmt::Write;
    use std::path::Path;

    /// A buffer cell `name` of `footprint` on the clock cells' tables whose
    /// output slews to `slew` nanoseconds at 100 fF, which sets its drive.
    fn buffer_cell(name: &str, footprint: &str, slew: f64) -> String {
        format!(
            "cell ({name}) {{ area : 0.8 ; cell_footprint : \"{footprint}\" ;
               pin (A) {{ direction : input ; capacitance : 1.0 ; }}
               pin (Z) {{ direction : output ; function : \"A\" ;
                 timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
                   cell_rise (delay_t) {{ values (\"0.03, 0.30\", \"0.04, 0.31\") ; }}
                   rise_transition (delay_t) {{
                     values (\"0.0, {slew}\", \"0.0, {slew}\") ; }} }} }} }}\n"
        )
    }

    /// The names `find_buffers` chooses among buffers `B0`, `B1`, ... of
    /// the `(footprint, site)` of `buffers`, each driving more weakly than
    /// the last, in a design without instances where they copy `BUF_X1`,
    /// which is not to be used.
    fn find_among(buffers: &[(&str, &str)]) -> Result<Vec<String>, LinkError> {
        let names: Vec<String> = (0..buffers.len()).map(|i| format!("B{i}")).collect();
        let copies: Vec<(&str, &str)> = names.iter().map(|n| ("BUF_X1", n.as_str())).collect();
        let mut db = design_with_copies(
            &def_text(
                "buffers",
                &DefSections {
                    components: "",
                    nets: "",
                    inputs: &[],
                    outputs: &[],
                },
            ),
            &copies,
        );
        let core = db.library.site(db.library.site_id("core").unwrap()).clone();
        for (name, &(_, site)) in names.iter().zip(buffers) {
            let site = db.library.site_id(site).unwrap_or_else(|| {
                db.library
                    .add_site(Site {
                        name: site.to_string(),
                        ..core.clone()
                    })
                    .unwrap()
            });
            let id = db.library.macro_id(name).unwrap();
            db.library.macros[id].site = Some(site);
        }
        let extra: String = names
            .iter()
            .zip(buffers)
            .enumerate()
            .map(|(i, (name, (footprint, _)))| buffer_cell(name, footprint, 0.1 * (i + 1) as f64))
            .collect();
        let lib = cells_with(&extra);
        let link = Link::new(&db, &lib, &["BUF_X1".to_string()]).unwrap();
        let set = link.find_buffers(&db)?;
        assert_eq!(set.lowest_drive, *set.cells.last().unwrap());
        Ok(set
            .cells
            .iter()
            .map(|b| lib.cell(b.id).name.clone())
            .collect())
    }

    #[test]
    fn the_two_strongest_buffers_of_each_of_five_drive_bands_are_chosen() {
        let chosen = find_among(&[("buf", "core"); 15]).unwrap();

        assert_eq!(
            chosen,
            [
                "B0", "B1", "B3", "B4", "B6", "B7", "B9", "B10", "B12", "B13"
            ]
        );
    }

    #[test]
    fn a_footprint_most_buffers_share_leaves_out_the_others() {
        let buffers = [
            ("buf", "core"),
            ("buf", "core"),
            ("dly", "core"),
            ("buf", "core"),
        ];

        let chosen = find_among(&buffers).unwrap();

        assert_eq!(chosen, ["B0", "B1", "B3"]);
    }

    #[test]
    fn footprints_split_evenly_leave_out_none() {
        let buffers = [
            ("buf", "core"),
            ("dly", "core"),
            ("dly", "core"),
            ("buf", "core"),
        ];

        let chosen = find_among(&buffers).unwrap();

        assert_eq!(chosen, ["B0", "B1", "B2", "B3"]);
    }

    #[test]
    fn buffers_on_two_sites_give_each_band_the_strongest_of_each_site() {
        let buffers: Vec<(&str, &str)> = (0..15)
            .map(|i| ("buf", if i % 2 == 0 { "core" } else { "tall" }))
            .collect();

        let chosen = find_among(&buffers).unwrap();

        assert_eq!(
            chosen,
            [
                "B0", "B1", "B4", "B3", "B6", "B7", "B10", "B9", "B12", "B13"
            ]
        );
    }

    #[test]
    fn a_library_of_only_clock_and_unusable_buffers_has_none_to_choose() {
        assert_eq!(find_among(&[]), Err(LinkError::NoBuffers));
    }

    /// The design of [`find_among`] over buffers `B0`, `B1`, ... copying
    /// `BUF_X1`, each with the Liberty attributes in `attributes`.
    fn attributed_buffers(attributes: &[&str]) -> (NetlistDB, Liberty) {
        let names: Vec<String> = (0..attributes.len()).map(|i| format!("B{i}")).collect();
        let copies: Vec<(&str, &str)> = names.iter().map(|n| ("BUF_X1", n.as_str())).collect();
        let sections = DefSections {
            components: "",
            nets: "",
            inputs: &[],
            outputs: &[],
        };
        let db = design_with_copies(&def_text("buffers", &sections), &copies);
        let extra: String = names
            .iter()
            .zip(attributes)
            .map(|(name, attribute)| {
                buffer_cell(name, "buf", 0.1).replacen(
                    "area : 0.8 ;",
                    &format!("area : 0.8 ; {attribute}"),
                    1,
                )
            })
            .collect();
        (db, cells_with(&extra))
    }

    /// The names of `cells` of `lib`.
    fn names(lib: &Liberty, cells: &[LibCellId]) -> Vec<String> {
        cells.iter().map(|&c| lib.cell(c).name.clone()).collect()
    }

    #[test]
    fn a_cell_whose_master_is_no_core_cell_has_no_sizes() {
        let (mut db, lib) = attributed_buffers(&["", ""]);
        let block = db.library.macro_id("B0").unwrap();
        db.library.macros[block].class = MacroClass::Block;
        let link = Link::new(&db, &lib, &[]).unwrap();
        let id = |name: &str| lib.cell_id(name).unwrap();

        let of_block = link.swappable_cells(id("B0"), SizingLimits::NONE);
        let of_core = link.swappable_cells(id("B1"), SizingLimits::NONE);

        assert_eq!(of_block, []);
        assert!(of_core.contains(&id("B1")) && of_core.contains(&id("B0")));
    }

    #[test]
    fn a_user_function_class_keeps_sizes_within_it() {
        let class = |name: &str| format!("user_function_class : \"{name}\" ;");
        let (db, lib) = attributed_buffers(&[&class("a"), &class("a"), &class("b"), ""]);
        let link = Link::new(&db, &lib, &[]).unwrap();
        let id = |name: &str| lib.cell_id(name).unwrap();

        let of_a = names(&lib, &link.swappable_cells(id("B0"), SizingLimits::NONE));
        let of_none = names(&lib, &link.swappable_cells(id("B3"), SizingLimits::NONE));

        assert!(of_a.iter().all(|n| n == "B0" || n == "B1"), "{of_a:?}");
        assert_eq!(of_a.len(), 2);
        assert!(
            ["B0", "B1", "B2", "B3"]
                .iter()
                .all(|n| of_none.iter().any(|m| m == n)),
            "{of_none:?}"
        );
    }

    #[test]
    fn clock_cells_and_power_management_buffers_are_not_chosen() {
        let attributes = [
            ("B0", "is_clock_cell : true ;"),
            ("B1", "is_level_shifter : true ;"),
            ("B2", "is_isolation_cell : true ;"),
            ("B3", "always_on : true ;"),
            ("B4", ""),
        ];
        let copies: Vec<(&str, &str)> = attributes.iter().map(|&(n, _)| ("BUF_X1", n)).collect();
        let sections = DefSections {
            components: "",
            nets: "",
            inputs: &[],
            outputs: &[],
        };
        let db = design_with_copies(&def_text("buffers", &sections), &copies);
        let extra: String = attributes
            .iter()
            .map(|&(name, attribute)| {
                buffer_cell(name, "buf", 0.1).replacen(
                    "area : 0.8 ;",
                    &format!("area : 0.8 ; {attribute}"),
                    1,
                )
            })
            .collect();
        let lib = cells_with(&extra);
        let link = Link::new(&db, &lib, &["BUF_X1".to_string()]).unwrap();

        let set = link.find_buffers(&db).unwrap();

        let chosen: Vec<&str> = set
            .cells
            .iter()
            .map(|b| lib.cell(b.id).name.as_str())
            .collect();
        assert_eq!(chosen, ["B4"]);
    }

    /// Buffers in library order, each slewing to `slew` nanoseconds at
    /// 2 fF, which sets its drive resistance.
    fn buffers(slews: &[(&str, f64)]) -> Liberty {
        let mut cells = String::new();
        for (name, slew) in slews {
            write!(
                cells,
                "cell ({name}) {{
                   pin (A) {{ direction : input; capacitance : 1.0; }}
                   pin (Z) {{ direction : output; function : \"A\";
                     timing () {{ related_pin : \"A\"; timing_sense : positive_unate;
                       cell_rise(scalar) {{ values (\"0.01\"); }}
                       rise_transition(load) {{ values (\"0.0, {slew}\"); }} }} }} }}"
            )
            .unwrap();
        }
        let src = format!(
            "library (t) {{ time_unit : \"1ns\"; capacitive_load_unit (1,ff);
               lu_table_template (load) {{
                 variable_1 : total_output_net_capacitance; index_1 (\"1.0, 2.0\"); }}
               {cells} }}"
        );
        parse_str(&src, Path::new("t.lib")).unwrap()
    }

    fn measures(area: i64, leakage_nw: f64) -> CellMeasures {
        CellMeasures {
            area: Some(DbuArea(area)),
            leakage: Some(Watts::new(leakage_nw * 1e-9)),
        }
    }

    #[test]
    fn a_size_over_four_times_the_leakage_is_not_offered() {
        let buf_x1 = measures(798, 21.438_247);
        let buf_x4 = measures(1862, 86.121_805);

        assert!(!SizingLimits::OPENROAD.allows(buf_x1, buf_x4));
    }

    #[test]
    fn a_size_just_under_four_times_the_leakage_is_offered() {
        let oai21_x1 = measures(1064, 22.619_394);
        let oai21_x4 = measures(2660, 90.477_187);

        assert!(SizingLimits::OPENROAD.allows(oai21_x1, oai21_x4));
    }

    #[test]
    fn a_size_over_four_times_the_area_is_not_offered_and_four_times_is() {
        let small = measures(100, 1.0);

        assert!(SizingLimits::OPENROAD.allows(small, measures(400, 1.0)));
        assert!(!SizingLimits::OPENROAD.allows(small, measures(401, 1.0)));
    }

    #[test]
    fn missing_or_zero_measures_and_lifted_limits_offer_every_size() {
        let none = CellMeasures {
            area: None,
            leakage: None,
        };
        let big = measures(10_000, 1000.0);

        assert!(SizingLimits::OPENROAD.allows(none, big));
        assert!(SizingLimits::OPENROAD.allows(measures(0, 0.0), measures(0, 0.0)));
        assert!(SizingLimits::NONE.allows(measures(1, 1.0), big));
    }

    #[test]
    fn any_leakage_over_a_leakless_size_is_not_offered() {
        assert!(!SizingLimits::OPENROAD.allows(measures(1, 0.0), measures(1, 1.0)));
    }

    fn leaky(body: &str) -> LibCell {
        let src = format!(
            "library (t) {{ leakage_power_unit : \"1nW\";
               cell (c) {{ {body} pin (A) {{ direction : input; capacitance : 1.0; }} }} }}"
        );
        parse_str(&src, Path::new("t.lib")).unwrap().cells[LibCellId(0)].clone()
    }

    #[test]
    fn leakage_is_the_cell_value_else_the_mean_over_states_else_unknown() {
        let stated = leaky("cell_leakage_power : 5.0; leakage_power () { value : 9.0; }");
        let by_state = leaky("leakage_power () { value : 2.0; } leakage_power () { value : 4.0; }");

        assert_eq!(cell_leakage(&stated), Some(Watts::new(5.0e-9)));
        assert!((cell_leakage(&by_state).unwrap().get() - 3.0e-9).abs() < 1e-20);
        assert_eq!(cell_leakage(&leaky("")), None);
    }

    #[test]
    fn equivalent_cells_run_from_weakest_drive_and_ties_keep_library_order() {
        let lib = buffers(&[("A", 0.2), ("B", 0.6), ("C", 0.4), ("D", 0.4)]);

        let (_, classes) = equivalence_classes(&lib, &IdVec::filled(false, 4));
        let names: Vec<&str> = classes[0]
            .iter()
            .map(|&c| lib.cell(c).name.as_str())
            .collect();

        assert_eq!(classes.len(), 1);
        assert_eq!(names, ["B", "C", "D", "A"]);
    }
}
