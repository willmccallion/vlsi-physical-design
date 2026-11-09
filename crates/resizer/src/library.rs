//! The timing library joined to the design's masters: which Liberty cell
//! each master is, which cells may be used, which buffers are inserted and
//! which sizes a cell can swap to.

use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{CellId, MacroId, MacroPinId, PinId, SiteId};
use pare_common::db::liberty::{LibCell, LibCellId, LibPortId, Liberty};
use pare_common::db::library::{CoreKind, MacroClass, PinDirection, SignalUse};
use pare_common::units::{Farads, Ohms, Watts};
use std::collections::HashMap;

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
    pub cells: Vec<LibCellId>,
    /// The chosen buffer with the largest drive resistance.
    pub lowest_drive: LibCellId,
}

/// A Liberty library joined to the masters of a design.
#[derive(Debug)]
pub struct Link<'a> {
    /// The timing library.
    pub liberty: &'a Liberty,
    cell_of_macro: Vec<Option<LibCellId>>,
    macro_of_cell: Vec<Option<MacroId>>,
    port_of_pin: Vec<Vec<Option<LibPortId>>>,
    dont_use: Vec<bool>,
    class_of: Vec<Option<usize>>,
    classes: Vec<Vec<LibCellId>>,
    master_area: Vec<Option<i64>>,
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
    /// Its master's area, in square database units.
    area: Option<i64>,
    /// Its leakage power.
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
            (Some(limit), Some(from), Some(to)) if from != 0 => to as f64 / from as f64 <= limit,
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
        let mut dont_use_flags: Vec<bool> = liberty.cells.iter().map(|c| c.dont_use).collect();
        for name in dont_use {
            let id = liberty
                .cell_id(name)
                .ok_or_else(|| LinkError::UnknownDontUse(name.clone()))?;
            dont_use_flags[id.index()] = true;
        }
        let mut macro_of_cell = vec![None; liberty.cells.len()];
        let mut cell_of_macro = Vec::with_capacity(db.library.macros.len());
        let mut port_of_pin = Vec::with_capacity(db.library.macros.len());
        for (m, master) in db.library.macros.iter().enumerate() {
            let cell = liberty.cell_id(&master.name);
            cell_of_macro.push(cell);
            if let Some(cell) = cell {
                macro_of_cell[cell.index()] = Some(MacroId::new(m));
            }
            let lib_cell = cell.map(|c| liberty.cell(c));
            port_of_pin.push(
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
                    let size = db.library.macros[m.index()].size;
                    size.x.get() * size.y.get()
                })
            })
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
        })
    }

    /// The Liberty cell of a master.
    #[must_use]
    pub(crate) fn cell_of_macro(&self, master: MacroId) -> Option<LibCellId> {
        self.cell_of_macro[master.index()]
    }

    /// The master of a Liberty cell.
    #[must_use]
    pub fn macro_of(&self, cell: LibCellId) -> Option<MacroId> {
        self.macro_of_cell[cell.index()]
    }

    /// Whether a Liberty cell has a master: `OpenROAD`'s link cell test.
    #[must_use]
    pub fn is_link_cell(&self, cell: LibCellId) -> bool {
        self.macro_of_cell[cell.index()].is_some()
    }

    /// Whether the cell is not to be used.
    #[must_use]
    pub fn dont_use(&self, cell: LibCellId) -> bool {
        self.dont_use[cell.index()]
    }

    /// The Liberty cell of an instance.
    #[must_use]
    pub fn cell(&self, db: &NetlistDB, instance: CellId) -> Option<(LibCellId, &'a LibCell)> {
        let id = self.cell_of_macro(db.cells[instance.index()].macro_id)?;
        Some((id, self.liberty.cell(id)))
    }

    /// The Liberty cell and port index of an instance pin.
    #[must_use]
    pub fn pin_port(&self, db: &NetlistDB, pin: PinId) -> Option<(LibCellId, LibPortId)> {
        let PinOwner::Cell {
            cell,
            pin: macro_pin,
        } = db.pins[pin.index()].owner
        else {
            return None;
        };
        let master = db.cells[cell.index()].macro_id;
        let lib = self.cell_of_macro(master)?;
        let port = self.port_of_pin[master.index()][macro_pin.index()]?;
        Some((lib, port))
    }

    /// The index in the master's pins of the Liberty port `port` of `cell`.
    #[must_use]
    pub fn macro_pin_of_port(&self, cell: LibCellId, port: LibPortId) -> Option<MacroPinId> {
        let master = self.macro_of(cell)?;
        self.port_of_pin[master.index()]
            .iter()
            .position(|p| *p == Some(port))
            .map(MacroPinId::new)
    }

    /// The cells an instance of `cell` may be swapped for, itself included,
    /// as `OpenROAD`'s `getSwappableCells`: the equivalent cells that have
    /// a master, may be used and are within `limits` of `cell`, or just the
    /// cell when it has no class.
    #[must_use]
    pub(crate) fn swappable_cells(&self, cell: LibCellId, limits: SizingLimits) -> Vec<LibCellId> {
        let Some(class) = self.class_of[cell.index()] else {
            return vec![cell];
        };
        let from = self.measures(cell);
        self.classes[class]
            .iter()
            .copied()
            .filter(|&c| !self.dont_use(c) && self.is_link_cell(c))
            .filter(|&c| limits.allows(from, self.measures(c)))
            .collect()
    }

    fn measures(&self, cell: LibCellId) -> CellMeasures {
        CellMeasures {
            area: self.master_area[cell.index()],
            leakage: cell_leakage(self.liberty.cell(cell)),
        }
    }

    /// Drive resistance of a buffer's output.
    #[must_use]
    pub(crate) fn buffer_drive_resistance(&self, buffer: LibCellId) -> Ohms {
        let cell = self.liberty.cell(buffer);
        cell.buffer_ports()
            .map_or(Ohms::ZERO, |(_, output)| cell.drive_resistance(output))
    }

    /// Capacitance of a buffer's input.
    #[must_use]
    pub(crate) fn buffer_input_capacitance(&self, buffer: LibCellId) -> Farads {
        let cell = self.liberty.cell(buffer);
        cell.buffer_ports()
            .map_or(Farads::ZERO, |(input, _)| cell.ports[input].capacitance())
    }

    /// The buffers to insert, as `OpenROAD`'s `findBuffers` chooses them:
    /// the usable buffers that are not clock buffers, sorted by drive
    /// resistance, narrowed to the dominant footprint and sites, split into
    /// five bands of drive resistance, and the best two of each band by
    /// drive resistance times input capacitance kept.
    pub fn find_buffers(&self, db: &NetlistDB) -> Result<BufferSet, LinkError> {
        let mut list: Vec<LibCellId> = (0..self.liberty.cells.len())
            .map(|i| LibCellId(i as u32))
            .filter(|&id| {
                let cell = self.liberty.cell(id);
                cell.is_buffer()
                    && !self.dont_use(id)
                    && self.is_link_cell(id)
                    && !is_clock_buffer(cell)
            })
            .collect();
        if list.is_empty() {
            return Err(LinkError::NoBuffers);
        }
        list.sort_by(|&a, &b| {
            self.buffer_drive_resistance(a)
                .total_cmp(&self.buffer_drive_resistance(b))
                .then_with(|| self.liberty.cell(a).name.cmp(&self.liberty.cell(b).name))
        });

        let footprints: Vec<Option<&str>> = list
            .iter()
            .map(|&id| self.liberty.cell(id).footprint.as_deref())
            .collect();
        let mut footprint_counts: Vec<(&str, usize)> = Vec::new();
        for footprint in footprints.iter().flatten() {
            match footprint_counts.iter_mut().find(|(f, _)| f == footprint) {
                Some((_, n)) => *n += 1,
                None => footprint_counts.push((footprint, 1)),
            }
        }
        let best_footprint = if footprint_counts.len() > 1 {
            footprint_counts
                .iter()
                .find(|(_, n)| *n as f64 / list.len() as f64 > 0.5)
                .map(|(f, _)| *f)
        } else {
            None
        };
        let filtered: Vec<LibCellId> = list
            .iter()
            .copied()
            .filter(|&id| {
                best_footprint
                    .is_none_or(|best| self.liberty.cell(id).footprint.as_deref() == Some(best))
            })
            .collect();

        let site_of = |id: LibCellId| {
            self.macro_of(id)
                .and_then(|m| db.library.cell_macro(m).site)
        };
        let mut site_counts: Vec<(Option<SiteId>, usize)> = Vec::new();
        for &id in &list {
            let site = site_of(id);
            match site_counts.iter_mut().find(|(s, _)| *s == site) {
                Some((_, n)) => *n += 1,
                None => site_counts.push((site, 1)),
            }
        }
        site_counts.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
        let best_sites: Vec<_> = site_counts.iter().take(2).map(|(s, _)| *s).collect();

        let n = filtered.len();
        let bucket_size = n / 5;
        let remainder = n % 5;
        let mut chosen = Vec::new();
        for bucket in 0..5 {
            let start = bucket * bucket_size + bucket.min(remainder);
            let end = (start + bucket_size + usize::from(bucket < remainder)).min(n);
            let mut band: Vec<LibCellId> = filtered[start.min(n)..end].to_vec();
            band.sort_by(|&a, &b| {
                let metric = |id: LibCellId| {
                    self.buffer_drive_resistance(id) * self.buffer_input_capacitance(id)
                };
                metric(a).total_cmp(&metric(b))
            });
            if best_sites.len() <= 1 {
                chosen.extend(band.iter().take(2));
            } else {
                for site in &best_sites {
                    if let Some(&id) = band.iter().find(|&&id| site_of(id) == *site) {
                        chosen.push(id);
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
    pub fn infer_clock_buffers(&self, db: &NetlistDB) -> Vec<LibCellId> {
        let candidates: Vec<LibCellId> = (0..self.liberty.cells.len())
            .map(|i| LibCellId(i as u32))
            .filter(|&id| {
                self.liberty.cell(id).is_buffer()
                    && self.is_link_cell(id)
                    && self.is_clock_cell_candidate(id)
            })
            .collect();
        let has_clock_input =
            |id: LibCellId| {
                self.macro_of(id).is_some_and(|m| {
                    db.library.cell_macro(m).pins.iter().any(|p| {
                        p.direction == PinDirection::Input && p.signal_use == SignalUse::Clock
                    })
                })
            };
        let named = |id: LibCellId, pattern: &str| {
            self.liberty
                .cell(id)
                .name
                .to_ascii_uppercase()
                .contains(pattern)
        };
        let tiers: [&dyn Fn(LibCellId) -> bool; 4] = [
            &|id| self.liberty.cell(id).is_clock_cell,
            &has_clock_input,
            &|id| named(id, "CLKBUF"),
            &|id| named(id, "BUF"),
        ];
        for tier in tiers {
            let chosen: Vec<LibCellId> =
                candidates.iter().copied().filter(|&id| tier(id)).collect();
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

/// Whether a buffer is meant for clock trees: `OpenROAD`'s default test,
/// `CLKBUF` in its name.
fn is_clock_buffer(cell: &LibCell) -> bool {
    cell.name.to_ascii_lowercase().contains("clkbuf")
}

/// Groups the cells that may be used into classes of mutually equivalent
/// cells, as `OpenSTA`'s `EquivCells`, each sorted by falling drive
/// resistance; a cell in no class maps to `None`.
fn equivalence_classes(
    liberty: &Liberty,
    dont_use: &[bool],
) -> (Vec<Option<usize>>, Vec<Vec<LibCellId>>) {
    let mut class_of: Vec<Option<usize>> = vec![None; liberty.cells.len()];
    let mut classes: Vec<Vec<LibCellId>> = Vec::new();
    let mut by_ports: HashMap<Vec<&str>, Vec<usize>> = HashMap::new();
    for (i, cell) in liberty.cells.iter().enumerate() {
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
            classes[class].push(LibCellId(i as u32));
            class_of[i] = Some(class);
        } else {
            class_of[i] = Some(classes.len());
            classes.push(vec![LibCellId(i as u32)]);
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
    use pare_common::db::parser::liberty::parse_str;
    use std::fmt::Write;
    use std::path::Path;

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
            area: Some(area),
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
        parse_str(&src, Path::new("t.lib")).unwrap().cells[0].clone()
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

        let (_, classes) = equivalence_classes(&lib, &[false; 4]);
        let names: Vec<&str> = classes[0]
            .iter()
            .map(|&c| lib.cell(c).name.as_str())
            .collect();

        assert_eq!(classes.len(), 1);
        assert_eq!(names, ["B", "C", "D", "A"]);
    }
}
