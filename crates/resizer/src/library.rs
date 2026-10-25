//! The timing library joined to the design's masters: which Liberty cell
//! each master is, which cells may be used, which buffers are inserted and
//! which sizes a cell can swap to.

use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{CellId, MacroId, PinId, SiteId};
use pare_common::db::liberty::{LibCell, LibCellId, Liberty};
use pare_common::db::library::{CoreKind, MacroClass};
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
    port_of_pin: Vec<Vec<Option<usize>>>,
    dont_use: Vec<bool>,
    class_of: Vec<Option<usize>>,
    classes: Vec<Vec<LibCellId>>,
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
        Ok(Self {
            liberty,
            cell_of_macro,
            macro_of_cell,
            port_of_pin,
            dont_use: dont_use_flags,
            class_of,
            classes,
        })
    }

    /// The Liberty cell of a master.
    #[must_use]
    pub fn cell_of_macro(&self, master: MacroId) -> Option<LibCellId> {
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
    pub fn pin_port(&self, db: &NetlistDB, pin: PinId) -> Option<(LibCellId, usize)> {
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
    pub fn macro_pin_of_port(&self, cell: LibCellId, port: usize) -> Option<usize> {
        let master = self.macro_of(cell)?;
        self.port_of_pin[master.index()]
            .iter()
            .position(|p| *p == Some(port))
    }

    /// Whether an instance is a plain standard cell, as `OpenROAD`'s
    /// `isLogicStdCell`: its master is of class `CORE` with no subclass.
    #[must_use]
    pub fn is_logic_std_cell(&self, db: &NetlistDB, instance: CellId) -> bool {
        matches!(
            db.cell_macro(instance).class,
            MacroClass::Core(CoreKind::Regular)
        )
    }

    /// The cells an instance of `cell` may be swapped for, itself included,
    /// as `OpenROAD`'s `getSwappableCells`: the equivalent cells that have
    /// a master and may be used, or just the cell when it has no class.
    #[must_use]
    pub fn swappable_cells(&self, cell: LibCellId) -> Vec<LibCellId> {
        let Some(class) = self.class_of[cell.index()] else {
            return vec![cell];
        };
        self.classes[class]
            .iter()
            .copied()
            .filter(|&c| !self.dont_use(c) && self.is_link_cell(c))
            .collect()
    }

    /// Drive resistance of a buffer's output.
    #[must_use]
    pub fn buffer_drive_resistance(&self, buffer: LibCellId) -> f64 {
        let cell = self.liberty.cell(buffer);
        cell.buffer_ports()
            .map_or(0.0, |(_, output)| cell.drive_resistance(output))
    }

    /// Capacitance of a buffer's input.
    #[must_use]
    pub fn buffer_input_capacitance(&self, buffer: LibCellId) -> f64 {
        let cell = self.liberty.cell(buffer);
        cell.buffer_ports()
            .map_or(0.0, |(input, _)| cell.ports[input].capacitance())
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

/// Whether a buffer is meant for clock trees: `OpenROAD`'s default test,
/// `CLKBUF` in its name.
fn is_clock_buffer(cell: &LibCell) -> bool {
    cell.name.to_ascii_lowercase().contains("clkbuf")
}

/// Groups the cells that may be used into classes of mutually equivalent
/// cells, as `OpenSTA`'s `EquivCells`; a cell in no class maps to `None`.
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
    (class_of, classes)
}
