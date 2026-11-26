//! The order of a run's stages, as the types of the design between them:
//! a design is placed before it is routed globally, and routed globally
//! before it is routed in detail. `docs/cli-flow.md` draws it.

use crate::placement::{PlacementStart, run_placement};
use crate::plan::FlowPlan;
use crate::repair::NetlistRepair;
use crate::report::Report;
use crate::routing::{
    RouteStart, RouterRun, design_leaf_clock_pins, global_route_with_repair, route_detailed,
    route_global,
};
use crate::ui;
use pare_common::config::{Config, DesignInput};
use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{bookshelf, def, lef};
use pare_router::LeafClockPins;
use std::path::Path;

/// A design on its way through the flow, at stage `S`.
pub(crate) struct Design<S> {
    db: NetlistDB,
    stage: S,
}

/// Read from its input, as yet untouched.
pub(crate) struct Loaded;

/// Placed, legal and written out, or placed by its input.
pub(crate) struct Placed<'p> {
    /// The netlist repair placement ran, for the repair on the global
    /// routes; none when it ran none.
    repair: Option<NetlistRepair<'p>>,
}

/// Routed globally, ready for detailed routing.
pub(crate) struct GloballyRouted {
    /// The global routes repair ran on, which detailed routing goes on
    /// from, or when no repair needed them, the leaf clock pins detailed
    /// routing's own global routing takes.
    start: RouteStart,
}

impl<S> Design<S> {
    /// The design as it stands.
    pub(crate) const fn db(&self) -> &NetlistDB {
        &self.db
    }
}

impl Design<Loaded> {
    /// Reads the design `config` names, with `placement` in place of its
    /// own, recording its name in `report`.
    pub(crate) fn load(
        config: &Config,
        placement: Option<&Path>,
        report: &mut Report,
    ) -> anyhow::Result<Self> {
        let db = read_design(config, placement)?;
        report.design.clone_from(&db.name);
        Ok(Self { db, stage: Loaded })
    }

    /// Places the design from `start` as `plan` resolves the config, then
    /// checks the placement and writes it out.
    pub(crate) fn place<'p>(
        self,
        config: &Config,
        plan: &'p FlowPlan,
        start: PlacementStart,
        report: &mut Report,
    ) -> anyhow::Result<Design<Placed<'p>>> {
        let mut db = self.db;
        let repair = run_placement(config, plan, &mut db, start, report)?;
        Ok(Design {
            db,
            stage: Placed { repair },
        })
    }

    /// The design as its input placed it, for a routing benchmark or the
    /// `route` command.
    pub(crate) fn placed_by_input(self) -> Design<Placed<'static>> {
        Design {
            db: self.db,
            stage: Placed { repair: None },
        }
    }
}

impl Design<Placed<'_>> {
    /// The pins that make a clock net a leaf, by the Liberty placement's
    /// repair read or else the config names.
    fn leaf_clock_pins(&self, config: &Config) -> anyhow::Result<LeafClockPins> {
        design_leaf_clock_pins(config, &self.db, self.stage.repair.as_ref())
    }

    /// Routes the design globally and assigns tracks, without detailed
    /// routing, writing the guides to `guides` when given.
    pub(crate) fn route_globally_only(
        &self,
        config: &Config,
        guides: Option<&Path>,
        report: &mut Report,
    ) -> anyhow::Result<()> {
        let leaves = self.leaf_clock_pins(config)?;
        route_global(config, &self.db, leaves, guides, report)
    }

    /// Routes the design globally and repairs it on the global routes'
    /// parasitics, when placement repaired the netlist and the config asks
    /// for repair on them.
    pub(crate) fn route_globally(
        self,
        config: &Config,
        report: &mut Report,
    ) -> anyhow::Result<Design<GloballyRouted>> {
        let leaves = self.leaf_clock_pins(config)?;
        let Self {
            mut db,
            stage: Placed { repair },
        } = self;
        let session = match repair {
            Some(mut repair) => global_route_with_repair(config, &mut db, &mut repair, report)?,
            None => None,
        };
        let start = session.map_or(RouteStart::FromScratch(leaves), |session| {
            RouteStart::OnGlobalRoutes(Box::new(session))
        });
        Ok(Design {
            db,
            stage: GloballyRouted { start },
        })
    }
}

impl Design<GloballyRouted> {
    /// Routes the design in detail as `run` says, then checks the result
    /// and writes it out; fails when it has opens or rule violations.
    pub(crate) fn route(
        self,
        config: &Config,
        run: &RouterRun,
        report: &mut Report,
    ) -> anyhow::Result<()> {
        let Self {
            mut db,
            stage: GloballyRouted { start },
        } = self;
        route_detailed(config, &mut db, run, start, report)
    }
}

/// Reads the design named by the `[input]` section. `placement` replaces the
/// input DEF, or for a Bookshelf design supplies a `.pl` to apply.
pub(crate) fn read_design(config: &Config, placement: Option<&Path>) -> anyhow::Result<NetlistDB> {
    match &config.input.design {
        DesignInput::Bookshelf {
            aux_file,
            num_layers,
        } => {
            ui::header("Input", &aux_file.display().to_string());
            let mut db = bookshelf::parse(aux_file, *num_layers)?;
            if let Some(pl) = placement {
                ui::header("Input", &pl.display().to_string());
                bookshelf::read_placement(&mut db, pl)?;
            }
            Ok(db)
        }
        DesignInput::LefDef {
            lef_files,
            def_file,
        } => {
            let def_path = placement.unwrap_or(def_file);
            for lef in lef_files {
                ui::header("Input", &lef.display().to_string());
            }
            ui::header("Input", &def_path.display().to_string());
            let units = def::read_units(def_path)?;
            let library = lef::parse_files(lef_files, units)?;
            Ok(def::parse(def_path, library)?)
        }
    }
}

#[cfg(test)]
mod tests;
