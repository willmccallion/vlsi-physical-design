//! The `view` command: one area of a design with its violations.

use crate::{ui, visualization};
use anyhow::Context;
use pare_common::db::design::NetlistDB;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::rect::Rect;
use pare_common::verify::drc::Marker;

use std::path::Path;

/// Draws the area `box_um` of `db`, in microns, with `nets` bright, and
/// lists the design rule violations there.
pub(crate) fn view_area(
    db: &NetlistDB,
    area: Rect<Dbu>,
    nets: &[String],
    layers: &[String],
    out: &Path,
    size: u32,
) -> anyhow::Result<()> {
    use pare_common::verify::drc::{Layout, Owner, Rules, check_area};
    let units = db.dbu();
    let layout = Layout::from_db(db);
    let rules = Rules::from_design(db);
    let report = check_area(&layout, &rules, area);
    let mut highlight = Vec::new();
    for name in nets {
        let net = db
            .net_id(name)
            .with_context(|| format!("no net '{name}'"))?;
        if let Some(owner) = layout.owner_index(Owner::Net(net)) {
            highlight.push(owner);
        }
    }
    let markers: Vec<&Marker> = report.violations.iter().chain(&report.advisory).collect();
    for m in &markers {
        let name = |o: pare_common::verify::drc::OwnerIndex| report.owners[o.index()].name(db);
        println!(
            "{} on {:?} at ({:.4}, {:.4})-({:.4}, {:.4}) between {} and {}",
            m.violation.name(),
            m.layer,
            units.to_microns(m.bbox.min.x),
            units.to_microns(m.bbox.min.y),
            units.to_microns(m.bbox.max.x),
            units.to_microns(m.bbox.max.y),
            name(m.owners.0),
            name(m.owners.1)
        );
    }
    let boxes: Vec<_> = markers.iter().map(|m| m.bbox).collect();
    let mut shown = vec![layers.is_empty(); db.library.routing_layers.len()];
    for name in layers {
        let layer = db
            .library
            .routing_layer_id(name)
            .with_context(|| format!("no routing layer '{name}'"))?;
        shown[layer.index()] = true;
    }
    visualization::draw_area(&layout, area, &shown, &highlight, &boxes, out, size)
        .with_context(|| format!("writing {}", out.display()))?;
    ui::wrote(out);
    Ok(())
}
