use crate::db::core::{LayerDirection, NetlistDB};
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use anyhow::{Context, Result};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

pub fn parse(db: &mut NetlistDB, aux_filename: &str) -> Result<()> {
    let aux_path = Path::new(aux_filename);
    let parent_dir = aux_path.parent().unwrap_or(Path::new("."));

    let file =
        File::open(aux_path).context(format!("Failed to open AUX file: {}", aux_filename))?;
    let reader = BufReader::new(file);

    let mut nodes_file = String::new();
    let mut nets_file = String::new();
    let mut pl_file = String::new();
    let mut scl_file = String::new();

    // Parse AUX file
    for line in reader.lines() {
        let line = line?;
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }
        if parts[0].starts_with("RowBasedPlacement") {
            for part in &parts[2..] {
                if part.ends_with(".nodes") {
                    nodes_file = part.to_string();
                } else if part.ends_with(".nets") {
                    nets_file = part.to_string();
                } else if part.ends_with(".pl") {
                    pl_file = part.to_string();
                } else if part.ends_with(".scl") {
                    scl_file = part.to_string();
                }
            }
        }
    }

    // Parse Nodes
    if !nodes_file.is_empty() {
        let path = parent_dir.join(&nodes_file);
        parse_nodes(db, path.to_str().unwrap())?;
    }

    // Parse PL
    if !pl_file.is_empty() {
        let path = parent_dir.join(&pl_file);
        parse_pl(db, path.to_str().unwrap())?;
    }

    // Parse SCL
    if !scl_file.is_empty() {
        let path = parent_dir.join(&scl_file);
        parse_scl(db, path.to_str().unwrap())?;
    }

    // Parse Nets
    if !nets_file.is_empty() {
        let path = parent_dir.join(&nets_file);
        parse_nets(db, path.to_str().unwrap())?;
    }

    // Inject Default Metal Stack
    if db.layers.is_empty() {
        let mut height_counts = std::collections::HashMap::new();
        for cell in &db.cells {
            if !cell.is_macro && cell.height > 0.0 {
                let h_key = (cell.height * 1000.0) as i64;
                *height_counts.entry(h_key).or_insert(0) += 1;
            }
        }

        let std_height = height_counts
            .into_iter()
            .max_by_key(|&(_, count)| count)
            .map(|(h, _)| h as f64 / 1000.0)
            .unwrap_or(16.0);

        let pitch = std_height / 8.0;
        let width = pitch * 0.5;

        log::warn!("Bookshelf: No layers defined. Synthesizing 6 default layers.");
        log::info!(
            "Bookshelf: Detected StdCell Height = {:.2}. Setting Layer Pitch = {:.2}",
            std_height,
            pitch
        );

        db.add_layer("M1".to_string(), LayerDirection::Horizontal, pitch, width);
        db.add_layer("M2".to_string(), LayerDirection::Vertical, pitch, width);
        db.add_layer("M3".to_string(), LayerDirection::Horizontal, pitch, width);
        db.add_layer("M4".to_string(), LayerDirection::Vertical, pitch, width);
        db.add_layer("M5".to_string(), LayerDirection::Horizontal, pitch, width);
        db.add_layer("M6".to_string(), LayerDirection::Vertical, pitch, width);
    }

    Ok(())
}

fn parse_nodes(db: &mut NetlistDB, filename: &str) -> Result<()> {
    log::info!("Parsing Nodes: {}", filename);
    let file = File::open(filename)?;
    let reader = BufReader::new(file);

    for line in reader.lines() {
        let line = line?;
        let line = line.trim();
        if line.is_empty()
            || line.starts_with('#')
            || line.starts_with("UCLA")
            || line.starts_with("Num")
        {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 3 {
            continue;
        }

        let name = parts[0].to_string();
        let width: f64 = parts[1].parse().unwrap_or(1.0);
        let height: f64 = parts[2].parse().unwrap_or(1.0);
        let is_terminal = line.contains("terminal");

        let lib_name = format!("BLK_{}_{}", width as i32, height as i32);

        if !db.macro_sizes.contains_key(&lib_name) {
            db.macro_sizes.insert(lib_name.clone(), (width, height));
        }

        db.add_cell(name, lib_name, width, height, is_terminal);
    }
    Ok(())
}

fn parse_pl(db: &mut NetlistDB, filename: &str) -> Result<()> {
    log::info!("Parsing PL: {}", filename);
    let file = File::open(filename)?;
    let reader = BufReader::new(file);

    for line in reader.lines() {
        let line = line?;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("UCLA") {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 3 {
            continue;
        }

        let name = parts[0];
        let x: f64 = parts[1].parse().unwrap_or(0.0);
        let y: f64 = parts[2].parse().unwrap_or(0.0);
        let is_fixed = line.contains("FIXED");

        let mut orientation = "N";
        if parts.len() >= 5 && parts[3] == ":" {
            orientation = parts[4];
        }

        if let Some(&id) = db.cell_name_map.get(name) {
            db.positions[id.index()] = Point::new(x, y);

            if orientation == "E"
                || orientation == "W"
                || orientation == "FE"
                || orientation == "FW"
            {
                let cell = &mut db.cells[id.index()];
                std::mem::swap(&mut cell.width, &mut cell.height);
            }

            if is_fixed {
                db.cells[id.index()].is_fixed = true;
            }
        }
    }
    Ok(())
}

fn parse_scl(db: &mut NetlistDB, filename: &str) -> Result<()> {
    log::info!("Parsing SCL: {}", filename);
    let file = File::open(filename)?;
    let reader = BufReader::new(file);

    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    let mut current_row_y = 0.0;
    let mut current_row_h = 1.0;
    let mut current_site_w = 1.0;

    for line in reader.lines() {
        let line = line?;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("UCLA") {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();

        if line.starts_with("Coordinate") {
            if let Some(val) = parts.get(2) {
                current_row_y = val.parse().unwrap_or(0.0);
            }
        } else if line.starts_with("Height") {
            if let Some(val) = parts.get(2) {
                current_row_h = val.parse().unwrap_or(1.0);
            }
        } else if line.starts_with("Sitewidth") {
            if let Some(val) = parts.get(2) {
                current_site_w = val.parse().unwrap_or(1.0);
            }
        } else if line.starts_with("SubrowOrigin")
            && let Some(origin_str) = parts.get(2)
        {
            let origin: f64 = origin_str.parse().unwrap_or(0.0);
            let num_sites_str = parts.get(5).unwrap_or(&"0");
            let num_sites: u32 = num_sites_str.parse().unwrap_or(0);

            let row_width = num_sites as f64 * current_site_w;

            min_x = min_x.min(origin);
            max_x = max_x.max(origin + row_width);
            min_y = min_y.min(current_row_y);
            max_y = max_y.max(current_row_y + current_row_h);
        }
    }

    if min_x < max_x {
        db.die_area = Rect::new(Point::new(min_x, min_y), Point::new(max_x, max_y));
        log::info!("Die Area inferred from SCL: {:?}", db.die_area);
    }

    Ok(())
}

fn parse_nets(db: &mut NetlistDB, filename: &str) -> Result<()> {
    log::info!("Parsing Nets: {}", filename);
    let file = File::open(filename)?;
    let reader = BufReader::new(file);

    let mut current_net_id = None;
    let mut net_counter = 0;

    for line in reader.lines() {
        let line = line?;
        let line = line.trim();
        if line.is_empty()
            || line.starts_with('#')
            || line.starts_with("UCLA")
            || line.starts_with("Num")
        {
            continue;
        }

        if line.starts_with("NetDegree") {
            let net_name = format!("n{}", net_counter);
            net_counter += 1;
            current_net_id = Some(db.add_net(net_name));
        } else if let Some(net_id) = current_net_id {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if let Some(node_name) = parts.first()
                && let Some(&cell_id) = db.cell_name_map.get(*node_name)
            {
                let cell = &db.cells[cell_id.index()];
                let offset = Point::new(cell.width / 2.0, cell.height / 2.0);
                db.add_pin(cell_id, net_id, offset, "pin".to_string());
            }
        }
    }
    Ok(())
}
