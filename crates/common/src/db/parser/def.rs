//! DEF (Design Exchange Format) Parser.
//!
//! Parses the industry-standard DEF format used to represent netlists,
//! component placements, pin locations, and routing information. This parser
//! handles the major DEF sections: DIEAREA, TRACKS, COMPONENTS, PINS, and NETS.
//! Supports multi-line statements (lines are accumulated until a semicolon).

use crate::db::core::{NetlistDB, TrackDef};
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use anyhow::Result;
use std::fs::File;
use std::io::{BufRead, BufReader};

/// Parses a DEF file and populates the netlist database.
///
/// Processes DEF statements by accumulating lines until a semicolon is found,
/// then parsing the complete statement. This correctly handles multi-line
/// statements common in real-world DEF files.
pub fn parse(db: &mut NetlistDB, filename: &str) -> Result<()> {
    let file = File::open(filename)?;
    let reader = BufReader::new(file);

    let mut in_components = false;
    let mut in_nets = false;
    let mut in_pins = false;
    let mut def_units = 1000.0;

    // Accumulator for multi-line statements.
    let mut stmt_buf = String::new();

    for line in reader.lines() {
        let line = line?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        // Accumulate into statement buffer.
        if !stmt_buf.is_empty() {
            stmt_buf.push(' ');
        }
        stmt_buf.push_str(trimmed);

        // If the line doesn't end with ';', keep accumulating.
        if !trimmed.ends_with(';') {
            // Section headers and END markers don't need ';'.
            let first = trimmed.split_whitespace().next().unwrap_or("");
            match first {
                "COMPONENTS" | "PINS" | "NETS" | "END" | "PROPERTYDEFINITIONS" => {}
                _ => continue,
            }
        }

        // We have a complete statement. Parse it.
        let owned: Vec<String> = stmt_buf.split_whitespace().map(ToString::to_string).collect();
        let parts: Vec<&str> = owned.iter().map(String::as_str).collect();
        stmt_buf.clear();

        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            "UNITS" => {
                for (i, &part) in parts.iter().enumerate() {
                    if part == "MICRONS" && i + 1 < parts.len() {
                        let val_str = parts[i + 1].trim_matches(';');
                        if let Ok(val) = val_str.parse::<f64>() {
                            def_units = val;
                            log::debug!("DEF Units updated to: {def_units}");
                        }
                        break;
                    }
                }
            }
            "DIEAREA" => {
                let x1: f64 = parts[2].parse()?;
                let y1: f64 = parts[3].parse()?;
                let x2: f64 = parts[6].parse()?;
                let y2: f64 = parts[7].parse()?;
                db.die_area = Rect::new(
                    Point::new(x1 / def_units, y1 / def_units),
                    Point::new(x2 / def_units, y2 / def_units),
                );
            }
            "TRACKS" => {
                let dir = parts[1].to_string();
                let start: f64 = parts[2].parse()?;
                let num: u32 = parts[4].parse()?;
                let step: f64 = parts[6].parse()?;
                let layer = parts[8].trim_matches(';').to_string();

                db.tracks.push(TrackDef {
                    layer,
                    direction: dir,
                    start: start / def_units,
                    num_tracks: num,
                    step: step / def_units,
                });
            }
            "COMPONENTS" => {
                in_components = true;
                in_pins = false;
                in_nets = false;
            }
            "PINS" => {
                in_pins = true;
                in_components = false;
                in_nets = false;
            }
            "NETS" => {
                in_nets = true;
                in_components = false;
                in_pins = false;
            }
            "PROPERTYDEFINITIONS" | "ROW" => {}
            "END" => {
                if parts.len() > 1 {
                    match parts[1] {
                        "COMPONENTS" => in_components = false,
                        "PINS" => in_pins = false,
                        "NETS" => in_nets = false,
                        _ => {}
                    }
                }
            }
            "-" => {
                if in_components {
                    let name = parts[1].to_string();
                    let lib_name = parts[2].to_string();
                    let mut x = 0.0;
                    let mut y = 0.0;
                    let mut is_fixed = false;

                    for (i, &part) in parts.iter().enumerate() {
                        if part == "("
                            && i + 2 < parts.len()
                            && let (Ok(px), Ok(py)) =
                                (parts[i + 1].parse::<f64>(), parts[i + 2].parse::<f64>())
                        {
                            x = px / def_units;
                            y = py / def_units;
                        }
                        if part == "FIXED" {
                            is_fixed = true;
                        }
                    }

                    let (width, height) = db.macro_sizes.get(&lib_name).copied().unwrap_or((1.0, 1.0));

                    let id = db.add_cell(name, lib_name, width, height, is_fixed);
                    db.positions[id.index()] = Point::new(x, y);
                } else if in_pins {
                    let pin_name = parts[1].to_string();
                    let mut net_name = String::new();
                    let mut x = 0.0;
                    let mut y = 0.0;

                    let mut i = 2;
                    while i < parts.len() {
                        if parts[i] == "NET" && i + 1 < parts.len() {
                            net_name = parts[i + 1].to_string();
                        } else if (parts[i] == "PLACED" || parts[i] == "FIXED")
                            && i + 3 < parts.len()
                            && parts[i + 1] == "("
                            && let (Ok(px), Ok(py)) = (
                                parts[i + 2].parse::<f64>(),
                                parts[i + 3].parse::<f64>(),
                            )
                        {
                            x = px / def_units;
                            y = py / def_units;
                        }
                        i += 1;
                    }

                    if !net_name.is_empty() {
                        let io_cell = db.get_or_create_io_cell();
                        let net_id = db.add_net(net_name);
                        db.add_pin(io_cell, net_id, Point::new(x, y), pin_name);
                    }
                } else if in_nets {
                    let net_name = parts[1].to_string();
                    let current_net_id = db.add_net(net_name);

                    let mut i = 2;
                    while i < parts.len() {
                        if parts[i] == "(" && i + 2 < parts.len() {
                            let node = parts[i + 1];

                            if node != "PIN"
                                && let Some(&cell_id) = db.cell_name_map.get(node)
                            {
                                let pin_name = parts[i + 2];
                                let lib_name = &db.cells[cell_id.index()].lib_name;

                                let offset = if let Some(pins) = db.macro_pins.get(lib_name)
                                    && let Some(&p) = pins.get(pin_name)
                                {
                                    p
                                } else {
                                    let cx = db.cells[cell_id.index()].width / 2.0;
                                    let cy = db.cells[cell_id.index()].height / 2.0;
                                    let h =
                                        pin_name.chars().fold(0, |acc, c| acc + c as usize);
                                    let dx = ((h % 7) as f64).mul_add(0.25, -0.75);
                                    let dy = (((h / 7) % 7) as f64).mul_add(0.25, -0.75);
                                    Point::new(cx + dx, cy + dy)
                                };

                                db.add_pin(
                                    cell_id,
                                    current_net_id,
                                    offset,
                                    pin_name.to_string(),
                                );
                            }
                        }
                        i += 1;
                    }
                }
            }
            _ => {}
        }
    }
    Ok(())
}
