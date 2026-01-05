use crate::db::core::{LayerDirection, NetlistDB};
use crate::geom::point::Point;
use anyhow::Result;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};

pub fn parse(db: &mut NetlistDB, filename: &str) -> Result<()> {
    let file = File::open(filename)?;
    let reader = BufReader::new(file);

    let mut current_macro = String::new();
    let mut current_layer = String::new();
    let mut current_pin = String::new();

    let mut in_layer = false;
    let mut in_macro = false;
    let mut in_pin = false;
    let mut layer_type = String::new();
    let mut layer_dir = LayerDirection::Unknown;
    let mut layer_pitch = 0.0;
    let mut layer_width = 0.0;

    for line in reader.lines() {
        let line = line?;
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            "LAYER" => {
                if !in_macro {
                    current_layer = parts[1].to_string();
                    in_layer = true;
                    layer_type = String::new();
                    layer_dir = LayerDirection::Unknown;
                    layer_pitch = 1.0;
                    layer_width = 1.0;
                }
            }
            "TYPE" => {
                if in_layer {
                    layer_type = parts[1].to_string();
                }
            }
            "DIRECTION" => {
                if in_layer {
                    layer_dir = match parts[1] {
                        "VERTICAL" => LayerDirection::Vertical,
                        "HORIZONTAL" => LayerDirection::Horizontal,
                        _ => LayerDirection::Unknown,
                    };
                }
            }
            "PITCH" => {
                if in_layer {
                    layer_pitch = parts[1].parse().unwrap_or(1.0);
                }
            }
            "WIDTH" => {
                if in_layer {
                    layer_width = parts[1].parse().unwrap_or(1.0);
                }
            }
            "END" => {
                if parts.len() > 1 {
                    if parts[1] == current_layer && in_layer {
                        if layer_type == "ROUTING" {
                            db.add_layer(
                                current_layer.clone(),
                                layer_dir.clone(),
                                layer_pitch,
                                layer_width,
                            );
                        }
                        in_layer = false;
                    } else if parts[1] == current_pin && in_pin {
                        in_pin = false;
                        current_pin.clear();
                    } else if parts[1] == current_macro && in_macro {
                        in_macro = false;
                        current_macro.clear();
                    }
                }
            }
            "MACRO" => {
                current_macro = parts[1].to_string();
                in_macro = true;
                db.macro_pins.insert(current_macro.clone(), HashMap::new());
            }
            "PIN" => {
                if in_macro {
                    current_pin = parts[1].to_string();
                    in_pin = true;
                }
            }
            "RECT" => {
                if in_pin
                    && !current_macro.is_empty()
                    && let (Ok(x1), Ok(y1), Ok(x2), Ok(y2)) = (
                        parts[1].parse::<f64>(),
                        parts[2].parse::<f64>(),
                        parts[3].parse::<f64>(),
                        parts[4].parse::<f64>(),
                    )
                {
                    let center_x = (x1 + x2) / 2.0;
                    let center_y = (y1 + y2) / 2.0;

                    if let Some(pins) = db.macro_pins.get_mut(&current_macro)
                        && !pins.contains_key(&current_pin)
                    {
                        pins.insert(current_pin.clone(), Point::new(center_x, center_y));
                    }
                }
            }
            "SIZE" => {
                if in_macro {
                    let w: f64 = parts[1].parse()?;
                    let h: f64 = parts[3].parse()?;
                    db.macro_sizes.insert(current_macro.clone(), (w, h));
                }
            }
            _ => {}
        }
    }

    if db.layers.is_empty() {
        log::warn!("No layers found in LEF. Adding default 6 layers.");
        db.add_layer("M1".to_string(), LayerDirection::Vertical, 1.0, 1.0);
        db.add_layer("M2".to_string(), LayerDirection::Horizontal, 1.0, 1.0);
        db.add_layer("M3".to_string(), LayerDirection::Vertical, 1.0, 1.0);
        db.add_layer("M4".to_string(), LayerDirection::Horizontal, 1.0, 1.0);
        db.add_layer("M5".to_string(), LayerDirection::Vertical, 1.0, 1.0);
        db.add_layer("M6".to_string(), LayerDirection::Horizontal, 1.0, 1.0);
    }

    Ok(())
}
