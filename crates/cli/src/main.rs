//! PARE -- Placement And Routing Engine.
//!
//! This module provides the entry point for PARE, orchestrating
//! the placement and routing workflows. It handles configuration loading,
//! command parsing, input validation, and coordinates the execution of
//! placement and routing algorithms through the common database interface.

use clap::{Parser, Subcommand};
use pare_common::db::core::NetlistDB;
use pare_common::geom::point::Point;
use pare_common::util::config::Config;
use pare_common::util::{check, logger, ui, visualization};
use pare_placer::physics::PhysicsContext;
use pare_placer::solver::nesterov::{NesterovOptimizer, NesterovParams};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter};
use std::path::{Path, PathBuf};
use std::time::Instant;

#[derive(Parser)]
#[command(author, version, about = "PARE -- Placement And Routing Engine", long_about = None)]
struct Args {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run placement only.
    Place {
        /// TOML configuration file.
        config: PathBuf,
    },
    /// Run routing only (requires a prior placement run).
    Route {
        /// TOML configuration file.
        config: PathBuf,
    },
    /// Run the full placement + routing flow.
    Flow {
        /// TOML configuration file.
        config: PathBuf,
    },
}

fn load_config(path: &Path) -> anyhow::Result<Config> {
    if !path.exists() {
        return Err(anyhow::anyhow!("Config file not found: {:?}", path));
    }
    ui::header("Config", &path.display().to_string());
    let config_str = std::fs::read_to_string(path)
        .map_err(|e| anyhow::anyhow!("Failed to read config file: {}", e))?;
    toml::from_str(&config_str)
        .map_err(|e| anyhow::anyhow!("Failed to parse config TOML: {}", e))
}

fn main() -> anyhow::Result<()> {
    logger::init();
    let args = Args::parse();
    let start_time = Instant::now();

    ui::banner();

    match args.command {
        Commands::Place { ref config } => {
            let config = load_config(config)?;
            validate_input_paths(&config)?;
            prepare_output_dir(&config.input.output_def)?;

            if run_placement(&config).is_err() {
                std::process::exit(1);
            }
        }
        Commands::Route { ref config } => {
            let config = load_config(config)?;
            if config.input.bookshelf_aux_file.is_none() {
                validate_lef_paths(&config)?;
            }
            if !Path::new(&config.input.output_def).exists() {
                return Err(anyhow::anyhow!(
                    "Placed DEF file missing: '{}'. Did you run 'place'?",
                    config.input.output_def
                ));
            }

            if run_routing(&config).is_err() {
                std::process::exit(1);
            }
        }
        Commands::Flow { ref config } => {
            let config = load_config(config)?;
            validate_input_paths(&config)?;
            prepare_output_dir(&config.input.output_def)?;

            if run_placement(&config).is_err() {
                std::process::exit(1);
            }

            if run_routing(&config).is_err() {
                std::process::exit(1);
            }
        }
    }

    ui::done(start_time.elapsed().as_secs_f64());
    Ok(())
}

/// Validates that all LEF library files specified in the configuration exist.
fn validate_lef_paths(config: &Config) -> anyhow::Result<()> {
    if config.input.bookshelf_aux_file.is_some() {
        return Ok(());
    }
    for lef in &config.input.lef_files {
        if !Path::new(lef).exists() {
            return Err(anyhow::anyhow!("Input LEF file missing: {}", lef));
        }
    }
    Ok(())
}

/// Validates that all required input files for placement exist.
fn validate_input_paths(config: &Config) -> anyhow::Result<()> {
    if let Some(aux) = &config.input.bookshelf_aux_file {
        if !Path::new(aux).exists() {
            return Err(anyhow::anyhow!("Input AUX file missing: {}", aux));
        }
        return Ok(());
    }

    validate_lef_paths(config)?;
    if !Path::new(&config.input.def_file).exists() {
        return Err(anyhow::anyhow!(
            "Input DEF file missing: {}",
            config.input.def_file
        ));
    }
    Ok(())
}

/// Creates the parent directory for an output file path if it does not exist.
fn prepare_output_dir(path_str: &str) -> anyhow::Result<()> {
    if let Some(parent) = Path::new(path_str).parent()
        && !parent.exists()
        && !parent.as_os_str().is_empty()
    {
        log::debug!("Creating output directory: {:?}", parent);
        std::fs::create_dir_all(parent)?;
    }
    Ok(())
}

/// Distributes I/O pins uniformly around the die perimeter.
fn place_io_pins(db: &mut NetlistDB) {
    let io_cell_id = match db.cell_name_map.get("IO_VIRTUAL_CELL") {
        Some(&id) => id,
        None => return,
    };

    let die_w = db.die_area.width();
    let die_h = db.die_area.height();

    let pins = db.cells[io_cell_id.index()].pins.clone();
    let num_pins = pins.len();

    if num_pins == 0 {
        return;
    }

    let perimeter = 2.0 * (die_w + die_h);
    let step = perimeter / (num_pins as f64);

    let mut current_dist = 0.0;

    for pin_id in pins {
        let x;
        let y;

        if current_dist < die_h {
            x = 0.0;
            y = current_dist;
        } else if current_dist < die_h + die_w {
            x = current_dist - die_h;
            y = die_h;
        } else if current_dist < 2.0 * die_h + die_w {
            x = die_w;
            y = die_h - (current_dist - (die_h + die_w));
        } else {
            x = die_w - (current_dist - (2.0 * die_h + die_w));
            y = 0.0;
        }

        let safe_x = x.max(0.0).min(die_w);
        let safe_y = y.max(0.0).min(die_h);

        db.pin_offsets[pin_id.index()] = Point::new(safe_x, safe_y);

        current_dist += step;
    }
    log::debug!("Placed {} IO pins around the perimeter.", num_pins);
}

/// Executes the complete placement workflow from netlist parsing to legalization.
fn run_placement(config: &Config) -> anyhow::Result<()> {
    let mut db = NetlistDB::new();

    if let Some(aux_path) = &config.input.bookshelf_aux_file {
        ui::header("Input", aux_path);
        pare_common::db::parser::bookshelf::parse(&mut db, aux_path)
            .map_err(|e| anyhow::anyhow!("Invalid Bookshelf syntax in '{}': {}", aux_path, e))?;
    } else {
        if let Some(lef_path) = config.input.lef_files.first() {
            ui::header("Input", lef_path);
            pare_common::db::parser::lef::parse(&mut db, lef_path)
                .map_err(|e| anyhow::anyhow!("Invalid LEF syntax in '{}': {}", lef_path, e))?;
        }

        ui::header("Input", &config.input.def_file);
        pare_common::db::parser::def::parse(&mut db, &config.input.def_file).map_err(|e| {
            anyhow::anyhow!("Invalid DEF syntax in '{}': {}", config.input.def_file, e)
        })?;
    }

    ui::header("Output", &config.input.output_def);

    place_io_pins(&mut db);

    let movable_cells = db.cells.iter().filter(|c| !c.is_fixed).count();
    let num_nets = db.num_nets();
    let total_cell_area: f64 = db
        .cells
        .iter()
        .filter(|c| !c.is_fixed)
        .map(|c| c.width * c.height)
        .sum();
    let die_area = db.die_area.width() * db.die_area.height();
    let utilization = total_cell_area / die_area;

    // Auto-detect placement parameters from design properties
    let die_diag = (db.die_area.width().powi(2) + db.die_area.height().powi(2)).sqrt();

    let wa_gamma = config.global_placement.wa_gamma.unwrap_or_else(|| {
        die_diag * 1.5
    });

    let bin_dimension = config.global_placement.bin_dimension.unwrap_or_else(|| {
        let movable = db.cells.iter().filter(|c| !c.is_fixed).count();
        ((movable as f64).sqrt().ceil() as usize).next_power_of_two().clamp(64, 512)
    });

    let target_density = config.global_placement.target_density.unwrap_or_else(|| {
        (utilization + 0.05).max(0.80)
    });

    let initial_learning_rate =
        config.global_placement.initial_learning_rate.unwrap_or_else(|| {
            let mut heights: Vec<f64> = db
                .cells
                .iter()
                .filter(|c| !c.is_fixed)
                .map(|c| c.height)
                .collect();
            heights.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let median_height = if heights.is_empty() {
                1.0
            } else {
                heights[heights.len() / 2]
            };
            (median_height * 0.0006).clamp(0.005, 1.0)
        });

    let electro_force = config.global_placement.electro_force_multiplier;

    // --- Placement section ---
    let phase_start = Instant::now();
    ui::section("Placement");
    ui::stat_row(
        "Cells:", &format!("{}", movable_cells),
        "Nets:", &format!("{}", num_nets),
    );
    ui::stat_row(
        "Utilization:", &format!("{:.2}%", utilization * 100.0),
        "Target Density:", &format!("{:.2}", target_density),
    );
    ui::stat_row(
        "Bin Grid:", &format!("{}x{}", bin_dimension, bin_dimension),
        "WA Gamma:", &format!("{:.1}", wa_gamma),
    );

    let mut physics = PhysicsContext::new(bin_dimension, bin_dimension);

    let params = NesterovParams {
        max_iterations: config.global_placement.placer_max_iterations,
        initial_learning_rate,
        convergence_threshold: config.global_placement.convergence_threshold,
        wa_gamma,
        target_density,
        electro_force_multiplier: electro_force,
    };
    let mut solver = NesterovOptimizer::new(params, db.num_cells());

    ui::placement_table_header();

    let result = solver
        .optimize(&mut db, &mut physics)
        .map_err(|e| anyhow::anyhow!(e))?;

    eprintln!();
    if result.converged {
        ui::check(&format!(
            "Converged at iter {} (overflow={:.4}, WL={:.0})",
            result.iterations, result.final_overflow, result.final_wirelength
        ));
    } else {
        log::warn!(
            "Did not converge after {} iterations (overflow={:.4})",
            result.iterations, result.final_overflow
        );
    }

    ui::phase_time(phase_start.elapsed().as_secs_f64());

    let output_dir = Path::new(&config.input.output_def)
        .parent()
        .unwrap_or(Path::new("."));

    // --- Legalization ---
    let phase_start = Instant::now();
    ui::phase("Legalization");
    let legalizer = pare_placer::legalize::abacus::AbacusLegalizer::new();
    legalizer.legalize(&mut db);
    ui::check(&format!("Legalized {} cells", movable_cells));

    ui::phase_time(phase_start.elapsed().as_secs_f64());

    // --- Placement Verification ---
    if let Err(e) = check::run_placement_check(&db) {
        return Err(anyhow::anyhow!(e));
    }

    // --- Output ---
    let phase_start = Instant::now();
    ui::phase("Output");
    let nesterov_png = output_dir.join("nesterov_placer.png");
    visualization::draw_placement(&db, nesterov_png.to_str().unwrap(), 1000, 1000);
    ui::wrote(nesterov_png.to_str().unwrap());

    let placed_png = output_dir.join("placed.png");
    visualization::draw_placement(&db, placed_png.to_str().unwrap(), 1000, 1000);
    ui::wrote(placed_png.to_str().unwrap());

    save_def(&db, &config.input.output_def)?;
    ui::wrote(&config.input.output_def);
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    Ok(())
}

/// Preloads cell geometry information from a Bookshelf nodes file.
fn preload_bookshelf_geometry(db: &mut NetlistDB, aux_path: &str) -> anyhow::Result<()> {
    log::debug!("Preloading Bookshelf Geometry from AUX: {}", aux_path);
    let path = Path::new(aux_path);
    let parent = path.parent().unwrap_or(Path::new("."));
    let file = File::open(path)?;
    let reader = BufReader::new(file);

    let mut nodes_file = String::new();

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
                }
            }
        }
    }

    if nodes_file.is_empty() {
        return Ok(());
    }

    let nodes_path = parent.join(&nodes_file);
    log::debug!("Reading Nodes: {:?}", nodes_path);
    let nfile = File::open(nodes_path)?;
    let nreader = BufReader::new(nfile);

    for line in nreader.lines() {
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

        let width: f64 = parts[1].parse().unwrap_or(1.0);
        let height: f64 = parts[2].parse().unwrap_or(1.0);

        let lib_name = format!("BLK_{}_{}", width as i32, height as i32);

        db.macro_sizes.insert(lib_name, (width, height));
    }
    Ok(())
}

/// Executes the complete routing workflow from placed netlist to routed design.
fn run_routing(config: &Config) -> anyhow::Result<()> {
    let mut db = NetlistDB::new();

    let is_bookshelf = config.input.bookshelf_aux_file.is_some();

    if let Some(aux) = &config.input.bookshelf_aux_file {
        preload_bookshelf_geometry(&mut db, aux)?;
    }

    if !is_bookshelf
        && let Some(lef_path) = config.input.lef_files.first()
        && Path::new(lef_path).exists()
    {
        log::debug!("Parsing LEF: {}", lef_path);
        pare_common::db::parser::lef::parse(&mut db, lef_path)
            .map_err(|e| anyhow::anyhow!("Invalid LEF syntax in '{}': {}", lef_path, e))?;
    }

    let input_def = &config.input.output_def;
    log::debug!("Parsing Placed DEF: {}", input_def);
    pare_common::db::parser::def::parse(&mut db, input_def)
        .map_err(|e| anyhow::anyhow!("Invalid Placed DEF syntax in '{}': {}", input_def, e))?;

    if is_bookshelf || db.layers.is_empty() {
        log::debug!("Bookshelf/No-LEF: Calculating routing pitch from cell geometry...");

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

        let pitch = std_height / 16.0;
        let width = pitch * 0.5;

        log::warn!(
            "Layer data missing! Synthesizing default layers (Cell Height={:.2}, Pitch={:.2}).",
            std_height,
            pitch
        );

        db.layers.clear();
        db.layer_name_map.clear();

        use pare_common::db::core::LayerDirection;
        db.add_layer("M1".to_string(), LayerDirection::Horizontal, pitch, width);
        db.add_layer("M2".to_string(), LayerDirection::Vertical, pitch, width);
        db.add_layer("M3".to_string(), LayerDirection::Horizontal, pitch, width);
        db.add_layer("M4".to_string(), LayerDirection::Vertical, pitch, width);
        db.add_layer("M5".to_string(), LayerDirection::Horizontal, pitch, width);
        db.add_layer("M6".to_string(), LayerDirection::Vertical, pitch, width);
    }

    if db.layers.is_empty() {
        return Err(anyhow::anyhow!("No layers defined! Cannot route."));
    }

    let phase_start = Instant::now();
    ui::section("Routing");
    ui::stat_row(
        "Nets:", &format!("{}", db.num_nets()),
        "Layers:", &format!("{}", db.layers.len()),
    );

    pare_router::route(&mut db, config).map_err(|e| anyhow::anyhow!(e))?;
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    // --- Verification ---
    let phase_start = Instant::now();
    ui::phase("Verification");
    check::run(&db).map_err(|e| anyhow::anyhow!("Verification Failed: {}", e))?;
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    // --- Output ---
    let phase_start = Instant::now();
    ui::phase("Output");
    let output_dir = Path::new(&config.input.output_def)
        .parent()
        .unwrap_or(Path::new("."));

    let routed_png = output_dir.join("routed.png");
    visualization::draw_routed_design(&db, routed_png.to_str().unwrap(), 2000, 2000);
    ui::wrote(routed_png.to_str().unwrap());

    let input_path = Path::new(&config.input.output_def);
    let parent = input_path.parent().unwrap_or(Path::new("."));
    let routed_filename = "routed.def";
    let output_path = parent.join(routed_filename);

    save_def(&db, output_path.to_str().unwrap())?;
    ui::wrote(output_path.to_str().unwrap());
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    Ok(())
}

/// Writes the netlist database to a DEF file in standard format.
fn save_def(db: &NetlistDB, filename: &str) -> std::io::Result<()> {
    use std::io::Write;
    let mut file = BufWriter::with_capacity(256 * 1024, std::fs::File::create(filename)?);

    writeln!(file, "VERSION 5.8 ;")?;
    writeln!(file, "DIVIDERCHAR \"/\" ;")?;
    writeln!(file, "BUSBITCHARS \"[]\" ;")?;
    writeln!(file, "DESIGN demo ;")?;
    writeln!(file, "UNITS DISTANCE MICRONS 1000 ;")?;

    let x1 = (db.die_area.min.x * 1000.0) as i32;
    let y1 = (db.die_area.min.y * 1000.0) as i32;
    let x2 = (db.die_area.max.x * 1000.0) as i32;
    let y2 = (db.die_area.max.y * 1000.0) as i32;
    writeln!(file, "DIEAREA ( {} {} ) ( {} {} ) ;", x1, y1, x2, y2)?;

    let real_cells: Vec<usize> = (0..db.num_cells())
        .filter(|&i| db.cells[i].name != "IO_VIRTUAL_CELL")
        .collect();

    writeln!(file, "COMPONENTS {} ;", real_cells.len())?;
    for &i in &real_cells {
        let pos = db.positions[i];
        let cell = &db.cells[i];
        writeln!(
            file,
            "- {} {} + PLACED ( {} {} ) N ;",
            cell.name,
            cell.lib_name,
            (pos.x * 1000.0) as i32,
            (pos.y * 1000.0) as i32
        )?;
    }
    writeln!(file, "END COMPONENTS")?;

    let mut io_pins = Vec::new();
    if let Some(&io_cell_id) = db.cell_name_map.get("IO_VIRTUAL_CELL") {
        let io_cell = &db.cells[io_cell_id.index()];
        for &pin_id in &io_cell.pins {
            io_pins.push(pin_id);
        }
    }

    if !io_pins.is_empty() {
        writeln!(file, "PINS {} ;", io_pins.len())?;
        for &pin_id in &io_pins {
            let pin_name = &db.pin_names[pin_id.index()];
            let net_id = db.pin_to_net[pin_id.index()];
            let net_name = &db.nets[net_id.index()].name;
            let pos = db.pin_offsets[pin_id.index()];

            writeln!(
                file,
                "- {} + NET {} + DIRECTION INOUT + USE SIGNAL + LAYER M1 ( 0 0 ) ( 100 100 ) + PLACED ( {} {} ) N ;",
                pin_name,
                net_name,
                (pos.x * 1000.0) as i32,
                (pos.y * 1000.0) as i32
            )?;
        }
        writeln!(file, "END PINS")?;
    }

    writeln!(file, "NETS {} ;", db.num_nets())?;
    for net in &db.nets {
        write!(file, "- {} ", net.name)?;
        for &pin_id in &net.pins {
            let cell_id = db.pin_to_cell[pin_id.index()];
            let cell_name = &db.cells[cell_id.index()].name;
            let pin_name = &db.pin_names[pin_id.index()];

            if cell_name == "IO_VIRTUAL_CELL" {
                write!(file, "( PIN {} ) ", pin_name)?;
            } else {
                write!(file, "( {} {} ) ", cell_name, pin_name)?;
            }
        }
        writeln!(file)?;

        for seg in &net.route_segments {
            let x1 = (seg.p1.x * 1000.0).round() as i32;
            let y1 = (seg.p1.y * 1000.0).round() as i32;
            let x2 = (seg.p2.x * 1000.0).round() as i32;
            let y2 = (seg.p2.y * 1000.0).round() as i32;

            if (x1 - x2).abs() < 2 && (y1 - y2).abs() < 2 {
                let layer_name = &db.layers[seg.layer as usize].name;
                let next_layer_idx = (seg.layer as usize + 1).min(db.layers.len() - 1);
                let next_layer_name = &db.layers[next_layer_idx].name;
                let via_name = format!("VIA_{}_{}", layer_name, next_layer_name);
                writeln!(
                    file,
                    "  + ROUTED {} ( {} {} ) {} ",
                    layer_name, x1, y1, via_name
                )?;
            } else {
                let layer_name = &db.layers[seg.layer as usize].name;
                writeln!(
                    file,
                    "  + ROUTED {} ( {} {} ) ( {} {} )",
                    layer_name, x1, y1, x2, y2
                )?;
            }
        }
        writeln!(file, "  ;")?;
    }
    writeln!(file, "END NETS")?;
    writeln!(file, "END DESIGN")?;
    Ok(())
}
