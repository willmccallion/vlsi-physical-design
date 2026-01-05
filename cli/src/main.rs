use clap::{Parser, Subcommand};
use eda_common::db::core::NetlistDB;
use eda_common::geom::point::Point;
use eda_common::util::config::Config;
use eda_common::util::{check, generator, logger, visualization};
use eda_placer::physics::PhysicsContext;
use eda_placer::solver::nesterov::{NesterovOptimizer, NesterovParams};
use std::path::{Path, PathBuf};

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long, value_name = "FILE", default_value = "config.toml")]
    config: PathBuf,

    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand)]
enum Commands {
    Place,
    Route,
    Flow,
    Generate {
        #[arg(long, default_value_t = 1000)]
        cells: usize,
        #[arg(long, default_value_t = 1000)]
        nets: usize,
    },
}

fn main() -> anyhow::Result<()> {
    logger::init();
    let args = Args::parse();

    let config = if args.config.exists() {
        log::info!("Loading configuration from {:?}", args.config);
        let config_str = std::fs::read_to_string(&args.config)
            .map_err(|e| anyhow::anyhow!("Failed to read config file: {}", e))?;
        toml::from_str(&config_str)
            .map_err(|e| anyhow::anyhow!("Failed to parse config TOML: {}", e))?
    } else {
        log::warn!(
            "Configuration file {:?} not found. Using internal defaults.",
            args.config
        );
        Config::default()
    };

    let command = args.command.unwrap_or(Commands::Flow);

    match command {
        Commands::Generate { cells, nets } => {
            if let Some(parent) = Path::new("inputs/random.def").parent() {
                std::fs::create_dir_all(parent)?;
            }
            log::info!("Generating random benchmark...");
            generator::generate_random_def("inputs/random.def", cells, nets)?;
        }
        Commands::Place => {
            validate_input_paths(&config)?;
            prepare_output_dir(&config.input.output_def)?;

            if run_placement(&config).is_err() {
                std::process::exit(1);
            }
        }
        Commands::Route => {
            validate_lef_paths(&config)?;
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
        Commands::Flow => {
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

    Ok(())
}

fn validate_lef_paths(config: &Config) -> anyhow::Result<()> {
    for lef in &config.input.lef_files {
        if !Path::new(lef).exists() {
            return Err(anyhow::anyhow!("Input LEF file missing: {}", lef));
        }
    }
    Ok(())
}

fn validate_input_paths(config: &Config) -> anyhow::Result<()> {
    validate_lef_paths(config)?;
    if !Path::new(&config.input.def_file).exists() {
        return Err(anyhow::anyhow!(
            "Input DEF file missing: {}",
            config.input.def_file
        ));
    }
    Ok(())
}

fn prepare_output_dir(path_str: &str) -> anyhow::Result<()> {
    if let Some(parent) = Path::new(path_str).parent()
        && !parent.exists()
    {
        log::info!("Creating output directory: {:?}", parent);
        std::fs::create_dir_all(parent)?;
    }
    Ok(())
}

fn place_io_pins(db: &mut NetlistDB) {
    log::info!("Running IO Placement...");

    let io_cell_id = match db.cell_name_map.get("IO_VIRTUAL_CELL") {
        Some(&id) => id,
        None => {
            log::warn!("No IO_VIRTUAL_CELL found. Skipping IO placement.");
            return;
        }
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
            // Left edge (moving up)
            x = 0.0;
            y = current_dist;
        } else if current_dist < die_h + die_w {
            // Top edge (moving right)
            x = current_dist - die_h;
            y = die_h;
        } else if current_dist < 2.0 * die_h + die_w {
            // Right edge (moving down)
            x = die_w;
            y = die_h - (current_dist - (die_h + die_w));
        } else {
            // Bottom edge (moving left)
            x = die_w - (current_dist - (2.0 * die_h + die_w));
            y = 0.0;
        }

        let safe_x = x.max(0.0).min(die_w);
        let safe_y = y.max(0.0).min(die_h);

        // Update the pin offset. Since IO cell is at (0,0), offset == absolute position.
        db.pin_offsets[pin_id.index()] = Point::new(safe_x, safe_y);

        current_dist += step;
    }
    log::info!("Placed {} IO pins around the perimeter.", num_pins);
}

fn run_placement(config: &Config) -> anyhow::Result<()> {
    let mut db = NetlistDB::new();

    if let Some(lef_path) = config.input.lef_files.first() {
        log::info!("Parsing LEF: {}", lef_path);
        eda_common::db::parser::lef::parse(&mut db, lef_path)
            .map_err(|e| anyhow::anyhow!("Invalid LEF syntax in '{}': {}", lef_path, e))?;
    }

    log::info!("Parsing DEF: {}", config.input.def_file);
    eda_common::db::parser::def::parse(&mut db, &config.input.def_file)
        .map_err(|e| anyhow::anyhow!("Invalid DEF syntax in '{}': {}", config.input.def_file, e))?;

    place_io_pins(&mut db);

    let total_cell_area: f64 = db
        .cells
        .iter()
        .filter(|c| !c.is_fixed)
        .map(|c| c.width * c.height)
        .sum();
    let die_area = db.die_area.width() * db.die_area.height();
    let utilization = total_cell_area / die_area;

    log::info!("Design Utilization: {:.2}%", utilization * 100.0);

    let target_density = config.global_placement.target_density;
    let electro_force = config.global_placement.electro_force_multiplier;

    log::info!("Starting Global Placement...");
    let mut physics = PhysicsContext::new(
        config.global_placement.bin_dimension,
        config.global_placement.bin_dimension,
    );

    let params = NesterovParams {
        max_iterations: config.global_placement.placer_max_iterations,
        initial_learning_rate: config.global_placement.initial_learning_rate,
        convergence_threshold: config.global_placement.convergence_threshold,
        wa_gamma: config.global_placement.wa_gamma,
        target_density,
        electro_force_multiplier: electro_force,
    };
    let mut solver = NesterovOptimizer::new(params, db.num_cells());

    solver
        .optimize(&mut db, &mut physics)
        .map_err(|e| anyhow::anyhow!(e))?;

    log::info!("Starting Legalization...");
    let legalizer = eda_placer::legalize::abacus::AbacusLegalizer::new();
    legalizer.legalize(&mut db);

    if let Err(e) = check::run_placement_check(&db) {
        return Err(anyhow::anyhow!(e));
    }

    log::info!("Generating placement visualization...");
    visualization::draw_placement(&db, "output/placed.png", 1000, 1000);

    log::info!("Writing placed DEF to {}", config.input.output_def);
    save_def(&db, &config.input.output_def)?;

    Ok(())
}

fn run_routing(config: &Config) -> anyhow::Result<()> {
    let mut db = NetlistDB::new();

    if let Some(lef_path) = config.input.lef_files.first() {
        log::info!("Parsing LEF: {}", lef_path);
        eda_common::db::parser::lef::parse(&mut db, lef_path)
            .map_err(|e| anyhow::anyhow!("Invalid LEF syntax in '{}': {}", lef_path, e))?;
    }

    let input_def = &config.input.output_def;
    log::info!("Parsing Placed DEF: {}", input_def);
    eda_common::db::parser::def::parse(&mut db, input_def)
        .map_err(|e| anyhow::anyhow!("Invalid Placed DEF syntax in '{}': {}", input_def, e))?;

    log::info!("Starting Routing...");

    eda_router::route(&mut db, config).map_err(|e| anyhow::anyhow!(e))?;

    log::info!("Generating routed visualization...");
    visualization::draw_routed_design(&db, "output/routed.png", 2000, 2000);

    check::run(&db).map_err(|e| anyhow::anyhow!("Verification Failed: {}", e))?;

    let input_path = Path::new(&config.input.output_def);
    let parent = input_path.parent().unwrap_or(Path::new("."));
    let routed_filename = "routed.def";
    let output_path = parent.join(routed_filename);

    log::info!("Writing routed DEF to {:?}", output_path);
    save_def(&db, output_path.to_str().unwrap())?;

    Ok(())
}

fn save_def(db: &NetlistDB, filename: &str) -> std::io::Result<()> {
    use std::io::Write;
    let mut file = std::fs::File::create(filename)?;

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
