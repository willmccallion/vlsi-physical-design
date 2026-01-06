use rand::Rng;
use std::fs::File;
use std::io::Write;

pub fn generate_random_def(
    filename: &str,
    num_cells: usize,
    num_nets: usize,
    target_utilization: f64,
) -> std::io::Result<()> {
    let mut file = File::create(filename)?;
    let mut rng = rand::thread_rng();

    let cell_w = 500;
    let cell_h = 2000;

    let cell_area = cell_w * cell_h;
    let total_cell_area = (num_cells as f64) * (cell_area as f64);

    let util = target_utilization.clamp(0.01, 0.99);
    let required_die_area = total_cell_area / util;
    let die_side = required_die_area.sqrt() as i32;

    let die_h = ((die_side / cell_h) * cell_h).max(cell_h * 10);
    let die_w = die_side.max(20000);

    log::info!(
        "Generating Benchmark: {} cells, {} nets, Die: {}x{} (Target Util: {:.1}%)",
        num_cells,
        num_nets,
        die_w,
        die_h,
        util * 100.0
    );

    writeln!(file, "VERSION 5.8 ;")?;
    writeln!(file, "DIVIDERCHAR \"/\" ;")?;
    writeln!(file, "BUSBITCHARS \"[]\" ;")?;
    writeln!(file, "DESIGN chain_demo ;")?;
    writeln!(file, "UNITS DISTANCE MICRONS 1000 ;")?;
    writeln!(file, "DIEAREA ( 0 0 ) ( {} {} ) ;", die_w, die_h)?;

    // Define Tracks
    writeln!(file, "TRACKS X 0 DO {} STEP 500 LAYER M2 ;", die_w / 500)?;
    writeln!(file, "TRACKS Y 0 DO {} STEP 500 LAYER M1 ;", die_h / 500)?;

    writeln!(file, "COMPONENTS {} ;", num_cells)?;
    for i in 0..num_cells {
        let max_x = die_w.saturating_sub(cell_w);
        let max_y = die_h.saturating_sub(cell_h);

        let x = if max_x > 0 {
            rng.gen_range(0..max_x)
        } else {
            0
        };
        let y = if max_y > 0 {
            rng.gen_range(0..max_y)
        } else {
            0
        };

        let y_snapped = (y / cell_h) * cell_h;

        writeln!(
            file,
            "- inst{} NAND2_X1 + PLACED ( {} {} ) N ;",
            i, x, y_snapped
        )?;
    }
    writeln!(file, "END COMPONENTS")?;

    let num_io = 2;
    writeln!(file, "PINS {} ;", num_io)?;
    writeln!(
        file,
        "- IN_PIN + NET net0 + DIRECTION INPUT + USE SIGNAL + LAYER M1 ( 0 0 ) ( 100 100 ) + PLACED ( 0 {} ) N ;",
        die_h / 2
    )?;
    writeln!(
        file,
        "- OUT_PIN + NET net{} + DIRECTION OUTPUT + USE SIGNAL + LAYER M1 ( 0 0 ) ( 100 100 ) + PLACED ( {} {} ) N ;",
        num_nets - 1,
        die_w,
        die_h / 2
    )?;
    writeln!(file, "END PINS")?;

    writeln!(file, "NETS {} ;", num_nets)?;

    for i in 0..num_nets {
        write!(file, "- net{} ", i)?;

        if i == 0 {
            write!(file, "( IN_PIN ) ")?;
        } else {
            let src_inst = (i - 1) % num_cells;
            write!(file, "( inst{} ZN ) ", src_inst)?;
        }

        if i == num_nets - 1 {
            write!(file, "( OUT_PIN ) ")?;
        } else {
            let sink_inst = i % num_cells;
            write!(file, "( inst{} A1 ) ", sink_inst)?;
        }

        writeln!(file, ";")?;
    }

    writeln!(file, "END NETS")?;
    writeln!(file, "END DESIGN")?;
    Ok(())
}
