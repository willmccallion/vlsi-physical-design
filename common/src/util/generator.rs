use rand::Rng;
use std::fs::File;
use std::io::Write;

pub fn generate_random_def(
    filename: &str,
    num_cells: usize,
    num_nets: usize,
) -> std::io::Result<()> {
    let mut file = File::create(filename)?;
    let mut rng = rand::thread_rng();

    let cell_w = 1500; // 1.5um approx for NAND2_X1
    let cell_h = 2000; // Standard row height
    let cell_area = cell_w * cell_h;
    let target_utilization = 0.40;
    let total_cell_area = (num_cells as f64) * (cell_area as f64);
    let required_die_area = total_cell_area / target_utilization;
    let die_side = required_die_area.sqrt() as i32;

    let die_h = ((die_side / cell_h) * cell_h).max(cell_h * 10);
    let die_w = die_side.max(20000);

    log::info!(
        "Generating Benchmark: {} cells, {} nets, Die: {}x{}",
        num_cells,
        num_nets,
        die_w,
        die_h
    );

    writeln!(file, "VERSION 5.8 ;")?;
    writeln!(file, "DIVIDERCHAR \"/\" ;")?;
    writeln!(file, "BUSBITCHARS \"[]\" ;")?;
    writeln!(file, "DESIGN chain_demo ;")?;
    writeln!(file, "UNITS DISTANCE MICRONS 1000 ;")?;
    writeln!(file, "DIEAREA ( 0 0 ) ( {} {} ) ;", die_w, die_h)?;

    writeln!(file, "TRACKS X 0 DO {} STEP 500 LAYER M2 ;", die_w / 500)?;
    writeln!(file, "TRACKS Y 0 DO {} STEP 500 LAYER M1 ;", die_h / 500)?;

    writeln!(file, "COMPONENTS {} ;", num_cells)?;
    for i in 0..num_cells {
        let x = rng.gen_range(0..die_w.saturating_sub(cell_w));
        let y = rng.gen_range(0..die_h.saturating_sub(cell_h));
        writeln!(file, "- inst{} NAND2_X1 + PLACED ( {} {} ) N ;", i, x, y)?;
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
