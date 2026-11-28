use super::*;
use crate::testing::{DefSections, cells_with, def_text, design_with_copies};
use pare_common::db::library::Site;
use pare_common::db::parser::liberty::parse_str;
use std::fmt::Write;
use std::path::Path;

/// A buffer cell `name` of `footprint` on the clock cells' tables whose
/// output slews to `slew` nanoseconds at 100 fF, which sets its drive.
fn buffer_cell(name: &str, footprint: &str, slew: f64) -> String {
    format!(
        "cell ({name}) {{ area : 0.8 ; cell_footprint : \"{footprint}\" ;
               pin (A) {{ direction : input ; capacitance : 1.0 ; }}
               pin (Z) {{ direction : output ; function : \"A\" ;
                 timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
                   cell_rise (delay_t) {{ values (\"0.03, 0.30\", \"0.04, 0.31\") ; }}
                   rise_transition (delay_t) {{
                     values (\"0.0, {slew}\", \"0.0, {slew}\") ; }} }} }} }}\n"
    )
}

/// The names `find_buffers` chooses among buffers `B0`, `B1`, ... of
/// the `(footprint, site)` of `buffers`, each driving more weakly than
/// the last, in a design without instances where they copy `BUF_X1`,
/// which is not to be used.
fn find_among(buffers: &[(&str, &str)]) -> Result<Vec<String>, LinkError> {
    let names: Vec<String> = (0..buffers.len()).map(|i| format!("B{i}")).collect();
    let copies: Vec<(&str, &str)> = names.iter().map(|n| ("BUF_X1", n.as_str())).collect();
    let mut db = design_with_copies(
        &def_text(
            "buffers",
            &DefSections {
                components: "",
                nets: "",
                inputs: &[],
                outputs: &[],
            },
        ),
        &copies,
    );
    let core = db.library.site(db.library.site_id("core").unwrap()).clone();
    for (name, &(_, site)) in names.iter().zip(buffers) {
        let site = db.library.site_id(site).unwrap_or_else(|| {
            db.library
                .add_site(Site {
                    name: site.to_string(),
                    ..core.clone()
                })
                .unwrap()
        });
        let id = db.library.macro_id(name).unwrap();
        db.library.macros[id].site = Some(site);
    }
    let extra: String = names
        .iter()
        .zip(buffers)
        .enumerate()
        .map(|(i, (name, (footprint, _)))| buffer_cell(name, footprint, 0.1 * (i + 1) as f64))
        .collect();
    let lib = cells_with(&extra);
    let link = Link::new(&db, &lib, &["BUF_X1".to_string()]).unwrap();
    let set = link.find_buffers(&db)?;
    assert_eq!(set.lowest_drive, *set.cells.last().unwrap());
    Ok(set
        .cells
        .iter()
        .map(|b| lib.cell(b.id).name.clone())
        .collect())
}

#[test]
fn the_two_strongest_buffers_of_each_of_five_drive_bands_are_chosen() {
    let chosen = find_among(&[("buf", "core"); 15]).unwrap();

    assert_eq!(
        chosen,
        [
            "B0", "B1", "B3", "B4", "B6", "B7", "B9", "B10", "B12", "B13"
        ]
    );
}

#[test]
fn a_footprint_most_buffers_share_leaves_out_the_others() {
    let buffers = [
        ("buf", "core"),
        ("buf", "core"),
        ("dly", "core"),
        ("buf", "core"),
    ];

    let chosen = find_among(&buffers).unwrap();

    assert_eq!(chosen, ["B0", "B1", "B3"]);
}

#[test]
fn footprints_split_evenly_leave_out_none() {
    let buffers = [
        ("buf", "core"),
        ("dly", "core"),
        ("dly", "core"),
        ("buf", "core"),
    ];

    let chosen = find_among(&buffers).unwrap();

    assert_eq!(chosen, ["B0", "B1", "B2", "B3"]);
}

#[test]
fn buffers_on_two_sites_give_each_band_the_strongest_of_each_site() {
    let buffers: Vec<(&str, &str)> = (0..15)
        .map(|i| ("buf", if i % 2 == 0 { "core" } else { "tall" }))
        .collect();

    let chosen = find_among(&buffers).unwrap();

    assert_eq!(
        chosen,
        [
            "B0", "B1", "B4", "B3", "B6", "B7", "B10", "B9", "B12", "B13"
        ]
    );
}

#[test]
fn a_library_of_only_clock_and_unusable_buffers_has_none_to_choose() {
    assert_eq!(find_among(&[]), Err(LinkError::NoBuffers));
}

/// The design of [`find_among`] over buffers `B0`, `B1`, ... copying
/// `BUF_X1`, each with the Liberty attributes in `attributes`.
fn attributed_buffers(attributes: &[&str]) -> (NetlistDB, Liberty) {
    let names: Vec<String> = (0..attributes.len()).map(|i| format!("B{i}")).collect();
    let copies: Vec<(&str, &str)> = names.iter().map(|n| ("BUF_X1", n.as_str())).collect();
    let sections = DefSections {
        components: "",
        nets: "",
        inputs: &[],
        outputs: &[],
    };
    let db = design_with_copies(&def_text("buffers", &sections), &copies);
    let extra: String = names
        .iter()
        .zip(attributes)
        .map(|(name, attribute)| {
            buffer_cell(name, "buf", 0.1).replacen(
                "area : 0.8 ;",
                &format!("area : 0.8 ; {attribute}"),
                1,
            )
        })
        .collect();
    (db, cells_with(&extra))
}

/// The names of `cells` of `lib`.
fn names(lib: &Liberty, cells: &[LibCellId]) -> Vec<String> {
    cells.iter().map(|&c| lib.cell(c).name.clone()).collect()
}

#[test]
fn a_cell_whose_master_is_no_core_cell_has_no_sizes() {
    let (mut db, lib) = attributed_buffers(&["", ""]);
    let block = db.library.macro_id("B0").unwrap();
    db.library.macros[block].class = MacroClass::Block;
    let link = Link::new(&db, &lib, &[]).unwrap();
    let id = |name: &str| lib.cell_id(name).unwrap();

    let of_block = link.swappable_cells(id("B0"), SizingLimits::NONE);
    let of_core = link.swappable_cells(id("B1"), SizingLimits::NONE);

    assert_eq!(of_block, []);
    assert!(of_core.contains(&id("B1")) && of_core.contains(&id("B0")));
}

#[test]
fn a_user_function_class_keeps_sizes_within_it() {
    let class = |name: &str| format!("user_function_class : \"{name}\" ;");
    let (db, lib) = attributed_buffers(&[&class("a"), &class("a"), &class("b"), ""]);
    let link = Link::new(&db, &lib, &[]).unwrap();
    let id = |name: &str| lib.cell_id(name).unwrap();

    let of_a = names(&lib, &link.swappable_cells(id("B0"), SizingLimits::NONE));
    let of_none = names(&lib, &link.swappable_cells(id("B3"), SizingLimits::NONE));

    assert!(of_a.iter().all(|n| n == "B0" || n == "B1"), "{of_a:?}");
    assert_eq!(of_a.len(), 2);
    assert!(
        ["B0", "B1", "B2", "B3"]
            .iter()
            .all(|n| of_none.iter().any(|m| m == n)),
        "{of_none:?}"
    );
}

#[test]
fn clock_cells_and_power_management_buffers_are_not_chosen() {
    let attributes = [
        ("B0", "is_clock_cell : true ;"),
        ("B1", "is_level_shifter : true ;"),
        ("B2", "is_isolation_cell : true ;"),
        ("B3", "always_on : true ;"),
        ("B4", ""),
    ];
    let copies: Vec<(&str, &str)> = attributes.iter().map(|&(n, _)| ("BUF_X1", n)).collect();
    let sections = DefSections {
        components: "",
        nets: "",
        inputs: &[],
        outputs: &[],
    };
    let db = design_with_copies(&def_text("buffers", &sections), &copies);
    let extra: String = attributes
        .iter()
        .map(|&(name, attribute)| {
            buffer_cell(name, "buf", 0.1).replacen(
                "area : 0.8 ;",
                &format!("area : 0.8 ; {attribute}"),
                1,
            )
        })
        .collect();
    let lib = cells_with(&extra);
    let link = Link::new(&db, &lib, &["BUF_X1".to_string()]).unwrap();

    let set = link.find_buffers(&db).unwrap();

    let chosen: Vec<&str> = set
        .cells
        .iter()
        .map(|b| lib.cell(b.id).name.as_str())
        .collect();
    assert_eq!(chosen, ["B4"]);
}

/// Buffers in library order, each slewing to `slew` nanoseconds at
/// 2 fF, which sets its drive resistance.
fn buffers(slews: &[(&str, f64)]) -> Liberty {
    let mut cells = String::new();
    for (name, slew) in slews {
        write!(
            cells,
            "cell ({name}) {{
                   pin (A) {{ direction : input; capacitance : 1.0; }}
                   pin (Z) {{ direction : output; function : \"A\";
                     timing () {{ related_pin : \"A\"; timing_sense : positive_unate;
                       cell_rise(scalar) {{ values (\"0.01\"); }}
                       rise_transition(load) {{ values (\"0.0, {slew}\"); }} }} }} }}"
        )
        .unwrap();
    }
    let src = format!(
        "library (t) {{ time_unit : \"1ns\"; capacitive_load_unit (1,ff);
               lu_table_template (load) {{
                 variable_1 : total_output_net_capacitance; index_1 (\"1.0, 2.0\"); }}
               {cells} }}"
    );
    parse_str(&src, Path::new("t.lib")).unwrap()
}

fn measures(area: i64, leakage_nw: f64) -> CellMeasures {
    CellMeasures {
        area: Some(DbuArea(area)),
        leakage: Some(Watts::new(leakage_nw * 1e-9)),
    }
}

#[test]
fn a_size_over_four_times_the_leakage_is_not_offered() {
    let buf_x1 = measures(798, 21.438_247);
    let buf_x4 = measures(1862, 86.121_805);

    assert!(!SizingLimits::OPENROAD.allows(buf_x1, buf_x4));
}

#[test]
fn a_size_just_under_four_times_the_leakage_is_offered() {
    let oai21_x1 = measures(1064, 22.619_394);
    let oai21_x4 = measures(2660, 90.477_187);

    assert!(SizingLimits::OPENROAD.allows(oai21_x1, oai21_x4));
}

#[test]
fn a_size_over_four_times_the_area_is_not_offered_and_four_times_is() {
    let small = measures(100, 1.0);

    assert!(SizingLimits::OPENROAD.allows(small, measures(400, 1.0)));
    assert!(!SizingLimits::OPENROAD.allows(small, measures(401, 1.0)));
}

#[test]
fn missing_or_zero_measures_and_lifted_limits_offer_every_size() {
    let none = CellMeasures {
        area: None,
        leakage: None,
    };
    let big = measures(10_000, 1000.0);

    assert!(SizingLimits::OPENROAD.allows(none, big));
    assert!(SizingLimits::OPENROAD.allows(measures(0, 0.0), measures(0, 0.0)));
    assert!(SizingLimits::NONE.allows(measures(1, 1.0), big));
}

#[test]
fn any_leakage_over_a_leakless_size_is_not_offered() {
    assert!(!SizingLimits::OPENROAD.allows(measures(1, 0.0), measures(1, 1.0)));
}

fn leaky(body: &str) -> LibCell {
    let src = format!(
        "library (t) {{ leakage_power_unit : \"1nW\";
               cell (c) {{ {body} pin (A) {{ direction : input; capacitance : 1.0; }} }} }}"
    );
    parse_str(&src, Path::new("t.lib")).unwrap().cells[LibCellId(0)].clone()
}

#[test]
fn leakage_is_the_cell_value_else_the_mean_over_states_else_unknown() {
    let stated = leaky("cell_leakage_power : 5.0; leakage_power () { value : 9.0; }");
    let by_state = leaky("leakage_power () { value : 2.0; } leakage_power () { value : 4.0; }");

    assert_eq!(cell_leakage(&stated), Some(Watts::new(5.0e-9)));
    assert!((cell_leakage(&by_state).unwrap().get() - 3.0e-9).abs() < 1e-20);
    assert_eq!(cell_leakage(&leaky("")), None);
}

#[test]
fn equivalent_cells_run_from_weakest_drive_and_ties_keep_library_order() {
    let lib = buffers(&[("A", 0.2), ("B", 0.6), ("C", 0.4), ("D", 0.4)]);

    let (_, classes) = equivalence_classes(&lib, &IdVec::filled(false, 4));
    let names: Vec<&str> = classes[0]
        .iter()
        .map(|&c| lib.cell(c).name.as_str())
        .collect();

    assert_eq!(classes.len(), 1);
    assert_eq!(names, ["B", "C", "D", "A"]);
}

/// A design of no instances with copies of `BUF_X1` named `copies`.
fn empty_design_with(copies: &[&str]) -> NetlistDB {
    let copies: Vec<(&str, &str)> = copies.iter().map(|&n| ("BUF_X1", n)).collect();
    design_with_copies(
        &def_text(
            "empty",
            &DefSections {
                components: "",
                nets: "",
                inputs: &[],
                outputs: &[],
            },
        ),
        &copies,
    )
}

#[test]
fn a_liberty_cell_without_a_master_is_not_a_link_cell() {
    let db = empty_design_with(&["B0"]);
    let lib = cells_with(&(buffer_cell("B0", "buf", 0.1) + &buffer_cell("LONELY_X1", "buf", 0.1)));
    let link = Link::new(&db, &lib, &[]).unwrap();

    assert!(link.is_link_cell(lib.cell_id("B0").unwrap()));
    assert!(!link.is_link_cell(lib.cell_id("LONELY_X1").unwrap()));
}

#[test]
fn a_cell_without_a_master_is_not_offered_as_a_size() {
    let db = empty_design_with(&["B0"]);
    let lib = cells_with(&(buffer_cell("B0", "buf", 0.1) + &buffer_cell("LONELY_X1", "buf", 0.1)));
    let link = Link::new(&db, &lib, &[]).unwrap();
    let b0 = lib.cell_id("B0").unwrap();
    let lonely = lib.cell_id("LONELY_X1").unwrap();
    assert_eq!(link.class_of[b0], link.class_of[lonely]);

    let sizes = link.swappable_cells(b0, SizingLimits::NONE);

    assert!(sizes.contains(&b0));
    assert!(!sizes.contains(&lonely));
}

#[test]
fn a_buffers_input_capacitance_is_its_input_ports() {
    let (db, lib) = attributed_buffers(&[""]);
    let link = Link::new(&db, &lib, &[]).unwrap();
    let buffer = lib.buffer_cell(lib.cell_id("B0").unwrap()).unwrap();

    assert_eq!(link.buffer_input_capacitance(buffer), Farads::new(1.0e-15));
}

#[test]
fn the_most_used_sites_come_first_and_only_two_are_kept() {
    let site = |i: usize| Some(SiteId::new(i));
    let used = [site(0), site(1), site(1), site(2), site(2), site(2), None];

    assert_eq!(most_used_sites(used.into_iter()), [site(2), site(1)]);
    assert_eq!(most_used_sites(std::iter::empty()), []);
}

#[test]
fn buffers_split_into_five_bands_whose_sizes_differ_by_at_most_one() {
    let names: Vec<String> = (0..15).map(|i| format!("C{i}")).collect();
    let slews: Vec<(&str, f64)> = names.iter().map(|n| (n.as_str(), 0.1)).collect();
    let lib = buffers(&slews);
    let all: Vec<BufferCell> = lib
        .cells
        .ids()
        .map(|id| lib.buffer_cell(id).unwrap())
        .collect();

    for (count, expected) in [
        (15, [3, 3, 3, 3, 3]),
        (7, [2, 2, 1, 1, 1]),
        (3, [1, 1, 1, 0, 0]),
        (0, [0, 0, 0, 0, 0]),
    ] {
        let bands: Vec<&[BufferCell]> = drive_bands(&all[..count]).collect();
        let sizes: Vec<usize> = bands.iter().map(|b| b.len()).collect();
        assert_eq!(sizes, expected, "{count} buffers");
        assert_eq!(bands.concat(), all[..count], "{count} buffers");
    }
}

#[test]
fn clock_cell_candidates_are_the_cells_that_may_be_used_and_are_not_power_cells() {
    let (db, lib) = attributed_buffers(&["", "", "is_level_shifter : true ;"]);
    let link = Link::new(&db, &lib, &["B1".to_string()]).unwrap();
    let candidate = |name: &str| link.is_clock_cell_candidate(lib.cell_id(name).unwrap());

    assert!(candidate("B0"));
    assert!(!candidate("B1"));
    assert!(!candidate("B2"));
}

/// The names `infer_clock_buffers` chooses of `lib` in `db` with the cells
/// `dont_use` not to be used.
fn inferred_clock_buffers(db: &NetlistDB, lib: &Liberty, dont_use: &[&str]) -> Vec<String> {
    let dont_use: Vec<String> = dont_use.iter().map(|n| (*n).to_string()).collect();
    let link = Link::new(db, lib, &dont_use).unwrap();
    link.infer_clock_buffers(db)
        .iter()
        .map(|b| lib.cell(b.id).name.clone())
        .collect()
}

const INVERTER: &str = "cell (INVBUF_X1) { area : 0.53 ; \
    pin (A) { direction : input ; capacitance : 1.6 ; } \
    pin (ZN) { direction : output ; function : \"!A\" ; } }\n";

#[test]
fn clock_buffers_are_found_by_name_among_buffers_that_may_be_used_with_a_master() {
    let db = design_with_copies(
        &def_text(
            "named",
            &DefSections {
                components: "",
                nets: "",
                inputs: &[],
                outputs: &[],
            },
        ),
        &[("INV_X1", "INVBUF_X1")],
    );
    let lib = cells_with(&(INVERTER.to_string() + &buffer_cell("LONELYBUF_X1", "buf", 0.1)));

    let chosen = inferred_clock_buffers(&db, &lib, &["CLKBUF_X3"]);

    assert_eq!(chosen, ["BUF_X1"]);
}

/// The design of `BUF_X1` copies `name`, whose pin `pin` has `signal_use`
/// and `direction`.
fn buffer_with_pin_use(
    name: &str,
    pin: &str,
    direction: PinDirection,
    signal_use: SignalUse,
) -> (NetlistDB, Liberty) {
    let mut db = empty_design_with(&[name]);
    let id = db.library.macro_id(name).unwrap();
    let macro_pin = db.library.macros[id]
        .pins
        .iter_mut()
        .find(|p| p.name == pin)
        .unwrap();
    macro_pin.direction = direction;
    macro_pin.signal_use = signal_use;
    (db, cells_with(&buffer_cell(name, "buf", 0.1)))
}

#[test]
fn a_buffer_with_a_clock_input_pin_is_a_clock_buffer() {
    let (db, lib) = buffer_with_pin_use("TREE_X1", "A", PinDirection::Input, SignalUse::Clock);

    let chosen = inferred_clock_buffers(&db, &lib, &["CLKBUF_X3"]);

    assert_eq!(chosen, ["TREE_X1"]);
}

#[test]
fn a_buffer_with_a_clock_output_pin_is_not_a_clock_buffer_for_that() {
    let (db, lib) = buffer_with_pin_use("TREE_X1", "Z", PinDirection::Output, SignalUse::Clock);

    let chosen = inferred_clock_buffers(&db, &lib, &["CLKBUF_X3"]);

    assert_eq!(chosen, ["BUF_X1"]);
}

#[test]
fn only_a_plain_core_cell_is_a_logic_std_cell() {
    let mut db = design_with_copies(
        &def_text(
            "one",
            &DefSections {
                components: "- i BUF_X1 + PLACED ( 2000 2800 ) N ;\n",
                nets: "",
                inputs: &[],
                outputs: &[],
            },
        ),
        &[],
    );
    let instance = db.cell_id("i").unwrap();
    let master = db.library.macro_id("BUF_X1").unwrap();
    assert!(is_logic_std_cell(&db, instance));

    for class in [MacroClass::Block, MacroClass::Core(CoreKind::Spacer)] {
        db.library.macros[master].class = class;
        assert!(!is_logic_std_cell(&db, instance), "{class:?}");
    }
}
