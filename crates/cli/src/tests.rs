use super::*;
use crate::testing::scratch_dir;

/// A configs directory holding an empty config at each of `files`.
fn configs_with(tag: &str, files: &[&str]) -> PathBuf {
    let configs = scratch_dir(tag);
    for file in files {
        let path = configs.join(file);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, "").unwrap();
    }
    configs
}

fn config_with_input(input: &str) -> Config {
    toml::from_str(&format!(
        "purpose = \"placement\"\n[input]\nlef_files = [\"a.lef\"]\ndef_file = \"a.def\"\n{input}"
    ))
    .unwrap()
}

#[test]
fn a_config_path_is_taken_as_it_is() {
    let configs = configs_with("resolve-path", &[]);

    for name in ["suite/design", "design.toml", "design.TOML"] {
        assert_eq!(
            resolve_config(Path::new(name), &configs).unwrap(),
            PathBuf::from(name)
        );
    }
}

#[test]
fn a_design_name_finds_its_config_in_whichever_suite_has_it() {
    let configs = configs_with("resolve-one", &["ibm/ibm01.toml", "openroad/aes.toml"]);

    let path = resolve_config(Path::new("aes"), &configs).unwrap();

    assert_eq!(path, configs.join("openroad/aes.toml"));
}

#[test]
fn a_design_name_without_a_config_says_where_to_get_one() {
    let configs = configs_with("resolve-none", &["openroad/aes.toml"]);

    let error = resolve_config(Path::new("ibex"), &configs).unwrap_err();

    assert_eq!(
        error.to_string(),
        format!(
            "no config for 'ibex' in {}/<suite>/; download it with scripts/download.py",
            configs.display()
        )
    );
}

#[test]
fn a_design_name_in_two_suites_is_ambiguous() {
    let configs = configs_with("resolve-two", &["a/gcd.toml", "b/gcd.toml"]);

    let error = resolve_config(Path::new("gcd"), &configs).unwrap_err();

    assert!(
        error.to_string().starts_with("'gcd' names several configs"),
        "{error}"
    );
}

#[test]
fn a_design_name_without_a_configs_directory_names_the_directory() {
    let missing = scratch_dir("resolve-missing").join("configs");

    let error = resolve_config(Path::new("aes"), &missing).unwrap_err();

    assert_eq!(
        error.to_string(),
        format!("cannot read {}/", missing.display())
    );
}

#[test]
fn the_output_directory_is_the_one_the_config_names() {
    let config = config_with_input("output_def = \"out/placed.def\"\noutput_dir = \"images\"");

    assert_eq!(output_dir(&config), PathBuf::from("images"));
}

#[test]
fn without_an_output_directory_results_go_beside_the_output_def() {
    let config = config_with_input("output_def = \"out/aes/placed.def\"");

    assert_eq!(output_dir(&config), PathBuf::from("out/aes"));
}

#[test]
fn an_output_def_without_a_directory_puts_results_in_output() {
    let config = config_with_input("output_def = \"placed.def\"");

    assert_eq!(output_dir(&config), PathBuf::from("output"));
}

#[test]
fn a_box_needs_four_corner_coordinates() {
    assert!(MicronBox::from_corners(&[1.0, 2.0, 3.0]).is_err());
}

#[test]
fn a_box_in_microns_is_rounded_onto_the_database_grid() {
    let units = DbuPerMicron::new(2000).unwrap();

    let area = MicronBox::from_corners(&[1.0, 2.5, 3.00025, 4.0])
        .unwrap()
        .in_dbu(units);

    assert_eq!(
        area,
        Rect::new(
            Point::new(Dbu::new(2000), Dbu::new(5000)),
            Point::new(Dbu::new(6001), Dbu::new(8000))
        )
    );
}

#[test]
fn a_box_given_high_corner_first_is_the_same_box() {
    let units = DbuPerMicron::new(1000).unwrap();
    let at = |corners: &[f64]| MicronBox::from_corners(corners).unwrap().in_dbu(units);

    assert_eq!(at(&[6.0, 6.0, 0.0, 0.0]), at(&[0.0, 0.0, 6.0, 6.0]));
    assert_eq!(at(&[6.0, 0.0, 0.0, 6.0]), at(&[0.0, 0.0, 6.0, 6.0]));
}
