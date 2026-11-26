use super::*;

#[test]
fn values_off_the_grid_are_rounded_counted_and_the_first_named() {
    let mut lx = Lexer::new("0.0004 0.002\n250.0001 0.0000004 ;", Path::new("t.lef"));
    let dbu = DbuPerMicron::new(1000).unwrap();
    let mut off_grid = OffGridValues::default();
    let mut rules = RuleReader::new(&mut lx, dbu, &mut off_grid);

    let read: Vec<Dbu> = (0..3).map(|_| rules.microns("length").unwrap()).collect();
    let area = rules.area("area").unwrap();

    assert_eq!(read, [Dbu::new(0), Dbu::new(2), Dbu::new(250_000)]);
    assert_eq!(area, DbuArea(0));
    assert_eq!(
        off_grid.warning(Path::new("t.lef"), dbu).as_deref(),
        Some(
            "t.lef: 3 values off the 1/1000 um grid rounded to the nearest unit, the \
                 first 0.0004 um at line 1"
        )
    );
}

#[test]
fn a_file_on_the_grid_warns_of_nothing() {
    let off_grid = OffGridValues::default();

    assert_eq!(
        off_grid.warning(Path::new("t.lef"), DbuPerMicron::new(1000).unwrap()),
        None
    );
}
