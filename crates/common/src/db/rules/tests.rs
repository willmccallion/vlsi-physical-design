use super::*;

fn two_widths() -> TwoWidthsTable {
    let row = |width, run_length: Option<i64>, spacings: [i64; 3]| TwoWidthsRow {
        width: Dbu::new(width),
        run_length: run_length.map(Dbu::new),
        spacings: spacings.map(Dbu::new).to_vec(),
    };
    TwoWidthsTable {
        rows: vec![
            row(0, None, [50, 50, 64]),
            row(74, Some(0), [50, 60, 74]),
            row(212, Some(302), [64, 74, 112]),
        ],
    }
}

#[test]
fn two_widths_rows_apply_only_to_shapes_wider_than_their_width() {
    let table = two_widths();

    assert_eq!(
        table.spacing(Dbu::new(74), Dbu::new(74), Dbu::new(1000)),
        Dbu::new(50)
    );
    assert_eq!(
        table.spacing(Dbu::new(75), Dbu::new(75), Dbu::new(1000)),
        Dbu::new(60)
    );
}

#[test]
fn two_widths_rows_step_down_until_the_run_length_condition_holds() {
    let table = two_widths();

    assert_eq!(
        table.spacing(Dbu::new(300), Dbu::new(300), Dbu::new(1000)),
        Dbu::new(112)
    );
    assert_eq!(
        table.spacing(Dbu::new(300), Dbu::new(300), Dbu::new(302)),
        Dbu::new(112)
    );
    assert_eq!(
        table.spacing(Dbu::new(300), Dbu::new(300), Dbu::new(301)),
        Dbu::new(60)
    );
}

#[test]
fn two_widths_treats_shapes_apart_as_no_run() {
    let table = two_widths();

    assert_eq!(
        table.spacing(Dbu::new(300), Dbu::new(10), Dbu::new(-40)),
        Dbu::new(50)
    );
}

fn parallel_run_length() -> ParallelRunLengthTable {
    ParallelRunLengthTable {
        run_lengths: vec![Dbu::new(0), Dbu::new(600)],
        rows: vec![
            (Dbu::new(0), vec![Dbu::new(140), Dbu::new(140)]),
            (Dbu::new(180), vec![Dbu::new(140), Dbu::new(180)]),
        ],
    }
}

#[test]
fn parallel_run_length_rows_apply_only_to_shapes_wider_than_their_width() {
    let table = parallel_run_length();

    assert_eq!(table.spacing(Dbu::new(180), Dbu::new(1000)), Dbu::new(140));
    assert_eq!(table.spacing(Dbu::new(181), Dbu::new(1000)), Dbu::new(180));
}

#[test]
fn parallel_run_length_columns_apply_only_to_longer_runs() {
    let table = parallel_run_length();

    assert_eq!(table.spacing(Dbu::new(200), Dbu::new(600)), Dbu::new(140));
    assert_eq!(table.spacing(Dbu::new(200), Dbu::new(601)), Dbu::new(180));
}

#[test]
fn two_widths_extremes_are_its_first_and_largest_entries() {
    let table = SpacingTable::TwoWidths(two_widths());

    assert_eq!(table.min_spacing(), Dbu::new(50));
    assert_eq!(table.max_spacing(), Dbu::new(112));
}
