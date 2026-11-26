use super::*;

const ROW_HEIGHT: Dbu = Dbu::new(100);

/// Rows `ROW_HEIGHT` high stacked from the bottom at zero.
fn rows(count: i64) -> Arch {
    let rows: Vec<Row> = (0..count)
        .map(|r| Row {
            left: Dbu::new(0),
            bottom: ROW_HEIGHT * r,
            height: ROW_HEIGHT,
            site_spacing: Dbu::new(10),
            site_width: Dbu::new(10),
            num_sites: 10,
            orient: Orientation::N,
            symmetry: Symmetry::default(),
        })
        .collect();
    Arch {
        rows: rows.into(),
        min_x: Dbu::new(0),
        max_x: Dbu::new(100),
        min_y: Dbu::new(0),
        max_y: ROW_HEIGHT * count,
    }
}

#[test]
fn a_point_below_the_rows_is_closest_to_the_first() {
    assert_eq!(rows(3).closest_row(Dbu::new(-5)), ArchRowId::new(0));
}

#[test]
fn a_point_above_the_rows_is_closest_to_the_last() {
    assert_eq!(rows(3).closest_row(Dbu::new(500)), ArchRowId::new(2));
}

#[test]
fn a_point_is_closest_to_the_row_whose_bottom_is_nearest() {
    let arch = rows(3);

    assert_eq!(
        [40, 60, 100, 160].map(|y| arch.closest_row(Dbu::new(y))),
        [0, 1, 1, 2].map(ArchRowId::new)
    );
}

#[test]
fn a_point_halfway_between_rows_is_closest_to_the_lower() {
    assert_eq!(rows(3).closest_row(Dbu::new(150)), ArchRowId::new(1));
}

#[test]
fn a_cell_spans_its_height_in_rows_rounded_half_up() {
    let arch = rows(3);

    assert_eq!(
        [100, 140, 150, 200].map(|h| arch.rows_spanned(Dbu::new(h))),
        [1, 1, 2, 2]
    );
}
