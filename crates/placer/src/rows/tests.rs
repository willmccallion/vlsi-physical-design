use super::*;

fn blocked_widths(kind: BlockageKind, sites: i64) -> Vec<(i64, i64)> {
    let row = span(0, 0, 200 * sites);
    SitePattern::new(kind)
        .stretches(&row, Dbu::ZERO, row.x_max)
        .iter()
        .map(|r| (r.min.x.get(), r.max.x.get()))
        .collect()
}

#[test]
fn a_soft_blockage_blocks_its_whole_stretch_at_once() {
    assert_eq!(blocked_widths(BlockageKind::Soft, 4), [(0, 800)]);
}

#[test]
fn a_partial_blockage_blocks_every_other_site_at_half_density() {
    assert_eq!(
        blocked_widths(BlockageKind::Partial(50.0), 4),
        [(0, 200), (400, 600)]
    );
}

#[test]
fn a_partial_blockage_of_full_density_still_blocks_its_first_site() {
    assert_eq!(blocked_widths(BlockageKind::Partial(100.0), 3), [(0, 200)]);
}

fn span(y: i64, x_min: i64, x_max: i64) -> RowSpan {
    RowSpan {
        y: Dbu::new(y),
        height: Dbu::new(2000),
        x_min: Dbu::new(x_min),
        x_max: Dbu::new(x_max),
        site: Dbu::new(200),
        orient: Orientation::N,
    }
}

fn extents(rows: &[RowSpan]) -> Vec<(i64, i64, i64)> {
    rows.iter()
        .map(|r| (r.y.get(), r.x_min.get(), r.x_max.get()))
        .collect()
}

#[test]
fn a_row_inside_another_at_the_same_height_merges_into_it() {
    let rows = vec![span(0, 0, 10_000), span(0, 200, 6_000)];

    assert_eq!(extents(&merge_overlapping(rows)), [(0, 0, 10_000)]);
}

#[test]
fn abutting_rows_on_one_grid_merge_but_separate_ones_do_not() {
    let rows = vec![
        span(0, 0, 4_000),
        span(0, 4_000, 6_000),
        span(0, 8_000, 9_000),
    ];

    assert_eq!(
        extents(&merge_overlapping(rows)),
        [(0, 0, 6_000), (0, 8_000, 9_000)]
    );
}

#[test]
fn rows_off_each_others_site_grid_stay_apart() {
    let rows = vec![span(0, 0, 4_000), span(0, 100, 6_000)];

    assert_eq!(merge_overlapping(rows).len(), 2);
}
