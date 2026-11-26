use super::*;

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

fn spans(list: &[(i64, i64)]) -> Vec<Interval> {
    list.iter()
        .map(|&(a, b)| Interval::new(Dbu::new(a), Dbu::new(b)))
        .collect()
}

fn span(lo: i64, hi: i64) -> Interval {
    Interval::new(Dbu::new(lo), Dbu::new(hi))
}

#[test]
fn a_fence_covering_the_band_gives_its_width() {
    let fence = [rect(100, 0, 400, 2000)];

    let got = inside(&fence, span(0, 1000), span(0, 1000));

    assert_eq!(got, spans(&[(100, 400)]));
}

#[test]
fn abutting_fence_rectangles_merge_into_one_interval() {
    let fence = [rect(100, 0, 400, 2000), rect(400, 0, 700, 2000)];

    let got = inside(&fence, span(0, 1000), span(0, 1000));

    assert_eq!(got, spans(&[(100, 700)]));
}

#[test]
fn a_fence_covering_part_of_the_band_height_gives_nothing() {
    let fence = [rect(100, 0, 400, 500)];

    let got = inside(&fence, span(0, 1000), span(0, 1000));

    assert_eq!(got, []);
}

#[test]
fn fence_rectangles_stacked_over_the_band_cover_it() {
    let fence = [rect(100, 0, 400, 500), rect(100, 500, 400, 1000)];

    let got = inside(&fence, span(0, 1000), span(0, 1000));

    assert_eq!(got, spans(&[(100, 400)]));
}

#[test]
fn outside_leaves_the_gaps_between_fences() {
    let fences = [rect(100, 0, 400, 2000), rect(600, 0, 700, 2000)];

    let got = free_intervals(&fences, span(0, 1000), span(0, 1000));

    assert_eq!(got, spans(&[(0, 100), (400, 600), (700, 1000)]));
}

#[test]
fn a_fence_touching_part_of_the_band_height_still_blocks_outside_cells() {
    let fences = [rect(100, 900, 400, 2000)];

    let got = free_intervals(&fences, span(0, 1000), span(0, 1000));

    assert_eq!(got, spans(&[(0, 100), (400, 1000)]));
}

#[test]
fn a_fence_only_abutting_the_band_does_not_block_it() {
    let fences = [rect(100, 1000, 400, 2000)];

    let got = free_intervals(&fences, span(0, 1000), span(0, 1000));

    assert_eq!(got, spans(&[(0, 1000)]));
}
