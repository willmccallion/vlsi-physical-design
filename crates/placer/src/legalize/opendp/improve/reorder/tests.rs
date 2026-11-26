use super::*;

const SITE: Dbu = Dbu::new(10);

fn lengths(values: &[i64]) -> Vec<Dbu> {
    values.iter().copied().map(Dbu::new).collect()
}

#[test]
fn cells_are_padded_evenly_in_whole_sites_over_the_gap() {
    let padding = even_padding(&lengths(&[20, 30]), Dbu::new(100), SITE);

    assert_eq!(padding, Some((lengths(&[10, 10]), lengths(&[10, 10]))));
}

#[test]
fn cells_filling_the_gap_get_no_padding() {
    let padding = even_padding(&lengths(&[20, 30]), Dbu::new(50), SITE);

    assert_eq!(padding, Some((lengths(&[0, 0]), lengths(&[0, 0]))));
}

#[test]
fn padding_that_would_fill_the_gap_exactly_is_left_off() {
    // Three spare sites split one right and two left; the two left
    // would close the gap, so only the right one is taken.
    let padding = even_padding(&lengths(&[10]), Dbu::new(40), SITE);

    assert_eq!(padding, Some((lengths(&[0]), lengths(&[10]))));
}

#[test]
fn cells_wider_than_the_gap_get_no_padding() {
    assert_eq!(even_padding(&lengths(&[20, 30]), Dbu::new(40), SITE), None);
}

#[test]
fn permutations_step_through_every_order_then_return_sorted() {
    let mut order = vec![0, 1, 2];
    let mut seen = vec![order.clone()];

    while next_permutation(&mut order) {
        seen.push(order.clone());
    }

    assert_eq!(
        seen,
        [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0]
        ]
    );
    assert_eq!(order, [0, 1, 2]);
}

#[test]
fn repeated_values_are_permuted_once_each() {
    let mut order = vec![0, 0, 1];
    let mut count = 1;

    while next_permutation(&mut order) {
        count += 1;
    }

    assert_eq!(count, 3);
}
