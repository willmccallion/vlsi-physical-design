use super::*;

#[test]
fn a_reset_forgets_every_node_it_had_reached() {
    let mut table = StampedTable::default();
    table.reset(4);
    table.set(2, 1.5, None);

    table.reset(4);

    assert_eq!(table.cost(2), None);
}

#[test]
fn a_reached_node_keeps_its_cost_and_parent() {
    let mut table = StampedTable::default();
    table.reset(4);

    table.set(3, 2.0, Some(1));
    table.set(1, 0.0, None);

    assert_eq!((table.cost(3), table.parent(3)), (Some(2.0), Some(1)));
    assert_eq!(table.parent(1), None);
}
