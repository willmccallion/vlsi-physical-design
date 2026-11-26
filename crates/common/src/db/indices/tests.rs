use super::*;

#[test]
fn each_push_returns_the_index_its_entry_is_found_at() {
    let mut names: IdVec<CellId, &str> = IdVec::new();

    let (a, b) = (names.push("a"), names.push("b"));

    assert_eq!((a, b), (CellId(0), CellId(1)));
    assert_eq!((names[a], names[b]), ("a", "b"));
}

#[test]
fn the_next_id_is_the_one_the_next_push_returns() {
    let mut names: IdVec<NetId, &str> = vec!["n0"].into();

    let next = names.next_id();

    assert_eq!(names.push("n1"), next);
}

#[test]
fn entries_enumerate_with_their_indices_in_order() {
    let names: IdVec<PinId, &str> = vec!["p0", "p1"].into();

    let listed: Vec<(PinId, &str)> = names.iter_enumerated().map(|(i, &n)| (i, n)).collect();

    assert_eq!(listed, vec![(PinId(0), "p0"), (PinId(1), "p1")]);
    assert_eq!(names.ids().collect::<Vec<_>>(), vec![PinId(0), PinId(1)]);
}

#[test]
fn the_top_layer_id_has_no_layer_above() {
    assert_eq!(LayerId(u8::MAX).above(), None);
    assert_eq!(CutLayerId(u8::MAX).above(), None);
    assert_eq!(CutLayerId(3).above(), Some(LayerId(4)));
}

#[test]
fn a_full_table_has_no_next_id() {
    let mut layers: IdVec<LayerId, ()> = IdVec::filled((), 255);
    assert_eq!(layers.try_next_id("layer"), Ok(LayerId(255)));
    let _ = layers.push(());

    let full = layers.try_next_id("layer");

    assert_eq!(
        full.map_err(|e| e.to_string()),
        Err("more than 256 layers".to_string())
    );
}

#[test]
fn an_index_past_the_end_gets_nothing() {
    let names: IdVec<CellId, &str> = IdVec::filled("x", 2);

    assert_eq!(names.get(CellId(1)), Some(&"x"));
    assert_eq!(names.get(CellId(2)), None);
}
