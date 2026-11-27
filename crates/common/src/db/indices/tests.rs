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

#[test]
fn a_table_is_empty_only_without_entries() {
    let mut names: IdVec<CellId, &str> = IdVec::new();
    assert!(names.is_empty());
    let _ = names.push("a");
    assert!(!names.is_empty());
}

#[test]
fn tables_are_equal_only_with_the_same_entries() {
    let a: IdVec<CellId, i32> = IdVec::from(vec![1, 2]);

    assert_eq!(a, IdVec::from(vec![1, 2]));
    assert_ne!(a, IdVec::from(vec![1, 3]));
}

#[test]
fn growing_appends_copies_up_to_the_length_and_never_shrinks() {
    let mut a: IdVec<CellId, i32> = IdVec::from(vec![1, 2]);

    a.grow_to(4, 0);
    assert_eq!(a, IdVec::from(vec![1, 2, 0, 0]));
    a.grow_to(1, 9);
    assert_eq!(a, IdVec::from(vec![1, 2, 0, 0]));
}

#[test]
fn a_routing_layer_has_the_cut_below_it_unless_it_is_the_first() {
    assert_eq!(LayerId(2).cut_below(), Some(CutLayerId(1)));
    assert_eq!(LayerId(0).cut_below(), None);
}

#[test]
fn a_layer_steps_toward_a_target_one_layer_at_a_time() {
    assert_eq!(LayerId(1).toward(LayerId(4)), LayerId(2));
    assert_eq!(LayerId(4).toward(LayerId(1)), LayerId(3));
    assert_eq!(LayerId(3).toward(LayerId(3)), LayerId(3));
}

#[test]
fn a_table_is_filled_changed_and_copied_entry_by_entry() {
    let mut names: IdVec<CellId, &str> = IdVec::filled("x", 3);

    names.fill("y");
    *names.get_mut(CellId(1)).unwrap() = "z";
    for (id, name) in names.iter_enumerated_mut() {
        if id == CellId(2) {
            *name = "w";
        }
    }
    let mut copy: IdVec<CellId, &str> = IdVec::filled("q", 1);
    copy.clone_from(&names);

    assert_eq!(names.as_slice(), ["y", "z", "w"]);
    assert_eq!(names.get_mut(CellId(3)), None);
    assert_eq!(copy.as_slice(), names.as_slice());
}

#[test]
fn layers_step_down_one_at_a_time_and_run_up_through_a_top() {
    assert_eq!(LayerId(3).below(), Some(LayerId(2)));
    assert_eq!(LayerId(0).below(), None);
    assert_eq!(
        LayerId(1).through(LayerId(3)).collect::<Vec<_>>(),
        [LayerId(1), LayerId(2), LayerId(3)]
    );
}
