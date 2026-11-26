use super::*;

#[test]
fn synthesized_stack_alternates_and_has_a_via_per_cut() {
    let mut lib = Library::new(BOOKSHELF_DBU);

    synthesize_layer_stack(&mut lib, Dbu::new(6400), 4).unwrap();

    let dirs: Vec<_> = lib.routing_layers.iter().map(|l| l.direction).collect();
    assert_eq!(
        dirs,
        [
            Direction::Horizontal,
            Direction::Vertical,
            Direction::Horizontal,
            Direction::Vertical
        ]
    );
    assert_eq!(lib.routing_layers[LayerId(0)].track_pitch(), Dbu::new(400));
    assert_eq!(lib.cut_layers.len(), 3);
    assert!((0..3).all(|c| lib.default_via(crate::db::indices::CutLayerId(c)).is_some()));
}
