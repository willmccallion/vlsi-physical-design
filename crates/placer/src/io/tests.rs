use super::*;

fn slot_row(count: usize) -> Vec<Slot> {
    (0..count)
        .map(|i| Slot {
            edge: Edge::Bottom,
            layer: LayerId::new(0),
            at: Point::new(Dbu::new(i64::try_from(i).unwrap()), Dbu::ZERO),
            blocked: false,
        })
        .collect()
}

fn section_lengths(count: usize) -> Vec<usize> {
    sections(&slot_row(count))
        .iter()
        .map(|s| s.slots.len())
        .collect()
}

#[test]
fn sections_split_an_edge_every_two_hundred_slots() {
    assert_eq!(section_lengths(428), vec![200, 200, 28]);
}

#[test]
fn sections_leave_out_a_last_slot_that_would_be_alone() {
    assert_eq!(section_lengths(201), vec![200]);
}

#[test]
fn a_blocked_slot_takes_up_a_section_but_offers_no_room() {
    let mut slots = slot_row(3);
    slots[1].blocked = true;

    let section = Section::new(&slots);

    assert_eq!(section.slots.len(), 3);
    assert_eq!(section.free_slots, 2);
    assert_eq!(section.middle().x, Dbu::new(1));
}

#[test]
fn slots_count_from_the_track_start_past_the_die_edge_and_corner() {
    let tracks = TrackDef {
        axis: TrackAxis::X,
        start: Dbu::new(70),
        count: 100,
        step: Dbu::new(100),
        layers: Vec::new(),
    };
    let reach = SlotReach {
        lo: Dbu::new(70),
        hi: Dbu::new(9930),
        apart: Dbu::new(180),
        max_corner: Dbu::new(2000),
    };

    let slots = reach.slots_of(&tracks);

    assert_eq!(slots.first(), Some(&Dbu::new(1670)));
    assert_eq!(slots.last(), Some(&Dbu::new(8270)));
    assert_eq!(slots[1] - slots[0], Dbu::new(200));
}

#[test]
fn a_box_on_two_edges_reaches_both_stretches() {
    let die = Rect::new(
        Point::new(Dbu::ZERO, Dbu::ZERO),
        Point::new(Dbu::new(1000), Dbu::new(1000)),
    );
    let corner = Rect::new(
        Point::new(Dbu::new(600), Dbu::ZERO),
        Point::new(Dbu::new(1000), Dbu::new(300)),
    );

    let stretches = stretches_on_die_edges(die, corner);

    assert_eq!(
        stretches,
        vec![
            EdgeStretch {
                edge: Edge::Bottom,
                lo: Dbu::new(600),
                hi: Dbu::new(1000)
            },
            EdgeStretch {
                edge: Edge::Right,
                lo: Dbu::ZERO,
                hi: Dbu::new(300)
            },
        ]
    );
}

#[test]
fn pin_rectangles_point_into_the_die() {
    let size = PinSize {
        half_width: Dbu::new(70),
        length: Dbu::new(140),
    };
    let at = Point::new(Dbu::new(1000), Dbu::new(0));

    let r = size.rect(Edge::Bottom, at);

    assert_eq!(
        r,
        Rect::new(
            Point::new(Dbu::new(930), Dbu::new(0)),
            Point::new(Dbu::new(1070), Dbu::new(140))
        )
    );
}
