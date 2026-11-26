use super::*;

fn sink_at(i: usize, x: i64, y: i64) -> ClockSink {
    ClockSink {
        name: format!("ff{i}/CK"),
        location: Point::new(Dbu::new(x), Dbu::new(y)),
        pin: PinId::new(i),
        input_cap: Farads32::ZERO,
        insertion_delay: Seconds::ZERO,
    }
}

#[test]
fn the_sink_region_leaves_out_the_outer_hundredth_of_the_sinks_on_each_side() {
    let mut clock = Clock::new(NetId::new(0), "clk", "clk");
    for i in 0..100 {
        clock.add_sink(sink_at(
            i,
            i64::saturating_from(i),
            2 * i64::saturating_from(i),
        ));
    }
    clock.add_sink(sink_at(100, 10_000, -10_000));

    let region = clock.sink_region();

    assert_eq!(
        region,
        Rect::new(
            Point::new(Dbu::new(1), Dbu::new(0)),
            Point::new(Dbu::new(99), Dbu::new(196))
        )
    );
}

#[test]
fn a_buffer_is_named_after_its_net_with_slashes_escaped() {
    let mut clock = Clock::new(NetId::new(0), "core/clk", "clk");

    let buffer = clock.add_buffer(
        "clkbuf_0",
        BufferCell {
            id: pare_common::db::liberty::LibCellId::new(0),
            input: pare_common::db::liberty::LibPortId::new(0),
            output: pare_common::db::liberty::LibPortId::new(1),
        },
        Point::default(),
    );

    assert_eq!(clock.name(buffer), "clkbuf_0_core\\/clk");
}
