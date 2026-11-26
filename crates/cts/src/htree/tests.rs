use super::*;
use crate::clock::ClockSink;
use crate::options::CtsOptions;
use pare_common::db::indices::{NetId, PinId};
use pare_common::db::liberty::{LibCellId, LibPortId};

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

fn buffer() -> BufferCell {
    BufferCell {
        id: LibCellId::new(0),
        input: LibPortId::new(0),
        output: LibPortId::new(1),
    }
}

/// What a tree context borrows: blockages and buffer sizes in a core
/// 100 database units a side.
struct Scene {
    settings: Settings,
    buffer_sizes: FxHashMap<BufferCell, Point<Dbu>>,
    blockages: Vec<Rect<Dbu>>,
}

impl Scene {
    /// The scene of `blockages`, its buffer of no size.
    fn new(blockages: Vec<Rect<Dbu>>) -> Self {
        Self {
            settings: Settings::new(CtsOptions::default()),
            buffer_sizes: FxHashMap::default(),
            blockages,
        }
        .with_buffer_size(0, 0)
    }

    fn with_buffer_size(mut self, width: i64, height: i64) -> Self {
        let _ = self
            .buffer_sizes
            .insert(buffer(), Point::new(Dbu::new(width), Dbu::new(height)));
        self
    }

    fn ctx(&self) -> TreeContext<'_> {
        TreeContext {
            settings: &self.settings,
            core: rect(0, 0, 100, 100),
            dbu: 1000,
            root_buffer: buffer(),
            sink_buffer: buffer(),
            buffer_sizes: &self.buffer_sizes,
            blockages: &self.blockages,
        }
    }
}

/// A builder whose tree unit is one database unit, its sinks spread
/// over the whole core.
fn builder() -> Builder {
    let mut builder = Builder::new(
        Clock::new(NetId::new(0), "clk", "clk"),
        None,
        LengthUnit::new(1),
    );
    builder.sink_region = Region::new(0.0, 0.0, 100.0, 100.0);
    builder
}

#[test]
fn a_dummy_in_a_blockage_moves_along_the_blockages_full_box() {
    // The dummy lands inside the first blockage; the first point at the
    // level's length from the parent along it is its lower left corner,
    // which the second blockage covers.
    let scene = Scene::new(vec![rect(60, 40, 80, 60), rect(55, 35, 65, 45)]);
    let mut builder = builder();
    let mut level = LevelTopology::new(20.0, Vec::new(), NO_CHOICE);
    let (dummy, branch) = level.add_branches(TreePoint::new(0.0, 0.0), TreePoint::new(30.0, 50.0));
    level.sinks[branch].push(SinkPoint::new(30.0, 40.0), InstRef::Sink(0));
    builder.topologies.push(level);

    builder.legalize_dummies(&scene.ctx());

    let moved = builder.topologies[0].points[dummy];
    assert!(moved.same(TreePoint::new(60.0, 40.0)), "{moved:?}");
}

#[test]
fn a_buffer_outside_every_blockage_stays_put() {
    let scene = Scene::new(vec![rect(40, 40, 60, 60)]).with_buffer_size(4, 2);
    let at = TreePoint::new(20.0, 50.0);

    let legal = builder().legalize_in_blockages(&scene.ctx(), at, buffer());

    assert!(legal.same(at), "{legal:?}");
}

#[test]
fn a_buffer_on_a_blockage_edge_is_outside_it() {
    let scene = Scene::new(vec![rect(40, 40, 60, 60)]).with_buffer_size(4, 2);
    let at = TreePoint::new(40.0, 50.0);

    let legal = builder().legalize_in_blockages(&scene.ctx(), at, buffer());

    assert!(legal.same(at), "{legal:?}");
}

#[test]
fn a_buffer_in_a_blockage_moves_its_width_clear_of_the_nearest_side() {
    let scene = Scene::new(vec![rect(40, 40, 60, 60)]).with_buffer_size(4, 2);

    let legal = builder().legalize_in_blockages(&scene.ctx(), TreePoint::new(45.0, 50.0), buffer());

    assert!(legal.same(TreePoint::new(36.0, 50.0)), "{legal:?}");
}

#[test]
fn a_buffer_in_a_blockage_skips_a_taken_spot_beside_it() {
    let scene = Scene::new(vec![rect(40, 40, 60, 60)]).with_buffer_size(4, 2);
    let mut builder = builder();
    builder.commit(TreePoint::new(36.0, 50.0));

    let legal = builder.legalize_in_blockages(&scene.ctx(), TreePoint::new(45.0, 50.0), buffer());

    assert!(legal.same(TreePoint::new(45.0, 60.0)), "{legal:?}");
}

/// A segment `length` units long with `delay`, entering and leaving at
/// slew and load 1, with a buffer at each share of `buffers`, the
/// first `wl_to_first_buffer` units in.
fn segment(length: u8, delay: u32, buffers: &[f64], wl_to_first_buffer: i32) -> WireSegment {
    WireSegment {
        length,
        load: 1,
        output_slew: 1,
        delay,
        input_cap: 1,
        input_slew: 1,
        buffer_locations: buffers.to_vec(),
        buffer_masters: buffers.iter().map(|_| buffer()).collect(),
        wl_to_first_buffer,
        last_wl: 0,
    }
}

/// A search for a segment `length` long from slew and load 1 with no
/// wire since the last buffer, under `wirelength_threshold`.
const fn query(length: i64, wirelength_threshold: i64) -> SegmentQuery {
    SegmentQuery {
        length: LengthSteps::new(length),
        input_slew: SlewSteps::FIRST,
        input_cap: CapSteps::new(1),
        slew_threshold: SlewSteps::new(MAX_SLEW),
        wirelength_threshold: LengthSteps::new(wirelength_threshold),
    }
}

const NO_CHOICE: SegmentChoice = SegmentChoice {
    slew: SlewSteps::new(0),
    cap: CapSteps::new(0),
    curr_wl: LengthSteps::new(0),
};

/// The segment `search` finds in `tech`, starting from no wire since
/// the last buffer.
fn choose(tech: &TechChar, search: &SegmentQuery) -> Option<usize> {
    let mut out = NO_CHOICE;
    min_delay_segment(tech, search, 1, &mut out)
}

#[test]
fn the_fastest_segment_within_the_tolerance_of_the_input_is_chosen() {
    let tech = TechChar::from_segments(
        vec![
            segment(2, 5, &[], 2),
            segment(2, 3, &[], 2),
            WireSegment {
                input_cap: 5,
                ..segment(2, 1, &[], 2)
            },
        ],
        1,
    );

    let chosen = choose(&tech, &query(2, 100));

    assert_eq!(chosen, Some(1));
}

#[test]
fn a_search_finding_nothing_widens_its_tolerance() {
    let tech = TechChar::from_segments(
        vec![WireSegment {
            input_cap: 3,
            ..segment(2, 1, &[], 2)
        }],
        1,
    );

    let chosen = choose(&tech, &query(2, 100));

    assert_eq!(chosen, Some(0));
}

#[test]
fn once_the_slew_reaches_its_threshold_a_buffered_segment_beats_a_faster_bare_one() {
    let at_threshold = |s: WireSegment| WireSegment { input_slew: 4, ..s };
    let tech = TechChar::from_segments(
        vec![
            at_threshold(segment(2, 1, &[], 2)),
            at_threshold(segment(2, 5, &[0.5], 1)),
        ],
        1,
    );
    let search = SegmentQuery {
        input_slew: SlewSteps::new(4),
        ..query(2, 100)
    };

    let chosen = choose(&tech, &search);

    assert_eq!(chosen, Some(1));
}

#[test]
fn a_bare_segment_raises_the_slew_and_adds_its_wire_to_the_unbuffered_run() {
    let tech = TechChar::from_segments(
        vec![WireSegment {
            last_wl: 2,
            ..segment(2, 1, &[], 2)
        }],
        1,
    );
    let mut out = SegmentChoice {
        curr_wl: LengthSteps::new(3),
        ..NO_CHOICE
    };

    let _ = min_delay_segment(&tech, &query(2, 100), 1, &mut out);

    assert_eq!(
        (out.slew, out.curr_wl),
        (SlewSteps::new(2), LengthSteps::new(5))
    );
}

#[test]
fn a_buffered_segment_whose_first_buffer_is_past_the_wire_limit_is_passed_over() {
    let tech = TechChar::from_segments(vec![segment(2, 1, &[0.5], 6), segment(2, 5, &[0.5], 1)], 1);

    let chosen = choose(&tech, &query(2, 5));

    assert_eq!(chosen, Some(1));
}

#[test]
fn a_levels_segments_add_up_to_its_branch_length() {
    let scene = Scene::new(Vec::new());
    let tech = TechChar::from_segments(vec![segment(2, 1, &[], 2), segment(1, 1, &[], 1)], 1);
    let mut tree = builder();
    tree.min_input_cap = CapSteps::new(1);
    tree.min_length_sink_region = LengthSteps::new(2);
    tree.top_level_sinks_clustered = BranchSinks {
        points: vec![SinkPoint::new(10.0, 50.0), SinkPoint::new(90.0, 50.0)],
        insts: vec![InstRef::Sink(0), InstRef::Sink(1)],
    };

    tree.level_topology(&scene.ctx(), &tech, 1, 10.0, 10.0);

    let lengths: Vec<u8> = tree.topologies[0]
        .wire_segments
        .iter()
        .map(|&k| tech.segment(k).length)
        .collect();
    assert_eq!(lengths, [2, 2, 1]);
}

#[test]
fn a_bare_segment_is_refused_under_a_limit_shorter_than_the_shortest_segment() {
    let tech = TechChar::from_segments(vec![segment(2, 1, &[], 2)], 1);

    assert_eq!(choose(&tech, &query(2, 1)), None);
}

/// The segments chosen for the first level of a tree whose length unit
/// is 14000 database units, 7 microns, under a wire length limit of
/// `limit_um` microns: 0, buffered 10 units (70 microns) in, or 1, a
/// slower bare segment.
fn first_level_under(limit_um: i64) -> Vec<usize> {
    let mut scene = Scene::new(Vec::new());
    scene.settings.max_wire_length = Some(Dbu::new(limit_um * 2000));
    let tech = TechChar::from_segments(
        vec![segment(10, 1, &[1.0], 10), segment(10, 9, &[], 10)],
        14_000,
    );
    let mut tree = builder();
    tree.unit = LengthUnit::new(14_000);
    tree.min_input_cap = CapSteps::new(1);
    tree.min_length_sink_region = LengthSteps::new(20);
    tree.sink_region = Region::new(0.0, 0.0, 40.0, 40.0);
    tree.top_level_sinks_clustered = BranchSinks {
        points: vec![SinkPoint::new(1.0, 1.0), SinkPoint::new(39.0, 39.0)],
        insts: vec![InstRef::Sink(0), InstRef::Sink(1)],
    };

    tree.level_topology(&scene.ctx(), &tech, 1, 20.0, 20.0);

    tree.topologies[0].wire_segments.clone()
}

#[test]
fn a_wire_length_limit_admits_a_first_buffer_within_it() {
    assert_eq!(first_level_under(100), [0]);
}

/// Where `SegmentBuilder` puts the buffers of `segments`, laid end to
/// end along the branch from `root` to `target`, in a tree whose unit
/// is one database unit.
fn buffers_along(
    root: TreePoint,
    target: TreePoint,
    segments: Vec<WireSegment>,
) -> Vec<Point<Dbu>> {
    let scene = Scene::new(Vec::new()).with_buffer_size(1, 1);
    let wires: Vec<usize> = (0..segments.len()).collect();
    let tech = TechChar::from_segments(segments, 1);
    let mut tree = builder();
    let driving = tree.clock.add_subnet("root");
    let mut branch = SegmentBuilder {
        inst_prefix: "b_".into(),
        net_prefix: "n_".into(),
        root,
        target,
        driving,
        levels: 0,
    };

    branch.build(&mut tree, &scene.ctx(), &tech, &wires);

    tree.clock.buffers.iter().map(|b| b.location).collect()
}

fn at(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

#[test]
fn a_buffer_halfway_along_a_straight_branch_sits_at_its_middle() {
    let placed = buffers_along(
        TreePoint::new(0.0, 50.0),
        TreePoint::new(10.0, 50.0),
        vec![segment(10, 1, &[0.5], 5)],
    );

    assert_eq!(placed, [at(5, 50)]);
}

#[test]
fn buffers_of_successive_segments_sit_where_each_segment_puts_them() {
    let placed = buffers_along(
        TreePoint::new(0.0, 50.0),
        TreePoint::new(20.0, 50.0),
        vec![segment(10, 1, &[0.5], 5), segment(10, 1, &[0.5], 5)],
    );

    assert_eq!(placed, [at(5, 50), at(15, 50)]);
}

#[test]
fn a_buffer_past_the_branchs_horizontal_run_turns_toward_its_target() {
    let placed = buffers_along(
        TreePoint::new(0.0, 0.0),
        TreePoint::new(4.0, 10.0),
        vec![segment(14, 1, &[0.5], 7)],
    );

    assert_eq!(placed, [at(4, 3)]);
}

#[test]
fn every_sink_hangs_off_one_leaf_net_driven_by_a_buffer() {
    let scene = Scene::new(Vec::new()).with_buffer_size(1, 1);
    let mut tech =
        TechChar::from_segments(vec![segment(2, 1, &[0.5], 1), segment(1, 1, &[], 1)], 1);
    let mut clock = Clock::new(NetId::new(0), "clk", "clk");
    for (i, (x, y)) in [(10, 10), (90, 10), (10, 90), (90, 90)]
        .into_iter()
        .enumerate()
    {
        clock.add_sink(ClockSink {
            name: format!("ff{i}/CK"),
            location: at(x, y),
            pin: PinId::new(i),
            input_cap: Farads32::ZERO,
            insertion_delay: Seconds::ZERO,
        });
    }
    let mut tree = Builder::new(clock, None, tech.length_unit());

    tree.run(&scene.ctx(), &mut tech);

    let mut reached = Vec::new();
    for subnet in tree.clock.subnets.iter().filter(|s| s.leaf) {
        assert!(matches!(subnet.driver(), InstRef::Buffer(_)), "{subnet:?}");
        for &sink in subnet.sinks() {
            if let InstRef::Sink(i) = sink {
                reached.push(i);
            }
        }
    }
    reached.sort_unstable();
    assert_eq!(reached, [0, 1, 2, 3]);
}

#[test]
fn a_blockages_legal_locations_are_its_corners_and_where_the_branch_axes_cross_it() {
    let blockage = TreeBox {
        low: TreePoint::new(40.0, 40.0),
        high: TreePoint::new(60.0, 60.0),
    };

    let found = Builder::legal_locations(
        TreePoint::new(0.0, 50.0),
        TreePoint::new(50.0, 50.0),
        blockage,
    );

    let found: Vec<(f64, f64)> = found.iter().map(|p| (p.x, p.y)).collect();
    assert_eq!(
        found,
        [
            (40.0, 40.0),
            (40.0, 60.0),
            (60.0, 60.0),
            (60.0, 40.0),
            (50.0, 40.0),
            (50.0, 60.0),
            (40.0, 50.0),
            (60.0, 50.0),
            (40.0, 50.0),
            (60.0, 50.0),
        ]
    );
}

#[test]
fn a_point_moved_along_a_blockage_keeps_its_distance_and_nears_the_sinks() {
    let scene = Scene::new(vec![rect(40, 20, 70, 60)]);
    let blockage = TreeBox {
        low: TreePoint::new(40.0, 20.0),
        high: TreePoint::new(70.0, 60.0),
    };
    let ctx = scene.ctx();
    let sinks = [TreePoint::new(66.0, 18.0)];
    let search = LegalSearch {
        ctx: &ctx,
        target: 30.0,
        parent: TreePoint::new(50.0, 0.0),
        sinks: &sinks,
        direction: Direction::North,
    };

    let moved = builder().adjust_along_blockage(&search, TreePoint::new(45.0, 20.0), blockage);

    assert!(
        moved.is_some_and(|p| p.same(TreePoint::new(60.0, 20.0))),
        "{moved:?}"
    );
}

#[test]
fn a_north_branch_blocked_ahead_moves_along_the_blockage_edge_toward_its_sinks() {
    let scene = Scene::new(vec![rect(40, 20, 70, 60)]);
    let ctx = scene.ctx();
    let sinks = [TreePoint::new(62.0, 18.0)];
    let search = LegalSearch {
        ctx: &ctx,
        target: 30.0,
        parent: TreePoint::new(50.0, 0.0),
        sinks: &sinks,
        direction: Direction::North,
    };

    let moved = builder().adjust_beyond_blockage(&search, TreePoint::new(50.0, 30.0));

    assert!(moved.same(TreePoint::new(60.0, 20.0)), "{moved:?}");
}
