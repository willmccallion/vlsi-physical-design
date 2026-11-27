use super::*;
use crate::clock::ClockSink;
use crate::options::CtsOptions;
use pare_common::db::indices::{NetId, PinId};
use pare_common::db::liberty::{LibCellId, LibPortId};
use pare_common::testing::{SeededRng, assert_golden};

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

/// A number in `0..n`, as a tree coordinate.
fn draw(rng: &mut SeededRng, n: u64) -> f64 {
    rng.below(n) as f64
}

/// A point at whole tree units in a 20-unit square, so that points fall on
/// edges, axes and each other often enough to tell `<` from `<=`.
fn grid_point(rng: &mut SeededRng) -> TreePoint {
    TreePoint::new(draw(rng, 21), draw(rng, 21))
}

/// The corners of a box of whole units inside the 20-unit square.
#[derive(Clone, Copy, Debug)]
struct GridBox([i64; 4]);

impl GridBox {
    fn random(rng: &mut SeededRng) -> Self {
        let whole = |v: u64| i64::try_from(v).unwrap();
        let (x1, y1) = (whole(rng.below(16)), whole(rng.below(16)));
        let (w, h) = (whole(1 + rng.below(5)), whole(1 + rng.below(5)));
        Self([x1, y1, x1 + w, y1 + h])
    }

    fn tree(self) -> TreeBox {
        let [x1, y1, x2, y2] = self.0.map(|v| v as f64);
        TreeBox {
            low: TreePoint::new(x1, y1),
            high: TreePoint::new(x2, y2),
        }
    }

    fn rect(self) -> Rect<Dbu> {
        let [x1, y1, x2, y2] = self.0;
        rect(x1, y1, x2, y2)
    }
}

const DIRECTIONS: [Direction; 4] = [
    Direction::North,
    Direction::East,
    Direction::South,
    Direction::West,
];

/// What the blockage geometry gives for seeded inputs on whole units.
fn blockage_geometry_text() -> String {
    let mut rng = SeededRng::new(17);
    let mut lines = Vec::new();
    for _ in 0..400 {
        let (parent, branch) = (grid_point(&mut rng), grid_point(&mut rng));
        let target = draw(&mut rng, 25);
        let blockage = GridBox::random(&mut rng).tree();
        let direction = rng.pick(&DIRECTIONS);
        let scale = rng.pick(&[0.0, 0.25, 0.5]);
        let margin = (draw(&mut rng, 4), draw(&mut rng, 4));
        lines.push(format!(
            "{parent:?} {branch:?} target {target} {blockage:?} {direction:?} scale {scale} margin {margin:?}"
        ));
        lines.push(format!(
            "  scale_position {:?}",
            scale_position(branch, parent, target, scale)
        ));
        lines.push(format!(
            "  along_blockage {:?}",
            along_blockage(parent, target, blockage, direction)
        ));
        lines.push(format!(
            "  beside_branch {:?}",
            beside_branch(parent, branch, target, margin, direction)
        ));
        lines.push(format!(
            "  legal_locations {:?}",
            Builder::legal_locations(parent, branch, blockage)
        ));
    }
    lines.join("\n") + "\n"
}

/// A seeded scene for a builder's blockage searches: blockages and taken
/// spots on whole units of a core 20 units a side, buffers a unit wide.
struct SearchCase {
    scene: Scene,
    builder: Builder,
    parent: TreePoint,
    point: TreePoint,
    sinks: Vec<TreePoint>,
    target: f64,
    direction: Direction,
}

impl SearchCase {
    fn random(rng: &mut SeededRng) -> Self {
        let blockages = (0..=rng.below(3))
            .map(|_| GridBox::random(rng).rect())
            .collect();
        let mut builder = builder();
        for _ in 0..rng.below(4) {
            builder.commit(grid_point(rng));
        }
        Self {
            scene: Scene::new(blockages).with_buffer_size(1, 1),
            builder,
            parent: grid_point(rng),
            point: grid_point(rng),
            sinks: (0..=rng.below(3)).map(|_| grid_point(rng)).collect(),
            target: draw(rng, 25),
            direction: rng.pick(&DIRECTIONS),
        }
    }

    fn ctx(&self) -> TreeContext<'_> {
        TreeContext {
            core: rect(0, 0, 20, 20),
            ..self.scene.ctx()
        }
    }

    /// The case's spots taken, as coordinates.
    fn taken(&self) -> Vec<(f64, f64)> {
        let mut taken: Vec<(f64, f64)> = self
            .builder
            .occupied
            .iter()
            .map(|&(x, y)| (f64::from_bits(x), f64::from_bits(y)))
            .collect();
        taken.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        taken
    }

    /// What each search gives in the case, one line each.
    fn describe(&self) -> Vec<String> {
        let (ctx, b, point) = (self.ctx(), &self.builder, self.point);
        let search = LegalSearch {
            ctx: &ctx,
            target: self.target,
            parent: self.parent,
            sinks: &self.sinks,
            direction: self.direction,
        };
        let blockage = b.find_blockage(&ctx, point);
        let mut lines = vec![
            format!(
                "blockages {:?} taken {:?} parent {:?} point {point:?} sinks {:?} target {} {:?}",
                self.scene.blockages,
                self.taken(),
                self.parent,
                self.sinks,
                self.target,
                self.direction
            ),
            format!("  find_blockage {blockage:?}"),
            format!("  legal_location {}", b.legal_location(&ctx, point)),
            format!(
                "  legalize_in_blockages {:?}",
                b.legalize_in_blockages(&ctx, point, buffer())
            ),
            format!(
                "  legalize_one_buffer {:?}",
                b.clone().legalize_one_buffer(&ctx, point, buffer())
            ),
            format!(
                "  adjust_beyond_blockage {:?}",
                b.adjust_beyond_blockage(&search, point)
            ),
            format!(
                "  weighted_distance {:?}",
                b.weighted_distance(point, self.parent, &self.sinks)
            ),
        ];
        if let Some(blockage) = blockage {
            let candidates = Builder::legal_locations(self.parent, point, blockage);
            lines.push(format!(
                "  adjust_along_blockage {:?}",
                b.adjust_along_blockage(&search, point, blockage)
            ));
            lines.push(format!(
                "  best_legal_location {:?}",
                b.best_legal_location(&search, point, &candidates, blockage)
            ));
        }
        lines
    }
}

/// What a builder's blockage searches give in seeded scenes.
fn blockage_search_text() -> String {
    let mut rng = SeededRng::new(29);
    let lines: Vec<String> = (0..150)
        .flat_map(|_| SearchCase::random(&mut rng).describe())
        .collect();
    lines.join("\n") + "\n"
}

fn golden_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}

#[test]
fn the_blockage_geometry_gives_what_it_always_has() {
    assert_golden(
        &golden_dir(),
        "htree_blockage_geometry.txt",
        &blockage_geometry_text(),
    );
}

#[test]
fn the_blockage_searches_give_what_they_always_have() {
    assert_golden(
        &golden_dir(),
        "htree_blockage_searches.txt",
        &blockage_search_text(),
    );
}

#[test]
fn no_search_leaves_a_buffer_inside_a_blockage() {
    let mut rng = SeededRng::new(31);
    for _ in 0..200 {
        let blockage = GridBox::random(&mut rng);
        let scene = Scene::new(vec![blockage.rect()]).with_buffer_size(1, 1);
        let ctx = TreeContext {
            core: rect(0, 0, 20, 20),
            ..scene.ctx()
        };
        let builder = builder();
        let point = grid_point(&mut rng);

        let moved = builder.legalize_in_blockages(&ctx, point, buffer());

        assert!(
            builder.find_blockage(&ctx, moved).is_none(),
            "{point:?} -> {moved:?} in {blockage:?}"
        );
        if builder.find_blockage(&ctx, point).is_none() {
            assert!(moved.same(point), "{point:?} -> {moved:?}");
        }
    }
}

/// A seeded tree to build: up to sixty sinks over a core 200 microns a
/// side, spread wide or packed close, on a grid so that they stack and line
/// up, some loading more than a sink buffer or delayed inside their cell;
/// the tree regular or over macros, its sinks clustered or not, under a
/// fanout limit or none, with a blockage or two now and then.
struct TreeCase {
    clock: Clock,
    tree_type: TreeType,
    settings: Settings,
    blockages: Vec<Rect<Dbu>>,
}

/// A whole number of `step`s below `count`, in database units.
fn steps(rng: &mut SeededRng, count: u64, step: i64) -> i64 {
    step * i64::try_from(rng.below(count)).unwrap()
}

fn random_tree_case(rng: &mut SeededRng) -> TreeCase {
    let mut clock = Clock::new(NetId::new(0), "clk", "clk");
    let (origin, step) = (steps(rng, 4, 80_000), rng.pick(&[200, 2000, 8000]));
    for i in 0..=rng.below(60) {
        let location = Point::new(
            Dbu::new(origin + steps(rng, 40, step)),
            Dbu::new(origin + steps(rng, 40, step)),
        );
        clock.add_sink(ClockSink {
            name: format!("ff{i}/CK"),
            location,
            pin: PinId::new(usize::try_from(i).unwrap()),
            input_cap: Farads32::of(pare_common::units::Farads::new(
                rng.pick(&[0.0, 1e-15, 4e-15]),
            )),
            insertion_delay: Seconds::new(rng.pick(&[0.0, 0.0, 2e-10])),
        });
    }
    let mut settings = Settings::new(CtsOptions {
        sink_clustering: rng.pick(&[
            SinkClusteringMode::Clustered,
            SinkClusteringMode::Unclustered,
        ]),
        ..CtsOptions::default()
    });
    settings.max_fanout = rng.pick(&[None, Some(4), Some(12)]);
    let blockages = (0..rng.below(3))
        .map(|_| {
            let (x, y) = (steps(rng, 16, 20_000), steps(rng, 16, 20_000));
            rect(
                x,
                y,
                x + 20_000 + steps(rng, 4, 20_000),
                y + 20_000 + steps(rng, 4, 20_000),
            )
        })
        .collect();
    TreeCase {
        clock,
        tree_type: rng.pick(&[TreeType::Regular, TreeType::Regular, TreeType::Macro]),
        settings,
        blockages,
    }
}

/// What building the tree of `case` gives: its buffers and its subnets.
fn built_tree_lines(case: TreeCase, tech: &TechChar, buffer: BufferCell) -> Vec<String> {
    let mut buffer_sizes = FxHashMap::default();
    let _ = buffer_sizes.insert(buffer, Point::new(Dbu::new(1900), Dbu::new(2800)));
    let ctx = TreeContext {
        settings: &case.settings,
        core: rect(0, 0, 400_000, 400_000),
        dbu: 2000,
        root_buffer: buffer,
        sink_buffer: buffer,
        buffer_sizes: &buffer_sizes,
        blockages: &case.blockages,
    };
    let sinks: Vec<String> = case
        .clock
        .sinks
        .iter()
        .map(|s| format!("({} {})", s.location.x.get(), s.location.y.get()))
        .collect();
    let mut lines = vec![format!(
        "{:?} {:?} fanout {:?} blockages {:?} sinks {}",
        case.tree_type,
        case.settings.sink_clustering,
        case.settings.max_fanout,
        case.blockages
            .iter()
            .map(|b| (b.min.x.get(), b.min.y.get(), b.max.x.get(), b.max.y.get()))
            .collect::<Vec<_>>(),
        sinks.join(" ")
    )];
    let mut tree = Builder::new(case.clock, None, tech.length_unit());
    tree.tree_type = case.tree_type;

    tree.run(&ctx, &mut tech.clone());

    for b in &tree.clock.buffers {
        lines.push(format!(
            "  {} at ({} {})",
            b.name,
            b.location.x.get(),
            b.location.y.get()
        ));
    }
    for s in &tree.clock.subnets {
        lines.push(format!(
            "  {} leaf {} level {:?}: {:?}",
            s.name, s.leaf, s.level, s.insts
        ));
    }
    lines
}

#[test]
fn seeded_sinks_get_the_trees_they_always_have() {
    let (tech, buffer) = crate::fixtures::clock_tech();
    let mut rng = SeededRng::new(53);
    let lines: Vec<String> = (0..60)
        .flat_map(|_| built_tree_lines(random_tree_case(&mut rng), &tech, buffer))
        .collect();

    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden");
    assert_golden(&dir, "htree_seeded_trees.txt", &(lines.join("\n") + "\n"));
}

#[test]
fn values_exactly_the_tolerance_apart_are_not_fuzzily_equal() {
    assert!(!fuzzy_eq(1e-6, 0.0, 1e-6));
    assert!(fuzzy_eq(0.5e-6, 0.0, 1e-6));
}
