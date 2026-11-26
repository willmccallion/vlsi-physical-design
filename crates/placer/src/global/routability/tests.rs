use super::*;
use crate::global::ObjectId;
use crate::global::density::DensityGrid;
use crate::global::wirelength::Terminal;
use crate::zones::Zone;

fn die(side: f64) -> Rect<f64> {
    Rect::new(Point::new(0.0, 0.0), Point::new(side, side))
}

/// A 4x4 grid of unit gcells of 10 tracks each, `taken` of them taken
/// before placement, with wires 0.1 wide.
fn params(taken: &[f64]) -> RoutabilityParams {
    RoutabilityParams {
        tiles: RoutingTiles {
            origin: Point::new(0.0, 0.0),
            size: 1.0,
            nx: 4,
            ny: 4,
            capacity: vec![10.0; 16],
            reduction: taken.to_vec(),
        },
        wire_width: 0.1,
        target_congestion: 1.0,
    }
}

fn map_of(congestion: Vec<f64>) -> CongestionMap {
    let n = congestion.len();
    CongestionMap {
        origin: Point::new(0.0, 0.0),
        size: 1.0,
        nx: 4,
        ny: n / 4,
        congestion,
    }
}

/// Standard cells of unit size at `centers`.
fn cells_at(centers: &[Point<f64>]) -> Objects {
    Objects {
        centers: centers.to_vec().into(),
        sizes: IdVec::filled(Point::new(1.0, 1.0), centers.len()),
        kinds: IdVec::filled(CellKind::Standard, centers.len()),
    }
}

/// A pin at `at` whose average point is `average` away from it.
fn pin(at: Point<f64>, average: Point<f64>) -> (Terminal, Point<f64>) {
    (Terminal::Fixed(at), average)
}

fn at(x: f64, y: f64) -> (Terminal, Point<f64>) {
    pin(Point::new(x, y), Point::new(0.0, 0.0))
}

fn two_pin(a: (Terminal, Point<f64>), b: (Terminal, Point<f64>)) -> Netlist {
    let mut netlist = Netlist::default();
    netlist.push([a, b], 1.0);
    netlist
}

fn field(cells: std::ops::Range<u32>, fillers: std::ops::Range<u32>) -> Field {
    let range = |r: std::ops::Range<u32>| ObjectRange::new(ObjectId(r.start), ObjectId(r.end));
    Field {
        zone: Zone::Outside,
        grid: DensityGrid::new(&die(4.0), 4, 0.5, &[die(4.0)], &[]),
        area: vec![die(4.0)],
        bounds: die(4.0),
        cells: range(cells),
        kept_fillers: fillers.len(),
        fillers: range(fillers),
        filler: Point::new(1.0, 1.0),
        utilization: 0.125,
    }
}

#[test]
fn a_net_spreads_its_wire_area_evenly_over_its_box() {
    let netlist = two_pin(at(0.5, 1.25), at(3.5, 1.75));

    let map = CongestionMap::rudy(&netlist, &IdVec::new(), &die(4.0), &params(&[0.0; 16]));

    let density = (3.0 + 0.5) * 0.1 / (3.0 * 0.5);
    let half = density * 0.5;
    let quarter = density * 0.25;
    assert!(
        (map.congestion[5] - half).abs() < 1e-12,
        "{:?}",
        map.congestion
    );
    assert!(
        (map.congestion[4] - quarter).abs() < 1e-12,
        "{:?}",
        map.congestion
    );
    assert!(map.congestion[..4].iter().all(|&c| c == 0.0));
}

#[test]
fn a_net_whose_pin_points_line_up_lays_no_wire() {
    let netlist = two_pin(at(0.5, 1.5), at(2.5, 1.5));

    let map = CongestionMap::rudy(&netlist, &IdVec::new(), &die(4.0), &params(&[0.0; 16]));

    assert!(
        map.congestion.iter().all(|&c| c == 0.0),
        "{:?}",
        map.congestion
    );
}

#[test]
fn a_nets_box_spans_its_pins_average_points() {
    let netlist = two_pin(
        at(0.5, 1.5),
        pin(Point::new(2.5, 1.5), Point::new(0.0, 0.5)),
    );

    let map = CongestionMap::rudy(&netlist, &IdVec::new(), &die(4.0), &params(&[0.0; 16]));

    let wire_area = (2.0 + 0.5) * 0.1;
    let laid: f64 = map.congestion.iter().sum();
    assert!((laid - wire_area).abs() < 1e-12, "{:?}", map.congestion);
    assert!(map.congestion[4..7].iter().all(|&c| c > 0.0));
}

#[test]
fn capacity_taken_before_placement_adds_its_share() {
    let mut taken = vec![0.0; 16];
    taken[0] = 5.0;
    let netlist = two_pin(at(0.0, 0.0), at(4.0, 4.0));

    let map = CongestionMap::rudy(&netlist, &IdVec::new(), &die(4.0), &params(&taken));

    let wire = 8.0 * 0.1 / 16.0;
    assert!(
        (map.congestion[0] - (0.5 + wire)).abs() < 1e-12,
        "{:?}",
        map.congestion
    );
    assert!(
        (map.congestion[2] - wire).abs() < 1e-12,
        "{:?}",
        map.congestion
    );
}

#[test]
fn a_gcell_without_capacity_counts_only_its_wire() {
    let mut params = params(&[3.0; 16]);
    params.tiles.capacity[3] = 0.0;

    let map = CongestionMap::rudy(&Netlist::default(), &IdVec::new(), &die(4.0), &params);

    assert!(map.congestion[3].abs() < 1e-12);
    assert!((map.congestion[2] - 0.3).abs() < 1e-12);
}

#[test]
fn a_gcell_the_die_cuts_short_spreads_wire_over_its_own_area() {
    let netlist = two_pin(at(3.0, 0.0), at(3.5, 1.0));

    let map = CongestionMap::rudy(&netlist, &IdVec::new(), &die(3.5), &params(&[0.0; 16]));

    let density = (0.5 + 1.0) * 0.1 / 0.5;
    assert!(
        (map.congestion[3] - density).abs() < 1e-12,
        "{:?}",
        map.congestion
    );
}

#[test]
fn the_metric_averages_the_most_congested_gcells() {
    let mut congestion = vec![0.0; 400];
    congestion[0] = 3.0;
    congestion[1] = 1.0;
    congestion[2] = 1.0;
    congestion[3] = 1.0;

    assert!((map_of(congestion).metric() - f64::midpoint(2.0, 1.5)).abs() < 1e-12);
}

#[test]
fn measured_congestion_is_taken_as_given() {
    let params = params(&[0.0; 16]);
    let mut congestion = vec![0.0; 16];
    congestion[0] = 2.0;

    let map = CongestionMap::measured(congestion, &params);

    assert!((map.inflation_at(Point::new(0.5, 0.5)) - MAX_INFLATION).abs() < 1e-12);
    assert!((map.inflation_at(Point::new(1.5, 1.5)) - 1.0).abs() < 1e-12);
}

#[test]
fn cells_grow_by_their_gcells_squared_congestion_from_the_threshold_up_to_a_cap() {
    let map = map_of(vec![std::f64::consts::SQRT_2, 9.0, 1.0, 1.01]);

    let growth = [0.5, 1.5, 2.5, 3.5].map(|x| map.inflation_at(Point::new(x, 0.5)));

    assert!((growth[0] - 2.0).abs() < 1e-12);
    assert!((growth[1] - 3.0).abs() < 1e-12);
    assert!((growth[2] - 1.0).abs() < 1e-12);
    assert!((growth[3] - 1.0201).abs() < 1e-12);
}

#[test]
fn growth_removes_whole_fillers_from_the_last_kept() {
    let mut objects = cells_at(&[Point::new(0.5, 0.5), Point::new(0.4, 0.6)]);
    for _ in 0..3 {
        let _ = objects.centers.push(Point::new(2.0, 2.0));
        let _ = objects.sizes.push(Point::new(1.0, 1.0));
    }
    let mut congestion = vec![0.0; 16];
    congestion[0] = 1.5;
    let mut f = field(0..2, 2..5);

    let (inflated, added) = grow(&map_of(congestion), &mut objects, &f, 1.0);
    remove_fillers(&objects, &mut f, added);

    assert_eq!(inflated, 2);
    assert!((added - 2.5).abs() < 1e-9);
    assert_eq!(f.kept_fillers, 1);
    assert!((f.grid.target() - 0.5).abs() < 1e-12);
}

#[test]
fn growth_past_the_fillers_raises_the_target_to_cover_the_cells() {
    let objects = Objects {
        centers: vec![Point::new(1.0, 1.0)].into(),
        sizes: vec![Point::new(3.0, 3.0)].into(),
        kinds: vec![CellKind::Standard].into(),
    };
    let mut f = field(0..1, 1..1);

    remove_fillers(&objects, &mut f, 5.0);

    assert!((f.grid.target() - 9.0 / 16.0).abs() < 1e-12);
}

#[test]
fn a_round_takes_at_most_a_sixth_of_the_growth_budget() {
    let objects = cells_at(&[Point::new(0.5, 0.5); 10]);
    let routability = Routability::new(
        params(&[0.0; 16]),
        &objects,
        &vec![field(0..10, 10..10)].into(),
    );
    let mut congestion = vec![0.0; 16];
    congestion[0] = 9.0;

    let scale = routability.budget_scale(&map_of(congestion), &objects);

    let allowed = 0.30 * 10.0 / 6.0;
    assert!((scale.scale - allowed / 20.0).abs() < 1e-12);
    assert!(!scale.spent);
}

#[test]
fn congestion_below_the_target_keeps_the_sizes_and_ends_routability() {
    let mut objects = cells_at(&[Point::new(0.5, 0.5)]);
    let mut fields = vec![field(0..1, 1..1)].into();
    let mut routability = Routability::new(params(&[0.0; 16]), &objects, &fields);

    let verdict = routability.check(0, &map_of(vec![0.5; 16]), &mut objects, &mut fields);

    assert_eq!(verdict, Verdict::Done);
    assert!(!routability.active());
}

#[test]
fn three_rounds_without_improvement_revert_to_the_least_congested_sizes() {
    let mut objects = cells_at(&[Point::new(0.5, 0.5)]);
    let mut fields = vec![field(0..1, 1..1)].into();
    let mut routability = Routability::new(params(&[0.0; 16]), &objects, &fields);
    let mut congestion = vec![0.0; 16];
    congestion[0] = 1.2;

    let verdicts: Vec<Verdict> = (0..4)
        .map(|k| routability.check(k, &map_of(congestion.clone()), &mut objects, &mut fields))
        .collect();

    assert_eq!(
        verdicts,
        [
            Verdict::Resume,
            Verdict::Resume,
            Verdict::Resume,
            Verdict::Revert
        ]
    );
    assert_eq!(objects.sizes[ObjectId(0)], Point::new(1.0, 1.0));
    assert!((fields[FieldId(0)].grid.target() - 0.5).abs() < 1e-12);
}
