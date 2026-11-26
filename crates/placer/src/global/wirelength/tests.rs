use super::*;

fn object(index: u32) -> (Terminal, Point<f64>) {
    point(Terminal::Object {
        index: ObjectId(index),
        offset: Point::new(0.0, 0.0),
    })
}

fn point(terminal: Terminal) -> (Terminal, Point<f64>) {
    (terminal, Point::new(0.0, 0.0))
}

#[test]
fn a_small_gamma_approaches_half_perimeter_wirelength() {
    let mut netlist = Netlist::default();
    netlist.push(
        [
            object(0),
            object(1),
            point(Terminal::Fixed(Point::new(5.0, 1.0))),
        ],
        1.0,
    );
    let centers = vec![Point::new(0.0, 0.0), Point::new(3.0, 4.0)].into();
    let mut grads = IdVec::filled(Point::new(0.0, 0.0), 2);

    let wl = netlist.gradient(&centers, 0.01, &mut Vec::new(), &mut grads);

    assert!((wl.hpwl - 9.0).abs() < 1e-12);
    assert!((wl.smooth - 9.0).abs() < 1e-6);
}

#[test]
fn net_weights_scale_the_smooth_length_but_not_the_half_perimeter() {
    let mut netlist = Netlist::default();
    netlist.push([object(0), object(1)], 3.0);
    let centers = vec![Point::new(0.0, 0.0), Point::new(3.0, 4.0)].into();
    let mut grads = IdVec::filled(Point::new(0.0, 0.0), 2);

    let wl = netlist.gradient(&centers, 0.01, &mut Vec::new(), &mut grads);

    assert!((wl.hpwl - 7.0).abs() < 1e-12);
    assert!((wl.smooth - 21.0).abs() < 1e-6);
}

#[test]
fn the_gradient_matches_a_finite_difference() {
    let mut netlist = Netlist::default();
    netlist.push([object(0), object(1), object(2)], 2.0);
    let centers: ObjectPoints = vec![
        Point::new(0.0, 1.0),
        Point::new(2.0, 3.0),
        Point::new(1.5, -1.0),
    ]
    .into();
    let mut grads = IdVec::filled(Point::new(0.0, 0.0), 3);
    let gamma = 0.7;
    let _ = netlist.gradient(&centers, gamma, &mut Vec::new(), &mut grads);

    let h = 1e-6;
    let third = ObjectId(2);
    let mut moved = centers;
    moved[third].x += h;
    let mut ignored = IdVec::filled(Point::new(0.0, 0.0), 3);
    let up = netlist.gradient(&moved, gamma, &mut Vec::new(), &mut ignored);
    moved[third].x -= h;
    moved[third].x -= h;
    let down = netlist.gradient(&moved, gamma, &mut Vec::new(), &mut ignored);

    let numeric = (up.smooth - down.smooth) / (2.0 * h);
    assert!(
        (grads[third].x - numeric).abs() < 1e-5,
        "{} vs {numeric}",
        grads[third].x
    );
}

#[test]
fn single_terminal_nets_are_dropped() {
    let mut netlist = Netlist::default();

    netlist.push([object(0)], 1.0);

    assert!(netlist.nets.is_empty() && netlist.terminals.is_empty());
}
