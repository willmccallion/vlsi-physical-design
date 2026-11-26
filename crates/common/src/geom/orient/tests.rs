use super::*;

fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::from_corners(pt(x0, y0), pt(x1, y1))
}

const SIZE: Point<Dbu> = Point::new(Dbu::new(10), Dbu::new(4));
const AT: Point<Dbu> = Point::new(Dbu::new(100), Dbu::new(200));
const PIN: Point<Dbu> = Point::new(Dbu::new(1), Dbu::new(1));

#[test]
fn every_orientation_keeps_the_body_at_the_location() {
    for o in Orientation::ALL {
        let body = o.place_rect(&rect(0, 0, 10, 4), SIZE, AT);
        let size = o.oriented_size(SIZE);

        assert_eq!(body, Rect::new(AT, AT + size), "{o}");
    }
}

#[test]
fn pin_near_the_lower_left_corner_lands_where_each_orientation_puts_it() {
    let expected = [
        (Orientation::N, pt(101, 201)),
        (Orientation::S, pt(109, 203)),
        (Orientation::FN, pt(109, 201)),
        (Orientation::FS, pt(101, 203)),
        (Orientation::W, pt(103, 201)),
        (Orientation::E, pt(101, 209)),
        (Orientation::FW, pt(101, 201)),
        (Orientation::FE, pt(103, 209)),
    ];

    for (o, want) in expected {
        assert_eq!(o.place_point(PIN, SIZE, AT), want, "{o}");
    }
}

#[test]
fn orientations_parse_from_their_def_names() {
    for o in Orientation::ALL {
        assert_eq!(o.as_str().parse::<Orientation>(), Ok(o));
    }
    assert!("NE".parse::<Orientation>().is_err());
}
