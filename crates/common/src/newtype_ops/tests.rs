#[derive(Clone, Copy, Debug, PartialEq)]
struct Count(i64);
crate::newtype_ops!(Count: Add, AddAssign, Sub, SubAssign, Neg, Sum, Add<i64>, Sub<i64>, Mul<i64>, Div<i64>);

#[derive(Clone, Copy, Debug, PartialEq)]
struct Charge(f32);
crate::newtype_ops!(Charge: Add, Sum);

#[test]
fn operators_between_values_act_on_the_field() {
    let mut c = Count(7);
    c += Count(5);
    c -= Count(2);

    assert_eq!(
        (c, Count(7) + Count(5), Count(7) - Count(5), -Count(7)),
        (Count(10), Count(12), Count(2), Count(-7))
    );
}

#[test]
fn operators_with_a_scalar_act_on_the_field() {
    let c = Count(7);

    assert_eq!(
        (c + 2, c - 2, c * 3, c / 2),
        (Count(9), Count(5), Count(21), Count(3))
    );
}

#[test]
fn a_sum_adds_the_fields() {
    let total: Count = [Count(1), Count(2), Count(3)].into_iter().sum();
    let none: Count = std::iter::empty().sum();
    let charge: Charge = [Charge(0.5), Charge(0.25)].into_iter().sum();

    assert_eq!((total, none, charge), (Count(6), Count(0), Charge(0.75)));
}
