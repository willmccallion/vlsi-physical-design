use super::*;

#[test]
fn the_twister_gives_the_standard_ten_thousandth_output() {
    let mut generator = Mt19937::new(5489);
    let mut last = 0;
    for _ in 0..10_000 {
        last = generator.next_u32();
    }
    assert_eq!(last, 4_123_659_995);
}

#[test]
fn uniform_draws_match_boost_over_std_mt19937() {
    let mut generator = Mt19937::new(777);

    let small: Vec<i32> = (0..12).map(|_| generator.uniform_int(0, 36)).collect();
    let large: Vec<i32> = (0..12).map(|_| generator.uniform_int(0, 999_999)).collect();

    assert_eq!(small, [5, 23, 11, 7, 2, 23, 17, 9, 30, 20, 34, 22]);
    assert_eq!(
        large,
        [
            727_152, 756_503, 768_669, 882_971, 269_265, 388_074, 644_174, 347_294, 93394, 387_666,
            79703, 520_558
        ]
    );
}

#[test]
fn equal_keys_end_where_libstdcxx_puts_them() {
    let mut long: Vec<i32> = (0..100).collect();
    let mut short: Vec<i32> = (0..13).collect();

    sort(&mut long, &|a, b| (a * 37) % 7 > (b * 37) % 7);
    sort(&mut short, &|a, b| (a * 5) % 3 < (b * 5) % 3);

    assert_eq!(
        long,
        [
            73, 24, 87, 31, 17, 80, 38, 10, 45, 94, 52, 3, 66, 59, 48, 62, 41, 76, 69, 34, 55, 83,
            27, 20, 97, 6, 90, 13, 86, 58, 65, 2, 51, 72, 9, 44, 93, 37, 23, 30, 16, 79, 12, 26,
            19, 54, 68, 5, 82, 61, 96, 47, 89, 40, 33, 75, 64, 99, 50, 85, 71, 78, 92, 1, 57, 8,
            15, 43, 22, 29, 36, 81, 4, 95, 11, 18, 88, 25, 32, 60, 39, 67, 74, 53, 46, 49, 98, 56,
            7, 0, 14, 91, 70, 35, 21, 42, 84, 28, 63, 77
        ]
    );
    assert_eq!(short, [0, 3, 6, 9, 12, 2, 5, 8, 11, 1, 4, 7, 10]);
}

#[test]
fn sorting_orders_by_the_comparison() {
    let mut v: Vec<i32> = (0..200).map(|i| (i * 7919) % 101).collect();
    sort(&mut v, &|a, b| a < b);
    assert!(v.windows(2).all(|w| w[0] <= w[1]));
}
