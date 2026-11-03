//! The C++ library behaviour `OpenDP`'s region placement depends on:
//! `std::mt19937`, Boost's `uniform_int_distribution` over it, and
//! libstdc++'s `std::sort`, whose order among equal elements the search's
//! results follow.

/// The 32-bit Mersenne Twister, as `std::mt19937`.
#[derive(Clone, Debug)]
pub(super) struct Mt19937 {
    state: [u32; 624],
    index: usize,
}

impl Mt19937 {
    /// The generator seeded as `std::mt19937(seed)`.
    pub(super) fn new(seed: u32) -> Self {
        let mut state = [0u32; 624];
        state[0] = seed;
        for i in 1..624 {
            let previous = state[i - 1];
            state[i] = 1_812_433_253u32
                .wrapping_mul(previous ^ (previous >> 30))
                .wrapping_add(u32::try_from(i).unwrap_or(0));
        }
        Self { state, index: 624 }
    }

    /// The next 32-bit output.
    pub(super) fn next_u32(&mut self) -> u32 {
        if self.index >= 624 {
            self.twist();
        }
        let mut y = self.state[self.index];
        self.index += 1;
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c_5680;
        y ^= (y << 15) & 0xefc6_0000;
        y ^ (y >> 18)
    }

    fn twist(&mut self) {
        for i in 0..624 {
            let y = (self.state[i] & 0x8000_0000) | (self.state[(i + 1) % 624] & 0x7fff_ffff);
            let mut next = self.state[(i + 397) % 624] ^ (y >> 1);
            if y & 1 != 0 {
                next ^= 0x9908_b0df;
            }
            self.state[i] = next;
        }
        self.index = 0;
    }

    /// A uniform integer in `[low, high]`, as Boost's
    /// `uniform_int_distribution<int>(low, high)` draws it from a 32-bit
    /// engine: whole buckets of the engine's range per value, rejecting the
    /// remainder.
    pub(super) fn uniform_int(&mut self, low: i32, high: i32) -> i32 {
        let range = high.wrapping_sub(low).cast_unsigned();
        if range == 0 {
            return low;
        }
        if range == u32::MAX {
            return low.wrapping_add(self.next_u32().cast_signed());
        }
        let buckets = range + 1;
        let mut bucket_size = u32::MAX / buckets;
        if u32::MAX % buckets == range {
            bucket_size += 1;
        }
        loop {
            let result = self.next_u32() / bucket_size;
            if result <= range {
                return low.wrapping_add(result.cast_signed());
            }
        }
    }
}

/// Sorts as libstdc++'s `std::sort` does with `less`: introsort with a
/// median-of-three pivot, heap sort past twice the depth of a balanced
/// recursion, and a final insertion sort over runs of 16.
pub(super) fn sort<T: Copy>(v: &mut [T], less: &impl Fn(&T, &T) -> bool) {
    if v.len() < 2 {
        return;
    }
    let depth = 2 * (usize::BITS - 1 - v.len().leading_zeros()) as usize;
    introsort_loop(v, depth, less);
    final_insertion_sort(v, less);
}

const THRESHOLD: usize = 16;

fn introsort_loop<T: Copy>(v: &mut [T], mut depth: usize, less: &impl Fn(&T, &T) -> bool) {
    let mut last = v.len();
    while last > THRESHOLD {
        if depth == 0 {
            heap_sort(&mut v[..last], less);
            return;
        }
        depth -= 1;
        let cut = partition_pivot(&mut v[..last], less);
        introsort_loop(&mut v[cut..last], depth, less);
        last = cut;
    }
}

fn partition_pivot<T: Copy>(v: &mut [T], less: &impl Fn(&T, &T) -> bool) -> usize {
    let last = v.len();
    let mid = last / 2;
    move_median_to_first(v, 1, mid, last - 1, less);
    unguarded_partition(v, 1, last, less)
}

fn move_median_to_first<T: Copy>(
    v: &mut [T],
    a: usize,
    b: usize,
    c: usize,
    less: &impl Fn(&T, &T) -> bool,
) {
    let target = if less(&v[a], &v[b]) {
        if less(&v[b], &v[c]) {
            b
        } else if less(&v[a], &v[c]) {
            c
        } else {
            a
        }
    } else if less(&v[a], &v[c]) {
        a
    } else if less(&v[b], &v[c]) {
        c
    } else {
        b
    };
    v.swap(0, target);
}

/// Partitions `v[first..last]` around `v[0]`.
fn unguarded_partition<T: Copy>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    less: &impl Fn(&T, &T) -> bool,
) -> usize {
    loop {
        while less(&v[first], &v[0]) {
            first += 1;
        }
        last -= 1;
        while less(&v[0], &v[last]) {
            last -= 1;
        }
        if first >= last {
            return first;
        }
        v.swap(first, last);
        first += 1;
    }
}

fn final_insertion_sort<T: Copy>(v: &mut [T], less: &impl Fn(&T, &T) -> bool) {
    if v.len() > THRESHOLD {
        insertion_sort(&mut v[..THRESHOLD], less);
        for i in THRESHOLD..v.len() {
            unguarded_linear_insert(v, i, less);
        }
    } else {
        insertion_sort(v, less);
    }
}

fn insertion_sort<T: Copy>(v: &mut [T], less: &impl Fn(&T, &T) -> bool) {
    for i in 1..v.len() {
        if less(&v[i], &v[0]) {
            let value = v[i];
            v.copy_within(0..i, 1);
            v[0] = value;
        } else {
            unguarded_linear_insert(v, i, less);
        }
    }
}

fn unguarded_linear_insert<T: Copy>(v: &mut [T], mut last: usize, less: &impl Fn(&T, &T) -> bool) {
    let value = v[last];
    while less(&value, &v[last - 1]) {
        v[last] = v[last - 1];
        last -= 1;
    }
    v[last] = value;
}

/// `std::__partial_sort` over the whole slice: a heap, then popped.
fn heap_sort<T: Copy>(v: &mut [T], less: &impl Fn(&T, &T) -> bool) {
    let len = v.len();
    if len >= 2 {
        let mut parent = (len - 2) / 2;
        loop {
            let value = v[parent];
            adjust_heap(v, parent, len, value, less);
            if parent == 0 {
                break;
            }
            parent -= 1;
        }
    }
    let mut last = len;
    while last > 1 {
        last -= 1;
        let value = v[last];
        v[last] = v[0];
        adjust_heap(&mut v[..last], 0, last, value, less);
    }
}

fn adjust_heap<T: Copy>(
    v: &mut [T],
    mut hole: usize,
    len: usize,
    value: T,
    less: &impl Fn(&T, &T) -> bool,
) {
    let top = hole;
    let mut second = hole;
    while second < len.saturating_sub(1) / 2 {
        second = 2 * (second + 1);
        if less(&v[second], &v[second - 1]) {
            second -= 1;
        }
        v[hole] = v[second];
        hole = second;
    }
    if len & 1 == 0 && len >= 2 && second == (len - 2) / 2 {
        second = 2 * (second + 1);
        v[hole] = v[second - 1];
        hole = second - 1;
    }
    let mut parent = hole.saturating_sub(1) / 2;
    while hole > top && less(&v[parent], &value) {
        v[hole] = v[parent];
        hole = parent;
        parent = hole.saturating_sub(1) / 2;
    }
    v[hole] = value;
}

#[cfg(test)]
mod tests {
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
                727_152, 756_503, 768_669, 882_971, 269_265, 388_074, 644_174, 347_294, 93394,
                387_666, 79703, 520_558
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
                73, 24, 87, 31, 17, 80, 38, 10, 45, 94, 52, 3, 66, 59, 48, 62, 41, 76, 69, 34, 55,
                83, 27, 20, 97, 6, 90, 13, 86, 58, 65, 2, 51, 72, 9, 44, 93, 37, 23, 30, 16, 79,
                12, 26, 19, 54, 68, 5, 82, 61, 96, 47, 89, 40, 33, 75, 64, 99, 50, 85, 71, 78, 92,
                1, 57, 8, 15, 43, 22, 29, 36, 81, 4, 95, 11, 18, 88, 25, 32, 60, 39, 67, 74, 53,
                46, 49, 98, 56, 7, 0, 14, 91, 70, 35, 21, 42, 84, 28, 63, 77
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
}
