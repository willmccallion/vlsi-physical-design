use super::*;

#[test]
fn a_length_in_tree_units_is_its_database_units_over_the_unit() {
    let units = Microns::new(7.0).in_tree_units(2000, 4000.0);

    assert!((units - 3.5).abs() < 1e-6, "{units}");
}

#[test]
fn a_length_in_single_precision_tree_units_is_its_database_units_over_the_unit() {
    let units = Microns::new(7.0).in_tree_units_single(2000.0, 4000.0);

    assert!((units - 3.5).abs() < 1e-6, "{units}");
}

/// The default settings with the cluster sizes capped at `limit`.
fn sizes_limited_to(limit: u32) -> Vec<u32> {
    let mut settings = Settings::new(CtsOptions::default());
    settings.limit_sink_clustering_sizes(limit);
    settings.sink_clustering_sizes
}

#[test]
fn a_fanout_limit_between_cluster_sizes_replaces_the_larger_ones() {
    assert_eq!(sizes_limited_to(25), [10, 20, 25]);
}

#[test]
fn a_fanout_limit_equal_to_a_cluster_size_ends_the_sizes_once() {
    assert_eq!(sizes_limited_to(20), [10, 20]);
}
