use super::*;

#[test]
fn design_repair_reports_as_fields_of_its_summary() {
    let summary = NetlistSummary {
        buffers_removed: 1,
        design: DesignRepairSummary {
            slew_violations: 2,
            repeaters: 3,
            ..DesignRepairSummary::default()
        },
        ..NetlistSummary::default()
    };

    let json = serde_json::to_string(&summary).unwrap();

    assert_eq!(
        json,
        "{\"buffers_removed\":1,\"input_buffers\":0,\"output_buffers\":0,\"buffer_cell\":\"\",\
             \"slew_violations\":2,\"cap_violations\":0,\"fanout_violations\":0,\"repaired_nets\":0,\"resized\":0,\
             \"repeaters\":3}"
    );
}

/// The keys of a report `scripts/compare.py` reads, by path.
const COMPARED_KEYS: [&str; 5] = [
    "/netlist",
    "/checks/wirelength_um",
    "/checks/vias",
    "/checks/violations",
    "/checks/opens",
];

#[test]
fn a_report_has_the_keys_compare_reads() {
    let report = Report {
        netlist: Some(NetlistSummary::default()),
        ..Report::default()
    };

    let json = serde_json::to_value(&report).unwrap();

    for key in COMPARED_KEYS {
        assert!(json.pointer(key).is_some(), "no {key} in {json}");
    }
}

#[test]
fn a_time_reports_in_nanoseconds() {
    let ns = nanos(Some(Seconds::new(2.5e-9))).unwrap();

    assert!((ns - 2.5).abs() < 1e-12, "{ns}");
}

#[test]
fn no_time_reports_as_none() {
    assert_eq!(nanos(None), None);
}

#[test]
fn wall_times_report_as_seconds() {
    let report = Report {
        phases: vec![PhaseTime {
            name: "routing",
            elapsed: Duration::from_millis(1500),
        }],
        wall: Duration::from_millis(2250),
        ..Report::default()
    };

    let json = serde_json::to_value(&report).unwrap();

    assert_eq!(json["phases"][0]["seconds"], 1.5);
    assert_eq!(json["wall_seconds"], 2.25);
}
