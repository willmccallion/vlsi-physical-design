use super::*;

#[test]
fn a_memory_line_of_a_status_file_is_read_in_kilobytes() {
    let status = "Name:\tpare\nVmPeak:\t  123456 kB\nVmHWM:\t   98304 kB\nVmRSS:\t 1000 kB\n";

    assert_eq!(field_kb(status, "VmHWM:"), Some(98_304));
    assert_eq!(field_kb(status, "VmRSS:"), Some(1000));
    assert_eq!(field_kb("Name:\tpare\n", "VmHWM:"), None);
}
