# Timing of a routed DEF, as OpenROAD-flow-scripts reports it at the end of its
# flow: parasitics extracted from the wires with the platform's RCX rules, then
# the worst and total negative slack, over all paths and per path group, so the
# paths a clock constrains can be read apart from the recovery checks of an
# unbuffered reset, and how many pins break their slew or load limit. Clocks
# stay ideal, as PARE builds no clock tree.
#
#   TIMING_LEF="tech.lef cells.lef" TIMING_LIB="cells.lib" TIMING_DEF=output/aes/routed.def \
#     TIMING_SDC=constraint.sdc TIMING_RCX=rcx_patterns.rules TIMING_DIR=cmp/aes \
#     nix develop .#eda -c openroad -no_init -no_splash -exit scripts/openroad_timing.tcl
#
# Writes the extracted parasitics to TIMING_DIR as SPEF and prints the slacks.

foreach lef $::env(TIMING_LEF) {
  read_lef $lef
}
foreach lib $::env(TIMING_LIB) {
  read_liberty $lib
}
read_def $::env(TIMING_DEF)
read_sdc $::env(TIMING_SDC)

define_process_corner -ext_model_index 0 X
extract_parasitics -ext_model_file $::env(TIMING_RCX)
set spef [file join $::env(TIMING_DIR) [file rootname [file tail $::env(TIMING_DEF)]].spef]
write_spef $spef
read_spef $spef

report_wns
report_tns
puts "drv max_slew [sta::max_slew_violation_count] max_cap [sta::max_capacitance_violation_count]"
report_worst_slack -max
report_worst_slack -min
report_clock_min_period

set clocks {}
foreach clock [all_clocks] {
  lappend clocks [get_name $clock]
}
foreach group [sta::path_group_names] {
  set wns 0.0
  set tns 0.0
  set failing 0
  set ends [find_timing_paths -path_delay max -path_group [list $group] \
    -group_path_count 100000000 -endpoint_path_count 1 -slack_max 0]
  foreach end $ends {
    set slack [$end slack]
    if {$slack < 0} {
      set tns [expr {$tns + $slack}]
      incr failing
      if {$slack < $wns} {
        set wns $slack
      }
    }
  }
  set kind [expr {$group in $clocks ? "clock" : "other"}]
  puts [format "group {%s} %s wns %.3f tns %.3f failing %d" $group $kind \
    [sta::time_sta_ui $wns] [sta::time_sta_ui $tns] $failing]
}
