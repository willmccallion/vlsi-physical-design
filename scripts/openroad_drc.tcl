# OpenROAD design rule check of a routed DEF.
#
#   DRC_LEF=inputs/ibex/Nangate45.lef DRC_DEF=output/ibex/routed.def DRC_RPT=drc.rpt \
#     nix develop .#eda -c openroad -no_init -no_splash -exit scripts/openroad_drc.tcl
#
# The DEF must carry the GCELLGRID it was routed on, as PARE writes it.

read_lef $::env(DRC_LEF)
read_def $::env(DRC_DEF)
if {[[ord::get_db_block] getGCellGrid] == "NULL"} {
  puts "error: $::env(DRC_DEF) has no GCELLGRID"
  exit 1
}
drt::check_drc -output_file $::env(DRC_RPT)
set violations [llength [regexp -all -inline {violation type} [read [open $::env(DRC_RPT)]]]]
puts "DRC violations: $violations"
