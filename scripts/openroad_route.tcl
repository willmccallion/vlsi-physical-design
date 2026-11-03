# OpenROAD global and detailed routing of a placed DEF, as OpenROAD-flow-scripts
# 26Q2 routes a Nangate45 design: its layer adjustments and routing layers,
# its global_route and detailed_route arguments, 64 detailed iterations. The
# repairs its flow runs between the two (resizing, antenna diodes) change the
# netlist and are left out, so the result routes exactly the given placement.
#
#   ROUTE_LEF="tech.lef cells.lef" ROUTE_DEF=output/aes/placed.def ROUTE_DIR=cmp/aes \
#     nix develop .#eda -c openroad -threads 28 -no_init -no_splash -exit scripts/openroad_route.tcl
#
# Writes ROUTE_DIR/openroad.def, with the gcell grid it was routed on, and the
# router's congestion, DRC and maze reports beside it.

foreach lef $::env(ROUTE_LEF) {
  read_lef $lef
}
read_def $::env(ROUTE_DEF)
set dir $::env(ROUTE_DIR)

set_global_routing_layer_adjustment metal2-metal3 0.5
set_global_routing_layer_adjustment metal4-metal10 0.25
set_routing_layers -clock metal4-metal10
set_routing_layers -signal metal2-metal10

pin_access
global_route -congestion_report_file $dir/congestion.rpt \
  -congestion_iterations 30 -congestion_report_iter_step 5 -verbose
detailed_route -output_drc $dir/drt.rpt -output_maze $dir/maze.log \
  -droute_end_iter 64 -verbose 1 -drc_report_iter_step 5
write_def $dir/openroad.def
