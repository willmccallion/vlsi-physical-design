# Writes the OpenROAD database ODB_IN as the DEF DEF_OUT, with the I/O pins
# the place_pin commands of PINS_TCL place, when it is set.
read_db $::env(ODB_IN)
if {[info exists ::env(PINS_TCL)]} {
  source $::env(PINS_TCL)
}
write_def $::env(DEF_OUT)
