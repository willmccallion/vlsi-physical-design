# Writes the OpenROAD database ODB_IN as the DEF DEF_OUT.
read_db $::env(ODB_IN)
write_def $::env(DEF_OUT)
