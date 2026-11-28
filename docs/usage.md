# Running PARE

**Requirements:** Rust 1.98.1, the version the flake's dev shell provides and `rust-toolchain.toml` pins, and Python 3. `nix develop .#eda` provides OpenROAD, with the `check_drc` fix above, for cross-checking results, and the yosys that OpenROAD-flow-scripts 26Q2 pins; aes, ibex, jpeg, ariane133 and bp_quad are synthesized and floorplanned with them, so download those inside it.

Benchmarks are downloaded by `scripts/download.py`, which also writes each design's config to `configs/<suite>/<design>.toml`. Each design has a purpose that decides what `pare run` does with it: placement benchmarks are only placed, routing benchmarks (whose placement is part of the input) are only routed, and the rest are placed and then routed.

| Suite | Designs | Purpose |
|---|---|---|
| `openroad` | gcd | routing |
| `openroad` | aes, ibex, jpeg, ariane133, bp_quad (synthesized and floorplanned by OpenROAD-flow-scripts) | place and route |
| `openroad` | swerv, netcard, leon3mp (OpenROAD placer tests) | place and route |
| `ibm` | ibm01–ibm18 (Bookshelf) | placement |
| `ispd2015` | 20 `mgc_*` designs, up to 1.3M nets | place and route; none routed clean yet, see [Known issues](results.md#known-issues) |
| `ispd2018`, `ispd2019` | `ispd18_test1`–`10`, `ispd19_test1`–`10` | routing; none routed clean yet, see [Known issues](results.md#known-issues) |

```bash
python3 scripts/download.py --list          # every design and its purpose
python3 scripts/download.py gcd aes ibex    # designs, a suite (e.g. ispd2018), or all

cargo run --release -- run aes              # does what the design's purpose says
cargo run --release -- place aes            # placement only (refused for routing benchmarks)
cargo run --release -- route gcd            # routing only (refused for placement benchmarks)
cargo run --release -- check aes --placement output/aes/routed.def

python3 scripts/compare.py aes ibex           # route the PARE placement with PARE and with OpenROAD, check and time both
python3 scripts/compare.py --flow aes ibex    # PARE's whole run against OpenROAD-flow-scripts' whole flow from the floorplan
python3 scripts/compare.py --flow --skip-openroad aes   # rerun PARE's side only, against the flow's earlier result
python3 scripts/net_report.py ibex rst_ni     # one net in both routings: wire per layer, vias, Elmore delay to its loads
```

The placed and routed DEFs and images of each stage are written to the output directory named in the config. A config that names `liberty_files` under `[input]` gets the netlist repair of [Netlist repair and timing](design.md#netlist-repair-and-timing), with the cells not to use and the signal wire's layer and RC (`signal_wire`) under `[netlist]`; one that names an `sdc_file` too is timed, placed timing-driven and has its setup paths repaired, with `timing_driven` under `[global_placement]` and `repair_timing` under `[netlist]` to turn either off. `scripts/download.py` writes them for the OpenROAD-flow-scripts designs from the platform's settings and the flow's constraint files.

## Looking inside a run

```bash
# Save the routing state after global routing, or after detailed round N, and resume from it later
cargo run --release -- route aes --placement output/aes/placed.def --save-state aes.state --save-after 20
cargo run --release -- route aes --placement output/aes/placed.def --resume aes.state

# Log every route found for some nets, or in every region meeting a box (in microns), with its cost part by part
cargo run --release -- route aes --placement output/aes/placed.def --trace-net _16807_ --trace-box 400 289 410 298

# Draw one area at its true geometry, chosen nets highlighted and violations outlined and listed
cargo run --release -- view aes --placement output/aes/routed.def --box 403 292 406.5 294.5 \
    --net _16807_ --layer metal2 --layer metal3 --out pair.png
```
