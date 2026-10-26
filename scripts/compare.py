#!/usr/bin/env python3
"""Compare PARE with OpenROAD on a design, judge both results with both
checkers and OpenROAD's timer, and tabulate them.

Usage:
    python3 scripts/compare.py aes ibex           # route PARE's placement with both routers
    python3 scripts/compare.py --flow aes ibex    # PARE's whole run against OpenROAD-flow-scripts' flow
    python3 scripts/compare.py --table aes ibex   # tabulate earlier runs (with --flow, the flow runs)

Without --flow, the placement is output/<design>/placed.def, as `pare place`
or `pare run` writes it, and both routers route it. With --flow, PARE runs
its whole flow from the design's floorplan while OpenROAD-flow-scripts runs
its own from the same floorplan, stage by stage as its Makefile does, with
the I/O placement, repairs and clock tree it does on the way; where PARE's
input took the flow's I/O placement, the flow goes on from there. Everything
lands in output/compare/route/<design>/ or output/compare/flow/<design>/:
PARE's routed DEF and JSON report, OpenROAD's routed DEF and logs, both
check_drc reports, both `pare check` reports and both timing logs. OpenROAD
runs in the eda shell (`nix develop .#eda`), with the check_drc fix the
README describes. Timing needs the design's OpenROAD-flow-scripts
constraint, so it is reported for the designs that flow synthesized. The
table's slacks are those of the paths the clocks constrain, with ideal
clocks; summary.json has every path group, the recovery checks of an
unbuffered reset among them, and the flow's own final report.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ORFS = ROOT / "inputs/orfs/OpenROAD-flow-scripts-26Q2/flow"
PLATFORM = ORFS / "platforms/nangate45"
PARE = ROOT / "target/release/pare"
EDA_SHELL = ["nix", "develop", ".#eda", "-c"]
EDA_TOOLS = ("openroad", "make")
SIDES = ("pare", "openroad")
MODES = ("route", "flow")

ORFS_STAGES = (
    "3_1_place_gp_skip_io",
    "3_2_place_iop",
    "3_3_place_gp",
    "3_4_place_resized",
    "3_5_place_dp",
    "4_1_cts",
    "5_1_grt",
    "5_2_route",
)
ORFS_PIN_TARGETS = ["do-3_1_place_gp_skip_io", "do-3_2_place_iop"]
ORFS_TARGETS = [
    "do-3_3_place_gp",
    "do-3_4_place_resized",
    "do-3_5_place_dp",
    "do-3_place",
    "do-3_place.sdc",
    "do-4_1_cts",
    "do-4_cts",
    "do-5_1_grt",
    "do-5_2_route",
    "do-5_3_fillcell",
    "do-5_route",
    "do-5_route.sdc",
    "do-6_1_fill",
    "do-6_1_fill.sdc",
    "do-6_final.sdc",
    "do-6_report",
]


def design_dir(mode, design):
    return ROOT / "output/compare" / mode / design


def config_of(design):
    return tomllib.loads((ROOT / "configs/openroad" / f"{design}.toml").read_text())


def lef_files(design):
    return [str(ROOT / lef) for lef in config_of(design)["input"]["lef_files"]]


def orfs_design(design):
    """The ORFS design directory, when the flow synthesized this design."""
    def_file = config_of(design)["input"]["def_file"]
    if not def_file.endswith("_orfs.def"):
        return None
    return ORFS / "designs/nangate45" / design


def orfs_results(design):
    return ORFS / "results/nangate45" / design / "base"


def orfs_logs(design):
    return ORFS / "logs/nangate45" / design / "base"


def orfs_pins_taken(design):
    """Whether PARE's input took the flow's I/O placement, which the download
    keeps as the place_pin commands of that stage."""
    return (orfs_results(design) / "3_2_place_iop.tcl").exists()


def orfs_variable(design, name):
    """The value of `export NAME = ...` in the design's config.mk, with the
    flow's path variables resolved."""
    text = (orfs_design(design) / "config.mk").read_text()
    m = re.search(rf"^export {name}\s*=\s*(.+?)\s*$", text, re.MULTILINE)
    if not m:
        return None
    value = m.group(1)
    for variable, replacement in (
        ("$(DESIGN_HOME)", str(ORFS / "designs")),
        ("$(PLATFORM_DIR)", str(PLATFORM)),
        ("$(PLATFORM)", "nangate45"),
        ("$(DESIGN_NICKNAME)", design),
    ):
        value = value.replace(variable, replacement)
    return value


def liberty_files(design):
    """The platform's library and the design's additional ones."""
    libs = [str(PLATFORM / "lib/NangateOpenCellLibrary_typical.lib")]
    additional = orfs_variable(design, "ADDITIONAL_LIBS")
    if additional:
        libs.extend(additional.split())
    return libs


def routed_def(out, side):
    return out / ("routed.def" if side == "pare" else "openroad.def")


def run(cmd, log, env=None):
    """Runs `cmd` under /usr/bin/time -v with its output in `log`, in the eda
    shell when it is one of that shell's tools, with the variables of `env`
    set, or unset where the value is None."""
    full = ["/usr/bin/time", "-v", *cmd]
    if cmd[0] in EDA_TOOLS:
        full = [*EDA_SHELL, *full]
    environment = {**os.environ, **(env or {})}
    environment = {k: v for k, v in environment.items() if v is not None}
    with open(log, "w") as out:
        result = subprocess.run(
            full,
            cwd=ROOT,
            stdout=out,
            stderr=subprocess.STDOUT,
            env=environment,
            check=False,
        )
    if result.returncode != 0:
        print(
            f"  failed ({result.returncode}), see {log.relative_to(ROOT)}", flush=True
        )
    return result.returncode == 0


def pare_config(design, out):
    """The design's config with its outputs redirected into `out`."""
    config = config_of(design)
    config["input"]["output_dir"] = str(out)
    config["input"]["output_def"] = str(out / "placed.def")
    path = out / "pare.toml"
    path.write_text(toml_text(config))
    return path


def toml_value(value):
    """A config value as TOML: scalars and arrays as JSON writes them, and
    tables inline."""
    if isinstance(value, dict):
        return "{ " + ", ".join(f"{k} = {toml_value(v)}" for k, v in value.items()) + " }"
    if isinstance(value, list):
        return "[" + ", ".join(toml_value(v) for v in value) + "]"
    return json.dumps(value)


def toml_text(config):
    """The config as TOML: tables of keys, as `pare` reads them."""
    lines = [f"purpose = {json.dumps(config['purpose'])}", ""]
    for table, values in config.items():
        if not isinstance(values, dict):
            continue
        lines.append(f"[{table}]")
        for key, value in values.items():
            lines.append(f"{key} = {toml_value(value)}")
        lines.append("")
    return "\n".join(lines)


def route_with_pare(design, out):
    print(f"{design}: PARE routing", flush=True)
    return run(
        [
            str(PARE),
            "route",
            str(pare_config(design, out)),
            "--placement",
            f"output/{design}/placed.def",
            "--report",
            str(out / "pare.json"),
        ],
        out / "pare.log",
    )


def run_pare(design, out):
    print(f"{design}: PARE placing and routing", flush=True)
    return run(
        [
            str(PARE),
            "run",
            str(pare_config(design, out)),
            "--report",
            str(out / "pare.json"),
        ],
        out / "pare.log",
    )


def route_with_openroad(design, out):
    print(f"{design}: OpenROAD routing", flush=True)
    return run(
        [
            "openroad",
            "-threads",
            str(os.cpu_count()),
            "-no_init",
            "-no_splash",
            "-exit",
            "scripts/openroad_route.tcl",
        ],
        out / "openroad.log",
        env={
            "ROUTE_LEF": " ".join(lef_files(design)),
            "ROUTE_DEF": f"output/{design}/placed.def",
            "ROUTE_DIR": str(out),
        },
    )


def flow_with_openroad(design, out):
    """Runs the flow's stages from the floorplan to the routed design and its
    final report, then writes the routed design, before fill, as a DEF. The
    report's layout images render offscreen, as the flow does without a
    display, and without a GTK theme, whose initialization exits for want of
    one."""
    print(f"{design}: OpenROAD-flow-scripts placing and routing", flush=True)
    targets = (
        ORFS_TARGETS if orfs_pins_taken(design) else ORFS_PIN_TARGETS + ORFS_TARGETS
    )
    routed = orfs_results(design) / "5_2_route.odb"
    routed.unlink(missing_ok=True)
    reported = run(
        [
            "make",
            "-C",
            str(ORFS),
            f"DESIGN_CONFIG=./designs/nangate45/{design}/config.mk",
            *targets,
        ],
        out / "openroad.log",
        env={"QT_QPA_PLATFORM": "offscreen", "QT_QPA_PLATFORMTHEME": None},
    )
    if not routed.exists():
        return False
    written = run(
        ["openroad", "-no_init", "-no_splash", "-exit", "scripts/odb_to_def.tcl"],
        out / "openroad.def.log",
        env={"ODB_IN": str(routed), "DEF_OUT": str(routed_def(out, "openroad"))},
    )
    return reported and written


def check_drc(design, out, side):
    print(f"{design}: OpenROAD check_drc of {side}", flush=True)
    return run(
        ["openroad", "-no_init", "-no_splash", "-exit", "scripts/openroad_drc.tcl"],
        out / f"{side}.drc.log",
        env={
            "DRC_LEF": " ".join(lef_files(design)),
            "DRC_DEF": str(routed_def(out, side)),
            "DRC_RPT": str(out / f"{side}.drc.rpt"),
        },
    )


def pare_check(design, out, side):
    print(f"{design}: pare check of {side}", flush=True)
    return run(
        [
            str(PARE),
            "check",
            str(out / "pare.toml"),
            "--placement",
            str(routed_def(out, side)),
            "--report",
            str(out / f"{side}.check.json"),
        ],
        out / f"{side}.check.log",
    )


def timing(design, out, side, propagated):
    """Times the routed design; `propagated` for designs with clock trees."""
    if orfs_design(design) is None:
        return None
    print(f"{design}: timing of {side}", flush=True)
    env = {
        "TIMING_LEF": " ".join(lef_files(design)),
        "TIMING_LIB": " ".join(liberty_files(design)),
        "TIMING_DEF": str(routed_def(out, side)),
        "TIMING_SDC": orfs_variable(design, "SDC_FILE"),
        "TIMING_RCX": str(PLATFORM / "rcx_patterns.rules"),
        "TIMING_DIR": str(out),
    }
    if propagated:
        env["TIMING_PROPAGATED"] = "1"
    return run(
        ["openroad", "-no_init", "-no_splash", "-exit", "scripts/openroad_timing.tcl"],
        out / f"{side}.timing.log",
        env=env,
    )


def compare(mode, design, skip_openroad=False):
    out = design_dir(mode, design)
    out.mkdir(parents=True, exist_ok=True)
    if mode == "route":
        route_with_pare(design, out)
        if not skip_openroad:
            route_with_openroad(design, out)
    else:
        run_pare(design, out)
        if not skip_openroad:
            flow_with_openroad(design, out)
    for side in SIDES:
        if routed_def(out, side).exists():
            check_drc(design, out, side)
            pare_check(design, out, side)
            timing(design, out, side, propagated=mode == "flow")


def time_and_memory(log):
    """The wall time and peak memory /usr/bin/time -v wrote at the end of `log`."""
    text = log.read_text(errors="replace") if log.exists() else ""
    wall = re.search(r"Elapsed \(wall clock\) time .*: (\S+)", text)
    rss = re.search(r"Maximum resident set size \(kbytes\): (\d+)", text)
    return (wall.group(1) if wall else None, int(rss.group(1)) if rss else None)


def orfs_time_and_memory(design):
    """The wall time of the flow's stages from the floorplan to the routed
    design added up, written as /usr/bin/time writes it, and the largest peak
    memory among them."""
    seconds = 0.0
    peak_kb = 0
    for stage in ORFS_STAGES:
        log = orfs_logs(design) / f"{stage}.log"
        if not log.exists():
            return (None, None)
        last = None
        for last in re.finditer(
            r"Elapsed time: (\S+)\[h:\]min:sec.*Peak memory: (\d+)KB",
            log.read_text(errors="replace"),
        ):
            pass
        if last is None:
            return (None, None)
        parts = last.group(1).split(":")
        seconds += float(parts[-1]) + 60 * int(parts[-2])
        if len(parts) == 3:
            seconds += 3600 * int(parts[0])
        peak_kb = max(peak_kb, int(last.group(2)))
    hours, rest = divmod(seconds, 3600)
    minutes, rest = divmod(rest, 60)
    if hours:
        return (f"{int(hours)}:{int(minutes):02d}:{rest:02.0f}", peak_kb)
    return (f"{int(minutes)}:{rest:05.2f}", peak_kb)


def orfs_report(design):
    """What the flow's own final report gives: its slacks, with propagated
    clocks, and its cell count without fill cells."""
    metrics = orfs_logs(design) / "6_report.json"
    if not metrics.exists():
        return {}
    m = json.loads(metrics.read_text())
    return {
        "flow_report_wns_ns": m.get("finish__timing__setup__ws"),
        "flow_report_tns_ns": m.get("finish__timing__setup__tns"),
        "flow_report_cells": m.get("finish__design__instance__count"),
        "flow_report_max_slew_violations": m.get("finish__timing__drv__max_slew"),
        "flow_report_max_cap_violations": m.get("finish__timing__drv__max_cap"),
    }


def cells(def_file):
    with open(def_file, errors="replace") as f:
        for line in f:
            m = re.match(r"COMPONENTS (\d+) ;", line)
            if m:
                return int(m.group(1))
    return None


def result(mode, design, side):
    out = design_dir(mode, design)
    r = {"mode": mode, "design": design, "router": side}
    if mode == "flow" and side == "openroad":
        r["wall"], r["rss_kb"] = orfs_time_and_memory(design)
        r.update(orfs_report(design))
    else:
        r["wall"], r["rss_kb"] = time_and_memory(out / f"{side}.log")
    report = out / "pare.json"
    if side == "pare" and report.exists():
        netlist = json.loads(report.read_text()).get("netlist")
        if netlist:
            r["netlist"] = netlist
    def_file = routed_def(out, side)
    if def_file.exists():
        r["cells"] = cells(def_file)
    check = out / f"{side}.check.json"
    if check.exists():
        c = json.loads(check.read_text())["checks"]
        r["wirelength_um"] = c["wirelength_um"]
        r["vias"] = c["vias"]
        r["pare_violations"] = sum(c["violations"].values())
        r["opens"] = c["opens"]
    drc = out / f"{side}.drc.log"
    if drc.exists():
        m = re.search(r"DRC violations: (\d+)", drc.read_text(errors="replace"))
        r["openroad_violations"] = int(m.group(1)) if m else None
    timing_log = out / f"{side}.timing.log"
    if timing_log.exists():
        r.update(slacks(timing_log.read_text(errors="replace")))
    return r


def slacks(timing_log):
    """The worst and total negative slack over all paths, per path group, and
    over the groups of the clocks alone."""
    r = {}
    for key, pattern in (
        ("wns_all_ns", r"wns max (-?[\d.]+)"),
        ("tns_all_ns", r"tns max (-?[\d.]+)"),
    ):
        m = re.search(pattern, timing_log)
        r[key] = float(m.group(1)) if m else None
    groups = {}
    for m in re.finditer(
        r"^group \{(.+)\} (clock|other) wns (-?[\d.]+) tns (-?[\d.]+) failing (\d+)$",
        timing_log,
        re.MULTILINE,
    ):
        groups[m.group(1)] = {
            "clock": m.group(2) == "clock",
            "wns_ns": float(m.group(3)),
            "tns_ns": float(m.group(4)),
            "failing": int(m.group(5)),
        }
    r["groups"] = groups
    drv = re.search(r"^drv max_slew (\d+) max_cap (\d+)$", timing_log, re.MULTILINE)
    if drv:
        r["max_slew_violations"] = int(drv.group(1))
        r["max_cap_violations"] = int(drv.group(2))
    clocked = [g for g in groups.values() if g["clock"]]
    if clocked:
        r["wns_ns"] = min(g["wns_ns"] for g in clocked)
        r["tns_ns"] = sum(g["tns_ns"] for g in clocked)
    return r


def cell(value, fmt="{}"):
    return "?" if value is None else fmt.format(value)


def table(results):
    lines = [
        "| Design | Tool | Cells | Wirelength | Vias | PARE violations | OpenROAD `check_drc` | Slew / cap violations | WNS | TNS | Time | Peak memory |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in results:
        pare_check = cell(r.get("pare_violations"))
        if r.get("opens"):
            pare_check += f" + {r['opens']} opens"
        lines.append(
            f"| {r['design']} | {'PARE' if r['router'] == 'pare' else 'OpenROAD'} "
            f"| {cell(r.get('cells'), '{:,}')} "
            f"| {cell(r.get('wirelength_um'), '{:,.0f} µm')} | {cell(r.get('vias'), '{:,}')} "
            f"| {pare_check} | {cell(r.get('openroad_violations'))} "
            f"| {cell(r.get('max_slew_violations'))} / {cell(r.get('max_cap_violations'))} "
            f"| {cell(r.get('wns_ns'), '{:.2f} ns')} | {cell(r.get('tns_ns'), '{:.1f} ns')} "
            f"| {cell(r.get('wall'))} | {cell(None if r.get('rss_kb') is None else r['rss_kb'] / 2**20, '{:.1f} GiB')} |"
        )
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "designs",
        nargs="+",
        help="designs with a placement in output/<design>/placed.def, or with --flow a floorplan",
    )
    parser.add_argument(
        "--flow",
        action="store_true",
        help="compare whole flows from the floorplan instead of routers on one placement",
    )
    parser.add_argument(
        "--table", action="store_true", help="only tabulate earlier runs"
    )
    parser.add_argument(
        "--skip-openroad",
        action="store_true",
        help="keep OpenROAD's earlier result for each design and run only PARE, then judge both",
    )
    args = parser.parse_args()
    mode = "flow" if args.flow else "route"
    if not args.table:
        if not PARE.exists():
            sys.exit(
                f"{PARE.relative_to(ROOT)} is missing: cargo build --release first"
            )
        for design in args.designs:
            compare(mode, design, args.skip_openroad)
    results = [result(mode, d, side) for d in args.designs for side in SIDES]
    summary = ROOT / "output/compare" / mode / "summary.json"
    summary.parent.mkdir(parents=True, exist_ok=True)
    summary.write_text(json.dumps(results, indent=2))
    print(table(results))


if __name__ == "__main__":
    main()
