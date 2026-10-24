#!/usr/bin/env python3
"""Route a design's PARE placement with PARE and with OpenROAD, judge both
results with both checkers and OpenROAD's timer, and tabulate them.

Usage:
    python3 scripts/compare.py aes ibex          # route with both, judge, tabulate
    python3 scripts/compare.py --table aes ibex  # tabulate earlier runs

The placement is output/<design>/placed.def, as `pare place` or `pare run`
writes it. Everything lands in output/compare/<design>/: PARE's routed DEF
and JSON report, OpenROAD's routed DEF and logs, both check_drc reports,
both `pare check` reports and both timing logs. OpenROAD runs in the eda
shell (`nix develop .#eda`), with the check_drc fix the README describes.
Timing needs the design's OpenROAD-flow-scripts constraint, so it is
reported for the designs that flow synthesized.
"""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import tomllib

ROOT = Path(__file__).resolve().parent.parent
ORFS = ROOT / "inputs/orfs/OpenROAD-flow-scripts-26Q2/flow"
PLATFORM = ORFS / "platforms/nangate45"
PARE = ROOT / "target/release/pare"
OPENROAD = ["nix", "develop", ".#eda", "-c"]
SIDES = ("pare", "openroad")


def design_dir(design):
    return ROOT / "output/compare" / design


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


def routed_def(design, side):
    return design_dir(design) / ("routed.def" if side == "pare" else "openroad.def")


def run(cmd, log, env=None):
    """Runs `cmd` under /usr/bin/time -v with its output in `log`."""
    full = ["/usr/bin/time", "-v", *cmd]
    if cmd[0] == "openroad":
        full = [*OPENROAD, *full]
    with open(log, "w") as out:
        result = subprocess.run(
            full,
            cwd=ROOT,
            stdout=out,
            stderr=subprocess.STDOUT,
            env={**os.environ, **(env or {})},
            check=False,
        )
    if result.returncode != 0:
        print(f"  failed ({result.returncode}), see {log.relative_to(ROOT)}")
    return result.returncode == 0


def route_with_pare(design):
    config = config_of(design)
    out = design_dir(design)
    config["input"]["output_dir"] = str(out)
    route_config = out / "pare.toml"
    route_config.write_text(toml_text(config))
    print(f"{design}: PARE routing")
    return run(
        [
            str(PARE),
            "route",
            str(route_config),
            "--placement",
            f"output/{design}/placed.def",
            "--report",
            str(out / "pare.json"),
        ],
        out / "pare.log",
    )


def toml_text(config):
    """The config as TOML: tables of scalar keys, as `pare` reads them."""
    lines = [f"purpose = {json.dumps(config['purpose'])}", ""]
    for table, values in config.items():
        if not isinstance(values, dict):
            continue
        lines.append(f"[{table}]")
        for key, value in values.items():
            lines.append(f"{key} = {json.dumps(value)}")
        lines.append("")
    return "\n".join(lines)


def route_with_openroad(design):
    out = design_dir(design)
    print(f"{design}: OpenROAD routing")
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


def check_drc(design, side):
    out = design_dir(design)
    print(f"{design}: OpenROAD check_drc of {side}")
    return run(
        ["openroad", "-no_init", "-no_splash", "-exit", "scripts/openroad_drc.tcl"],
        out / f"{side}.drc.log",
        env={
            "DRC_LEF": " ".join(lef_files(design)),
            "DRC_DEF": str(routed_def(design, side)),
            "DRC_RPT": str(out / f"{side}.drc.rpt"),
        },
    )


def pare_check(design, side):
    out = design_dir(design)
    print(f"{design}: pare check of {side}")
    return run(
        [
            str(PARE),
            "check",
            str(out / "pare.toml"),
            "--placement",
            str(routed_def(design, side)),
            "--report",
            str(out / f"{side}.check.json"),
        ],
        out / f"{side}.check.log",
    )


def timing(design, side):
    if orfs_design(design) is None:
        return None
    libs = [str(PLATFORM / "lib/NangateOpenCellLibrary_typical.lib")]
    additional = orfs_variable(design, "ADDITIONAL_LIBS")
    if additional:
        libs.extend(additional.split())
    out = design_dir(design)
    print(f"{design}: timing of {side}")
    return run(
        ["openroad", "-no_init", "-no_splash", "-exit", "scripts/openroad_timing.tcl"],
        out / f"{side}.timing.log",
        env={
            "TIMING_LEF": " ".join(lef_files(design)),
            "TIMING_LIB": " ".join(libs),
            "TIMING_DEF": str(routed_def(design, side)),
            "TIMING_SDC": orfs_variable(design, "SDC_FILE"),
            "TIMING_RCX": str(PLATFORM / "rcx_patterns.rules"),
            "TIMING_DIR": str(out),
        },
    )


def compare(design):
    design_dir(design).mkdir(parents=True, exist_ok=True)
    route_with_pare(design)
    route_with_openroad(design)
    for side in SIDES:
        if routed_def(design, side).exists():
            check_drc(design, side)
            pare_check(design, side)
            timing(design, side)


def time_and_memory(log):
    text = log.read_text(errors="replace") if log.exists() else ""
    wall = re.search(r"Elapsed \(wall clock\) time .*: (\S+)", text)
    rss = re.search(r"Maximum resident set size \(kbytes\): (\d+)", text)
    return (wall.group(1) if wall else None, int(rss.group(1)) if rss else None)


def result(design, side):
    out = design_dir(design)
    r = {"design": design, "router": side}
    r["wall"], r["rss_kb"] = time_and_memory(out / f"{side}.log")
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
        text = timing_log.read_text(errors="replace")
        for key, pattern in (
            ("wns_ns", r"wns max (-?[\d.]+)"),
            ("tns_ns", r"tns max (-?[\d.]+)"),
        ):
            m = re.search(pattern, text)
            r[key] = float(m.group(1)) if m else None
    return r


def cell(value, fmt="{}"):
    return "?" if value is None else fmt.format(value)


def table(results):
    lines = [
        "| Design | Router | Wirelength | Vias | PARE violations | OpenROAD `check_drc` | WNS | TNS | Time | Peak memory |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in results:
        pare_check = cell(r.get("pare_violations"))
        if r.get("opens"):
            pare_check += f" + {r['opens']} opens"
        lines.append(
            f"| {r['design']} | {'PARE' if r['router'] == 'pare' else 'OpenROAD'} "
            f"| {cell(r.get('wirelength_um'), '{:,.0f} µm')} | {cell(r.get('vias'), '{:,}')} "
            f"| {pare_check} | {cell(r.get('openroad_violations'))} "
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
        help="designs with a placement in output/<design>/placed.def",
    )
    parser.add_argument(
        "--table", action="store_true", help="only tabulate earlier runs"
    )
    args = parser.parse_args()
    if not args.table:
        if not PARE.exists():
            sys.exit(
                f"{PARE.relative_to(ROOT)} is missing: cargo build --release first"
            )
        for design in args.designs:
            compare(design)
    results = [result(d, side) for d in args.designs for side in SIDES]
    (ROOT / "output/compare/summary.json").write_text(json.dumps(results, indent=2))
    print(table(results))


if __name__ == "__main__":
    main()
