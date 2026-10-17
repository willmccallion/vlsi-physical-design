#!/usr/bin/env python3
"""Route and check small windows of placed designs, each in a minute or two.

    scripts/windows.py [NAME ...] [--placed DEF] [--replace] [--rounds N] [--no-openroad]

Windows are listed in scripts/windows.txt, one per line:

    name  design  X0 Y0 X1 Y1  [partial]

`design` names a config in configs/<suite>/ whose [input] gives the LEF
files; the window is cut from output/<design>/placed.def unless --placed
names another DEF. Each window is cropped with scripts/crop.py into
output/windows/<name>/, routed with the release build of pare, and checked
by PARE and, unless --no-openroad, by OpenROAD's drt::check_drc on one tile
(the DEF's GCELLGRID rewritten to a single gcell, so no marker is split at a
tile edge). With --replace the cropped placement is first legalized and
refined by `pare place --from`, so placement-side fixes are exercised too.
The exit status is nonzero when any window has markers left.
"""

import argparse
import os
import re
import subprocess
import sys
import time
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PARE = ROOT / "target" / "release" / "pare"
WINDOWS = ROOT / "scripts" / "windows.txt"
OUT = ROOT / "output" / "windows"
OPENROAD = ["nix", "develop", ".#eda", "-c", "openroad", "-no_init", "-no_splash", "-exit"]


def find_config(design):
    hits = list((ROOT / "configs").glob(f"*/{design}.toml"))
    if len(hits) != 1:
        sys.exit(f"{design}: expected one config, found {hits}")
    return hits[0]


def read_windows(names):
    rows = []
    for line in WINDOWS.read_text().splitlines():
        line = line.split("#")[0].strip()
        if not line:
            continue
        parts = line.split()
        name, design, box, flags = parts[0], parts[1], parts[2:6], parts[6:]
        if names and name not in names:
            continue
        rows.append((name, design, [float(v) for v in box], "partial" in flags))
    return rows


def one_tile(src, dst):
    """Copies a routed DEF with its GCELLGRID collapsed to one gcell."""
    text = src.read_text()
    die = re.search(r"DIEAREA \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)", text)
    x0, y0, x1, y1 = (int(v) for v in die.groups())
    grid = f"GCELLGRID X {x0} DO 2 STEP {x1 - x0} ;\nGCELLGRID Y {y0} DO 2 STEP {y1 - y0} ;\n"
    text = re.sub(r"GCELLGRID [XY] .*?;\n", "", text)
    text = text.replace("DIEAREA", grid + "DIEAREA", 1) if "GCELLGRID" not in text else text
    dst.write_text(text)


def run(name, design, box, partial, placed, rounds, openroad, replace):
    config_path = find_config(design)
    config = tomllib.loads(config_path.read_text())
    lefs = [str(ROOT / p) for p in config["input"]["lef_files"]]
    cells_lef = lefs[-1]
    placed = placed or ROOT / "output" / design / "placed.def"
    d = OUT / name
    d.mkdir(parents=True, exist_ok=True)
    in_def = d / "in.def"
    crop = [sys.executable, "-I", str(ROOT / "scripts" / "crop.py"), str(placed), cells_lef]
    crop += [str(v) for v in box] + [str(in_def)] + (["--partial"] if partial else [])
    cropped = subprocess.run(crop, capture_output=True, text=True, check=True).stdout.strip()

    routing = dict(config.get("routing", {}))
    if rounds is not None:
        routing["detailed_iterations"] = rounds
    lef_list = "lef_files = [" + ", ".join(f'"{p}"' for p in config["input"]["lef_files"]) + "]"
    toml = [
        'purpose = "place_and_route"',
        "",
        "[input]",
        lef_list,
        f'def_file = "{in_def}"',
        f'output_def = "{d / "placed.def"}"',
        "",
        "[global_placement]",
        "routability_driven = false",
        "",
        "[io_placement]",
        "enabled = false",
        "",
        "[routing]",
    ] + [f"{k} = {repr(v).replace(chr(39), chr(34))}" for k, v in routing.items()]
    (d / "window.toml").write_text("\n".join(toml) + "\n")

    env = dict(os.environ, RUST_LOG="pare=debug")
    start = time.time()
    to_route = in_def
    if replace:
        with open(d / "place.log", "w") as log:
            placed_proc = subprocess.run(
                [str(PARE), "place", str(d / "window.toml"), "--from", str(in_def)],
                stdout=log, stderr=subprocess.STDOUT, env=env, cwd=ROOT,
            )
        if placed_proc.returncode != 0:
            sys.exit(f"{name}: pare place --from failed, see {d / 'place.log'}")
        to_route = d / "placed.def"
    with open(d / "route.log", "w") as log:
        proc = subprocess.run(
            ["/usr/bin/time", "-v", str(PARE), "route", str(d / "window.toml"), "--placement", str(to_route)],
            stdout=log, stderr=subprocess.STDOUT, env=env, cwd=ROOT,
        )
    route_s = time.time() - start
    log = (d / "route.log").read_text(errors="replace")
    pare = int(m.group(1)) if (m := re.search(r"DRC violations:\s+(\d+)", log)) else -1
    gr = int(m.group(1)) if (m := re.search(r"GR overflow:\s+(\d+)", log)) else -1
    rss = int(m.group(1)) // 1024 if (m := re.search(r"Maximum resident set size \(kbytes\): (\d+)", log)) else -1
    unreserved = log.count("none of")

    ored = None
    if openroad and (d / "routed.def").exists():
        one_tile(d / "routed.def", d / "one_tile.def")
        env = dict(os.environ, DRC_LEF=" ".join(lefs), DRC_DEF=str(d / "one_tile.def"), DRC_RPT=str(d / "or.rpt"))
        with open(d / "or.log", "w") as log_or:
            subprocess.run(OPENROAD + [str(ROOT / "scripts" / "openroad_drc.tcl")], stdout=log_or, stderr=subprocess.STDOUT, env=env, cwd=ROOT)
        text = (d / "or.log").read_text(errors="replace")
        ored = int(m.group(1)) if (m := re.search(r"DRC violations: (\d+)", text)) else -1
    return {
        "name": name, "cropped": cropped, "exit": proc.returncode, "pare": pare, "openroad": ored,
        "gr": gr, "unreserved": unreserved, "seconds": route_s, "rss_mb": rss,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("names", nargs="*")
    parser.add_argument("--placed", type=Path, help="placed DEF to crop instead of the design's output")
    parser.add_argument("--replace", action="store_true", help="legalize and refine the cropped placement first")
    parser.add_argument("--rounds", type=int, help="detailed routing rounds, overriding the config")
    parser.add_argument("--no-openroad", action="store_true")
    args = parser.parse_args()
    if not PARE.exists():
        sys.exit("build first: cargo build --release")
    failed = 0
    print(f"{'window':14s} {'pare':>6s} {'openroad':>8s} {'gr':>6s} {'unresv':>6s} {'s':>6s} {'MB':>6s}  cropped")
    for name, design, box, partial in read_windows(args.names):
        r = run(name, design, box, partial, args.placed, args.rounds, not args.no_openroad, args.replace)
        ored = "-" if r["openroad"] is None else str(r["openroad"])
        print(f"{r['name']:14s} {r['pare']:6d} {ored:>8s} {r['gr']:6d} {r['unreserved']:6d} {r['seconds']:6.0f} {r['rss_mb']:6d}  {r['cropped']}", flush=True)
        if r["pare"] != 0 or (r["openroad"] not in (None, 0)):
            failed += 1
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
