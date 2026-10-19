#!/usr/bin/env python3
"""Download benchmarks into inputs/ and write their run configs.

Every design has a purpose that decides what `pare run` does with it:
  placement        made to test placement; placed, never routed
  routing          made to test routing; its placement is part of the input
  place_and_route  placed, then routed

Usage:
    python scripts/download.py --list             # every design, by suite
    python scripts/download.py gcd aes            # designs by name
    python scripts/download.py ispd2018           # a whole suite
    python scripts/download.py all                # everything (several GB)
    python scripts/download.py --configs          # rewrite every config only

Designs land in inputs/<suite>/<design>/, technology files shared by a
suite in inputs/tech/, and configs in configs/<suite>/<design>.toml.

aes, ibex and jpeg are synthesized and floorplanned by OpenROAD-flow-scripts
26Q2, with the yosys and OpenROAD of `nix develop .#eda`, so download them
inside that shell.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request

OPENROAD = "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master"
IBM = "http://vlsicad.eecs.umich.edu/BK/Slots/cache/er.cs.ucla.edu/benchmarks/ibm-place/files"
ISPD = "https://www.ispd.cc/contests"
ORFS = "https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts.git"
ORFS_TAG = "26Q2"
ORFS_DIR = f"inputs/orfs/OpenROAD-flow-scripts-{ORFS_TAG}"

NANGATE45_LEF = "inputs/tech/nangate45/Nangate45.lef"
NANGATE45_IO = {"horizontal_layer": "metal5", "vertical_layer": "metal6"}
ORFS_NANGATE45 = f"{ORFS_DIR}/flow/platforms/nangate45/lef"
ORFS_NANGATE45_LEF = [
    f"{ORFS_NANGATE45}/NangateOpenCellLibrary.tech.lef",
    f"{ORFS_NANGATE45}/NangateOpenCellLibrary.macro.mod.lef",
]


def orfs(nickname, sources, about, io=None, io_enabled=None):
    """A Nangate45 design that OpenROAD-flow-scripts synthesizes from its
    `sources` under flow/designs/src and floorplans: die, rows, tracks, tap
    cells, power grid, and the I/O pins only where its floorplan gives
    them."""
    return {
        "suite": "openroad",
        "purpose": "place_and_route",
        "about": about,
        "orfs": nickname,
        "orfs_sources": sources,
        "lef": ORFS_NANGATE45_LEF,
        "def": f"{nickname}_orfs.def",
        "io": io,
        "io_enabled": io_enabled,
        "placer_iterations": 2000,
    }


def openroad(def_path, purpose, about, def_name=None, io=None, placer_iterations=2000):
    """A Nangate45 design from the OpenROAD repository's test inputs."""
    return {
        "suite": "openroad",
        "purpose": purpose,
        "about": about,
        "files": {def_name or os.path.basename(def_path): f"{OPENROAD}/{def_path}"},
        "lef": [NANGATE45_LEF],
        "def": def_name or os.path.basename(def_path),
        "io": io,
        "placer_iterations": placer_iterations,
    }


def ibm(num, num_layers=None):
    """An IBM placement benchmark in Bookshelf format."""
    name = f"ibm{num}"
    return {
        "suite": "ibm",
        "purpose": "placement",
        "about": "IBM-PLACE (ISPD 2004), Bookshelf",
        "archive": f"{IBM}/{name}-place-bookshelf.tar.gz",
        "aux": f"{name}.aux",
        "num_layers": num_layers,
        "placer_iterations": 2000,
    }


def ispd2015(name, max_utilization, hidden=False):
    """An ISPD 2015 detailed-routing-driven placement contest design, with
    the maximum utilization its placement.constraints file sets."""
    archive = "hidden.tgz" if hidden else "ispd_2015_contest_benchmark.tgz"
    member = f"hidden/{name}" if hidden else name
    return {
        "suite": "ispd2015",
        "purpose": "place_and_route",
        "about": "ISPD 2015 detailed-routing-driven placement contest",
        "suite_archive": f"{ISPD}/15/web/benchmarks/{archive}",
        "member": member,
        "lef_names": ["tech.lef", "cells.lef"],
        "def": "floorplan.def",
        "placer_iterations": 2000,
        "target_density": max_utilization,
        # The contest forbids moving I/O pins, though the floorplans mark them PLACED.
        "io_enabled": False,
    }


def ispd_routing(year, num):
    """An ISPD 2018 or 2019 initial detailed routing contest design."""
    name = f"ispd{year}_test{num}"
    path = f"{ISPD}/18/{name}.tgz" if year == 18 else f"{ISPD}/19/benchmarks/{name}.tgz"
    return {
        "suite": f"ispd20{year}",
        "purpose": "routing",
        "about": f"ISPD 20{year} initial detailed routing contest",
        "archive": path,
        "lef_names": [f"{name}.input.lef"],
        "def": f"{name}.input.def",
    }


DESIGNS = {
    "gcd": openroad(
        "src/grt/test/gcd.def",
        "routing",
        "GCD, ~500 nets, placed (OpenROAD global router test)",
        placer_iterations=1000,
    ),
    "aes": orfs(
        "aes",
        ["aes"],
        "AES cipher, ~13k nets, ORFS 26Q2 synthesis and floorplan (32% utilization)",
        io_enabled=False,
    ),
    "ibex": orfs(
        "ibex",
        ["ibex_sv"],
        "Ibex RISC-V core, ORFS 26Q2 synthesis and floorplan (50% utilization)",
        io=NANGATE45_IO,
    ),
    "jpeg": orfs(
        "jpeg",
        ["jpeg"],
        "JPEG encoder, ORFS 26Q2 synthesis and floorplan (80% utilization)",
        io=NANGATE45_IO,
    ),
    "swerv": openroad(
        "src/gpl/test/medium04.def",
        "place_and_route",
        "SweRV core, ~100k nets (OpenROAD placer test)",
        def_name="swerv.def",
        io=NANGATE45_IO,
    ),
    "netcard": openroad(
        "src/gpl/test/large01.def",
        "place_and_route",
        "Network card, ~290k nets (OpenROAD placer test)",
        def_name="netcard.def",
        io=NANGATE45_IO,
    ),
    "leon3mp": openroad(
        "src/gpl/test/large02.def",
        "place_and_route",
        "LEON3 SPARC, ~407k nets (OpenROAD placer test)",
        def_name="leon3mp.def",
        io=NANGATE45_IO,
    ),
}
for _n in range(1, 19):
    DESIGNS[f"ibm{_n:02d}"] = ibm(f"{_n:02d}", num_layers=10 if _n == 18 else None)
for _name, _util in [
    ("mgc_des_perf_1", 0.906), ("mgc_des_perf_a", 0.429), ("mgc_des_perf_b", 0.497),
    ("mgc_edit_dist_a", 0.455), ("mgc_fft_1", 0.835), ("mgc_fft_2", 0.65),
    ("mgc_fft_a", 0.5), ("mgc_fft_b", 0.6), ("mgc_matrix_mult_1", 0.802),
    ("mgc_matrix_mult_a", 0.6), ("mgc_matrix_mult_b", 0.6), ("mgc_pci_bridge32_a", 0.384),
    ("mgc_pci_bridge32_b", 0.143), ("mgc_superblue11_a", 0.65), ("mgc_superblue12", 0.65),
    ("mgc_superblue16_a", 0.55),
]:
    DESIGNS[_name] = ispd2015(_name, _util)
for _name, _util in [
    ("mgc_matrix_mult_2", 0.8), ("mgc_matrix_mult_c", 0.6),
    ("mgc_superblue14", 0.56), ("mgc_superblue19", 0.53),
]:
    DESIGNS[_name] = ispd2015(_name, _util, hidden=True)
for _year in (18, 19):
    for _n in range(1, 11):
        DESIGNS[f"ispd{_year}_test{_n}"] = ispd_routing(_year, _n)

SUITES = sorted({d["suite"] for d in DESIGNS.values()})


def design_dir(name):
    return f"inputs/{DESIGNS[name]['suite']}/{name}"


def fetch(url, path):
    """Downloads `url` to `path` unless a non-empty file is already there."""
    if os.path.exists(path) and os.path.getsize(path) > 0:
        print(f"    have {path}")
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(f"    downloading {url}")
    request = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    partial = path + ".part"
    with urllib.request.urlopen(request) as response, open(partial, "wb") as out:
        shutil.copyfileobj(response, out)
    os.replace(partial, path)


def extract(archive, member_prefix, dest, keep):
    """Extracts the regular files under `member_prefix` that `keep` accepts
    into `dest`, flattened, refusing anything that would land outside it."""
    os.makedirs(dest, exist_ok=True)
    with tarfile.open(archive) as tar:
        for m in tar.getmembers():
            if not m.isfile():
                continue
            parts = m.name.split("/")
            if member_prefix and "/".join(parts[:-1]).rstrip("/") != member_prefix:
                continue
            base = parts[-1]
            if not keep(base):
                continue
            m.name = base
            tar.extract(m, dest, filter="data")


def orfs_checkout():
    """OpenROAD-flow-scripts at ORFS_TAG, with only what synthesizing and
    floorplanning the Nangate45 designs here reads."""
    if not os.path.exists(f"{ORFS_DIR}/.git"):
        os.makedirs(os.path.dirname(ORFS_DIR), exist_ok=True)
        subprocess.run(
            ["git", "clone", "--quiet", "--depth", "1", "--filter=blob:none", "--sparse",
             "--branch", ORFS_TAG, ORFS, ORFS_DIR],
            check=True,
        )
    sources = sorted({s for d in DESIGNS.values() for s in d.get("orfs_sources", [])})
    subprocess.run(
        ["git", "-C", ORFS_DIR, "sparse-checkout", "set", "flow/scripts", "flow/util",
         "flow/platforms/common", "flow/platforms/nangate45", "flow/designs/nangate45"]
        + [f"flow/designs/src/{s}" for s in sources],
        check=True,
    )


def orfs_floorplan(name, dest):
    """Synthesizes and floorplans design `name` with OpenROAD-flow-scripts
    and writes the floorplan as a DEF into `dest`."""
    d = DESIGNS[name]
    out = f"{dest}/{d['def']}"
    if os.path.exists(out):
        print(f"    have {out}")
        return
    tools = {t: shutil.which(t) for t in ("yosys", "openroad")}
    missing = [t for t, path in tools.items() if path is None]
    if missing:
        sys.exit(f"{name} needs {' and '.join(missing)}: run inside `nix develop .#eda`")
    orfs_checkout()
    subprocess.run(
        ["make", "-C", f"{ORFS_DIR}/flow",
         f"DESIGN_CONFIG=./designs/nangate45/{d['orfs']}/config.mk",
         f"YOSYS_EXE={tools['yosys']}", f"OPENROAD_EXE={tools['openroad']}", "floorplan"],
        check=True,
    )
    os.makedirs(dest, exist_ok=True)
    odb = f"{ORFS_DIR}/flow/results/nangate45/{d['orfs']}/base/2_floorplan.odb"
    subprocess.run(
        [tools["openroad"], "-no_init", "-no_splash", "-exit", "scripts/odb_to_def.tcl"],
        env={**os.environ, "ODB_IN": odb, "DEF_OUT": out},
        check=True,
    )


def download(name, cache):
    d = DESIGNS[name]
    dest = design_dir(name)
    print(f"[{name}] {d['about']}")
    if d.get("orfs"):
        orfs_floorplan(name, dest)
    elif d["suite"] == "openroad":
        fetch(f"{OPENROAD}/test/Nangate45/Nangate45.lef", NANGATE45_LEF)
        for file, url in d["files"].items():
            fetch(url, f"{dest}/{file}")
    elif d["suite"] == "ibm":
        if os.path.exists(f"{dest}/{d['aux']}"):
            print(f"    have {dest}")
            return
        tarball = f"{cache}/{name}.tar.gz"
        fetch(d["archive"], tarball)
        extract(tarball, None, dest, lambda base: True)
    elif d["suite"] == "ispd2015":
        if os.path.exists(f"{dest}/{d['def']}"):
            print(f"    have {dest}")
            return
        tarball = f"{cache}/{os.path.basename(d['suite_archive'])}"
        fetch(d["suite_archive"], tarball)
        extract(tarball, d["member"], dest, lambda base: not base.endswith(".jpg"))
    else:
        if os.path.exists(f"{dest}/{d['def']}"):
            print(f"    have {dest}")
            return
        tarball = f"{cache}/{name}.tgz"
        fetch(d["archive"], tarball)
        extract(tarball, None, dest, lambda base: not base.startswith("."))


def toml_list(values):
    return "[" + ", ".join(f'"{v}"' for v in values) + "]"


def config_text(name):
    """The run config of a design."""
    d = DESIGNS[name]
    dest = design_dir(name)
    lines = [f'purpose = "{d["purpose"]}"', "", "[input]"]
    if d["suite"] == "ibm":
        lines.append(f'bookshelf_aux_file = "{dest}/{d["aux"]}"')
        if d.get("num_layers"):
            lines.append(f"num_layers = {d['num_layers']}")
    else:
        lefs = d["lef"] if "lef" in d else [f"{dest}/{f}" for f in d["lef_names"]]
        lines.append(f"lef_files = {toml_list(lefs)}")
        lines.append(f'def_file = "{dest}/{d["def"]}"')
    lines.append(f'output_def = "output/{name}/placed.def"')
    if d["purpose"] != "routing":
        lines += ["", "[global_placement]", f"placer_max_iterations = {d['placer_iterations']}"]
        if d.get("target_density") is not None:
            lines.append(f"target_density = {d['target_density']}")
    if d.get("io"):
        lines += ["", "[io_placement]"]
        lines += [f'{k} = "{v}"' for k, v in d["io"].items()]
    if d.get("io_enabled") is False:
        lines += ["", "[io_placement]", "enabled = false"]
    return "\n".join(lines) + "\n"


def config_path(name):
    return f"configs/{DESIGNS[name]['suite']}/{name}.toml"


def write_config(name):
    path = config_path(name)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(config_text(name))


def resolve(targets):
    names = []
    for t in targets:
        if t == "all":
            names += list(DESIGNS)
        elif t in SUITES:
            names += [n for n, d in DESIGNS.items() if d["suite"] == t]
        elif t in DESIGNS:
            names.append(t)
        else:
            sys.exit(f"unknown design or suite '{t}'; see --list")
    return list(dict.fromkeys(names))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("targets", nargs="*", help="designs, suites, or 'all'")
    parser.add_argument("--list", action="store_true", help="list designs by suite")
    parser.add_argument("--configs", action="store_true", help="write every config without downloading")
    args = parser.parse_args()

    if args.list:
        for suite in SUITES:
            print(suite)
            for n, d in DESIGNS.items():
                if d["suite"] == suite:
                    print(f"  {n:22} {d['purpose']:16} {d['about']}")
        return
    if args.configs:
        for n in DESIGNS:
            write_config(n)
        print(f"wrote {len(DESIGNS)} configs")
        return
    if not args.targets:
        parser.print_help()
        sys.exit(1)
    with tempfile.TemporaryDirectory(prefix="pare-download-") as cache:
        for n in resolve(args.targets):
            download(n, cache)
            write_config(n)
            print(f"    run with: cargo run --release -- run {n}")


if __name__ == "__main__":
    main()
