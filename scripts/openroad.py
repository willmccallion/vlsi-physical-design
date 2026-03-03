#!/usr/bin/env python3
"""Download OpenROAD Nangate45 benchmarks (LEF/DEF format).

Usage:
    python scripts/openroad.py ibex       # RISC-V core (~29k cells)
    python scripts/openroad.py jpeg       # JPEG encoder (~99k cells)
    python scripts/openroad.py swerv      # SweRV RISC-V (~135k cells)
    python scripts/openroad.py netcard    # Network card (~275k cells)
    python scripts/openroad.py leon3mp    # LEON3 SPARC (~313k cells)
    python scripts/openroad.py all        # Download all
"""
import os
import sys
import urllib.request
import shutil

RAW = "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master"
LEF_URL = f"{RAW}/test/Nangate45/Nangate45.lef"

BENCHMARKS = {
    "ibex": {
        "def_url": f"{RAW}/src/gpl/test/design/nangate45/ibex_core/ibex_core.def",
        "def_name": "ibex.def",
        "desc": "RISC-V CPU core (~29k cells, 33k nets)",
    },
    "jpeg": {
        "def_url": f"{RAW}/src/gpl/test/medium03.def",
        "def_name": "jpeg.def",
        "desc": "JPEG encoder (~99k cells, 98k nets)",
    },
    "swerv": {
        "def_url": f"{RAW}/src/gpl/test/medium04.def",
        "def_name": "swerv.def",
        "desc": "WD SweRV RISC-V core (~135k cells, 140k nets)",
    },
    "netcard": {
        "def_url": f"{RAW}/src/gpl/test/large01.def",
        "def_name": "netcard.def",
        "desc": "Network card (~275k cells, 290k nets)",
    },
    "leon3mp": {
        "def_url": f"{RAW}/src/gpl/test/large02.def",
        "def_name": "leon3mp.def",
        "desc": "LEON3 SPARC processor (~313k cells, 407k nets)",
    },
}


def fetch(url: str, path: str):
    if os.path.exists(path) and os.path.getsize(path) > 1000:
        print(f"    Already have {os.path.basename(path)}")
        return
    print(f"    Downloading {os.path.basename(path)}...")
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp, open(path, "wb") as f:
        shutil.copyfileobj(resp, f)


def download(name: str):
    bench = BENCHMARKS[name]
    input_dir = f"inputs/{name}"

    print(f"[{name}] {bench['desc']}")
    os.makedirs(input_dir, exist_ok=True)

    fetch(LEF_URL, os.path.join(input_dir, "Nangate45.lef"))
    fetch(bench["def_url"], os.path.join(input_dir, bench["def_name"]))

    print(f"[{name}] Done. Run with:")
    print(f"    pare flow configs/{name}.toml")


def main():
    if len(sys.argv) < 2:
        print(__doc__.strip())
        sys.exit(1)

    arg = sys.argv[1].lower()

    if arg == "all":
        for name in BENCHMARKS:
            download(name)
            print()
    elif arg in BENCHMARKS:
        download(arg)
    else:
        print(f"Unknown benchmark: {arg}")
        print(f"Valid: {', '.join(BENCHMARKS)}")
        sys.exit(1)


if __name__ == "__main__":
    main()
