#!/usr/bin/env python3
"""Download IBM ISPD benchmarks (Bookshelf format) from UMich.

Usage:
    python scripts/ibm.py 01          # IBM01 (~12k cells)
    python scripts/ibm.py 05          # IBM05 (~28k cells)
    python scripts/ibm.py 10          # IBM10 (~68k cells)
    python scripts/ibm.py 13          # IBM13 (~81k cells)
    python scripts/ibm.py 14          # IBM14 (~145k cells)
    python scripts/ibm.py 18          # IBM18 (~210k cells)
    python scripts/ibm.py all         # Download all
"""
import os
import sys
import tarfile
import urllib.request
import shutil

VALID = ["01", "02", "03", "04", "05", "06", "07", "08", "09",
         "10", "11", "12", "13", "14", "15", "16", "17", "18"]

BASE_URL = "http://vlsicad.eecs.umich.edu/BK/Slots/cache/er.cs.ucla.edu/benchmarks/ibm-place/files"


def download(num: str):
    name = f"ibm{num}"
    input_dir = f"inputs/{name}"
    aux_path = os.path.join(input_dir, f"{name}.aux")

    if os.path.exists(aux_path):
        print(f"[{name}] Already downloaded.")
        return

    os.makedirs(input_dir, exist_ok=True)
    tarball = f"/tmp/{name}.tar.gz"
    url = f"{BASE_URL}/{name}-place-bookshelf.tar.gz"

    print(f"[{name}] Downloading from UMich...")
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp, open(tarball, "wb") as f:
        shutil.copyfileobj(resp, f)

    print(f"[{name}] Extracting...")
    with tarfile.open(tarball) as tar:
        for member in tar.getmembers():
            if member.isfile():
                member.name = os.path.basename(member.name)
                tar.extract(member, input_dir)

    os.remove(tarball)
    print(f"[{name}] Done. Run with:")
    print(f"  pare flow configs/{name}.toml")


def main():
    if len(sys.argv) < 2:
        print(__doc__.strip())
        sys.exit(1)

    arg = sys.argv[1].lower()

    if arg == "all":
        for num in VALID:
            download(num)
    elif arg.lstrip("0") == "" and arg != "":
        # Handle bare numbers like "1" -> "01"
        download(arg.zfill(2))
    elif arg.zfill(2) in VALID:
        download(arg.zfill(2))
    else:
        print(f"Unknown benchmark: ibm{arg}")
        print(f"Valid: {', '.join(VALID)}")
        sys.exit(1)


if __name__ == "__main__":
    main()
