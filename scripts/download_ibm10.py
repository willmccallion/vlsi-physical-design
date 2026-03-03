#!/usr/bin/env python3
"""Download IBM10 benchmark (~68k cells) from UMich IBM-PLACE 1.0."""
import os
import tarfile
import urllib.request
import shutil

NAME = "ibm10"
INPUT_DIR = "inputs/ibm10"
EXTRACT_DIR = INPUT_DIR
URL = f"http://vlsicad.eecs.umich.edu/BK/Slots/cache/er.cs.ucla.edu/benchmarks/ibm-place/files/{NAME}-place-bookshelf.tar.gz"


def main():
    if os.path.exists(os.path.join(EXTRACT_DIR, f"{NAME}.aux")):
        print(f"[{NAME}] Already downloaded.")
        return

    os.makedirs(EXTRACT_DIR, exist_ok=True)
    tarball = f"/tmp/{NAME}.tar.gz"

    print(f"[{NAME}] Downloading from UMich...")
    req = urllib.request.Request(URL, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp, open(tarball, "wb") as f:
        shutil.copyfileobj(resp, f)

    print(f"[{NAME}] Extracting...")
    with tarfile.open(tarball) as tar:
        for member in tar.getmembers():
            if member.isfile():
                member.name = os.path.basename(member.name)
                tar.extract(member, EXTRACT_DIR)

    os.remove(tarball)
    print(f"[{NAME}] Done. Run with:")
    print(f"  cargo run --release -- flow configs/{NAME}.toml")


if __name__ == "__main__":
    main()
