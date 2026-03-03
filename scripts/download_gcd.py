#!/usr/bin/env python3
"""Download the GCD benchmark (~1k cells, sanity check)."""
import os
import urllib.request
import shutil

INPUT_DIR = "inputs/gcd"
DEF_URL = "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/src/grt/test/gcd.def"
LEF_URL = "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/test/Nangate45/Nangate45.lef"


def download(url, path):
    if os.path.exists(path) and os.path.getsize(path) > 1000:
        print(f"  Already have {os.path.basename(path)}")
        return
    print(f"  Downloading {url}...")
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp, open(path, "wb") as f:
        shutil.copyfileobj(resp, f)


def main():
    os.makedirs(INPUT_DIR, exist_ok=True)
    print("[gcd] Downloading GCD benchmark...")
    download(DEF_URL, os.path.join(INPUT_DIR, "gcd.def"))
    download(LEF_URL, os.path.join(INPUT_DIR, "gcd.lef"))
    print("[gcd] Done. Run with:")
    print("  cargo run --release -- flow configs/config_gcd.toml")


if __name__ == "__main__":
    main()
