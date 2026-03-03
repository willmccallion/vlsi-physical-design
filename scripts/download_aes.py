#!/usr/bin/env python3
"""Download the AES benchmark (~45k cells, AES encryption block)."""
import os
import urllib.request
import shutil

INPUT_DIR = "inputs/aes"
DEF_URL = "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/test/upf_aes.defok"
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
    print("[aes] Downloading AES benchmark...")
    download(DEF_URL, os.path.join(INPUT_DIR, "aes.def"))
    download(LEF_URL, os.path.join(INPUT_DIR, "Nangate45.lef"))
    print("[aes] Done. Run with:")
    print("  cargo run --release -- flow configs/aes.toml")


if __name__ == "__main__":
    main()
