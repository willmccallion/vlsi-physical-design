#!/usr/bin/env python3
import os
import sys
import urllib.request
import gzip
import shutil

INPUT_DIR = "inputs/benchmarks"
OUTPUT_DIR = "output"

# GitHub mirror for the raw ISPD98/ICCAD04 benchmark files
IBM_GITHUB_BASE = "https://raw.githubusercontent.com/ckmarkoh/101_2_pdpa2/master/benchmark/ibm01"

# Standard Cell Library for non-IBM designs
LEF_URL = "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/test/Nangate45/Nangate45.lef"

DESIGNS = {
    "gcd": {
        "url": "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/src/grt/test/gcd.def",
        "lef_url": LEF_URL,
        "config_file": "config_gcd.toml",
        "description": "Tiny (~1k cells, Sanity Check)",
        "is_bookshelf": False
    },
    "ibm01": {
        "url": IBM_GITHUB_BASE,
        "config_file": "config_ibm01.toml",
        "description": "Classic ISPD98 Mixed-Size Benchmark (~12k cells)",
        "is_bookshelf": True
    }
}

def setup_directories():
    for d in [INPUT_DIR, OUTPUT_DIR]:
        if not os.path.exists(d):
            os.makedirs(d)
            print(f"[INFO] Created directory: {d}")

def download_file(url, filepath):
    if os.path.exists(filepath):
        if os.path.getsize(filepath) < 1000:
            print(f" {filepath} looks invalid (too small). Deleting.")
            os.remove(filepath)
        else:
            print(f"[INFO] Found {os.path.basename(filepath)}")
            return

    print(f"[INFO] Downloading {url}...")
    try:
        req = urllib.request.Request(
            url, 
            data=None, 
            headers={'User-Agent': 'Mozilla/5.0'}
        )
        with urllib.request.urlopen(req) as response, open(filepath, 'wb') as out_file:
            shutil.copyfileobj(response, out_file)

        if url.endswith(".gz") and not url.endswith(".tar.gz") and not filepath.endswith(".tar.gz"):
            if filepath.endswith(".gz"):
                print(f"[INFO] Decompressing {filepath}...")
                with gzip.open(filepath, 'rb') as f_in:
                    with open(filepath[:-3], 'wb') as f_out: 
                        shutil.copyfileobj(f_in, f_out)
                os.remove(filepath)
                print(f"[INFO] Extracted to {filepath[:-3]}")

    except Exception as e:
        print(f" Failed to download {url}")
        print(f"        Error: {e}")
        if os.path.exists(filepath):
            os.remove(filepath)

def process_ibm01(data):
    extract_dir = os.path.join(INPUT_DIR, "ibm01_raw")
    if not os.path.exists(extract_dir):
        os.makedirs(extract_dir)
        print(f"[INFO] Created {extract_dir}")

    files_to_download = {
        "ibm01.nodes": "ibm01.nodes",
        "ibm01.nets":  "ibm01.nets",
        "ibm01-cu85.pl": "ibm01.pl",
        "ibm01-cu85.scl": "ibm01.scl"
    }

    print("[INFO] Downloading IBM01 files from GitHub mirror...")
    all_files_present = True
    for remote_name, local_name in files_to_download.items():
        url = f"{data['url']}/{remote_name}"
        local_path = os.path.join(extract_dir, local_name)
        if not os.path.exists(local_path):
            download_file(url, local_path)
        if not os.path.exists(local_path):
            all_files_present = False

    if not all_files_present:
        print("[ERROR] Failed to download all required IBM01 files.")
        return

    # Generate AUX file
    aux_path = os.path.join(extract_dir, "ibm01.aux")
    if not os.path.exists(aux_path):
        print("[INFO] Generating ibm01.aux...")
        with open(aux_path, "w") as f:
            f.write("RowBasedPlacement : ibm01.nodes ibm01.nets ibm01.pl ibm01.scl\n")

    output_def_path = os.path.join(OUTPUT_DIR, "ibm01_placed.def")

def main():
    setup_directories()

    print("\n Benchmark setup complete.")
    print("You can now run the tool:")
    print("-" * 50)
    print(f"1. cargo run --release -- --config configs/config_gcd.toml flow")
    print(f"2. cargo run --release -- --config configs/config_ibm01.toml flow")
    print("-" * 50)

if __name__ == "__main__":
    main()
