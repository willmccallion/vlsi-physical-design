#!/usr/bin/env python3
import os
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
    "aes": {
        "url": "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/test/upf_aes.defok",
        "lef_url": LEF_URL,
        "config_file": "config_aes.toml",
        "description": "Large (~45k cells, AES Encryption block)",
        "is_bookshelf": False,
        "local_filename": "aes.def"
    },
    "ibm01": {
        "url": IBM_GITHUB_BASE,
        "config_file": "config_ibm01.toml",
        "description": "Classic ISPD98 Mixed-Size Benchmark (~12k cells)",
        "is_bookshelf": True
    },
    "ibm05": {
        "url": "https://raw.githubusercontent.com/ckmarkoh/101_2_pdpa2/master/benchmark/ibm05",
        "config_file": "config_ibm05.toml",
        "description": "ISPD98 Benchmark (~29k cells)",
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

def process_lef_def(name, data):
    local_filename = data.get("local_filename", f"{name}.def")
    def_path = os.path.join(INPUT_DIR, local_filename)
    download_file(data["url"], def_path)

    lef_path = os.path.join(INPUT_DIR, "Nangate45.lef")
    download_file(data["lef_url"], lef_path)

def process_bookshelf(name, data):
    extract_dir = os.path.join(INPUT_DIR, f"{name}_raw")
    if not os.path.exists(extract_dir):
        os.makedirs(extract_dir)
        print(f"[INFO] Created {extract_dir}")

    # ibm01 uses non-standard filenames for pl/scl
    file_map = {
        "ibm01": {
            "ibm01.nodes":    "ibm01.nodes",
            "ibm01.nets":     "ibm01.nets",
            "ibm01-cu85.pl":  "ibm01.pl",
            "ibm01-cu85.scl": "ibm01.scl",
        },
        "ibm05": {
            "ibm05.nodes": "ibm05.nodes",
            "ibm05.nets":  "ibm05.nets",
            "ibm05.pl":    "ibm05.pl",
            "ibm05.scl":   "ibm05.scl",
            "ibm05.wts":   "ibm05.wts",
        },
    }

    files_to_download = file_map.get(name, {
        f"{name}.nodes": f"{name}.nodes",
        f"{name}.nets":  f"{name}.nets",
        f"{name}.pl":    f"{name}.pl",
        f"{name}.scl":   f"{name}.scl",
    })

    print(f"[INFO] Downloading {name.upper()} files from GitHub mirror...")
    all_files_present = True
    for remote_name, local_name in files_to_download.items():
        url = f"{data['url']}/{remote_name}"
        local_path = os.path.join(extract_dir, local_name)
        if not os.path.exists(local_path):
            download_file(url, local_path)
        if not os.path.exists(local_path):
            all_files_present = False

    if not all_files_present:
        print(f"[ERROR] Failed to download all required {name.upper()} files.")
        return

    aux_path = os.path.join(extract_dir, f"{name}.aux")
    if not os.path.exists(aux_path):
        print(f"[INFO] Generating {name}.aux...")
        core_files = " ".join(f"{name}.{ext}" for ext in ["nodes", "nets", "pl", "scl"])
        if name == "ibm05":
            core_files = "ibm05.nodes ibm05.nets ibm05.wts ibm05.pl ibm05.scl"
        with open(aux_path, "w") as f:
            f.write(f"RowBasedPlacement : {core_files}\n")

def main():
    setup_directories()

    for name, data in DESIGNS.items():
        print(f"\n[INFO] Setting up: {data['description']}")
        if data.get("is_bookshelf"):
            process_bookshelf(name, data)
        else:
            process_lef_def(name, data)

    print("\nBenchmark setup complete.")
    print("You can now run the tool:")
    print("-" * 50)
    print("1. cargo run --release -- --config configs/config_gcd.toml flow")
    print("2. cargo run --release -- --config configs/config_aes.toml flow")
    print("3. cargo run --release -- --config configs/config_ibm01.toml flow")
    print("4. cargo run --release -- --config configs/config_ibm05.toml flow")
    print("-" * 50)

if __name__ == "__main__":
    main()
