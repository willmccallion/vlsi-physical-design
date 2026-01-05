#!/usr/bin/env python3
import os
import sys
import urllib.request
import gzip
import shutil

INPUT_DIR = "inputs/benchmarks"
OUTPUT_DIR = "output"

DESIGNS = {
    "gcd": {
        "url": "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/src/grt/test/gcd.def",
        "config_file": "config_gcd.toml",
        "bin_dimension": 32,
        "description": "Tiny (~1k cells, Sanity Check)"
    },
    "aes": {
        "url": "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/test/upf_aes.defok",
        "config_file": "config_aes.toml",
        "bin_dimension": 64,
        "description": "Large (~50k cells, AES Encryption)"
    },
}

LEF_URL = "https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/test/Nangate45/Nangate45.lef"

CONFIG_TEMPLATE = """
[global_placement]
target_density = 0.70
bin_dimension = {bin_dimension}
placer_max_iterations = 1000
initial_learning_rate = 0.005
convergence_threshold = 2e-4
wa_gamma = 4.0
electro_force_multiplier = 40.0

[legalization]
algorithm = "abacus"

[global_routing]
gcell_size = 64
capacity = 15
max_iterations = 100
history_increment = 1.0
initial_penalty = 1.0
penalty_multiplier = 1.5

[detailed_routing]
max_iterations = 50
history_increment = 5.0
initial_penalty = 5.0
penalty_multiplier = 2.0
capacity = 1
astar_window_margin_base = 10
astar_window_margin_max = 50
ripup_radius = 2

[input]
lef_files = ["{lef_file}"]
def_file = "{def_file}"
output_def = "{output_file}"
"""

def setup_directories():
    if not os.path.exists(INPUT_DIR):
        os.makedirs(INPUT_DIR)
        print(f"[INFO] Created directory: {INPUT_DIR}")
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"[INFO] Created directory: {OUTPUT_DIR}")

def download_file(url, filepath):
    if os.path.exists(filepath):
        if os.path.getsize(filepath) < 1000:
            print(f"[WARN] {filepath} looks invalid (too small). Deleting.")
            os.remove(filepath)
        else:
            print(f"[INFO] Found {os.path.basename(filepath)}")
            return

    print(f"[INFO] Downloading {os.path.basename(filepath)}...")
    try:
        req = urllib.request.Request(
            url, 
            data=None, 
            headers={
                'User-Agent': 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_9_3) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/35.0.1916.47 Safari/537.36'
            }
        )

        with urllib.request.urlopen(req) as response, open(filepath, 'wb') as out_file:
            shutil.copyfileobj(response, out_file)

        if url.endswith(".gz") and not filepath.endswith(".gz"):
            pass 
        elif filepath.endswith(".gz"):
            # Decompress
            print(f"[INFO] Decompressing {filepath}...")
            with gzip.open(filepath, 'rb') as f_in:
                with open(filepath[:-3], 'wb') as f_out: # Remove .gz
                    shutil.copyfileobj(f_in, f_out)
            os.remove(filepath) # Remove the .gz file
            print(f"[INFO] Extracted to {filepath[:-3]}")

    except Exception as e:
        print(f"[ERROR] Failed to download {url}")
        print(f"        Error: {e}")
        if os.path.exists(filepath):
            os.remove(filepath)

def generate_configs():
    lef_filename = "Nangate45.lef"
    lef_path = os.path.join(INPUT_DIR, lef_filename)

    download_file(LEF_URL, lef_path)

    print("\n[INFO] Processing Designs...")

    for name, data in DESIGNS.items():
        is_gz = data['url'].endswith(".gz")
        def_filename = f"{name}.def.gz" if is_gz else f"{name}.def"
        def_path = os.path.join(INPUT_DIR, def_filename)
        final_def_path = def_path[:-3] if is_gz else def_path

        output_def_path = os.path.join(OUTPUT_DIR, f"{name}_placed.def")

        print(f"   -> Processing {name} [{data['description']}]")
        download_file(data['url'], def_path)

        if os.path.exists(final_def_path):
            config_content = CONFIG_TEMPLATE.format(
                bin_dimension=data['bin_dimension'],
                lef_file=lef_path,
                def_file=final_def_path,
                output_file=output_def_path
            )

            with open(data['config_file'], "w") as f:
                f.write(config_content)

            print(f"      Generated config: {data['config_file']}")
        else:
            print(f"      [SKIP] Could not generate config for {name} (download failed)")

def main():
    setup_directories()
    generate_configs()

    print("\n[SUCCESS] Benchmark setup complete.")
    print("You can now run the tool with increasing difficulty:")
    print("-" * 50)
    print(f"1. cargo run --release -- --config config_gcd.toml flow")
    print(f"2. !!WORK IN PROGRESS!! cargo run --release -- --config config_aes.toml flow")
    print("-" * 50)

if __name__ == "__main__":
    main()
