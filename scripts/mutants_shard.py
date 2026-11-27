#!/usr/bin/env python3
"""Run one shard of a mutation-testing run, as .github/workflows/mutants.yml does on each runner.

    scripts/mutants_shard.py K N

mutants-run.env at the root of the checkout, when there is one, sets MUTANTS_ARGS, the cargo
mutants arguments that pick the packages and tests (such as --workspace or --package), and
MUTANTS_TARGETS, a file listing the mutants to test by name; without a target list every mutant
is tested. Outcomes go to target/mutants-shard/mutants.out. Exits 0 whenever cargo mutants tested
its mutants, whatever it found, so every shard's outcomes are kept.
"""

import os
import pathlib
import shlex
import subprocess
import sys

OUTPUT = pathlib.Path("target/mutants-shard")
RUN_CONFIG = pathlib.Path("mutants-run.env")
# cargo mutants' exit statuses for a run that tested its mutants: every one caught, some missed,
# some timed out.
TESTED = (0, 2, 3)
# Each test binary runs through this, capped in memory: some mutants make a loop allocate without
# bound, and the cap fails their test instead of exhausting the runner.
TEST_RUNNER = pathlib.Path("scripts/capped_test.sh")
COMMON = ["--gitignore", "true", "--cap-lints", "true"]


def run_config():
    """The KEY=value settings of mutants-run.env, if the checkout has one."""
    if not RUN_CONFIG.exists():
        return {}
    lines = (line.strip() for line in RUN_CONFIG.read_text().splitlines())
    pairs = (line.split("=", 1) for line in lines if line and not line.startswith("#"))
    return {key.strip(): value.strip() for key, value in pairs}


def seed_caught(targets):
    """Marks every mutant the target list leaves out as caught, so --iterate skips it."""
    listing = ["cargo", "mutants", "--list", "--workspace", *COMMON]
    names = subprocess.run(listing, check=True, capture_output=True, text=True).stdout.splitlines()
    wanted = set(targets.read_text().splitlines())
    outcomes = OUTPUT / "mutants.out"
    outcomes.mkdir(exist_ok=True)
    others = (name for name in names if name not in wanted)
    (outcomes / "previously_caught.txt").write_text("".join(f"{name}\n" for name in others))


def main():
    shard, shards = sys.argv[1], sys.argv[2]
    OUTPUT.mkdir(parents=True, exist_ok=True)
    config = run_config()
    extra = shlex.split(config.get("MUTANTS_ARGS", ""))
    targets = config.get("MUTANTS_TARGETS")
    if targets:
        seed_caught(pathlib.Path(targets))
        extra.append("--iterate")
    command = [
        "cargo", "mutants", *COMMON, "--shard", f"{shard}/{shards}", "--output", str(OUTPUT), *extra
    ]
    runner = {"CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUNNER": str(TEST_RUNNER.resolve())}
    status = subprocess.run(command, check=False, env=os.environ | runner).returncode
    sys.exit(0 if status in TESTED else status)


main()
