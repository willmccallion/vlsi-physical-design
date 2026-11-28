#!/usr/bin/env python3
"""Mutation-test what a pull request touches, as .github/workflows/mutants-pr.yml does.

    scripts/mutants_diff.py lines BASE
    scripts/mutants_diff.py files BASE
    scripts/mutants_diff.py report SHARDS_DIR

As Google runs mutation testing at code review: one mutant per line, each run against its own
crate's tests, and the survivors shown as advice rather than a gate. `lines` takes the lines
`git diff BASE...HEAD` changes; `files`, for a pull request that fixes a bug, takes every line of
the files it changes. Either keeps the first mutant cargo mutants lists on each line, spreads a
capped pick across them when there are more, writes them to mutants-targets.txt with a
mutants-run.env for scripts/mutants_shard.py, and prints `count=N` and `shards=[...]` for
$GITHUB_OUTPUT. `report` reads the shards' outcomes, prints a warning on the line of each
surviving mutant, adds a summary to the file $GITHUB_STEP_SUMMARY names (stdout without it), and
exits 0 whatever survived.
"""

import os
import pathlib
import subprocess
import sys
from dataclasses import dataclass

TARGETS = pathlib.Path("mutants-targets.txt")
RUN_CONFIG = pathlib.Path("mutants-run.env")
MUTANTS_PER_SHARD = 13
# A mutant that runs this long has turned a loop endless, and counts as caught.
MINIMUM_TEST_TIMEOUT_S = 300
COMMON = ["--gitignore", "true", "--cap-lints", "true"]


@dataclass(frozen=True)
class Scope:
    """How much of a pull request is mutated, and the most mutants and shards it may take."""

    max_mutants: int
    max_shards: int


# The router's tests take about twenty minutes a mutant on a runner: these keep the slowest
# shard within a job's six hours.
CHANGED_LINES = Scope(max_mutants=100, max_shards=8)
CHANGED_FILES = Scope(max_mutants=300, max_shards=16)


def git(*args):
    return subprocess.run(["git", *args], check=True, capture_output=True, text=True).stdout


def listed(*selection):
    """The names of the mutants cargo mutants makes in the selected code."""
    command = ["cargo", "mutants", "--list", "--workspace", *selection, *COMMON]
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout.splitlines()


def mutants_on_changed_lines(base):
    diff = git("diff", f"{base}...HEAD", "--", "*.rs")
    if not diff:
        return []
    changes = pathlib.Path("target/changes.diff")
    changes.parent.mkdir(exist_ok=True)
    changes.write_text(diff)
    return listed("--in-diff", str(changes))


def mutants_in_changed_files(base):
    paths = git("diff", "--name-only", f"{base}...HEAD", "--", "*.rs").split()
    existing = [p for p in paths if pathlib.Path(p).exists()]
    if not existing:
        return []
    return listed(*(arg for p in existing for arg in ("--file", p)))


def line_of(name):
    """The file and line a mutant's name starts with, `crates/x/src/y.rs:12`."""
    path, line, _ = name.split(":", 2)
    return f"{path}:{line}"


def one_per_line(names):
    """The first mutant on each line, in the order cargo mutants lists them."""
    seen = set()
    kept = []
    for name in names:
        if line_of(name) not in seen:
            seen.add(line_of(name))
            kept.append(name)
    return kept


def spread(names, limit):
    """At most limit of names, evenly spaced through them so all the code is sampled."""
    if len(names) <= limit:
        return names
    return [names[i * len(names) // limit] for i in range(limit)]


def plan(names, scope):
    targets = spread(one_per_line(names), scope.max_mutants)
    TARGETS.write_text("".join(f"{name}\n" for name in targets))
    RUN_CONFIG.write_text(
        "# What the pull request touches, one mutant a line, against each crate's own tests.\n"
        f"MUTANTS_ARGS=--workspace --minimum-test-timeout {MINIMUM_TEST_TIMEOUT_S} --jobs 2\n"
        f"MUTANTS_TARGETS={TARGETS}\n"
    )
    shards = min(scope.max_shards, -(-len(targets) // MUTANTS_PER_SHARD))
    print(f"count={len(targets)}")
    print(f"shards={list(range(shards))}")


def outcomes(shards_dir, outcome):
    """Every shard's mutants of one outcome, sorted."""
    files = pathlib.Path(shards_dir).glob(f"*/{outcome}.txt")
    return sorted(name for f in files for name in f.read_text().splitlines() if name)


def summary(counts, missed):
    """The run's counts and its survivors, as Markdown."""
    text = (
        "### Mutants on what the pull request touches\n\n"
        f"{counts['caught']} caught, {counts['timeout']} timed out (caught: a loop never ends), "
        f"{len(missed)} survived, {counts['unviable']} did not build.\n\n"
    )
    if missed:
        text += "Each survivor is advice: a test may be missing, or the mutant may change nothing.\n\n"
        text += "".join(f"- `{name}`\n" for name in missed)
    return text


def report(shards_dir):
    missed = outcomes(shards_dir, "missed")
    counts = {o: len(outcomes(shards_dir, o)) for o in ("caught", "timeout", "unviable")}
    for name in missed:
        path, line, rest = name.split(":", 2)
        mutation = rest.split(": ", 1)[1]
        print(f"::warning file={path},line={line},title=Surviving mutant::{mutation}")
    destination = os.environ.get("GITHUB_STEP_SUMMARY")
    if destination:
        with open(destination, "a", encoding="utf-8") as out:
            out.write(summary(counts, missed))
    else:
        print(summary(counts, missed))


def main():
    command, argument = sys.argv[1], sys.argv[2]
    if command == "lines":
        plan(mutants_on_changed_lines(argument), CHANGED_LINES)
    elif command == "files":
        plan(mutants_in_changed_files(argument), CHANGED_FILES)
    elif command == "report":
        report(argument)
    else:
        sys.exit(f"unknown command {command}: lines BASE, files BASE or report SHARDS_DIR")


main()
