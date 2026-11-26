#!/usr/bin/env python3
"""Mutation-test the changes since a base: every mutant cargo mutants makes in them must fail a test.

    scripts/mutants_diff.py [BASE]

BASE defaults to origin/main; the changes are `git diff BASE...HEAD`. Each mutant runs its own
crate's tests first, and the ones that survive run again under the whole workspace's, since a test
in a crate built on the mutated one catches a mutant just as well. Outcomes go to
target/mutants-diff/mutants.out (missed.txt lists the survivors). Exits as cargo mutants does: 0
when every mutant was caught, 2 when some were missed, 3 when some timed out.
"""

import pathlib
import subprocess
import sys

OUTPUT = pathlib.Path("target/mutants-diff")
SURVIVED = (2, 3)
# cargo mutants times its baseline on the mutated crate's tests even when each mutant runs the
# whole workspace's, so the timeout it derives from that baseline is far too short there.
WORKSPACE_TEST_TIMEOUT_S = 600


def changes_since(base):
    """The diff of HEAD against the commit where it left base."""
    diff = ["git", "diff", f"{base}...HEAD"]
    return subprocess.run(diff, check=True, capture_output=True, text=True).stdout


def mutants(changes, *extra):
    """cargo mutants over the mutants in the diff file changes; its exit status."""
    command = ["cargo", "mutants", "--in-diff", str(changes), "--output", str(OUTPUT), *extra]
    return subprocess.run(command, check=False).returncode


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "origin/main"
    OUTPUT.mkdir(parents=True, exist_ok=True)
    changes = OUTPUT / "changes.diff"
    changes.write_text(changes_since(base))
    status = mutants(changes)
    if status in SURVIVED:
        status = mutants(
            changes,
            "--iterate",
            "--test-workspace=true",
            f"--minimum-test-timeout={WORKSPACE_TEST_TIMEOUT_S}",
        )
    sys.exit(status)


main()
