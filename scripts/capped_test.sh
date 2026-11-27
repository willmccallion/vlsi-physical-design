#!/usr/bin/env bash
# capped_test.sh BINARY [ARGS...]: runs a test binary, and whatever it starts, in a systemd scope of
# its own capped at 2 GB without swap, as the runner user. A mutant that makes a loop allocate
# without bound then fails its test alone. scripts/mutants_shard.py has cargo run every test
# binary through it; it needs passwordless sudo, as GitHub's runners have.
exec sudo --preserve-env systemd-run --scope --quiet --uid "$(id -u)" --gid "$(id -g)" \
  -p MemoryMax=2G -p MemorySwapMax=0 -- "$@"
