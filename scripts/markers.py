#!/usr/bin/env python3
"""Histogram of the design rule markers in a PARE log by kind, layer and place.

    scripts/markers.py route.log [BIN_UM] [--nets N]

The log must come from a run with RUST_LOG=pare=debug, whose final check
prints one "drc: <Kind> on <layer> at (x0, y0) - (x1, y1) between A and B"
line per marker. BIN_UM sets the square bin for the location histogram
(default 40). --nets N also lists the N nets involved in most markers.
"""
import collections
import re
import sys

args = [a for a in sys.argv[1:] if not a.startswith("--")]
log = args[0]
b = float(args[1]) if len(args) > 1 else 40.0
nets_wanted = int(sys.argv[sys.argv.index("--nets") + 1]) if "--nets" in sys.argv else 0
pat = re.compile(r"drc: (.+?) on (\S+) at \(([\d.]+), ([\d.]+)\) - \(([\d.]+), ([\d.]+)\) between (\S+) and (\S+)")
kinds = collections.Counter(); layers = collections.Counter(); bins = collections.Counter(); nets = collections.Counter(); seen = 0
for line in open(log, errors="replace"):
    m = pat.search(line)
    if not m: continue
    seen += 1
    kind, layer, x0, y0, x1, y1, a, c = m.groups()
    kinds[kind] += 1; layers[layer] += 1; nets[a] += 1; nets[c] += 1
    cx = (float(x0) + float(x1)) / 2; cy = (float(y0) + float(y1)) / 2
    bins[(int(cx // b) * b, int(cy // b) * b)] += 1
print("markers:", seen)
print("by kind:", kinds.most_common())
print("by layer:", layers.most_common())
top = bins.most_common(12)
print(f"top {b:.0f} um bins (x, y, count):", [(x, y, n) for (x, y), n in top])
print("share in top 12 bins: %.2f" % (sum(n for _, n in top) / max(seen, 1)))
if nets_wanted:
    print("nets in most markers:", nets.most_common(nets_wanted))
