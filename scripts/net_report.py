#!/usr/bin/env python3
"""How each router routed one net of a compared design, and how slow it is.

Usage:
    python3 scripts/net_report.py ibex rst_ni

For both routings in output/compare/<design>/ it reports the net's wirelength
per layer and its vias from the routed DEF, and the Elmore delay from its
driver to its loads over the parasitics the timing run extracted, with the
loads' pin capacitances from the liberty files. The Elmore delay is the
first-moment estimate, not the timer's, but it is computed the same way for
both routings.
"""

import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import compare

UNIT_SCALE = {"FF": 1e-15, "PF": 1e-12, "OHM": 1.0, "KOHM": 1e3}


def net_block(text, header, net):
    """The lines of `net` in the section `header` opens, pin list included."""
    section = text[text.index(f"\n{header} ") :]
    start = re.search(rf"^\s*- {re.escape(net)}(?=\s)", section, re.MULTILINE).start()
    end = re.compile(r";\s*$", re.MULTILINE).search(section, start)
    return section[start : end.end()]


def routed_wires(def_file, net):
    """Wirelength per layer in microns, the via count and the segment count."""
    text = def_file.read_text(errors="replace")
    dbu = int(re.search(r"^UNITS DISTANCE MICRONS (\d+)", text, re.MULTILINE).group(1))
    block = net_block(text, "NETS", net)
    routing = block[block.index("+ ROUTED") :] if "+ ROUTED" in block else ""
    tokens = re.findall(
        r"\(\s*(-?\d+|\*)\s+(-?\d+|\*)(?:\s+-?\d+)?\s*\)|(\S+)", routing
    )
    length = Counter()
    vias = 0
    segments = 0
    layer = None
    prev = None
    expect_layer = False
    skip = 0
    for x, y, word in tokens:
        if skip:
            skip -= 1
            continue
        if word:
            if word in ("+", ";"):
                continue
            if word in ("ROUTED", "NEW"):
                expect_layer = True
                prev = None
                continue
            if expect_layer:
                layer = word
                expect_layer = False
                continue
            if word in ("TAPERRULE", "RECT"):
                skip = 1
                continue
            vias += 1
            continue
        if prev is None:
            prev = (x, y)
            continue
        px, py = prev
        cx = px if x == "*" else x
        cy = py if y == "*" else y
        length[layer] += abs(int(cx) - int(px)) + abs(int(cy) - int(py))
        segments += 1
        prev = (cx, cy)
    return {layer: n / dbu for layer, n in length.items()}, vias, segments


def pin_capacitances(lib_files):
    """(cell, pin) -> capacitance in farads, from the liberty files."""
    caps = {}
    for lib in lib_files:
        text = Path(lib).read_text(errors="replace")
        unit = re.search(r"capacitive_load_unit\s*\(\s*([\d.]+)\s*,\s*(\w+)\s*\)", text)
        scale = float(unit.group(1)) * UNIT_SCALE[unit.group(2).upper()]
        cell = None
        pin = None
        for line in text.splitlines():
            m = re.match(r"\s*cell\s*\(\s*\"?([^\")]+)\"?\s*\)", line)
            if m:
                cell = m.group(1)
                continue
            m = re.match(r"\s*pin\s*\(\s*\"?([^\")]+)\"?\s*\)", line)
            if m:
                pin = m.group(1)
                continue
            m = re.match(r"\s*capacitance\s*:\s*([\d.eE+-]+)", line)
            if m and cell and pin:
                caps[(cell, pin)] = float(m.group(1)) * scale
    return caps


def elmore_delays(spef_file, net, pin_caps):
    """Elmore delay in seconds from the net's port or driving pin to each load."""
    text = spef_file.read_text(errors="replace")
    c_unit = re.search(r"^\*C_UNIT\s+([\d.]+)\s+(\w+)", text, re.MULTILINE)
    r_unit = re.search(r"^\*R_UNIT\s+([\d.]+)\s+(\w+)", text, re.MULTILINE)
    c_scale = float(c_unit.group(1)) * UNIT_SCALE[c_unit.group(2).upper()]
    r_scale = float(r_unit.group(1)) * UNIT_SCALE[r_unit.group(2).upper()]
    index = re.search(rf"^(\*\d+) {re.escape(net)}$", text, re.MULTILINE).group(1)
    start = text.index(f"\n*D_NET {index} ") + 1
    block = text[start : text.index("\n*END", start)]
    cap = defaultdict(float)
    adjacent = defaultdict(list)
    loads = []
    driver = None
    section = None
    for line in block.splitlines()[1:]:
        if line.startswith(("*CONN", "*CAP", "*RES")):
            section = line.split()[0]
            continue
        f = line.split()
        if section == "*CONN":
            direction = f[2]
            if direction == "I" and f[0] == "*I":
                cell = f[f.index("*D") + 1]
                loads.append(f[1])
                cap[f[1]] += pin_caps[(cell, f[1].rsplit(":", 1)[1])]
            elif direction == "I" and f[0] == "*P" or direction == "O" and f[0] == "*I":
                driver = f[1]
        elif section == "*CAP" and len(f) == 3:
            cap[f[1]] += float(f[2]) * c_scale
        elif section == "*RES" and len(f) == 4:
            r = float(f[3]) * r_scale
            adjacent[f[1]].append((f[2], r))
            adjacent[f[2]].append((f[1], r))
    parent = {driver: None}
    order = []
    stack = [driver]
    while stack:
        node = stack.pop()
        order.append(node)
        for other, r in adjacent[node]:
            if other not in parent:
                parent[other] = (node, r)
                stack.append(other)
    downstream = dict(cap)
    for node in reversed(order):
        if parent[node]:
            downstream[parent[node][0]] += downstream[node]
    delay = {driver: 0.0}
    for node in order[1:]:
        up, r = parent[node]
        delay[node] = delay[up] + r * downstream[node]
    return [delay[load] for load in loads if load in delay], len(loads)


def report(design, side, net, pin_caps):
    length, vias, segments = routed_wires(compare.routed_def(design, side), net)
    total = sum(length.values())
    print(f"{design} {net} routed by {'PARE' if side == 'pare' else 'OpenROAD'}:")
    print(f"  {total:,.0f} um of wire, {vias:,} vias, {segments:,} segments")
    for layer in sorted(length, key=lambda name: int(re.sub(r"\D", "", name) or 0)):
        print(
            f"    {layer:<8} {length[layer]:>10,.0f} um  {100 * length[layer] / total:5.1f}%"
        )
    spef = compare.design_dir(design) / f"{compare.routed_def(design, side).stem}.spef"
    if not spef.exists():
        return
    delays, loads = elmore_delays(spef, net, pin_caps)
    delays.sort()
    print(
        f"  Elmore delay to its {loads:,} loads ({len(delays):,} reached): "
        f"worst {delays[-1] * 1e9:.2f} ns, median {delays[len(delays) // 2] * 1e9:.2f} ns"
    )


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    design, net = sys.argv[1:]
    pin_caps = pin_capacitances(compare.liberty_files(design))
    for side in compare.SIDES:
        report(design, side, net, pin_caps)


if __name__ == "__main__":
    main()
