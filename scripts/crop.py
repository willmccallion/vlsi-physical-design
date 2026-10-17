#!/usr/bin/env python3
"""Crop a placed DEF to a box so that a window of a design can be routed and
checked on its own in minutes.

    scripts/crop.py placed.def cells.lef X0 Y0 X1 Y1 out.def [--partial]

The box is in microns. The crop keeps the cells lying wholly inside the box,
macros crossing its edge as FIXED, the I/O pins inside it, the nets whose
pins are all among those (or, with --partial, every net with at least two
pins inside, dropping the pins outside), and the rows, tracks, vias,
non-default rules and special-net segments clipped to the box. Region
references are dropped, so fences do not constrain the window.
"""

import argparse
import re
import sys

ROW_FIELDS = 12


def macro_table(lef):
    sizes = {}
    blocks = set()
    for m in re.finditer(r"MACRO (\S+)(.*?)END \1", lef, re.S):
        name, body = m.group(1), m.group(2)
        size = re.search(r"SIZE\s+([\d.]+)\s+BY\s+([\d.]+)", body)
        if size:
            sizes[name] = (float(size.group(1)), float(size.group(2)))
        if "CLASS BLOCK" in body:
            blocks.add(name)
    return sizes, blocks


def section(lines, name):
    start = next(
        i for i, l in enumerate(lines) if l.startswith(name + " ") and l.rstrip().endswith(";")
    )
    end = next(i for i in range(start, len(lines)) if lines[i].startswith("END " + name))
    return start, end


def crop(src, lef_path, box, out, partial):
    txt = open(src).read()
    units = int(re.search(r"UNITS DISTANCE MICRONS (\d+)", txt).group(1))
    bx0, by0, bx1, by1 = (int(round(v * units)) for v in box)
    sizes_um, blocks = macro_table(open(lef_path).read())
    sizes = {k: (int(round(w * units)), int(round(h * units))) for k, (w, h) in sizes_um.items()}
    lines = txt.split("\n")

    head = []
    for l in lines:
        if l.startswith("DIEAREA"):
            head.append(f"DIEAREA ( {bx0} {by0} ) ( {bx1} {by1} ) ;")
            continue
        if l.startswith("ROW "):
            t = l.split()
            if len(t) < ROW_FIELDS:
                continue
            rx, ry, step, n = int(t[3]), int(t[4]), int(t[11]), int(t[7])
            if not by0 <= ry < by1:
                continue
            first = rx + max(0, -(-(bx0 - rx) // step)) * step
            count = (min(bx1, rx + n * step) - first) // step
            if count > 0:
                head.append(f"ROW {t[1]} {t[2]} {first} {ry} {t[5]} DO {count} BY 1 STEP {step} 0 ;")
            continue
        if l.startswith(("TRACKS", "VERSION", "DIVIDERCHAR", "BUSBITCHARS", "DESIGN", "UNITS", "#")):
            head.append(l)
            continue
        if l.startswith("COMPONENTS"):
            break

    cs, ce = section(lines, "COMPONENTS")
    comps, kept, edge_macros = [], set(), 0
    for l in lines[cs + 1 : ce]:
        m = re.match(r"- (\S+) (\S+)\s+\+ (PLACED|FIXED) \( (-?\d+) (-?\d+) \) (\S+)", l)
        if not m:
            continue
        w, h = sizes.get(m.group(2), (0, 0))
        if m.group(6) in ("E", "W", "FE", "FW"):
            w, h = h, w
        cx, cy = int(m.group(4)), int(m.group(5))
        inside = bx0 <= cx and cx + w <= bx1 and by0 <= cy and cy + h <= by1
        touches = cx < bx1 and cx + w > bx0 and cy < by1 and cy + h > by0
        line = re.sub(r"\s*\+ REGION \S+", "", l)
        if inside:
            comps.append(line)
            kept.add(m.group(1))
        elif touches and m.group(2) in blocks:
            comps.append(line.replace("+ PLACED", "+ FIXED"))
            kept.add(m.group(1))
            edge_macros += 1

    ps, pe = section(lines, "PINS")
    io_pins, io_kept, block = [], set(), []

    def flush_pin():
        if not block:
            return
        text = "\n".join(block)
        m = re.search(r"PLACED \( (-?\d+) (-?\d+) \)", text)
        name = block[0].split()[1]
        if m and bx0 <= int(m.group(1)) <= bx1 and by0 <= int(m.group(2)) <= by1:
            io_pins.extend(block)
            io_kept.add(name)

    for l in lines[ps + 1 : pe]:
        if l.startswith(("- ", "   - ")) and block:
            flush_pin()
            block = []
        if l.strip():
            block.append(l)
    flush_pin()

    ns, ne = section(lines, "NETS")
    nets = []
    for l in lines[ns + 1 : ne]:
        if not l.startswith("- "):
            continue
        conns = re.findall(r"\( (\S+) (\S+) \)", l)
        inside = [
            (c, p) for c, p in conns if (c != "PIN" and c in kept) or (c == "PIN" and p in io_kept)
        ]
        if conns and len(inside) == len(conns):
            nets.append(l)
        elif partial and len(inside) >= 2:
            net_head = l.split(" (")[0]
            net_tail = l[l.rfind(")") + 1 :]
            nets.append(net_head + "".join(f" ( {c} {p} )" for c, p in inside) + net_tail)

    ss, se = section(lines, "SPECIALNETS")
    special, cur = [], None

    def flush_special():
        if cur and cur["segs"]:
            special.append(cur["head"])
            for i, s in enumerate(cur["segs"]):
                special.append(("  + ROUTED " if i == 0 else "  NEW ") + s)
            special[-1] += " ;"

    for l in lines[ss + 1 : se]:
        if l.startswith("- "):
            flush_special()
            cur = {"head": l.rstrip(" ;"), "segs": []}
            continue
        m = re.match(r"\s*(?:\+ ROUTED|NEW)\s+(.*?)\s*;?\s*$", l)
        if not m or cur is None:
            continue
        body = m.group(1)
        pts = [(int(a), int(b)) for a, b in re.findall(r"\( (-?\d+) (-?\d+) \)", body)]
        if not pts:
            continue
        wm = re.match(r"\S+ (\d+)", body)
        half = int(wm.group(1)) // 2 if wm else 0
        sx0, sy0 = min(p[0] for p in pts) - half, min(p[1] for p in pts) - half
        sx1, sy1 = max(p[0] for p in pts) + half, max(p[1] for p in pts) + half
        if sx0 < bx1 and sx1 > bx0 and sy0 < by1 and sy1 > by0:
            cur["segs"].append(body)
    flush_special()

    kept_sections = []
    for name in ("VIAS", "NONDEFAULTRULES"):
        try:
            a, b = section(lines, name)
            kept_sections += lines[a : b + 1] + [""]
        except StopIteration:
            pass

    out_lines = head + [""] + kept_sections
    out_lines += [f"COMPONENTS {len(comps)} ;"] + comps + ["END COMPONENTS", ""]
    out_lines += [f"PINS {len(io_kept)} ;"] + io_pins + ["END PINS", ""]
    special_count = sum(1 for l in special if l.startswith("- "))
    out_lines += [f"SPECIALNETS {special_count} ;"] + special + ["END SPECIALNETS", ""]
    out_lines += [f"NETS {len(nets)} ;"] + nets + ["END NETS", "", "END DESIGN", ""]
    open(out, "w").write("\n".join(out_lines))
    segments = sum(1 for l in special if not l.startswith("- "))
    print(
        f"kept {len(comps)} cells ({edge_macros} edge macros fixed), {len(io_kept)} I/O pins, "
        f"{len(nets)} nets, {segments} PG segments"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("placed_def")
    parser.add_argument("cells_lef")
    parser.add_argument("box", nargs=4, type=float, metavar=("X0", "Y0", "X1", "Y1"))
    parser.add_argument("out_def")
    parser.add_argument("--partial", action="store_true", help="keep nets with two or more pins inside, dropping the rest")
    args = parser.parse_args()
    crop(args.placed_def, args.cells_lef, args.box, args.out_def, args.partial)


if __name__ == "__main__":
    sys.exit(main())
