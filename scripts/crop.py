#!/usr/bin/env python3
"""Crop a placed DEF to a box so that a window of a design can be routed and
checked on its own in minutes.

    scripts/crop.py placed.def cells.lef X0 Y0 X1 Y1 out.def [--partial | --boundary-pins --tech-lef tech.lef]

The box is in microns. The crop keeps the cells lying wholly inside the box,
macros crossing its edge as FIXED, the I/O pins inside it, the nets whose
pins are all among those, and the rows, tracks, vias, non-default rules,
blockages and special-net segments clipped to the box. Region references
are dropped, so fences do not constrain the window.

A net with pins both inside and outside is dropped by default. With
--partial it is kept with only its inside pins. With --boundary-pins it is
kept with an I/O pin on each box edge its outside pins lie beyond, where the
line from the inside pins toward those pins leaves the box, on a routing
layer that runs across that edge and on a free track of it, so that the
window carries the routing the crossing nets demand of it.
"""

import argparse
import re
import sys

ROW_FIELDS = 12

# Boundary pins: layer by die edge, size along and across the edge in DBU.
BOUNDARY_PIN_LENGTH = 400
BOUNDARY_PIN_WIDTH = 100


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


def pin_positions(lines, sizes, lef, units):
    """Absolute pin centers by (component, pin) and (PIN, name), from the DEF
    components and the LEF pin shapes (the shape's center, N orientation
    mirrored as the component says)."""
    pin_rects = {}
    for m in re.finditer(r"MACRO (\S+)(.*?)END \1", lef, re.S):
        master, body = m.group(1), m.group(2)
        for pm in re.finditer(r"PIN (\S+)(.*?)END \2", body, re.S):
            rects = [tuple(float(v) for v in r.groups()) for r in re.finditer(r"RECT ([\d.]+) ([\d.]+) ([\d.]+) ([\d.]+)", pm.group(2))]
            if rects:
                pin_rects[(master, pm.group(1))] = rects[0]
    comps = {}
    cs, ce = section(lines, "COMPONENTS")
    for l in lines[cs + 1 : ce]:
        m = re.match(r"- (\S+) (\S+)\s+\+ (?:PLACED|FIXED) \( (-?\d+) (-?\d+) \) (\S+)", l)
        if m:
            comps[m.group(1)] = (m.group(2), int(m.group(3)), int(m.group(4)), m.group(5))

    def position(comp, pin):
        if comp == "PIN":
            return io_positions.get(pin)
        master, x, y, orient = comps.get(comp, (None, 0, 0, "N"))
        if master is None:
            return None
        w, h = sizes.get(master, (0, 0))
        r = pin_rects.get((master, pin))
        if r is None:
            return (x + w // 2, y + h // 2)
        cx, cy = int(round((r[0] + r[2]) / 2 * units)), int(round((r[1] + r[3]) / 2 * units))
        if orient in ("FN", "S"):
            cx = w - cx
        if orient in ("FS", "S"):
            cy = h - cy
        return (x + cx, y + cy)

    io_positions = {}
    ps, pe = section(lines, "PINS")
    name = None
    for l in lines[ps + 1 : pe]:
        if l.startswith(("- ", "   - ")):
            name = l.split()[1]
        m = re.search(r"PLACED \( (-?\d+) (-?\d+) \)", l)
        if m and name:
            io_positions[name] = (int(m.group(1)), int(m.group(2)))
    return position


def special_rects(lines, box):
    """The boxes of the special-net segments touching `box`, with their layer."""
    bx0, by0, bx1, by1 = box
    out = []
    ss, se = section(lines, "SPECIALNETS")
    for l in lines[ss + 1 : se]:
        m = re.match(r"\s*(?:\+ ROUTED|NEW)\s+(\S+) (\d+)(.*)", l)
        if not m:
            continue
        layer, half = m.group(1), int(m.group(2)) // 2
        pts = [(int(a), int(b)) for a, b in re.findall(r"\( (-?\d+) (-?\d+) \)", m.group(3))]
        if len(pts) < 2:
            continue
        x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
        y0, y1 = min(p[1] for p in pts), max(p[1] for p in pts)
        r = (x0 - half, y0 - half, x1 + half, y1 + half) if y0 == y1 else (x0 - half, y0, x1 + half, y1)
        if y0 == y1:
            r = (x0, y0 - half, x1, y1 + half)
        if r[0] < bx1 and r[2] > bx0 and r[1] < by1 and r[3] > by0:
            out.append((layer, r))
    return out


def blockages_in(lines, box):
    """Each blockage of the BLOCKAGES section as its head (the text before
    its shapes) with its rectangles clipped to `box`, dropping those that
    miss it."""
    try:
        start, end = section(lines, "BLOCKAGES")
    except StopIteration:
        return []
    bx0, by0, bx1, by1 = box
    text = " ".join(lines[start + 1 : end])
    out = []
    for block in re.findall(r"-\s+(.*?);", text):
        head = block.split("RECT")[0].split("POLYGON")[0].strip()
        rects = []
        for r in re.finditer(r"RECT \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)", block):
            x0, y0, x1, y1 = (int(v) for v in r.groups())
            x0, x1 = sorted((x0, x1))
            y0, y1 = sorted((y0, y1))
            cx0, cy0, cx1, cy1 = max(x0, bx0), max(y0, by0), min(x1, bx1), min(y1, by1)
            if cx0 < cx1 and cy0 < cy1:
                rects.append((cx0, cy0, cx1, cy1))
        if rects:
            out.append((head, rects))
    return out


def clipped_blockages(lines, box):
    """The BLOCKAGES section with each blockage's rectangles clipped to
    `box`, dropping those that miss it."""
    kept = [
        f"- {head} " + " ".join(f"RECT ( {x0} {y0} ) ( {x1} {y1} )" for x0, y0, x1, y1 in rects) + " ;"
        for head, rects in blockages_in(lines, box)
    ]
    if not kept:
        return []
    return [f"BLOCKAGES {len(kept)} ;"] + kept + ["END BLOCKAGES", ""]


def covers(rects, box):
    """Whether the union of `rects` covers all of `box`."""
    bx0, by0, bx1, by1 = box
    xs = sorted({bx0, bx1} | {v for r in rects for v in (r[0], r[2]) if bx0 < v < bx1})
    ys = sorted({by0, by1} | {v for r in rects for v in (r[1], r[3]) if by0 < v < by1})
    for xa, xb in zip(xs, xs[1:]):
        for ya, yb in zip(ys, ys[1:]):
            cx, cy = (xa + xb) / 2, (ya + yb) / 2
            if not any(r[0] <= cx <= r[2] and r[1] <= cy <= r[3] for r in rects):
                return False
    return True


def fully_blocked_layers(lines, box):
    """Layers that routing blockages cover across all of `box`."""
    per_layer = {}
    for head, rects in blockages_in(lines, box):
        m = re.match(r"LAYER (\S+)", head)
        if m:
            per_layer.setdefault(m.group(1), []).extend(rects)
    return {layer for layer, rects in per_layer.items() if covers(rects, box)}


def layer_directions(lef):
    """Routing layers in LEF order with their preferred direction."""
    out = []
    for m in re.finditer(r"LAYER (\S+)\s+TYPE\s+ROUTING\s*;(.*?)END \1", lef, re.S):
        d = re.search(r"DIRECTION\s+(\w+)", m.group(2))
        out.append((m.group(1), d.group(1) if d else "HORIZONTAL"))
    return out


class BoundaryPins:
    """I/O pins on the box edges for nets that leave the box."""

    def __init__(self, box, lef, lines, blockers, die, min_layer_index=1):
        """Pins go on the routing layers from `min_layer_index` up to the
        lowest one a blockage covers across the box: a via cannot pass a
        layer blocked there, so a crossing net never reaches those above."""
        self.bx0, self.by0, self.bx1, self.by1 = box
        self.blockers = blockers
        dx0, dy0, dx1, dy1 = die
        self.die_edges = {
            edge
            for edge, same in (
                ("left", self.bx0 <= dx0),
                ("right", self.bx1 >= dx1),
                ("bottom", self.by0 <= dy0),
                ("top", self.by1 >= dy1),
            )
            if same
        }
        layers = layer_directions(lef)[min_layer_index:]
        blocked = fully_blocked_layers(lines, box)
        reachable = next((i for i, (n, _) in enumerate(layers) if n in blocked), len(layers))
        layers = layers[:reachable]
        self.horizontal = [n for n, d in layers if d == "HORIZONTAL"]
        self.vertical = [n for n, d in layers if d == "VERTICAL"]
        self.tracks = {}
        for l in lines:
            m = re.match(r"TRACKS ([XY]) (-?\d+) DO (\d+) STEP (\d+) LAYER (.*?) ;", l)
            if m:
                for layer in m.group(5).split():
                    self.tracks[(layer, m.group(1))] = (int(m.group(2)), int(m.group(4)))
        self.taken = set()
        self.pins = []
        self.count = 0

    def snap(self, layer, axis, value):
        start, step = self.tracks.get((layer, axis), (0, 200))
        k = max(0, round((value - start) / step))
        return start + k * step

    def exit(self, inside, outside):
        """The edge and point where the line from `inside` to `outside`
        leaves the box, if it does."""
        (ix, iy), (ox, oy) = inside, outside
        dx, dy = ox - ix, oy - iy
        best = None
        for edge, t in (
            ("left", (self.bx0 - ix) / dx if dx else None),
            ("right", (self.bx1 - ix) / dx if dx else None),
            ("bottom", (self.by0 - iy) / dy if dy else None),
            ("top", (self.by1 - iy) / dy if dy else None),
        ):
            if t is not None and 0 <= t <= 1 and (best is None or t < best[1]):
                best = (edge, t)
        if best is None or best[0] in self.die_edges:
            return None
        edge, t = best
        return edge, (ix + dx * t, iy + dy * t)

    def add(self, net, edge, x, y):
        """A pin for `net` on `edge` at the point nearest (`x`, `y`)."""
        layers = self.horizontal if edge in ("left", "right") else self.vertical
        if not layers:
            return
        layer = layers[self.count % len(layers)]
        self.count += 1
        along_axis = "Y" if edge in ("left", "right") else "X"
        along = self.snap(layer, along_axis, y if along_axis == "Y" else x)
        start, step = self.tracks.get((layer, along_axis), (0, 200))
        half = BOUNDARY_PIN_WIDTH // 2

        def rect(along):
            if edge == "left":
                return self.bx0, along - half, BOUNDARY_PIN_LENGTH, BOUNDARY_PIN_WIDTH
            if edge == "right":
                return self.bx1 - BOUNDARY_PIN_LENGTH, along - half, BOUNDARY_PIN_LENGTH, BOUNDARY_PIN_WIDTH
            if edge == "bottom":
                return along - half, self.by0, BOUNDARY_PIN_WIDTH, BOUNDARY_PIN_LENGTH
            return along - half, self.by1 - BOUNDARY_PIN_LENGTH, BOUNDARY_PIN_WIDTH, BOUNDARY_PIN_LENGTH

        def blocked(along):
            px, py, w, h = rect(along)
            margin = BOUNDARY_PIN_WIDTH
            return (layer, edge, along) in self.taken or any(
                bl == layer and px - margin < x1 and px + w + margin > x0 and py - margin < y1 and py + h + margin > y0
                for bl, (x0, y0, x1, y1) in self.blockers
            )

        for tries in range(1, 400):
            if not blocked(along):
                break
            along += step if tries % 2 else -step * tries
        self.taken.add((layer, edge, along))
        px, py, w, h = rect(along)
        name = f"{net}_bp{len(self.pins)}"
        self.pins.append((name, net, layer, px, py, w, h))
        return name

    def def_lines(self):
        out = []
        for name, net, layer, px, py, w, h in self.pins:
            out.append(f"- {name} + NET {net} + DIRECTION INOUT + USE SIGNAL")
            out.append(f"  + LAYER {layer} ( 0 0 ) ( {w} {h} )")
            out.append(f"  + PLACED ( {px} {py} ) N ;")
        return out


def crop(src, lef_path, box, out, partial, boundary, tech_lef):
    txt = open(src).read()
    units = int(re.search(r"UNITS DISTANCE MICRONS (\d+)", txt).group(1))
    bx0, by0, bx1, by1 = (int(round(v * units)) for v in box)
    lef = open(lef_path).read()
    sizes_um, blocks = macro_table(lef)
    sizes = {k: (int(round(w * units)), int(round(h * units))) for k, (w, h) in sizes_um.items()}
    lines = txt.split("\n")
    position = pin_positions(lines, sizes, lef, units) if boundary else None
    boundary_pins = None
    if boundary:
        if tech_lef is None:
            sys.exit("--boundary-pins needs --tech-lef for the routing layers' directions")
        blockers = special_rects(lines, (bx0, by0, bx1, by1))
        die = re.search(r"DIEAREA \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)", txt)
        die = tuple(int(v) for v in die.groups()) if die else (bx0, by0, bx1, by1)
        boundary_pins = BoundaryPins((bx0, by0, bx1, by1), open(tech_lef).read(), lines, blockers, die)

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
    if boundary_pins:
        layer, rect = None, None
        for l in io_pins:
            m = re.search(r"LAYER (\S+) \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)", l)
            if m:
                layer, rect = m.group(1), tuple(int(v) for v in m.groups()[1:])
            m = re.search(r"PLACED \( (-?\d+) (-?\d+) \)", l)
            if m and layer:
                px, py = int(m.group(1)), int(m.group(2))
                boundary_pins.blockers.append((layer, (px + rect[0], py + rect[1], px + rect[2], py + rect[3])))

    ns, ne = section(lines, "NETS")
    nets = []
    for l in lines[ns + 1 : ne]:
        if not l.startswith("- "):
            continue
        conns = re.findall(r"\( (\S+) (\S+) \)", l)
        inside = [
            (c, p) for c, p in conns if (c != "PIN" and c in kept) or (c == "PIN" and p in io_kept)
        ]
        net_head = l.split(" (")[0]
        net_tail = l[l.rfind(")") + 1 :]
        if conns and len(inside) == len(conns):
            nets.append(l)
        elif partial and len(inside) >= 2:
            nets.append(net_head + "".join(f" ( {c} {p} )" for c, p in inside) + net_tail)
        elif boundary and inside:
            net = net_head.split()[1]
            centers = [position(c, p) for c, p in inside]
            centers = [c for c in centers if c]
            if not centers:
                continue
            centroid = (sum(c[0] for c in centers) // len(centers), sum(c[1] for c in centers) // len(centers))
            exits = {}
            for c, p in conns:
                if (c, p) in inside:
                    continue
                outside = position(c, p)
                found = boundary_pins.exit(centroid, outside) if outside else None
                if found:
                    exits.setdefault(found[0], []).append(found[1])
            added = []
            for edge, points in exits.items():
                x = sum(q[0] for q in points) / len(points)
                y = sum(q[1] for q in points) / len(points)
                name = boundary_pins.add(net, edge, x, y)
                if name:
                    added.append(name)
            if added:
                conns_text = "".join(f" ( {c} {p} )" for c, p in inside) + "".join(f" ( PIN {n} )" for n in added)
                nets.append(net_head + conns_text + net_tail)
    if boundary_pins:
        io_pins.extend(boundary_pins.def_lines())
        io_kept.update(p[0] for p in boundary_pins.pins)

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
    kept_sections += clipped_blockages(lines, (bx0, by0, bx1, by1))

    out_lines = head + [""] + kept_sections
    out_lines += [f"COMPONENTS {len(comps)} ;"] + comps + ["END COMPONENTS", ""]
    out_lines += [f"PINS {len(io_kept)} ;"] + io_pins + ["END PINS", ""]
    special_count = sum(1 for l in special if l.startswith("- "))
    out_lines += [f"SPECIALNETS {special_count} ;"] + special + ["END SPECIALNETS", ""]
    out_lines += [f"NETS {len(nets)} ;"] + nets + ["END NETS", "", "END DESIGN", ""]
    open(out, "w").write("\n".join(out_lines))
    segments = sum(1 for l in special if not l.startswith("- "))
    added = len(boundary_pins.pins) if boundary_pins else 0
    print(
        f"kept {len(comps)} cells ({edge_macros} edge macros fixed), {len(io_kept) - added} I/O pins, "
        f"{added} boundary pins, {len(nets)} nets, {segments} PG segments"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("placed_def")
    parser.add_argument("cells_lef")
    parser.add_argument("box", nargs=4, type=float, metavar=("X0", "Y0", "X1", "Y1"))
    parser.add_argument("out_def")
    parser.add_argument("--partial", action="store_true", help="keep nets with two or more pins inside, dropping the rest")
    parser.add_argument("--boundary-pins", action="store_true", help="keep crossing nets with a pin on the box edge per outside pin")
    parser.add_argument("--tech-lef", help="technology LEF, for the layer directions boundary pins need")
    args = parser.parse_args()
    crop(args.placed_def, args.cells_lef, args.box, args.out_def, args.partial, args.boundary_pins, args.tech_lef)


if __name__ == "__main__":
    sys.exit(main())
