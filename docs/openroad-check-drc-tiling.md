# OpenROAD `check_drc` reports spacing markers that are not there

PARE judges its routing with OpenROAD's `drt::check_drc`. On 2026-10-18,
with PARE at commit `9c4aac0` and OpenROAD 26Q2 from nixpkgs, `check_drc`
reported markers on designs PARE's own checker found clean:

| Design | PARE | `check_drc` (stock) |
|---|---|---|
| aes | 0 | 3 |
| ibex | 0 | 5 |
| jpeg | 0 | 39 |

Every one was a "Metal Spacing" marker with a single net as its source,
between two via pads of that net on one track, 0.005 to 0.05 um apart, where
PARE's checker found the gap filled by metal of the same net.

This is OpenROAD issue
[#11697](https://github.com/The-OpenROAD-Project/OpenROAD/issues/11697),
reported independently on 2026-10-10, with a proposed fix in
[#11698](https://github.com/The-OpenROAD-Project/OpenROAD/pull/11698).

## Cause

`TritonRoute::getDRCMarkers` checks the design in tiles of 7x7 gcells. Each
tile's worker loads the routed objects meeting the tile grown by
`MTSAFEDIST` (2000 database units, `src/drt/src/drt-global.h`). Routed vias
are found by their whole bounding box, which reaches as far as their widest
pad, and wires by their own. A worker at a tile edge can therefore load two
vias of a net but not the wire that joins them, and it sees a gap between
their pads. The markers of every worker were kept if they fell inside the
requested area, not only those inside the worker's own check region, so
that phantom gap was reported.

## Evidence

All on the aes floorplan (2000 units per micron, gcell step 5700), with a
net reduced to two `via2_5` 380 units apart on the metal3 track at y = 476700
and a metal3 wire between them:

1. `check_drc` reports one Metal Spacing marker between the via pads.
   Removing the wire, extending it far past both vias, or splitting it at
   the vias changes nothing; replacing one via by a metal3 rectangle of the
   same size removes the marker.
2. Moving the two vias and the wire sideways keeps the marker; moving them
   up by one track pitch (280 units), or by 1400, 2800 or 5600, removes it.
3. Keeping the geometry and changing only `GCELLGRID Y` from step 5700 to
   5600 or 5800 removes it. A gcell grid has no bearing on spacing.
4. Prediction: with `GCELLGRID Y` step 5588 a tile edge falls at y = 508508,
   which should flag the pair of net `_00006_` on the track at y = 506380
   (vias at x = 637330 and 637710), one of 259 such pairs on aes that stock
   `check_drc` did not flag. Run on the full routed aes design with only that
   grid change, `check_drc` reported exactly that marker,
   (318.7350, 253.1550) - (318.7850, 253.2250) on metal3, and the three
   original markers disappeared.

With the fix applied (below), `check_drc` reports 0 on the routed aes,
ibex and jpeg of the table above, 0 for the two vias joined by the wire, and
still 1 for the same two vias without the wire, where the gap is real.

## What PARE does about it

`nix/openroad-check-drc-worker-region.patch` applies the source change of
#11698 to the OpenROAD that `nix develop .#eda` builds: a worker's markers
are kept only inside its own check region, as OpenROAD's routing workers
already do. Results in the README are checked with that build, and the
count from stock OpenROAD is given beside them while the fix is not
upstream.
