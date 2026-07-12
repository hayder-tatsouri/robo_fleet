#!/usr/bin/env python3
"""Rasterize the Novation City OSM data into a Nav2 occupancy grid.

Reads both building AND road models from `sousse_buildings.sdf` and produces a
three-tier costmap that makes PGuard *prefer* roads without being unable to
cross grass in emergencies.

Nav2 with `negate: 0` computes  p = (255 - pixel) / 255  and then:
    p >= occupied_thresh  -> LETHAL (100)
    p <= free_thresh      -> FREE (0)
    else                  -> NO_INFORMATION (-1, planner pays cost via static layer)

We stamp three grey levels chosen so those thresholds land where we want:

    pixel value | probability | tier              | outcome
    ------------+-------------+-------------------+--------------------
    0           |  1.00       | buildings         | LETHAL
    100         |  0.608      | grass / off-road  | NO_INFORMATION (~90 cost)
    254         |  0.004      | roads / footways  | FREE

Road widths come straight from OSM (already baked into the SDF OBBs by
scripts/fetch_osm.py), so alleys stay narrow and boulevards stay wide.

The world origin (0, 0) is the Enova HQ (datum 35.8173, 10.5912). We map a
1200 m x 1200 m outdoor patch centred there, at 1 m/cell -> 1200x1200 image.
"""
from __future__ import annotations

import math
import pathlib
import re

WORLD_SDF = pathlib.Path("/workspace/src/my_pguard_bot/worlds/sousse_buildings.sdf")
OUT_DIR = pathlib.Path("/workspace/src/my_pguard_bot/maps")
OUT_DIR.mkdir(parents=True, exist_ok=True)

EXTENT_M = 1200.0
RES_M = 1.0
CELLS = int(EXTENT_M / RES_M)

V_BUILDING = 0     # LETHAL
V_OFFROAD = 100    # NO_INFORMATION (expensive but crossable)
V_ROAD = 254       # FREE (cheap)


sdf = WORLD_SDF.read_text()
model_re = re.compile(
    r'<model[^>]*name="(?P<name>(?:bld|road)_[^"]+)"[^>]*>.*?'
    r'<pose>\s*(?P<x>-?[\d.]+)\s+(?P<y>-?[\d.]+)\s+(?P<z>-?[\d.]+)\s+'
    r'[\d.]+\s+[\d.]+\s+(?P<yaw>-?[\d.]+)\s*</pose>.*?'
    r'<box><size>\s*(?P<w>[\d.]+)\s+(?P<d>[\d.]+)\s+(?P<h>[\d.]+)\s*</size></box>',
    re.DOTALL,
)

buildings: list[tuple[float, float, float, float, float]] = []
roads: list[tuple[float, float, float, float, float]] = []
for m in model_re.finditer(sdf):
    kind = "bld" if m.group("name").startswith("bld_") else "road"
    cx = float(m.group("x")); cy = float(m.group("y"))
    yaw = float(m.group("yaw"))
    w = float(m.group("w")); d = float(m.group("d"))
    (buildings if kind == "bld" else roads).append((cx, cy, w, d, yaw))

print(f"Parsed {len(buildings)} buildings and {len(roads)} road segments")


grid = bytearray([V_OFFROAD] * (CELLS * CELLS))


def stamp(boxes: list[tuple[float, float, float, float, float]], value: int) -> None:
    """Rasterize a list of oriented (cx, cy, w, d, yaw) boxes into `grid`."""
    for cx, cy, w, d, yaw in boxes:
        r = math.hypot(w, d) / 2 + 0.5
        x_min = int((cx - r + EXTENT_M / 2) / RES_M)
        x_max = int((cx + r + EXTENT_M / 2) / RES_M) + 1
        y_min = int((cy - r + EXTENT_M / 2) / RES_M)
        y_max = int((cy + r + EXTENT_M / 2) / RES_M) + 1
        c, s = math.cos(-yaw), math.sin(-yaw)
        hw, hd = w / 2, d / 2
        for row in range(max(0, y_min), min(CELLS, y_max)):
            wy = row * RES_M - EXTENT_M / 2
            for col in range(max(0, x_min), min(CELLS, x_max)):
                wx = col * RES_M - EXTENT_M / 2
                dx, dy = wx - cx, wy - cy
                lx = dx * c - dy * s
                ly = dx * s + dy * c
                if abs(lx) <= hw and abs(ly) <= hd:
                    grid[row * CELLS + col] = value


# Stamp roads first, then buildings on top so that a building crossed by a
# road segment still shows as lethal.
stamp(roads, V_ROAD)
stamp(buildings, V_BUILDING)


pgm_path = OUT_DIR / "novation_city.pgm"
with open(pgm_path, "wb") as f:
    f.write(b"P5\n# Novation City occupancy grid (buildings + roads from OSM)\n")
    f.write(f"{CELLS} {CELLS}\n255\n".encode())
    # PGM row 0 is top of image, but grid row 0 is south -> flip vertically.
    for row in range(CELLS - 1, -1, -1):
        f.write(bytes(grid[row * CELLS:(row + 1) * CELLS]))
print(f"Wrote {pgm_path}  ({pgm_path.stat().st_size} bytes)")


# free_thresh chosen just above p=0.004 (road cells) and just below p=0.608
# (off-road cells) so only roads count as FREE. occupied_thresh sits above
# p=0.608 so off-road is NO_INFORMATION rather than LETHAL.
yaml_path = OUT_DIR / "novation_city.yaml"
yaml_path.write_text(
    f"""image: novation_city.pgm
resolution: {RES_M}
origin: [{-EXTENT_M/2}, {-EXTENT_M/2}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.20
"""
)
print(f"Wrote {yaml_path}")
