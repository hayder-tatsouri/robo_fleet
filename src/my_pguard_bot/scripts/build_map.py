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

HERE = pathlib.Path(__file__).resolve().parent
PKG_ROOT = HERE.parent  # my_pguard_bot/
WORLD_SDF = PKG_ROOT / "worlds" / "sousse_buildings.sdf"
OUT_DIR = PKG_ROOT / "maps"
OUT_DIR.mkdir(parents=True, exist_ok=True)

EXTENT_M = 1200.0
RES_M = 1.0
CELLS = int(EXTENT_M / RES_M)

V_BUILDING = 0     # LETHAL
V_OFFROAD = 100    # NO_INFORMATION (expensive but crossable)
V_ROAD = 254       # FREE (cheap)


sdf = WORLD_SDF.read_text()

# Building models unchanged: each `bld_*` has its own <model><pose><box>.
bld_re = re.compile(
    r'<model[^>]*name="(?P<name>bld_[^"]+)"[^>]*>.*?'
    r'<pose>\s*(?P<x>-?[\d.]+)\s+(?P<y>-?[\d.]+)\s+(?P<z>-?[\d.]+)\s+'
    r'[\d.]+\s+[\d.]+\s+(?P<yaw>-?[\d.]+)\s*</pose>.*?'
    r'<box><size>\s*(?P<w>[\d.]+)\s+(?P<d>[\d.]+)\s+(?P<h>[\d.]+)\s*</size></box>',
    re.DOTALL,
)

# Roads: fetch_osm.py now emits ONE aggregate <model name="road_*"> per street
# with many child <visual>s (much faster for Gazebo to load), and emits
# `<!-- ROAD_SEG cx=.. cy=.. w=.. d=.. yaw=.. -->` XML comments for each
# per-segment stamp so this rasterizer can still paint road cells.
road_seg_re = re.compile(
    r'<!--\s*ROAD_SEG\s+cx=(?P<cx>-?[\d.]+)\s+cy=(?P<cy>-?[\d.]+)\s+'
    r'w=(?P<w>[\d.]+)\s+d=(?P<d>[\d.]+)\s+yaw=(?P<yaw>-?[\d.]+)\s*-->'
)

buildings: list[tuple[float, float, float, float, float]] = []
for m in bld_re.finditer(sdf):
    cx = float(m.group("x")); cy = float(m.group("y"))
    yaw = float(m.group("yaw"))
    w = float(m.group("w")); d = float(m.group("d"))
    buildings.append((cx, cy, w, d, yaw))

roads: list[tuple[float, float, float, float, float]] = []
for m in road_seg_re.finditer(sdf):
    roads.append((
        float(m.group("cx")), float(m.group("cy")),
        float(m.group("w")),  float(m.group("d")),
        float(m.group("yaw")),
    ))

# Fallback: if no ROAD_SEG comments were found (e.g. the SDF was generated
# by an older fetch_osm.py that emitted per-segment models), parse the
# legacy `<model name="road_*"><pose>...<box>` layout.
if not roads:
    legacy_road_re = re.compile(
        r'<model[^>]*name="(?P<name>road_[^"]+)"[^>]*>.*?'
        r'<pose>\s*(?P<x>-?[\d.]+)\s+(?P<y>-?[\d.]+)\s+(?P<z>-?[\d.]+)\s+'
        r'[\d.]+\s+[\d.]+\s+(?P<yaw>-?[\d.]+)\s*</pose>.*?'
        r'<box><size>\s*(?P<w>[\d.]+)\s+(?P<d>[\d.]+)\s+(?P<h>[\d.]+)\s*</size></box>',
        re.DOTALL,
    )
    for m in legacy_road_re.finditer(sdf):
        roads.append((
            float(m.group("x")), float(m.group("y")),
            float(m.group("w")), float(m.group("d")),
            float(m.group("yaw")),
        ))

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
