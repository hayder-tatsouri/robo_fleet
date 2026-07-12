#!/usr/bin/env python3
"""Generate a GPS waypoint loop for the PGuard perimeter patrol.

Strategy:
  1. Parse `worlds/sousse_buildings.sdf` to extract every building's AABB in
     the local ENU frame (meters, origin = spherical_coordinates datum).
  2. Compute the convex hull of the corners of all AABBs.
  3. Offset the hull outward by SAFETY_M to keep the robot clear of walls.
  4. Resample the offset polygon at fixed arc-length spacing so consecutive
     waypoints are ~SPACING_M apart.
  5. Convert each (x, y) back to (lat, lon) via the inverse of the
     equirectangular projection used in scripts/fetch_osm.py.
  6. Emit config/patrol_waypoints.yaml — a list of {lat, lon, yaw_deg}
     dicts, consumed by nodes/patrol_client.py.

Run once (offline):
    python3 scripts/generate_waypoints.py
"""
from __future__ import annotations

import math
import re
from pathlib import Path

LAT0 = 35.8173
LON0 = 10.5912
EARTH_R = 6378137.0

SAFETY_M = 12.0     # perimeter offset from building convex hull
SPACING_M = 100.0   # waypoint spacing along the perimeter (100 m gives ~20 wpts
                    # on the full Novation City hull -> ~2 min laps in sim)
MIN_HULL_POINTS = 4  # fall back to a fixed loop if too few buildings parsed

SDF_PATH = Path(__file__).resolve().parent.parent / "worlds" / "sousse_buildings.sdf"
OUT_YAML = Path(__file__).resolve().parent.parent / "config" / "patrol_waypoints.yaml"


# ---------- projection helpers ----------

def enu_to_latlon(x: float, y: float) -> tuple[float, float]:
    """Inverse of the equirectangular projection in scripts/fetch_osm.py."""
    lat = LAT0 + math.degrees(y / EARTH_R)
    lon = LON0 + math.degrees(x / (EARTH_R * math.cos(math.radians(LAT0))))
    return lat, lon


# ---------- SDF parsing ----------

_POSE_RE = re.compile(
    r'<model name="bld_[^"]+">.*?<pose>([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)[^<]*</pose>'
    r'.*?<box><size>([\d.]+)\s+([\d.]+)\s+([\d.]+)</size></box>',
    re.DOTALL,
)


def load_building_footprints(sdf_path: Path) -> list[tuple[float, float]]:
    """Return the 4 AABB corners for every building. One flat list of (x,y)."""
    text = sdf_path.read_text()
    corners: list[tuple[float, float]] = []
    for cx_s, cy_s, _cz, w_s, d_s, _h in _POSE_RE.findall(text):
        cx, cy, w, d = float(cx_s), float(cy_s), float(w_s), float(d_s)
        hw, hd = w / 2, d / 2
        corners.extend([(cx - hw, cy - hd), (cx + hw, cy - hd),
                        (cx + hw, cy + hd), (cx - hw, cy + hd)])
    return corners


# ---------- convex hull (Andrew's monotone chain) ----------

def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]  # CCW, no repeat


# ---------- polygon offset (outward normal, mitered) ----------

def offset_polygon(poly: list[tuple[float, float]], d: float) -> list[tuple[float, float]]:
    """Offset a CCW polygon outward by d meters using edge-normal miter joins."""
    n = len(poly)
    if n < 3:
        return poly

    def edge_normal(p, q):
        ex, ey = q[0] - p[0], q[1] - p[1]
        length = math.hypot(ex, ey) or 1.0
        # CCW polygon: outward normal is the right-hand perpendicular of the edge
        return (ey / length, -ex / length)

    out: list[tuple[float, float]] = []
    for i in range(n):
        pprev = poly[(i - 1) % n]
        pcurr = poly[i]
        pnext = poly[(i + 1) % n]
        n1 = edge_normal(pprev, pcurr)
        n2 = edge_normal(pcurr, pnext)
        bx, by = n1[0] + n2[0], n1[1] + n2[1]
        blen = math.hypot(bx, by) or 1.0
        cos_half = max(0.2, (n1[0] * n2[0] + n1[1] * n2[1] + 1.0) / 2.0) ** 0.5
        miter = d / cos_half
        miter = min(miter, 3.0 * d)  # clamp to avoid spikes on sharp corners
        out.append((pcurr[0] + bx / blen * miter,
                    pcurr[1] + by / blen * miter))
    return out


# ---------- perimeter resampling ----------

def resample_loop(poly: list[tuple[float, float]], spacing: float
                  ) -> list[tuple[float, float]]:
    """Uniformly resample a closed polygon at `spacing` meters."""
    closed = poly + [poly[0]]
    seg_lengths = [math.hypot(closed[i + 1][0] - closed[i][0],
                              closed[i + 1][1] - closed[i][1])
                   for i in range(len(closed) - 1)]
    total = sum(seg_lengths)
    n_samples = max(6, int(round(total / spacing)))
    step = total / n_samples

    samples: list[tuple[float, float]] = []
    s_target = 0.0
    s_acc = 0.0
    idx = 0
    for _ in range(n_samples):
        while idx < len(seg_lengths) and s_acc + seg_lengths[idx] < s_target:
            s_acc += seg_lengths[idx]
            idx += 1
        if idx >= len(seg_lengths):
            break
        t = (s_target - s_acc) / seg_lengths[idx] if seg_lengths[idx] > 0 else 0
        x1, y1 = closed[idx]
        x2, y2 = closed[idx + 1]
        samples.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
        s_target += step
    return samples


# ---------- fallback loop (used if SDF has no buildings) ----------

def default_square_loop() -> list[tuple[float, float]]:
    r = 40.0
    return [(r, r), (-r, r), (-r, -r), (r, -r)]


# ---------- writer ----------

def emit_yaml(waypoints_xy: list[tuple[float, float]]) -> str:
    lines = [
        "# Auto-generated by scripts/generate_waypoints.py",
        f"# Datum: lat={LAT0}, lon={LON0}",
        f"# Perimeter offset: {SAFETY_M} m,  spacing: {SPACING_M} m",
        f"# Waypoints: {len(waypoints_xy)}",
        "waypoints:",
    ]
    n = len(waypoints_xy)
    for i, (x, y) in enumerate(waypoints_xy):
        lat, lon = enu_to_latlon(x, y)
        nx = waypoints_xy[(i + 1) % n][0] - x
        ny = waypoints_xy[(i + 1) % n][1] - y
        yaw = math.degrees(math.atan2(ny, nx))
        lines.append(f"  - {{lat: {lat:.8f}, lon: {lon:.8f}, "
                     f"yaw_deg: {yaw:.2f}, enu_x: {x:.2f}, enu_y: {y:.2f}}}")
    return "\n".join(lines) + "\n"


def main() -> int:
    if not SDF_PATH.exists():
        print(f"ERROR: {SDF_PATH} not found. Run scripts/fetch_osm.py first.")
        return 1

    corners = load_building_footprints(SDF_PATH)
    print(f"Building AABB corners parsed: {len(corners)}")

    if len(corners) < MIN_HULL_POINTS:
        print("Too few buildings; falling back to a default 40 m square loop.")
        loop = default_square_loop()
    else:
        hull = convex_hull(corners)
        print(f"Convex hull vertices: {len(hull)}")
        offset = offset_polygon(hull, SAFETY_M)
        loop = resample_loop(offset, SPACING_M)
        print(f"Resampled perimeter waypoints: {len(loop)}")

    OUT_YAML.parent.mkdir(parents=True, exist_ok=True)
    OUT_YAML.write_text(emit_yaml(loop))
    print(f"Wrote {OUT_YAML}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
