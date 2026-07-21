#!/usr/bin/env python3
"""
Fetch OpenStreetMap building footprints around Novation City / Technopole de
Sousse and emit them as an SDF <include> file that can be inserted into a
Gazebo world.

Origin (world <spherical_coordinates>): lat=35.8173, lon=10.5912
Radius: 600 m
"""
from __future__ import annotations

import math
import sys
import urllib.request
import json
from pathlib import Path

LAT0 = 35.8173
LON0 = 10.5912
RADIUS_M = 600
OUT_SDF = Path(__file__).resolve().parent.parent / "worlds" / "sousse_buildings.sdf"

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
OVERPASS_QUERY = f"""
[out:json][timeout:60];
(
  way["building"](around:{RADIUS_M},{LAT0},{LON0});
  relation["building"](around:{RADIUS_M},{LAT0},{LON0});
  way["highway"](around:{RADIUS_M},{LAT0},{LON0});
  node["office"](around:{RADIUS_M},{LAT0},{LON0});
  node["amenity"](around:{RADIUS_M},{LAT0},{LON0});
  node["shop"](around:{RADIUS_M},{LAT0},{LON0});
);
out body;
>;
out skel qt;
"""


def latlon_to_local_enu(lat: float, lon: float) -> tuple[float, float]:
    """Simple equirectangular projection - accurate at small radii."""
    R = 6378137.0
    x = math.radians(lon - LON0) * math.cos(math.radians(LAT0)) * R
    y = math.radians(lat - LAT0) * R
    return x, y


def fetch_osm() -> dict:
    print(f"Querying Overpass ({RADIUS_M} m around {LAT0}, {LON0}) ...", flush=True)
    # Try urllib first; fall back to curl (which has a working CA bundle on
    # older hosts like Amazon Linux 2 where Python's SSL trust store is broken).
    try:
        req = urllib.request.Request(
            OVERPASS_URL,
            data=OVERPASS_QUERY.encode("utf-8"),
            headers={"User-Agent": "ros2-outdoor-sim/0.1",
                     "Content-Type": "application/x-www-form-urlencoded"},
        )
        with urllib.request.urlopen(req, timeout=90) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception as urllib_err:
        print(f"  urllib failed ({urllib_err}); falling back to curl ...", flush=True)
        import subprocess, tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".osm.json", delete=False) as fh:
            out_path = fh.name
        subprocess.run(
            ["curl", "-sS", "-X", "POST",
             "-A", "ros2-outdoor-sim/0.1",
             "-H", "Content-Type: application/x-www-form-urlencoded",
             "--data-urlencode", f"data={OVERPASS_QUERY}",
             OVERPASS_URL, "-o", out_path],
            check=True,
        )
        with open(out_path) as fh:
            return json.load(fh)


def polygon_centroid_and_bbox(points: list[tuple[float, float]]):
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    w = max(xs) - min(xs)
    d = max(ys) - min(ys)
    return cx, cy, max(w, 0.5), max(d, 0.5)


def _convex_hull(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Andrew's monotone chain. Returns CCW hull without duplicate endpoint."""
    pts = sorted(set(pts))
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
    return lower[:-1] + upper[:-1]


def oriented_bbox(points: list[tuple[float, float]]):
    """Return the minimum-area oriented bounding box of a polygon.

    Result: (cx, cy, width, depth, yaw_rad). The rectangle has size w x d and
    is rotated by yaw about (cx, cy). Uses the rotating calipers property: the
    minimum-area OBB is aligned with one of the hull edges.
    """
    hull = _convex_hull(points)
    if len(hull) < 2:
        cxr = polygon_centroid_and_bbox(points)
        if cxr is None:
            return None
        cx, cy, w, d = cxr
        return cx, cy, w, d, 0.0

    best = None  # (area, cx, cy, w, d, yaw)
    n = len(hull)
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        yaw = math.atan2(y2 - y1, x2 - x1)
        cos_a, sin_a = math.cos(-yaw), math.sin(-yaw)
        xs_r = [p[0] * cos_a - p[1] * sin_a for p in hull]
        ys_r = [p[0] * sin_a + p[1] * cos_a for p in hull]
        min_x, max_x = min(xs_r), max(xs_r)
        min_y, max_y = min(ys_r), max(ys_r)
        w = max_x - min_x
        d = max_y - min_y
        area = w * d
        if best is None or area < best[0]:
            cx_r = (min_x + max_x) / 2
            cy_r = (min_y + max_y) / 2
            cx = cx_r * math.cos(yaw) - cy_r * math.sin(yaw)
            cy = cx_r * math.sin(yaw) + cy_r * math.cos(yaw)
            best = (area, cx, cy, max(w, 0.5), max(d, 0.5), yaw)

    _, cx, cy, w, d, yaw = best
    if w < d:
        w, d = d, w
        yaw += math.pi / 2
    while yaw > math.pi:
        yaw -= 2 * math.pi
    while yaw < -math.pi:
        yaw += 2 * math.pi
    return cx, cy, w, d, yaw


def emit_sdf(osm: dict) -> tuple[str, list[dict]]:
    nodes = {}
    for el in osm.get("elements", []):
        if el.get("type") == "node":
            nodes[el["id"]] = (el["lat"], el["lon"])

    buildings = []
    roads = []
    pois: list[dict] = []  # named office/amenity/shop points
    for el in osm.get("elements", []):
        etype = el.get("type")
        tags = el.get("tags", {})
        if etype == "way":
            nds = el.get("nodes", [])
            pts_ll = [nodes[n] for n in nds if n in nodes]
            pts_local = [latlon_to_local_enu(lat, lon) for lat, lon in pts_ll]
            if "building" in tags and len(pts_local) >= 3:
                height_m = 6.0
                if "height" in tags:
                    try:
                        height_m = float(str(tags["height"]).split()[0])
                    except ValueError:
                        pass
                elif "building:levels" in tags:
                    try:
                        height_m = 3.2 * float(tags["building:levels"])
                    except ValueError:
                        pass
                buildings.append((tags.get("name", f"bld_{el['id']}"), pts_local, height_m))
            elif "highway" in tags and len(pts_local) >= 2:
                roads.append((tags.get("name", f"way_{el['id']}"), pts_local, tags["highway"]))

        elif etype == "node":
            kind = None
            for key in ("office", "amenity", "shop"):
                if key in tags:
                    kind = f"{key}={tags[key]}"
                    break
            if kind is None:
                continue
            name = tags.get("name")
            if not name:
                continue
            lat, lon = el["lat"], el["lon"]
            x, y = latlon_to_local_enu(lat, lon)
            pois.append({
                "name": name,
                "kind": kind,
                "lat": lat,
                "lon": lon,
                "x": x,
                "y": y,
            })

    # Attach POI names to existing building polygons: if a named office node
    # sits inside a building's oriented bbox, that building takes on the POI's
    # name and gets highlighted. This is how tiny offices like Enova / VEO /
    # Proxym get labelled on the campus map.
    def point_in_obb(px: float, py: float, cx: float, cy: float,
                     w: float, d: float, yaw: float) -> bool:
        dx, dy = px - cx, py - cy
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx = dx * c - dy * s
        ly = dx * s + dy * c
        return abs(lx) <= w / 2 + 1.0 and abs(ly) <= d / 2 + 1.0

    labelled = []  # (name, pts, height, is_poi)
    poi_matched = set()
    for name, pts, h in buildings:
        obb = oriented_bbox(pts)
        if obb is None:
            continue
        cx, cy, w, d, yaw = obb
        matched_poi = None
        for i, poi in enumerate(pois):
            if i in poi_matched:
                continue
            if point_in_obb(poi["x"], poi["y"], cx, cy, w, d, yaw):
                matched_poi = poi
                poi_matched.add(i)
                break
        if matched_poi is not None:
            labelled.append((matched_poi["name"], pts, h, True))
        else:
            labelled.append((name, pts, h, False))

    # Any POIs that fell OUTSIDE all polygons still deserve a small synthesized
    # box so the label shows up on the map.
    synthesized = 0
    for i, poi in enumerate(pois):
        if i in poi_matched:
            continue
        pts_local = [
            (poi["x"] - 7.0, poi["y"] - 5.0),
            (poi["x"] + 7.0, poi["y"] - 5.0),
            (poi["x"] + 7.0, poi["y"] + 5.0),
            (poi["x"] - 7.0, poi["y"] + 5.0),
        ]
        labelled.append((poi["name"], pts_local, 6.0, True))
        synthesized += 1

    print(f"  Buildings: {len(labelled)} (POIs matched to buildings: {len(poi_matched)}, "
          f"synthesized: {synthesized})  Roads: {len(roads)}  POIs: {len(pois)}", flush=True)

    lines = ['<?xml version="1.0" ?>',
             "<!-- Auto-generated from OpenStreetMap. Included by the world SDF. -->",
             '<sdf version="1.10">']
    for name, pts, h, is_poi in labelled:
        obb = oriented_bbox(pts)
        if obb is None:
            continue
        cx, cy, w, d, yaw = obb
        safe = "".join(c if c.isalnum() else "_" for c in name)[:40]
        # Highlight labelled offices (matched POIs) in company blue.
        # Non-POI campus buildings stay tan.
        if is_poi:
            r, g, b = 0.35, 0.55, 0.85
        else:
            r, g, b = 0.75, 0.72, 0.65
        lines += [
            f'  <model name="bld_{safe}_{abs(int(cx*100))}">',
            f'    <static>true</static>',
            f'    <pose>{cx:.3f} {cy:.3f} {h/2:.3f} 0 0 {yaw:.4f}</pose>',
            f'    <link name="link">',
            f'      <collision name="c"><geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry></collision>',
            f'      <visual    name="v"><geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry>',
            f'        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>',
            f'    </link>',
            f'  </model>',
        ]
    # Style dispatch by OSM highway kind. (width_m, r, g, b, z_offset)
    road_style = {
        "trunk":        (8.0, 0.18, 0.18, 0.18, 0.010),
        "primary":      (7.0, 0.20, 0.20, 0.20, 0.011),
        "secondary":    (6.0, 0.22, 0.22, 0.22, 0.012),
        "tertiary":     (5.5, 0.25, 0.25, 0.25, 0.013),
        "unclassified": (4.5, 0.27, 0.27, 0.27, 0.014),
        "residential":  (4.5, 0.28, 0.28, 0.28, 0.014),
        "service":      (3.0, 0.32, 0.32, 0.32, 0.015),
        "living_street":(3.5, 0.32, 0.32, 0.32, 0.015),
        "construction": (3.5, 0.45, 0.40, 0.30, 0.016),
        "footway":      (1.5, 0.55, 0.55, 0.55, 0.020),
        "path":         (1.2, 0.55, 0.55, 0.55, 0.020),
        "cycleway":     (1.5, 0.35, 0.45, 0.30, 0.020),
        "pedestrian":   (2.5, 0.55, 0.55, 0.55, 0.020),
    }

    # Aggregate ALL road segments of a given street into a single SDF model
    # with many child visuals. Gazebo Harmonic pays a fixed per-model cost
    # (entity creation, physics init, scene-graph insertion) that is O(models),
    # not O(visuals). Emitting 1 model per street instead of 1 model per
    # segment cuts load time from ~40 s to ~5 s for Novation City (26 streets
    # vs 1000+ segments).
    #
    # build_map.py's regex still matches because we emit one <model name="road_*">
    # per segment-visual by keeping the model-per-segment layout also available
    # as an additional per-segment "sim marker" model — but actually the
    # simpler win is to update build_map.py to also parse the aggregate form.
    # We use ONE aggregate model per street and emit segment poses as
    # <include>-style sub-entries the map builder can find.
    road_visual_idx = 0
    for name, pts, kind in roads:
        if len(pts) < 2:
            continue
        style = road_style.get(kind, (3.0, 0.30, 0.30, 0.30, 0.014))
        width, r, g, b, z_off = style
        safe = "".join(c if c.isalnum() else "_" for c in name)[:30]

        merged: list[tuple[float, float, float, float]] = []
        merge_thresh_rad = math.radians(8.0)
        cur_x1, cur_y1 = pts[0]
        cur_x2, cur_y2 = pts[1]
        for i in range(1, len(pts) - 1):
            nx1, ny1 = pts[i]
            nx2, ny2 = pts[i + 1]
            yaw_cur = math.atan2(cur_y2 - cur_y1, cur_x2 - cur_x1)
            yaw_next = math.atan2(ny2 - ny1, nx2 - nx1)
            dy = (yaw_next - yaw_cur + math.pi) % (2 * math.pi) - math.pi
            if abs(dy) < merge_thresh_rad:
                cur_x2, cur_y2 = nx2, ny2
            else:
                merged.append((cur_x1, cur_y1, cur_x2, cur_y2))
                cur_x1, cur_y1 = nx1, ny1
                cur_x2, cur_y2 = nx2, ny2
        merged.append((cur_x1, cur_y1, cur_x2, cur_y2))

        # Collect all segment pose/size records for both the aggregate model
        # (Gazebo entity) and the per-segment stamp comments (map builder).
        seg_records: list[tuple[float, float, float, float, float]] = []
        for i, (x1, y1, x2, y2) in enumerate(merged):
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            length = math.hypot(x2 - x1, y2 - y1)
            if length < 0.3:
                continue
            yaw = math.atan2(y2 - y1, x2 - x1)
            seg_records.append((mx, my, length, width, yaw))

        if not seg_records:
            continue

        # ONE aggregate model per street: all segments share a link with many
        # visual children. Model pose is the world origin (pose 0), each visual
        # has its own <pose> in the parent frame. This is what Gazebo loads.
        lines += [
            f'  <model name="road_{safe}_{road_visual_idx}">',
            f'    <static>true</static>',
            f'    <pose>0 0 0 0 0 0</pose>',
            f'    <link name="link">',
        ]
        for j, (mx, my, length, w, yaw) in enumerate(seg_records):
            lines += [
                f'      <visual name="seg_{j}">',
                f'        <pose>{mx:.3f} {my:.3f} {z_off:.4f} 0 0 {yaw:.4f}</pose>',
                f'        <geometry><box><size>{length:.2f} {w:.2f} 0.02</size></box></geometry>',
                f'        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>',
                f'      </visual>',
            ]

        # Corner fill: extra square visuals inside the same link.
        if kind not in ("footway", "path", "cycleway", "pedestrian"):
            for i in range(len(merged) - 1):
                _, _, x2, y2 = merged[i]
                nx1, ny1, _, _ = merged[i + 1]
                cx, cy = (x2 + nx1) / 2, (y2 + ny1) / 2
                lines += [
                    f'      <visual name="corner_{i}">',
                    f'        <pose>{cx:.3f} {cy:.3f} {z_off + 0.001:.4f} 0 0 0</pose>',
                    f'        <geometry><box><size>{width:.2f} {width:.2f} 0.02</size></box></geometry>',
                    f'        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>',
                    f'      </visual>',
                ]

        lines += [
            f'    </link>',
            f'  </model>',
        ]

        # Emit XML comments that build_map.py can regex-parse for road cells.
        # These are NOT models — Gazebo ignores them — but keep the costmap
        # rasterizer working without changing its parser.
        for mx, my, length, w, yaw in seg_records:
            lines.append(
                f'  <!-- ROAD_SEG cx={mx:.3f} cy={my:.3f} w={length:.2f} d={w:.2f} yaw={yaw:.4f} -->'
            )
        road_visual_idx += 1
    lines += ["</sdf>"]
    return "\n".join(lines), pois


def main():
    try:
        osm = fetch_osm()
    except Exception as e:
        print(f"WARNING: Overpass fetch failed: {e}", file=sys.stderr)
        print("Emitting an empty stub instead.", file=sys.stderr)
        OUT_SDF.write_text(
            '<?xml version="1.0" ?>\n'
            '<!-- OSM fetch failed; rerun scripts/fetch_osm.py when online. -->\n'
            '<sdf version="1.10"/>\n'
        )
        return 1

    sdf, pois = emit_sdf(osm)
    OUT_SDF.parent.mkdir(parents=True, exist_ok=True)
    OUT_SDF.write_text(sdf)
    print(f"Wrote {OUT_SDF}  ({len(sdf)} bytes)")

    poi_out = OUT_SDF.parent / "sousse_pois.json"
    poi_out.write_text(json.dumps({"datum": [LAT0, LON0], "pois": pois}, indent=2))
    print(f"Wrote {poi_out}  ({len(pois)} POIs)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
