#!/usr/bin/env python3
"""Collapse per-segment road models in sousse_buildings.sdf into aggregate
per-street models, without re-hitting Overpass.

Why: Gazebo Harmonic pays a fixed per-model cost (entity creation, physics
init, scene-graph insertion). Novation City has 26 buildings + 1090 road
segment models = 1116 top-level models. Loading takes 30-60 s on CPU-only
hosts. Merging all road segments into ~26 aggregate models (one per street)
drops that to <10 s while keeping the identical visual output.

Roads are re-emitted as:
  <model name="road_<n>">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <visual name="seg_0"> <pose>..</pose> <box>..</box> ... </visual>
      <visual name="seg_1"> ... </visual>
      ...
    </link>
  </model>

Also emits <!-- ROAD_SEG cx=.. cy=.. w=.. d=.. yaw=.. --> comments so
build_map.py can still rasterize each segment for the Nav2 costmap.

Run offline:
    python3 scripts/collapse_road_models.py

Then rerun scripts/build_world.py to refresh novation_city.sdf and rebuild:
    python3 scripts/build_world.py
    python3 scripts/build_map.py
    colcon build --symlink-install --packages-select my_pguard_bot
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG_ROOT = HERE.parent  # my_pguard_bot/
SDF_PATH = PKG_ROOT / "worlds" / "sousse_buildings.sdf"

MODEL_RE = re.compile(
    r'<model[^>]*name="(?P<name>[^"]+)"[^>]*>(?P<body>.*?)</model>',
    re.DOTALL,
)

POSE_RE = re.compile(
    r'<pose>\s*(?P<x>-?[\d.]+)\s+(?P<y>-?[\d.]+)\s+(?P<z>-?[\d.]+)\s+'
    r'(?P<r>-?[\d.]+)\s+(?P<p>-?[\d.]+)\s+(?P<yw>-?[\d.]+)\s*</pose>',
    re.DOTALL,
)
BOX_RE = re.compile(
    r'<box><size>\s*(?P<w>[\d.]+)\s+(?P<d>[\d.]+)\s+(?P<h>[\d.]+)\s*</size></box>'
)
MATERIAL_RE = re.compile(
    r'<material>(?P<inner>.*?)</material>', re.DOTALL,
)


def street_of(name: str) -> str:
    """`road_MyStreet_3_1234` -> `MyStreet`.  `road_MyStreet_corner_2_1234` -> `MyStreet`."""
    parts = name.split("_")
    # trim leading 'road' and trailing numeric/corner tokens
    if not parts or parts[0] != "road":
        return name
    tail = parts[1:]
    while tail and (tail[-1].isdigit() or tail[-1] == "corner"):
        tail.pop()
    return "_".join(tail) if tail else "unnamed"


def main() -> int:
    if not SDF_PATH.exists():
        print(f"missing: {SDF_PATH}", file=sys.stderr)
        return 1

    text = SDF_PATH.read_text()

    n_bld = 0
    n_road_in = 0
    n_road_out = 0

    kept_chunks: list[str] = []
    # streets: name -> list of (pose_str, box_str, material_str)
    streets: dict[str, list[tuple[str, str, str]]] = {}
    # street_seg_records: name -> list of (cx, cy, w, d, yaw) for map comments
    street_records: dict[str, list[tuple[float, float, float, float, float]]] = {}

    last_end = 0
    for m in MODEL_RE.finditer(text):
        # keep everything before this model as-is
        kept_chunks.append(text[last_end:m.start()])
        last_end = m.end()

        name = m.group("name")
        body = m.group("body")

        if name.startswith("bld_"):
            # buildings untouched
            kept_chunks.append(m.group(0))
            n_bld += 1
            continue

        if not name.startswith("road_"):
            # unknown -> keep unchanged
            kept_chunks.append(m.group(0))
            continue

        # road model: extract pose + box + material
        n_road_in += 1
        pm = POSE_RE.search(body)
        bm = BOX_RE.search(body)
        mm = MATERIAL_RE.search(body)
        if not (pm and bm):
            kept_chunks.append(m.group(0))  # keep unchanged if parse fails
            continue

        cx = float(pm.group("x"))
        cy = float(pm.group("y"))
        cz = float(pm.group("z"))
        yaw = float(pm.group("yw"))
        w = float(bm.group("w"))
        d = float(bm.group("d"))
        h = float(bm.group("h"))
        material_inner = mm.group("inner") if mm else "<ambient>0.3 0.3 0.3 1</ambient>"

        street = street_of(name)

        pose_str = f'<pose>{cx:.3f} {cy:.3f} {cz:.4f} 0 0 {yaw:.4f}</pose>'
        box_str = f'<geometry><box><size>{w:.2f} {d:.2f} {h:.2f}</size></box></geometry>'
        material_str = f'<material>{material_inner}</material>'
        streets.setdefault(street, []).append((pose_str, box_str, material_str))
        street_records.setdefault(street, []).append((cx, cy, w, d, yaw))

    # trailing text after the last model
    kept_chunks.append(text[last_end:])
    kept_text = "".join(kept_chunks)

    # Emit aggregate road models before the closing </sdf>
    aggregate_lines: list[str] = []
    for i, (street, segs) in enumerate(sorted(streets.items())):
        aggregate_lines.append(f'  <model name="road_{street}_{i}">')
        aggregate_lines.append(f'    <static>true</static>')
        aggregate_lines.append(f'    <pose>0 0 0 0 0 0</pose>')
        aggregate_lines.append(f'    <link name="link">')
        for j, (pose_str, box_str, material_str) in enumerate(segs):
            aggregate_lines.append(f'      <visual name="seg_{j}">')
            aggregate_lines.append(f'        {pose_str}')
            aggregate_lines.append(f'        {box_str}')
            aggregate_lines.append(f'        {material_str}')
            aggregate_lines.append(f'      </visual>')
        aggregate_lines.append(f'    </link>')
        aggregate_lines.append(f'  </model>')
        n_road_out += 1

        # ROAD_SEG comments for the map builder
        for cx, cy, w, d, yaw in street_records[street]:
            aggregate_lines.append(
                f'  <!-- ROAD_SEG cx={cx:.3f} cy={cy:.3f} w={w:.2f} d={d:.2f} yaw={yaw:.4f} -->'
            )

    aggregate_block = "\n".join(aggregate_lines) + "\n"

    # Insert before the outer </sdf> close tag.
    if "</sdf>" not in kept_text:
        print("no closing </sdf> found in input", file=sys.stderr)
        return 1
    out_text = kept_text.replace("</sdf>", aggregate_block + "</sdf>", 1)

    SDF_PATH.write_text(out_text)
    print(f"Buildings kept:     {n_bld}")
    print(f"Road models in:     {n_road_in}")
    print(f"Aggregate models:   {n_road_out}")
    print(f"Reduction:          {n_road_in} -> {n_road_out}"
          f" ({(1 - n_road_out / max(n_road_in, 1)) * 100:.1f}% fewer models)")
    print(f"Wrote {SDF_PATH}  ({SDF_PATH.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
