#!/usr/bin/env python3
"""
Compose the final novation_city.sdf by inlining sousse_buildings.sdf into
novation_city.template.sdf. SDF <include> requires a model directory, so
concatenation is the simplest robust approach.
"""
from pathlib import Path
import re
import sys

HERE = Path(__file__).resolve().parent
WORLDS = HERE.parent / "worlds"

TEMPLATE = WORLDS / "novation_city.template.sdf"
BUILDINGS = WORLDS / "sousse_buildings.sdf"
OUT = WORLDS / "novation_city.sdf"


def main() -> int:
    if not TEMPLATE.exists():
        print(f"missing template: {TEMPLATE}", file=sys.stderr); return 1
    if not BUILDINGS.exists():
        print(f"missing buildings SDF (run scripts/fetch_osm.py first): {BUILDINGS}", file=sys.stderr)
        return 1

    tmpl = TEMPLATE.read_text()
    blds = BUILDINGS.read_text()
    inner = re.search(r'<sdf[^>]*>(.*)</sdf>', blds, re.DOTALL)
    if not inner:
        print("could not extract inner content from buildings SDF", file=sys.stderr); return 1
    inner_xml = inner.group(1).strip()

    marker = "<!-- BUILDINGS_HERE -->"
    if marker not in tmpl:
        print(f"template missing marker {marker!r}", file=sys.stderr); return 1

    OUT.write_text(tmpl.replace(marker, inner_xml))
    print(f"Wrote {OUT}  ({OUT.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
