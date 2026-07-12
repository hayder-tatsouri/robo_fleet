#!/usr/bin/env python3
"""Overlay POI (company) name labels onto sim_top.png.

Reads:
    docs/images/sim_top.png       - raw top-down Gazebo capture
    worlds/sousse_pois.json       - list of {name, x, y} in world-local metres
Writes:
    docs/images/sim_top_labeled.png

The top camera in the world SDF is at (0, 0, 200) with pitch=+90 deg (looking
straight down along -Z), horizontal_fov=1.4 rad, 1024x1024 image.
At altitude z_cam and horizontal FoV theta_h, the ground footprint width is
    W = 2 * z_cam * tan(theta_h / 2)
So world (x, y) -> pixel:
    px = W/2 - (world_x)   scaled to image width  [image X grows right; world X grows east; camera looks down -Z with default yaw pointing... hmm depends on frame]
Because the camera pose is `0 0 200  0 1.5707 0` (pitch = +pi/2), the camera
optical axis points straight down. In Gazebo default camera frame the image
X axis is world +X (east) mirrored to the left, image Y axis is world +Y
(north) mirrored downward.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
PKG = HERE.parent
IMG_IN = PKG / "docs" / "images" / "sim_top.png"
POI_JSON = PKG / "worlds" / "sousse_pois.json"
IMG_OUT = PKG / "docs" / "images" / "sim_top_labeled.png"

# Must match the world SDF `top_cam` model.
CAM_ALT_M = 220.0
CAM_FOV_RAD = 1.30
# Camera is offset from origin (see world SDF `top_cam` pose).
CAM_OFFSET_X = -30.0
CAM_OFFSET_Y = 40.0
IMG_W = 1024
IMG_H = 1024


def world_to_pixel(x: float, y: float) -> tuple[float, float]:
    """(x_east, y_north) metres in world frame -> (u, v) pixel in the image.

    The top camera is at (CAM_OFFSET_X, CAM_OFFSET_Y, CAM_ALT_M) pitched down.
    Coordinates are relative to the camera's ground centre; convert to pixel
    positions using the ground FoV footprint.
    """
    ground_half_w = CAM_ALT_M * math.tan(CAM_FOV_RAD / 2.0)
    # World -> camera-centred metres:
    dx = x - CAM_OFFSET_X
    dy = y - CAM_OFFSET_Y
    # After empirical check for pitched-down camera in ENU world:
    # image X grows with world +y (north points right in image),
    # image Y grows with world -x (east points UP on the image).
    u = IMG_W / 2 + (dy / ground_half_w) * (IMG_W / 2)
    v = IMG_H / 2 - (dx / ground_half_w) * (IMG_H / 2)
    return u, v


def main() -> int:
    if not IMG_IN.exists():
        print(f"missing {IMG_IN}")
        return 1
    if not POI_JSON.exists():
        print(f"missing {POI_JSON}")
        return 1

    data = json.loads(POI_JSON.read_text())
    pois = data["pois"]

    fig = plt.figure(figsize=(10, 10), dpi=110)
    ax = fig.add_subplot(1, 1, 1)
    img = plt.imread(str(IMG_IN))
    ax.imshow(img)
    ax.set_xlim(0, IMG_W)
    ax.set_ylim(IMG_H, 0)
    ax.axis("off")

    ground_half_w = CAM_ALT_M * math.tan(CAM_FOV_RAD / 2.0)
    ax.set_title(
        f"Novation City / Technopole de Sousse - datum {data['datum'][0]:.4f}, {data['datum'][1]:.4f}\n"
        f"Top-down Gazebo view (alt {CAM_ALT_M:.0f} m, ground {2*ground_half_w:.0f} m wide) with OSM POIs overlaid",
        fontsize=10,
    )

    palette = {
        "office": "#1e88e5",
        "amenity": "#e53935",
        "shop": "#8e24aa",
    }

    seen = set()
    for poi in pois:
        name = poi["name"]
        kind = poi["kind"]
        u, v = world_to_pixel(poi["x"], poi["y"])
        if not (0 <= u <= IMG_W and 0 <= v <= IMG_H):
            continue
        base = kind.split("=", 1)[0]
        color = palette.get(base, "#333333")
        marker = "*" if name == "Enova Robotics" else "o"
        ms = 18 if name == "Enova Robotics" else 8
        ax.plot(u, v, marker=marker, color=color, markersize=ms,
                markeredgecolor="white", markeredgewidth=1.2, zorder=5)
        label = name if len(name) <= 34 else name[:31] + "..."
        emphasis = dict(fontsize=10, fontweight="bold") if name == "Enova Robotics" else dict(fontsize=8)
        ax.annotate(
            label, xy=(u, v), xytext=(10, -6), textcoords="offset points",
            color=color,
            bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                      edgecolor=color, linewidth=0.8, alpha=0.85),
            arrowprops=dict(arrowstyle="-", color=color, linewidth=0.6),
            **emphasis,
        )
        seen.add(name)

    # Datum crosshair.
    u0, v0 = world_to_pixel(0.0, 0.0)
    ax.plot(u0, v0, "+", color="black", markersize=18, markeredgewidth=1.5, zorder=6)
    ax.annotate("datum (0, 0)", xy=(u0, v0), xytext=(-70, 20),
                textcoords="offset points", fontsize=8, color="black")

    # Compass rose (north = up here? depends on projection). Draw arrows for
    # world +x (east) and world +y (north) for orientation clarity.
    e_u, e_v = world_to_pixel(50, 0)
    n_u, n_v = world_to_pixel(0, 50)
    ax.annotate("", xy=(e_u, e_v), xytext=(u0, v0),
                arrowprops=dict(arrowstyle="->", color="red", lw=2))
    ax.text(e_u + 5, e_v + 5, "+x east", fontsize=8, color="red")
    ax.annotate("", xy=(n_u, n_v), xytext=(u0, v0),
                arrowprops=dict(arrowstyle="->", color="green", lw=2))
    ax.text(n_u + 5, n_v + 5, "+y north", fontsize=8, color="green")

    IMG_OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(IMG_OUT), dpi=110, bbox_inches="tight")
    print(f"Wrote {IMG_OUT} with {len(seen)} labels")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
