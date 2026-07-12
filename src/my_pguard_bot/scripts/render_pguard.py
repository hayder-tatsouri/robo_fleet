#!/usr/bin/env python3
"""Render an engineering-style 4-panel view of the PGuard URDF to PNG.

Keeps the geometry in lockstep with description/pguard.urdf.xacro. Produces
front / side / top / isometric-info panels using pure 2D matplotlib
projections (matplotlib's mplot3d has poor z-ordering for opaque solids;
orthographic 2D projections are cleaner and more useful anyway).

Usage:
    python3 scripts/render_pguard.py
    -> writes docs/images/pguard_sim.png
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

OUT_PATH = (Path(__file__).resolve().parent.parent
            / "docs" / "images" / "pguard_sim.png")


# ---------- URDF-mirrored constants ----------
BASE_L, BASE_W, BASE_H = 1.50, 1.10, 1.10
TURRET_H = 0.50
WHEEL_R, WHEEL_W = 0.25, 0.20
WHEEL_SEP_X, WHEEL_SEP_Y = 1.00, 1.05

CHASSIS_BOTTOM_Z = WHEEL_R          # base of the box (wheel_radius lift)
CHASSIS_TOP_Z    = WHEEL_R + BASE_H
TURRET_TOP_Z     = CHASSIS_TOP_Z + TURRET_H
DOME_Z           = TURRET_TOP_Z + 0.10
TOTAL_H          = DOME_Z + 0.14    # top of the dome sphere


BLACK = "#1a1a1e"
DARK  = "#0c0c0c"
RED   = "#d81b1b"
YEL   = "#e6d02e"
GREEN = "#2ecc71"
GRAY  = "#666"
SONAR = "#ff5252"


def add_box(ax, x, y, w, h, color=BLACK, edge="black", lw=0.6, alpha=1.0):
    ax.add_patch(mpatches.Rectangle((x, y), w, h, facecolor=color,
                                    edgecolor=edge, linewidth=lw, alpha=alpha))


def add_circle(ax, cx, cy, r, color=BLACK, edge="black", lw=0.6, alpha=1.0):
    ax.add_patch(mpatches.Circle((cx, cy), r, facecolor=color,
                                 edgecolor=edge, linewidth=lw, alpha=alpha))


# ---------- side view (x, z) — robot facing right ----------
def draw_side(ax):
    ax.set_title("Side view  (x, z)", fontsize=10)
    ax.axhline(0, color="#888", lw=0.4)

    # wheels (front + rear, closer wheel drawn on top)
    add_circle(ax,  WHEEL_SEP_X/2, WHEEL_R, WHEEL_R, DARK)
    add_circle(ax, -WHEEL_SEP_X/2, WHEEL_R, WHEEL_R, DARK)

    # chassis
    add_box(ax, -BASE_L/2, CHASSIS_BOTTOM_Z, BASE_L, BASE_H, BLACK)

    # turret pole
    add_box(ax, -0.08, CHASSIS_TOP_Z, 0.16, TURRET_H, DARK)

    # dome (turret head)
    add_circle(ax, 0, DOME_Z, 0.14, DARK)

    # red beacon (slightly right of turret pole in x, but on the roof)
    add_circle(ax, 0.10, CHASSIS_TOP_Z + 0.05, 0.06, RED)

    # GPS puck (rear-top of chassis)
    add_box(ax, -BASE_L/2 + 0.09, CHASSIS_TOP_Z, 0.12, 0.06, YEL)

    # front + rear sonars (side profile)
    add_box(ax,  BASE_L/2, CHASSIS_BOTTOM_Z + BASE_H/2 - 0.015, 0.05, 0.03, GRAY)
    add_box(ax, -BASE_L/2 - 0.05, CHASSIS_BOTTOM_Z + BASE_H/2 - 0.015, 0.05, 0.03, GRAY)

    # camera lens on the front of the dome
    add_box(ax, 0.11, DOME_Z - 0.02, 0.03, 0.04, "#c0c0c0")

    # dimension arrows
    _dim(ax, x=-BASE_L/2 - 0.05, y1=0, y2=TOTAL_H, label=f"H = {TOTAL_H:.2f} m",
         side="left", offset=0.25)
    _dim(ax, y=-0.15, x1=-BASE_L/2, x2=BASE_L/2, label=f"L = {BASE_L:.2f} m",
         side="bottom", offset=0.12)

    _finish(ax, xlim=(-1.4, 1.4), ylim=(-0.4, 2.3), aspect=1)


# ---------- front view (y, z) — looking at the robot's face ----------
def draw_front(ax):
    ax.set_title("Front view  (y, z)  — as seen from ahead", fontsize=10)
    ax.axhline(0, color="#888", lw=0.4)

    # front-left / front-right wheels (viewed from front, both visible)
    add_box(ax,  WHEEL_SEP_Y/2 - WHEEL_W/2, 0, WHEEL_W, 2 * WHEEL_R, DARK)
    add_box(ax, -WHEEL_SEP_Y/2 - WHEEL_W/2, 0, WHEEL_W, 2 * WHEEL_R, DARK)

    # chassis (front face)
    add_box(ax, -BASE_W/2, CHASSIS_BOTTOM_Z, BASE_W, BASE_H, BLACK)

    # turret pole
    add_box(ax, -0.08, CHASSIS_TOP_Z, 0.16, TURRET_H, DARK)

    # dome
    add_circle(ax, 0, DOME_Z, 0.14, DARK)

    # front camera lens (on the dome, facing us -> circle)
    add_circle(ax, 0, DOME_Z, 0.05, "#c0c0c0")
    add_circle(ax, 0, DOME_Z, 0.025, "#111")

    # beacon (visible on right side of chassis roof)
    add_circle(ax, 0.25, CHASSIS_TOP_Z + 0.05, 0.06, RED)

    # left + right sonars (side profile)
    add_box(ax,  BASE_W/2, CHASSIS_BOTTOM_Z + BASE_H/2 - 0.015, 0.05, 0.03, GRAY)
    add_box(ax, -BASE_W/2 - 0.05, CHASSIS_BOTTOM_Z + BASE_H/2 - 0.015, 0.05, 0.03, GRAY)

    _dim(ax, y=-0.15, x1=-BASE_W/2, x2=BASE_W/2, label=f"W = {BASE_W:.2f} m",
         side="bottom", offset=0.12)
    _dim(ax, x=BASE_W/2 + 0.1, y1=0, y2=TOTAL_H, label=f"H = {TOTAL_H:.2f} m",
         side="right", offset=0.25)

    _finish(ax, xlim=(-1.1, 1.1), ylim=(-0.4, 2.3), aspect=1)


# ---------- top view (x, y) — looking straight down ----------
def draw_top(ax):
    ax.set_title("Top view  (x, y)  — bird's eye", fontsize=10)

    # chassis footprint
    add_box(ax, -BASE_L/2, -BASE_W/2, BASE_L, BASE_W, BLACK, alpha=0.9)

    # 4 wheels (rectangles from above)
    for (wx, wy) in [( WHEEL_SEP_X/2,  WHEEL_SEP_Y/2),
                     ( WHEEL_SEP_X/2, -WHEEL_SEP_Y/2),
                     (-WHEEL_SEP_X/2,  WHEEL_SEP_Y/2),
                     (-WHEEL_SEP_X/2, -WHEEL_SEP_Y/2)]:
        add_box(ax, wx - WHEEL_R, wy - WHEEL_W/2, 2 * WHEEL_R, WHEEL_W, DARK)

    # turret + dome (concentric circles at centroid)
    add_circle(ax, 0, 0, 0.14, DARK)
    add_circle(ax, 0, 0, 0.05, "#c0c0c0")

    # beacon (offset from centroid)
    add_circle(ax, 0.10, 0.25, 0.06, RED)

    # GPS puck (rear-top of chassis, top view)
    add_circle(ax, -BASE_L/2 + 0.15, 0, 0.06, YEL)

    # sonars + detection cones (2D fans)
    _sonar_fan(ax,  BASE_L/2 + 0.02, 0,             0, 2.5)   # front
    _sonar_fan(ax, -BASE_L/2 - 0.02, 0,           180, 2.5)   # rear
    _sonar_fan(ax, 0,                BASE_W/2 + 0.02, 90, 2.5)   # left
    _sonar_fan(ax, 0,               -BASE_W/2 - 0.02, -90, 2.5)  # right

    # forward arrow
    ax.annotate("", xy=(BASE_L/2 + 0.45, 0), xytext=(BASE_L/2, 0),
                arrowprops=dict(arrowstyle="->", color="red", lw=1.5))
    ax.text(BASE_L/2 + 0.50, 0.03, "+x (fwd)", color="red", fontsize=8)

    _dim(ax, y=-BASE_W/2 - 0.35, x1=-BASE_L/2, x2=BASE_L/2,
         label=f"L = {BASE_L:.2f} m", side="bottom", offset=0.20)
    _dim(ax, x=-BASE_L/2 - 0.35, y1=-BASE_W/2, y2=BASE_W/2,
         label=f"W = {BASE_W:.2f} m", side="left", offset=0.20)

    _finish(ax, xlim=(-3.2, 3.2), ylim=(-3.2, 3.2), aspect=1)


# ---------- info panel ----------
def draw_info(ax):
    ax.axis("off")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.02, 0.95, "PGuard simulation model",
            fontsize=14, fontweight="bold", va="top")
    ax.text(0.02, 0.90,
            "URDF: src/my_pguard_bot/description/pguard.urdf.xacro",
            fontsize=8, va="top", color="#555", family="monospace")

    rows = [
        ("Length",           f"{BASE_L:.2f} m",     "150 cm"),
        ("Width",            f"{BASE_W:.2f} m",     "110 cm"),
        ("Height",           f"{TOTAL_H:.2f} m",    "160 cm"),
        ("Chassis mass",     "220 kg",              "~250 kg"),
        ("Total mass",       "~250 kg",             "250 kg"),
        ("Drive",            "4-w skid steer",      "4-wheel drive"),
        ("Wheel radius",     f"{WHEEL_R:.2f} m",    "-"),
        ("Wheel base",       f"{WHEEL_SEP_X:.2f} x {WHEEL_SEP_Y:.2f} m", "-"),
        ("Max speed",        "2.0 m/s",             "3.33 m/s (12 km/h)"),
        ("GPS",              "RTK, 2 cm",           "centimetric"),
        ("IMU",              "100 Hz",              "onboard IMU"),
        ("Obstacle sensing", "4x ultrasonic 5 m",   "LiDAR + ultrasonics"),
        ("Cameras",          "1 stub",              "4x 360 + pan/tilt + thermal"),
        ("Beacon",           "red",                 "red + 2 LED headlights"),
    ]
    ax.text(0.02, 0.83, "Spec",           fontsize=9, fontweight="bold")
    ax.text(0.36, 0.83, "Our sim",        fontsize=9, fontweight="bold")
    ax.text(0.66, 0.83, "Real PGuard",    fontsize=9, fontweight="bold",
            color="#c00")
    y = 0.80
    for label, sim_val, real_val in rows:
        ax.text(0.02, y, label,      fontsize=8, family="monospace")
        ax.text(0.36, y, sim_val,    fontsize=8, family="monospace")
        ax.text(0.66, y, real_val,   fontsize=8, family="monospace", color="#500")
        y -= 0.045

    ax.text(0.02, 0.10,
            "Legend:  [box] chassis   (o) wheels/turret   red = beacon   "
            "yellow = GPS   gray = sonar   pink cones = sonar FOV",
            fontsize=8, color="#333")
    ax.text(0.02, 0.05,
            "Not yet modelled: 4× 360° cameras, thermal cam, day/night cam "
            "w/ 32× zoom, mic + speakers, LiDAR.",
            fontsize=8, color="#a00", style="italic")


# ---------- helpers ----------
def _sonar_fan(ax, x, y, heading_deg, r_m, half_angle_deg=15):
    center = (x, y)
    wedge = mpatches.Wedge(center, r_m,
                           heading_deg - half_angle_deg,
                           heading_deg + half_angle_deg,
                           facecolor=SONAR, alpha=0.20,
                           edgecolor=SONAR, linewidth=0.6)
    ax.add_patch(wedge)
    ax.add_patch(mpatches.Rectangle((x - 0.03, y - 0.03), 0.06, 0.06,
                                    facecolor=GRAY, edgecolor="black",
                                    linewidth=0.4))


def _dim(ax, *, x=None, y=None, x1=None, x2=None, y1=None, y2=None,
         label="", side="bottom", offset=0.15):
    """Draw a simple dimension line + label."""
    if side in ("bottom", "top"):
        yy = y
        ax.annotate("", xy=(x2, yy), xytext=(x1, yy),
                    arrowprops=dict(arrowstyle="<->", color="#555", lw=0.8))
        ax.text((x1 + x2)/2, yy - offset if side == "bottom" else yy + offset,
                label, ha="center", va="top" if side == "bottom" else "bottom",
                fontsize=8, color="#333")
    else:  # left / right
        xx = x
        ax.annotate("", xy=(xx, y2), xytext=(xx, y1),
                    arrowprops=dict(arrowstyle="<->", color="#555", lw=0.8))
        ax.text(xx - offset if side == "left" else xx + offset,
                (y1 + y2)/2, label,
                ha="right" if side == "left" else "left", va="center",
                fontsize=8, color="#333", rotation=90)


def _finish(ax, *, xlim, ylim, aspect=1):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect(aspect)
    ax.grid(True, color="#e6e6e6", linewidth=0.5)
    ax.set_axisbelow(True)


def render() -> None:
    fig = plt.figure(figsize=(14, 9), dpi=140, facecolor="white")
    gs = fig.add_gridspec(2, 3, width_ratios=[1.2, 1.2, 1.6],
                          height_ratios=[1, 1], wspace=0.25, hspace=0.28)

    ax_side  = fig.add_subplot(gs[0, 0])
    ax_front = fig.add_subplot(gs[0, 1])
    ax_top   = fig.add_subplot(gs[1, 0])
    ax_info  = fig.add_subplot(gs[:, 2])
    # dedicate second row/col1 to info that spills over
    ax_ref   = fig.add_subplot(gs[1, 1])

    draw_side(ax_side)
    draw_front(ax_front)
    draw_top(ax_top)
    draw_info(ax_info)

    # Try to embed the real photo as a reference if available.
    real_photo = OUT_PATH.parent / "pguard_real_oilsite.jpg"
    if real_photo.exists():
        img = plt.imread(real_photo)
        ax_ref.imshow(img)
        ax_ref.set_title("Reference: real PGuard on patrol\n"
                         "(© Enova Robotics, enovarobotics.eu)",
                         fontsize=9)
        ax_ref.axis("off")
    else:
        ax_ref.axis("off")

    fig.suptitle(
        "PGuard: real robot vs. our simulation URDF",
        fontsize=13, fontweight="bold", y=0.99)

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PATH, dpi=140, bbox_inches="tight", facecolor="white")
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    render()
