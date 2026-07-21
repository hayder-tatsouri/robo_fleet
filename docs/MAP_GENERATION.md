# Novation City Map Generation

How the Gazebo world (`novation_city.sdf`), the Nav2 occupancy grid (`novation_city.pgm` + `.yaml`), and the perimeter patrol waypoints are built from real OpenStreetMap data of Technopôle de Sousse, Tunisia.

**All scripts live in `src/my_pguard_bot/scripts/` and can be re-run independently.**

---

## 1. The datum

Everything is anchored to a single geodetic origin:

| Parameter | Value |
|---|---|
| Latitude | 35.8173° N |
| Longitude | 10.5912° E |
| Elevation | 50.0 m |
| World frame orientation | ENU (X=East, Y=North, Z=Up) |
| Fetch radius | 600 m |
| Map extent | 1200 m × 1200 m |

This point is roughly the Enova / Novation City HQ. It appears in three places:

- `LAT0`, `LON0` constants in `fetch_osm.py` and `generate_waypoints.py`
- `<spherical_coordinates>` block in `worlds/novation_city.template.sdf`
- Nav2 `map_server` picks up the offset from `maps/novation_city.yaml` (`origin: [-600, -600, 0]`)

If you change the datum, change it in **all four** places or GPS↔map alignment breaks.

---

## 2. Pipeline overview

```
                       (needs internet)
                              │
                              ▼
   ┌──────────────────┐   ┌────────────────┐   ┌──────────────────────┐
   │ Overpass API     │──►│ fetch_osm.py   │──►│ worlds/              │
   │ (openstreetmap)  │   │                │   │  sousse_buildings.sdf│
   └──────────────────┘   └────────────────┘   │  sousse_pois.json    │
                                                └──────────┬───────────┘
                                                           │
                        ┌──────────────────────────────────┼──────────────┐
                        │                                  │              │
                        ▼                                  ▼              ▼
             ┌───────────────────┐             ┌──────────────────┐  ┌────────────────────────┐
             │ build_world.py    │             │ build_map.py     │  │ generate_waypoints.py  │
             │ template + blds   │             │ SDF → 3-tier PGM │  │ hull + offset + resample│
             └─────────┬─────────┘             └────────┬─────────┘  └────────────┬───────────┘
                       ▼                                ▼                         ▼
             worlds/novation_city.sdf         maps/novation_city.pgm    config/patrol_waypoints.yaml
             (Gazebo world)                   maps/novation_city.yaml   (lat/lon waypoint loop)
```

Only **step 1** needs internet. Everything else is deterministic and offline.

---

## 3. `fetch_osm.py` — OSM → Gazebo SDF models

### 3.1 Overpass query

```
[out:json][timeout:60];
(
  way["building"](around:600, 35.8173, 10.5912);
  relation["building"](around:600, 35.8173, 10.5912);
  way["highway"](around:600, 35.8173, 10.5912);
  node["office"](around:600, 35.8173, 10.5912);
  node["amenity"](around:600, 35.8173, 10.5912);
  node["shop"](around:600, 35.8173, 10.5912);
);
out body;
>;
out skel qt;
```

Fetch strategy: `urllib.request.urlopen` first, then fall back to `curl` (the Amazon Linux 2 base image has a broken Python SSL trust store; `curl` uses the system CA bundle).

### 3.2 Projection (lat/lon → local ENU metres)

Equirectangular projection, accurate at small radii:

```
R = 6378137.0
x = radians(lon - LON0) · cos(radians(LAT0)) · R
y = radians(lat - LAT0) · R
```

Inverse (used by `generate_waypoints.py`) is trivial.

### 3.3 Buildings → oriented bounding boxes

Each `way["building"]` is a polygon of lat/lon nodes. We:

1. Project every node to local ENU.
2. Compute the **convex hull** (Andrew's monotone chain).
3. Compute the **minimum-area oriented bounding box** using the rotating-calipers property: the min-area OBB is aligned with one of the hull edges. We enumerate each edge, rotate the hull so that edge becomes horizontal, take the axis-aligned bbox, and keep the smallest.
4. Result: `(cx, cy, w, d, yaw)` — centre, width, depth, rotation.

Building height:
- `tags["height"]` — used directly if present (e.g. `"12 m"` → 12.0)
- `tags["building:levels"]` — multiplied by 3.2 m/level
- else default **6.0 m**

Each building emits one static SDF `<model>` with a `<box>` collision and visual, tan colour `(0.75, 0.72, 0.65)`.

### 3.4 POI matching (the trick that labels tiny offices)

OSM often stores small companies as **nodes** (`office=company`, `amenity=cafe`, `shop=*`) rather than tagging their building. To make labels like "Enova", "VEO", "Proxym" appear on real buildings:

For each named POI node:
1. Project to local ENU.
2. Test `point_in_obb` against every building's OBB (with 1 m tolerance).
3. If it hits, the building takes on the POI's name and gets highlighted **company blue** `(0.35, 0.55, 0.85)`.
4. If it doesn't hit any building (POI is on a plaza, road, etc.), synthesise a small 14 m × 10 m × 6 m box at the POI's position so the label still shows.

### 3.5 Roads

Each `way["highway"]` is a linestring. We:

1. Project every node to local ENU.
2. **Merge near-collinear segments** to reduce model count. Threshold: consecutive segments whose direction differs by < 8° are fused into one.
3. Emit each merged segment as a thin static box, rotated to lie along the segment.
4. Add **corner-fill boxes** at joints between drivable segments (`w × w`) to smooth intersection appearance. Skipped for `footway`/`path`/`cycleway`/`pedestrian` where visual smoothing isn't needed.

Width and colour come from an OSM-class → style dispatch table:

| OSM `highway` | width (m) | grey level |
|---|---|---|
| `trunk` | 8.0 | 0.18 |
| `primary` | 7.0 | 0.20 |
| `secondary` | 6.0 | 0.22 |
| `tertiary` | 5.5 | 0.25 |
| `unclassified` / `residential` | 4.5 | 0.28 |
| `service` | 3.0 | 0.32 |
| `living_street` | 3.5 | 0.32 |
| `footway` / `path` | 1.2–1.5 | 0.55 |
| `cycleway` | 1.5 | 0.35, 0.45, 0.30 (greenish) |
| `pedestrian` | 2.5 | 0.55 |

All road boxes are stamped at a small positive Z (0.010–0.020 m) so they render above the ground plane but under the wheels.

### 3.6 Outputs

- `worlds/sousse_buildings.sdf` — one XML file with **26 building models + ~1000 road segment models** (as of the current query). Wrapped in an outer `<sdf version="1.10">` but no `<world>` — that comes from the template.
- `worlds/sousse_pois.json` — `{datum: [lat, lon], pois: [{name, kind, lat, lon, x, y}, …]}`. Useful for the web dashboard's map overlay and for future POI-based waypoint synthesis.

**Fail-safe**: if the Overpass fetch raises (offline, DNS block, timeout), an empty stub SDF is written and the script exits 1. Rerun when you're back online.

---

## 4. `build_world.py` — template + buildings → final world SDF

Trivial but necessary because SDF `<include>` requires a model directory hierarchy (name/model.config/model.sdf), which is overkill for a one-off scene:

1. Read `worlds/novation_city.template.sdf`.
2. Read `worlds/sousse_buildings.sdf` and extract the inner content (everything between `<sdf …>` and `</sdf>`).
3. Substitute at the `<!-- BUILDINGS_HERE -->` marker in the template.
4. Write `worlds/novation_city.sdf`.

The **template** provides everything that isn't buildings/roads:

- Physics, scene-broadcaster, sensors, IMU, NavSat plugins
- `<spherical_coordinates>` (the ENU datum)
- Sun light + ambient scene
- Ground plane (tan, 2000 × 2000 m, mu=1.0)
- Two headless cameras used by the web dashboard:
  - `chase_cam` — 960×540, publishes to `world_cam/chase`, moved at runtime by `aim_chase_cam.py`
  - `top_cam` — 1024×1024, publishes to `world_cam/top`, fixed at 220 m altitude

---

## 5. `build_map.py` — SDF → Nav2 occupancy grid

The map rasterizer that makes Nav2 prefer roads. See **[`COSTMAP.md`](./COSTMAP.md)** for the costmap math and Nav2-layer wiring — this section only covers the PGM generation.

### 5.1 Grid geometry

| Parameter | Value |
|---|---|
| Extent | 1200 m × 1200 m |
| Resolution | 1.0 m/cell |
| Grid size | 1200 × 1200 cells (1.44 M cells, ~1.4 MB PGM) |
| Origin | `(-600, -600, 0)` — world (0, 0) sits at the centre of the image |

### 5.2 Three-tier encoding

Instead of the usual binary occupied/free encoding, we use three grey levels chosen so that Nav2's `negate=0` interpretation `p = (255 − pixel) / 255` lands in the right zones:

| pixel | probability p | tier | Nav2 outcome |
|:---:|:---:|---|---|
| 0 | 1.00 | buildings | **LETHAL** (cost 254) |
| 100 | 0.608 | grass / off-road | **NO_INFORMATION** (cost ~90, non-lethal) |
| 254 | 0.004 | roads / footways | **FREE** (cost 0) |

With `occupied_thresh: 0.65` and `free_thresh: 0.20`:
- `p ≥ 0.65` → lethal (buildings only)
- `p ≤ 0.20` → free (roads only)
- otherwise → no-information (grass) — planner can cross but pays a heavy cost

### 5.3 Rasterization

1. Regex-parse `sousse_buildings.sdf` for both `bld_*` and `road_*` models. Extract `(cx, cy, w, d, yaw)` from `<pose>` and `<box><size>`.
2. Initialise the grid to `V_OFFROAD = 100` everywhere.
3. **Stamp roads first** with value 254, then **buildings on top** with value 0 — so a building crossed by a road segment still shows as lethal.
4. For each oriented box:
   - Compute an AABB around the OBB (radius `r = √(w² + d²) / 2 + 0.5`) as the pixel scan window.
   - For every pixel in that window, rotate `(wx − cx, wy − cy)` by `−yaw` and test if it falls in the local `[±w/2, ±d/2]` rectangle.
5. Write the PGM (P5 binary). Vertically flip while writing — PGM row 0 is image-top, our grid row 0 is south.

### 5.4 Outputs

- `maps/novation_city.pgm` — 1200×1200 raw P5 (~1.4 MB)
- `maps/novation_city.yaml`:

```yaml
image: novation_city.pgm
resolution: 1.0
origin: [-600.0, -600.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.20
```

Typical distribution on the current dataset:
- 1.26 % buildings (lethal)
- 5.92 % roads (free)
- 92.82 % off-road (no-info)

That road fraction is why the planner tracks pavement instead of cutting across grass.

---

## 6. `generate_waypoints.py` — perimeter patrol loop

Produces `config/patrol_waypoints.yaml` consumed by `patrol_client.py` for the autonomous perimeter loop.

### 6.1 Algorithm

1. Load every building AABB from `sousse_buildings.sdf` (regex on `bld_*` model poses + sizes). Emit 4 axis-aligned corners per building.
2. **Convex hull** of all corners (Andrew's monotone chain, CCW, no duplicate endpoint).
3. **Offset outward** by `SAFETY_M = 12.0` m using mitered edge-normal joins:
   - For each hull vertex, take the two adjacent edge outward normals.
   - Bisect them, scale by `d / cos(half-angle)`.
   - Clamp miter length to `3·d` to avoid spikes at sharp corners.
4. **Resample** the offset polygon at `SPACING_M = 100.0` m arc-length spacing. `n_samples = max(6, round(perimeter / 100))` — currently ~20 waypoints for a ~2-minute lap in sim.
5. For each waypoint, compute `yaw = atan2(next.y − this.y, next.x − this.x)` (tangent-following heading).
6. Inverse-project each `(x, y)` back to `(lat, lon)`. Emit YAML with `lat/lon/yaw_deg/enu_x/enu_y` per waypoint.

Fallback: if fewer than 4 building corners are parsed (broken SDF), emit a default 40 m square loop.

### 6.2 Output

```yaml
# Auto-generated by scripts/generate_waypoints.py
# Datum: lat=35.8173, lon=10.5912
# Perimeter offset: 12.0 m,  spacing: 100.0 m
# Waypoints: 20
waypoints:
  - {lat: 35.81836000, lon: 10.59245000, yaw_deg: -142.31, enu_x: 118.42, enu_y: 117.86}
  - ...
```

---

## 7. Re-running the pipeline

### 7.1 Full regeneration (needs internet for step 1)

```bash
cd ~/robo_fleet     # or wherever your clone lives

python3 src/my_pguard_bot/scripts/fetch_osm.py
python3 src/my_pguard_bot/scripts/build_world.py
python3 src/my_pguard_bot/scripts/build_map.py
python3 src/my_pguard_bot/scripts/generate_waypoints.py

colcon build --symlink-install --packages-select my_pguard_bot
source install/setup.bash
```

### 7.2 Offline rebuild (edit the template, keep OSM data)

If you just tweaked `novation_city.template.sdf` (cameras, lighting, plugins) but not the OSM query:

```bash
python3 src/my_pguard_bot/scripts/build_world.py
colcon build --symlink-install --packages-select my_pguard_bot
```

### 7.3 Change the datum or radius

Edit **both** places:

1. `scripts/fetch_osm.py`: `LAT0`, `LON0`, `RADIUS_M`
2. `worlds/novation_city.template.sdf`: `<spherical_coordinates>` block

Also consider updating:
- `scripts/build_map.py`: `EXTENT_M` (map size) if you moved beyond ±600 m
- `scripts/generate_waypoints.py`: `LAT0`, `LON0`
- `config/nav2_params.yaml`: `global_costmap.width/height/origin_x/origin_y`

Then rerun the whole pipeline.

---

## 8. What's already committed

All four outputs are checked into the repo so a fresh clone works without internet:

- `src/my_pguard_bot/worlds/sousse_buildings.sdf` (~0.4 MB, **26 buildings + 2 aggregate road models** — each aggregate holds all segments of a given street as child `<visual>`s so Gazebo loads 30× faster)
- `src/my_pguard_bot/worlds/sousse_pois.json`
- `src/my_pguard_bot/worlds/novation_city.sdf`
- `src/my_pguard_bot/maps/novation_city.pgm` + `.yaml`
- `src/my_pguard_bot/config/patrol_waypoints.yaml`

Rerun the scripts only if you want to refresh OSM data or change parameters.

### 8.1 If you regenerate with an older `fetch_osm.py`

Old versions emitted **one `<model>` per road segment** (~1000+ top-level models), which made Gazebo load extremely slowly. To collapse an existing SDF without re-hitting Overpass:

```bash
python3 src/my_pguard_bot/scripts/collapse_road_models.py
python3 src/my_pguard_bot/scripts/build_world.py
python3 src/my_pguard_bot/scripts/build_map.py
colcon build --symlink-install --packages-select my_pguard_bot
```

The collapse script:
- Preserves every `<visual>` (identical rendering)
- Emits `<!-- ROAD_SEG ... -->` XML comments that `build_map.py` regex-parses to rasterize road cells into the PGM
- Keeps buildings untouched

---

## 9. Historical footnote — the road-preference bug

Before the current rewrite (see `docs/PROJECT.md §12.1`), `build_map.py` **only rasterised buildings** — roads were ignored. The map came out **98.74 % free**, so Nav2 had zero incentive to stay on pavement and PGuard cut straight across grass to reach every goal.

Three concurrent bugs made it a puzzle:

1. `build_map.py` never stamped roads.
2. `nav2_params.yaml`'s `global_costmap.plugins` was missing `static_layer` — even if the map had been correct, Nav2 wasn't reading it.
3. `CMakeLists.txt`'s `install(DIRECTORY …)` didn't include `maps/`, so nothing landed in `install/share/my_pguard_bot/maps/` and `map_server` failed with `bad file novation_city.yaml`.

All three are fixed in the current branch. See `COSTMAP.md` for the full costmap layer stack.
