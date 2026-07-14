# How the Nav2 Costmap is Calculated

The costmap is what Nav2 actually plans on. It combines the static map (from `build_map.py`), live sonar observations, and an inflation halo into a single 2-D cost grid. This doc explains **exactly how each cell's cost is derived**, layer by layer, from raw inputs to the final value the planner reads.

Configuration lives in `src/my_pguard_bot/config/nav2_params.yaml`.

---

## 1. Two costmaps, one planner

Nav2 maintains **two independent costmaps** simultaneously:

| Costmap | Purpose | Size | Resolution | Frame |
|---|---|---|---|---|
| **Global** | Long-horizon planning (`planner_server` uses this) | 1400 × 1400 m, fixed | 1.0 m/cell | `map` |
| **Local** | Short-horizon obstacle avoidance (`controller_server` uses this) | 15 × 15 m, rolling | 0.15 m/cell | `odom` |

The global costmap is anchored to the world datum. The local costmap slides with the robot (`rolling_window: true`) and is much finer to catch nearby dynamic obstacles.

---

## 2. Nav2 cost values

Every cell holds a `uint8` cost:

| Value | Symbol | Meaning |
|:---:|---|---|
| 0 | `FREE_SPACE` | Traversable, no penalty |
| 1–252 | (gradient) | Increasing cost — planner avoids but can cross |
| 253 | `INSCRIBED_INFLATED_OBSTACLE` | Robot's inscribed radius touches an obstacle here |
| 254 | `LETHAL_OBSTACLE` | Would collide — planner forbids |
| 255 | `NO_INFORMATION` | Unknown (only visible if `track_unknown_space: true`) |

The planner (`NavfnPlanner`) sums costs along candidate paths and picks the minimum-cost route. Any cell ≥ 253 is treated as a hard wall.

---

## 3. Global costmap — layer stack

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 1.0
      width: 1400          # metres
      height: 1400
      origin_x: -700.0     # centres the costmap on the world datum
      origin_y: -700.0
      track_unknown_space: true
      rolling_window: false
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

Layers are applied **in order**. Each layer can only raise a cell's cost or convert `NO_INFORMATION` into something else. The final value at each cell is the max produced by any layer.

### 3.1 Static layer — the pre-built map

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: true
  subscribe_to_updates: true
  trinary_costmap: false
  lethal_cost_threshold: 100
```

`map_server` loads `maps/novation_city.pgm` + `.yaml` and publishes it on `/map`. The static layer subscribes and translates each PGM pixel to a costmap cell.

**Pixel → probability** (with `negate: 0` in the yaml):

```
p = (255 - pixel) / 255
```

**Probability → cost** — this is where `trinary_costmap` matters:

Our PGM has **three** pixel values from `build_map.py`:

| pixel | probability p | tier |
|:---:|:---:|---|
| 0 | 1.000 | building |
| 100 | 0.608 | grass / off-road |
| 254 | 0.004 | road / footway |

With yaml thresholds `occupied_thresh: 0.65` and `free_thresh: 0.20`:

**Case A — `trinary_costmap: true` (what we do NOT want)**
- `p ≥ 0.65` → `LETHAL (254)` — buildings
- `p ≤ 0.20` → `FREE (0)` — roads
- otherwise → `LETHAL (254)` — **grass becomes lethal too**, robot cannot leave a road even in emergencies

**Case B — `trinary_costmap: false` (what we use)**
- `p ≥ 0.65` → `LETHAL (254)` — buildings
- `p ≤ 0.20` → `FREE (0)` — roads
- otherwise → scaled cost from `lethal_cost_threshold`

For grass (`p ≈ 0.608`) with `lethal_cost_threshold: 100`, the layer emits a cost around **90–130** — high enough that the planner strongly prefers roads, but not lethal, so PGuard can cut across grass to reach a stranded goal or dodge a road blockage.

**Result** on the current PGM (1200 × 1200 map = 1.44 M cells):

| Zone | Fraction | Cost after static layer |
|---|---:|:---:|
| Buildings | 1.26 % | 254 (LETHAL) |
| Roads | 5.92 % | 0 (FREE) |
| Grass / off-road | 92.82 % | ~90 (non-lethal gradient) |

### 3.2 Obstacle layer — live sonar

```yaml
obstacle_layer:
  observation_sources: sonar_front sonar_rear sonar_left sonar_right
  sonar_front: {topic: /sonar/front, data_type: LaserScan,
                obstacle_max_range: 5.0, raytrace_max_range: 6.0,
                marking: true, clearing: true, max_obstacle_height: 2.0}
  # ... 3 more, symmetric on rear/left/right
```

Each of the 4 sonars publishes a `sensor_msgs/LaserScan`. For every scan:

1. **Marking**: any return within `obstacle_max_range: 5.0` m stamps the corresponding cell as `LETHAL (254)`.
2. **Clearing**: ray-trace from the sensor to `raytrace_max_range: 6.0` m; any previously-marked cell along the free portion of the ray is reset to `FREE (0)`.
3. `max_obstacle_height: 2.0` m — only returns within 2 m of ground plane count (ignores tree canopies, overhangs).

This layer is what makes the robot react to obstacles that aren't in the static map (parked cars, pedestrians, temporary barriers).

### 3.3 Inflation layer — safety halo

```yaml
inflation_layer:
  cost_scaling_factor: 2.0
  inflation_radius: 2.0     # metres
```

For every `LETHAL (254)` cell, the layer paints a decaying cost gradient outward up to `inflation_radius: 2.0` m. The cost at distance `d` from a lethal cell is:

```
cost(d) = 252 · exp(-cost_scaling_factor · (d - inscribed_radius))
```

where `inscribed_radius` is the robot's inscribed circle radius (see §5.1). For our `robot_radius: 0.95` and `cost_scaling_factor: 2.0`:

| distance from wall | cost |
|:---:|:---:|
| ≤ 0.95 m (inscribed) | 253 |
| 1.0 m | ~229 |
| 1.5 m | ~84 |
| 2.0 m | ~31 |
| > 2.0 m | 0 |

Higher `cost_scaling_factor` → steeper falloff (robot hugs walls). Lower → smoother, wider avoidance corridor. `2.0` is our tuned value for a 1.5×1.1 m chassis at ~1.5 m/s cruise.

### 3.4 Final global cost = max over all three layers

For a road-adjacent grass cell 1.2 m from a building:

- Static: ~90 (grass gradient)
- Obstacle: 0 (nothing there)
- Inflation: ~180 (close to a building)

Cell final cost = **max(90, 0, 180) = 180** — planner will strongly avoid, but not forbid.

---

## 4. Local costmap — layer stack

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      resolution: 0.15         # 6.7 cells / metre
      width: 15
      height: 15
      rolling_window: true
      robot_radius: 0.95
      plugins: ["obstacle_layer", "inflation_layer"]
```

The local costmap **has no static layer** — it's built entirely from live observations. It's fine (0.15 m/cell) and rolls with the robot so it always represents the immediate 15 m surroundings.

### 4.1 Local obstacle layer

Same 4 sonars as global, same `marking`/`clearing`/`max_range` values. The higher resolution means a single sonar return marks a small cluster of cells rather than one big blob.

### 4.2 Local inflation layer

```yaml
inflation_layer:
  cost_scaling_factor: 3.0    # steeper than global (3.0 vs 2.0)
  inflation_radius: 1.5       # tighter halo (1.5 m vs 2.0 m)
```

Steeper + tighter than the global layer. The controller needs enough margin to slow down but not so much that it can't thread a driveway.

| distance from obstacle | local cost |
|:---:|:---:|
| ≤ 0.95 m | 253 |
| 1.0 m | ~226 |
| 1.25 m | ~106 |
| 1.5 m | 0 |

---

## 5. Robot footprint

```yaml
robot_radius: 0.95
```

Nav2 supports two footprint models:
- **Circular** (what we use) — a single circumscribed radius. Cheap, symmetric.
- **Polygon footprint** — arbitrary polygon, more accurate for elongated chassis.

For our 1.5 m × 1.1 m Ackermann chassis, the **circumscribed radius** = √(0.75² + 0.55²) ≈ **0.93 m**. We round up to **0.95 m** to include the antenna mount and mudguards.

The **inscribed radius** (largest circle that fits inside the footprint) is 0.55 m, half the chassis width. Nav2 derives it internally from the footprint. `INSCRIBED_INFLATED_OBSTACLE (253)` fires when a cell is within that inscribed radius of any lethal cell.

If we ever swap to a polygon footprint, replace `robot_radius: 0.95` with:

```yaml
footprint: "[[0.85, 0.55], [0.85, -0.55], [-0.65, -0.55], [-0.65, 0.55]]"
```

(centre at rear axle, x forward, y left — standard REP-103).

---

## 6. Publishing rates

| Quantity | Rate | Why |
|---|---|---|
| `global_costmap.update_frequency` | 1.0 Hz | Global is huge (1400×1400); only obstacle+inflation refresh, static rarely changes |
| `global_costmap.publish_frequency` | 1.0 Hz | For RViz + dashboard |
| `local_costmap.update_frequency` | 5.0 Hz | Reactive to fast obstacles |
| `local_costmap.publish_frequency` | 2.0 Hz | Enough for RViz |
| `controller_frequency` | 20.0 Hz | Regulated Pure Pursuit control loop |

`always_send_full_costmap: true` on both — keeps RViz + dashboard in sync at the cost of ~5 MB/s on the global topic. Acceptable indoors; if you're bandwidth-constrained flip to incremental updates.

---

## 7. Sanity-checking the costmap live

Once the stack is up:

```bash
# Global costmap raw values
ros2 topic echo /global_costmap/costmap --no-arr | head -20

# Static layer only
ros2 topic hz /map

# Inspect a specific cell (map frame, metres)
ros2 topic echo /global_costmap/costmap_updates
```

In **RViz2**:
- Add a `Map` display, topic `/global_costmap/costmap`, colour scheme `costmap` — you see the full stack (static + inflated).
- Add another `Map` display, topic `/map`, colour scheme `map` — you see only the raw static input.
- The difference is the inflation halo.

In the **web dashboard**:
- The map background (`dashboard/novation_city_color.png`) is a static render of the PGM. It does **not** show inflation.
- The green polyline is the `NavigateToPose` result — you can visually confirm the planner is following roads.

---

## 8. Common failure modes

### "Planner produces no path"

Almost always one of:

1. **Goal is on a lethal cell** — e.g. inside a building. Nav2 refuses. Move the goal 2+ m away from any wall.
2. **Start pose is unknown** — no `map → odom` transform. Check EKF: `ros2 topic hz /odometry/filtered`.
3. **Costmap didn't load** — `map_server` failed silently. Check `ros2 lifecycle get /map_server` returns `active`, and `ros2 service call /map_server/get_map …` returns a non-empty grid.

### "Robot won't leave the driveway"

Usually `trinary_costmap: true` accidentally re-set — grass becomes lethal, robot is boxed in. Flip back to `false`.

### "Robot cuts across grass to every goal"

The road-preference bug from §12.1 of `PROJECT.md`. Verify:

1. `maps/novation_city.pgm` has **three** distinct pixel values (roads at 254). `hexdump -C maps/novation_city.pgm | head -5` should show a mix of `00`, `64`, `fe`.
2. `nav2_params.yaml` `global_costmap.plugins` **includes `static_layer`** first.
3. `maps/` is listed in `src/my_pguard_bot/CMakeLists.txt` `install(DIRECTORY …)` — else `install/share/my_pguard_bot/maps/` will be empty.

Rebuild:

```bash
python3 src/my_pguard_bot/scripts/build_map.py
colcon build --symlink-install --packages-select my_pguard_bot
```

### "Robot oscillates near walls"

Inflation layer is too shallow. Increase `inflation_radius` from 2.0 → 2.5 m or drop `cost_scaling_factor` from 2.0 → 1.5 for a smoother gradient.

### "Robot ignores a nearby obstacle"

Sonar out of range (> 5 m) or below the ground return threshold. Check `ros2 topic echo /sonar/front` — should publish at ~10 Hz with `range < 5.0` when something's in front.

---

## 9. Quick reference — the numbers that matter

| Parameter | Value | File |
|---|---:|---|
| Global costmap size | 1400 × 1400 m | `nav2_params.yaml` |
| Global resolution | 1.0 m/cell | `nav2_params.yaml` |
| Local costmap size | 15 × 15 m | `nav2_params.yaml` |
| Local resolution | 0.15 m/cell | `nav2_params.yaml` |
| Robot radius | 0.95 m | `nav2_params.yaml` |
| Global inflation radius | 2.0 m | `nav2_params.yaml` |
| Local inflation radius | 1.5 m | `nav2_params.yaml` |
| Sonar obstacle range | 5.0 m | `nav2_params.yaml` |
| Sonar raytrace range | 6.0 m | `nav2_params.yaml` |
| Static PGM resolution | 1.0 m/cell | `maps/novation_city.yaml` |
| Static PGM extent | 1200 × 1200 m | `build_map.py` |
| `occupied_thresh` | 0.65 | `maps/novation_city.yaml` |
| `free_thresh` | 0.20 | `maps/novation_city.yaml` |
| `lethal_cost_threshold` | 100 | `nav2_params.yaml` (static layer) |

---

## 10. See also

- **`MAP_GENERATION.md`** — how `novation_city.pgm` is built from OSM
- **`PROJECT.md`** — full stack overview and the road-preference bug post-mortem (§12.1)
- **[Nav2 Costmap 2D docs](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)** — upstream reference
