# Outdoor Robot Simulation with GPS and Maps using ROS 2 and Gazebo

## Introduction

Simulating outdoor robots with GPS-based navigation requires a carefully orchestrated pipeline connecting Gazebo's physics engine with ROS 2's navigation and localization stacks. Unlike indoor scenarios where pre-built maps and AMCL suffice, outdoor navigation relies on geodetic coordinates, satellite-based positioning, and sensor fusion to operate across large, unstructured environments. This report covers the complete stack: world creation with geo-referencing, GPS sensor simulation, map integration strategies, Nav2 configuration for outdoor use, and the end-to-end pipeline tying everything together.

The stack targeted here is **ROS 2 Jazzy + Gazebo Harmonic (Sim 8.x)** on Ubuntu 24.04. Gazebo Classic reached end-of-life in January 2025 and is deliberately not covered.

## Geo-Referenced World Setup in Gazebo

The foundation of outdoor simulation is anchoring the Gazebo world to real GPS coordinates via the WGS84 geodetic system. Gazebo internally operates in Cartesian coordinates but provides a spherical coordinate system that projects between WGS84 and a local tangent plane. [1](https://gazebosim.org/api/sim/10/spherical_coordinates.html)

The world SDF file defines the origin's geodetic position using the `<spherical_coordinates>` element:

```xml
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>47.397742</latitude_deg>
  <longitude_deg>8.545594</longitude_deg>
  <elevation>488.0</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
```

The ENU (East-North-Up) convention maps X to East, Y to North, and Z to Up. This frame orientation must be consistent throughout the pipeline — particularly in how `robot_localization` interprets IMU data.

**Heading convention gotcha:** Gazebo's `<heading_deg>` is measured from East toward North (counterclockwise), *not* from North clockwise like a compass. To align the world's +X axis with true North, set `heading_deg=90`. Getting this wrong rotates every subsequent GPS-derived pose by a constant offset.

For terrain, Gazebo supports heightmaps from DEM (Digital Elevation Model) data via GDAL. [2](https://get.gazebosim.org/tutorials?cat=build_world&tut=dem) The heightmap image must have side length **(2ⁿ) + 1** pixels (e.g., 65, 129, 257, 513, 1025); Gazebo's heightmap loader will reject other sizes. Resample your DEM with:

```bash
gdalwarp -ts 513 513 input.tif heightmap.tif
gdal_translate -of PNG -ot UInt16 -scale heightmap.tif heightmap.png
```

For a 100 m × 100 m plot, 513 samples/side ≈ 20 cm/pixel resolution; 129 samples/side ≈ 78 cm/pixel and is usually too coarse for anything but very smooth terrain. Choose the smallest power-of-two-plus-one that preserves the features you care about — larger heightmaps significantly increase load time and physics cost.

For populated outdoor environments, the **Forest3D** tool automates terrain generation from DEM data with procedural placement of trees, rocks, and vegetation for Gazebo Harmonic. [3](https://discourse.openrobotics.org/t/forest3d-generate-populated-outdoor-environments-for-gazebo/51551)

## GPS/NavSat Sensor Simulation

Gazebo Harmonic provides a native NavSat sensor type that computes latitude, longitude, and altitude from the robot's Cartesian position relative to the world's spherical coordinates. The NavSat system plugin must be loaded at the **world level** — not the model level. [4](https://robotics.stackexchange.com/questions/107536/how-to-use-gazebo-ignition-navsat-plugin)

A critical gotcha: **explicitly declaring plugins in `<world>` overrides Gazebo's default server config** (`~/.gz/sim/8/server.config`). Once you add any `<plugin>` element to the world, you must include the Physics, Sensors, SceneBroadcaster, UserCommands, IMU, and NavSat plugins yourself, or those subsystems silently disappear.

The sensor is attached to a robot link via URDF's `<gazebo>` extension tag:

```xml
<gazebo reference="gps_link">
  <sensor name="navsat_sensor" type="navsat">
    <always_on>1</always_on>
    <update_rate>10</update_rate>
    <topic>navsat</topic>
    <gz_frame_id>gps_link</gz_frame_id>
    <navsat>
      <position_sensing>
        <horizontal>
          <noise type="gaussian"><mean>0.0</mean><stddev>0.5</stddev></noise>
        </horizontal>
        <vertical>
          <noise type="gaussian"><mean>0.0</mean><stddev>1.0</stddev></noise>
        </vertical>
      </position_sensing>
    </navsat>
  </sensor>
</gazebo>
```

Realistic noise parameters vary widely by receiver class: [5](https://www.mdpi.com/1424-8220/20/21/6050)

| Receiver class | Horizontal stddev | Vertical stddev |
|---|---|---|
| Consumer / smartphone GPS | 3–10 m | 5–15 m |
| Automotive-grade | 1–3 m | 2–5 m |
| DGPS / SBAS-corrected | 0.3–1 m | 0.5–2 m |
| RTK (fixed solution) | 0.01–0.05 m | 0.02–0.10 m |

**Critical unit-of-measure gotcha:** The Gazebo Harmonic NavSat plugin applies horizontal position noise **directly in degrees of latitude/longitude, not meters** (as of gz-sim 8.x). One degree of latitude is ~111 km, so a stddev of `1.5` produces GPS fixes that jump hundreds of kilometers between samples. To model 1.5 m of horizontal noise you need `<stddev>1.35e-5</stddev>` (1.5 / 111000). Vertical noise is applied in meters, matching intuition. This asymmetry is a documented quirk of the plugin — verify with `gz topic -e -t /navsat` if your GPS fixes look wildly off.

Match your `<stddev>` values to the hardware you intend to deploy on — an EKF tuned against 3 m noise in sim will behave very differently against 3 cm RTK data on hardware.

**Sim-to-real caveat:** the NavSat plugin does not model **multipath**, **satellite geometry (DOP variation)**, or **signal loss under tree canopy or near buildings**. On real hardware these are the dominant error sources. If your simulation needs to stress-test degraded-GPS behavior, drop the `/gps/fix` topic programmatically in geometrically plausible regions, or inject time-varying noise via a wrapper node.

## Map Integration Strategies

Several approaches exist for integrating real-world maps into the simulation pipeline:

**OpenStreetMap to Gazebo:** The `gazebo_osm` tool converts OSM data into SDF world files with roads, buildings, and basic geometry. [6](https://github.com/osrf/gazebo_osm)

**Satellite imagery ground planes:** Gazebo's `libStaticMapPlugin.so` downloads Google Maps imagery at runtime to create textured ground planes. [7](https://get.gazebosim.org/tutorials?cat=build_world&tut=static_map_plugin)

**Occupancy grid generation:** For Nav2 path planning, maps can be generated without SLAM using tools like the `gazebo_map_creator` plugin, which produces PGM/YAML files directly from the Gazebo world. [8](https://medium.com/@arshad.mehmood/ros2-gazebo-world-map-generator-a103b510a7e5) The newer **SDF2MAP** tool converts SDF files directly to occupancy grids without running the simulation. [9](https://discourse.openrobotics.org/t/sdf2map-generate-ros-2d-maps-directly-from-gzsim-worlds/50598)

**Geo-referenced visualization:** Mapviz provides 2D top-down visualization with satellite tile overlays, ideal for monitoring outdoor robots. [10](https://roboticsknowledgebase.com/wiki/tools/mapviz/)

## Sensor Fusion with `robot_localization`

The `robot_localization` package provides the sensor fusion backbone for outdoor navigation via a dual-EKF architecture. [11](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)

The recommended pattern uses two EKF instances:

1. **EKF Local** (`world_frame: odom`) — fuses wheel odometry + IMU, publishes `odom → base_link` TF. Provides smooth, continuous localization.
2. **EKF Global** (`world_frame: map`) — fuses the same inputs PLUS GPS-derived odometry from `navsat_transform_node`, publishes `map → odom` TF to correct drift.

The `navsat_transform_node` converts `sensor_msgs/NavSatFix` into `nav_msgs/Odometry` using a **local Cartesian projection** anchored at the datum (the origin lat/lon), or UTM if `broadcast_utm_transform` is enabled. Its output feeds the global EKF's position input. [12](https://docs.ros.org/en/jazzy/p/robot_localization/)

Key configuration points:
- GPS odometry should only contribute x,y position to the global EKF (not orientation or velocity).
- Set `odom1_differential: false` — the GPS provides **absolute** position; differential mode would double-integrate it into nonsense.
- Set the datum explicitly to match your Gazebo `<spherical_coordinates>` origin. This is what closes the loop between the simulated world and the ROS `map` frame.

## Nav2 GPS Waypoint Navigation

Nav2's GPS Waypoint Follower accepts `geographic_msgs/GeoPose` waypoints and navigates to them using the standard `NavigateToPose` action server. It converts GPS coordinates to the map frame via `robot_localization`'s `/fromLL` service. [[13](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html), [14](https://github.com/ros-navigation/navigation2/pull/2814)]

For outdoor use, Nav2 parameters differ significantly from indoor defaults:

| Parameter | Indoor Default | Outdoor Setting | Reason |
|-----------|----------------|-----------------|--------|
| `xy_goal_tolerance` | 0.25 m | 1.0–2.0 m (consumer GPS); 0.1–0.3 m (RTK) | Match to receiver accuracy |
| `allow_unknown` | false | true | No pre-built map |
| `lookahead_dist` | 0.6 m | 2.0–5.0 m | Larger open spaces |
| `controller_frequency` | 20 Hz | 10 Hz (pure GPS waypoints); 20 Hz (with LiDAR obstacle avoidance) | Rendering + sensor rates |
| `movement_time_allowance` | 10 s | 30 s | GPS delay tolerance |

The **Regulated Pure Pursuit** controller is well-suited for outdoor paths with its velocity scaling and larger lookahead distances.

## The `ros_gz_bridge`

Gazebo Harmonic communicates via Gazebo Transport (not ROS topics), so `ros_gz_bridge` translates between them. A YAML configuration maps Gazebo topics to ROS 2. **Use the split `ros_topic_name` / `gz_topic_name` schema** — the older single `topic_name` key is a legacy shortcut and will fail whenever the two names differ:

```yaml
- ros_topic_name: "/gps/fix"
  gz_topic_name: "/navsat"
  ros_type_name: "sensor_msgs/msg/NavSatFix"
  gz_type_name: "gz.msgs.NavSat"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

Launch with:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml
```

**The clock bridge is essential.** ROS 2 nodes started with `use_sim_time: true` block indefinitely on the first `now()` call until a `/clock` message arrives. Missing the clock bridge manifests as nodes appearing to hang or refusing to publish anything — a symptom often misdiagnosed as a QoS or DDS problem.

## TF and Sensor Mounting

Static transforms between the robot base and its sensors (`base_link → gps_link`, `base_link → imu_link`, `base_link → lidar_link`, etc.) should be published on `/tf_static`, not `/tf`. Emit them from the URDF via `robot_state_publisher` — never handroll them into `tf2_ros static_transform_publisher` for permanent sensor mounts, which sends duplicate messages on `/tf` and can pollute the TF tree.

Only two dynamic transforms should exist in the pipeline:

- `odom → base_link` — published by **EKF Local** only.
- `map → odom` — published by **EKF Global** only.

If either transform gets a second publisher (e.g. Gazebo's ground-truth pose plugin also emits `odom → base_link`), TF will silently pick whichever arrives most recently and produce erratic behavior.

## Common Pitfalls and Troubleshooting

The most frequent issues when setting up this pipeline:

**NavSat outputs all zeros.** The `gz-sim-navsat-system` plugin is missing from the world SDF, or `<spherical_coordinates>` is not defined. Both are mandatory.

**ENU/NED mismatch.** If your IMU reports in NED (North-East-Down) but `robot_localization` expects ENU, set `yaw_offset: 1.5707963` (π/2) in `navsat_transform_node` config.

**Duplicate TF transforms.** As above: only the local EKF publishes `odom → base_link`; only the global EKF publishes `map → odom`. [15](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml)

**PROJ library errors.** Gazebo requires the PROJ data files for coordinate projection. Missing `proj.db` causes spherical coordinates to silently fail. [16](https://robotics.stackexchange.com/questions/114450/proj-error-when-trying-to-set-spherical-coordinates-in-gazebo)

**Nodes hang and publish nothing in sim.** `use_sim_time: true` is set but the `/clock` bridge is not running. Every ROS 2 node in the sim graph must consume `/clock` for TF timestamps to be consistent.

**EKF diverges on real hardware but works in sim.** Nearly always a **time-sync problem** between IMU and GPS. In simulation `use_sim_time: true` gives you free perfect sync; on hardware you need NTP or PTP between the sensor sources.

## Multi-Robot Considerations

For multi-robot outdoor deployments, isolate each robot's ROS graph with a unique `ROS_DOMAIN_ID` (0–101) or run them all on one domain and namespace their topics (`/robot1/gps/fix`, `/robot2/gps/fix`, …). The `ros_gz_bridge` supports a `namespace` parameter that prefixes all bridged topics — pair this with per-robot bridge configs to keep the sim graph tidy.

## Recommended Resources and Reference Implementations

The following repositories provide working end-to-end examples:

| Repository | Description |
|-----------|-------------|
| [17](https://github.com/SaxionMechatronics/smart_diffbot) | Complete diff-drive robot with GNSS navigation in Gazebo + Nav2 |
| [18](https://github.com/ros-navigation/navigation2_tutorials) | Official `nav2_gps_waypoint_follower_demo` package |
| [19](https://github.com/rosblox/nav2_outdoor_example) | Minimal GNSS/IMU localization with modern Gazebo |
| [20](https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors) | GPS waypoint following tutorial with Gazebo Harmonic |
| [21](https://github.com/saiaravind19/gazebo_terrain_generator) | Real-world elevation + satellite terrain for Gazebo |

For the official Nav2 GPS navigation tutorial, start at [13](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html). The `smart_diffbot` repository from SaxionMechatronics is particularly valuable as it provides a tested, complete ROS 2 workspace with all launch files, configs, and URDF ready to run. [17](https://github.com/SaxionMechatronics/smart_diffbot)


## References

\[1\] https://gazebosim.org/api/sim/10/spherical_coordinates.html

\[2\] https://get.gazebosim.org/tutorials?cat=build_world&tut=dem

\[3\] https://discourse.openrobotics.org/t/forest3d-generate-populated-outdoor-environments-for-gazebo/51551

\[4\] https://robotics.stackexchange.com/questions/107536/how-to-use-gazebo-ignition-navsat-plugin

\[5\] https://www.mdpi.com/1424-8220/20/21/6050

\[6\] https://github.com/osrf/gazebo_osm

\[7\] https://get.gazebosim.org/tutorials?cat=build_world&tut=static_map_plugin

\[8\] https://medium.com/@arshad.mehmood/ros2-gazebo-world-map-generator-a103b510a7e5

\[9\] https://discourse.openrobotics.org/t/sdf2map-generate-ros-2d-maps-directly-from-gzsim-worlds/50598

\[10\] https://roboticsknowledgebase.com/wiki/tools/mapviz/

\[11\] https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html

\[12\] https://docs.ros.org/en/jazzy/p/robot_localization/

\[13\] https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html

\[14\] https://github.com/ros-navigation/navigation2/pull/2814

\[15\] https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml

\[16\] https://robotics.stackexchange.com/questions/114450/proj-error-when-trying-to-set-spherical-coordinates-in-gazebo

\[17\] https://github.com/SaxionMechatronics/smart_diffbot

\[18\] https://github.com/ros-navigation/navigation2_tutorials

\[19\] https://github.com/rosblox/nav2_outdoor_example

\[20\] https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors

\[21\] https://github.com/saiaravind19/gazebo_terrain_generator
