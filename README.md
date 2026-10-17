# tof_lunabotics

ROS 2 crater (negative obstacle) detection and avoidance for the Lunabotics rover, using a
**TFmini Plus ToF rangefinder** tilted down in front of the rover. The ZED 2i camera and
RTAB-Map handle positive obstacles; this project adds the holes they cannot see.

- `crater_detector` gives a fast **stop signal**: `/crater_detected` (true/false).
- `crater_mapper` builds a **crater map** for Nav2: `/crater_map` (OccupancyGrid).

Status: both nodes work in simulation and in a hand-held laptop test with the ZED.
On the rover (Orin), MAVROS is installed and connects, but the Pixhawk currently reports
`PreArm: Rangefinder 1: Not Detected`, so no end-to-end rover test has been done yet.
See **[Pending work](#pending-work)**.

---

## Contents
1. [Hardware and data flow](#hardware-and-data-flow)
2. [Repository structure](#repository-structure)
3. [How the nodes work](#how-the-nodes-work)
4. [Topics, services, frames](#topics-services-frames)
5. [Install and build](#install-and-build)
6. [Running](#running)
7. [Rover integration (orin_files)](#rover-integration-orin_files)
8. [Pixhawk / ArduPilot setup](#pixhawk--ardupilot-setup)
9. [What has been tested](#what-has-been-tested)
10. [Known issues and gotchas](#known-issues-and-gotchas)
11. [Pending work](#pending-work)

---

## Hardware and data flow

```
TFmini Plus (I2C, addr 0x10)
   → Pixhawk 2.4.8 "Pixhawk1" (ArduCopter 4.7.0 firmware, used only as a sensor hub)
   → USB /dev/ttyACM0
   → MAVROS on the companion computer → /mavros/rangefinder_pub (sensor_msgs/Range)
   → crater_detector → /crater_detected
   → crater_mapper (+ TF map → … → tof from RTAB-Map/EKF/URDF) → /crater_map → Nav2 costmaps
```

| Machine | OS / ROS | Use |
|---|---|---|
| Rover: Nvidia Orin (`nano@…`) | Ubuntu 22.04, ROS 2 **Humble** | the real system |
| Dev laptop | Ubuntu 24.04, ROS 2 **Jazzy** | development, simulation, hand-held tests |

The code uses only APIs common to Humble and Jazzy (rclpy, tf2_ros, numpy).

**Sensor mount (current plan):** ToF on top of the ZED 2i, centred, tilted **33° down from
horizontal**. With the lens ≈ 0.65 m above the ground this gives a flat-ground reading
**d ≈ 1.2 m**, hitting the ground ≈ 1.0 m ahead. Not yet measured on the real rover.

TFmini Plus I2C wiring to the Pixhawk I2C port: red = 5 V, black = GND, white = SDA,
green = SCL. The sensor must be switched to I2C mode once (it ships in UART mode).

---

## Repository structure

The repo **is** the `src` folder of a colcon workspace (`~/crater_ws/src`).

```
.
├── README.md
├── crater_detector/                    ROS 2 ament_python package (the only package)
│   ├── crater_detector/
│   │   ├── crater_detector_node.py     range-jump detector → /crater_detected
│   │   └── crater_mapper_node.py       TF-projected crater grid → /crater_map, /crater_cloud, /crater_info
│   ├── config/
│   │   ├── crater_detector.yaml        detector parameters (commented)
│   │   ├── crater_mapper.yaml          mapper parameters (commented)
│   │   ├── mavros_crater_pluginlists.yaml   MAVROS: only 5 plugins
│   │   └── mavros_crater_overrides.yaml     MAVROS: timesync off, rangefinder frame "tof"
│   ├── launch/
│   │   ├── crater_detector.launch.py   detector only
│   │   ├── crater_system.launch.py     detector + mapper + optional static TF (base → tof)
│   │   └── mavros_crater.launch.py     minimal MAVROS for the ToF
│   ├── package.xml, setup.py, setup.cfg, resource/, test/ (template lint tests only)
├── orin_files/                         ready-to-copy versions of rover files (see Rover integration)
│   ├── cac_rover_tof_test.urdf         rover URDF + tof link
│   ├── nav2_params_3_tof_test.yaml     Nav2 params + crater layer
│   └── rover_bringup_tof_test.py       rover bringup + crater nodes
└── stolen_files/                       unmodified copies of the rover team's files (reference only)
    ├── cac_rover.urdf / .xacro (+ _gz, _ign simulation variants)
    ├── nav2_params_3.yaml
    ├── rover_bringup.launch.py
    └── gay.txt                         the RTAB-Map launch command the team uses (shell alias)
```

`orin_files/` and `stolen_files/` have no `package.xml`, so colcon ignores them.
The real versions of those files live in the rover's own packages on the Orin
(`cac_rover_description`, `zed_aruco_localization`, …), which are **not** in this repo.

---

## How the nodes work

### crater_detector (`crater_detector_node.py`): fast stop signal, no TF needed
1. **Calibrate:** median of the first 30 valid readings = flat-ground distance **d**
   (or set `baseline` to a fixed value). Rover must be still on flat ground at startup.
2. **Filter:** median of the last 5 readings = d_new (removes single spikes).
3. **Classify:** crater point if `d_new − d > jump_threshold`, or if there is no return
   (range ≥ max_range).
4. **Debounce:** `/crater_detected` flips only after 3 consecutive agreeing readings.
5. **Watchdog:** warns "Sensor not connected" if no data for 1 s, and calls
   `/mavros/set_message_interval` to request `DISTANCE_SENSOR` at 20 Hz (needed because the
   Pixhawk's stream rates are 0). Warns "MAVROS not running" if that service is missing.

Geometry: a crater of depth D lengthens the reading by D / sin(tilt). At 33°,
`jump_threshold: 0.09` ≈ craters ≥ 5 cm deep. **Re-tune it if the tilt changes**
(`jump_threshold ≈ min_depth / sin(tilt)`).
Limitation: d is fixed, so it assumes constant sensor height and tilt (true on the rover,
not when the rig is carried by hand; use `~/recalibrate`).

### crater_mapper (`crater_mapper_node.py`): crater map for Nav2
For every reading:
1. Look up TF `map → tof` at the reading's stamp (falls back to the latest transform;
   **no waiting**, see Known issues). Skips the reading if the pose is older than
   `max_tf_age` (0.25 s), e.g. when ZED tracking is lost.
2. Hit point = sensor origin + range × beam direction (sensor +x axis) in `map`.
3. **Ground level:** median z of the first 30 readings once they agree within 5 cm; then a
   rolling median of ground points (follows gentle slopes).
4. Depth = ground level − z:
   - |depth| < `ground_tol` (5 cm) → ground: cell = 0 (free), remember as "last ground point".
   - depth > `depth_threshold` (8 cm) → crater point.
   - otherwise (bumps, in-between) → ignored.
5. **Mark a crater point:** +1 hit in a disc of `mark_radius` (12 cm), plus along the line
   from the last ground point (only if < 0.3 s old and < 1.5 m away: the beam passed over
   that ground). **No return** → mark where the beam would have hit flat ground.
6. A cell becomes **100 (crater)** after `confirm_hits` (3) hits and is never cleared
   (except via `~/clear`).

**Grid geometry follows `map_topic`** (RTAB-Map's grid): same size, origin and resolution,
re-gridding and keeping marks when it grows. This keeps the two Nav2 static layers from
resizing each other. Until that map arrives, a fallback 20×20 m grid is used.

**Outputs** (published only when something changed, at ≤ 2 Hz):
- `/crater_map`: 100 crater / 0 seen ground / −1 unknown.
- `/crater_cloud`: crater cells as points 0.2 m above ground (alternative Nav2 input).
- `/crater_info`: JSON for the largest crater: 8-connected clusters → convex hull →
  minimum-area rectangle (length, width), diameter, depth (measured, or the 0.50 m
  assumption if nothing deeper was measured).

---

## Topics, services, frames

| Name | Type | Publisher → user |
|---|---|---|
| `/mavros/rangefinder_pub` | `sensor_msgs/Range` (best effort) | MAVROS → both nodes |
| `/crater_detected` | `std_msgs/Bool` | detector → (future) behaviour node |
| `/crater_detector/baseline` | `std_msgs/Float32` | detector → debugging |
| `/crater_map` | `nav_msgs/OccupancyGrid` (transient local) | mapper → Nav2 `crater_layer`, RViz |
| `/crater_cloud` | `sensor_msgs/PointCloud2` (transient local) | mapper → optional obstacle layer |
| `/crater_info` | `std_msgs/String` JSON (transient local) | mapper → logging |
| `/crater_detector/recalibrate` | `std_srvs/Trigger` | re-measure d |
| `/crater_mapper/clear` | `std_srvs/Trigger` | wipe the crater map (keeps ground level) |

Frames on the rover: `map → odom → base_footprint → base_link → elecbox_link →
zed_camera_link → tof`.
- `map → odom`: RTAB-Map. `odom → base_footprint`: EKF (`robot_localization`), expected.
- `zed_camera_link → tof`: from the URDF (`orin_files/cac_rover_tof_test.urdf`), or from
  `crater_system.launch.py` when `publish_static_tf:=true`.
- `zed_camera_link` is the ZED's bottom tripod-screw hole (x forward, y left, z up).
- Only **one** node may publish `map → odom` (ZED wrapper must have `publish_map_tf: false`).

---

## Install and build

```bash
mkdir -p ~/crater_ws && cd ~/crater_ws
git clone https://github.com/anishvikranth/tof_lunabotics.git src
source /opt/ros/$ROS_DISTRO/setup.bash            # humble on the Orin, jazzy on the laptop
sudo apt install -y ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install && source install/setup.bash
```

MAVROS one-time setup:
```bash
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
sudo usermod -aG dialout $USER      # then log out/in
sudo apt remove -y modemmanager     # it grabs /dev/ttyACM0
```

With `--symlink-install`, edits to Python files and YAML configs take effect after
restarting the node (no rebuild).

---

## Running

Every terminal: `source /opt/ros/$ROS_DISTRO/setup.bash && source ~/crater_ws/install/setup.bash`
(on the rover also source the rover workspace).

### Sensor only (bench)
```bash
ros2 launch crater_detector mavros_crater.launch.py fcu_url:=/dev/ttyACM0:115200
ros2 launch crater_detector crater_detector.launch.py
ros2 topic echo /crater_detected
```
Expect `Requested DISTANCE_SENSOR at 20 Hz` → `Sensor connected` → `Ground distance d = …`.

### Full system on the rover (Orin): test order
1. **MAVROS:** `ros2 launch crater_detector mavros_crater.launch.py fcu_url:=/dev/ttyACM0:115200`
   (wait for `CON: Got HEARTBEAT`).
2. **Rover bringup** (ZED via zed_aruco_loc, URDF, EKF, zed_tf_node):
   `ros2 launch <bringup_pkg> rover_bringup.launch.py`. Check `ros2 topic hz /odometry/filtered`.
3. **RTAB-Map** (command from `stolen_files/gay.txt`):
   ```bash
   ros2 launch rtabmap_launch rtabmap.launch.py \
     rtabmap_viz:=false visual_odometry:=false odom_topic:=/odometry/filtered \
     rgb_topic:=/zed/zed/rgb/color/rect/image \
     camera_info_topic:=/zed/zed/rgb/color/rect/camera_info \
     depth_topic:=/zed/zed/depth/depth_registered approx_sync:=true \
     args:="--delete_db_on_start --Grid/CellSize 0.06 --Grid/MaxObstacleHeight 1.0 --Grid/MaxGroundAngle 15 \
            --GridGlobal/MinSize 50.0 --Optimizer/Robust true --Reg/Force3DoF false \
            --Grid/MinGroundHeight -0.15 --Grid/RayTracing true"
   ```
   Check `ros2 run tf2_ros tf2_echo map base_footprint` keeps updating.
4. **Crater nodes:**
   ```bash
   # URDF with the tof link installed:
   ros2 launch crater_detector crater_system.launch.py publish_static_tf:=false map_topic:=/rtabmap/grid_prob_map
   # old URDF (no tof link): publish it from the launch file
   ros2 launch crater_detector crater_system.launch.py base_frame:=zed_camera_link \
     x:=0.0 z:=0.041 pitch_deg:=33 map_topic:=/rtabmap/grid_prob_map
   ```
   Expect `Ground distance d ≈ 1.2 m`, `Crater grid now follows /rtabmap/grid_prob_map`,
   `Ground level set`, and no `No TF` / `Pose … old` warnings.
5. **RViz:** Fixed Frame `map`; Map `/rtabmap/grid_prob_map`; Map `/crater_map`
   (Durability **Transient Local**); TF.
6. Teleop slowly (≤ 0.2 m/s) toward a step or hole; watch `/crater_detected`,
   `/crater_map`, `ros2 topic echo /crater_info`.
7. Nav2 with `orin_files/nav2_params_3_tof_test.yaml`; goal on the far side of a crater.

### Laptop without a rover
- **Hand-held:** ZED wrapper alone provides `map → odom → zed_camera_link`
  (`ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i`), then
  `crater_system.launch.py base_frame:=zed_camera_link x:=… z:=… pitch_deg:=…`.
- **Bench, no odometry:** `ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link`.

### Launch arguments of `crater_system.launch.py`
`x`, `y`, `z` (m) and `pitch_deg` (positive = down) of the ToF relative to `base_frame`
(default `base_link`), `sensor_frame` (`tof`), `publish_static_tf` (`true`),
`map_topic` (`/map`). Show them with `--show-args`.

---

## Rover integration (`orin_files`)

Copies of the rover team's files with the crater changes applied. On the Orin, back up the
original, copy the file over it (drop the `_tof_test` suffix), rebuild that package.

| File | Replaces | Change vs. original (`stolen_files/`) |
|---|---|---|
| `cac_rover_tof_test.urdf` | `cac_rover_description/urdf/cac_rover.urdf` | adds `tof` link: `zed_camera_link → tof`, xyz (0, 0, 0.041), pitch 0.576 rad (33°). **Placeholder values, measure them.** |
| `nav2_params_3_tof_test.yaml` | the team's Nav2 params | `use_maximum: true` and `crater_layer` (StaticLayer on `/crater_map`) in local and global costmaps; nothing else changed |
| `rover_bringup_tof_test.py` | `rover_bringup.launch.py` | includes `crater_system.launch.py` (URDF tof frame, `/rtabmap/grid_prob_map`) and optional stock MAVROS (`start_mavros:=true fcu_url:=…`) |

Why `use_maximum: true`: by default a Nav2 StaticLayer **overwrites** the layers below it,
so `/crater_map`'s unknown/free cells would erase RTAB-Map's walls. With it, layers merge by
maximum. It is a costmap-level parameter, not a per-layer one. Crater value 100 ≥ the
global `lethal_cost_threshold: 60`, so craters are lethal.

Notes on the team's Nav2 config: `robot_base_frame: base_footprint`, max speed 0.2 m/s,
`robot_radius` 0.5, local costmap 3×3 m rolling in `odom`, **no ZED obstacle layer**.

---

## Pixhawk / ArduPilot setup

Current parameters (read via MAVROS):

| Parameter | Value | Meaning |
|---|---|---|
| `RNGFND1_TYPE` | 25 | Benewake TFmini Plus over I2C |
| `RNGFND1_ADDR` | 16 | I2C address 0x10 |
| `RNGFND1_MIN` / `RNGFND1_MAX` | 0.2 / 8.0 m | valid range |
| `RNGFND1_ORIENT` | 0 | forward (MAVROS does not check it) |
| `MAV1_EXTRA3` | **0** | USB stream group off; the detector requests only `DISTANCE_SENSOR` |

In ArduPilot 4.7 the old `SR0_*` stream-rate parameters are named `MAV1_*` (USB link).
ArduPilot only scans I2C sensors at boot: **reboot the Pixhawk after (re)connecting the ToF**
(unplug USB, or `ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 246, param1: 1.0}"`).
PreArm errors about RC, gyros or battery only block arming and can be ignored.

---

## What has been tested

| Test | Result |
|---|---|
| Simulated rover approaching a 1 m × 0.5 m crater, ±40° sweep | detector true at the crater; 110 crater cells, **all** within 15 cm of the real rim, none on flat ground; size estimate 0.79 × 0.46 m (underestimate, see Pending) |
| Simulated 3 s pose freeze | mapper skips readings with `Pose … is 0.3 s old`, no wrong marks |
| `/map` size/origin/resolution change | grid re-maps, marks kept |
| Laptop + Pixhawk + ToF + ZED 2i (hand-held) | sensor 20 Hz, TF `map → tof` OK, ground level set, free cells marked |
| Minimal MAVROS launch | exactly 5 plugins load, timesync off, services present |
| URDF copy in robot_state_publisher | `base_footprint → tof` as expected |
| **Rover (Orin)** | MAVROS connects; **ToF not detected by the Pixhawk, no data yet** |
| Nav2 with the crater layer | **not tested** |

---

## Known issues and gotchas

- **`ros2 param`/`ros2 node` says "Node not found" for MAVROS** (both machines): the ROS 2
  daemon misses MAVROS's sub-nodes. Use `--no-daemon --spin-time 5`, or call the service
  directly (`ros2 service call /mavros/param/set_parameters …`).
- **No rangefinder data although MAVROS is connected:** the Pixhawk's stream rates are 0.
  The detector requests `DISTANCE_SENSOR` itself; check the MAVROS log for
  `Rangefinder 1: Not Detected` (sensor/wiring/reboot problem, not ROS).
- **Odd readings (0.10 m or ~11.8 m constant):** sensor blocked or no real return. Fixed
  once on the laptop by re-seating the sensor.
- **`TM: RTT too high for timesync` (seconds), `WP: timeout`:** the full MAVROS launch
  overloads the USB link. Use `mavros_crater.launch.py` (5 plugins, timesync off).
- **TF "extrapolation into the past" in the mapper:** was caused by `lookup_transform`
  with a timeout blocking the node's single-threaded executor (fixed: no timeout). If it
  reappears, check whether an odometry link really froze (ZED tracking `LOST` = spatial
  memory status 7 on `/zed/zed_node/pose/status`).
- **ZED "CAMERA STREAM FAILED TO START":** camera enumerated at USB 2.0
  (`lsusb -t` shows 480M). Flip the USB-C plug at the camera, use a direct USB 3 port and
  the original cable.
- **Detector stuck at true:** d was calibrated at a different sensor height/tilt. Call
  `/crater_detector/recalibrate` with the sensor in its working pose on flat ground.
- **Double Ctrl+C** can print a `KeyboardInterrupt` traceback in `destroy_node`; harmless.

---

## Pending work

In rough priority order.

1. **Fix `PreArm: Rangefinder 1: Not Detected` on the rover.** Check TFmini wiring after the
   remount on the ZED (SDA/SCL not swapped, 5 V present), then reboot the Pixhawk.
2. **Run the rover test order** (Running → Full system), stages 1–6, and record results.
3. **Measure the real ToF mount** relative to the ZED screw hole (x, y, z, tilt) and update
   `orin_files/cac_rover_tof_test.urdf`. Confirm d ≈ 1.2 m on flat ground; if the tilt
   changes, update `jump_threshold` (≈ 0.05 / sin(tilt)).
4. **Install the `orin_files`** into the rover packages (back up originals).
5. **Check for TF conflicts** on the running rover: `ros2 run tf2_tools view_frames`.
   Not yet reviewed: `cac_rover_description/config/ekf.yaml` (should publish
   `odom → base_footprint`), the `zed_tf_node` source (must not duplicate URDF
   `base_link → zed_camera_link`), `zed_aruco_loc.launch.py` (must not publish
   `map → odom` while RTAB-Map does).
6. **Verify Nav2 merging:** with `/crater_map` publishing, RTAB-Map's obstacles must still
   appear in the global costmap (confirms `use_maximum` on Humble), and a goal behind a
   crater must route around it.
7. **Write the stop-and-scan behaviour node** (not started). On `/crater_detected` = true:
   stop (cancel the Nav2 goal or a zero-velocity `twist_mux` input), rotate in place
   +40° → −40° → 0 at ≈ 10°/s so the mapper covers the crater, optionally back up 0.3 m
   and repeat, then resend the goal. Subscribe with queue depth 1 and treat a value older
   than ~1 s as stale (the detector stops publishing if the sensor drops out).
8. **Switch the bringup's optional MAVROS** in `rover_bringup_tof_test.py` from the stock
   `apm.launch` to `mavros_crater.launch.py`.
9. **Tune on real terrain** (sand, rocks): `jump_threshold`, `depth_threshold`,
   `confirm_hits`, `mark_radius`, inflation radius.
10. **Improve size estimates:** a single stop + sweep underestimates (near wall hidden;
    0.79 × 0.46 m for a 1 × 1 m crater in simulation). Add a second scan from 0.3 m back.
11. **Consider a ZED obstacle layer** in Nav2 (the team's config has none) with
    `min_obstacle_height` ≈ 0.05 m.
12. Nice to have: crater marks do not move with RTAB-Map loop closures; `/crater_info` is a
    JSON string (could become a custom message); no unit tests beyond the template lint tests;
    the detector does not use IMU pitch.

### Background
The original algorithm design doc (Claude Docs, private, ask the repo owner for access):
"ToF Crater Detection & Avoidance — Algorithm". Some numbers there predate later decisions
(20° tilt, `/map` topic); this README is the current source of truth.
