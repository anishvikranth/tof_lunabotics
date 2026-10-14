# tof_lunabotics

ROS 2 crater detection from a downward-tilted TFmini Plus ToF sensor (via MAVROS).

`crater_detector` publishes `std_msgs/Bool` on `/crater_detected`: `true` when the range
jumps above the flat-ground distance d by more than `jump_threshold`, else `false`.

## Install (clone as the workspace `src` folder)

```bash
mkdir -p ~/crater_ws && cd ~/crater_ws
git clone git@github.com:anishvikranth/tof_lunabotics.git src
colcon build --symlink-install
source install/setup.bash
ros2 launch crater_detector crater_detector.launch.py
```

Parameters: `src/crater_detector/config/crater_detector.yaml`.
Recalibrate ground distance: `ros2 service call /crater_detector/recalibrate std_srvs/srv/Trigger`.

## Crater mapper (crater layer for Nav2)

`crater_mapper` projects each ToF reading into the `map` frame with TF (`map → … → lidar`),
tracks the local ground level, and marks points more than `depth_threshold` below ground.

| Output | Type | Use |
|---|---|---|
| `/crater_map` | `nav_msgs/OccupancyGrid` (transient local) | Nav2 static layer; same size/origin as RTAB-Map's `/map` |
| `/crater_cloud` | `sensor_msgs/PointCloud2` | alternative: a dedicated Nav2 obstacle layer |
| `/crater_info` | `std_msgs/String` (JSON) | size of the largest crater: center, length, width, diameter, depth |

Clear the map: `ros2 service call /crater_mapper/clear std_srvs/srv/Trigger`.
Parameters: `crater_detector/config/crater_mapper.yaml`.

Requires a working TF tree `map → odom → base_link` (RTAB-Map / ZED). Only one node may
publish `map → odom` (set the ZED wrapper's `publish_map_tf: false` when RTAB-Map runs).

### Run everything

```bash
cd ~/crater_ws && colcon build --symlink-install --packages-select crater_detector
source install/setup.bash
ros2 launch crater_detector crater_system.launch.py            # detector + mapper + base_link->lidar TF
# mount arguments (defaults shown):
ros2 launch crater_detector crater_system.launch.py x:=0.30 z:=0.45 pitch_deg:=20.0
# URDF already publishes base_link->lidar:
ros2 launch crater_detector crater_system.launch.py publish_static_tf:=false
```

### Nav2 costmap (global and local)

`use_maximum` is a costmap-level parameter: it makes static layers merge by maximum
instead of overwriting each other, so `/crater_map` cannot erase RTAB-Map's walls.

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      use_maximum: true
      track_unknown_space: true
      plugins: ["static_layer", "crater_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /map
        map_subscribe_transient_local: true
      crater_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /crater_map
        map_subscribe_transient_local: true
      # obstacle_layer (ZED point cloud) and inflation_layer as usual
```

Local costmap: same `use_maximum: true` and `crater_layer` block, with `rolling_window: true`.
