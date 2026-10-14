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
