---
description: Run every check that works without ROS or hardware, then report what still needs the robot
---

Verify the `raftar_lidar_pkg` package as far as is possible without a ROS
install or the rover. Run these from `src/raftar_lidar_pkg`:

1. **Unit tests** — `python -m pytest test/ -q`
   Covers `geo.py` (GPS→ENU) and `scan_utils.py` (LaserScan sector reading,
   including the wrap-around cases the top LiDAR depends on).

2. **Lint** — `python -m pyflakes raftar_lidar_pkg/*.py launch/*.py`
   Catches undefined names and unused imports. Must be silent.

3. **Syntax** — `python -m compileall -q raftar_lidar_pkg launch setup.py`

4. **Cross-file consistency** — these are the failure mode this repo is
   most prone to, because they fail *silently* at runtime:
   - Every topic in `config/nav2_local_costmap.yaml` under
     `observation_sources` must be published by a remap in
     `launch/rover_advance_v2.launch.py`.
   - No physical constant (mount offset, mount angle, frame name, topic
     name, serial port) may be hardcoded outside `rover_config.py`. Grep
     for stray `/scan`, `laser_`, `base_link`, `map`, `odom` string
     literals in the nodes and launch files.
   - `top_center_deg` / `low_center_deg` must be derived via
     `forward_bearing_deg()`, never typed as a number.

5. If ROS 2 is available (check `command -v colcon`), also run
   `colcon build --symlink-install` and report the result. If it is not
   available, say so plainly rather than skipping silently.

Then report:
- What passed.
- What is **not** covered by any of the above and still needs hardware:
  LiDAR mount orientation (`TOP_LIDAR_YAW_DEG`), the yaw-rate sign
  (`invert_yaw_rate`), MAVROS connectivity, and Nav2 lifecycle activation.

Do not run `ros2 run`, `ros2 launch`, `ros2 topic pub`, `ros2 service call`
or `ros2 param set` as part of this. They command a real vehicle.
