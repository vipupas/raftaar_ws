# CLAUDE.md — agent briefing for `raftaar_ws`

Read this before changing anything. It is written for a coding agent picking
up the repo cold.

## What this is

A **ROS 2 Humble workspace** containing one package, `raftar_lidar_pkg`.
It adds LiDAR obstacle avoidance to a **large ArduPilot rover** (2.19 m ×
2.10 m, skid-steer) whose flight controller is a CubePilot **CubeOrange+**.

The whole design is one sentence:

> The rover normally drives its own GPS mission in ArduPilot **AUTO** mode.
> When a LiDAR sees an obstacle, ROS switches it to **GUIDED**, steers
> around the obstacle with Nav2's local planner, then switches back to AUTO.

**This code drives a vehicle that can injure people.** Treat every change to
velocity, mode, or geometry as safety-critical. See "Safety rules" below.

## Architecture

```
 LiDARs ──▶ obstacle_mode_switcher_dual ──▶ /mavros/set_mode (AUTO⇄GUIDED)
                      │
                      └──▶ /obstacle_detected ──▶ coordinator_node
                                                        │
                                          straight-line FollowPath goal
                                                        ▼
                          local costmap ──▶ controller_server (MPPI)
                                                        │  /cmd_vel
                                                        ▼
                                              velocity_smoother
                                                        │  /cmd_vel_smoothed
                                                        ▼
                                mav_to_teleop ──▶ /mavros/setpoint_raw/local
```

**Ownership is split and must stay split:**

- `obstacle_mode_switcher_dual` is the **only** thing that changes flight
  mode. Never add a `set_mode` call anywhere else — two nodes fighting over
  one setting is a bug that only shows up in the field.
- `coordinator_node` **only** manages Nav2 goals. It has no mode authority.

## File map

| File | Role |
|---|---|
| `raftar_lidar_pkg/rover_config.py` | **Single source of truth** for geometry, frames, topics, serial ports |
| `raftar_lidar_pkg/geo.py` | GPS → local ENU (pure Python, no ROS, no deps) |
| `raftar_lidar_pkg/scan_utils.py` | Read a sector out of a LaserScan (pure Python, no ROS) |
| `raftar_lidar_pkg/obstacle_mode_switcher_dual.py` | **Live** detector + mode switcher |
| `raftar_lidar_pkg/coordinator_node.py` | Nav2 goal management, mission tracking, stuck watchdog |
| `raftar_lidar_pkg/mav_to_cmd_vel.py` | `Twist` → `PositionTarget` (entry point `mav_to_teleop`) |
| `raftar_lidar_pkg/obstacle_mode_switcher_3.py` | 3-LiDAR variant, not launched |
| `raftar_lidar_pkg/obstacle_mode_switcher_adv.py` | 1-LiDAR variant, not launched |
| `raftar_lidar_pkg/object_detect_rplidar.py` | Simplest variant, used by the bench launch |
| `launch/rover_advance_v2.launch.py` | **The** launch file. Everything else defers to it |
| `launch/rover_advance.launch.py` | Thin wrapper: v2 with `start_mavros:=false` |
| `launch/mode_switch_with_lidar.launch.py` | Bench test: 1 LiDAR + RViz, no Nav2 |
| `launch/Static_transform.launch.py` | TF only, for alignment checks |
| `config/nav2_local_costmap.yaml` | Costmap + MPPI controller + velocity smoother |

## Invariants — do not break these

**1. Physical facts live in `rover_config.py` and nowhere else.**
Mount positions, mount angles, frame names, topic names, serial ports.
Several past bugs were the same fact written in two places that drifted
apart. If you need a constant in a launch file, `import rover_config as cfg`
— launch files can import it because the package installs to site-packages.

**2. A sensor's mount yaw drives two things and they are derived, not typed.**
The static transform (`TOP_LIDAR_YAW_DEG`) tells the costmap where a hit
really is. `forward_bearing_deg()` tells the detector which bearing in the
*sensor's own* frame is "straight ahead". Never hardcode `top_center_deg`.

**3. Topic names must agree across three places** — the launch remap, the
node parameter, and `config/nav2_local_costmap.yaml`. A costmap subscribed
to a topic nobody publishes fails **silently**: no error, no data, planner
blind. Always grep all three after a topic change.

**4. TF tree is `map → odom → base_link → {laser_top, laser_low}`.**
`map→odom` is static (launch). `odom→base_link` is live (coordinator).
Every frame has exactly one parent. If MAVROS is ever configured with
`tf/send: true`, set `publish_odom_tf: false` on the coordinator or the
tree gets two publishers for one edge.

**5. ROS is ENU, MAVLink `FRAME_BODY_NED` is NED.** Positive `Twist.angular.z`
is a **left** turn; positive MAVLink `yaw_rate` is a **right** turn. The
conversion in `mav_to_cmd_vel.py` negates it (`invert_yaw_rate`, default
`True`). Getting this wrong makes the rover steer *into* obstacles.

**6. Nav2 plugin and critic names carry suffixes.** It is
`nav2_mppi_controller::MPPIController`, and critics are `ObstaclesCritic`,
`PathFollowCritic`, … — not `Obstacles`, `PathFollowed`. Wrong names mean
`controller_server` fails to configure at startup.

**7. Mission item 0 is HOME.** Never drive to `waypoints[0]`. Use
`coordinator_node._target_waypoint()`, which prefers ArduPilot's own
`is_current` flag and skips items without a real lat/lon.

## Reference material in `.claude/`

`.claude/rules/` and `.claude/skills/` hold vendored ROS 2 and Nav2
reference documentation — see `.claude/VENDORED.md` for its provenance and
licence status. **Consult `rules/nav2_parameters.md` and
`skills/nav2_controllers/SKILL.md` before touching
`config/nav2_local_costmap.yaml`**; they carry the correct plugin and
critic identifiers, which is the exact thing that was wrong in this repo
before. `skills/ros2_transforms/` is worth reading before changing the TF
tree.

Those files describe generic ROS 2. Where they conflict with this document,
**this document wins** — it is the one that knows about the rover.

## Build and test

**This repo is developed on Windows but only builds on Linux/WSL.** ROS 2
is not installed on the Windows side.

```bash
# In WSL (Ubuntu 22.04 = Humble)
cd ~/raftaar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

**`geo.py`, `scan_utils.py` and `rover_config.py` import nothing from ROS
on purpose** — so their logic is testable anywhere, including on Windows:

```bash
cd src/raftar_lidar_pkg
python -m pytest test/ -q          # no ROS needed
python -m pyflakes raftar_lidar_pkg/*.py launch/*.py
```

Keep it that way. If you add pure geometry or maths, put it in one of those
three modules and add tests, rather than burying it in a node where it can
only be exercised on the robot.

## Safety rules for an agent working here

- **Never run `ros2 topic pub`, `ros2 service call`, `ros2 param set`,
  `ros2 run`, or `ros2 launch` against live hardware without the user
  explicitly asking.** These move a 2-metre vehicle.
- Changes to velocity limits, `invert_yaw_rate`, mount geometry, or mode
  logic must be flagged to the user as requiring a **bench test with the
  wheels off the ground** before any field run.
- Do not "clean up" the debounce / cooldown / watchdog logic to make it
  simpler. Each guard exists because of a specific failure mode, documented
  in the docstrings.
- Prefer failing safe. `nearest_in_sector` returns `inf` (= clear) for an
  empty scan, but every caller must also handle "no data at all" — a dead
  LiDAR reads as "clear", which is the dangerous direction.

## Known unverified

- `TOP_LIDAR_YAW_DEG = 180.0` and the LiDAR mount offsets in
  `rover_config.py` are inherited from the original code and were mutually
  contradictory before being consolidated. **They need a tape measure and
  an RViz check.**
- `invert_yaw_rate = True` is derived from the ENU/NED convention
  difference, not from an observed hardware test.

## Git conventions

**No AI attribution in git history.** Do not add `Co-Authored-By: Claude`,
"Generated with Claude Code", or any similar trailer or footer to commit
messages or pull request bodies. The work is authored by the repository's
maintainers. `.claude/settings.json` enforces this via `attribution`, but
honour it regardless of whether that setting is loaded.

Commit messages themselves should stay as they are: explain *why* a change
was made and what would have gone wrong without it, not just what changed.

## Style

Match the existing code: `snake_case`, docstrings that explain *why* a
guard exists rather than what a line does, comments only where the reason
is non-obvious. Nodes declare parameters with defaults from
`rover_config.py`. No new third-party dependencies without a strong reason
— the last one (`pymap3d`) broke `rosdep install` on fresh machines.
