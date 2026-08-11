# Raftaar — LiDAR obstacle avoidance for an ArduPilot rover

ROS 2 workspace that lets a large GPS-guided rover drive itself around
obstacles its flight controller cannot see.

The rover normally follows a GPS mission using ArduPilot's own **AUTO**
mode. ArduPilot knows nothing about obstacles. This stack watches two
LiDARs, and when something blocks the path it takes over — switching the
vehicle into **GUIDED** mode, steering around the obstruction with Nav2's
local planner, then handing control back so the mission continues.

```
        AUTO                    GUIDED                     AUTO
  ArduPilot drives   ──▶   ROS steers around   ──▶   ArduPilot resumes
   its GPS mission          the obstacle             its GPS mission
```

---

## Hardware

| Part | Detail |
|---|---|
| Companion computer | NVIDIA Jetson, runs everything in this repo |
| Flight controller | CubePilot CubeOrange+ running ArduPilot Rover |
| Link between them | USB serial, via MAVROS |
| Top LiDAR | RPLidar, 1.15 m above ground, 256000 baud → `/scan_top` |
| Low LiDAR | RPLidar, 0.58 m above ground, 115200 baud → `/scan_low` |
| Chassis | 2.19 m long × 2.10 m wide, skid-steer |

Two LiDARs at different heights catch what a single one would miss — a kerb
or tow bar low down, a branch or wing mirror higher up. Either one seeing an
obstacle is enough to trigger avoidance.

---

## How it works

```
  ┌────────────┐  ┌────────────┐
  │ /scan_top  │  │ /scan_low  │
  └─────┬──────┘  └──────┬─────┘
        └────────┬───────┘
                 ▼
    obstacle_mode_switcher_dual          ──▶  /mavros/set_mode   AUTO ⇄ GUIDED
                 │
                 ▼  /obstacle_detected (Bool)
         coordinator_node
                 │   builds a straight line from here to the
                 │   current mission waypoint, sends it as a
                 ▼   Nav2 FollowPath goal
         controller_server (MPPI)  ◀── local costmap ◀── both LiDARs
                 │  /cmd_vel
                 ▼
         velocity_smoother
                 │  /cmd_vel_smoothed
                 ▼
           mav_to_teleop            ──▶  /mavros/setpoint_raw/local
                                              (ArduPilot, in GUIDED)
```

The straight line handed to Nav2 goes *through* the obstacle on purpose.
It is only a hint — the local planner scores thousands of candidate
trajectories against the costmap each cycle and deviates from that line as
much as it needs to. That deviation is the avoidance manoeuvre.

**Nodes**

| Node | Job |
|---|---|
| `obstacle_mode_switcher_dual` | Watches both LiDARs. Owns the flight mode. |
| `coordinator_node` | Tracks the mission, sends/cancels Nav2 goals, runs the stuck watchdog. |
| `mav_to_teleop` | Converts ROS velocities into MAVLink setpoints. |
| `controller_server` | Nav2's MPPI local planner. |
| `velocity_smoother` | Bounds acceleration so a heavy rover does not lurch. |

**Safety behaviours**

- **Debounce (0.5 s)** — an obstacle must persist before it counts, so a
  single noisy beam cannot trigger a mode change.
- **Cooldown (2 s)** — minimum gap between mode commands.
- **MANUAL lock** — if the rover is in any mode other than AUTO or GUIDED,
  the switcher issues no commands at all. A human who has taken over stays
  in control.
- **Stuck watchdog (8 s)** — if the planner makes no headway while an
  obstacle is still present, the rover stops and holds position **in
  GUIDED**. It deliberately does *not* resume AUTO, which would drive it
  back into the obstacle it just failed to pass.
- **Command timeout (0.5 s)** — if Nav2 goes quiet, `mav_to_teleop`
  commands zero velocity rather than letting the last one stand.

---

## Build

Requires **ROS 2 Humble** on Ubuntu 22.04, plus `rplidar_ros`, `mavros`
and the Nav2 stack.

```bash
cd ~/raftaar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Everything, including MAVROS
ros2 launch raftar_lidar_pkg rover_advance_v2.launch.py

# Same, but run MAVROS yourself in another terminal
ros2 launch raftar_lidar_pkg rover_advance.launch.py

# Bench test: one LiDAR + RViz, no Nav2, no avoidance
ros2 launch raftar_lidar_pkg mode_switch_with_lidar.launch.py
```

---

## Commissioning checklist

Work through this in order on a new machine or after changing mounts.
**Do not skip to the field.**

**1. Sensors are alive**
```bash
ros2 topic hz /scan_top /scan_low
```

**2. TF tree is a single chain**
```bash
ros2 run tf2_tools view_frames    # expect map → odom → base_link → lasers
```

**3. LiDAR orientation is correct** — the one thing most likely to be wrong.
In RViz, set Fixed Frame to `base_link` and display both scans. Walk in
front of the rover. **Both point clouds must show you in front.** If the
top LiDAR shows you behind, flip `TOP_LIDAR_YAW_DEG` in
`src/raftar_lidar_pkg/raftar_lidar_pkg/rover_config.py` between `180.0`
and `0.0` and rebuild.

**4. Detection triggers**
```bash
ros2 topic echo /obstacle_detected
```
Walk into each LiDAR's cone in turn; it should flip `true` after ~0.5 s.

**5. Nav2 came up**
```bash
ros2 lifecycle get /controller_server    # must be "active"
```

**6. Steering direction — wheels off the ground.** Put the rover on stands,
arm it, enter GUIDED, and block a LiDAR. Confirm the wheels turn in the
direction that would steer *away* from the obstacle. If they turn the wrong
way, flip `invert_yaw_rate` on the `mav_to_teleop` node in
`launch/rover_advance_v2.launch.py`.

Only after all six pass should the rover drive under its own control, and
then first in a large open area with a human on the RC transmitter ready to
switch to MANUAL.

---

## Tuning

Almost everything physical lives in one file:
`src/raftar_lidar_pkg/raftar_lidar_pkg/rover_config.py` — mount positions
and angles, detection thresholds and cone widths, speed limits, topic and
frame names, serial ports. Change a value there and the launch files,
transforms and node defaults all follow.

Planner behaviour lives in `src/raftar_lidar_pkg/config/nav2_local_costmap.yaml`
— costmap size and inflation, MPPI sampling, and the critic weights that
decide how strongly the rover prefers avoiding obstacles over following its
line.

## Tests

The geometry and scan-parsing logic is deliberately free of ROS imports so
it can be tested without a robot or even a ROS install:

```bash
cd src/raftar_lidar_pkg
python -m pytest test/ -q
```

## Layout

```
raftaar_ws/
├── CLAUDE.md                    briefing for AI coding agents
└── src/raftar_lidar_pkg/
    ├── raftar_lidar_pkg/        nodes + pure-Python helpers
    ├── launch/                  launch files
    ├── config/                  Nav2 parameters
    ├── rviz/                    RViz layout
    └── test/                    ROS-free unit tests
```

## Licence

Apache-2.0
