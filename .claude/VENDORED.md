# Vendored agent configuration

Part of `.claude/` is copied from a third-party template. This file records
what came from where, so the two can be told apart and re-synced later.

## Source

| | |
|---|---|
| Repository | https://github.com/harunkurtdev/ros2-claude-code-template |
| Commit | `5b3a43df2242c4198169911f637ba039d531cb8f` |
| Dated | 2026-06-03 |
| Copied on | 2026-08-12 |

**Licence status — unresolved.** The template's `README.md` states
"available under the Apache 2.0 License", but the repository contains **no
`LICENSE` file** and GitHub's licence detection reports none. The stated
intent is clear enough to proceed on, but it is not a licence grant. Worth
opening an issue upstream asking them to add the file. If the answer comes
back restrictive, delete the files listed under "Vendored" below — nothing
in this project depends on them to run.

## Vendored (do not edit — edits make re-syncing painful)

```
rules/    nav2_architecture.md  nav2_msgs_reference.md  nav2_parameters.md
          ros2_communication.md  ros2_general.md  ros2_nodes.md  testing.md

skills/   nav2_costmap  nav2_controllers  nav2_core_interfaces  nav2_servers
          ros2_transforms  ros2_lifecycle  ros2_launch_config  ros2_messaging
          ros2_node_creation  ros2_service_action  ros2_testing  ros2_bag

commands/ build.md  test.md  lint.md  ros2.md  nav2.md

agents/   ros2-style-reviewer.md
```

Why these: the Nav2 material is accurate where it matters most. It gives
the correct plugin identifier `nav2_mppi_controller::MPPIController` and
the correct critic names (`ObstaclesCritic`, `PathFollowCritic`, …) — which
is precisely the mistake that had been sitting in
`config/nav2_local_costmap.yaml` and would have stopped `controller_server`
from configuring at all. Having the reference in-repo makes that class of
bug much less likely to recur.

## Deliberately NOT vendored

| Excluded | Why |
|---|---|
| All `gz-*` (Gazebo/ECS) | No simulation in this project |
| All `vda5050*` | AGV fleet protocol, unused (one file alone is 22 KB) |
| All `behaviortree*` | This stack deliberately bypasses `bt_navigator` |
| All `ros2_control*` | Actuation goes through MAVROS/ArduPilot, not `ros2_control` |
| `clean_architecture.md`, ECS agents | Heavy patterns for a six-node package |
| `nav2_localization`, `nav2_behavior_tree` | No AMCL, no behaviour tree |
| Their `settings.json` | **See below — actively unsafe here** |

### Why their `settings.json` was not taken

It is written for Gazebo simulation and allows:

```json
"defaultMode": "acceptEdits",
"allow": ["Bash(ros2:*)", "Bash(python:*)", "Bash(pip install:*)"]
```

`Bash(ros2:*)` silently auto-approves `ros2 topic pub`, `ros2 service
call`, `ros2 run` and `ros2 launch`. In a simulator that is harmless. Here
those commands reach a live CubeOrange+ over a serial cable and move a
2.19 m vehicle. This project's own `settings.json` keeps every one of them
behind a confirmation prompt.

Four ideas from theirs *were* worth adopting and are in ours: the
`WebFetch` allows for `docs.ros.org` and `navigation.ros.org`, and the
denies on `Read(~/.ssh/**)`, `Read(./.env*)` and `rm -rf build/install/log`.

## Local caveats for the vendored commands

- `/lint` expects a `pre-commit` config and `origin/main`; neither is set
  up here. It falls back to the ament linters, which is fine.
- `raftar_lidar_pkg` is pure Python, so the C++ linters (`ament_uncrustify`,
  `ament_cpplint`, `clang-tidy`) in `/lint` do not apply.
- `/build` and `/test` assume `colcon`, which means WSL or the Jetson.
  On Windows use `/verify` instead — it runs the ROS-free checks.

## Ours, not vendored

`CLAUDE.md`, `commands/verify.md`, `settings.json`, and this file. The
template has no coverage of MAVROS, ArduPilot, MAVLink, ENU-vs-NED sign
conventions, or the flight-mode handoff — which is the genuinely hard and
safety-critical part of this project. That knowledge lives in `CLAUDE.md`.
