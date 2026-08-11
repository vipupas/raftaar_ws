# Development container

ROS 2 Humble + Nav2 in a box, so anyone on the team can build and test this
package **without installing ROS and without any hardware**.

Nothing is installed on your machine — it all lives inside the image, and
`docker rmi raftaar-dev` removes every trace.

## Start

```bash
cd docker
docker compose up -d            # first run builds the image (~5 min, ~1.4 GB)
docker compose exec dev bash    # shell inside, workspace at /ws
```

## Build and test

Inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select raftar_lidar_pkg
```

The pure-Python tests also run without ROS, on any machine:

```bash
cd src/raftar_lidar_pkg && python3 -m pytest test/ -q
```

## Check that Nav2 actually loads

This is the check worth knowing, because it is the one that used to fail.
The controller plugin and its critics are resolved by name at runtime, so a
typo does not show up until the node is started.

```bash
# inside the container, after colcon build
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom &
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link &

ros2 run nav2_controller controller_server --ros-args \
  --params-file install/raftar_lidar_pkg/share/raftar_lidar_pkg/config/nav2_local_costmap.yaml &

ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
ros2 lifecycle get /controller_server        # must print: active [3]
```

A healthy run logs:

```
Created controller : FollowPath of type nav2_mppi_controller::MPPIController
Critic loaded : mppi::critics::ConstraintCritic
Critic loaded : mppi::critics::ObstaclesCritic
... 8 critics total
```

If a name is wrong you get `Transitioning failed`, the node stays
`unconfigured`, `/follow_path` never appears, and `coordinator_node` waits
forever. That is what the configuration in the initial commit did.

## Scope and limits

**This image is for development only.** It deliberately does not support
running the real rover:

- **Architecture.** Built for x86_64. The Jetson is arm64 and would need
  its own build.
- **No hardware access.** No USB passthrough for the LiDARs or the flight
  controller.
- **`ROS_LOCALHOST_ONLY=1`.** DDS traffic cannot leave the container, so
  nodes in here cannot discover or command a real rover that happens to be
  on the same network. Do not remove this without a good reason.
- **No restart policy.** The container will not come back on its own. For
  a stack that steers a two-metre vehicle, automatic restarts should be a
  deliberate decision, not a default.
