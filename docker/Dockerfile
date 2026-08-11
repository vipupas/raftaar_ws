# Development environment for raftaar_ws.
#
# Builds and runs the ROS 2 side of the stack with no hardware attached -
# no LiDARs, no flight controller. Enough to compile the package, run the
# tests, and bring up Nav2's controller_server to confirm the MPPI
# configuration actually loads.
#
# This is a *development* image. It is not intended for the Jetson: the
# rover needs USB device passthrough and an arm64 build, neither of which
# is set up here.

FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pytest \
      ros-humble-nav2-controller \
      ros-humble-nav2-mppi-controller \
      ros-humble-nav2-velocity-smoother \
      ros-humble-nav2-lifecycle-manager \
      ros-humble-nav2-costmap-2d \
      ros-humble-nav2-msgs \
      ros-humble-mavros-msgs \
      ros-humble-geographic-msgs \
      ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Keep DDS discovery inside the container so it cannot find - or be found
# by - a real robot on the network.
ENV ROS_LOCALHOST_ONLY=1
ENV RCUTILS_COLORIZED_OUTPUT=0

# Source ROS and the workspace overlay (if built) for every shell.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ws
CMD ["sleep", "infinity"]
