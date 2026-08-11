"""
rover_config.py
~~~~~~~~~~~~~~~
Single source of truth for the Raftaar rover's physical layout and its
topic / frame names.

Why this file exists
--------------------
The same physical fact used to be written down in two or three different
places (a static transform in one launch file, a sector angle in a node
parameter, a topic name in the Nav2 YAML).  When they drifted apart the
rover silently misbehaved - e.g. the mode switcher watched the top LiDAR
while the costmap watched a topic nobody published.

Everything that describes "how the rover is physically built" now lives
here, and the launch files import it.  Change a number here and every
consumer follows.

Coordinate conventions (ROS REP-103)
------------------------------------
  base_link : origin at the rover's centre, projected onto the ground.
              +x forward, +y left, +z up.  Yaw is right-hand about +z,
              i.e. positive yaw turns left / counter-clockwise.
  All distances are metres, all angles in this file are degrees.
"""

import math

# ---------------------------------------------------------------------------
# Frame names
# ---------------------------------------------------------------------------
MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
TOP_LIDAR_FRAME = 'laser_top'
LOW_LIDAR_FRAME = 'laser_low'

# ---------------------------------------------------------------------------
# Topic names
# ---------------------------------------------------------------------------
TOP_SCAN_TOPIC = '/scan_top'
LOW_SCAN_TOPIC = '/scan_low'
OBSTACLE_TOPIC = '/obstacle_detected'

# Nav2's controller publishes raw velocity here ...
CMD_VEL_TOPIC = '/cmd_vel'
# ... the velocity smoother ramps it and republishes here.  This is what
# actually reaches the flight controller.
CMD_VEL_SMOOTHED_TOPIC = '/cmd_vel_smoothed'

SETPOINT_TOPIC = '/mavros/setpoint_raw/local'

# ---------------------------------------------------------------------------
# Chassis dimensions
# ---------------------------------------------------------------------------
CHASSIS_LENGTH_M = 2.19
CHASSIS_WIDTH_M = 2.10

# Footprint polygon in base_link, counter-clockwise from front-right.
# Used by the Nav2 costmap for collision checking.
FOOTPRINT = [
    (CHASSIS_LENGTH_M / 2.0, CHASSIS_WIDTH_M / 2.0),    # front-left
    (CHASSIS_LENGTH_M / 2.0, -CHASSIS_WIDTH_M / 2.0),   # front-right
    (-CHASSIS_LENGTH_M / 2.0, -CHASSIS_WIDTH_M / 2.0),  # rear-right
    (-CHASSIS_LENGTH_M / 2.0, CHASSIS_WIDTH_M / 2.0),   # rear-left
]

# Radius of the circle that encloses the whole rover.
CIRCUMSCRIBED_RADIUS_M = math.hypot(CHASSIS_LENGTH_M / 2.0, CHASSIS_WIDTH_M / 2.0)

# How far the planner should stay away from obstacles.
SAFETY_MARGIN_M = 0.5
INFLATION_RADIUS_M = round(CIRCUMSCRIBED_RADIUS_M + SAFETY_MARGIN_M, 2)  # ~2.02

# ---------------------------------------------------------------------------
# LiDAR mounting
# ---------------------------------------------------------------------------
# !! VERIFY THESE WITH A TAPE MEASURE BEFORE DRIVING !!
#
# (x, y, z) is the sensor's position relative to base_link.
# yaw_deg is how far the sensor body is rotated about +z (left-positive).
#
# The yaw matters twice over, which is exactly why it lives here:
#   1. The static transform tells the costmap where a hit really is.
#   2. The mode switcher needs to know which bearing in the *sensor's own*
#      frame corresponds to "straight ahead" for the rover.
# Previously (1) said 0 deg and (2) said 180 deg.  They cannot both be
# right: with the old settings the costmap would have mirrored every
# obstacle the top LiDAR saw onto the wrong side of the rover.
TOP_LIDAR_XYZ = (0.73, 0.0, 1.15)
TOP_LIDAR_YAW_DEG = 180.0          # top unit is mounted facing backwards

LOW_LIDAR_XYZ = (1.08, 0.0, 0.58)
LOW_LIDAR_YAW_DEG = 0.0            # low unit is mounted facing forwards


def forward_bearing_deg(mount_yaw_deg: float) -> float:
    """Bearing, in a sensor's own frame, that points straight ahead for the rover.

    If the sensor body is rotated +yaw relative to base_link, then the
    rover's forward direction appears at -yaw inside the sensor's data.
    """
    return -mount_yaw_deg + 0.0    # + 0.0 turns -0.0 into 0.0 for tidy logs


# Obstacles above this height are ignored by each LiDAR's costmap layer, so
# neither one marks the sky or a low overhang it cannot actually hit.
TOP_MAX_OBSTACLE_HEIGHT_M = TOP_LIDAR_XYZ[2] + 0.35
LOW_MAX_OBSTACLE_HEIGHT_M = LOW_LIDAR_XYZ[2] + 0.42

# ---------------------------------------------------------------------------
# Detection tuning
# ---------------------------------------------------------------------------
OBSTACLE_THRESHOLD_TOP_M = 2.0
OBSTACLE_THRESHOLD_LOW_M = 2.0
DETECTION_HALF_ANGLE_DEG = 45.0    # half-width of the cone each LiDAR watches

# ---------------------------------------------------------------------------
# Motion limits (skid-steer: no sideways motion)
# ---------------------------------------------------------------------------
MAX_FORWARD_SPEED = 0.5       # m/s
MAX_REVERSE_SPEED = -0.2      # m/s, only used to back out of a corner
MAX_YAW_RATE = 0.5            # rad/s

# ---------------------------------------------------------------------------
# Serial devices
# ---------------------------------------------------------------------------
FCU_URL = (
    'serial:///dev/serial/by-id/'
    'usb-CubePilot_CubeOrange+_230042001851323031393637-if00:115200'
)
TOP_LIDAR_PORT = (
    '/dev/serial/by-id/'
    'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_'
    '393144b960165644bc33f96580944cbe-if00-port0'
)
TOP_LIDAR_BAUD = 256000
LOW_LIDAR_PORT = (
    '/dev/serial/by-id/'
    'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
)
LOW_LIDAR_BAUD = 115200
