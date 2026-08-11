"""Tests for scan_utils.py — reading a sector out of a LaserScan.

Runs without ROS: LaserScan is duck-typed by the fake below, since
nearest_in_sector only ever touches plain attributes.

The wrap-around cases matter most. The top LiDAR is mounted rotated, so the
cone it must watch straddles the seam where the scan array ends and
restarts. Earlier versions of this code got that wrong in two different
ways.
"""

import math

import pytest

from raftar_lidar_pkg import rover_config as cfg
from raftar_lidar_pkg.scan_utils import nearest_in_sector

BEAMS = 360
BACKGROUND = 8.0        # "nothing nearby" distance filling the scan


class FakeScan:
    """Minimal stand-in for sensor_msgs/LaserScan."""

    def __init__(self, ranges, angle_min=-math.pi, angle_max=math.pi,
                 range_min=0.15, range_max=12.0):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = (angle_max - angle_min) / (len(ranges) - 1)
        self.range_min = range_min
        self.range_max = range_max


def scan_with(hits, background=BACKGROUND):
    """A full scan of `background`, with `{degrees: distance}` poked in."""
    ranges = [background] * BEAMS
    scan = FakeScan(ranges)
    for deg, dist in hits.items():
        idx = int(round((math.radians(deg) - scan.angle_min) / scan.angle_increment))
        ranges[idx % BEAMS] = dist
    return scan


# --------------------------------------------------------------------------
# Basic sector selection
# --------------------------------------------------------------------------

def test_sees_object_in_the_cone():
    assert nearest_in_sector(scan_with({0: 1.5}), 0.0, 45.0) == pytest.approx(1.5)


def test_ignores_object_outside_the_cone():
    assert nearest_in_sector(scan_with({180: 0.3}), 0.0, 45.0) == pytest.approx(BACKGROUND)


def test_returns_the_nearest_of_several():
    got = nearest_in_sector(scan_with({-20: 3.0, 0: 1.2, 20: 2.0}), 0.0, 45.0)
    assert got == pytest.approx(1.2)


@pytest.mark.parametrize('deg', [-44, -20, 0, 20, 44])
def test_inside_cone_edges(deg):
    assert nearest_in_sector(scan_with({deg: 2.0}), 0.0, 45.0) == pytest.approx(2.0)


@pytest.mark.parametrize('deg', [-90, -60, 60, 90, 135])
def test_outside_cone_edges(deg):
    assert nearest_in_sector(scan_with({deg: 2.0}), 0.0, 45.0) == pytest.approx(BACKGROUND)


# --------------------------------------------------------------------------
# Wrap-around — a cone centred on 180 deg straddles the end of the array
# --------------------------------------------------------------------------

@pytest.mark.parametrize('deg', [136, 170, 180, -170, -136])
def test_wrapped_cone_sees_everything_inside_it(deg):
    assert nearest_in_sector(scan_with({deg: 1.3}), 180.0, 45.0) == pytest.approx(1.3)


@pytest.mark.parametrize('deg', [0, 45, 90, 130, -130, -90])
def test_wrapped_cone_ignores_everything_outside_it(deg):
    assert nearest_in_sector(scan_with({deg: 0.3}), 180.0, 45.0) == pytest.approx(BACKGROUND)


def test_negative_and_positive_centre_are_equivalent():
    """-180 and +180 name the same direction."""
    scan = scan_with({175: 1.1})
    assert nearest_in_sector(scan, 180.0, 45.0) == nearest_in_sector(scan, -180.0, 45.0)


# --------------------------------------------------------------------------
# Invalid readings
# --------------------------------------------------------------------------

def test_rejects_readings_below_range_min():
    """Sub-range_min returns are sensor artefacts, not real obstacles."""
    assert nearest_in_sector(scan_with({0: 0.10}), 0.0, 45.0) == pytest.approx(BACKGROUND)


def test_rejects_readings_above_range_max():
    assert nearest_in_sector(scan_with({0: 99.0}), 0.0, 45.0) == pytest.approx(BACKGROUND)


def test_rejects_nan_and_inf():
    scan = scan_with({0: float('nan'), 1: float('inf')})
    assert nearest_in_sector(scan, 0.0, 45.0) == pytest.approx(BACKGROUND)


def test_clear_path_reads_infinite():
    """inf means clear, so `nearest <= threshold` is correctly False."""
    assert math.isinf(nearest_in_sector(FakeScan([float('inf')] * BEAMS), 0.0, 45.0))


def test_empty_scan_reads_infinite():
    class Empty:
        ranges = []
        angle_min = angle_max = angle_increment = 0.0
        range_min = range_max = 0.0

    assert math.isinf(nearest_in_sector(Empty(), 0.0, 45.0))


def test_full_circle_sector():
    assert nearest_in_sector(scan_with({95: 0.9}), 0.0, 180.0) == pytest.approx(0.9)


# --------------------------------------------------------------------------
# The configured values, end to end
# --------------------------------------------------------------------------
# The two LiDARs are bolted on at different angles, so "straight ahead" is
# a different bearing in each one's data. Whatever the mount angles are,
# an obstacle in front of the ROVER must be visible to both, and one behind
# it to neither.

TOP_CENTRE = cfg.forward_bearing_deg(cfg.TOP_LIDAR_YAW_DEG)
LOW_CENTRE = cfg.forward_bearing_deg(cfg.LOW_LIDAR_YAW_DEG)
HALF = cfg.DETECTION_HALF_ANGLE_DEG


def test_top_lidar_sees_obstacle_ahead_of_the_rover():
    ahead_in_sensor_frame = TOP_CENTRE
    got = nearest_in_sector(scan_with({ahead_in_sensor_frame: 1.5}), TOP_CENTRE, HALF)
    assert got == pytest.approx(1.5)


def test_low_lidar_sees_obstacle_ahead_of_the_rover():
    got = nearest_in_sector(scan_with({LOW_CENTRE: 1.5}), LOW_CENTRE, HALF)
    assert got == pytest.approx(1.5)


def test_neither_lidar_reacts_to_obstacle_behind_the_rover():
    top_behind = TOP_CENTRE + 180.0
    low_behind = LOW_CENTRE + 180.0
    assert nearest_in_sector(scan_with({top_behind: 0.3}), TOP_CENTRE, HALF) \
        == pytest.approx(BACKGROUND)
    assert nearest_in_sector(scan_with({low_behind: 0.3}), LOW_CENTRE, HALF) \
        == pytest.approx(BACKGROUND)


def test_thresholds_are_positive_and_within_lidar_range():
    assert 0.0 < cfg.OBSTACLE_THRESHOLD_TOP_M < 12.0
    assert 0.0 < cfg.OBSTACLE_THRESHOLD_LOW_M < 12.0


def test_inflation_radius_covers_the_chassis():
    """The planner must keep more than the rover's own half-diagonal clear."""
    assert cfg.INFLATION_RADIUS_M > cfg.CIRCUMSCRIBED_RADIUS_M
