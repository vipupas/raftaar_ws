"""
scan_utils.py
~~~~~~~~~~~~~
Shared helpers for reading a sector out of a sensor_msgs/LaserScan.

This logic used to exist in four separate copies across the nodes, each
slightly different - some handled sectors that wrap past the end of the
scan, some did not; none of them honoured the scan's own range_min /
range_max.  One copy is easier to trust.

What a LaserScan actually is
----------------------------
A flat array of distances, one per beam, taken while the sensor spins.
The message does not store an angle per beam; instead it stores where the
first beam pointed (``angle_min``) and how far the sensor turned between
beams (``angle_increment``).  So:

    angle_of(i) = angle_min + i * angle_increment

and going the other way, the index nearest a given angle is:

    index_of(a) = round((a - angle_min) / angle_increment)
"""

import math

# Angles are compared in a wrapped space one full turn wide.
_TWO_PI = 2.0 * math.pi


def _index_of(msg, angle_rad: float) -> int:
    """Array index whose beam points closest to ``angle_rad``, clamped in range."""
    idx = int(round((angle_rad - msg.angle_min) / msg.angle_increment))
    return max(0, min(len(msg.ranges) - 1, idx))


def _sector_segments(msg, lo_rad: float, hi_rad: float):
    """Index ranges covering the arc from ``lo_rad`` to ``hi_rad``.

    A sector such as "180 deg +/- 45 deg" straddles the seam where the scan
    array ends and restarts, so it cannot be expressed as one contiguous
    slice.  In that case two segments are returned instead of one.
    """
    n = len(msg.ranges)

    def wrap_into_scan(a: float) -> float:
        """Shift an angle by whole turns until it lands in the scan's own window."""
        while a < msg.angle_min:
            a += _TWO_PI
        while a >= msg.angle_min + _TWO_PI:
            a -= _TWO_PI
        return a

    lo = wrap_into_scan(lo_rad)
    hi = wrap_into_scan(hi_rad)

    if lo <= hi:
        return [(_index_of(msg, lo), _index_of(msg, hi))]

    # Wrapped: take the tail of the array and the head of it.
    return [(_index_of(msg, lo), n - 1), (0, _index_of(msg, hi))]


def nearest_in_sector(msg, center_deg: float, half_deg: float,
                      min_valid_m: float = 0.05) -> float:
    """Closest valid return inside a cone of the scan.

    Args:
        msg:         a sensor_msgs/LaserScan.
        center_deg:  middle of the cone, in the *sensor's own* frame.
        half_deg:    half-width of the cone.  90 gives a 180 deg view.
        min_valid_m: readings at or below this are treated as sensor noise.

    Returns:
        Distance in metres, or ``inf`` when the cone contains nothing valid.
        ``inf`` is deliberate: it means "clear", and every comparison of the
        form ``nearest <= threshold`` is then correctly False.
    """
    ranges = msg.ranges
    n = len(ranges)
    if n == 0 or msg.angle_increment == 0.0:
        return float('inf')

    # A reading only counts if it lies inside the sensor's own stated limits.
    # REP-117: anything outside [range_min, range_max] carries no information.
    lo_limit = max(min_valid_m, msg.range_min)
    hi_limit = msg.range_max

    half = abs(math.radians(half_deg))
    if half >= math.pi:
        segments = [(0, n - 1)]                       # full circle
    else:
        center = math.radians(center_deg)
        segments = _sector_segments(msg, center - half, center + half)

    nearest = float('inf')
    for start, end in segments:
        if start > end:                               # guard against odd drivers
            start, end = end, start
        for i in range(start, end + 1):
            r = ranges[i]
            if math.isnan(r) or math.isinf(r):
                continue
            if r < lo_limit or r > hi_limit:
                continue
            if r < nearest:
                nearest = r

    return nearest
