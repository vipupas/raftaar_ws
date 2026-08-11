"""
geo.py
~~~~~~
GPS (latitude / longitude / altitude) to local ENU (East / North / Up)
conversion, in pure Python.

This replaces the `pymap3d` dependency.  pymap3d is a fine library, but it
is not packaged for rosdep, so `rosdep install` could never set up a fresh
Jetson - the node just crashed on import.  The maths below is short,
standard, and has no install step.

Method
------
1. Convert both the target and the reference point from geodetic
   (lat/lon/alt on the WGS-84 ellipsoid) to ECEF - a Cartesian frame with
   its origin at the centre of the Earth.
2. Subtract, giving a vector from reference to target in ECEF.
3. Rotate that vector into the local East/North/Up frame that sits
   tangent to the ellipsoid at the reference point.
"""

import math

# WGS-84 ellipsoid, the datum every GPS receiver reports in.
_A = 6378137.0                      # semi-major axis (equatorial radius), m
_F = 1.0 / 298.257223563            # flattening
_E2 = _F * (2.0 - _F)               # first eccentricity squared


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float):
    """Geodetic coordinates -> Earth-Centred, Earth-Fixed metres."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    # Radius of curvature in the prime vertical.
    n = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)

    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - _E2) + alt_m) * sin_lat
    return x, y, z


def geodetic_to_enu(lat_deg: float, lon_deg: float, alt_m: float,
                    ref_lat_deg: float, ref_lon_deg: float, ref_alt_m: float):
    """Geodetic coordinates -> metres East / North / Up of a reference point.

    Drop-in replacement for ``pymap3d.geodetic2enu``.
    """
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    x0, y0, z0 = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)

    dx = x - x0
    dy = y - y0
    dz = z - z0

    lat0 = math.radians(ref_lat_deg)
    lon0 = math.radians(ref_lon_deg)

    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    # Standard ECEF -> ENU rotation at (lat0, lon0).
    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz

    return east, north, up
