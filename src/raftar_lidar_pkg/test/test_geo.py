"""Tests for geo.py — GPS to local ENU conversion.

Runs without ROS. `python -m pytest test/ -q` from the package root.
"""

import math

import pytest

from raftar_lidar_pkg.geo import geodetic_to_ecef, geodetic_to_enu

# Somewhere in Delhi; the exact spot does not matter, only that the
# reference and the target share it.
REF_LAT, REF_LON, REF_ALT = 28.6139, 77.2090, 216.0

WGS84_A = 6378137.0
WGS84_B = 6356752.314245


def test_ecef_equator_prime_meridian():
    """Sea level at 0N 0E sits on the +x axis, one equatorial radius out."""
    x, y, z = geodetic_to_ecef(0.0, 0.0, 0.0)
    assert x == pytest.approx(WGS84_A, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(0.0, abs=1e-9)


def test_ecef_north_pole_uses_polar_radius():
    """The pole is closer to the centre than the equator — the flattening."""
    _, _, z = geodetic_to_ecef(90.0, 0.0, 0.0)
    assert z == pytest.approx(WGS84_B, abs=1e-3)


def test_reference_point_maps_to_origin():
    e, n, u = geodetic_to_enu(REF_LAT, REF_LON, REF_ALT, REF_LAT, REF_LON, REF_ALT)
    assert (e, n, u) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_latitude_step_is_pure_north():
    """0.001 deg of latitude is ~111 m north and no east component."""
    e, n, _ = geodetic_to_enu(REF_LAT + 0.001, REF_LON, REF_ALT,
                              REF_LAT, REF_LON, REF_ALT)
    assert n == pytest.approx(110.8, abs=1.0)
    assert e == pytest.approx(0.0, abs=1e-6)


def test_longitude_step_shrinks_with_latitude():
    """A degree of longitude is narrower away from the equator, by cos(lat)."""
    expected = 111320.0 * math.cos(math.radians(REF_LAT)) * 0.001
    e, n, _ = geodetic_to_enu(REF_LAT, REF_LON + 0.001, REF_ALT,
                              REF_LAT, REF_LON, REF_ALT)
    assert e == pytest.approx(expected, abs=1.0)
    # Not exactly zero: a line of constant latitude is a parallel, which
    # curves, so travelling due east also carries you a hair north of the
    # tangent plane. Sub-millimetre over ~98 m, but real.
    assert n == pytest.approx(0.0, abs=0.01)


def test_altitude_change_is_pure_up():
    e, n, u = geodetic_to_enu(REF_LAT, REF_LON, REF_ALT + 10.0,
                              REF_LAT, REF_LON, REF_ALT)
    assert u == pytest.approx(10.0, abs=1e-6)
    assert e == pytest.approx(0.0, abs=1e-9)
    assert n == pytest.approx(0.0, abs=1e-9)


def test_signs_follow_enu_convention():
    """North and east are positive; south and west are negative."""
    e, n, _ = geodetic_to_enu(REF_LAT + 0.001, REF_LON + 0.001, REF_ALT,
                              REF_LAT, REF_LON, REF_ALT)
    assert e > 0 and n > 0

    e, n, _ = geodetic_to_enu(REF_LAT - 0.001, REF_LON - 0.001, REF_ALT,
                              REF_LAT, REF_LON, REF_ALT)
    assert e < 0 and n < 0


def test_offset_is_antisymmetric():
    """Swapping target and reference flips the vector."""
    lat, lon = REF_LAT + 0.004, REF_LON + 0.005
    fwd = geodetic_to_enu(lat, lon, REF_ALT, REF_LAT, REF_LON, REF_ALT)
    rev = geodetic_to_enu(REF_LAT, REF_LON, REF_ALT, lat, lon, REF_ALT)
    # Not exact — the tangent plane tilts between the two points — but a
    # few hundred metres apart it should agree to well under a metre.
    assert fwd[0] == pytest.approx(-rev[0], abs=0.5)
    assert fwd[1] == pytest.approx(-rev[1], abs=0.5)
