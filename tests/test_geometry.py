from math import pi

from lawn_path_planner.geometry import poly_bounds, rotate_points
from lawn_path_planner.types import Point


def test_poly_bounds_returns_min_max():
    poly: list[Point] = [(0.0, 1.0), (2.0, -1.0), (3.0, 4.0)]
    assert poly_bounds(poly) == (0.0, -1.0, 3.0, 4.0)


def test_rotate_points_half_turn():
    poly: list[Point] = [(1.0, 0.0), (-1.0, 0.0)]
    rotated = rotate_points(poly, pi, (0.0, 0.0))
    assert rotated[0] == (-1.0, 0.0)
    assert rotated[1] == (1.0, 0.0)
