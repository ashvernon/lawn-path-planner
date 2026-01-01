from __future__ import annotations

import math
from typing import List

from .types import Point


def poly_bounds(poly: List[Point]) -> tuple[float, float, float, float]:
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def point_in_poly(x: float, y: float, poly: List[Point]) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_int = x1 + (y - y1) * (x2 - x1) / (y2 - y1 + 1e-12)
            if x_int > x:
                inside = not inside
    return inside


def rotate_points(poly: List[Point], angle_rad: float, origin: Point) -> List[Point]:
    ox, oy = origin
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    out: List[Point] = []
    for x, y in poly:
        dx, dy = x - ox, y - oy
        rx = ox + c * dx - s * dy
        ry = oy + s * dx + c * dy
        out.append((rx, ry))
    return out
