from __future__ import annotations

import math
from collections import deque
from typing import List, Optional

import numpy as np

from .config import DIRS4
from .geometry import poly_bounds, point_in_poly
from .types import Cell, Point


def bfs_path(grid: np.ndarray, start: Cell, goal: Cell) -> Optional[List[Cell]]:
    if start == goal:
        return []
    height, width = grid.shape
    queue = deque([start])
    prev = {start: None}
    while queue:
        x, y = queue.popleft()
        for dx, dy in DIRS4:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and grid[ny, nx] == 1:
                if (nx, ny) not in prev:
                    prev[(nx, ny)] = (x, y)
                    if (nx, ny) == goal:
                        queue.clear()
                        break
                    queue.append((nx, ny))
    if goal not in prev:
        return None
    path: List[Cell] = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path


def first_free_cell(grid: np.ndarray) -> Cell:
    ys, xs = np.where(grid == 1)
    if len(xs) == 0:
        return (0, 0)
    return (int(xs[0]), int(ys[0]))


def rasterize_polygon(poly_m: List[Point], cells_per_meter: float, blade_w_m: float) -> tuple[np.ndarray, float, int]:
    cell_size_m = 1.0 / cells_per_meter
    minx, miny, maxx, maxy = poly_bounds(poly_m)
    pad = max(blade_w_m, cell_size_m) * 1.5
    minx -= pad
    miny -= pad
    maxx += pad
    maxy += pad

    width = int(math.ceil((maxx - minx) / cell_size_m))
    height = int(math.ceil((maxy - miny) / cell_size_m))
    grid = np.zeros((height, width), dtype=np.uint8)

    for gy in range(height):
        cy = miny + (gy + 0.5) * cell_size_m
        for gx in range(width):
            cx = minx + (gx + 0.5) * cell_size_m
            if point_in_poly(cx, cy, poly_m):
                grid[gy, gx] = 1

    sweep_step_cells = max(1, int(round(blade_w_m / cell_size_m)))
    return grid, cell_size_m, sweep_step_cells
