from __future__ import annotations

import math
from typing import List, Optional

from .config import ANGLE_STEPS, TURN_WEIGHT
from .geometry import poly_bounds, rotate_points
from .grid import bfs_path, first_free_cell, nearest_free_cell, rasterize_polygon
from .types import Cell, PlanResult, Point


def boustrophedon_targets(grid, sweep_step_cells: int) -> List[Cell]:
    height, width = grid.shape
    targets: List[Cell] = []
    band = 0
    y = 0
    while y < height:
        rows = range(y, min(height, y + sweep_step_cells))
        left_to_right = band % 2 == 0
        for ry in rows:
            xs = range(width) if left_to_right else range(width - 1, -1, -1)
            for x in xs:
                if grid[ry, x] == 1:
                    targets.append((x, ry))
        band += 1
        y += sweep_step_cells
    return targets


def targets_to_path(grid, start: Cell, targets: List[Cell]) -> List[Cell]:
    path: List[Cell] = []
    cur = start
    for target in targets:
        if target == cur:
            continue
        segment = bfs_path(grid, cur, target)
        if segment is None:
            continue
        path.extend(segment)
        cur = target
    return path


def score_path(path: List[Cell], turn_weight: float) -> tuple[float, int, int, int]:
    steps = len(path)
    turns = 0
    u_turns = 0
    prev_dir = None
    cur = None
    for nxt in path:
        if cur is None:
            cur = nxt
            continue
        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]
        ndir = (dx, dy)
        if prev_dir is not None and ndir != prev_dir:
            turns += 1
            if ndir == (-prev_dir[0], -prev_dir[1]):
                u_turns += 1
        prev_dir = ndir
        cur = nxt
    return steps + turn_weight * turns, steps, turns, u_turns


def build_lane_centerlines(grid, sweep_step_cells: int) -> List[tuple[Cell, Cell]]:
    height, width = grid.shape
    lanes: List[tuple[Cell, Cell]] = []
    y = 0
    while y < height:
        y_mid = min(height - 1, y + sweep_step_cells // 2)
        row = grid[y_mid, :]
        x = 0
        while x < width:
            while x < width and row[x] == 0:
                x += 1
            if x >= width:
                break
            x0 = x
            while x < width and row[x] == 1:
                x += 1
            x1 = x - 1
            if x1 >= x0:
                lanes.append(((x0, y_mid), (x1, y_mid)))
        y += sweep_step_cells
    return lanes


def compute_best_plan(
    poly_m: List[Point],
    cells_per_meter: float,
    blade_w_m: float,
    angle_step_deg: int,
    *,
    start_point: Optional[Point] = None,
    obstacles: Optional[List[List[Point]]] = None,
) -> PlanResult:
    minx, miny, maxx, maxy = poly_bounds(poly_m)
    origin = ((minx + maxx) / 2.0, (miny + maxy) / 2.0)

    best: PlanResult | None = None
    for deg in range(0, 180, angle_step_deg):
        angle = math.radians(deg)
        rot = rotate_points(poly_m, angle, origin)
        rot_obstacles = [rotate_points(ob, angle, origin) for ob in obstacles or []]
        rot_start = rotate_points([start_point], angle, origin)[0] if start_point else None

        grid, cell_size_m, sweep_step_cells, minx, miny = rasterize_polygon(
            rot, cells_per_meter, blade_w_m, rot_obstacles
        )
        if grid.sum() == 0:
            continue

        if rot_start is not None:
            sx = int((rot_start[0] - minx) / cell_size_m)
            sy = int((rot_start[1] - miny) / cell_size_m)
            start_candidate = (sx, sy)
            start = nearest_free_cell(grid, start_candidate) or first_free_cell(grid)
        else:
            start = first_free_cell(grid)
        targets = boustrophedon_targets(grid, sweep_step_cells)
        path = targets_to_path(grid, start, targets)
        score, steps, turns, u_turns = score_path(path, TURN_WEIGHT)
        lanes = build_lane_centerlines(grid, sweep_step_cells)

        candidate = PlanResult(
            deg=deg,
            grid=grid,
            path=path,
            start=start,
            cell_size_m=cell_size_m,
            sweep_step_cells=sweep_step_cells,
            score=score,
            steps=steps,
            turns=turns,
            u_turns=u_turns,
            lanes=lanes,
        )
        if best is None or candidate.score < best.score:
            best = candidate

    if best is None:
        raise RuntimeError("No plan found. Try lowering cells/m or simplifying polygon.")
    return best


def maybe_cap_resolution(poly_m: List[Point], cells_per_m_in: float, blade_w_m_in: float, max_cells: int) -> float:
    minx, miny, maxx, maxy = poly_bounds(poly_m)
    cell_size = 1.0 / cells_per_m_in
    pad = max(blade_w_m_in, cell_size) * 1.5
    width_m = (maxx - minx) + 2 * pad
    height_m = (maxy - miny) + 2 * pad
    width = int(math.ceil(width_m / cell_size))
    height = int(math.ceil(height_m / cell_size))
    if width * height <= max_cells:
        return cells_per_m_in
    current = cells_per_m_in
    while current > 1:
        cell_size = 1.0 / current
        pad = max(blade_w_m_in, cell_size) * 1.5
        width = int(math.ceil(width_m / cell_size))
        height = int(math.ceil(height_m / cell_size))
        if width * height <= max_cells:
            return current
        current -= 1
    return 1.0


def next_angle_step_idx(angle_step_idx: int) -> int:
    return (angle_step_idx + 1) % len(ANGLE_STEPS)
