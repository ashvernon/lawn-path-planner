#!/usr/bin/env python3
"""
Lawn Path Planner MVP v3.1

Adds:
1) Draw planned stripes as visible lanes (thick sweep bands).
2) Show estimated mowing time: distance / mower_speed.

Controls (new):
- Mower speed:
    1 / 2 : decrease / increase speed (m/s) by 0.1
- Lanes display:
    L : toggle lanes on/off

Existing controls:
- Draw:
    Click (release) adds point | hold left+drag draws | P adds point at cursor
    BACKSPACE undo | ENTER plan | R reset | ESC quit
- Params:
    [ ] blade width | , . resolution | A angle step
- Plan:
    SPACE pause | +/- animation speed | R redraw
"""

from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import pygame

Point = Tuple[float, float]
Cell = Tuple[int, int]

WIN_W, WIN_H = 1280, 820
FPS = 60

COL_BG = (18, 18, 22)
COL_TEXT = (235, 235, 245)
COL_DIM = (170, 170, 190)
COL_WARN = (255, 220, 90)

COL_POLY = (240, 240, 240)
COL_POLY_EDGE = (220, 220, 230)
COL_CURSOR = (255, 140, 140)

COL_GRASS = (60, 140, 80)
COL_MOWN = (125, 210, 135)
COL_BLOCK = (70, 70, 78)
COL_PATH = (255, 220, 90)
COL_MOWER = (245, 245, 245)

# lane color (bright, visible)
COL_LANE = (120, 190, 255)

DIRS4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]

TURN_WEIGHT = 2.5
DEFAULT_CELLS_PER_M = 8
DEFAULT_BLADE_W_M = 0.5
ANGLE_STEPS = [5, 10, 15]
MAX_GRID_CELLS = 260_000

DRAG_ADD_ENABLED = True
DRAG_MIN_DIST_PX = 12


# ----------------------------
# Geometry / grid utilities
# ----------------------------
def poly_bounds(poly: List[Point]) -> Tuple[float, float, float, float]:
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


def bfs_path(grid: np.ndarray, start: Cell, goal: Cell) -> Optional[List[Cell]]:
    if start == goal:
        return []
    H, W = grid.shape
    q = deque([start])
    prev = {start: None}
    while q:
        x, y = q.popleft()
        for dx, dy in DIRS4:
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and grid[ny, nx] == 1:
                if (nx, ny) not in prev:
                    prev[(nx, ny)] = (x, y)
                    if (nx, ny) == goal:
                        q.clear()
                        break
                    q.append((nx, ny))
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


def rasterize_polygon(poly_m: List[Point], cells_per_meter: float, blade_w_m: float) -> Tuple[np.ndarray, float, int]:
    cell_size_m = 1.0 / cells_per_meter
    minx, miny, maxx, maxy = poly_bounds(poly_m)
    pad = max(blade_w_m, cell_size_m) * 1.5
    minx -= pad
    miny -= pad
    maxx += pad
    maxy += pad

    W = int(math.ceil((maxx - minx) / cell_size_m))
    H = int(math.ceil((maxy - miny) / cell_size_m))
    grid = np.zeros((H, W), dtype=np.uint8)

    # MVP fill (slow-ish, but capped)
    for gy in range(H):
        cy = miny + (gy + 0.5) * cell_size_m
        for gx in range(W):
            cx = minx + (gx + 0.5) * cell_size_m
            if point_in_poly(cx, cy, poly_m):
                grid[gy, gx] = 1

    sweep_step_cells = max(1, int(round(blade_w_m / cell_size_m)))
    return grid, cell_size_m, sweep_step_cells


# ----------------------------
# Planning
# ----------------------------
def boustrophedon_targets(grid: np.ndarray, sweep_step_cells: int) -> List[Cell]:
    """Cell visitation ordering (zig-zag bands)."""
    H, W = grid.shape
    targets: List[Cell] = []
    band = 0
    y = 0
    while y < H:
        rows = range(y, min(H, y + sweep_step_cells))
        left_to_right = (band % 2 == 0)
        for ry in rows:
            xs = range(W) if left_to_right else range(W - 1, -1, -1)
            for x in xs:
                if grid[ry, x] == 1:
                    targets.append((x, ry))
        band += 1
        y += sweep_step_cells
    return targets


def targets_to_path(grid: np.ndarray, start: Cell, targets: List[Cell]) -> List[Cell]:
    """Connect targets with BFS shortest paths."""
    path: List[Cell] = []
    cur = start
    for t in targets:
        if t == cur:
            continue
        p = bfs_path(grid, cur, t)
        if p is None:
            continue
        path.extend(p)
        cur = t
    return path


def score_path(path: List[Cell], turn_weight: float) -> Tuple[float, int, int]:
    steps = len(path)
    turns = 0
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
        prev_dir = ndir
        cur = nxt
    return steps + turn_weight * turns, steps, turns


def build_lane_centerlines(grid: np.ndarray, sweep_step_cells: int) -> List[Tuple[Cell, Cell]]:
    """
    Build visible "lane" segments for each band.
    For each band (sweep_step_cells tall), find contiguous x-runs of free cells
    in the middle row of that band and make line segments across each run.
    """
    H, W = grid.shape
    lanes: List[Tuple[Cell, Cell]] = []
    y = 0
    while y < H:
        y_mid = min(H - 1, y + sweep_step_cells // 2)
        row = grid[y_mid, :]
        x = 0
        while x < W:
            while x < W and row[x] == 0:
                x += 1
            if x >= W:
                break
            x0 = x
            while x < W and row[x] == 1:
                x += 1
            x1 = x - 1
            if x1 >= x0:
                lanes.append(((x0, y_mid), (x1, y_mid)))
        y += sweep_step_cells
    return lanes


def compute_best_plan(poly_m: List[Point], cells_per_meter: float, blade_w_m: float, angle_step_deg: int) -> dict:
    minx, miny, maxx, maxy = poly_bounds(poly_m)
    origin = ((minx + maxx) / 2.0, (miny + maxy) / 2.0)

    best = None
    for deg in range(0, 180, angle_step_deg):
        ang = math.radians(deg)
        rot = rotate_points(poly_m, ang, origin)

        grid, cell_size_m, sweep_step_cells = rasterize_polygon(rot, cells_per_meter, blade_w_m)
        if grid.sum() == 0:
            continue

        start = first_free_cell(grid)
        targets = boustrophedon_targets(grid, sweep_step_cells)
        path = targets_to_path(grid, start, targets)
        score, steps, turns = score_path(path, TURN_WEIGHT)

        # visible lanes
        lanes = build_lane_centerlines(grid, sweep_step_cells)

        cand = dict(
            deg=deg,
            grid=grid,
            path=path,
            start=start,
            cell_size_m=cell_size_m,
            sweep_step_cells=sweep_step_cells,
            score=score,
            steps=steps,
            turns=turns,
            lanes=lanes,
        )
        if best is None or cand["score"] < best["score"]:
            best = cand

    if best is None:
        raise RuntimeError("No plan found. Try lowering cells/m or simplifying polygon.")
    return best


# ----------------------------
# Render state
# ----------------------------
@dataclass
class PlanState:
    grid: np.ndarray
    path: List[Cell]
    start: Cell
    deg: int
    sweep_step_cells: int
    cell_size_m: float
    steps: int
    turns: int
    score: float
    lanes: List[Tuple[Cell, Cell]]

    path_i: int = 0
    visited: Optional[np.ndarray] = None

    def __post_init__(self):
        self.visited = np.zeros_like(self.grid, dtype=np.uint8)
        sx, sy = self.start
        if self.grid[sy, sx] == 1:
            self.visited[sy, sx] = 1


def draw_text(screen, font, x, y, s, col=COL_TEXT):
    screen.blit(font.render(s, True, col), (x, y))


def fit_grid_to_window(grid_w: int, grid_h: int) -> Tuple[int, int, int]:
    margin_left = 20
    margin_top = 180
    max_draw_w = WIN_W - margin_left * 2
    max_draw_h = WIN_H - margin_top - 30
    cell_px = max(3, min(max_draw_w // max(1, grid_w), max_draw_h // max(1, grid_h)))
    ox = margin_left + (max_draw_w - grid_w * cell_px) // 2
    oy = margin_top + (max_draw_h - grid_h * cell_px) // 2
    return cell_px, ox, oy


def poly_px_to_m(poly_px: List[Tuple[int, int]], scale_m_per_px: float, origin_px: Tuple[int, int]) -> List[Point]:
    ox, oy = origin_px
    return [((x - ox) * scale_m_per_px, (y - oy) * scale_m_per_px) for (x, y) in poly_px]


class PlannerJob:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = False
        self.error: Optional[str] = None
        self.result: Optional[dict] = None

    def start(self, poly_m: List[Point], cells_per_m: float, blade_w_m: float, angle_step_deg: int):
        with self.lock:
            if self.running:
                return
            self.running = True
            self.error = None
            self.result = None

        def worker():
            try:
                best = compute_best_plan(poly_m, cells_per_m, blade_w_m, angle_step_deg)
                with self.lock:
                    self.result = best
            except Exception as e:
                with self.lock:
                    self.error = str(e)
            finally:
                with self.lock:
                    self.running = False

        threading.Thread(target=worker, daemon=True).start()

    def poll(self):
        with self.lock:
            return self.running, self.error, self.result


# ----------------------------
# Main
# ----------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Lawn Path Planner MVP v3.1")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)

    poly_px: List[Tuple[int, int]] = []
    scale_m_per_px = 0.02
    origin_px = (100, 200)

    blade_w_m = DEFAULT_BLADE_W_M
    cells_per_m = DEFAULT_CELLS_PER_M
    angle_step_idx = 1  # 10°
    mower_speed_mps = 0.7  # typical push mower walking pace ~0.6–1.0 m/s

    show_lanes = True

    plan: Optional[PlanState] = None
    paused = False
    anim_speed = 2  # steps per frame (start slow so you can see it)
    mode = "draw"   # draw | planning | plan
    status_msg = ""

    job = PlannerJob()

    left_down = False
    last_added: Optional[Tuple[int, int]] = None

    def add_point(pt: Tuple[int, int], why: str):
        nonlocal status_msg, last_added
        poly_px.append(pt)
        last_added = pt
        status_msg = f"Added point {pt} ({why}). Total points: {len(poly_px)}"

    def maybe_cap_resolution(poly_m: List[Point], cells_per_m_in: float, blade_w_m_in: float) -> float:
        minx, miny, maxx, maxy = poly_bounds(poly_m)
        cell_size = 1.0 / cells_per_m_in
        pad = max(blade_w_m_in, cell_size) * 1.5
        w_m = (maxx - minx) + 2 * pad
        h_m = (maxy - miny) + 2 * pad
        W = int(math.ceil(w_m / cell_size))
        H = int(math.ceil(h_m / cell_size))
        if W * H <= MAX_GRID_CELLS:
            return cells_per_m_in
        c = cells_per_m_in
        while c > 1:
            cell_size = 1.0 / c
            pad = max(blade_w_m_in, cell_size) * 1.5
            W = int(math.ceil(w_m / cell_size))
            H = int(math.ceil(h_m / cell_size))
            if W * H <= MAX_GRID_CELLS:
                return c
            c -= 1
        return 1.0

    running = True
    while running:
        clock.tick(FPS)
        mx, my = pygame.mouse.get_pos()

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False

            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False

                # blade/res/angle controls
                if ev.key == pygame.K_LEFTBRACKET:
                    blade_w_m = max(0.05, blade_w_m - 0.05)
                elif ev.key == pygame.K_RIGHTBRACKET:
                    blade_w_m = min(3.0, blade_w_m + 0.05)
                elif ev.key == pygame.K_COMMA:
                    cells_per_m = max(1, int(cells_per_m) - 1)
                elif ev.key == pygame.K_PERIOD:
                    cells_per_m = min(30, int(cells_per_m) + 1)
                elif ev.key == pygame.K_a:
                    angle_step_idx = (angle_step_idx + 1) % len(ANGLE_STEPS)

                # mower speed controls (m/s)
                elif ev.key == pygame.K_1:
                    mower_speed_mps = max(0.1, mower_speed_mps - 0.1)
                elif ev.key == pygame.K_2:
                    mower_speed_mps = min(2.5, mower_speed_mps + 0.1)

                # lanes toggle
                elif ev.key == pygame.K_l:
                    show_lanes = not show_lanes

                if mode == "draw":
                    if ev.key == pygame.K_p:
                        add_point((mx, my), "P key")
                    elif ev.key == pygame.K_BACKSPACE and poly_px:
                        poly_px.pop()
                        status_msg = f"Removed last point. Total points: {len(poly_px)}"
                    elif ev.key == pygame.K_r:
                        poly_px.clear()
                        status_msg = "Reset polygon."
                        last_added = None
                    elif ev.key == pygame.K_RETURN:
                        if len(poly_px) >= 3:
                            poly_m = poly_px_to_m(poly_px, scale_m_per_px, origin_px)
                            capped = maybe_cap_resolution(poly_m, float(cells_per_m), blade_w_m)
                            if capped != float(cells_per_m):
                                status_msg = f"Auto-lowered resolution to {int(capped)} cells/m for speed."
                                cells_per_m = int(capped)

                            mode = "planning"
                            paused = False
                            plan = None
                            job.start(poly_m, float(cells_per_m), blade_w_m, ANGLE_STEPS[angle_step_idx])
                        else:
                            status_msg = "Need at least 3 points."
                elif mode == "plan":
                    if ev.key == pygame.K_SPACE:
                        paused = not paused
                    elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        anim_speed = max(1, anim_speed - 1)
                    elif ev.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                        anim_speed = min(400, anim_speed + 1)
                    elif ev.key == pygame.K_r:
                        mode = "draw"
                        poly_px.clear()
                        plan = None
                        paused = False
                        status_msg = "Redraw lawn."
                        last_added = None

            elif ev.type == pygame.MOUSEBUTTONDOWN and mode == "draw":
                if ev.button == 1:
                    left_down = True

            elif ev.type == pygame.MOUSEBUTTONUP and mode == "draw":
                if ev.button == 1:
                    left_down = False
                    add_point(ev.pos, "mouse up")

            elif ev.type == pygame.MOUSEMOTION and mode == "draw":
                if DRAG_ADD_ENABLED and left_down:
                    if last_added is None:
                        add_point(ev.pos, "drag start")
                    else:
                        dx = ev.pos[0] - last_added[0]
                        dy = ev.pos[1] - last_added[1]
                        if dx * dx + dy * dy >= DRAG_MIN_DIST_PX * DRAG_MIN_DIST_PX:
                            add_point(ev.pos, "drag")

        # poll planning thread
        if mode == "planning":
            running_flag, err, res = job.poll()
            if not running_flag:
                if err:
                    status_msg = f"Planning failed: {err}"
                    mode = "draw"
                elif res:
                    plan = PlanState(
                        grid=res["grid"],
                        path=res["path"],
                        start=res["start"],
                        deg=res["deg"],
                        sweep_step_cells=res["sweep_step_cells"],
                        cell_size_m=res["cell_size_m"],
                        steps=res["steps"],
                        turns=res["turns"],
                        score=res["score"],
                        lanes=res["lanes"],
                    )
                    mode = "plan"
                    status_msg = f"Plan ready. Angle={res['deg']}°"

        # animate
        if mode == "plan" and plan is not None and not paused:
            for _ in range(anim_speed):
                if plan.path_i >= len(plan.path):
                    paused = True  # stop when done so it doesn't look like "nothing happened"
                    break
                x, y = plan.path[plan.path_i]
                plan.visited[y, x] = 1
                plan.path_i += 1

        # render
        screen.fill(COL_BG)

        # header
        draw_text(screen, font, 20, 16, "Lawn Path Planner MVP v3.1")
        draw_text(
            screen, font, 20, 36,
            f"Blade: {blade_w_m:.2f} m | Resolution: {int(cells_per_m)} cells/m | Angle step: {ANGLE_STEPS[angle_step_idx]}° "
            f"( [ ] blade, , . res, A angle-step )",
            COL_DIM,
        )
        draw_text(
            screen, font, 20, 56,
            f"Mower speed: {mower_speed_mps:.1f} m/s  (1/2 adjust) | Lanes: {'ON' if show_lanes else 'OFF'} (L toggle)",
            COL_DIM,
        )

        if status_msg:
            draw_text(screen, font, 20, 78, status_msg, COL_WARN)

        # cursor marker
        if mode == "draw":
            pygame.draw.circle(screen, COL_CURSOR, (mx, my), 3)

        if mode == "draw":
            draw_text(screen, font, 20, 110,
                      "Draw: click (release) adds point | hold left+drag draws | P adds point | ENTER plan | R reset",
                      COL_DIM)
            pygame.draw.circle(screen, (255, 120, 120), origin_px, 4)
            draw_text(screen, font, origin_px[0] + 8, origin_px[1] - 8, "origin", (255, 120, 120))
            draw_text(screen, font, 20, 132, f"Scale: 1 px = {scale_m_per_px:.3f} m  (1000px ~ {1000*scale_m_per_px:.1f}m)", COL_DIM)

            if len(poly_px) >= 1:
                for p in poly_px:
                    pygame.draw.circle(screen, COL_POLY, p, 4)
            if len(poly_px) >= 2:
                pygame.draw.lines(screen, COL_POLY_EDGE, False, poly_px, 2)

        elif mode == "planning":
            draw_text(screen, font, 20, 110,
                      "Planning… Tip: press , to lower resolution or A to increase angle-step if it’s slow.",
                      COL_WARN)
            if len(poly_px) >= 2:
                pygame.draw.lines(screen, COL_POLY_EDGE, False, poly_px, 2)

        elif mode == "plan" and plan is not None:
            grid = plan.grid
            H, W = grid.shape
            cell_px, ox, oy = fit_grid_to_window(W, H)

            # draw cells
            for y in range(H):
                for x in range(W):
                    r = pygame.Rect(ox + x * cell_px, oy + y * cell_px, cell_px - 1, cell_px - 1)
                    if grid[y, x] == 0:
                        pygame.draw.rect(screen, COL_BLOCK, r)
                    else:
                        pygame.draw.rect(screen, COL_MOWN if plan.visited[y, x] else COL_GRASS, r)

            # draw lanes (thick lines)
            if show_lanes:
                # thickness ~ blade width in pixels
                thickness = max(2, int(plan.sweep_step_cells * cell_px * 0.9))
                for (a, b) in plan.lanes:
                    ax, ay = a
                    bx, by = b
                    p1 = (ox + ax * cell_px + cell_px // 2, oy + ay * cell_px + cell_px // 2)
                    p2 = (ox + bx * cell_px + cell_px // 2, oy + by * cell_px + cell_px // 2)
                    pygame.draw.line(screen, COL_LANE, p1, p2, thickness)

            # draw upcoming path points
            if plan.path_i < len(plan.path):
                for j in range(plan.path_i, min(len(plan.path), plan.path_i + 1600)):
                    x, y = plan.path[j]
                    px = ox + x * cell_px + cell_px // 2
                    py = oy + y * cell_px + cell_px // 2
                    if 0 <= px < WIN_W and 0 <= py < WIN_H:
                        screen.set_at((px, py), COL_PATH)

            # mower marker
            if plan.path_i == 0:
                mx2, my2 = plan.start
            else:
                mx2, my2 = plan.path[min(plan.path_i - 1, len(plan.path) - 1)] if plan.path else plan.start
            center = (ox + mx2 * cell_px + cell_px // 2, oy + my2 * cell_px + cell_px // 2)
            pygame.draw.circle(screen, COL_MOWER, center, max(3, cell_px // 3))

            free = int(grid.sum())
            covered = int(plan.visited.sum())
            cov = (covered / free) if free else 0.0

            # distance and time estimate
            travel_m = plan.steps * plan.cell_size_m
            est_time_s = travel_m / max(1e-6, mower_speed_mps)
            est_min = est_time_s / 60.0

            draw_text(screen, font, 20, 110,
                      f"Best angle: {plan.deg}° | blade≈{plan.sweep_step_cells} cells | paused={paused} | anim_speed={anim_speed} steps/frame",
                      COL_DIM)
            draw_text(screen, font, 20, 132, f"Coverage: {covered}/{free} ({cov*100:.1f}%)", COL_DIM)
            draw_text(screen, font, 20, 154,
                      f"Distance (approx): {travel_m:.1f} m | Est time @ {mower_speed_mps:.1f} m/s: {est_min:.1f} min",
                      COL_DIM)
            draw_text(screen, font, 20, 176,
                      f"Steps: {plan.steps} | Turns: {plan.turns} | Score: {plan.score:.1f}  (SPACE pause, +/- anim speed, R redraw)",
                      COL_DIM)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
