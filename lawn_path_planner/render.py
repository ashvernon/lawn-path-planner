from __future__ import annotations

import pygame

from .config import COL_BLOCK, COL_DIM, COL_GRASS, COL_LANE, COL_MOWER, COL_MOWN, COL_PATH, COL_TEXT, WIN_H, WIN_W
from .metrics import estimate_coverage, estimate_distance_m, estimate_time_min
from .types import PlanState


def draw_text(screen, font, x, y, text, color=COL_TEXT):
    screen.blit(font.render(text, True, color), (x, y))


def fit_grid_to_window(grid_w: int, grid_h: int) -> tuple[int, int, int]:
    margin_left = 20
    margin_top = 180
    max_draw_w = WIN_W - margin_left * 2
    max_draw_h = WIN_H - margin_top - 30
    cell_px = max(3, min(max_draw_w // max(1, grid_w), max_draw_h // max(1, grid_h)))
    ox = margin_left + (max_draw_w - grid_w * cell_px) // 2
    oy = margin_top + (max_draw_h - grid_h * cell_px) // 2
    return cell_px, ox, oy


def draw_plan(screen, font, plan: PlanState, show_lanes: bool, mower_speed_mps: float, paused: bool, anim_speed: int):
    grid = plan.grid
    height, width = grid.shape
    cell_px, ox, oy = fit_grid_to_window(width, height)

    for y in range(height):
        for x in range(width):
            rect = pygame.Rect(ox + x * cell_px, oy + y * cell_px, cell_px - 1, cell_px - 1)
            if grid[y, x] == 0:
                pygame.draw.rect(screen, COL_BLOCK, rect)
            else:
                pygame.draw.rect(screen, COL_MOWN if plan.visited[y, x] else COL_GRASS, rect)

    if show_lanes:
        thickness = max(2, int(plan.sweep_step_cells * cell_px * 0.9))
        for a, b in plan.lanes:
            ax, ay = a
            bx, by = b
            p1 = (ox + ax * cell_px + cell_px // 2, oy + ay * cell_px + cell_px // 2)
            p2 = (ox + bx * cell_px + cell_px // 2, oy + by * cell_px + cell_px // 2)
            pygame.draw.line(screen, COL_LANE, p1, p2, thickness)

    if plan.path_i < len(plan.path):
        for step in range(plan.path_i, min(len(plan.path), plan.path_i + 1600)):
            x, y = plan.path[step]
            px = ox + x * cell_px + cell_px // 2
            py = oy + y * cell_px + cell_px // 2
            if 0 <= px < WIN_W and 0 <= py < WIN_H:
                screen.set_at((px, py), COL_PATH)

    if plan.path_i == 0:
        mx, my = plan.start
    else:
        mx, my = plan.path[min(plan.path_i - 1, len(plan.path) - 1)] if plan.path else plan.start
    center = (ox + mx * cell_px + cell_px // 2, oy + my * cell_px + cell_px // 2)
    pygame.draw.circle(screen, COL_MOWER, center, max(3, cell_px // 3))

    free, covered, coverage = estimate_coverage(plan)
    travel_m = estimate_distance_m(plan)
    est_time_min = estimate_time_min(travel_m, mower_speed_mps)

    draw_text(
        screen,
        font,
        20,
        110,
        f"Best angle: {plan.deg}° | blade≈{plan.sweep_step_cells} cells | paused={paused} | anim_speed={anim_speed} steps/frame",
        COL_DIM,
    )
    draw_text(screen, font, 20, 132, f"Coverage: {covered}/{free} ({coverage * 100:.1f}%)", COL_DIM)
    draw_text(
        screen,
        font,
        20,
        154,
        f"Distance (approx): {travel_m:.1f} m | Est time @ {mower_speed_mps:.1f} m/s: {est_time_min:.1f} min",
        COL_DIM,
    )
    draw_text(
        screen,
        font,
        20,
        176,
        f"Steps: {plan.steps} | Turns: {plan.turns} | Score: {plan.score:.1f}  (SPACE pause, +/- anim speed, R redraw)",
        COL_DIM,
    )
