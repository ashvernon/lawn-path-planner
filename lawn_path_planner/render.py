from __future__ import annotations

import pygame

from .config import (
    COL_BLOCK,
    COL_DIM,
    COL_GRASS,
    COL_LANE,
    COL_MOWER,
    COL_MOWN,
    COL_PATH,
    COL_TEXT,
    WIN_H,
    WIN_W,
)
from .metrics import estimate_coverage
from .recommendation import Recommendation, compute_path_complexity_metrics, recommend_mower
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


def draw_plan(
    screen,
    font,
    plan: PlanState,
    show_lanes: bool,
    mower_speed_mps: float,
    paused: bool,
    anim_speed: int,
    turn_penalties_on: bool,
    turn_penalty_90_s: float,
    turn_penalty_180_s: float,
    shape_features: dict | None = None,
    mower_prefs: dict | None = None,
    show_recommendations: bool = True,
):
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
    path_metrics = compute_path_complexity_metrics(
        plan,
        mower_speed_mps,
        turn_penalty_90_s,
        turn_penalty_180_s,
        turn_penalties_on,
    )
    ineff_pct = path_metrics["inefficiency_pct"]
    decision_time_min = path_metrics["decision_time_min"]

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
        "Distance: {:.1f} m | Base time (no turns): {:.1f} min | Turn-adjusted: {:.1f} min".format(
            path_metrics["distance_m"],
            path_metrics["base_time_min"],
            path_metrics["turn_time_min"],
        ),
        COL_DIM,
    )
    draw_text(
        screen,
        font,
        20,
        176,
        "Overlap est: {:.1f}% | Decision time (turn+overlap): {:.1f} min | Time ({}) toggle: {:.1f} min".format(
            ineff_pct,
            decision_time_min,
            "with" if turn_penalties_on else "no",
            path_metrics["time_with_toggle_min"],
        ),
        COL_DIM,
    )
    draw_text(
        screen,
        font,
        20,
        198,
        "Steps: {} | Turn density: {:.1f}/100m | Turn penalties: {} (T toggle) | Turns: {} ({} U-turns) | Score: {:.1f}  (SPACE pause, +/- anim speed, R redraw)".format(
            plan.steps,
            path_metrics["turn_density_per_100m"],
            "ON" if turn_penalties_on else "OFF",
            plan.turns,
            plan.u_turns,
            plan.score,
        ),
        COL_DIM,
    )

    if show_recommendations and shape_features:
        combined_features = {**shape_features, **path_metrics}
        recs = recommend_mower(combined_features, mower_prefs)
        draw_recommendation_panel(
            screen,
            font,
            recs,
            path_metrics,
            shape_features,
            mower_prefs or {},
            decision_time_min,
            mower_speed_mps,
        )


def draw_recommendation_panel(
    screen,
    font,
    recs: list[Recommendation],
    path_metrics: dict,
    shape_features: dict,
    prefs: dict,
    decision_time_min: float,
    mower_speed_mps: float,
):
    panel_x = WIN_W - 420
    panel_y = 20
    panel_w = 400
    panel_h = 250

    pygame.draw.rect(screen, (36, 36, 42), pygame.Rect(panel_x - 8, panel_y - 8, panel_w + 16, panel_h + 16), border_radius=8)
    pygame.draw.rect(screen, (52, 52, 60), pygame.Rect(panel_x - 4, panel_y - 4, panel_w + 8, panel_h + 8), border_radius=8)

    draw_text(screen, font, panel_x, panel_y, "Mower recommendation (M toggle)")

    assumptions = f"Decision time uses turn penalties + overlap @ {mower_speed_mps:.1f} m/s"
    draw_text(screen, font, panel_x, panel_y + 18, assumptions, COL_DIM)

    draw_text(
        screen,
        font,
        panel_x,
        panel_y + 38,
        "Prefs F1 budget/F2 effort/F3 noise/F4 storage/F5 terrain:",
        COL_DIM,
    )
    prefs_line = "Budget: {} | Effort: {} | Noise: {} | Storage: {} | Terrain: {}".format(
        prefs.get("budget", "Medium"),
        prefs.get("effort", "Medium"),
        prefs.get("noise", "Medium"),
        prefs.get("storage", "Normal"),
        prefs.get("terrain", "Flat"),
    )
    draw_text(screen, font, panel_x, panel_y + 58, prefs_line, COL_DIM)

    reasons_y = panel_y + 82
    for idx, rec in enumerate(recs[:3]):
        header = "#{} {} (score {:.1f})".format(idx + 1, rec.category, rec.score)
        draw_text(screen, font, panel_x, reasons_y, header)
        reasons_y += 18
        for reason in rec.reasons[:3]:
            draw_text(screen, font, panel_x + 12, reasons_y, f"- {reason}", COL_DIM)
            reasons_y += 18
        if rec.warnings:
            draw_text(screen, font, panel_x + 12, reasons_y, f"! {rec.warnings[0]}", COL_WARN)
            reasons_y += 18

    summary_y = panel_y + panel_h - 52
    draw_text(
        screen,
        font,
        panel_x,
        summary_y,
        "Area: {:.0f} m² | Obstacles: {} ({:.1f}%) | Turn density: {:.1f}/100m".format(
            shape_features.get("area_m2", 0.0),
            shape_features.get("obstacle_count", 0),
            shape_features.get("obstacle_fraction_pct", 0.0),
            path_metrics.get("turn_density_per_100m", 0.0),
        ),
        COL_DIM,
    )
    draw_text(
        screen,
        font,
        panel_x,
        summary_y + 18,
        "Decision time: {:.1f} min | Base time: {:.1f} min | Overlap: {:.1f}%".format(
            decision_time_min,
            path_metrics.get("base_time_min", 0.0),
            path_metrics.get("inefficiency_pct", 0.0),
        ),
        COL_DIM,
    )
