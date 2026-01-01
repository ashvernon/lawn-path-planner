from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np

import pygame

from .config import (
    COL_BLOCK,
    COL_DIM,
    COL_GRASS,
    COL_LANE,
    COL_MOWER,
    COL_MOWN,
    COL_PANEL_BG,
    COL_PANEL_BORDER,
    COL_PANEL_DARK,
    COL_PATH,
    COL_TEXT,
    COL_WARN,
    GRID_PADDING,
    HUD_GAP,
    HUD_MARGIN,
    HUD_PADDING,
    HUD_PANEL_WIDTH,
    HUD_SECTION_GAP,
    WIN_H,
    WIN_W,
)
from .metrics import estimate_coverage
from .recommendation import Recommendation, compute_path_complexity_metrics, recommend_mower
from .types import PlanState


def draw_text(screen, font, x, y, text, color=COL_TEXT):
    screen.blit(font.render(text, True, color), (x, y))


def layout_rects() -> tuple[pygame.Rect, pygame.Rect]:
    hud_rect = pygame.Rect(
        HUD_MARGIN,
        HUD_MARGIN,
        HUD_PANEL_WIDTH,
        WIN_H - HUD_MARGIN * 2,
    )
    grid_rect = pygame.Rect(
        hud_rect.right + HUD_GAP,
        HUD_MARGIN,
        WIN_W - hud_rect.width - HUD_GAP - HUD_MARGIN,
        WIN_H - HUD_MARGIN * 2,
    )
    return hud_rect, grid_rect


def draw_panel_background(screen: pygame.Surface, rect: pygame.Rect):
    pygame.draw.rect(screen, COL_PANEL_DARK, rect.inflate(8, 8), border_radius=10)
    pygame.draw.rect(screen, COL_PANEL_BORDER, rect.inflate(4, 4), border_radius=10)
    pygame.draw.rect(screen, COL_PANEL_BG, rect, border_radius=10)


def wrap_lines(font: pygame.font.Font, text: str, max_width: int) -> list[str]:
    wrapped: list[str] = []
    for paragraph in text.splitlines():
        if not paragraph:
            wrapped.append("")
            continue
        words = paragraph.split(" ")
        line = ""
        for word in words:
            candidate = f"{line} {word}".strip()
            if font.size(candidate)[0] <= max_width:
                line = candidate
            else:
                if line:
                    wrapped.append(line)
                line = word
        if line:
            wrapped.append(line)
    return wrapped


def draw_wrapped(
    screen: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    color: Tuple[int, int, int],
    x: int,
    y: int,
    max_width: int,
    line_gap: int = 2,
) -> int:
    for line in wrap_lines(font, text, max_width):
        draw_text(screen, font, x, y, line, color)
        y += font.get_linesize() + line_gap
    return y


def draw_section(
    screen: pygame.Surface,
    fonts: dict[str, pygame.font.Font],
    title: str,
    lines: Iterable[tuple[str, Tuple[int, int, int], str]],
    origin: tuple[int, int],
    max_width: int,
) -> int:
    x, y = origin
    y = draw_wrapped(screen, fonts["title"], title, COL_TEXT, x, y, max_width, line_gap=4)
    for text, color, font_key in lines:
        y = draw_wrapped(screen, fonts[font_key], text, color, x, y, max_width)
    return y + HUD_SECTION_GAP


def fit_grid_to_rect(grid_w: int, grid_h: int, grid_rect: pygame.Rect) -> tuple[int, int, int]:
    max_draw_w = max(1, grid_rect.w - GRID_PADDING * 2)
    max_draw_h = max(1, grid_rect.h - GRID_PADDING * 2)
    cell_px = max(3, min(max_draw_w // max(1, grid_w), max_draw_h // max(1, grid_h)))
    ox = grid_rect.x + GRID_PADDING + (max_draw_w - grid_w * cell_px) // 2
    oy = grid_rect.y + GRID_PADDING + (max_draw_h - grid_h * cell_px) // 2
    return cell_px, ox, oy


def draw_heatmap(
    screen: pygame.Surface,
    grid: np.ndarray,
    visit_counts: np.ndarray,
    cell_px: int,
    origin: tuple[int, int],
    mode: str,
):
    vmax = int(visit_counts.max())
    if vmax <= 0:
        return

    height, width = visit_counts.shape
    overlay = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    ox, oy = origin

    for y in range(height):
        for x in range(width):
            if grid[y, x] == 0:
                continue
            visits = int(visit_counts[y, x])
            if visits <= 0:
                continue
            if mode == "overlap" and visits < 2:
                continue

            if mode == "log":
                norm = math.log1p(visits) / math.log1p(vmax)
            elif mode == "overlap":
                denom = max(1, vmax - 1)
                norm = (visits - 1) / denom
            else:
                norm = visits / vmax

            alpha = int(40 + norm * 180)
            color = (255, 170, 60, max(0, min(255, alpha)))
            rect = pygame.Rect(ox + x * cell_px, oy + y * cell_px, cell_px - 1, cell_px - 1)
            pygame.draw.rect(overlay, color, rect)

    screen.blit(overlay, (0, 0))


def draw_plan(
    screen: pygame.Surface,
    font: pygame.font.Font,
    plan: PlanState,
    show_lanes: bool,
    show_heatmap: bool,
    heatmap_mode: str,
    mower_speed_mps: float,
    paused: bool,
    anim_speed: int,
    turn_penalties_on: bool,
    turn_penalty_90_s: float,
    turn_penalty_180_s: float,
    shape_features: dict | None,
    mower_prefs: dict | None,
    show_recommendations: bool,
    grid_rect: pygame.Rect,
) -> dict:
    grid = plan.grid
    height, width = grid.shape
    cell_px, ox, oy = fit_grid_to_rect(width, height, grid_rect)

    pygame.draw.rect(screen, COL_PANEL_DARK, grid_rect, border_radius=8)

    for y in range(height):
        for x in range(width):
            rect = pygame.Rect(ox + x * cell_px, oy + y * cell_px, cell_px - 1, cell_px - 1)
            if grid[y, x] == 0:
                pygame.draw.rect(screen, COL_BLOCK, rect)
            else:
                pygame.draw.rect(screen, COL_MOWN if plan.visited[y, x] else COL_GRASS, rect)

    if show_heatmap:
        draw_heatmap(screen, grid, plan.visit_counts, cell_px, (ox, oy), heatmap_mode)

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
            if grid_rect.collidepoint(px, py):
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

    recommendations: list[Recommendation] = []
    if show_recommendations and shape_features:
        combined_features = {**shape_features, **path_metrics}
        recommendations = recommend_mower(combined_features, mower_prefs or {})

    return {
        "free": free,
        "covered": covered,
        "coverage": coverage,
        "path_metrics": path_metrics,
        "deg": plan.deg,
        "sweep_step_cells": plan.sweep_step_cells,
        "steps": plan.steps,
        "turns": plan.turns,
        "u_turns": plan.u_turns,
        "score": plan.score,
        "recommendations": recommendations,
    }


def summarize_recommendations(recs: list[Recommendation]) -> list[str]:
    lines: list[str] = []
    for idx, rec in enumerate(recs[:2]):
        header = f"#{idx + 1} {rec.category} (score {rec.score:.1f})"
        lines.append(header)
        for reason in rec.reasons[:2]:
            lines.append(f"- {reason}")
        if rec.warnings:
            lines.append(f"! {rec.warnings[0]}")
    return lines


def draw_hud_panel(screen: pygame.Surface, fonts: dict[str, pygame.font.Font], hud_rect: pygame.Rect, info: dict):
    draw_panel_background(screen, hud_rect)
    x = hud_rect.x + HUD_PADDING
    y = hud_rect.y + HUD_PADDING
    max_width = hud_rect.w - HUD_PADDING * 2

    title = "Lawn Path Planner MVP v3.1"
    y = draw_wrapped(screen, fonts["title"], title, COL_TEXT, x, y, max_width, line_gap=4)

    status_lines: list[tuple[str, tuple[int, int, int], str]] = []
    mode_line = f"Mode: {info['mode'].upper()}"
    if info.get("paused"):
        mode_line += " (paused)"
    if info.get("planning"):
        mode_line += " (planning…)"
    status_lines.append((mode_line, COL_TEXT, "body"))
    status_lines.append(
        (
            "Draw: {draw_mode} | Obstacles: {obstacles} | Start: {start}".format(
                draw_mode=info.get("draw_mode", "boundary").upper(),
                obstacles=info.get("obstacle_count", 0),
                start="SET" if info.get("start_set") else "AUTO",
            ),
            COL_DIM,
            "body",
        )
    )
    if info.get("status_msg"):
        status_lines.append((info["status_msg"], COL_WARN, "body"))

    y = draw_section(screen, fonts, "Status", status_lines, (x, y), max_width)

    param_lines = [
        (
            "Blade {blade:.2f}m | Resolution {res} cells/m | Angle {angle}°".format(
                blade=info.get("blade_w_m", 0.0),
                res=int(info.get("cells_per_m", 0)),
                angle=info.get("angle_step", 0),
            ),
            COL_DIM,
            "mono",
        ),
        (
            "Speed {speed:.1f} m/s | Lanes {lanes} (L toggle)".format(
                speed=info.get("mower_speed_mps", 0.0),
                lanes="ON" if info.get("show_lanes") else "OFF",
            ),
            COL_DIM,
            "mono",
        ),
        (
            "Heatmap: {heatmap} (H toggle / Shift+H mode {mode})".format(
                heatmap="ON" if info.get("show_heatmap") else "OFF",
                mode=info.get("heatmap_mode", "visits").upper(),
            ),
            COL_DIM,
            "body",
        ),
        (
            "Recommendations: {} (M toggle)".format(
                "ON" if info.get("show_recommendations", True) else "OFF"
            ),
            COL_DIM,
            "body",
        ),
        (
            "Turn penalties: {} (T toggle)".format(
                "ON" if info.get("turn_penalties_on") else "OFF"
            ),
            COL_DIM,
            "body",
        ),
        (
            "Scale: 1 px = {scale:.3f} m (1000px ~ {thousand:.1f} m)".format(
                scale=info.get("scale_m_per_px", 0.0),
                thousand=1000 * info.get("scale_m_per_px", 0.0),
            ),
            COL_DIM,
            "body",
        ),
    ]
    y = draw_section(screen, fonts, "Parameters", param_lines, (x, y), max_width)

    metrics = info.get("plan_metrics")
    metrics_lines: list[tuple[str, tuple[int, int, int], str]] = []
    if metrics:
        coverage_pct = metrics.get("coverage", 0.0) * 100
        metrics_lines.append(
            (
                "Coverage: {covered}/{free} ({pct:.1f}%)".format(
                    covered=metrics.get("covered", 0),
                    free=metrics.get("free", 0),
                    pct=coverage_pct,
                ),
                COL_TEXT,
                "mono",
            )
        )
        metrics_lines.append(
            (
                "Best angle {deg}° | Blade step ~{sweep} cells | Steps {steps}".format(
                    deg=metrics.get("deg", 0),
                    sweep=metrics.get("sweep_step_cells", 0),
                    steps=metrics.get("steps", 0),
                ),
                COL_DIM,
                "mono",
            )
        )
        pm = metrics.get("path_metrics", {})
        metrics_lines.append(
            (
                "Distance {dist:.1f} m | Base {base:.1f} min | Turn-adjusted {turn:.1f} min".format(
                    dist=pm.get("distance_m", 0.0),
                    base=pm.get("base_time_min", 0.0),
                    turn=pm.get("turn_time_min", 0.0),
                ),
                COL_DIM,
                "mono",
            )
        )
        metrics_lines.append(
            (
                "Overlap {overlap:.1f}% | Decision {decision:.1f} min | Time toggle {toggle:.1f} min".format(
                    overlap=pm.get("inefficiency_pct", 0.0),
                    decision=pm.get("decision_time_min", 0.0),
                    toggle=pm.get("time_with_toggle_min", 0.0),
                ),
                COL_DIM,
                "mono",
            )
        )
        metrics_lines.append(
            (
                "Turns {turns} ({u_turns} U) | Density {density:.1f}/100m | Score {score:.1f}".format(
                    turns=metrics.get("turns", 0),
                    u_turns=metrics.get("u_turns", 0),
                    density=pm.get("turn_density_per_100m", 0.0),
                    score=metrics.get("score", 0.0),
                ),
                COL_DIM,
                "mono",
            )
        )
    else:
        metrics_lines.append(("No plan yet. Press ENTER to compute.", COL_DIM, "body"))
    y = draw_section(screen, fonts, "Metrics", metrics_lines, (x, y), max_width)

    controls_lines: list[tuple[str, tuple[int, int, int], str]] = []
    if info.get("mode") == "draw":
        controls_lines.append(
            (
                "Draw: B boundary / O obstacle / S start | click or drag to add points | ENTER to plan", COL_DIM, "body"
            )
        )
        controls_lines.append(("R to reset, P to add point at cursor", COL_DIM, "body"))
    elif info.get("mode") == "plan":
        controls_lines.append(("SPACE pause/resume | +/- animation speed | R redraw", COL_DIM, "body"))
        controls_lines.append(("H heatmap overlay | Shift+H cycle mode | L lanes toggle", COL_DIM, "body"))
    elif info.get("planning"):
        controls_lines.append(("Planning… lower resolution with , or change angle with A if slow", COL_WARN, "body"))
    y = draw_section(screen, fonts, "Controls", controls_lines, (x, y), max_width)

    recs = metrics.get("recommendations") if metrics else []
    if recs:
        rec_lines = [(line, COL_DIM if not line.startswith("!") else COL_WARN, "body") for line in summarize_recommendations(recs)]
        draw_section(screen, fonts, "Recommendations (M toggle)", rec_lines, (x, y), max_width)
