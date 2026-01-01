from __future__ import annotations

from typing import List, Optional, Tuple

import pygame

from . import config
from .config import ANGLE_STEPS, TURN_PENALTIES_ENABLED, TURN_PENALTY_180_S, TURN_PENALTY_90_S
from .geometry import point_in_poly
from .planning import compute_best_plan, maybe_cap_resolution, next_angle_step_idx
from .recommendation import compute_shape_features
from .render import draw_hud_panel, draw_panel_background, draw_plan, layout_rects
from .types import PlanState, PlannerJob, Point


def poly_px_to_m(poly_px: List[Tuple[int, int]], scale_m_per_px: float, origin_px: Tuple[int, int]) -> List[Point]:
    ox, oy = origin_px
    return [((x - ox) * scale_m_per_px, (y - oy) * scale_m_per_px) for (x, y) in poly_px]


def main():
    pygame.init()
    screen = pygame.display.set_mode((config.WIN_W, config.WIN_H))
    pygame.display.set_caption("Lawn Path Planner MVP v3.1")
    clock = pygame.time.Clock()
    font_body = pygame.font.SysFont("consolas", 14)
    font_title = pygame.font.SysFont("consolas", 17)
    font_mono = pygame.font.SysFont("consolas", 14)
    fonts = {"body": font_body, "title": font_title, "mono": font_mono}

    hud_rect, grid_rect = layout_rects()

    poly_px: List[Tuple[int, int]] = []
    obstacles_px: List[List[Tuple[int, int]]] = []
    active_obstacle_px: List[Tuple[int, int]] = []
    scale_m_per_px = config.DEFAULT_SCALE_M_PER_PX
    origin_px = config.DEFAULT_ORIGIN_PX

    blade_w_m = config.DEFAULT_BLADE_W_M
    cells_per_m = config.DEFAULT_CELLS_PER_M
    angle_step_idx = config.INITIAL_ANGLE_STEP_IDX
    mower_speed_mps = config.INITIAL_MOWER_SPEED

    show_lanes = config.DEFAULT_SHOW_LANES
    show_heatmap = False
    heatmap_modes = ["visits", "overlap", "log"]
    heatmap_mode_idx = 0
    draw_mode = "boundary"
    start_px: Optional[Tuple[int, int]] = None
    turn_penalties_on = TURN_PENALTIES_ENABLED
    show_recommendations = True

    plan: Optional[PlanState] = None
    paused = False
    anim_speed = config.INITIAL_ANIM_SPEED
    mode = "draw"
    status_msg = ""

    mower_prefs = {
        "budget": "Medium",
        "effort": "Medium",
        "noise": "Medium",
        "storage": "Normal",
        "terrain": "Flat",
    }

    job = PlannerJob()

    left_down = False
    last_added: Optional[Tuple[int, int]] = None
    last_obstacle_added: Optional[Tuple[int, int]] = None
    last_poly_m: Optional[List[Point]] = None
    last_obstacles_m: Optional[List[List[Point]]] = None
    shape_features: Optional[dict] = None

    pref_options = {
        "budget": ["Low", "Medium", "High"],
        "effort": ["Low", "Medium", "High"],
        "noise": ["Low", "Medium", "High"],
        "storage": ["Small", "Normal"],
        "terrain": ["Flat", "Some slope", "Steep"],
    }

    def add_point(pt: Tuple[int, int], why: str):
        nonlocal status_msg, last_added
        if not grid_rect.collidepoint(pt):
            status_msg = "Use the map area (right of HUD) to add points."
            return
        poly_px.append(pt)
        last_added = pt
        status_msg = f"Added point {pt} ({why}). Total points: {len(poly_px)}"

    def add_obstacle_point(pt: Tuple[int, int], why: str):
        nonlocal status_msg, last_obstacle_added
        if not grid_rect.collidepoint(pt):
            status_msg = "Use the map area (right of HUD) to add points."
            return
        active_obstacle_px.append(pt)
        last_obstacle_added = pt
        status_msg = f"Added obstacle point {pt} ({why}). Points: {len(active_obstacle_px)}"

    def cycle_pref(key: str):
        nonlocal status_msg
        options = pref_options[key]
        cur = mower_prefs[key]
        idx = (options.index(cur) + 1) % len(options)
        mower_prefs[key] = options[idx]
        status_msg = f"{key.title()} set to {options[idx]}"

    def point_in_lawn(px: Tuple[int, int]) -> bool:
        if len(poly_px) < 3:
            return False
        if not point_in_poly(px[0], px[1], poly_px):
            return False
        for hole in obstacles_px:
            if point_in_poly(px[0], px[1], hole):
                return False
        if active_obstacle_px and point_in_poly(px[0], px[1], active_obstacle_px):
            return False
        return True

    def trigger_planning():
        nonlocal mode, paused, plan, status_msg, cells_per_m, shape_features, last_poly_m, last_obstacles_m

        if len(poly_px) < 3:
            status_msg = "Need at least 3 points."
            return

        obstacles_all_px = list(obstacles_px)
        if len(active_obstacle_px) >= 3:
            obstacles_all_px.append(list(active_obstacle_px))

        poly_m = poly_px_to_m(poly_px, scale_m_per_px, origin_px)
        obstacles_m = [poly_px_to_m(ob, scale_m_per_px, origin_px) for ob in obstacles_all_px]
        last_poly_m = poly_m
        last_obstacles_m = obstacles_m
        shape_features = compute_shape_features(poly_m, obstacles_m)

        capped = maybe_cap_resolution(poly_m, float(cells_per_m), blade_w_m, config.MAX_GRID_CELLS)
        if capped != float(cells_per_m):
            status_msg = f"Auto-lowered resolution to {int(capped)} cells/m for speed."
            cells_per_m = int(capped)

        start_m: Optional[Point] = None
        if start_px and point_in_lawn(start_px):
            sx, sy = start_px
            ox, oy = origin_px
            start_m = ((sx - ox) * scale_m_per_px, (sy - oy) * scale_m_per_px)
        elif start_px:
            status_msg = "Start point not inside lawn; using auto start."

        mode = "planning"
        paused = False
        plan = None
        job.start(
            poly_m,
            float(cells_per_m),
            blade_w_m,
            ANGLE_STEPS[angle_step_idx],
            compute_best_plan,
            start_point=start_m,
            obstacles=obstacles_m,
        )

    running = True
    while running:
        clock.tick(config.FPS)
        mx, my = pygame.mouse.get_pos()

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False

            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False

                if ev.key == pygame.K_m:
                    show_recommendations = not show_recommendations
                    status_msg = f"Recommendations {'shown' if show_recommendations else 'hidden'}"

                if ev.key == pygame.K_F1:
                    cycle_pref("budget")
                elif ev.key == pygame.K_F2:
                    cycle_pref("effort")
                elif ev.key == pygame.K_F3:
                    cycle_pref("noise")
                elif ev.key == pygame.K_F4:
                    cycle_pref("storage")
                elif ev.key == pygame.K_F5:
                    cycle_pref("terrain")

                if ev.key == pygame.K_LEFTBRACKET:
                    blade_w_m = max(0.05, blade_w_m - 0.05)
                elif ev.key == pygame.K_RIGHTBRACKET:
                    blade_w_m = min(3.0, blade_w_m + 0.05)
                elif ev.key == pygame.K_COMMA:
                    cells_per_m = max(1, int(cells_per_m) - 1)
                elif ev.key == pygame.K_PERIOD:
                    cells_per_m = min(30, int(cells_per_m) + 1)
                elif ev.key == pygame.K_a:
                    angle_step_idx = next_angle_step_idx(angle_step_idx)

                elif ev.key == pygame.K_1:
                    mower_speed_mps = max(0.1, mower_speed_mps - 0.1)
                elif ev.key == pygame.K_2:
                    mower_speed_mps = min(2.5, mower_speed_mps + 0.1)

                elif ev.key == pygame.K_l:
                    show_lanes = not show_lanes

                elif ev.key == pygame.K_h:
                    if ev.mod & pygame.KMOD_SHIFT:
                        heatmap_mode_idx = (heatmap_mode_idx + 1) % len(heatmap_modes)
                        show_heatmap = True
                        status_msg = (
                            f"Heatmap mode: {heatmap_modes[heatmap_mode_idx].replace('_', ' ').title()}"
                        )
                    else:
                        show_heatmap = not show_heatmap
                        status_msg = f"Heatmap {'ON' if show_heatmap else 'OFF'}"

                elif ev.key == pygame.K_t:
                    turn_penalties_on = not turn_penalties_on
                    status_msg = f"Turn penalties {'ON' if turn_penalties_on else 'OFF'}"

                if mode == "draw":
                    if ev.key == pygame.K_o:
                        draw_mode = "obstacle"
                        status_msg = "Obstacle mode: click to add points, ENTER to finalize obstacle."
                    elif ev.key == pygame.K_s:
                        draw_mode = "start"
                        status_msg = "Start-point mode: click inside lawn to set start."
                    elif ev.key == pygame.K_b:
                        draw_mode = "boundary"
                        status_msg = "Boundary mode: draw lawn outline."

                if mode == "draw":
                    if draw_mode == "boundary" and ev.key == pygame.K_p:
                        add_point((mx, my), "P key")
                    elif draw_mode == "obstacle" and ev.key == pygame.K_p:
                        add_obstacle_point((mx, my), "P key")

                    if ev.key == pygame.K_BACKSPACE:
                        if draw_mode == "boundary" and poly_px:
                            poly_px.pop()
                            status_msg = f"Removed last point. Total points: {len(poly_px)}"
                            last_added = poly_px[-1] if poly_px else None
                        elif draw_mode == "obstacle" and active_obstacle_px:
                            active_obstacle_px.pop()
                            status_msg = f"Removed obstacle point. Points: {len(active_obstacle_px)}"
                            last_obstacle_added = active_obstacle_px[-1] if active_obstacle_px else None
                    elif ev.key == pygame.K_r:
                        poly_px.clear()
                        obstacles_px.clear()
                        active_obstacle_px.clear()
                        start_px = None
                        shape_features = None
                        last_poly_m = None
                        last_obstacles_m = None
                        show_heatmap = False
                        status_msg = "Reset lawn, obstacles, and start."
                        last_added = None
                        last_obstacle_added = None
                    elif ev.key == pygame.K_RETURN:
                        if draw_mode == "obstacle":
                            if len(active_obstacle_px) >= 3:
                                obstacles_px.append(list(active_obstacle_px))
                                active_obstacle_px.clear()
                                last_obstacle_added = None
                                status_msg = f"Saved obstacle #{len(obstacles_px)}."
                            else:
                                status_msg = "Need at least 3 points to save obstacle."
                        else:
                            trigger_planning()
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
                        obstacles_px.clear()
                        active_obstacle_px.clear()
                        start_px = None
                        plan = None
                        paused = False
                        show_heatmap = False
                        status_msg = "Redraw lawn."
                        last_added = None
                        last_obstacle_added = None
                        shape_features = None
                        last_poly_m = None
                        last_obstacles_m = None

            elif ev.type == pygame.MOUSEBUTTONDOWN and mode == "draw":
                if ev.button == 1:
                    left_down = grid_rect.collidepoint(ev.pos)
                    if not left_down:
                        status_msg = "Click inside the map area to draw."

            elif ev.type == pygame.MOUSEBUTTONUP and mode == "draw":
                if ev.button == 1:
                    left_down = False
                    if draw_mode == "boundary":
                        add_point(ev.pos, "mouse up")
                    elif draw_mode == "obstacle":
                        add_obstacle_point(ev.pos, "mouse up")
                    elif draw_mode == "start":
                        if not grid_rect.collidepoint(ev.pos):
                            status_msg = "Start must be inside the map area."
                        elif point_in_lawn(ev.pos):
                            start_px = ev.pos
                            draw_mode = "boundary"
                            status_msg = f"Start set at {ev.pos}. Press ENTER to plan."
                        else:
                            status_msg = "Start must be inside lawn and outside obstacles."

            elif ev.type == pygame.MOUSEMOTION and mode == "draw":
                if config.DRAG_ADD_ENABLED and left_down:
                    if draw_mode == "boundary":
                        if last_added is None:
                            add_point(ev.pos, "drag start")
                        else:
                            dx = ev.pos[0] - last_added[0]
                            dy = ev.pos[1] - last_added[1]
                            if dx * dx + dy * dy >= config.DRAG_MIN_DIST_PX * config.DRAG_MIN_DIST_PX:
                                add_point(ev.pos, "drag")
                    elif draw_mode == "obstacle":
                        if last_obstacle_added is None:
                            add_obstacle_point(ev.pos, "drag start")
                        else:
                            dx = ev.pos[0] - last_obstacle_added[0]
                            dy = ev.pos[1] - last_obstacle_added[1]
                            if dx * dx + dy * dy >= config.DRAG_MIN_DIST_PX * config.DRAG_MIN_DIST_PX:
                                add_obstacle_point(ev.pos, "drag")

        _, error, result = job.poll()
        if mode == "planning":
            if error:
                status_msg = f"Planning failed: {error}"
                mode = "draw"
            elif result is not None:
                plan = PlanState.from_result(result)
                mode = "plan"
                status_msg = "Plan ready. Press SPACE to pause/resume."
        elif mode == "plan" and plan is not None and not paused:
            for _ in range(anim_speed):
                if plan.path_i >= len(plan.path):
                    paused = True
                    break
                x, y = plan.path[plan.path_i]
                plan.visit_counts[y, x] += 1
                plan.visited[y, x] = 1
                plan.path_i += 1

        screen.fill(config.COL_BG)
        plan_metrics = None

        if mode == "draw":
            draw_panel_background(screen, grid_rect)
            pygame.draw.circle(screen, config.COL_CURSOR, (mx, my), 3)
            pygame.draw.circle(screen, (255, 120, 120), origin_px, 4)

            if len(poly_px) >= 1:
                for p in poly_px:
                    pygame.draw.circle(screen, config.COL_POLY, p, 4)
            if len(poly_px) >= 2:
                pygame.draw.lines(screen, config.COL_POLY_EDGE, False, poly_px, 2)

            for obs in obstacles_px:
                if len(obs) >= 1:
                    for p in obs:
                        pygame.draw.circle(screen, config.COL_BLOCK, p, 3)
                if len(obs) >= 2:
                    pygame.draw.lines(screen, config.COL_BLOCK, True, obs, 2)
            if active_obstacle_px:
                if len(active_obstacle_px) >= 2:
                    pygame.draw.lines(screen, config.COL_BLOCK, False, active_obstacle_px, 2)
                for p in active_obstacle_px:
                    pygame.draw.circle(screen, config.COL_BLOCK, p, 3)

            if start_px:
                pygame.draw.circle(screen, config.COL_PATH, start_px, 6, 2)

        elif mode == "planning":
            draw_panel_background(screen, grid_rect)
            if len(poly_px) >= 2:
                pygame.draw.lines(screen, config.COL_POLY_EDGE, False, poly_px, 2)
            for obs in obstacles_px:
                if len(obs) >= 2:
                    pygame.draw.lines(screen, config.COL_BLOCK, True, obs, 2)
            if start_px:
                pygame.draw.circle(screen, config.COL_PATH, start_px, 6, 2)

        elif mode == "plan" and plan is not None:
            plan_metrics = draw_plan(
                screen,
                font_body,
                plan,
                show_lanes,
                show_heatmap,
                heatmap_modes[heatmap_mode_idx],
                mower_speed_mps,
                paused,
                anim_speed,
                turn_penalties_on,
                TURN_PENALTY_90_S,
                TURN_PENALTY_180_S,
                shape_features,
                mower_prefs,
                show_recommendations,
                grid_rect,
            )
        else:
            draw_panel_background(screen, grid_rect)

        hud_info = {
            "mode": mode,
            "draw_mode": draw_mode,
            "obstacle_count": len(obstacles_px) + (1 if len(active_obstacle_px) >= 3 else 0),
            "start_set": bool(start_px),
            "status_msg": status_msg,
            "blade_w_m": blade_w_m,
            "cells_per_m": cells_per_m,
            "angle_step": ANGLE_STEPS[angle_step_idx],
            "mower_speed_mps": mower_speed_mps,
            "show_lanes": show_lanes,
            "turn_penalties_on": turn_penalties_on,
            "planning": mode == "planning",
            "paused": paused,
            "plan_metrics": plan_metrics,
            "scale_m_per_px": scale_m_per_px,
            "show_recommendations": show_recommendations,
            "show_heatmap": show_heatmap,
            "heatmap_mode": heatmap_modes[heatmap_mode_idx],
        }

        draw_hud_panel(screen, fonts, hud_rect, hud_info)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
