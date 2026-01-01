from __future__ import annotations

from typing import List, Optional, Tuple

import pygame

from . import config
from .config import ANGLE_STEPS
from .planning import compute_best_plan, maybe_cap_resolution, next_angle_step_idx
from .render import draw_plan, draw_text
from .types import PlanState, PlannerJob, Point


def poly_px_to_m(poly_px: List[Tuple[int, int]], scale_m_per_px: float, origin_px: Tuple[int, int]) -> List[Point]:
    ox, oy = origin_px
    return [((x - ox) * scale_m_per_px, (y - oy) * scale_m_per_px) for (x, y) in poly_px]


def main():
    pygame.init()
    screen = pygame.display.set_mode((config.WIN_W, config.WIN_H))
    pygame.display.set_caption("Lawn Path Planner MVP v3.1")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)

    poly_px: List[Tuple[int, int]] = []
    scale_m_per_px = config.DEFAULT_SCALE_M_PER_PX
    origin_px = config.DEFAULT_ORIGIN_PX

    blade_w_m = config.DEFAULT_BLADE_W_M
    cells_per_m = config.DEFAULT_CELLS_PER_M
    angle_step_idx = config.INITIAL_ANGLE_STEP_IDX
    mower_speed_mps = config.INITIAL_MOWER_SPEED

    show_lanes = True

    plan: Optional[PlanState] = None
    paused = False
    anim_speed = config.INITIAL_ANIM_SPEED
    mode = "draw"
    status_msg = ""

    job = PlannerJob()

    left_down = False
    last_added: Optional[Tuple[int, int]] = None

    def add_point(pt: Tuple[int, int], why: str):
        nonlocal status_msg, last_added
        poly_px.append(pt)
        last_added = pt
        status_msg = f"Added point {pt} ({why}). Total points: {len(poly_px)}"

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

                if mode == "draw":
                    if ev.key == pygame.K_p:
                        add_point((mx, my), "P key")
                    elif ev.key == pygame.K_BACKSPACE and poly_px:
                        poly_px.pop()
                        status_msg = f"Removed last point. Total points: {len(poly_px)}"
                        last_added = poly_px[-1] if poly_px else None
                    elif ev.key == pygame.K_r:
                        poly_px.clear()
                        status_msg = "Reset polygon."
                        last_added = None
                    elif ev.key == pygame.K_RETURN:
                        if len(poly_px) >= 3:
                            poly_m = poly_px_to_m(poly_px, scale_m_per_px, origin_px)
                            capped = maybe_cap_resolution(poly_m, float(cells_per_m), blade_w_m, config.MAX_GRID_CELLS)
                            if capped != float(cells_per_m):
                                status_msg = f"Auto-lowered resolution to {int(capped)} cells/m for speed."
                                cells_per_m = int(capped)

                            mode = "planning"
                            paused = False
                            plan = None
                            job.start(poly_m, float(cells_per_m), blade_w_m, ANGLE_STEPS[angle_step_idx], compute_best_plan)
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
                if config.DRAG_ADD_ENABLED and left_down:
                    if last_added is None:
                        add_point(ev.pos, "drag start")
                    else:
                        dx = ev.pos[0] - last_added[0]
                        dy = ev.pos[1] - last_added[1]
                        if dx * dx + dy * dy >= config.DRAG_MIN_DIST_PX * config.DRAG_MIN_DIST_PX:
                            add_point(ev.pos, "drag")

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
                plan.visited[y, x] = 1
                plan.path_i += 1

        screen.fill(config.COL_BG)

        draw_text(screen, font, 20, 16, "Lawn Path Planner MVP v3.1")
        draw_text(
            screen,
            font,
            20,
            36,
            f"Blade: {blade_w_m:.2f} m | Resolution: {int(cells_per_m)} cells/m | Angle step: {ANGLE_STEPS[angle_step_idx]}°  ( [ ] blade, , . res, A angle-step )",
            config.COL_DIM,
        )
        draw_text(
            screen,
            font,
            20,
            56,
            f"Mower speed: {mower_speed_mps:.1f} m/s  (1/2 adjust) | Lanes: {'ON' if show_lanes else 'OFF'} (L toggle)",
            config.COL_DIM,
        )

        if status_msg:
            draw_text(screen, font, 20, 78, status_msg, config.COL_WARN)

        if mode == "draw":
            pygame.draw.circle(screen, config.COL_CURSOR, (mx, my), 3)
            draw_text(
                screen,
                font,
                20,
                110,
                "Draw: click (release) adds point | hold left+drag draws | P adds point | ENTER plan | R reset",
                config.COL_DIM,
            )
            pygame.draw.circle(screen, (255, 120, 120), origin_px, 4)
            draw_text(screen, font, origin_px[0] + 8, origin_px[1] - 8, "origin", (255, 120, 120))
            draw_text(
                screen,
                font,
                20,
                132,
                f"Scale: 1 px = {scale_m_per_px:.3f} m  (1000px ~ {1000 * scale_m_per_px:.1f}m)",
                config.COL_DIM,
            )

            if len(poly_px) >= 1:
                for p in poly_px:
                    pygame.draw.circle(screen, config.COL_POLY, p, 4)
            if len(poly_px) >= 2:
                pygame.draw.lines(screen, config.COL_POLY_EDGE, False, poly_px, 2)

        elif mode == "planning":
            draw_text(
                screen,
                font,
                20,
                110,
                "Planning… Tip: press , to lower resolution or A to increase angle-step if it’s slow.",
                config.COL_WARN,
            )
            if len(poly_px) >= 2:
                pygame.draw.lines(screen, config.COL_POLY_EDGE, False, poly_px, 2)

        elif mode == "plan" and plan is not None:
            draw_plan(screen, font, plan, show_lanes, mower_speed_mps, paused, anim_speed)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
