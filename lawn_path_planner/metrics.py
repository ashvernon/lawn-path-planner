from __future__ import annotations

from .types import PlanState


def estimate_distance_m(plan: PlanState) -> float:
    return plan.steps * plan.cell_size_m


def estimate_time_min(
    distance_m: float,
    mower_speed_mps: float,
    turns: int,
    u_turns: int,
    turn_penalty_90_s: float,
    turn_penalty_180_s: float,
    penalties_enabled: bool,
) -> float:
    est_time_s = distance_m / max(1e-6, mower_speed_mps)
    if penalties_enabled:
        ninety_turns = max(0, turns - u_turns)
        est_time_s += ninety_turns * turn_penalty_90_s
        est_time_s += u_turns * turn_penalty_180_s
    return est_time_s / 60.0


def estimate_coverage(plan: PlanState) -> tuple[int, int, float]:
    free = int(plan.grid.sum())
    covered = int(plan.visited.sum())
    coverage = (covered / free) if free else 0.0
    return free, covered, coverage


def estimate_overlap_inefficiency(plan: PlanState) -> float:
    lawn_area_m2 = float(plan.grid.sum()) * plan.cell_size_m * plan.cell_size_m
    if lawn_area_m2 <= 0:
        return 0.0

    sweep_width_m = plan.sweep_step_cells * plan.cell_size_m
    lane_area_m2 = 0.0
    for (ax, ay), (bx, _by) in plan.lanes:
        length_cells = abs(bx - ax) + 1
        lane_area_m2 += length_cells * sweep_width_m * plan.cell_size_m

    inefficiency = max(0.0, (lane_area_m2 / lawn_area_m2) - 1.0)
    return inefficiency * 100.0
