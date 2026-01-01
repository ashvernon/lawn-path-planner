from __future__ import annotations

from .types import PlanState


def estimate_distance_m(plan: PlanState) -> float:
    return plan.steps * plan.cell_size_m


def estimate_time_min(distance_m: float, mower_speed_mps: float) -> float:
    est_time_s = distance_m / max(1e-6, mower_speed_mps)
    return est_time_s / 60.0


def estimate_coverage(plan: PlanState) -> tuple[int, int, float]:
    free = int(plan.grid.sum())
    covered = int(plan.visited.sum())
    coverage = (covered / free) if free else 0.0
    return free, covered, coverage
