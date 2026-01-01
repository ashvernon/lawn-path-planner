from lawn_path_planner.planning import compute_best_plan
from lawn_path_planner.types import Point


def test_compute_best_plan_returns_plan_for_simple_polygon():
    poly: list[Point] = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    result = compute_best_plan(poly, cells_per_meter=4.0, blade_w_m=0.5, angle_step_deg=30)

    assert result.grid.sum() > 0
    assert result.path is not None
    assert 0 <= result.deg < 180
