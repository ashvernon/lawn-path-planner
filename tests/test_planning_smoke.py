from lawn_path_planner.planning import compute_best_plan
from lawn_path_planner.types import Point


def test_compute_best_plan_returns_plan_for_simple_polygon():
    poly: list[Point] = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    result = compute_best_plan(poly, cells_per_meter=4.0, blade_w_m=0.5, angle_step_deg=30)

    assert result.grid.sum() > 0
    assert result.path is not None
    assert 0 <= result.deg < 180


def test_compute_best_plan_respects_obstacles_and_start():
    poly: list[Point] = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    obstacle: list[Point] = [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)]

    no_obstacle = compute_best_plan(poly, cells_per_meter=2.0, blade_w_m=0.5, angle_step_deg=180)
    with_obstacle = compute_best_plan(
        poly,
        cells_per_meter=2.0,
        blade_w_m=0.5,
        angle_step_deg=180,
        start_point=(0.2, 0.2),
        obstacles=[obstacle],
    )

    assert with_obstacle.grid.sum() < no_obstacle.grid.sum()
    sx, sy = with_obstacle.start
    assert with_obstacle.grid[sy, sx] == 1
