from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import threading
import numpy as np

Point = Tuple[float, float]
Cell = Tuple[int, int]


@dataclass
class PlanResult:
    deg: int
    grid: np.ndarray
    path: List[Cell]
    start: Cell
    cell_size_m: float
    sweep_step_cells: int
    score: float
    steps: int
    turns: int
    u_turns: int
    lanes: List[Tuple[Cell, Cell]]


@dataclass
class PlanState:
    deg: int
    grid: np.ndarray
    path: List[Cell]
    start: Cell
    cell_size_m: float
    sweep_step_cells: int
    score: float
    steps: int
    turns: int
    u_turns: int
    lanes: List[Tuple[Cell, Cell]]
    path_i: int = 0
    visited: Optional[np.ndarray] = None

    def __post_init__(self):
        self.visited = np.zeros_like(self.grid, dtype=np.uint8)
        sx, sy = self.start
        if self.grid[sy, sx] == 1:
            self.visited[sy, sx] = 1

    @classmethod
    def from_result(cls, result: PlanResult) -> "PlanState":
        return cls(
            deg=result.deg,
            grid=result.grid,
            path=result.path,
            start=result.start,
            cell_size_m=result.cell_size_m,
            sweep_step_cells=result.sweep_step_cells,
            score=result.score,
            steps=result.steps,
            turns=result.turns,
            u_turns=result.u_turns,
            lanes=result.lanes,
        )


@dataclass
class PlannerJob:
    lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    running: bool = False
    error: Optional[str] = None
    result: Optional[PlanResult] = None

    def start(
        self,
        poly_m: List[Point],
        cells_per_m: float,
        blade_w_m: float,
        angle_step_deg: int,
        worker_fn,
        start_point: Optional[Point] = None,
        obstacles: Optional[List[List[Point]]] = None,
    ):
        with self.lock:
            if self.running:
                return
            self.running = True
            self.error = None
            self.result = None

        def worker():
            try:
                best = worker_fn(
                    poly_m,
                    cells_per_m,
                    blade_w_m,
                    angle_step_deg,
                    start_point=start_point,
                    obstacles=obstacles,
                )
                with self.lock:
                    self.result = best
            except Exception as exc:  # noqa: BLE001
                with self.lock:
                    self.error = str(exc)
            finally:
                with self.lock:
                    self.running = False

        threading.Thread(target=worker, daemon=True).start()

    def poll(self):
        with self.lock:
            return self.running, self.error, self.result
