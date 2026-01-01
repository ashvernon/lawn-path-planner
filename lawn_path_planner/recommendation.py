from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

from .metrics import estimate_distance_m, estimate_overlap_inefficiency, estimate_time_min
from .types import PlanState


@dataclass
class Recommendation:
    category: str
    score: float
    reasons: List[str]
    warnings: List[str]


def _polygon_area(poly: List[tuple[float, float]]) -> float:
    area = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def _polygon_perimeter(poly: List[tuple[float, float]]) -> float:
    perim = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        dx = x2 - x1
        dy = y2 - y1
        perim += math.hypot(dx, dy)
    return perim


def compute_shape_features(
    poly_m: List[tuple[float, float]], obstacles_m: Optional[List[List[tuple[float, float]]]] = None
) -> Dict[str, float]:
    if len(poly_m) < 3:
        return {
            "area_m2": 0.0,
            "perimeter_m": 0.0,
            "compactness": 0.0,
            "obstacle_count": 0,
            "obstacle_fraction_pct": 0.0,
            "narrowness_proxy": 0.0,
            "obstacle_area_m2": 0.0,
        }

    outer_area = _polygon_area(poly_m)
    obstacle_area = sum(_polygon_area(ob) for ob in obstacles_m or [])
    net_area = max(0.0, outer_area - obstacle_area)
    perimeter = _polygon_perimeter(poly_m)
    compactness = 0.0
    if perimeter > 0:
        compactness = min(1.0, 4 * math.pi * net_area / max(perimeter * perimeter, 1e-9))

    narrowness_proxy = 0.0
    if net_area > 0:
        narrowness_proxy = perimeter / math.sqrt(net_area)

    obstacle_fraction = 0.0
    if outer_area > 0:
        obstacle_fraction = (obstacle_area / outer_area) * 100.0

    return {
        "area_m2": net_area,
        "perimeter_m": perimeter,
        "compactness": compactness,
        "obstacle_count": len(obstacles_m or []),
        "obstacle_fraction_pct": obstacle_fraction,
        "narrowness_proxy": narrowness_proxy,
        "obstacle_area_m2": obstacle_area,
    }


def compute_path_complexity_metrics(
    plan: PlanState,
    mower_speed_mps: float,
    turn_penalty_90_s: float,
    turn_penalty_180_s: float,
    penalties_enabled: bool,
) -> Dict[str, float]:
    distance_m = estimate_distance_m(plan)
    base_time_min = estimate_time_min(
        distance_m,
        mower_speed_mps,
        plan.turns,
        plan.u_turns,
        turn_penalty_90_s,
        turn_penalty_180_s,
        penalties_enabled=False,
    )
    turn_time_min = estimate_time_min(
        distance_m,
        mower_speed_mps,
        plan.turns,
        plan.u_turns,
        turn_penalty_90_s,
        turn_penalty_180_s,
        penalties_enabled=True,
    )
    time_with_toggle_min = estimate_time_min(
        distance_m,
        mower_speed_mps,
        plan.turns,
        plan.u_turns,
        turn_penalty_90_s,
        turn_penalty_180_s,
        penalties_enabled=penalties_enabled,
    )
    ineff_pct = estimate_overlap_inefficiency(plan)
    decision_time_min = turn_time_min * (1 + ineff_pct / 100.0)

    turn_density = 0.0
    if distance_m > 0:
        turn_density = plan.turns / (distance_m / 100.0)

    return {
        "distance_m": distance_m,
        "base_time_min": base_time_min,
        "turn_time_min": turn_time_min,
        "time_with_toggle_min": time_with_toggle_min,
        "decision_time_min": decision_time_min,
        "turn_density_per_100m": turn_density,
        "inefficiency_pct": ineff_pct,
        "turns": plan.turns,
        "u_turns": plan.u_turns,
    }


def _add_reason(reasons: List[str], reason: str, max_reasons: int = 3):
    if len(reasons) < max_reasons:
        reasons.append(reason)


def _apply_pref_bias(
    category: str,
    prefs: Dict[str, str],
    reasons: List[str],
    score: float,
    add_reason,
) -> float:
    budget = prefs.get("budget", "Medium")
    effort = prefs.get("effort", "Medium")
    noise = prefs.get("noise", "Medium")
    storage = prefs.get("storage", "Normal")
    terrain = prefs.get("terrain", "Flat")

    if budget == "Low" and category in ("Push mower", "Battery/electric walk-behind"):
        score += 1.0
        add_reason(f"Budget {budget}: favors simpler mowers")
    elif budget == "High" and category in ("Ride-on mower", "Robotic mower"):
        score += 0.8
        add_reason(f"Budget {budget}: higher-end ok")

    if effort == "Low" and category in ("Ride-on mower", "Robotic mower", "Self-propelled mower"):
        score += 1.5
        add_reason("Low effort preference")
    elif effort == "High" and category == "Push mower":
        score += 0.5
        add_reason("High effort tolerance")

    if noise == "High" and category in ("Battery/electric walk-behind", "Robotic mower"):
        score += 1.2
        add_reason("Noise-sensitive: prefers quieter options")
    elif noise == "Low" and category == "Ride-on mower":
        score += 0.3

    if storage == "Small" and category == "Ride-on mower":
        score -= 2.0
        add_reason("Limited storage penalises ride-on")

    if terrain == "Steep" and category == "Ride-on mower":
        score -= 1.2
        add_reason("Steep terrain penalises ride-on")
    elif terrain == "Steep" and category == "Robotic mower":
        score -= 0.8
        add_reason("Steep terrain harder for robots")

    return score


def recommend_mower(features: Dict[str, float], prefs: Optional[Dict[str, str]] = None) -> List[Recommendation]:
    prefs = prefs or {}
    categories: Iterable[str] = (
        "Push mower",
        "Self-propelled mower",
        "Battery/electric walk-behind",
        "Robotic mower",
        "Ride-on mower",
    )

    area = features.get("area_m2", 0.0)
    decision_time = features.get("decision_time_min", 0.0)
    turn_density = features.get("turn_density_per_100m", 0.0)
    ineff = features.get("inefficiency_pct", 0.0)
    obstacle_fraction = features.get("obstacle_fraction_pct", 0.0)
    obstacle_count = features.get("obstacle_count", 0)
    narrowness = features.get("narrowness_proxy", 0.0)

    recommendations: List[Recommendation] = []

    for category in categories:
        score = 0.0
        reasons: List[str] = []
        warnings: List[str] = []
        add_reason = lambda r, reasons=reasons: _add_reason(reasons, r)

        if category == "Ride-on mower":
            if area >= 2000:
                score += 4.0
                add_reason(f"Area: {area:.0f} m² suits ride-on")
            elif area >= 1500:
                score += 3.4
                add_reason(f"Area: {area:.0f} m² near ride-on threshold")
            if decision_time >= 90:
                score += 2.5
                add_reason(f"Turn-adjusted time: {decision_time:.0f} min")
            elif decision_time >= 60:
                score += 1.6
                add_reason(f"Turn-adjusted time: {decision_time:.0f} min")
            if obstacle_fraction >= 12 or obstacle_count >= 6:
                score -= 1.4
                warnings.append("Ride-on caution: many obstacles")
            if narrowness >= 8.0:
                score -= 0.6
                warnings.append("Ride-on caution: narrow passages")

        elif category == "Self-propelled mower":
            if decision_time >= 35:
                score += 2.2
                add_reason(f"Time: {decision_time:.0f} min benefits assist")
            if turn_density >= 12:
                score += 1.5
                add_reason(f"Turn density: {turn_density:.1f}/100m")
            if ineff >= 12:
                score += 0.8
                add_reason(f"Overlap inefficiency: {ineff:.1f}%")
            if area >= 800:
                score += 0.6
                add_reason(f"Area: {area:.0f} m²")

        elif category == "Push mower":
            if area <= 250:
                score += 2.8
                add_reason(f"Area: {area:.0f} m² is compact")
            if decision_time <= 25:
                score += 1.6
                add_reason(f"Time: {decision_time:.0f} min is low")
            if turn_density <= 8:
                score += 0.7
                add_reason(f"Turn density: {turn_density:.1f}/100m")
            if area >= 700 or decision_time >= 40:
                score -= 1.5
                warnings.append("Push mower effort may be high")

        elif category == "Battery/electric walk-behind":
            if area <= 900:
                score += 1.5
                add_reason(f"Area: {area:.0f} m²")
            if decision_time <= 60:
                score += 1.0
                add_reason(f"Time: {decision_time:.0f} min")
            if obstacle_fraction < 15:
                score += 0.8
                add_reason(f"Obstacle fraction: {obstacle_fraction:.1f}%")
            if area >= 1400:
                score -= 1.2
                warnings.append("Battery runtime may be limiting")

        elif category == "Robotic mower":
            if 300 <= area <= 2000:
                score += 2.4
                add_reason(f"Area: {area:.0f} m² fits robotic range")
            elif area > 2000:
                score += 1.2
                add_reason(f"Area: {area:.0f} m² large but possible")
            if obstacle_fraction <= 8 and obstacle_count <= 4:
                score += 2.2
                add_reason(f"Low obstacle fraction: {obstacle_fraction:.1f}%")
            if turn_density <= 10:
                score += 0.8
                add_reason(f"Turn density: {turn_density:.1f}/100m")
            if obstacle_fraction >= 12 or narrowness >= 9.5:
                warnings.append("Robotic mower not recommended: high obstacle complexity / narrow corridors")
                score -= 1.8

        score = _apply_pref_bias(category, prefs, reasons, score, add_reason)

        recommendations.append(Recommendation(category, score, reasons, warnings))

    recommendations.sort(key=lambda r: (-r.score, r.category))
    return recommendations
