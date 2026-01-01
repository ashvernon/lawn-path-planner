from lawn_path_planner.recommendation import compute_shape_features, recommend_mower


def base_features():
    return {
        "area_m2": 800.0,
        "decision_time_min": 50.0,
        "turn_density_per_100m": 10.0,
        "inefficiency_pct": 5.0,
        "obstacle_fraction_pct": 5.0,
        "obstacle_count": 1,
        "narrowness_proxy": 7.0,
    }


def test_push_mower_for_small_simple_lawn():
    features = base_features()
    features.update({"area_m2": 180.0, "decision_time_min": 18.0, "turn_density_per_100m": 6.0})

    recs = recommend_mower(features)
    assert recs[0].category == "Push mower"
    assert any("compact" in r.lower() for r in recs[0].reasons)


def test_ride_on_for_large_time_consuming_lawn():
    features = base_features()
    features.update({"area_m2": 2100.0, "decision_time_min": 95.0, "turn_density_per_100m": 9.0})

    recs = recommend_mower(features)
    assert recs[0].category == "Ride-on mower"
    assert any("ride-on" in warn or "caution" in warn for warn in recs[0].warnings) is False


def test_robotic_warns_on_obstacle_complexity():
    features = base_features()
    features.update({"obstacle_fraction_pct": 15.0, "obstacle_count": 6, "narrowness_proxy": 10.5})

    recs = recommend_mower(features)
    robo = next(r for r in recs if r.category == "Robotic mower")
    assert any("not recommended" in w.lower() for w in robo.warnings)


def test_shape_features_handles_holes():
    square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    hole = [(2.0, 2.0), (8.0, 2.0), (8.0, 8.0), (2.0, 8.0)]

    features = compute_shape_features(square, [hole])
    assert features["area_m2"] == 64.0
    assert round(features["compactness"], 2) < 1.0
