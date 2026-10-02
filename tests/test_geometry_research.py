"""Analytical geometry/degeneracy tests, no sensor or board performance claims."""

import json
import math

import numpy as np
import pytest

from research.geometry import (
    DEFAULT_CONFIG, Wall, axis_fov_from_diagonal, intersect_wall, rotation,
    run_geometry, zone_rays, _config, _sensor_geometry, _observe,
    _measurement_model, _predict, _jacobian, _observability,
)


@pytest.fixture(scope="module")
def default_result():
    return run_geometry()


def scenarios(result):
    return {scene["id"]: scene for scene in result["scenarios"]}


def test_hand_solvable_perpendicular_and_slant_intersection():
    wall = Wall("x=2", (2, -3), (2, 3), 4)
    assert intersect_wall((0, 0, 2.5), (1, 0, 0), wall) == pytest.approx(2)
    assert intersect_wall((0, 0, 2.5), (2, 1, 0), wall) == pytest.approx(math.sqrt(5))
    # Physical slant distance sqrt(8), endpoint (2,0,0.5).
    assert intersect_wall((0, 0, 2.5), (1, 0, -1), wall) == pytest.approx(math.sqrt(8))


@pytest.mark.parametrize("direction,max_range,wall", [
    ((0, 1, 0), 3, Wall("parallel", (2, -3), (2, 3), 4)),
    ((-1, 0, 0), 3, Wall("behind", (2, -3), (2, 3), 4)),
    ((1, 0, 0), 1.99, Wall("out of range", (2, -3), (2, 3), 4)),
    ((1, 1, 0), 3, Wall("outside segment", (2, -1), (2, 1), 4)),
    ((1, 0, 0), 3, Wall("above wall", (2, -3), (2, 3), 1.1)),
    ((1, 0, -1), 3, Wall("below wall", (2, -3), (2, 3), 4, 1)),
])
def test_no_false_intersections(direction, max_range, wall):
    assert intersect_wall((0, 0, 2.5), direction, wall, max_range) is None


def test_finite_segment_and_height_boundaries():
    wall = Wall("boundary", (2, -1), (2, 1), 2.5)
    assert intersect_wall((0, 0, 2.5), (2, 1, 0), wall) == pytest.approx(math.sqrt(5))
    assert intersect_wall((0, 0, 2.5001), (1, 0, 0), wall) is None


def test_diagonal_not_horizontal_and_centres_not_fov_edges():
    axis = axis_fov_from_diagonal(65)
    assert 48 < axis < 49
    assert math.tan(math.radians(65 / 2)) == pytest.approx(math.sqrt(2) * math.tan(math.radians(axis / 2)))
    rays = zone_rays()
    assert rays.shape == (64, 3)
    np.testing.assert_allclose(np.linalg.norm(rays, axis=1), 1)
    # 8 cells: outermost centre is 7/8 of the half-width, not on the edge.
    assert np.max(np.abs(rays[:, 1] / rays[:, 0])) == pytest.approx(0.875 * math.tan(math.radians(axis / 2)))


def test_mount_rotation_and_roll_have_3d_effect():
    np.testing.assert_allclose(rotation(pitch=math.pi / 2) @ [1, 0, 0], [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(rotation(roll=math.pi / 2) @ [0, 1, 0], [0, 0, 1], atol=1e-12)
    tilted = _sensor_geometry(_config({"mount_pitch_deg": -30, "mount_roll_deg": 15}))
    level = _sensor_geometry(_config(None))
    assert not np.allclose(tilted[0][:, 2], level[0][:, 2])
    np.testing.assert_allclose(np.linalg.norm(tilted[0], axis=1), 1)


def test_axial_depth_vs_radial_and_nearest_wall():
    cfg = _config({"monte_carlo_trials": 0})
    geometry = _sensor_geometry(cfg)
    walls = [Wall("far", (2.8, -8), (2.8, 8), 4), Wall("near", (2, -8), (2, 8), 4)]
    hits, _ = _observe(walls, cfg, geometry)
    forward = [hit for hit in hits if hit["ray"] < 64]
    assert len(forward) == 64
    assert all(hit["wall"] == 1 for hit in forward)
    np.testing.assert_allclose([hit["range_m"] for hit in forward], 1.96)
    assert all(hit["radial_range_m"] > hit["range_m"] for hit in forward)
    model = _measurement_model(walls, forward, cfg, geometry)
    np.testing.assert_allclose(_predict(np.zeros(3), model), 1.96)
    jq = _jacobian(np.zeros(3), model, 1.0)
    np.testing.assert_allclose(jq[:, 0], -1, atol=1e-9)
    np.testing.assert_allclose(jq[:, 1], 0, atol=1e-9)


def test_synthetic_layout_degeneracy(default_result):
    scenes = scenarios(default_result)
    assert scenes["single_wall"]["rank"] == 2
    assert scenes["perpendicular_corner"]["rank"] == 3
    assert scenes["parallel_corridor"]["rank"] == 2
    assert scenes["open_area"]["rank"] == 0
    assert scenes["perpendicular_corner"]["smallest_singular_value"] > 0.1
    for scene, expected_null in (("single_wall", [0, 1, 0]), ("parallel_corridor", [1, 0, 0])):
        null = scenes[scene]["observability"]["nullspace_q"][0]
        assert abs(np.dot(null, expected_null)) == pytest.approx(1, abs=1e-6)
        assert scenes[scene]["smallest_singular_value"] < 1e-7
        assert scenes[scene]["monte_carlo"]["translation_rmse_m"] is None


def test_yaw_scale_and_rms_normalization_do_not_count_rays_as_confidence():
    cfg = _config(None)
    geo = _sensor_geometry(cfg)
    walls = [Wall("wall", (2, -8), (2, 8), 4)]
    hits, _ = _observe(walls, cfg, geo)
    model = _measurement_model(walls, hits, cfg, geo)
    j1 = _jacobian(np.zeros(3), model, 1)
    j2 = _jacobian(np.zeros(3), model, 2)
    np.testing.assert_allclose(j2[:, :2], j1[:, :2], atol=1e-8)
    np.testing.assert_allclose(j2[:, 2], j1[:, 2] / 2, atol=1e-8)
    duplicated = _measurement_model(walls, hits * 2, cfg, geo)
    obs1, obs2 = _observability(model, cfg, len(hits)), _observability(duplicated, cfg, 2 * len(hits))
    np.testing.assert_allclose(obs1["singular_values"], obs2["singular_values"], atol=1e-8)


def test_at_2_5m_level_sensors_miss_low_structures(default_result):
    scenes = scenarios(default_result)
    for key in ("low_structure_0_3m", "low_structure_1_1m"):
        assert scenes[key]["hit_count"] == 0
        assert scenes[key]["rank"] == 0
    # Even the ideal whole FoV, beyond the centres, cannot reach 1.1m at 3m.
    assert default_result["metrics"]["level_fov_envelope_min_height_m"] > 1.1
    tilted = scenarios(run_geometry({"mount_pitch_deg": -30, "monte_carlo_trials": 0}))
    assert tilted["low_structure_1_1m"]["hit_count"] > 0


def test_positive_pitch_points_up_and_deliberate_downward_case_hits_both():
    upward = scenarios(run_geometry({"mount_pitch_deg": 30.0, "monte_carlo_trials": 0}))
    for key in ("low_structure_0_3m", "low_structure_1_1m"):
        assert upward[key]["hit_count"] == 0 and upward[key]["rank"] == 0
    downward = scenarios(run_geometry({"height_m": 2.5, "mount_pitch_deg": -40.0,
                                      "max_range_m": 4.0, "monte_carlo_trials": 0}))
    assert downward["low_structure_0_3m"]["hit_count"] == 16
    assert downward["low_structure_1_1m"]["hit_count"] == 48
    assert downward["low_structure_0_3m"]["rank"] == 3
    assert downward["low_structure_1_1m"]["rank"] == 3


def test_json_determinism_and_public_contract(default_result):
    assert DEFAULT_CONFIG == {"height_m": 2.5, "max_range_m": 3.0, "mount_pitch_deg": 0.0,
                              "yaw_separation_deg": 90.0, "noise_mm": 30.0}
    assert default_result == run_geometry()
    json.dumps(default_result, ensure_ascii=False, allow_nan=False)
    assert isinstance(default_result["summary"], str)
    assert isinstance(default_result["metrics"], dict)
    assert all(isinstance(note, str) for note in default_result["assumptions"])
    assert all(scene["synthetic"] for scene in default_result["scenarios"])
    assert default_result["metrics"]["hardware_benchmark"] is False


def test_zero_noise_known_association_estimator_recovers_pose():
    result = scenarios(run_geometry({"noise_mm": 0, "monte_carlo_trials": 4}))
    mc = result["perpendicular_corner"]["monte_carlo"]
    assert mc["status"] == "ok" and mc["successful_trials"] == 4
    assert mc["translation_rmse_m"] < 1e-8 and mc["yaw_rmse_deg"] < 1e-6


def test_latency_is_geometric_error_not_pc_timing():
    result = scenarios(run_geometry({"noise_mm": 0, "latency_ms": 100, "velocity_x_m_s": 1,
                                     "monte_carlo_trials": 3}))
    corner = result["perpendicular_corner"]
    np.testing.assert_allclose(corner["scene"]["capture_poses_xy_yaw_rad"], [[-0.1, 0, 0], [-0.1, 0, 0]])
    mc = corner["monte_carlo"]
    assert mc["successful_trials"] == 3
    np.testing.assert_allclose(mc["mean_error_xy_yaw_deg"], [-0.1, 0, 0], atol=1e-6)


def test_shared_bias_does_not_average_away():
    corner = scenarios(run_geometry({"noise_mm": 0, "bias_mm": 50, "monte_carlo_trials": 3}))["perpendicular_corner"]
    np.testing.assert_allclose(corner["monte_carlo"]["mean_error_xy_yaw_deg"], [-0.05, -0.05, 0], atol=1e-6)


def test_datasheet_axis_alternative_and_radial_mode():
    result = run_geometry({"fov_model": "datasheet_axes", "distance_mode": "radial", "monte_carlo_trials": 0})
    assert result["metrics"]["horizontal_fov_deg"] == 45
    corner = scenarios(result)["perpendicular_corner"]
    assert corner["rank"] == 3
    assert all(hit["range_m"] == hit["radial_range_m"] for hit in corner["scene"]["hits"])
    json.dumps(result, allow_nan=False)


def test_gui_float_inputs_and_degenerate_json_are_finite():
    result = run_geometry({key: float(value) for key, value in DEFAULT_CONFIG.items()})
    json.dumps(result, ensure_ascii=False, allow_nan=False)
    assert scenarios(result)["single_wall"]["observability"]["condition_number"] is None
    assert any("正值朝上" in note for note in result["assumptions"])


def test_complete_shared_noise_preserves_json_and_rank():
    result = run_geometry({"noise_correlation": 1.0, "monte_carlo_trials": 4.0})
    json.dumps(result, allow_nan=False)
    assert scenarios(result)["perpendicular_corner"]["rank"] == 3
    assert scenarios(result)["perpendicular_corner"]["monte_carlo"]["successful_trials"] == 4


def test_float_seed_upper_bound_and_negative_measurements():
    # Seed+scenario remains valid for NumPy SeedSequence at the admitted limit.
    result = run_geometry({"seed": float(2**32 - 1), "bias_mm": -10000, "noise_mm": 0,
                           "monte_carlo_trials": 2})
    json.dumps(result, allow_nan=False)
    mc = scenarios(result)["perpendicular_corner"]["monte_carlo"]
    assert mc["successful_trials"] == 0 and mc["failed_trials"] == 2


@pytest.mark.parametrize("bad", [
    {"height_m": float("nan")}, {"noise_mm": float("inf")}, {"max_range_m": 0},
    {"noise_mm": -1}, {"noise_correlation": 1.1}, {"diagonal_fov_deg": 180},
    {"yaw_length_scale_m": 0}, {"monte_carlo_trials": 129}, {"seed": -1},
    {"monte_carlo_trials": 1.5}, {"distance_mode": "unknown"}, {"unrecognized": 1},
    {"velocity_x_m_s": 1e308}, {"mount_pitch_deg": 1e308}, {"yaw_length_scale_m": 1e-300},
])
def test_invalid_config_rejected(bad):
    with pytest.raises(ValueError):
        run_geometry(bad)


def test_import_and_run_preserve_defaults_and_global_rng(default_result):
    before = DEFAULT_CONFIG.copy()
    np.random.seed(9)
    expected = np.random.RandomState(9).normal(size=5)
    run_geometry({"noise_mm": 0, "monte_carlo_trials": 1})
    np.testing.assert_array_equal(np.random.normal(size=5), expected)
    assert DEFAULT_CONFIG == before
