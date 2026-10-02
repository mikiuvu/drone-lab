import numpy as np
import pytest
from research.integration import (
    ClockSample,
    clock_offset,
    convert_pose_enu_flu,
    convert_position_covariance,
    admit_estimate,
    run_integration,
)


def test_axis_conversion_preserves_rotations():
    p, r = convert_pose_enu_flu([1, 2, 3], np.eye(3))
    assert p.tolist() == [2, 1, -3]
    assert np.allclose(r @ np.array([1, 0, 0]), [0, 1, 0])
    assert np.allclose(r.T @ r, np.eye(3)) and np.isclose(np.linalg.det(r), 1)
    assert np.allclose(
        convert_position_covariance(np.diag([1, 4, 9])), np.diag([4, 1, 9])
    )


def test_bad_rotation_and_covariance_rejected():
    with pytest.raises(ValueError):
        convert_pose_enu_flu([0, 0, 0], np.diag([1, 1, -1]))
    with pytest.raises(ValueError):
        convert_position_covariance(np.diag([1, -1, 1]))


def test_clock_path_asymmetry_is_not_magic():
    # True offset=40 ms; forward=8 ms, reverse=2 ms => estimated 43 ms.
    sample = ClockSample(1_000_000_000, 1_048_000_000, 1_010_000_000)
    result = clock_offset([sample])
    assert result["offset_ns"] == 43_000_000
    assert result["uncertainty_bound_ns"] == 5_000_000
    with pytest.raises(ValueError):
        clock_offset([ClockSample(10, 0, 9)])


def test_stale_future_reset_and_unobservability_gate():
    result = run_integration()
    assert result["admission_cases"]["fresh"]["accepted"]
    assert sum(v["accepted"] for v in result["admission_cases"].values()) == 1
    assert result["metrics"]["translation_age_error_m"] == 0.05
    assert result["metrics"]["rotation_chord_error_m"] > 0.23


def test_no_zero_covariance_shortcut():
    ok, reason = admit_estimate(
        now_us=10,
        capture_us=5,
        epoch=1,
        expected_epoch=1,
        covariance=np.zeros((3, 3)),
        observable=True,
        clock_uncertainty_us=1,
    )
    assert not ok and reason == "zero_variance_claim"
