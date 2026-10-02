import numpy as np
import pytest
from research.integration import ClockSample
from research.uncertainty import (
    timing_position_covariance,
    covariance_intersection,
    fit_clock_mapping,
)


def test_clock_jitter_rotational_lever_arm():
    covariance = timing_position_covariance([1, 0, 0], [0, 0, 1], [3, 0, 0], 0.005)
    assert np.allclose(covariance, np.outer([1, 3, 0], [1, 3, 0]) * 0.005**2)
    assert covariance[0, 1] > 0
    assert np.linalg.eigvalsh(covariance).min() > -1e-12


def test_fusing_same_information_must_not_halve_variance():
    mean, covariance = covariance_intersection(
        [1, 2, 3], np.eye(3), [1, 2, 3], np.eye(3)
    )
    assert np.allclose(mean, [1, 2, 3]) and np.allclose(covariance, np.eye(3))
    with pytest.raises(ValueError):
        covariance_intersection([0, 0, 0], np.zeros((3, 3)), [0, 0, 0], np.eye(3))


def test_clock_drift_expiration_and_epoch():
    rate = 1.00005
    offset = 40_000_000
    samples = [
        ClockSample(t, int((t + 2_000_000) * rate + offset), t + 4_000_000)
        for t in range(1_000_000_000, 5_000_000_000, 1_000_000_000)
    ]
    mapping = fit_clock_mapping(samples, epoch="boot-A")
    assert abs(mapping.rate - rate) < 1e-10
    stamp = 5_000_000_000
    result = mapping.remote_to_local(
        int(stamp * rate + offset), now_local_ns=stamp + 1_000_000, epoch="boot-A"
    )
    assert abs(result["local_ns"] - stamp) < 1
    with pytest.raises(ValueError):
        mapping.remote_to_local(1, now_local_ns=stamp, epoch="boot-B")
    with pytest.raises(ValueError):
        mapping.remote_to_local(1, now_local_ns=10_000_000_000, epoch="boot-A")


def test_asymmetric_transport_bound_and_reset():
    samples = [
        ClockSample(t, t + 8_000_000 + 40_000_000, t + 10_000_000)
        for t in range(1_000_000_000, 4_000_000_000, 1_000_000_000)
    ]
    mapping = fit_clock_mapping(samples, epoch="A")
    result = mapping.remote_to_local(
        4_040_000_000, now_local_ns=4_005_000_000, epoch="A"
    )
    assert abs(result["local_ns"] - 4_000_000_000) <= result["uncertainty_bound_ns"]
    samples[-1] = ClockSample(3_000_000_000, 1000, 3_010_000_000)
    with pytest.raises(ValueError, match="reset"):
        fit_clock_mapping(samples, epoch="A")
