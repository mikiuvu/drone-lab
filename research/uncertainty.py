"""Clock-mapping and uncertainty-propagation contracts for offline analysis."""

from dataclasses import dataclass
import math
import numpy as np
from .integration import ClockSample, convert_position_covariance


def timing_position_covariance(velocity, angular_rate, lever_arm, sigma_seconds):
    """First-order time sensitivity: dp/dt = v + omega cross r.

    All vectors in one world frame; sigma_seconds is random jitter standard
    deviation, NOT RTT/2 or an uncalibrated systematic delay bound.
    """
    vectors = [np.asarray(v, dtype=float) for v in (velocity, angular_rate, lever_arm)]
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in vectors):
        raise ValueError("Expected finite 3-vectors in a common frame")
    if not math.isfinite(sigma_seconds) or sigma_seconds < 0:
        raise ValueError("Timing sigma must be finite and nonnegative")
    sensitivity = vectors[0] + np.cross(vectors[1], vectors[2])
    return np.outer(sensitivity, sensitivity) * sigma_seconds**2


def covariance_intersection(mean_a, covariance_a, mean_b, covariance_b, weight=0.5):
    """Conservative Gaussian fusion reference for unknown cross-correlation.

    Inputs must be individually consistent; CI does not fix unmodeled common bias.
    This 3D position-only function intentionally has no attitude or epoch handling.
    """
    a, b = np.asarray(mean_a, dtype=float), np.asarray(mean_b, dtype=float)
    if (
        a.shape != (3,)
        or b.shape != (3,)
        or not np.isfinite(a).all()
        or not np.isfinite(b).all()
    ):
        raise ValueError("Expected finite position means")
    if not math.isfinite(weight) or not 0 <= weight <= 1:
        raise ValueError("Weight must be in [0,1]")
    pa, pb = (
        np.asarray(covariance_a, dtype=float),
        np.asarray(covariance_b, dtype=float),
    )
    for covariance in (pa, pb):
        convert_position_covariance(covariance)
        if np.linalg.eigvalsh(covariance).min() <= 0:
            raise ValueError("CI requires positive definite covariances")
    ia, ib = np.linalg.inv(pa), np.linalg.inv(pb)
    covariance = np.linalg.inv(weight * ia + (1 - weight) * ib)
    mean = covariance @ (weight * ia @ a + (1 - weight) * ib @ b)
    return mean, covariance


@dataclass(frozen=True)
class ClockMapping:
    """Affine offline fit; the caller owns response pairing and reset detection."""

    local_reference_ns: float
    remote_reference_ns: float
    rate: float
    epoch: str
    valid_from_ns: int
    valid_until_ns: int
    observed_residual_bound_ns: float
    transport_bound_ns: float
    drift_bound_ppm: float

    def remote_to_local(self, remote_ns, *, now_local_ns, epoch):
        if epoch != self.epoch:
            raise ValueError("Clock epoch changed")
        if not all(
            isinstance(v, (int, float)) and math.isfinite(v)
            for v in (remote_ns, now_local_ns)
        ):
            raise ValueError("Invalid clock values")
        if not self.valid_from_ns <= now_local_ns <= self.valid_until_ns:
            raise ValueError("Clock mapping expired or not yet valid")
        local = (
            self.local_reference_ns + (remote_ns - self.remote_reference_ns) / self.rate
        )
        age = max(0, now_local_ns - self.valid_from_ns)
        bound = (
            self.observed_residual_bound_ns
            + self.transport_bound_ns
            + age * self.drift_bound_ppm * 1e-6
        )
        if local > now_local_ns + bound:
            raise ValueError("Mapped timestamp is in the future")
        return {
            "local_ns": local,
            "uncertainty_bound_ns": bound,
            "epoch": epoch,
            "uncertainty_kind": "deterministic_budget_not_gaussian_sigma",
        }


def fit_clock_mapping(
    samples, *, epoch, valid_for_s=2.0, max_rtt_ms=20.0, drift_bound_ppm=100.0
):
    """Fit paired clock samples, with explicit transport and extrapolation budgets.

    RTT asymmetry can bias offset and slope. Bounds are assumptions, not inferred
    confidence intervals. Clock frequency changes require refitting/invalidation.
    """
    if not isinstance(epoch, str) or not epoch:
        raise ValueError("Nonempty epoch required")
    params = [valid_for_s, max_rtt_ms, drift_bound_ppm]
    if not all(
        isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in params
    ):
        raise ValueError("Positive finite validity, RTT and drift budget required")
    selected = []
    for s in samples:
        if not isinstance(s, ClockSample):
            raise ValueError("ClockSample required")
        values = [s.local_send_ns, s.local_receive_ns, s.remote_reply_ns]
        if not all(isinstance(v, int) and v >= 0 for v in values):
            raise ValueError("Nonnegative integer nanoseconds required")
        if 0 <= s.roundtrip_ns <= max_rtt_ms * 1e6:
            selected.append(s)
    if len(selected) < 3:
        raise ValueError("At least three valid paired samples required")
    mid = np.array([(s.local_send_ns + s.local_receive_ns) / 2 for s in selected])
    remote = np.array([s.remote_reply_ns for s in selected], dtype=float)
    if np.any(np.diff(mid) <= 0) or np.any(np.diff(remote) <= 0):
        raise ValueError("Clock samples must advance; reset invalidates fit")
    origin = mid[0]
    remote_origin = remote[0]
    x = (mid - origin) / 1e9
    y = (remote - remote_origin) / 1e9
    if x[-1] < 0.1:
        raise ValueError("Insufficient sample time span")
    rate, intercept = np.linalg.lstsq(
        np.column_stack([x, np.ones_like(x)]), y, rcond=None
    )[0]
    if not math.isfinite(rate) or abs(rate - 1) > drift_bound_ppm * 1e-6:
        raise ValueError("Fitted clock drift exceeds configured budget")
    residual = float(np.max(np.abs(y - (rate * x + intercept)))) * 1e9
    anchor = int(selected[-1].local_receive_ns)
    return ClockMapping(
        float(origin),
        float(remote_origin + intercept * 1e9),
        float(rate),
        epoch,
        anchor,
        anchor + int(valid_for_s * 1e9),
        residual,
        max(s.roundtrip_ns / 2 for s in selected),
        drift_bound_ppm,
    )
