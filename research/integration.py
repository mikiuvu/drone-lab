"""Executable interface checks for a future localization -> flight-controller link.

All functions are offline. No serial transport, arming, or setpoint publishing.
"""

from dataclasses import dataclass
import math
import numpy as np

ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def convert_pose_enu_flu(position, body_to_world):
    """Return NED position and FRD-to-NED rotation, with explicit frames."""
    p = np.asarray(position, dtype=float)
    rotation = np.asarray(body_to_world, dtype=float)
    if p.shape != (3,) or rotation.shape != (3, 3):
        raise ValueError("Position must have 3 elements; rotation must be 3x3")
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(rotation)):
        raise ValueError("Pose contains a non-finite number")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
        np.linalg.det(rotation), 1
    ):
        raise ValueError("Input must be a proper body-to-world rotation")
    return ENU_TO_NED @ p, ENU_TO_NED @ rotation @ FLU_TO_FRD


def convert_position_covariance(covariance):
    """3x3 position covariance only; attitude-error conventions require their own Jacobian."""
    p = np.asarray(covariance, dtype=float)
    if p.shape != (3, 3) or not np.all(np.isfinite(p)) or not np.allclose(p, p.T):
        raise ValueError("Covariance must be finite, symmetric and 3x3")
    if np.min(np.linalg.eigvalsh(p)) < -1e-9:
        raise ValueError("Covariance must be positive semidefinite")
    return ENU_TO_NED @ p @ ENU_TO_NED.T


@dataclass(frozen=True)
class ClockSample:
    local_send_ns: int
    remote_reply_ns: int
    local_receive_ns: int

    @property
    def roundtrip_ns(self):
        return self.local_receive_ns - self.local_send_ns

    @property
    def offset_ns(self):
        # remote clock = local clock + offset; response processing is part of RTT.
        return self.remote_reply_ns - (self.local_send_ns + self.local_receive_ns) / 2


def clock_offset(samples, maximum_rtt_ms=20):
    """Estimate clock offset from minimum RTT assuming symmetric path delays."""
    accepted = [s for s in samples if 0 <= s.roundtrip_ns <= maximum_rtt_ms * 1e6]
    if not accepted:
        raise ValueError("No valid clock sample within RTT limit")
    best = min(accepted, key=lambda s: s.roundtrip_ns)
    return {
        "offset_ns": best.offset_ns,
        "rtt_ns": best.roundtrip_ns,
        "uncertainty_bound_ns": best.roundtrip_ns / 2,
        "accepted": len(accepted),
    }


def admit_estimate(
    *,
    now_us,
    capture_us,
    epoch,
    expected_epoch,
    covariance,
    observable,
    clock_uncertainty_us,
    maximum_age_us=150000,
):
    """Check timestamp, epoch, covariance and geometry before admitting an estimate."""
    values = [now_us, capture_us, clock_uncertainty_us, maximum_age_us]
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        return False, "nonfinite_time"
    if epoch != expected_epoch:
        return False, "clock_or_estimator_reset"
    if clock_uncertainty_us < 0 or clock_uncertainty_us > 5000:
        return False, "clock_uncertain"
    if capture_us > now_us:
        return False, "future_timestamp"
    if maximum_age_us <= 0 or now_us - capture_us > maximum_age_us:
        return False, "stale"
    if not observable:
        return False, "geometry_degenerate"
    try:
        convert_position_covariance(covariance)
    except ValueError:
        return False, "invalid_covariance"
    if np.min(np.diag(covariance)) <= 0:
        return False, "zero_variance_claim"
    return True, "accepted_for_offline_evaluation"


DEFAULT_CONFIG = {
    "speed_mps": 1.0,
    "yaw_rate_deg_s": 90.0,
    "range_m": 3.0,
    "delay_ms": 50.0,
}


def run_integration(config=None):
    cfg = DEFAULT_CONFIG | (config or {})
    if any(
        not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0
        for v in cfg.values()
    ):
        raise ValueError("Parameters must be nonnegative finite numbers")
    delay = cfg["delay_ms"] / 1000
    angle = math.radians(cfg["yaw_rate_deg_s"]) * delay
    angular_position_error = 2 * cfg["range_m"] * abs(math.sin(angle / 2))
    positional_age_error = cfg["speed_mps"] * delay
    samples = [
        ClockSample(1000000000, 1042000000, 1004000000),
        ClockSample(2000000000, 2048000000, 2010000000),
    ]
    sync = clock_offset(samples)
    base = dict(
        now_us=1000000,
        capture_us=950000,
        epoch=1,
        expected_epoch=1,
        covariance=np.diag([0.01, 0.01, 0.04]),
        observable=True,
        clock_uncertainty_us=2000,
    )
    cases = {
        "fresh": base,
        "stale": base | {"capture_us": 600000},
        "reboot": base | {"epoch": 2},
        "single_wall": base | {"observable": False},
        "future": base | {"capture_us": 1000001},
        "unknown_clock": base | {"clock_uncertainty_us": 40000},
    }
    gates = {
        name: {"accepted": (answer := admit_estimate(**case))[0], "reason": answer[1]}
        for name, case in cases.items()
    }
    return {
        "summary": f"{cfg['delay_ms']:g} ms 时间错配：平移约 {positional_age_error * 100:.1f} cm；"
        f"{cfg['range_m']:g} m 处旋转点位误差约 {angular_position_error * 100:.1f} cm。",
        "metrics": {
            "translation_age_error_m": positional_age_error,
            "rotation_chord_error_m": angular_position_error,
            "symmetric_offset_ms": sync["offset_ns"] / 1e6,
            "clock_uncertainty_bound_ms": sync["uncertainty_bound_ns"] / 1e6,
        },
        "config": cfg,
        "admission_cases": gates,
        "assumptions": [
            "匀速平移与恒定角速度解析模型；两类误差不可简单按独立高斯量合并。",
            "TIMESYNC 使用合成纳秒时间样本，估计依赖近似对称传输。",
            "示例门槛：数据年龄 150 ms，时钟不确定度 5 ms。",
            "坐标转换为 ENU/NED、FLU/FRD；模块输出转换结果与准入判断。",
            "2.5D 模型的高度为外部输入。",
        ],
    }
