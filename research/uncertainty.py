"""@file uncertainty.py
@brief 离线传播随机时间抖动、融合未知相关的位置估计，并拟合带预算的时钟映射。
"""

from dataclasses import dataclass
import math
import numpy as np
from .integration import ClockSample, convert_position_covariance


def timing_position_covariance(velocity, angular_rate, lever_arm, sigma_seconds):
    """@brief 将随机时间抖动的一阶敏感性传播成位置协方差。
    @param velocity 世界系线速度，m/s，三元素。
    @param angular_rate 同一世界系角速度，rad/s，三元素。
    @param lever_arm 同一世界系杠杆臂，m，三元素。
    @param sigma_seconds 随机时间抖动标准差，s，非负。
    @return (v+ω×r)(v+ω×r)ᵀ·σ²，3×3 协方差，m²。
    @note RTT/2 或未标定固定延迟界不是随机标准差，不能直接代入。
    """
    vectors = [np.asarray(v, dtype=float) for v in (velocity, angular_rate, lever_arm)]
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in vectors):
        raise ValueError("Expected finite 3-vectors in a common frame")
    if not math.isfinite(sigma_seconds) or sigma_seconds < 0:
        raise ValueError("Timing sigma must be finite and nonnegative")
    sensitivity = vectors[0] + np.cross(vectors[1], vectors[2])
    return np.outer(sensitivity, sensitivity) * sigma_seconds**2


def covariance_intersection(mean_a, covariance_a, mean_b, covariance_b, weight=0.5):
    """@brief 用协方差交集在未知互相关时融合两个三维位置高斯估计。
    @param mean_a 第一组位置均值，m。
    @param covariance_a 第一组正定位置协方差，m²。
    @param mean_b 第二组位置均值，m。
    @param covariance_b 第二组正定位置协方差，m²。
    @param weight 信息矩阵权重，范围 [0,1]。
    @return 融合均值与协方差。
    @note 输入估计各自须一致；该方法不修复未建模的共同偏差，也不处理姿态或 epoch。
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
    """@brief 带有效期、代次和确定性误差预算的仿射映射：remote=remote_ref+rate·(local-local_ref)。
    @note 调用者负责往返配对及重置检测，rate 是无量纲时钟速率比。
    """

    local_reference_ns: float  # 本地拟合参考时间，ns。
    remote_reference_ns: float  # 远端拟合参考时间，ns。
    rate: float  # 远端/本地时钟速率比，无量纲。
    epoch: str  # 重置代次，拒绝跨代次映射。
    valid_from_ns: int  # 本地有效期起点，ns。
    valid_until_ns: int  # 本地有效期终点，ns。
    observed_residual_bound_ns: float  # 最大观测残差，ns。
    transport_bound_ns: float  # 半 RTT 传输预算，ns。
    drift_bound_ppm: float  # 外推漂移预算，ppm。

    def remote_to_local(self, remote_ns, *, now_local_ns, epoch):
        """@brief 将远端 ns 时间映射到本地，并检查代次、有效期及未来时间。
        @param remote_ns 远端时间，ns。
        @param now_local_ns 当前本地时间，ns。
        @param epoch 当前代次，须匹配拟合代次。
        @return local_ns、uncertainty_bound_ns、epoch 和预算类型；不合法输入抛 ValueError。
        @note 预算为观测残差界+传输界+有效期起点后的漂移增长，不是置信区间。
        """
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
    """@brief 筛选配对样本、以秒尺度最小二乘拟合时钟速率和偏移。
    @param samples 至少三个严格递增的 ClockSample，覆盖至少 0.1 s。
    @param epoch 非空代次标识。
    @param valid_for_s 最后一次接收后的有效时长，s。
    @param max_rtt_ms 最大允许 RTT，ms。
    @param drift_bound_ppm 漂移预算，ppm，同时限制拟合速率偏差。
    @return ClockMapping，包含最大拟合残差与最大半 RTT 的误差预算。
    @note 时延不对称可偏置斜率和偏移；频率变化或重置后应失效并重拟合。
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
    # 减参考点后转换为秒，降低大纳秒时间戳对最小二乘数值精度的影响。
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
