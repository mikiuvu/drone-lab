"""Offline control readiness and failure policy reference.

Returns mode transitions, input evidence and control-readiness decisions.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from typing import Any, Iterable, Mapping


class Mode(str, Enum):
    STANDBY = "STANDBY"
    NAVIGATE = "NAVIGATE"
    DEGRADED = "DEGRADED"
    RECOVER = "RECOVER"
    TERMINATED = "TERMINATED"


FAILSAFE_REQUEST = "REQUEST_CONFIGURED_FC_FAILSAFE"
NO_CONTROL = "NO_CONTROL"
NAVIGATE_DECISION = "NAVIGATE_REFERENCE_ONLY"
TERMINATE_DECISION = "TERMINATE_REFERENCE_ONLY"

DEFAULT_CONFIG: dict[str, Any] = {
    "required_axes": ("x", "y", "z", "yaw"),
    "clock_confidence_min": 0.8,
    "odom_timeout_s": 0.25,
    "setpoint_timeout_s": 0.5,
    "recovery_samples": 3,
    "recovery_window_s": 1.0,
    "initial_mode": Mode.STANDBY.value,
}


@dataclass(frozen=True)
class Evidence:
    time_s: float
    estimator_epoch: str | int | None
    setpoint_epoch: str | int | None
    connection_alive: bool
    observable_axes: tuple[str, ...]
    position_valid: bool
    velocity_valid: bool
    yaw_valid: bool
    clock_confidence: float
    odom_time_s: float | None
    odom_frame_seq: int | None
    setpoint_time_s: float | None
    fresh_odom: bool
    fresh_setpoint: bool
    terminal_flags: tuple[str, ...]
    state_valid: bool
    reasons: tuple[str, ...]


@dataclass
class ControlPolicy:
    """Stateful policy evaluator with explicit, injected event times."""

    config: Mapping[str, Any] = field(default_factory=dict)
    mode: Mode = Mode.STANDBY
    estimator_epoch: str | int | None = None
    setpoint_epoch: str | int | None = None
    recovery_samples: int = 0
    recovery_started_odom_time_s: float | None = None
    last_valid_odom_time_s: float | None = None
    last_valid_epoch: str | int | None = None
    last_odom_time_s: float | None = None
    last_odom_frame_seq: int | None = None
    frame_seq_seen: bool = False
    transitions: int = 0
    epoch_resets: int = 0
    setpoint_epoch_resets: int = 0
    discarded_command_count: int = 0
    duplicate_samples: int = 0
    stale_samples: int = 0
    terminal_samples: int = 0
    invalid_samples: int = 0
    valid_samples: int = 0

    def __post_init__(self) -> None:
        self.config = _validate_config(self.config)
        self.mode = Mode(self.config["initial_mode"])
        if self.mode == Mode.TERMINATED:
            self.terminal_samples = 1

    def step(self, sample: Mapping[str, Any]) -> dict[str, Any]:
        evidence = _evidence(sample, self.config)
        previous = self.mode
        reasons: list[str] = list(evidence.reasons)
        estimator_changed = (
            self.estimator_epoch is not None
            and evidence.estimator_epoch != self.estimator_epoch
        )
        setpoint_changed = (
            self.setpoint_epoch is not None
            and evidence.setpoint_epoch != self.setpoint_epoch
        )
        if estimator_changed:
            self._reset_for_estimator_epoch(evidence.estimator_epoch)
            self.epoch_resets += 1
            reasons.append("estimator_epoch_changed_old_commands_discarded")
        if setpoint_changed:
            self._reset_for_setpoint_epoch(evidence.setpoint_epoch)
            self.setpoint_epoch_resets += 1
            reasons.append("setpoint_epoch_changed_old_commands_discarded")
        if estimator_changed or setpoint_changed:
            self.discarded_command_count += 1

        independent = self._is_independent(evidence, reasons)
        terminal = bool(evidence.terminal_flags)
        if terminal:
            self.mode = Mode.TERMINATED
            self.terminal_samples += 1
            reasons.append("terminal_sensor_flag")
        elif self.mode == Mode.TERMINATED:
            reasons.append("terminated_is_latched")
        elif estimator_changed or setpoint_changed:
            self.invalid_samples += 1
            self.recovery_samples = 0
            self.recovery_started_odom_time_s = None
            if self.mode in (Mode.NAVIGATE, Mode.RECOVER):
                self.mode = Mode.DEGRADED
            reasons.append("epoch_change_requires_fresh_independent_samples")
        elif not evidence.state_valid:
            self.invalid_samples += 1
            if not evidence.fresh_odom or not evidence.fresh_setpoint:
                self.stale_samples += 1
            self.recovery_samples = 0
            self.recovery_started_odom_time_s = None
            if self.mode in (Mode.NAVIGATE, Mode.RECOVER):
                self.mode = Mode.DEGRADED
            reasons.append("state_not_valid_for_navigation")
        elif not independent:
            self.invalid_samples += 1
            reasons.append("duplicate_odom_sample_not_counted_for_recovery")
        else:
            self.valid_samples += 1
            self._accept_valid_sample(evidence)
            self._advance_valid_state(evidence, reasons)

        if self.mode != previous:
            self.transitions += 1
        self.estimator_epoch = evidence.estimator_epoch
        self.setpoint_epoch = evidence.setpoint_epoch
        if evidence.state_valid and independent:
            self.last_valid_epoch = evidence.estimator_epoch
        return self._decision(evidence, previous, reasons)

    def _reset_for_estimator_epoch(self, epoch: str | int | None) -> None:
        self.estimator_epoch = epoch
        self._reset_sample_identity()

    def _reset_for_setpoint_epoch(self, epoch: str | int | None) -> None:
        self.setpoint_epoch = epoch
        self._reset_sample_identity()

    def _reset_sample_identity(self) -> None:
        self.recovery_samples = 0
        self.recovery_started_odom_time_s = None
        self.last_valid_odom_time_s = None
        self.last_valid_epoch = None
        self.last_odom_time_s = None
        self.last_odom_frame_seq = None
        self.frame_seq_seen = False

    def _is_independent(self, evidence: Evidence, reasons: list[str]) -> bool:
        if evidence.odom_time_s is None:
            return False
        if self.last_odom_time_s is not None and evidence.odom_time_s <= self.last_odom_time_s:
            self.duplicate_samples += 1
            reasons.append("odom_time_not_strictly_new")
            return False
        if evidence.odom_frame_seq is not None:
            if self.frame_seq_seen and (
                self.last_odom_frame_seq is None
                or evidence.odom_frame_seq <= self.last_odom_frame_seq
            ):
                self.duplicate_samples += 1
                reasons.append("odom_frame_seq_not_strictly_new")
                return False
        elif self.frame_seq_seen:
            self.duplicate_samples += 1
            reasons.append("odom_frame_seq_missing_after_seen")
            return False
        return True

    def _accept_valid_sample(self, evidence: Evidence) -> None:
        self.last_valid_odom_time_s = evidence.odom_time_s
        self.last_odom_time_s = evidence.odom_time_s
        if evidence.odom_frame_seq is not None:
            self.last_odom_frame_seq = evidence.odom_frame_seq
            self.frame_seq_seen = True

    def _advance_valid_state(self, evidence: Evidence, reasons: list[str]) -> None:
        if self.mode == Mode.STANDBY:
            self.mode = Mode.NAVIGATE
            reasons.append("initial_independent_valid_sample")
            return
        if self.mode == Mode.NAVIGATE:
            reasons.append("independent_valid_sample_maintains_navigation_readiness")
            return
        if self.mode == Mode.DEGRADED:
            odom_time = evidence.odom_time_s
            if odom_time is None:
                return
            if self.recovery_started_odom_time_s is None:
                self.recovery_started_odom_time_s = odom_time
            if odom_time - self.recovery_started_odom_time_s > float(self.config["recovery_window_s"]):
                self.recovery_samples = 1
                self.recovery_started_odom_time_s = odom_time
                reasons.append("recovery_window_restarted")
            else:
                self.recovery_samples += 1
            needed = int(self.config["recovery_samples"])
            reasons.append(f"independent_recovery_samples_{self.recovery_samples}_of_{needed}")
            if self.recovery_samples >= needed:
                self.mode = Mode.RECOVER
                reasons.append("recovery_hysteresis_satisfied")
            return
        if self.mode == Mode.RECOVER:
            self.mode = Mode.NAVIGATE
            self.recovery_samples = 0
            self.recovery_started_odom_time_s = None
            reasons.append("post_recovery_confirmation_sample")

    def _decision(self, evidence: Evidence, previous: Mode, reasons: list[str]) -> dict[str, Any]:
        if self.mode == Mode.TERMINATED:
            decision = TERMINATE_DECISION
            requested_action = FAILSAFE_REQUEST
            reasons.append("terminal_mode_latched")
        elif self.mode == Mode.NAVIGATE:
            decision = NAVIGATE_DECISION
            requested_action = "NONE"
        else:
            decision = NO_CONTROL
            requested_action = FAILSAFE_REQUEST
            reasons.append("no_position_based_control_reference")
            if "z" not in evidence.observable_axes or not evidence.position_valid:
                reasons.append("missing_altitude_does_not_imply_safe_landing")

        return {
            "time_s": evidence.time_s,
            "estimator_epoch": evidence.estimator_epoch,
            "setpoint_epoch": evidence.setpoint_epoch,
            "previous_mode": previous.value,
            "mode": self.mode.value,
            "decision": decision,
            "requested_action": requested_action,
            "connection_alive": evidence.connection_alive,
            "state_valid": evidence.state_valid,
            "fresh_odom": evidence.fresh_odom,
            "fresh_setpoint": evidence.fresh_setpoint,
            "terminal_sensor_flags": list(evidence.terminal_flags),
            "recovery_samples": self.recovery_samples,
            "discarded_command_count": self.discarded_command_count,
            "reasons": _unique(reasons),
            "evidence": {
                "observable_axes": list(evidence.observable_axes),
                "position_valid": evidence.position_valid,
                "velocity_valid": evidence.velocity_valid,
                "yaw_valid": evidence.yaw_valid,
                "clock_confidence": evidence.clock_confidence,
                "odom_time_s": evidence.odom_time_s,
                "odom_frame_seq": evidence.odom_frame_seq,
                "setpoint_time_s": evidence.setpoint_time_s,
            },
        }


def run_control(config: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Run deterministic offline evaluation over explicit-time samples."""

    if config is not None and not isinstance(config, Mapping):
        raise TypeError("config must be a mapping or None")
    options = dict(config or {})
    samples_provided = "samples" in options
    samples = options.pop("samples", None)
    if not samples_provided:
        samples = _default_fault_samples()
    if samples is None:
        raise TypeError("config['samples'] must be an iterable of sample mappings")
    if not isinstance(samples, Iterable) or isinstance(samples, (str, bytes, Mapping)):
        raise TypeError("config['samples'] must be an iterable of sample mappings")
    policy = ControlPolicy(options)
    decisions = [policy.step(sample) for sample in samples]
    final = decisions[-1] if decisions else None
    metrics = {
        "valid_samples": policy.valid_samples,
        "invalid_samples": policy.invalid_samples,
        "stale_samples": policy.stale_samples,
        "duplicate_samples": policy.duplicate_samples,
        "epoch_resets": policy.epoch_resets,
        "setpoint_epoch_resets": policy.setpoint_epoch_resets,
        "discarded_command_count": policy.discarded_command_count,
        "terminal_samples": policy.terminal_samples,
    }
    final_mode = policy.mode.value
    final_decision = final["decision"] if final else NO_CONTROL
    summary = (
        f"最终模式={final_mode}；决定={final_decision}；样本数={len(decisions)}；"
        f"有效={metrics['valid_samples']}；无效={metrics['invalid_samples']}；"
        f"重复={metrics['duplicate_samples']}；终止锁存={'是' if policy.mode == Mode.TERMINATED else '否'}"
    )
    return {
        "summary": summary,
        "decisions": decisions,
        "metrics": metrics,
        "assumptions": [
            "离线状态机输出模式、处理意图和准入原因；MAVLink 由调用方接入。",
            "NAVIGATE_REFERENCE_ONLY 表示输入满足控制器的数据准入条件。",
            "连接生命与估计状态有效性分开判断。",
            "恢复只计入同一 epoch 内严格递增的 odom_time_s；若提供 odom_frame_seq，它也必须严格递增。",
            "fresh_odom/fresh_setpoint 不能绕过来源时间的新鲜度检查。",
            "高度失效时，实际动作由飞控配置的失效保护决定。",
            "UWB 输入需要独立时间、epoch 或时钟映射、协方差、年龄上限和健康标志。",
            "clock_confidence 为外部时间同步模块提供的准入指标。",
        ],
    }


def _default_fault_samples() -> list[dict[str, Any]]:
    """Provide a deterministic GUI demonstration of safety transitions."""
    def valid(time_s: float, odom_time_s: float, *, epoch: int = 1, **extra: Any) -> dict[str, Any]:
        item: dict[str, Any] = {
            "time_s": time_s,
            "estimator_epoch": epoch,
            "setpoint_epoch": epoch,
            "connection_alive": True,
            "observable_axes": ["x", "y", "z", "yaw"],
            "position_valid": True,
            "velocity_valid": True,
            "yaw_valid": True,
            "clock_confidence": 1.0,
            "odom_time_s": odom_time_s,
            "setpoint_time_s": time_s,
            "odom_frame_seq": max(0, int(round(odom_time_s * 100.0))),
            "terminal_sensor_flags": [],
        }
        item.update(extra)
        return item

    return [
        valid(0.0, 0.0),
        valid(0.1, -1.0),
        valid(0.2, 0.2),
        valid(0.3, 0.3),
        valid(0.4, 0.4),
        valid(0.5, 0.4),
        valid(0.6, 0.5),
        valid(0.7, 0.6, epoch=2),
        valid(0.8, 0.7, epoch=2, observable_axes=["x", "y", "yaw"]),
        valid(0.9, 0.8, epoch=2, terminal_sensor_flags=["critical_battery"]),
    ]


def _validate_config(config: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(config, Mapping):
        raise TypeError("config must be a mapping")
    merged = dict(DEFAULT_CONFIG)
    merged.update(dict(config))
    axes = merged["required_axes"]
    if isinstance(axes, (str, bytes)) or not isinstance(axes, Iterable):
        raise ValueError("required_axes must be a non-empty iterable of strings")
    axes = tuple(str(axis) for axis in axes)
    if not axes or any(not axis or axis not in {"x", "y", "z", "yaw"} for axis in axes):
        raise ValueError("required_axes must contain only x, y, z, yaw")
    merged["required_axes"] = tuple(dict.fromkeys(axes))
    confidence = _bounded_number(merged["clock_confidence_min"], "clock_confidence_min", 0.0, 1.0)
    odom_timeout = _positive_number(merged["odom_timeout_s"], "odom_timeout_s")
    setpoint_timeout = _positive_number(merged["setpoint_timeout_s"], "setpoint_timeout_s")
    recovery_window = _positive_number(merged["recovery_window_s"], "recovery_window_s")
    recovery_samples = merged["recovery_samples"]
    if isinstance(recovery_samples, bool) or not isinstance(recovery_samples, int) or recovery_samples < 1 or recovery_samples > 1000000:
        raise ValueError("recovery_samples must be an integer in [1, 1000000]")
    try:
        Mode(merged["initial_mode"])
    except (TypeError, ValueError) as exc:
        raise ValueError("initial_mode must be a known control mode") from exc
    merged.update(
        clock_confidence_min=confidence,
        odom_timeout_s=odom_timeout,
        setpoint_timeout_s=setpoint_timeout,
        recovery_window_s=recovery_window,
    )
    return merged


def _evidence(sample: Mapping[str, Any], config: Mapping[str, Any]) -> Evidence:
    if not isinstance(sample, Mapping):
        raise TypeError("each control sample must be a mapping")
    time_s = _finite_number(sample.get("time_s"), "time_s")
    estimator_epoch = _epoch(sample.get("estimator_epoch"), "estimator_epoch")
    setpoint_epoch = _epoch(sample.get("setpoint_epoch"), "setpoint_epoch")
    connection_alive = _strict_bool(sample.get("connection_alive", False), "connection_alive")
    axes = _axes(sample.get("observable_axes", ()))
    position_valid = _strict_bool(sample.get("position_valid", False), "position_valid")
    velocity_valid = _strict_bool(sample.get("velocity_valid", False), "velocity_valid")
    yaw_valid = _strict_bool(sample.get("yaw_valid", False), "yaw_valid")
    clock_confidence = _finite_number(sample.get("clock_confidence", 0.0), "clock_confidence")
    if not 0.0 <= clock_confidence <= 1.0:
        raise ValueError("clock_confidence must be in [0, 1]")
    odom_time_s = _optional_finite_number(sample.get("odom_time_s"), "odom_time_s")
    setpoint_time_s = _optional_finite_number(sample.get("setpoint_time_s"), "setpoint_time_s")
    odom_frame_seq = _optional_nonnegative_int(sample.get("odom_frame_seq"), "odom_frame_seq")
    fresh_odom = _fresh_from_time(time_s, odom_time_s, float(config["odom_timeout_s"]))
    fresh_setpoint = _fresh_from_time(time_s, setpoint_time_s, float(config["setpoint_timeout_s"]))
    reasons: list[str] = []
    if not connection_alive:
        reasons.append("connection_not_alive")
    if estimator_epoch is None:
        reasons.append("estimator_epoch_missing")
    if setpoint_epoch is None:
        reasons.append("setpoint_epoch_missing")
    elif estimator_epoch is not None and setpoint_epoch != estimator_epoch:
        reasons.append("setpoint_epoch_mismatch")
    if not set(config["required_axes"]).issubset(set(axes)):
        reasons.append("required_observable_axis_missing")
    if not position_valid:
        reasons.append("position_invalid")
    if not velocity_valid:
        reasons.append("velocity_invalid")
    if not yaw_valid:
        reasons.append("yaw_invalid")
    if clock_confidence < float(config["clock_confidence_min"]):
        reasons.append("clock_confidence_below_threshold")
    if not fresh_odom:
        reasons.append("odom_stale_or_missing")
    if not fresh_setpoint:
        reasons.append("setpoint_stale_or_missing")
    for kind, derived in (("odom", fresh_odom), ("setpoint", fresh_setpoint)):
        claim_name = f"fresh_{kind}"
        if claim_name in sample:
            claim = _strict_bool(sample[claim_name], claim_name)
            if claim != derived:
                reasons.append(f"{claim_name}_claim_mismatch")
    terminal_flags = _terminal_flags(sample.get("terminal_sensor_flags", ()))
    state_valid = (
        connection_alive
        and estimator_epoch is not None
        and setpoint_epoch is not None
        and estimator_epoch == setpoint_epoch
        and set(config["required_axes"]).issubset(set(axes))
        and position_valid
        and velocity_valid
        and yaw_valid
        and clock_confidence >= float(config["clock_confidence_min"])
        and fresh_odom
        and fresh_setpoint
        and not terminal_flags
        and not any(reason.endswith("_claim_mismatch") for reason in reasons)
    )
    return Evidence(
        time_s=time_s,
        estimator_epoch=estimator_epoch,
        setpoint_epoch=setpoint_epoch,
        connection_alive=connection_alive,
        observable_axes=tuple(sorted(axes)),
        position_valid=position_valid,
        velocity_valid=velocity_valid,
        yaw_valid=yaw_valid,
        clock_confidence=clock_confidence,
        odom_time_s=odom_time_s,
        odom_frame_seq=odom_frame_seq,
        setpoint_time_s=setpoint_time_s,
        fresh_odom=fresh_odom,
        fresh_setpoint=fresh_setpoint,
        terminal_flags=terminal_flags,
        state_valid=state_valid,
        reasons=tuple(reasons),
    )


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _optional_finite_number(value: Any, name: str) -> float | None:
    if value is None:
        return None
    return _finite_number(value, name)


def _bounded_number(value: Any, name: str, lower: float, upper: float) -> float:
    number = _finite_number(value, name)
    if number < lower or number > upper:
        raise ValueError(f"{name} must be in [{lower}, {upper}]")
    return number


def _positive_number(value: Any, name: str) -> float:
    number = _finite_number(value, name)
    if number <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return number


def _strict_bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a boolean")
    return value


def _epoch(value: Any, name: str) -> str | int | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        raise ValueError(f"{name} must be a non-empty string or integer")
    if isinstance(value, str) and not value.strip():
        raise ValueError(f"{name} must be a non-empty string or integer")
    return value


def _optional_nonnegative_int(value: Any, name: str) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _axes(value: Any) -> tuple[str, ...]:
    if isinstance(value, Mapping):
        return tuple(str(key) for key, enabled in value.items() if _strict_bool(enabled, "observable_axes flag"))
    if isinstance(value, (str, bytes)) or not isinstance(value, Iterable):
        raise ValueError("observable_axes must be an iterable or boolean mapping")
    axes = tuple(str(axis) for axis in value)
    if any(not axis for axis in axes):
        raise ValueError("observable_axes cannot contain empty names")
    return axes


def _fresh_from_time(now_s: float, source_s: float | None, timeout_s: float) -> bool:
    return source_s is not None and 0.0 <= now_s - source_s <= timeout_s


def _terminal_flags(value: Any) -> tuple[str, ...]:
    if isinstance(value, Mapping):
        flags = []
        for key, active in value.items():
            if _strict_bool(active, "terminal sensor flag") and not str(key):
                raise ValueError("terminal sensor flag names cannot be empty")
            if active:
                flags.append(str(key))
        return tuple(sorted(flags))
    if isinstance(value, (str, bytes)) or not isinstance(value, Iterable):
        raise ValueError("terminal_sensor_flags must be an iterable or boolean mapping")
    flags = tuple(str(flag) for flag in value)
    if any(not flag for flag in flags):
        raise ValueError("terminal sensor flag names cannot be empty")
    return tuple(sorted(flags))


def _unique(values: Iterable[str]) -> list[str]:
    return list(dict.fromkeys(values))
