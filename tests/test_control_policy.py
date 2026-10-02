
import pytest

from research.control_policy import (
    FAILSAFE_REQUEST,
    NAVIGATE_DECISION,
    NO_CONTROL,
    TERMINATE_DECISION,
    Mode,
    run_control,
)


AXES = ["x", "y", "z", "yaw"]


def sample(t, *, odom_t=None, setpoint_t=None, seq=None, **overrides):
    value = {
        "time_s": t,
        "estimator_epoch": 7,
        "setpoint_epoch": 7,
        "connection_alive": True,
        "observable_axes": AXES,
        "position_valid": True,
        "velocity_valid": True,
        "yaw_valid": True,
        "clock_confidence": 1.0,
        "odom_time_s": t if odom_t is None else odom_t,
        "setpoint_time_s": t if setpoint_t is None else setpoint_t,
        "terminal_sensor_flags": [],
    }
    if seq is not None:
        value["odom_frame_seq"] = seq
    value.update(overrides)
    return value


def test_default_is_deterministic_chinese_summary_and_no_command():
    first = run_control()
    second = run_control()
    assert first == second
    assert isinstance(first["summary"], str)
    assert "最终模式=TERMINATED" in first["summary"]
    assert "样本数=10" in first["summary"]
    assert isinstance(first["metrics"], dict)
    assert first["summary"] and "MAVLink" in first["assumptions"][0]


def test_explicit_empty_samples_remain_empty_standby():
    result = run_control({"samples": []})
    assert result["summary"] == "最终模式=STANDBY；决定=NO_CONTROL；样本数=0；有效=0；无效=0；重复=0；终止锁存=否"
    assert result["decisions"] == []


def test_default_fault_sequence_covers_gui_contract():
    result = run_control({})
    assert [item["mode"] for item in result["decisions"]] == [
        "NAVIGATE", "DEGRADED", "DEGRADED", "DEGRADED", "RECOVER",
        "RECOVER", "NAVIGATE", "DEGRADED", "DEGRADED", "TERMINATED",
    ]
    assert result["metrics"]["duplicate_samples"] == 2
    assert result["metrics"]["epoch_resets"] == 1
    assert result["metrics"]["setpoint_epoch_resets"] == 1
    assert result["metrics"]["discarded_command_count"] == 1
    assert "missing_altitude_does_not_imply_safe_landing" in result["decisions"][8]["reasons"]


def test_stale_odom_degrades_and_requests_configured_failsafe():
    result = run_control({"samples": [sample(0), sample(0.1, odom_t=-1.0)]})
    decision = result["decisions"][-1]
    assert decision["mode"] == Mode.DEGRADED.value
    assert decision["connection_alive"] is True
    assert decision["state_valid"] is False
    assert decision["requested_action"] == FAILSAFE_REQUEST
    assert "odom_stale_or_missing" in decision["reasons"]


def test_connection_loss_is_separate_from_invalid_estimator_state():
    result = run_control({"samples": [sample(0), sample(0.1, connection_alive=False)]})
    decision = result["decisions"][-1]
    assert decision["connection_alive"] is False
    assert decision["state_valid"] is False
    assert "connection_not_alive" in decision["reasons"]
    assert "position_invalid" not in decision["reasons"]


def test_repeated_odom_timestamp_cannot_satisfy_recovery_even_when_time_advances():
    events = [sample(0), sample(0.1, odom_t=0.1), sample(0.2, odom_t=0.2)]
    events.extend(sample(t, odom_t=0.2) for t in (0.3, 0.4, 0.5, 0.6))
    result = run_control({"samples": events})
    modes = [item["mode"] for item in result["decisions"]]
    assert modes == ["NAVIGATE", "NAVIGATE", "NAVIGATE", "NAVIGATE", "NAVIGATE", "DEGRADED", "DEGRADED"]
    assert result["metrics"]["duplicate_samples"] == 4
    assert all("odom_time_not_strictly_new" in item["reasons"] for item in result["decisions"][3:])


def test_duplicate_odom_timestamp_does_not_recover_from_degraded():
    events = [sample(0), sample(0.1, odom_t=0.1, fresh_odom=False), sample(0.2, odom_t=0.2)]
    events.extend(sample(t, odom_t=0.2) for t in (0.3, 0.4, 0.5, 0.6))
    result = run_control({"samples": events})
    assert [item["mode"] for item in result["decisions"]] == ["NAVIGATE", "DEGRADED", "DEGRADED", "DEGRADED", "DEGRADED", "DEGRADED", "DEGRADED"]
    assert result["metrics"]["duplicate_samples"] == 4


def test_fresh_claim_cannot_bypass_stale_source_timestamp():
    result = run_control({"samples": [sample(0), sample(1.0, odom_t=0.0, fresh_odom=True)]})
    decision = result["decisions"][-1]
    assert decision["fresh_odom"] is False
    assert decision["state_valid"] is False
    assert "fresh_odom_claim_mismatch" in decision["reasons"]


def test_epoch_and_setpoint_epoch_change_discard_old_commands():
    result = run_control({"samples": [sample(0), sample(0.1, estimator_epoch=8, setpoint_epoch=8, odom_t=0.1)]})
    decision = result["decisions"][-1]
    assert decision["mode"] == Mode.DEGRADED.value
    assert decision["discarded_command_count"] == 1
    assert result["metrics"]["epoch_resets"] == 1
    assert result["metrics"]["setpoint_epoch_resets"] == 1
    assert "estimator_epoch_changed_old_commands_discarded" in decision["reasons"]
    assert "setpoint_epoch_changed_old_commands_discarded" in decision["reasons"]


def test_setpoint_epoch_mismatch_invalidates_even_when_estimator_is_same():
    result = run_control({"samples": [sample(0), sample(0.1, setpoint_epoch=8, odom_t=0.1)]})
    decision = result["decisions"][-1]
    assert decision["mode"] == Mode.DEGRADED.value
    assert decision["state_valid"] is False
    assert "setpoint_epoch_mismatch" in decision["reasons"]
    assert result["metrics"]["setpoint_epoch_resets"] == 1


def test_missing_altitude_never_maps_to_land_safe():
    result = run_control({"samples": [sample(0, observable_axes=["x", "y", "yaw"])]})
    decision = result["decisions"][-1]
    assert decision["mode"] == Mode.STANDBY.value
    assert decision["decision"] == NO_CONTROL
    assert decision["requested_action"] == FAILSAFE_REQUEST
    assert "missing_altitude_does_not_imply_safe_landing" in decision["reasons"]
    assert "land" not in decision["requested_action"].lower()


def test_recovery_requires_hysteresis_then_confirmation():
    events = [sample(0), sample(0.1, odom_t=0.1, fresh_odom=False)]
    events.extend(sample(t, odom_t=t) for t in (0.2, 0.3, 0.4, 0.5))
    result = run_control({"samples": events})
    modes = [item["mode"] for item in result["decisions"]]
    assert modes == ["NAVIGATE", "DEGRADED", "DEGRADED", "DEGRADED", "RECOVER", "NAVIGATE"]
    assert result["decisions"][4]["decision"] == NO_CONTROL
    assert result["decisions"][5]["decision"] == NAVIGATE_DECISION


def test_terminal_sensor_flag_latches_terminated():
    events = [sample(0), sample(0.1, terminal_sensor_flags={"critical_battery": True}, odom_t=0.1), sample(0.2, odom_t=0.2)]
    result = run_control({"samples": events})
    assert result["decisions"][1]["mode"] == Mode.TERMINATED.value
    assert result["decisions"][1]["decision"] == TERMINATE_DECISION
    assert result["decisions"][1]["requested_action"] == FAILSAFE_REQUEST
    assert result["decisions"][2]["mode"] == Mode.TERMINATED.value
    assert result["summary"].startswith("最终模式=TERMINATED")


@pytest.mark.parametrize(
    "field,value",
    [
        ("connection_alive", "false"),
        ("position_valid", "false"),
        ("velocity_valid", "false"),
        ("yaw_valid", "false"),
        ("clock_confidence", float("nan")),
        ("estimator_epoch", []),
        ("setpoint_epoch", {}),
        ("odom_time_s", float("nan")),
    ],
)
def test_malformed_boundary_values_are_rejected(field, value):
    with pytest.raises(ValueError):
        run_control({"samples": [sample(0, **{field: value})]})


def test_frame_sequence_must_also_be_new_when_present():
    result = run_control({"samples": [sample(0, seq=1), sample(0.1, odom_t=0.1, seq=1)]})
    decision = result["decisions"][-1]
    assert decision["state_valid"] is True
    assert "odom_frame_seq_not_strictly_new" in decision["reasons"]
    assert result["metrics"]["duplicate_samples"] == 1


@pytest.mark.parametrize(
    "key,value",
    [
        ("clock_confidence_min", -0.1),
        ("clock_confidence_min", 1.1),
        ("odom_timeout_s", 0),
        ("setpoint_timeout_s", float("nan")),
        ("recovery_samples", 0),
        ("recovery_samples", True),
        ("recovery_window_s", -1),
        ("initial_mode", "UNKNOWN"),
        ("required_axes", "xyz"),
    ],
)
def test_config_bounds_are_validated(key, value):
    with pytest.raises((TypeError, ValueError)):
        run_control({key: value})
