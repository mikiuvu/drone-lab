"""localization_study 输出的契约测试。

读取 build/research-20261023/localization-transit.json 与其 trace JSONL，
校验结构、组合数、指纹一致性、时间戳单调性与必报字段。输出缺失时整体跳过。
"""

import json
import os
from collections import defaultdict

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RESULT_PATH = os.path.join(REPO_ROOT, "build", "research-20261023", "localization-transit.json")
TRACE_CANDIDATES = (
    os.path.join(REPO_ROOT, "build", "research-20261023", "localization-transit.trace.jsonl"),
    RESULT_PATH + ".trace.jsonl",
)


def resolve_trace_path():
    for candidate in TRACE_CANDIDATES:
        if os.path.exists(candidate):
            return candidate
    return None

SCENARIOS = ("ideal", "noise", "occluders", "combined", "adverse", "blackout")
COMM_MODES = ("on", "off", "intermittent")
FUSION_MODES = ("tof_imu_height", "tof_flow_imu_height")
RUN_KEYS = (
    "window",
    "scenario",
    "profile",
    "seed",
    "comm",
    "fusion",
    "samples",
    "xy_rmse_m",
    "xy_p50_m",
    "xy_p95_m",
    "max_xy_error_m",
    "xy_valid_fraction",
    "xy_structural_fraction",
    "false_valid_fraction",
    "common_drift_rmse_m",
    "pair_relative_rmse_m",
    "tof_calls",
    "tof_accepted",
    "uwb_calls",
    "uwb_accepted",
    "uwb_rejections",
    "collaboration_evidence",
    "input_checksum",
    "vehicles",
)


@pytest.fixture(scope="module")
def result():
    if not os.path.exists(RESULT_PATH):
        pytest.skip(f"missing study output: {RESULT_PATH}")
    with open(RESULT_PATH, encoding="utf-8") as handle:
        return json.load(handle)


def test_top_level_sections(result):
    for key in (
        "schema",
        "generator",
        "terrain",
        "trajectory",
        "windows",
        "rates_hz",
        "profiles",
        "scenarios",
        "communication_modes",
        "fusion_modes",
        "sensor_noise",
        "initialization",
        "limitations",
        "sensor_model",
        "runs",
        "paired_by_seed",
    ):
        assert key in result, f"missing section {key}"
    assert result["schema"] == 1
    assert result["limitations"], "limitations must be documented"


def test_scenarios_and_profiles(result):
    names = [scenario["name"] for scenario in result["scenarios"]]
    for scenario in SCENARIOS:
        assert scenario in names, f"missing scenario {scenario}"
    for profile in ("ideal", "nominal", "adverse"):
        fields = result["profiles"][profile]
        for field in (
            "noise_std_m",
            "range_sigma_per_m",
            "bias_bound_m",
            "scale_bound",
            "frame_bias_std_m",
            "drop_probability",
        ):
            assert field in fields, f"profile {profile} missing {field}"


def test_run_matrix_is_complete(result):
    windows = [window["name"] for window in result["windows"]]
    seeds = sorted({run["seed"] for run in result["runs"]})
    expected = len(windows) * len(SCENARIOS) * len(seeds) * len(COMM_MODES) * len(FUSION_MODES)
    assert len(result["runs"]) == expected, "run matrix is incomplete"
    seen = {
        (run["window"], run["scenario"], run["seed"], run["comm"], run["fusion"])
        for run in result["runs"]
    }
    assert len(seen) == expected, "duplicate run combinations"


def test_run_fields_and_ranges(result):
    for run in result["runs"]:
        for key in RUN_KEYS:
            assert key in run, f"{run.get('scenario')} missing {key}"
        assert run["samples"] > 0
        for key in ("xy_valid_fraction", "xy_structural_fraction", "false_valid_fraction"):
            assert 0.0 <= run[key] <= 1.0, f"{key} out of range"
        assert run["xy_p50_m"] <= run["xy_p95_m"] <= run["max_xy_error_m"] + 1e-9
        assert len(run["vehicles"]) == 4
        for vehicle in run["vehicles"]:
            assert vehicle["init_offset_m"] == pytest.approx(0.1, abs=1e-6)


def test_checksum_is_shared_across_modes(result):
    fingerprints = defaultdict(set)
    for run in result["runs"]:
        key = (run["window"], run["scenario"], run["seed"])
        fingerprints[key].add(run["input_checksum"])
    for key, values in fingerprints.items():
        assert len(values) == 1, f"fingerprint differs across modes for {key}: {values}"


def test_communication_off_has_no_uwb_calls(result):
    for run in result["runs"]:
        if run["comm"] == "off":
            assert run["uwb_calls"] == 0
            assert run["uwb_accepted"] == 0


def test_collaboration_claim_requires_accepted_updates(result):
    for run in result["runs"]:
        if run["comm"] == "off" or run["uwb_accepted"] == 0:
            assert run["collaboration_evidence"] == "no_accepted_updates"
        else:
            assert run["collaboration_evidence"] == "accepted_updates"


def test_sensor_model_reports_occlusion_and_error(result):
    assert result["sensor_model"], "sensor_model must not be empty"
    for entry in result["sensor_model"]:
        for key in (
            "rays",
            "robot_occluded_rays",
            "robot_occluded_fraction",
            "no_return",
            "dropped",
            "out_of_range",
            "valid_returns",
        ):
            assert key in entry, f"sensor_model missing {key}"
        assert entry["rays"] > 0
        assert entry["robot_occluded_rays"] <= entry["rays"]


def test_paired_by_seed_is_paired(result):
    assert result["paired_by_seed"], "paired comparison must not be empty"
    for entry in result["paired_by_seed"]:
        assert "seed" in entry
        expected = entry["xy_rmse_on_m"] - entry["xy_rmse_off_m"]
        assert entry["delta_rmse_on_minus_off_m"] == pytest.approx(expected, abs=1e-12)


def test_trace_timestamps_increase(result):
    trace_path = resolve_trace_path()
    if trace_path is None:
        pytest.skip(f"missing trace, looked for {TRACE_CANDIDATES}")
    last = {}
    with open(trace_path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            key = (record["w"], record["s"], record["k"], record["c"], record["f"], record["v"])
            previous = last.get(key)
            if previous is not None:
                assert record["t"] > previous, f"non-increasing trace time for {key}"
            last[key] = record["t"]
    assert last, "trace must contain samples"
