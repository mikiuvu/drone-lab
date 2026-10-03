import csv
import json
import math

import pytest

from simulation.flight_data import FlightHistory, aligned_samples, export_csv


def estimate(vehicle, stamp, **extra):
    return {
        "kind": "estimate",
        "source": "ESTIMATE",
        "vehicle": vehicle,
        "sim_time_us": stamp,
        "x_m": 1,
        "y_m": 2,
        "z_m": 0.75,
        "vx_mps": 3,
        "vy_mps": 4,
        "vz_mps": 0,
        "yaw_rad": 0.1,
        "position_confidence": 0.9,
        "flow_valid": True,
        "xy_valid": True,
        **extra,
    }


def write_rows(path, rows):
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")


def make_run(folder, count=1):
    write_rows(
        folder / "truth_evaluation_only.jsonl",
        [
            {
                "kind": "truth_evaluation_only",
                "vehicle": i,
                "sim_time_us": t,
                "x_m": 0.7,
                "y_m": 1.6,
                "z_m": 0.75,
                "yaw_rad": 0.1,
            }
            for t in (1_000_000, 1_100_000)
            for i in range(count)
        ],
    )
    for i in range(count):
        write_rows(
            folder / f"companion_{i}.jsonl",
            [estimate(i, 1_000_000), estimate(i, 1_100_000)],
        )
    write_rows(
        folder / "team_mission.jsonl",
        [
            {"sim_time_us": 1_000_000, "phase": "CONTACT", "active_vehicle": 0},
            {"sim_time_us": 1_050_000, "phase": "CONTACT", "active_vehicle": 0},
            {"sim_time_us": 1_100_000, "phase": "RETREAT", "active_vehicle": 0},
        ],
    )


@pytest.mark.parametrize("count", [1, 6, 12])
def test_fleet_history_and_error_csv(tmp_path, count):
    make_run(tmp_path, count)
    data = FlightHistory(tmp_path)
    assert data.poll()
    snapshot = data.snapshot()
    assert snapshot["ids"] == list(range(count))
    assert len(snapshot["phases"]) == 2
    assert not data.poll()
    for i in range(count):
        matched = list(aligned_samples(snapshot["estimates"][i], snapshot["truths"][i]))
        assert len(matched) == 2
        assert matched[0][2] == pytest.approx(0.5)
    path = tmp_path / "export.csv"
    export_csv(snapshot, path, [count - 1])
    with path.open(encoding="utf-8-sig", newline="") as stream:
        exported = list(csv.DictReader(stream))
    assert len(exported) == 2
    assert exported[0]["vehicle"] == str(count - 1)
    assert float(exported[0]["xy_error_m"]) == pytest.approx(0.5)
    assert float(exported[0]["vy_mps"]) == 4


def test_alignment_preserves_missing_truth_as_nan():
    estimates = [(1.0, 1, 2, 0.75), (1.5, 1, 2, 0.75), (2.0, 1, 2, 0.75)]
    truths = [(1.01, 0.7, 1.6, 0.75), (2.01, 0.7, 1.6, 0.75)]
    matches = list(aligned_samples(estimates, truths))
    assert matches[0][2] == pytest.approx(0.5)
    assert matches[1][1] is None and math.isnan(matches[1][2])
    assert matches[2][2] == pytest.approx(0.5)


def test_alignment_does_not_turn_high_speed_sampling_offset_into_error():
    estimates = [(1.05, 0.2, 0, 0.75)]
    truths = [(1.0, 0, 0, 0.75), (1.1, 0.4, 0, 0.75)]
    sample, truth, error = next(aligned_samples(estimates, truths))
    assert truth[0] == sample[0]
    assert error == pytest.approx(0)


def test_incremental_partial_lines_and_truncation(tmp_path):
    make_run(tmp_path)
    data = FlightHistory(tmp_path)
    data.poll()
    path = tmp_path / "companion_0.jsonl"
    payload = json.dumps(estimate(0, 1_200_000))
    with path.open("a", encoding="utf-8") as stream:
        stream.write(payload[:40])
    assert not data.poll()
    with path.open("a", encoding="utf-8") as stream:
        stream.write(payload[40:] + "\n")
    assert data.poll()
    assert len(data.estimates[0]) == 3
    write_rows(path, [estimate(0, 2_000_000)])
    assert data.poll()
    data.poll()
    assert [row[0] for row in data.estimates[0]] == [2.0]


def test_bad_identity_duplicates_and_memory_limit(tmp_path):
    make_run(tmp_path)
    data = FlightHistory(tmp_path, max_points=3)
    data.poll()
    data.consume(
        0,
        [
            estimate(1, 2_000_000),
            estimate(0, 1_000_000),
            estimate(0, 2_000_000, source="TRUTH"),
            estimate(0, 2_000_000, x_m=math.nan),
        ],
    )
    assert len(data.estimates[0]) == 2
    data.consume(0, [estimate(0, t) for t in range(2_000_000, 4_000_000, 10_000)])
    assert len(data.estimates[0]) == 3 and data.dropped > 0
    data.discover()
    path = tmp_path / "companion_20.jsonl"
    write_rows(path, [estimate(20, 3_000_000)])
    data.poll()
    assert 20 in data.snapshot()["ids"]


def test_missing_directory_is_clear(tmp_path):
    with pytest.raises(ValueError, match="没有飞行日志"):
        FlightHistory(tmp_path)


def test_dashboard_playback_and_render(tmp_path):
    from PySide6.QtWidgets import QApplication

    from app.flight_dashboard import FlightDashboard

    app = QApplication.instance() or QApplication([])
    make_run(tmp_path, 6)
    data = FlightHistory(tmp_path)
    data.poll()
    window = FlightDashboard()
    window.accept_snapshot(data.snapshot())
    window.show()
    app.processEvents()
    assert len(window.plot_data) == 6
    assert len(window.axes) == 6
    assert not window.grab().isNull()
    window.vehicle.setCurrentIndex(6)
    assert list(window.plot_data) == [5]
    window.slider.setValue(5000)
    assert window.cursor == pytest.approx(1.05)
    assert not window.follow.isChecked()
    assert "XY误差" in window.readout.text()
    window.toggle_play()
    assert window.player.isActive()
    window.close()
    app.processEvents()


def test_loader_can_close_while_reading(tmp_path):
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication

    from app.flight_dashboard import FlightDashboard

    app = QApplication.instance() or QApplication([])
    make_run(tmp_path)
    window = FlightDashboard(tmp_path)
    window.show()
    QTest.qWait(120)
    window.close()
    assert window.worker is None
    app.processEvents()


def test_lab_closes_reader_with_main_window(tmp_path):
    from unittest.mock import patch

    from PySide6.QtCore import QCoreApplication, QEvent
    from PySide6.QtWidgets import QApplication

    from app.main import LabWindow

    app = QApplication.instance() or QApplication([])
    make_run(tmp_path)
    window = LabWindow()
    window.gazebo_panel.timer.stop()
    with patch.object(window.gazebo_panel, "refresh"):
        window.gazebo_panel.current_folder = tmp_path
        window.gazebo_panel.open_flight_dashboard()
    dashboard = window.gazebo_panel.flight_dashboard
    assert dashboard.worker.isRunning()
    with patch.object(window, "save_settings"):
        window.close()
    assert dashboard.worker is None
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    app.processEvents()
