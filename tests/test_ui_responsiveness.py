"""后台读盘阻塞、重复刷新、退出与日志来源的界面回归。"""

import json
import threading
import time
from unittest.mock import patch

import numpy as np
import pytest
from PySide6.QtCore import QTimer
from PySide6.QtGui import QImage
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QFileDialog, QMessageBox

from app.data import demonstration
from app.simulation_panel import GazeboPanel
from app.simulation_reader import SimulationReader
from app.widgets import SensorCard
from app.workers import Worker


@pytest.fixture(scope="module")
def qt():
    return QApplication.instance() or QApplication([])


def until(predicate, seconds=4):
    end = time.monotonic() + seconds
    while not predicate() and time.monotonic() < end:
        QTest.qWait(10)
    assert predicate()


def make_run(root, name="run", count=1):
    folder = root / name
    folder.mkdir()
    runtime = root / "simulation/runtime"
    runtime.mkdir(parents=True, exist_ok=True)
    config = {"name": name, "spawns": [
        {"id": i, "x": i, "y": 0, "z": 0.5} for i in range(count)]}
    (runtime / "manager.json").write_text(
        json.dumps({"run_dir": str(folder), "map": config}), encoding="utf-8")
    return folder, config


def test_slow_refresh_keeps_gui_alive_coalesces_requests_and_closes(qt, tmp_path, monkeypatch):
    panel = GazeboPanel(tmp_path)
    panel.timer.stop()
    entered, release = threading.Event(), threading.Event()
    calls, ticks = [], []

    def blocked(cancelled):
        calls.append(threading.get_ident())
        entered.set()
        assert release.wait(4)
        return None

    monkeypatch.setattr(panel.reader, "read", blocked)
    pulse = QTimer()
    pulse.timeout.connect(lambda: ticks.append(1))
    pulse.start(10)
    completed = []
    panel.shutdown_complete.connect(lambda: completed.append(1))
    try:
        panel.refresh()
        until(entered.is_set)
        first = panel.refresh_worker
        for _ in range(50):
            panel.refresh()
        QTest.qWait(100)
        assert len(ticks) >= 3
        assert panel.refresh_worker is first and len(calls) == 1
        assert calls[0] != threading.get_ident()
        start = time.monotonic()
        panel.close()
        assert time.monotonic() - start < 0.2
        assert panel.refresh_worker is first and first.isRunning()
        release.set()
        until(lambda: panel.refresh_worker is None)
        assert completed and len(calls) == 1
    finally:
        release.set()
        pulse.stop()
        panel.close()
        until(lambda: panel.refresh_worker is None)


def test_reader_caches_images_and_resets_on_new_run(qt, tmp_path):
    folder, config = make_run(tmp_path)
    grid = tmp_path / "terrain.npz"
    np.savez(grid, z=np.zeros((4, 4)))
    config["terrain_height_grid"] = str(grid)
    state = tmp_path / "simulation/runtime/manager.json"
    state.write_text(json.dumps({"run_dir": str(folder), "map": config}), encoding="utf-8")
    (folder / "camera").mkdir()
    camera = folder / "camera/camera_0.ppm"
    image = QImage(8, 8, QImage.Format.Format_RGB888)
    image.fill(0xff00ff)
    assert image.save(str(camera))
    reader = SimulationReader(tmp_path)
    with patch("app.simulation_reader.np.load", wraps=np.load) as loading:
        first = reader.read()
        second = reader.read()
        assert loading.call_count == 1
        assert first["terrain"] is second["terrain"]
        assert first["cameras"][0] is second["cameras"][0]
        np.savez(grid, z=np.ones((5, 5)))
        third = reader.read()
        assert loading.call_count == 2
        assert third["terrain"].size() != first["terrain"].size()
    other, _ = make_run(tmp_path, "other", 6)
    fourth = reader.read()
    assert fourth["folder"] == other
    assert len(fourth["snapshot"]["vehicles"]) == 6
    assert fourth["cameras"] == {} and fourth["terrain"] is None
    assert first["snapshot"]["folder"] == str(folder)


def test_decoding_and_metrics_run_off_gui_and_report_uses_snapshot(qt, tmp_path, monkeypatch):
    folder, _ = make_run(tmp_path)
    panel = GazeboPanel(tmp_path)
    panel.timer.stop()
    observed = []
    original = panel.reader.read

    def read(cancelled):
        observed.append(threading.get_ident())
        return original(cancelled)

    monkeypatch.setattr(panel.reader, "read", read)
    try:
        panel.refresh()
        until(lambda: panel.refresh_worker is None)
        assert observed[0] != threading.get_ident()
        assert panel.snapshot["folder"] == str(folder)
        report = tmp_path / "report.json"
        monkeypatch.setattr(QFileDialog, "getSaveFileName", lambda *a: (str(report), ""))
        with patch.object(panel.reader.metrics, "snapshot", side_effect=AssertionError("UI read")):
            panel.save_report()
        assert json.loads(report.read_text(encoding="utf-8"))["folder"] == str(folder)
    finally:
        panel.close()
        until(lambda: panel.refresh_worker is None)


def test_refresh_error_recovers_on_next_request(qt, tmp_path, monkeypatch):
    make_run(tmp_path)
    panel = GazeboPanel(tmp_path)
    panel.timer.stop()
    try:
        with patch.object(panel.reader, "read", side_effect=OSError("disk unavailable")):
            panel.refresh()
            until(lambda: panel.refresh_worker is None)
        assert "disk unavailable" in panel.status.text()
        panel.refresh()
        until(lambda: panel.refresh_worker is None)
        assert panel.snapshot is not None
    finally:
        panel.close()


def test_refresh_burst_runs_one_followup(qt, tmp_path, monkeypatch):
    panel = GazeboPanel(tmp_path)
    panel.timer.stop()
    entered, release = threading.Event(), threading.Event()
    calls = []

    def blocked(cancelled):
        calls.append(1)
        entered.set()
        assert release.wait(4)
        return None

    monkeypatch.setattr(panel.reader, "read", blocked)
    try:
        panel.refresh()
        until(entered.is_set)
        for _ in range(50):
            panel.refresh()
        release.set()
        until(lambda: panel.refresh_worker is None)
        assert len(calls) == 2
    finally:
        release.set()
        panel.close()
        until(lambda: panel.refresh_worker is None)


def test_run_changed_during_read_discards_old_snapshot(qt, tmp_path, monkeypatch):
    folder, _ = make_run(tmp_path)
    reader = SimulationReader(tmp_path)
    read_terrain = reader.read_terrain
    newer = []

    def switch(config):
        newer.append(make_run(tmp_path, "newer")[0])
        return read_terrain(config)

    with patch.object(reader, "read_terrain", side_effect=switch):
        assert reader.read() is None
    snapshot = reader.read()
    assert snapshot["folder"] == newer[0] and snapshot["folder"] != folder


def test_main_close_keeps_child_reader_alive_until_finished(qt, tmp_path, monkeypatch):
    import app.main as main
    from app import flight_dashboard
    from shiboken6 import isValid

    monkeypatch.setattr(main, "ROOT", tmp_path)
    monkeypatch.setattr(main.LabWindow, "save_settings", lambda *a, **k: None)
    entered, release = threading.Event(), threading.Event()

    class SlowHistory:
        backlog = False

        def __init__(self, folder):
            pass

        def poll(self, rounds):
            entered.set()
            assert release.wait(5)
            return False

    monkeypatch.setattr(flight_dashboard, "FlightHistory", SlowHistory)
    window = main.LabWindow()
    panel = window.gazebo_panel
    panel.timer.stop()
    panel.current_folder = tmp_path
    window.show()
    panel.open_flight_dashboard()
    child = panel.flight_dashboard
    try:
        until(entered.is_set)
        window.close()
        QTest.qWait(120)
        assert window.isVisible() and isValid(child) and child.is_busy()
        release.set()
        until(lambda: not window.isVisible())
        assert panel.flight_dashboard is None and not isValid(child)
    finally:
        release.set()
        window.close()
        until(lambda: panel.flight_dashboard is None)


def test_historical_rate_ignores_burst_delivery_and_clock_reset(qt):
    card = SensorCard(0)
    try:
        for i in range(21):
            card.receive(demonstration(i / 10, i)[0], historical=True)
        assert card.tiles["rate"].value.text() == "10.0"
        assert card.tiles["rate"].title.text() == "数据帧率"
        card.receive(demonstration(0, 0)[0], historical=True)
        assert card.tiles["rate"].value.text() == "—"
        for i in range(1, 11):
            card.receive(demonstration(i / 10, i)[0], historical=True)
        assert card.tiles["rate"].value.text() == "10.0"
    finally:
        card.close()


def test_live_rate_needs_time_window_and_does_not_invent_frequency(qt):
    card = SensorCard(0)
    try:
        with patch("app.widgets.time.monotonic", return_value=100.0):
            for i in range(30):
                card.receive(demonstration(i / 10, i)[0])
        assert card.tiles["rate"].value.text() == "—"
        card.reset()
        for i in range(21):
            with patch("app.widgets.time.monotonic", return_value=100 + i / 10):
                card.receive(demonstration(i / 10, i)[0])
        assert card.tiles["rate"].value.text() == "10.0"
        assert card.tiles["rate"].title.text() == "接收帧率"
    finally:
        card.close()


def test_logs_are_routed_by_worker_source(qt, tmp_path, monkeypatch):
    import app.main as main

    monkeypatch.setattr(main, "ROOT", tmp_path)
    monkeypatch.setattr(main.LabWindow, "save_settings", lambda *a, **k: None)
    monkeypatch.setattr(QMessageBox, "warning", lambda *a: None)
    window = main.LabWindow()
    window.gazebo_panel.timer.stop()

    class SourceWorker(Worker):
        def run(self):
            self.message.emit("connection message")
            self.failure.emit("connection failure")

    try:
        window.events.clear()
        window.nano_events.clear()
        window.attach_worker("fc", SourceWorker())
        until(lambda: not window.workers)
        assert "connection message" in window.events.toPlainText()
        assert window.nano_events.toPlainText() == ""
        window.events.clear()
        window.attach_worker("nano", SourceWorker())
        until(lambda: not window.workers)
        assert window.events.toPlainText() == ""
        assert "connection failure" in window.nano_events.toPlainText()
        window.log("")
    finally:
        window.close()


def test_main_window_waits_without_freezing_for_reader_exit(qt, tmp_path, monkeypatch):
    import app.main as main

    monkeypatch.setattr(main, "ROOT", tmp_path)
    monkeypatch.setattr(main.LabWindow, "save_settings", lambda *a, **k: None)
    window = main.LabWindow()
    window.gazebo_panel.timer.stop()
    entered, release = threading.Event(), threading.Event()

    def blocked(cancelled):
        entered.set()
        assert release.wait(4)
        return None

    monkeypatch.setattr(window.gazebo_panel.reader, "read", blocked)
    window.show()
    try:
        window.gazebo_panel.refresh()
        until(entered.is_set)
        start = time.monotonic()
        window.close()
        assert time.monotonic() - start < 0.2
        assert window.isVisible()
        QTest.qWait(150)
        assert window.isVisible()
        release.set()
        until(lambda: not window.isVisible())
        assert window.gazebo_panel.refresh_worker is None
    finally:
        release.set()
        window.close()
        until(lambda: window.gazebo_panel.refresh_worker is None)
