"""读取被阻塞时的界面响应、目录切换与 Qt 对象生命周期测试。"""

import gc
import threading
import time
import weakref
from pathlib import Path

import pytest
from PySide6.QtCore import QTimer
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication
from shiboken6 import isValid

from app import flight_dashboard
from app.flight_dashboard import FlightDashboard, FlightLoader


def wait_until(qapp, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        qapp.processEvents()
        if predicate():
            return True
        QTest.qWait(10)
    qapp.processEvents()
    return predicate()


@pytest.fixture
def reader_lab(monkeypatch):
    """保留真实 FlightLoader.run，只用可释放的 Event 阻塞文件读取。"""
    qapp = QApplication.instance() or QApplication([])
    gates = {}
    opened = []
    windows = []
    counts = {"active": 0, "maximum": 0}
    lock = threading.Lock()
    original_run = FlightLoader.run

    def gate(folder):
        return gates.setdefault(
            Path(folder),
            {"entered": threading.Event(), "release": threading.Event(), "error": None},
        )

    class BlockingHistory:
        backlog = False

        def __init__(self, folder):
            self.folder = Path(folder)
            self.gate = gate(folder)
            opened.append(self.folder)

        def poll(self, rounds):
            self.gate["entered"].set()
            self.gate["release"].wait(5.0)
            if self.gate["error"] is not None:
                raise OSError(self.gate["error"])
            return True

        def snapshot(self):
            return {"folder": str(self.folder), "ids": [], "estimates": {}}

    def counted_run(worker):
        with lock:
            counts["active"] += 1
            counts["maximum"] = max(counts["maximum"], counts["active"])
        try:
            original_run(worker)
        finally:
            with lock:
                counts["active"] -= 1

    def dashboard(folder=None):
        window = FlightDashboard(folder)
        windows.append(window)
        return window

    monkeypatch.setattr(flight_dashboard, "FlightHistory", BlockingHistory)
    monkeypatch.setattr(FlightLoader, "run", counted_run)
    yield qapp, gate, dashboard, opened, counts
    # 断言失败时也释放读取器，避免测试退出时销毁仍在运行的 QThread。
    for window in windows:
        if isValid(window):
            window.shutdown()
    for state in gates.values():
        state["release"].set()
    assert wait_until(qapp, lambda: all(not w.is_busy() for w in windows))
    qapp.processEvents()


def test_close_retains_blocked_thread_and_keeps_gui_timer_running(reader_lab, tmp_path):
    qapp, gate, dashboard, _, _ = reader_lab
    state = gate(tmp_path)
    window = dashboard(tmp_path)
    window.show()
    destroyed = []
    completed = []
    window.destroyed.connect(lambda: destroyed.append(True))
    window.shutdown_complete.connect(lambda: completed.append(window.is_busy()))
    assert wait_until(qapp, state["entered"].is_set)
    worker_ref = weakref.ref(window.worker)
    worker_destroyed = []
    window.worker.destroyed.connect(lambda: worker_destroyed.append(True))

    ticks = []
    timer = QTimer(window)
    timer.setInterval(10)
    timer.timeout.connect(lambda: ticks.append(True))
    timer.start()
    started_close = time.monotonic()
    assert not window.close()
    assert time.monotonic() - started_close < 0.2
    gc.collect()
    assert window.is_busy()
    assert worker_ref() is not None and isValid(worker_ref())
    assert worker_ref().isRunning()
    assert not destroyed and not worker_destroyed and not completed

    ticks.clear()
    QTest.qWait(80)
    assert ticks
    window.shutdown()
    window.close()
    assert not completed
    state["release"].set()
    assert wait_until(qapp, lambda: bool(destroyed))
    assert worker_destroyed
    assert completed == [False]
    assert window.snapshot is None


def test_many_folder_changes_start_only_latest_reader_and_ignore_old_snapshot(
    reader_lab, tmp_path
):
    qapp, gate, dashboard, opened, counts = reader_lab
    first = tmp_path / "first"
    folders = [tmp_path / f"run-{i}" for i in range(30)]
    window = dashboard(first)
    assert wait_until(qapp, gate(first)["entered"].is_set)
    for folder in folders:
        window.load_folder(folder)
    assert opened == [first]
    assert len(window.findChildren(FlightLoader)) == 1
    gate(first)["release"].set()
    assert wait_until(qapp, gate(folders[-1])["entered"].is_set)
    assert opened == [first, folders[-1]]
    assert window.snapshot is None
    assert "读取失败" not in window.summary.text()
    assert counts["maximum"] == 1

    gate(folders[-1])["release"].set()
    assert wait_until(qapp, lambda: window.snapshot is not None)
    assert Path(window.snapshot["folder"]) == folders[-1]
    assert len(window.findChildren(FlightLoader)) == 1


def test_late_reader_error_does_not_replace_new_directory_status(reader_lab, tmp_path):
    qapp, gate, dashboard, _, _ = reader_lab
    first, latest = tmp_path / "broken", tmp_path / "latest"
    gate(first)["error"] = "old directory read failed"
    window = dashboard(first)
    assert wait_until(qapp, gate(first)["entered"].is_set)
    window.load_folder(latest)
    expected = window.summary.text()
    gate(first)["release"].set()
    assert wait_until(qapp, gate(latest)["entered"].is_set)
    assert window.summary.text() == expected


def test_shutdown_cancels_pending_folder_and_rejects_further_loads(reader_lab, tmp_path):
    qapp, gate, dashboard, opened, _ = reader_lab
    first, pending = tmp_path / "first", tmp_path / "pending"
    window = dashboard(first)
    completed = []
    window.shutdown_complete.connect(lambda: completed.append(window.is_busy()))
    assert wait_until(qapp, gate(first)["entered"].is_set)
    window.load_folder(pending)
    window.shutdown()
    window.load_folder(tmp_path / "after-close")
    assert window.is_busy()
    gate(first)["release"].set()
    assert wait_until(qapp, lambda: bool(completed))
    assert completed == [False]
    assert opened == [first]


def test_idle_shutdown_emits_completion_once(reader_lab):
    _, _, dashboard, _, _ = reader_lab
    window = dashboard()
    completed = []
    window.shutdown_complete.connect(lambda: completed.append(window.is_busy()))
    window.shutdown()
    window.shutdown()
    assert completed == [False]


@pytest.mark.parametrize("action", ["reject", "accept", "done"])
def test_dialog_exit_methods_wait_for_reader(reader_lab, tmp_path, action):
    qapp, gate, dashboard, _, _ = reader_lab
    window = dashboard(tmp_path)
    window.show()
    assert wait_until(qapp, gate(tmp_path)["entered"].is_set)
    if action == "done":
        window.done(0)
    else:
        getattr(window, action)()
    assert isValid(window) and window.is_busy() and window.isVisible()
    gate(tmp_path)["release"].set()
    assert wait_until(qapp, lambda: not isValid(window))
