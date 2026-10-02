import json
import os
import math
import socket
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PySide6.QtWidgets import QApplication, QMessageBox
from PySide6.QtTest import QTest
from app.data import (
    demonstration,
    validate_record,
    valid_distances,
    Recorder,
    load_recording,
)
from app.workers import sensor_arguments, FCWorker


@pytest.fixture(scope="session")
def qt():
    return QApplication.instance() or QApplication([])


def test_record_and_replay(tmp_path):
    path = tmp_path / "data.jsonl"
    rec = Recorder(path)
    for item in demonstration(1):
        rec.write(item)
    rec.close()
    rows = load_recording(path)
    assert len(rows) == 3 and rows[0][0] == 0
    assert len(valid_distances(rows[0][1])) == 62
    assert rows[0][1]["source"] == "DEMO"
    with pytest.raises(FileExistsError):
        Recorder(path)


@pytest.mark.parametrize("damage", ["nan", "size", "status", "sensor"])
def test_reject_bad_frames(damage):
    item = demonstration(0)[0]
    if damage == "nan":
        item["distance_mm"][0] = float("nan")
    if damage == "size":
        item["distance_mm"].pop()
    if damage == "status":
        item["status"][0] = -1
    if damage == "sensor":
        item["sensor"] = 9
    with pytest.raises(ValueError):
        validate_record(item)


def test_sensor_configuration():
    config = {
        "sensors": [
            {"bus": "/dev/i2c-1", "address": "0x29"},
            {"bus": "/dev/i2c-2", "address": "0x29"},
        ],
        "hz": 10,
    }
    assert sensor_arguments(config) == [
        "--sensor",
        "/dev/i2c-1:0x29",
        "--sensor",
        "/dev/i2c-2:0x29",
        "--hz",
        "10",
    ]
    config["sensors"][1]["bus"] = "/dev/i2c-1"
    with pytest.raises(ValueError, match="同总线"):
        sensor_arguments(config)
    config["sensors"][0]["bus"] = "/dev/i2c-1; reboot"
    with pytest.raises(ValueError):
        sensor_arguments(config)


def test_invalid_replay_time(tmp_path):
    path = tmp_path / "bad.jsonl"
    item = demonstration(0)[0]
    item["record_time_s"] = -1
    path.write_text(json.dumps(item), encoding="utf-8")
    with pytest.raises(ValueError, match="第 1 行"):
        load_recording(path)


def test_live_mavlink_parse_and_stop(qt):
    from pymavlink import mavutil

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    worker = FCWorker(f"udpin:127.0.0.1:{port}", 115200)
    received = []
    errors = []
    worker.record.connect(received.append)
    worker.failure.connect(errors.append)
    worker.start()
    sender = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=1)
    try:
        for _ in range(20):
            sender.mav.heartbeat_send(2, 12, 0, 0, 3)
            sender.mav.attitude_send(1234, 0.1, 0.2, 0.3, 0, 0, 0)
            QTest.qWait(50)
            if any(r["type"] == "attitude" for r in received):
                break
        assert not errors
        a = next(r for r in received if r["type"] == "attitude")
        assert (
            a["source_time_us"] == 1234000
            and abs(a["roll_deg"] - math.degrees(0.1)) < 0.001
        )
    finally:
        worker.stop()
        assert worker.wait(3000)
        sender.close()


def test_gui_demo_record_replay(qt, tmp_path, monkeypatch):
    import app.main as main

    for name in ("logs", "config", "samples", "docs"):
        (tmp_path / name).mkdir()
    monkeypatch.setattr(main, "ROOT", tmp_path)
    errors = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: errors.append(args[-1]))
    window = main.LabWindow()
    window.show()
    try:
        window.demo_button.click()
        QTest.qWait(350)
        assert window.mode == "demo" and window.cards[0].total >= 2
        window.record_button.click()
        QTest.qWait(350)
        window.record_button.click()
        path = next((tmp_path / "logs").glob("DEMO*.jsonl"))
        assert len(load_recording(path)) >= 3
        window.stop_sources()
        window.open_replay(path)
        window.play_pause()
        QTest.qWait(500)
        assert window.cards[0].total > 0 and not errors
        output = tmp_path / "export.csv"
        monkeypatch.setattr(
            main.QFileDialog, "getSaveFileName", lambda *a: (str(output), "CSV")
        )
        window.export_csv()
        assert output.read_text(encoding="utf-8-sig").startswith(
            "record_time_s,source,sensor,zone"
        )
        window.slider.setValue(500)
        window.seek()
        assert window.replay_time >= 0 and window.cards[0].map.frame is not None
        window.save_settings(silent=True)
        assert "password" not in (tmp_path / "config" / "local.json").read_text(
            encoding="utf-8"
        )
    finally:
        window.stop_sources()
        window.close()
