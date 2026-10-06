import importlib.util
import json
import subprocess
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "drone_gui_launcher", ROOT / "simulation" / "launch_linux.py"
)
LAUNCHER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCHER)


def response(stdout=b"", stderr=b"", returncode=0):
    return SimpleNamespace(stdout=stdout, stderr=stderr, returncode=returncode)


def live_process(name, pid=101):
    return {
        "name": name,
        "pid": pid,
        "command": ["gz", "sim", "-s" if name == "gazebo" else "-g"],
        "log": "",
        "start_ticks": "1",
    }


def write_world(folder, name="demo"):
    (folder / "world.sdf").write_text(
        f'<sdf version="1.9"><world name="{name}" /></sdf>', encoding="utf-8"
    )


def test_gui_environment_replaces_offscreen_backend():
    result = LAUNCHER.gui_environment(
        {
            "DISPLAY": ":0",
            "WAYLAND_DISPLAY": "wayland-0",
            "QT_QPA_PLATFORM": "offscreen",
        }
    )

    assert result["QT_QPA_PLATFORM"] == "xcb"


def test_gui_environment_requires_display():
    with pytest.raises(RuntimeError, match="无可用显示"):
        LAUNCHER.gui_environment({"QT_QPA_PLATFORM": "offscreen"})


def test_launch_gui_uses_config_and_waits_for_new_process(tmp_path, monkeypatch):
    write_world(tmp_path)
    server = live_process("gazebo")
    started = live_process("gazebo_gui", pid=102)
    start_one = Mock(return_value=started)
    wait_for_gui = Mock()
    monkeypatch.setattr(LAUNCHER, "alive", lambda item: True)
    monkeypatch.setattr(LAUNCHER, "start_one", start_one)
    monkeypatch.setattr(LAUNCHER, "wait_for_gui", wait_for_gui)

    items = [server]
    result = LAUNCHER.launch_gui(
        items,
        tmp_path,
        {"GZ_PARTITION": "drone-lab-test", "DISPLAY": ":0"},
    )

    assert result is started
    assert items == [server, started]
    args = start_one.call_args.args
    assert args[0] == "gazebo_gui"
    assert args[1] == [
        "gz",
        "sim",
        "-g",
        "-v",
        "4",
        "--gui-config",
        LAUNCHER.GUI_CONFIG,
    ]
    wait_for_gui.assert_called_once_with(
        started, server, {"GZ_PARTITION": "drone-lab-test", "DISPLAY": ":0"}, "demo"
    )


def test_launch_gui_reuses_live_gui_process(tmp_path, monkeypatch):
    write_world(tmp_path)
    server = live_process("gazebo")
    existing = live_process("gazebo_gui", pid=103)
    start_one = Mock()
    wait_for_gui = Mock()
    monkeypatch.setattr(LAUNCHER, "alive", lambda item: True)
    monkeypatch.setattr(LAUNCHER, "start_one", start_one)
    monkeypatch.setattr(LAUNCHER, "wait_for_gui", wait_for_gui)

    items = [server, existing]
    result = LAUNCHER.launch_gui(
        items,
        tmp_path,
        {"GZ_PARTITION": "drone-lab-test", "DISPLAY": ":0"},
    )

    assert result is existing
    assert items == [server, existing]
    start_one.assert_not_called()
    wait_for_gui.assert_called_once()


def test_open_gui_refuses_stopped_server(monkeypatch):
    monkeypatch.setattr(LAUNCHER, "alive", lambda item: True)

    with pytest.raises(RuntimeError, match="服务器未运行"):
        LAUNCHER.open_gui(
            {"stopped": True, "processes": [live_process("gazebo")]}
        )


def test_open_gui_preserves_saved_partition(tmp_path, monkeypatch):
    active = tmp_path / "manager.json"
    state = {
        "run_dir": str(tmp_path),
        "processes": [live_process("gazebo")],
        "gazebo_environment": {
            "GZ_PARTITION": "drone-lab-saved",
            "IGN_PARTITION": "drone-lab-saved",
            "GZ_SIM_RESOURCE_PATH": "/saved/models",
        },
    }
    captured = {}

    def launch_gui(items, run_dir, env):
        captured.update(env)
        return live_process("gazebo_gui", pid=104)

    monkeypatch.setattr(LAUNCHER, "ACTIVE", active)
    monkeypatch.setattr(LAUNCHER, "alive", lambda item: True)
    monkeypatch.setattr(LAUNCHER, "launch_gui", launch_gui)
    save_manager = Mock()
    monkeypatch.setattr(LAUNCHER, "save_manager", save_manager)

    LAUNCHER.open_gui(state)

    assert captured["GZ_PARTITION"] == "drone-lab-saved"
    assert captured["IGN_PARTITION"] == "drone-lab-saved"
    assert captured["GZ_SIM_RESOURCE_PATH"] == "/saved/models"
    save_manager.assert_called_once_with(state)


def test_save_manager_retries_transient_permission_error(tmp_path, monkeypatch):
    active = tmp_path / "manager.json"
    monkeypatch.setattr(LAUNCHER, "ACTIVE", active)
    monkeypatch.setattr(LAUNCHER.time, "sleep", lambda delay: None)
    original_replace = Path.replace
    attempts = 0

    def replace(self, target):
        nonlocal attempts
        if target == active and self.name.endswith(".tmp"):
            attempts += 1
            if attempts <= 3:
                raise PermissionError("manager is being read")
        return original_replace(self, target)

    monkeypatch.setattr(Path, "replace", replace)
    state = {"run_dir": "new", "processes": []}

    LAUNCHER.save_manager(state)

    assert attempts == 4
    assert json.loads(active.read_text(encoding="utf-8")) == state
    assert not list(tmp_path.glob("manager.*.tmp"))


def test_save_manager_preserves_old_state_after_persistent_permission_error(
    tmp_path, monkeypatch
):
    active = tmp_path / "manager.json"
    old_text = json.dumps({"run_dir": "old", "processes": []}, indent=2)
    active.write_text(old_text, encoding="utf-8")
    monkeypatch.setattr(LAUNCHER, "ACTIVE", active)
    monkeypatch.setattr(LAUNCHER.time, "sleep", lambda delay: None)
    original_replace = Path.replace
    attempts = 0

    def replace(self, target):
        nonlocal attempts
        if target == active and self.name.endswith(".tmp"):
            attempts += 1
            raise PermissionError("manager is being read")
        return original_replace(self, target)

    monkeypatch.setattr(Path, "replace", replace)

    with pytest.raises(PermissionError, match="being read"):
        LAUNCHER.save_manager({"run_dir": "new", "processes": []})

    assert attempts == 40
    assert active.read_text(encoding="utf-8") == old_text
    assert not list(tmp_path.glob("manager.*.tmp"))


def test_wait_for_gui_uses_string_msg_v_and_accepts_empty_marker_list(
    tmp_path, monkeypatch
):
    gui_log = tmp_path / "gazebo_gui.log"
    gui_log.write_text("ready", encoding="utf-8")
    gui = live_process("gazebo_gui", pid=105)
    gui["log"] = str(gui_log)
    server = live_process("gazebo")
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if command == ["gz", "service", "-l"]:
            return response(b"/gazebo/worlds\n/marker/list\n")
        if "/gazebo/worlds" in command:
            return response(b'data: "demo"\n')
        return response(b"data: {}\n")

    monkeypatch.setattr(LAUNCHER, "alive", lambda item: True)
    monkeypatch.setattr(LAUNCHER.subprocess, "run", run)

    LAUNCHER.wait_for_gui(gui, server, {"GZ_PARTITION": "drone-lab-test"}, "demo")

    world_call = next(command for command in calls if "/gazebo/worlds" in command)
    assert world_call[world_call.index("--reptype") + 1] == "gz.msgs.StringMsg_V"


def test_wait_for_gui_rejects_world_stderr_even_with_zero_exit(tmp_path, monkeypatch):
    gui_log = tmp_path / "gazebo_gui.log"
    gui_log.write_text("service timeout", encoding="utf-8")
    gui = live_process("gazebo_gui", pid=106)
    gui["log"] = str(gui_log)
    server = live_process("gazebo")
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if command == ["gz", "service", "-l"]:
            return response(b"/gazebo/worlds\n/marker/list\n")
        if "/gazebo/worlds" in command:
            return response(b'data: "demo"\n', b"Service call timed out\n", 0)
        return response(b"data: {}\n")

    monkeypatch.setattr(LAUNCHER, "alive", lambda item: True)
    monkeypatch.setattr(LAUNCHER.subprocess, "run", run)
    clock = iter((0.0, 0.0, 91.0))
    monkeypatch.setattr(LAUNCHER.time, "monotonic", lambda: next(clock))

    with pytest.raises(RuntimeError, match="未就绪"):
        LAUNCHER.wait_for_gui(gui, server, {"GZ_PARTITION": "drone-lab-test"}, "demo")

    assert any(command == ["gz", "service", "-l"] for command in calls)
    assert any("/gazebo/worlds" in command for command in calls)
    assert not any("/marker/list" in command for command in calls)


def test_wait_for_gui_rejects_marker_stderr_even_with_zero_exit(tmp_path, monkeypatch):
    gui_log = tmp_path / "gazebo_gui.log"
    gui_log.write_text("marker service timeout", encoding="utf-8")
    gui = live_process("gazebo_gui", pid=107)
    gui["log"] = str(gui_log)
    server = live_process("gazebo")
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if command == ["gz", "service", "-l"]:
            return response(b"/gazebo/worlds\n/marker/list\n")
        if "/gazebo/worlds" in command:
            return response(b'data: "demo"\n')
        return response(b"", b"Service call timed out\n", 0)

    monkeypatch.setattr(LAUNCHER, "alive", lambda item: True)
    monkeypatch.setattr(LAUNCHER.subprocess, "run", run)
    clock = iter((0.0, 0.0, 91.0))
    monkeypatch.setattr(LAUNCHER.time, "monotonic", lambda: next(clock))

    with pytest.raises(RuntimeError, match="未就绪"):
        LAUNCHER.wait_for_gui(gui, server, {"GZ_PARTITION": "drone-lab-test"}, "demo")

    assert any("/gazebo/worlds" in command for command in calls)
    assert any("/marker/list" in command for command in calls)
