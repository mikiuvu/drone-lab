"""Supervisor validation without starting or arming a vehicle."""

import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "drone_launcher", ROOT / "simulation/launch_linux.py"
)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


def test_world_uses_four_independent_models_and_height_plugin(tmp_path):
    world_name, config = launcher.prepare_world(
        tmp_path / "world.sdf", ROOT / "simulation/maps/calibration.json"
    )
    assert (
        world_name == "calibration_not_official_field" and not config["official_field"]
    )
    world = ET.parse(tmp_path / "world.sdf").getroot().find("world")
    names = [node.findtext("name") for node in world.findall("include")]
    assert names == ["micro_0", "micro_1", "micro_2", "micro_3"]
    assert [n.findtext("uri") for n in world.findall("include")] == [
        f"model://micro_{i}" for i in range(4)
    ]
    plugins = [
        p
        for p in world.findall("plugin")
        if p.get("filename") == "libpeer_height_system.so"
    ]
    assert len(plugins) == 1 and plugins[0].findtext("publish_hz") == "50"


def test_micro_camera_not_below_contact_plane():
    for directory in ("generated_micro", "generated_micro_horizontal"):
        manifest = json.loads(
            (ROOT / "simulation/models" / directory / "sensor_contract.json").read_text(
                encoding="utf-8"
            )
        )
        assert manifest["mass_total_kg"] == 0.18
        assert manifest["camera_origin_body_m"][2] >= 0
        assert all(p[2] >= 0 for p in manifest["tof_origins_body_m"])
        assert "uncalibrated" in manifest["dynamics"]
        for i in range(4):
            params = manifest["px4_parameters"]
            ct = params[f"CA_ROTOR{i}_CT"]
            idle = params[f"SIM_GZ_EC_MIN{i + 1}"]
            maximum = params[f"SIM_GZ_EC_MAX{i + 1}"]
            assert idle > 0  # Gazebo marks zero-RPM ESCs unarmed.
            assert (
                abs(
                    idle
                    + (maximum - idle) * params["MPC_THR_HOVER"]
                    - manifest["hover_speed_rad_s"]
                )
                < 1e-8
            )
            assert abs(ct * params[f"CA_ROTOR{i}_KM"]) > 0.05
            assert abs(ct * params[f"CA_ROTOR{i}_PX"]) > 0.05
            assert abs(ct * params[f"CA_ROTOR{i}_PY"]) > 0.05


def test_pid_not_found_does_not_kill_random_process():
    assert launcher.alive({"pid": 99999999, "start_ticks": "0"}) is False
