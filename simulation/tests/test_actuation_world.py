"""Optional actuation telemetry preserves model references and vehicle placement."""

import xml.etree.ElementTree as ET

from simulation.launch_linux import prepare_world


def world_config(tmp_path):
    template = tmp_path / "template.sdf"
    template.write_text('<sdf version="1.9"><world name="actuation_test"/></sdf>', encoding="utf-8")
    return {"world_file": str(template), "spawns": [
        {"id": i, "x": i + .25, "y": -.5, "z": .8, "roll": 0, "pitch": 0, "yaw": .3}
        for i in range(2)]}


def test_actuation_recording_is_disabled_by_default(tmp_path):
    output = tmp_path / "world.sdf"
    prepare_world(output, tmp_path / "map.json", config=world_config(tmp_path))
    includes = ET.parse(output).findall(".//include")
    assert len(includes) == 2
    assert all(not item.findall("plugin") for item in includes)
    assert not list(tmp_path.glob("actuation_*.jsonl"))


def test_actuation_recording_is_per_vehicle_without_changing_placement(tmp_path):
    output = tmp_path / "world.sdf"
    config = world_config(tmp_path)
    prepare_world(output, tmp_path / "map.json", config=config, actuation_log=True)
    includes = ET.parse(output).findall(".//include")
    paths = []
    for include, spawn in zip(includes, config["spawns"]):
        assert include.findtext("uri") == f"model://micro_{spawn['id']}"
        assert [float(value) for value in include.findtext("pose").split()] == [
            spawn[key] for key in ("x", "y", "z", "roll", "pitch", "yaw")]
        recorder, = include.findall("plugin")
        assert recorder.get("name") == "drone_sim::ActuationRecorder"
        assert recorder.findtext("vehicle") == str(spawn["id"])
        assert recorder.findtext("publish_hz") == "20"
        path = recorder.findtext("output")
        assert path == str((tmp_path / f"actuation_{spawn['id']}.jsonl").resolve())
        paths.append(path)
    assert len(set(paths)) == 2
