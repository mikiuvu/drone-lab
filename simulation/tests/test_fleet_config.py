import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from simulation.fleet_config import configure_fleet
from simulation.launch_linux import prepare_world

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("count", [1, 4, 6])
@pytest.mark.parametrize("anchors", [False, True])
def test_count_and_sensor_contract(count, anchors, tmp_path):
    path = ROOT / "maps/rmuc_2026_sentry26.json"
    original = json.loads(path.read_text(encoding="utf-8"))
    config = configure_fleet(original, count, anchors)
    assert len(original["spawns"]) == 4
    assert len(config["spawns"]) == count
    assert config["team_mission"]["order"] == list(range(count))
    assert len(config["team_mission"]["assembly"]) == count
    _, saved = prepare_world(tmp_path / "world.sdf", path, config)
    world = ET.parse(tmp_path / "world.sdf").getroot().find("world")
    models = [
        i
        for i in world.findall("include")
        if i.findtext("name", "").startswith("micro_")
    ]
    assert len(models) == count
    plugin = world.find("plugin[@name='drone_sim::PeerHeightSystem']")
    assert int(plugin.findtext("vehicle_count")) == count
    assert plugin.findtext("flow_enabled") == "true"
    assert int(plugin.findtext("anchor_count")) == (4 if anchors else 0)
    assert len(saved["vehicle_waypoints"]) == count


def test_custom_large_fleet_has_no_four_vehicle_limit():
    count = 80
    config = {"spawns": [{"id": i, "x": i * 0.4, "y": 0, "z": 0.5} for i in range(count)]}
    assert len(configure_fleet(config)["spawns"]) == count


def test_explicit_anchor_positions_preserved():
    anchor = {"id": 42, "x": 1, "y": 2, "z": 3}
    config = {"spawns": [{"id": 0, "x": 0, "y": 0, "z": 0.5}], "uwb_anchors": [anchor]}
    assert configure_fleet(config, enable_anchors=True)["uwb_anchors"] == [anchor]
    assert configure_fleet(config)["uwb_anchors"] == [anchor]
    assert configure_fleet(config, enable_anchors=False)["uwb_anchors"] == []


@pytest.mark.parametrize("count", [0, -1, True, 2.5])
def test_invalid_cardinality_rejected(count):
    with pytest.raises(ValueError):
        configure_fleet({"spawns": [{"id": 0, "x": 0, "y": 0, "z": 0.5}]}, count)


def test_map_hangar_capacity_rejects_extra_vehicles():
    config = {"max_vehicle_count": 1, "spawns": [{"id": 0, "x": 0, "y": 0, "z": 1.6}]}
    with pytest.raises(ValueError, match="at most 1"):
        configure_fleet(config, 2)
    assert len(configure_fleet(config)["spawns"]) == 1
