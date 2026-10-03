import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from simulation.contact_evaluation import PressureSwitch, armor_contact
from simulation.moving_armor import add_moving_armor


def test_plate_has_independent_collision_and_slide(tmp_path):
    world = ET.Element("world", name="test")
    source = (
        Path(__file__).resolve().parents[1]
        / "models/terrain/rmuc_2026_sentry26/terrain_triangles.f32"
    )
    config = {"moving_armor": {"enabled": True}}
    model = add_moving_armor(world, config, tmp_path, source)
    original = np.fromfile(source, dtype="<f4").reshape(-1, 3, 3)
    report = config["moving_armor"]["partition"]
    assert report["moving_faces"] + report["fixed_faces"] == len(original)
    assert not model.findall(".//geometry/box")
    assert len(world.findall("model")) == 2
    assert model.findtext("static") == "false"
    assert model.find("link[@name='armor']/collision[@name='armor_plate']") is not None
    assert model.findtext("joint[@name='armor_slide']/axis/xyz") == "0 1 0"
    assert float(model.findtext("joint[@name='armor_slide']/axis/limit/effort")) > 0
    assert (
        model.findtext(
            ".//collision[@name='armor_plate']/surface/contact/collide_bitmask"
        )
        == "2"
    )


def test_base_and_rail_collisions_are_not_armor_hits():
    def contact(name):
        return SimpleNamespace(
            collision1=SimpleNamespace(name=name),
            collision2=SimpleNamespace(name="micro_0::base_link::body_collision"),
        )

    assert not armor_contact([contact("terrain::collision")])
    assert not armor_contact(
        [contact("enemy_base_moving_armor::rail::rail_collision_0")]
    )
    assert armor_contact([contact("enemy_base_moving_armor::armor::armor_plate")])


def test_pressure_switch_counts_contacts_not_frames():
    switch = PressureSwitch()
    assert switch.update(0, 1000000, False) is None
    assert switch.update(0, 1000000, True)["sequence"] == 1
    assert switch.update(0, 1050000, True) is None
    assert switch.update(0, 1040000, True) is None
    assert switch.update(0, 1100000, True) is None
    assert switch.update(0, 1400000, True)["sequence"] == 2
    assert switch.update(1, 1400000, True)["sequence"] == 1
