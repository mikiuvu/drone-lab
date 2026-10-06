import xml.etree.ElementTree as ET
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from simulation.contact_evaluation import PressureSwitch, armor_contact
from simulation.moving_armor import add_moving_armor
from simulation.base_mesh import split_region


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
    assert float(model.findtext("joint[@name='armor_slide']/axis/limit/upper")) == 0.28
    assert model.find(".//visual[@name='green_dart_guide']") is not None
    assert not model.findall(".//visual[@name='original_base_logo_white']")
    assert model.findtext(".//collision[@name='armor_plate']/geometry/mesh/uri").endswith(
        "armor_hit_surface.stl"
    )
    assert model.find(".//collision[@name='carriage_housing']") is not None

    # 分色只裁分原三角形，三种材质合并后的面积应与原滑台完全相同。
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    def read(name):
        return np.fromfile(tmp_path / name, dtype=dtype, offset=84)["vertices"]
    def area(triangles):
        return np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                       triangles[:, 2] - triangles[:, 0]), axis=1).sum() / 2
    blue = read("carriage_blue_surfaces.stl")
    green = read("carriage_green_guide.stl")
    body = read("carriage_body_surfaces.stl")
    assert np.isclose(area(blue) + area(green) + area(body),
                      area(read("base_original_carriage.stl")), rtol=2e-5)
    assert np.isclose(area(blue), 2 * .055 * .005, rtol=2e-4)
    assert np.isclose(np.ptp(green[:, :, 1]), .055, atol=1e-6)
    specs = json.loads((source.parents[3] / "original_armor_faces.json").read_text(encoding="utf-8"))
    center, up, normal = (np.asarray(specs[key]) for key in ("center_m", "panel_up", "panel_normal"))
    for side in (-1, 1):
        strip = blue[(blue[:, :, 1].mean(axis=1) - center[1]) * side > 0]
        assert np.isclose(np.ptp(strip @ up), .055, atol=2e-6)
        assert np.isclose(np.ptp(strip[:, :, 1]), .005, atol=2e-6)
        assert np.allclose((strip - center) @ normal, 0, atol=5e-6)


def test_surface_clip_preserves_geometry_and_winding():
    original = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
    inside, outside = split_region(original, [(np.array([1., 0., 0.]), .25),
                                              (np.array([0., 1., 0.]), .25)])
    combined = np.concatenate([inside, outside])
    normals = np.cross(combined[:, 1] - combined[:, 0], combined[:, 2] - combined[:, 0])
    assert np.all(normals[:, 2] > 0)
    assert np.isclose(normals[:, 2].sum() / 2, .5)
    assert np.all(inside[:, :, :2] >= .25 - 1e-9)


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
    assert not armor_contact([contact("enemy_base_moving_armor::armor::carriage_housing")])
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
