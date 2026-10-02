"""Static-prior consistency checks; no synthetic vehicle flight evidence."""

import hashlib
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / "models/terrain/rmuc_2026_sentry26"


def test_render_collision_and_estimator_use_identical_terrain():
    stl = (ASSET / "terrain.stl").read_bytes()
    n = struct.unpack_from("<I", stl, 80)[0]
    dtype = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
    visible = np.frombuffer(stl, dtype=dtype, offset=84)["v"]
    prior = np.fromfile(ASSET / "terrain_triangles.f32", dtype="<f4").reshape(-1, 3, 3)
    assert len(visible) == n == len(prior)
    np.testing.assert_array_equal(visible, prior)
    model = ET.parse(ASSET / "model.sdf")
    assert model.findtext(".//visual/geometry/mesh/uri") == model.findtext(
        ".//collision/geometry/mesh/uri"
    )
    provenance = json.loads((ASSET / "provenance.json").read_text(encoding="utf-8"))
    assert hashlib.sha256(stl).hexdigest() == provenance["derived_sha256"]
    assert (
        provenance["original_triangle_count"]
        == n + provenance["excluded_triangle_count"]
    )
    assert np.max(prior[:, :, 2]) < 5
    assert provenance["season"] == 2026 and not provenance["official_2027_geometry"]


def test_four_initial_positions_clear_static_mesh_and_separate():
    config = json.loads(
        (ROOT / "maps/rmuc_2026_sentry26.json").read_text(encoding="utf-8")
    )
    triangles = np.fromfile(ASSET / "terrain_triangles.f32", dtype="<f4").reshape(
        -1, 3, 3
    )
    ground_z = config["home_marker"]["ground_z_m"]
    marker_size = config["home_marker"]["size_m"]
    for spawn in config["spawns"]:
        assert ground_z + 0.0125 < spawn["z"]
        assert abs(spawn["x"]) < 1 and abs(spawn["y"]) < 1
        low = np.array([spawn["x"] - 0.05, spawn["y"] - 0.05, ground_z + 0.04])
        high = np.array(
            [
                spawn["x"] + 0.05,
                spawn["y"] + 0.05,
                config["team_mission"]["cruise_z_m"] + 0.03,
            ]
        )
        overlap = (triangles.max(axis=1) >= low).all(axis=1) & (
            triangles.min(axis=1) <= high
        ).all(axis=1)
        assert not overlap.any(), (
            f"Home {spawn['id']} intersects overhead or support geometry"
        )
        for peer in config["spawns"]:
            if peer["id"] != spawn["id"]:
                assert (
                    np.hypot(spawn["x"] - peer["x"], spawn["y"] - peer["y"])
                    > marker_size
                )


def test_home_markers_are_visual_only_and_match_spawn_coordinates(tmp_path):
    from simulation.launch_linux import prepare_world

    _, config = prepare_world(
        tmp_path / "world.sdf", ROOT / "maps/rmuc_2026_sentry26.json"
    )
    world = ET.parse(tmp_path / "world.sdf").getroot().find("world")
    for spawn in config["spawns"]:
        marker = world.find(f"model[@name='home_{spawn['id']}']")
        assert marker is not None and not marker.findall(".//collision")
        values = [float(v) for v in marker.findtext("pose").split()]
        assert values[:2] == [spawn["x"], spawn["y"]]
        assert len(marker.findall(".//visual")) == 4


def test_micro_protective_frame_matches_scale_and_landing_clearance():
    scale = json.loads(
        (ROOT / "docs/scale-verification.json").read_text(encoding="utf-8")
    )
    mission = json.loads(
        (ROOT / "maps/rmuc_2026_sentry26.json").read_text(encoding="utf-8")
    )["team_mission"]
    for folder in ("generated_micro", "generated_micro_horizontal"):
        model = ET.parse(ROOT / "models" / folder / "micro_0/model.sdf")
        shape = model.find(
            ".//link[@name='base_link']/collision[@name='body_collision']/geometry/cylinder"
        )
        radius = float(shape.findtext("radius"))
        height = float(shape.findtext("length"))
        np.testing.assert_allclose(
            scale["vehicle_extent_m"], [radius * 2, radius * 2, height]
        )
        assert mission["body_clearance_m"] == height / 2
        assert mission["contact_radius_m"] == radius
        assert (
            abs(sum(float(x.text) for x in model.findall(".//inertial/mass")) - 0.18)
            < 1e-10
        )
        assert max(scale["vehicle_extent_m"]) < scale["mass_derived_extent_limit_m"]
