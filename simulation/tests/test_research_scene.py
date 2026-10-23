"""占位场景、共享噪声配置与独立世界生成的回归测试。"""

import json
import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from simulation.research.scene import (
    DEFAULT_SCENARIOS,
    generate,
    load_scenarios,
    make_ground_sampler,
)

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
MAP = ROOT / "maps" / "rmuc_2027.json"
SCENARIOS = DEFAULT_SCENARIOS

PROFILE_FIELDS = (
    "noise_std_m",
    "range_sigma_per_m",
    "bias_bound_m",
    "scale_bound",
    "frame_bias_std_m",
    "drop_probability",
)

# 与 scenarios.json 约定一致的固定剖面；改动这里等同于改动共享接口。
EXPECTED_PROFILES = {
    "ideal": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "nominal": (0.010, 0.005, 0.015, 0.005, 0.005, 0.03),
    "adverse": (0.025, 0.015, 0.030, 0.010, 0.015, 0.10),
}


def write_fake_scene(root):
    """构造不依赖 CAD 资产的最小地图、世界模板与场景文件。

    @param root 临时目录。
    @return (地图路径, 场景路径)。
    """
    (root / "worlds").mkdir(parents=True)
    (root / "terrain").mkdir()
    xs = np.arange(-4.0, 4.0001, 0.1)
    ys = np.arange(-4.0, 4.0001, 0.1)
    zz = np.tile(0.4 + 0.05 * xs, (len(ys), 1))
    np.savez(root / "terrain" / "fake_grid.npz", x=xs, y=ys, z=zz)
    (root / "terrain" / "fake.f32").write_bytes(
        np.zeros(9, dtype="<f4").tobytes()
    )
    world = ET.Element("sdf", version="1.10")
    node = ET.SubElement(world, "world", name="fake_field")
    include = ET.SubElement(node, "include")
    ET.SubElement(include, "uri").text = "model://fake_terrain"
    ET.SubElement(include, "name").text = "fake_terrain"
    ET.ElementTree(world).write(
        root / "worlds" / "fake.sdf", encoding="utf-8", xml_declaration=True
    )
    spawns = [
        {"id": index, "x": x, "y": y, "z": 0.6, "roll": 0, "pitch": 0, "yaw": 0}
        for index, (x, y) in enumerate(((0.0, 0.0), (0.3, 0.0), (-0.3, 0.0), (0.0, 0.3)))
    ]
    map_data = {
        "schema_version": 1,
        "name": "fake_field",
        "season": 2027,
        "world_file": "worlds/fake.sdf",
        "terrain_triangles_file": "terrain/fake.f32",
        "terrain_height_grid": "terrain/fake_grid.npz",
        "bounds": {"x_min": -4.0, "x_max": 4.0, "y_min": -4.0, "y_max": 4.0},
        "initialization": {"type": "configured_spawn_prior", "source": "fake"},
        "spawns": spawns,
        "max_vehicle_count": 4,
    }
    (root / "map.json").write_text(
        json.dumps(map_data, indent=2), encoding="utf-8"
    )
    scenarios = {
        "schema_version": 1,
        "profiles": {
            name: dict(zip(PROFILE_FIELDS, values))
            for name, values in EXPECTED_PROFILES.items()
        },
        "robots": [
            {
                "id": "fake_hero_0",
                "role": "hero",
                "center_m": [2.0, -2.0, 0.0],
                "size_m": [0.6, 0.5, 0.6],
                "velocity_mps": [0.0, 0.0, 0.0],
            },
            {
                "id": "fake_infantry_1",
                "role": "infantry",
                "center_m": [3.0, -2.0, 0.0],
                "size_m": [0.45, 0.4, 0.45],
                "velocity_mps": [0.0, 0.0, 0.0],
            },
        ],
        "research": {
            "default_profile": "nominal",
            "default_groups": ["route"],
            "seed": 2027,
            "scenario_groups": {
                "route": ["fake_hero_0", "fake_infantry_1"],
                "platform": ["fake_platform_0"],
            },
            "pressure_robots": [
                {
                    "id": "fake_platform_0",
                    "role": "engineer",
                    "center_m": [0.0, 0.0, 0.0],
                    "size_m": [0.6, 0.5, 0.6],
                    "velocity_mps": [0.0, 0.0, 0.0],
                    "standing": "non_competition_platform",
                }
            ],
        },
    }
    (root / "scenarios.json").write_text(
        json.dumps(scenarios, indent=2), encoding="utf-8"
    )
    return root / "map.json", root / "scenarios.json"


def test_shared_profiles_match_the_fixed_contract():
    data = load_scenarios(SCENARIOS)
    assert set(data["profiles"]) == set(EXPECTED_PROFILES)
    for name, expected in EXPECTED_PROFILES.items():
        profile = data["profiles"][name]
        assert tuple(profile[field] for field in PROFILE_FIELDS) == expected
        for field in PROFILE_FIELDS:
            assert math.isfinite(profile[field]) and profile[field] >= 0.0
    assert data["profiles"]["ideal"] == dict.fromkeys(PROFILE_FIELDS, 0.0)
    for field in PROFILE_FIELDS:
        assert data["profiles"]["adverse"][field] >= data["profiles"]["nominal"][field]


def test_placeholders_rest_on_the_terrain_grid():
    data = load_scenarios(SCENARIOS)
    base = json.loads(MAP.read_text(encoding="utf-8"))
    ground = make_ground_sampler((MAP.parent / base["terrain_height_grid"]).resolve())
    cruise = base["hover_z_m"]
    assert data["robots"], "shared robot list must not be empty"
    for robot in data["robots"]:
        x, y, z = robot["center_m"]
        height = robot["size_m"][2]
        assert z - height / 2.0 == pytest.approx(ground(x, y), abs=1e-3)
        assert z + height / 2.0 < cruise - 0.5, robot["id"]
        assert base["bounds"]["x_min"] <= x <= base["bounds"]["x_max"]
        assert base["bounds"]["y_min"] <= y <= base["bounds"]["y_max"]
    research = data["research"]
    for robot in research["pressure_robots"]:
        assert robot["standing"] == "non_competition_platform"
        x, y, z = robot["center_m"]
        assert z - robot["size_m"][2] / 2.0 == pytest.approx(ground(x, y), abs=1e-3)
        assert z + robot["size_m"][2] / 2.0 > cruise - 0.3, robot["id"]


def test_generated_world_adds_static_boxes_without_touching_the_prior(tmp_path):
    map_path, scenarios_path = write_fake_scene(tmp_path)
    base = json.loads(map_path.read_text(encoding="utf-8"))
    out = tmp_path / "out"
    report = generate(
        map_path, out, profile_name="adverse", seed=4099, scenarios_path=scenarios_path
    )
    world = ET.parse(out / "world.sdf").getroot().find("world")
    models = {model.get("name"): model for model in world.findall("model")}
    for robot in report["robots"]:
        model = models["unknown_" + robot["id"]]
        assert model.findtext("static") == "true"
        collision = model.find("link/collision/geometry/box/size")
        visual = model.find("link/visual/geometry/box/size")
        assert collision is not None and visual is not None
        assert collision.text == visual.text
        assert [float(value) for value in collision.text.split()] == robot["size_m"]
        pose = [float(value) for value in model.findtext("pose").split()]
        assert pose[:3] == robot["center_m"]
    includes = [node.findtext("uri") for node in world.findall("include")]
    assert includes.count("model://fake_terrain") == 1
    assert sum(1 for uri in includes if uri.startswith("model://micro")) == len(
        base["spawns"]
    )
    scene_map = json.loads((out / "map.json").read_text(encoding="utf-8"))
    assert scene_map["spawns"] == base["spawns"]
    assert scene_map["initialization"] == base["initialization"]
    assert scene_map["terrain_triangles_file"].endswith("fake.f32")
    research = scene_map["research"]
    assert research["profile"] == "adverse"
    assert research["seed"] == 4099
    assert research["groups"] == ["route"]
    assert research["tof_noise"] == load_scenarios(scenarios_path)["profiles"]["adverse"]
    assert [robot["id"] for robot in research["robots"]] == [
        robot["id"] for robot in report["robots"]
    ]
    assert report["files"][:2] == [str(out / "map.json"), str(out / "world.sdf")]
    for path in report["files"]:
        assert Path(path).is_file()


def test_generation_is_byte_for_byte_deterministic(tmp_path):
    map_path, scenarios_path = write_fake_scene(tmp_path)
    generate(map_path, tmp_path / "first", seed=2027, scenarios_path=scenarios_path)
    generate(map_path, tmp_path / "second", seed=2027, scenarios_path=scenarios_path)
    for name in ("map.json", "world.sdf"):
        assert (tmp_path / "first" / name).read_bytes() == (
            tmp_path / "second" / name
        ).read_bytes()


def test_platform_pressure_group_is_marked_and_guarded(tmp_path):
    map_path, scenarios_path = write_fake_scene(tmp_path)
    with pytest.raises(ValueError, match="spawn"):
        generate(
            map_path, tmp_path / "blocked", scenarios_path=scenarios_path,
            groups=["platform"],
        )
    report = generate(
        map_path, tmp_path / "allowed", scenarios_path=scenarios_path,
        groups=["platform"], allow_spawn_overlap=True,
    )
    assert report["spawn_overlap"]
    assert all(
        robot["standing"] == "non_competition_platform" for robot in report["robots"]
    )
    scene_map = json.loads((tmp_path / "allowed" / "map.json").read_text(encoding="utf-8"))
    assert scene_map["research"]["groups"] == ["platform"]


HEADER_PROGRAM = """
#include "simulation/research/tof_error_model.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

using namespace drone_sim::research;

int main()
{
  std::array<double, kTofPixelCount> depth{};
  std::array<bool, kTofPixelCount> source{};
  for (int i = 0; i < kTofPixelCount; ++i)
  {
    depth[i] = 1.5;
    source[i] = true;
  }
  TofEdgePolicy edge;
  TofNoiseProfile ideal;
  ideal.noise_std_m = 0.0;
  ideal.drop_probability = 0.0;
  const auto identity = ApplyTofFrame(ideal, edge, 2027, 0, 0, 1000, depth, source);
  int identical = 0;
  for (int i = 0; i < kTofPixelCount; ++i)
    identical += identity.valid[i] && identity.distance_m[i] == 1.5 ? 1 : 0;
  std::printf("ideal_identity=%d\\n", identical);

  TofNoiseProfile nominal;
  nominal.noise_std_m = 0.01;
  nominal.range_sigma_per_m = 0.005;
  nominal.bias_bound_m = 0.015;
  nominal.scale_bound = 0.005;
  nominal.frame_bias_std_m = 0.005;
  nominal.drop_probability = 0.03;
  std::printf("profile_valid=%d\\n", TofProfileIsValid(nominal) ? 1 : 0);

  const auto first = ApplyTofFrame(nominal, edge, 2027, 0, 0, 1000, depth, source);
  const auto second = ApplyTofFrame(nominal, edge, 2027, 0, 0, 1000, depth, source);
  const auto later = ApplyTofFrame(nominal, edge, 2027, 0, 0, 2000, depth, source);
  int same = 1;
  int frame_differs = 0;
  int finite = 1;
  for (int i = 0; i < kTofPixelCount; ++i)
  {
    if (first.distance_m[i] != second.distance_m[i] || first.valid[i] != second.valid[i])
      same = 0;
    if (first.distance_m[i] != later.distance_m[i])
      frame_differs = 1;
    if (!std::isfinite(first.distance_m[i]) || first.distance_m[i] < 0.0)
      finite = 0;
    if (first.valid[i] && first.distance_m[i] <= 0.0)
      finite = 0;
  }
  std::printf("deterministic=%d\\n", same);
  std::printf("frame_varies=%d\\n", frame_differs);
  std::printf("finite_nonnegative=%d\\n", finite);

  TofNoiseProfile far;
  far.drop_probability = 0.0;
  const TofNoiseDevice device = MakeTofNoiseDevice(far, 2027, 0, 0);
  const TofFrameError frame = MakeTofFrameError(far, 2027, 0, 0, 1000);
  const auto beyond = ApplyTofPixel(far, device, frame, 6.0, 0);
  const auto clamped = ApplyTofPixel(far, device, frame, 6.0, 0, kDefaultTofMinRangeM,
                                     kDefaultTofMaxRangeM, TofRangePolicy::kClamp);
  std::printf("out_of_range_invalid=%d\\n", beyond.valid ? 0 : 1);
  std::printf("clamp_policy=%d\\n",
              (clamped.valid && std::abs(clamped.distance_m - kDefaultTofMaxRangeM) < 1e-12)
                  ? 1
                  : 0);

  TofNoiseProfile bad = far;
  bad.noise_std_m = -1.0;
  std::printf("negative_rejected=%d\\n", TofProfileIsValid(bad) ? 0 : 1);
  std::printf("default_noise_std_m=%.3f\\n", TofNoiseProfile{}.noise_std_m);
  std::printf("default_drop_probability=%.2f\\n", TofNoiseProfile{}.drop_probability);
  return 0;
}
"""


def test_tof_error_model_header_is_compilable_and_deterministic(tmp_path):
    compiler = shutil.which("g++") or shutil.which("c++")
    if compiler is None:
        pytest.skip("no C++ compiler available")
    source = tmp_path / "tof_error_model_check.cc"
    source.write_text(HEADER_PROGRAM, encoding="utf-8")
    binary = tmp_path / ("tof_error_model_check.exe" if os.name == "nt" else "tof_error_model_check")
    build = subprocess.run(
        [compiler, "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-I", str(REPO),
         "-o", str(binary), str(source)],
        capture_output=True, text=True,
    )
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(binary)], capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    values = dict(line.split("=") for line in run.stdout.split())
    assert values["ideal_identity"] == "64"
    assert values["profile_valid"] == "1"
    assert values["deterministic"] == "1"
    assert values["frame_varies"] == "1"
    assert values["finite_nonnegative"] == "1"
    assert values["out_of_range_invalid"] == "1"
    assert values["clamp_policy"] == "1"
    assert values["negative_rejected"] == "1"
    assert values["default_noise_std_m"] == "0.003"
    assert values["default_drop_probability"] == "0.03"
