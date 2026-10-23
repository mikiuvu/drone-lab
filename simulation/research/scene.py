# @file scene.py
# @brief 由 RMUC2027 地图与 scenarios.json 生成含未知机器人占位的独立 SDF 世界。

"""生成含未知机器人占位的 RMUC2027 独立世界，并输出可复现的噪声配置。"""

import argparse
import copy
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

try:
    from simulation.launch_linux import prepare_world
except ImportError:
    from launch_linux import prepare_world

DEFAULT_SCENARIOS = Path(__file__).resolve().parent / "scenarios.json"
ROBOT_PREFIX = "unknown_"
# 角色配色只用于可视化区分，不参与任何测量。
ROLE_COLORS = {
    "hero": ("0.62 0.18 0.18 1", "0.78 0.26 0.24 1"),
    "infantry": ("0.16 0.34 0.62 1", "0.24 0.46 0.78 1"),
    "engineer": ("0.58 0.46 0.12 1", "0.76 0.62 0.18 1"),
    "sentry": ("0.22 0.52 0.30 1", "0.30 0.66 0.40 1"),
}
DEFAULT_COLOR = ("0.45 0.45 0.48 1", "0.58 0.58 0.60 1")
# 出生机体的最大半尺寸，用于判断占位是否压在出生点上。
SPAWN_HALF_EXTENT_M = 0.06


def load_scenarios(path=DEFAULT_SCENARIOS):
    """读取并校验 scenarios.json 的固定字段。

    @param path 场景文件路径。
    @return 解析后的字典。
    """
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("profiles", "robots"):
        if key not in data:
            raise ValueError(f"scenarios file is missing {key!r}: {path}")
    for name, profile in data["profiles"].items():
        missing = [
            field
            for field in (
                "noise_std_m",
                "range_sigma_per_m",
                "bias_bound_m",
                "scale_bound",
                "frame_bias_std_m",
                "drop_probability",
            )
            if field not in profile
        ]
        if missing:
            raise ValueError(f"profile {name!r} is missing {missing}")
    for robot in data["robots"]:
        for key in ("id", "role", "center_m", "size_m"):
            if key not in robot:
                raise ValueError(f"robot entry is missing {key!r}: {robot.get('id')}")
    return data


def make_ground_sampler(grid_path):
    """构造按最近网格节点查询地面高度的函数。

    @param grid_path terrain_height_grid.npz 路径。
    @return 接受 (x, y) 返回地面 z（m）的函数。
    """
    data = np.load(grid_path, allow_pickle=False)
    xs, ys, zs = data["x"], data["y"], data["z"]

    def ground(x, y):
        column = int(np.argmin(np.abs(xs - x)))
        row = int(np.argmin(np.abs(ys - y)))
        value = float(zs[row, column])
        if not math.isfinite(value):
            raise ValueError(f"terrain grid has no height at ({x}, {y})")
        return value

    return ground


def resolve_robots(scenarios, groups, ground):
    """按分组选取占位，并用地形高度把每个箱体落到地面。

    @param scenarios 已解析的 scenarios.json。
    @param groups 需要纳入的场景分组名序列。
    @param ground 地面高度查询函数。
    @return 机器人列表，center_m 的 z 为底面贴地的几何中心。
    """
    research = scenarios.get("research", {})
    available = dict(research.get("scenario_groups", {}))
    if not available:
        available = {"all": [robot["id"] for robot in scenarios["robots"]]}
    catalogue = {}
    for robot in list(scenarios["robots"]) + list(research.get("pressure_robots", [])):
        catalogue[robot["id"]] = robot
    selected = []
    seen = set()
    for group in groups:
        if group not in available:
            raise ValueError(
                f"unknown scenario group {group!r}; available: {sorted(available)}"
            )
        for identifier in available[group]:
            if identifier in seen:
                continue
            seen.add(identifier)
            entry = catalogue.get(identifier)
            if entry is None:
                raise ValueError(f"group {group!r} references unknown robot {identifier!r}")
            selected.append(entry)
    robots = []
    for entry in selected:
        size = [float(value) for value in entry["size_m"]]
        if len(size) != 3 or any(value <= 0.0 for value in size):
            raise ValueError(f"robot {entry['id']!r} has an invalid size_m")
        center = [float(entry["center_m"][0]), float(entry["center_m"][1])]
        center.append(round(ground(center[0], center[1]) + size[2] / 2.0, 4))
        robot = {
            "id": entry["id"],
            "role": entry["role"],
            "center_m": center,
            "size_m": size,
            "velocity_mps": [float(v) for v in entry.get("velocity_mps", (0.0, 0.0, 0.0))],
        }
        if "standing" in entry:
            robot["standing"] = entry["standing"]
        robots.append(robot)
    return robots


def spawn_overlaps(robots, spawns, half_extent=SPAWN_HALF_EXTENT_M):
    """检查占位箱体是否覆盖出生机体所在空间。

    @param robots 机器人列表。
    @param spawns 地图出生点列表。
    @param half_extent 出生机体在三个方向上的半尺寸（m）。
    @return (robot_id, spawn_id) 列表。
    """
    hits = []
    for robot in robots:
        center = robot["center_m"]
        half = [value / 2.0 for value in robot["size_m"]]
        for spawn in spawns:
            if (
                abs(spawn["x"] - center[0]) <= half[0] + half_extent
                and abs(spawn["y"] - center[1]) <= half[1] + half_extent
                and abs(spawn["z"] - center[2]) <= half[2] + half_extent
            ):
                hits.append((robot["id"], spawn["id"]))
    return hits


def add_robot_model(world, robot):
    """把一个占位机器人写成静态箱体模型，同时提供 collision 与 visual。

    @param world 世界 XML 元素。
    @param robot 机器人条目。
    @return 模型名称。
    """
    name = ROBOT_PREFIX + robot["id"]
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(
        f"{value:.6g}" for value in list(robot["center_m"]) + [0.0, 0.0, 0.0]
    )
    link = ET.SubElement(model, "link", name="body")
    size = " ".join(f"{value:.6g}" for value in robot["size_m"])
    ambient, diffuse = ROLE_COLORS.get(robot.get("role"), DEFAULT_COLOR)
    for tag in ("collision", "visual"):
        node = ET.SubElement(link, tag, name=tag)
        box = ET.SubElement(ET.SubElement(node, "geometry"), "box")
        ET.SubElement(box, "size").text = size
        if tag == "visual":
            material = ET.SubElement(node, "material")
            ET.SubElement(material, "ambient").text = ambient
            ET.SubElement(material, "diffuse").text = diffuse
    return name


def insert_robot_models(world_path, robots):
    """把占位模型插入已生成的世界，重复调用不会产生重名模型。

    @param world_path 世界 SDF 路径。
    @param robots 机器人列表。
    @return 本次新增的模型名称列表。
    """
    world_path = Path(world_path)
    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{world_path} has no <world> element")
    existing = {model.get("name") for model in world.findall("model")}
    added = []
    for robot in robots:
        if ROBOT_PREFIX + robot["id"] in existing:
            continue
        added.append(add_robot_model(world, robot))
    if added:
        ET.indent(tree)
        tree.write(world_path, encoding="utf-8", xml_declaration=True)
    return added


def atomic_write_json(path, payload):
    """整文件写入后重命名，避免读到半份配置。

    @param path 目标路径。
    @param payload 可序列化对象。
    """
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def build_scene_map(base_map, map_path, robots, profile_name, profile, seed, groups,
                    grid_path, scenarios_path=DEFAULT_SCENARIOS):
    """在基础地图上附加本次研究的机器人、剖面与种子。

    @param base_map 基础地图字典。
    @param map_path 基础地图路径，用于解析相对资源路径。
    @param robots 已落地的机器人列表。
    @param profile_name 剖面名称。
    @param profile 剖面参数。
    @param seed 随机种子。
    @param groups 本次启用的场景分组。
    @param grid_path 地形高度网格绝对路径。
    @param scenarios_path 本次采用的 scenarios.json 路径。
    @return 可直接交给 prepare_world 的地图字典。
    """
    scene_map = copy.deepcopy(base_map)
    scene_map["world_file"] = str((map_path.parent / base_map["world_file"]).resolve())
    if "terrain_triangles_file" in base_map:
        scene_map["terrain_triangles_file"] = str(
            (map_path.parent / base_map["terrain_triangles_file"]).resolve()
        )
    scene_map["terrain_height_grid"] = str(grid_path)
    scene_map["research"] = {
        "scenarios": str(Path(scenarios_path).resolve()),
        "profile": profile_name,
        "seed": seed,
        "groups": list(groups),
        "robots": robots,
        "tof_noise": dict(profile),
        "standing_note": "占位箱体底面贴地形高度网格；兵种包络为工程近似尺寸。",
    }
    return scene_map


def generate(map_path, output_dir, profile_name=None, seed=None, groups=None,
             scenarios_path=DEFAULT_SCENARIOS, allow_spawn_overlap=False):
    """生成 map.json 与 world.sdf。

    @param map_path 基础地图 JSON 路径。
    @param output_dir 输出目录。
    @param profile_name 剖面名称；None 时取 scenarios.json 的默认剖面。
    @param seed 随机种子；None 时取 scenarios.json 的默认种子。
    @param groups 场景分组；None 时取 scenarios.json 的默认分组。
    @param scenarios_path scenarios.json 路径。
    @param allow_spawn_overlap 允许占位箱体覆盖出生点。
    @return 结果摘要字典。
    """
    map_path = Path(map_path).resolve()
    output_dir = Path(output_dir).resolve()
    base_map = json.loads(map_path.read_text(encoding="utf-8"))
    scenarios = load_scenarios(scenarios_path)
    research = scenarios.get("research", {})
    profile_name = profile_name or research.get("default_profile", "nominal")
    if profile_name not in scenarios["profiles"]:
        raise ValueError(
            f"unknown profile {profile_name!r}; available: {sorted(scenarios['profiles'])}"
        )
    profile = scenarios["profiles"][profile_name]
    seed = int(research.get("seed", 2027) if seed is None else seed)
    groups = list(research.get("default_groups", ["route"]) if groups is None else groups)
    grid_path = (map_path.parent / base_map["terrain_height_grid"]).resolve()
    ground = make_ground_sampler(grid_path)
    robots = resolve_robots(scenarios, groups, ground)
    overlaps = spawn_overlaps(robots, base_map["spawns"])
    if overlaps and not allow_spawn_overlap:
        raise ValueError(
            "robot placeholders overlap vehicle spawn envelopes: "
            f"{overlaps}; select a group without platform standing or pass "
            "--allow-spawn-overlap"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    scene_map = build_scene_map(
        base_map, map_path, robots, profile_name, profile, seed, groups, grid_path,
        scenarios_path,
    )
    map_out = output_dir / "map.json"
    world_out = output_dir / "world.sdf"
    atomic_write_json(map_out, scene_map)
    world_name, config = prepare_world(world_out, map_out)
    inserted = insert_robot_models(world_out, robots)
    report = {
        "map": str(map_out),
        "world": str(world_out),
        "world_name": world_name,
        "profile": profile_name,
        "profile_values": dict(profile),
        "seed": seed,
        "groups": groups,
        "vehicles": len(config["spawns"]),
        "robots": robots,
        "models": inserted,
        "spawn_overlap": overlaps,
        "files": [str(map_out), str(world_out)],
    }
    report_path = output_dir / "scene_report.json"
    atomic_write_json(report_path, report)
    report["files"].append(str(report_path))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, type=Path, help="基础地图 JSON")
    parser.add_argument("--output-dir", required=True, type=Path, help="输出目录")
    parser.add_argument("--profile", choices=["ideal", "nominal", "adverse"])
    parser.add_argument("--seed", type=int)
    parser.add_argument("--groups", help="逗号分隔的场景分组，默认取 scenarios.json")
    parser.add_argument("--scenarios", type=Path, default=DEFAULT_SCENARIOS)
    parser.add_argument("--allow-spawn-overlap", action="store_true")
    args = parser.parse_args(argv)
    groups = None
    if args.groups:
        groups = [item.strip() for item in args.groups.split(",") if item.strip()]
    report = generate(
        args.map,
        args.output_dir,
        profile_name=args.profile,
        seed=args.seed,
        groups=groups,
        scenarios_path=args.scenarios,
        allow_spawn_overlap=args.allow_spawn_overlap,
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
