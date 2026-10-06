"""基地顶部飞镖检测模块沿原滑轨移动，小装甲板正面提供触压反馈。"""

import xml.etree.ElementTree as ET
from pathlib import Path

try:
    from simulation.base_mesh import partition
except ModuleNotFoundError:
    from base_mesh import partition


def node(parent, tag, text=None, **attrs):
    result = ET.SubElement(parent, tag, attrs)
    if text is not None:
        result.text = str(text)
    return result


def add_moving_armor(world, config, output=None, source=None):
    if not config.get("moving_armor", {}).get("enabled", False):
        return None
    if output is None or source is None:
        raise ValueError("原模型分离需要输出目录和源三角网格")
    output = Path(output).resolve()
    report = partition(source, output)
    for include in list(world.findall("include")):
        if "rmuc_2026_sentry26" in (include.findtext("uri") or ""):
            world.remove(include)
    fixed = node(world, "model", name="rmuc_2026_sentry26")
    node(fixed, "static", "true")
    link = node(fixed, "link", name="terrain")

    def mesh(parent, tag, name, path):
        item = node(parent, tag, name=name)
        node(node(node(item, "geometry"), "mesh"), "uri", str(path))
        return item

    visual = mesh(link, "visual", "terrain_visual", output / "terrain_fixed.stl")
    material = node(visual, "material")
    node(material, "ambient", "0.42 0.48 0.50 1")
    node(material, "diffuse", "0.55 0.61 0.63 1")
    fixed_collision = mesh(
        link, "collision", "terrain_collision", output / "terrain_fixed.stl"
    )
    node(node(node(fixed_collision, "surface"), "contact"), "collide_bitmask", 1)
    model = node(world, "model", name="enemy_base_moving_armor")
    node(model, "static", "false")
    anchor = node(model, "link", name="original_rail_anchor")
    node(anchor, "gravity", "false")
    joint = node(model, "joint", name="world_anchor", type="fixed")
    node(joint, "parent", "world")
    node(joint, "child", "original_rail_anchor")
    armor = node(model, "link", name="armor")
    node(armor, "gravity", "false")
    inertial = node(armor, "inertial")
    node(inertial, "pose", "11.58 0.0012 1.48 0 0 0")
    node(inertial, "mass", 0.2)
    inertia = node(inertial, "inertia")
    for key in ("ixx", "iyy", "izz"):
        node(inertia, key, 0.002)
    for key in ("ixy", "ixz", "iyz"):
        node(inertia, key, 0)
    visual = mesh(
        armor, "visual", "original_carriage", output / "carriage_body_surfaces.stl"
    )
    material = node(visual, "material")
    node(material, "ambient", "0.025 0.028 0.032 1")
    node(material, "diffuse", "0.06 0.065 0.07 1")
    visual = mesh(
        armor, "visual", "original_blue_surfaces", output / "carriage_blue_surfaces.stl"
    )
    material = node(visual, "material")
    for key in ("ambient", "diffuse", "emissive"):
        node(material, key, "0 0.03 1 1")
    visual = mesh(armor, "visual", "green_dart_guide", output / "carriage_green_guide.stl")
    material = node(visual, "material")
    for key in ("ambient", "diffuse", "emissive"):
        node(material, key, "0.01 1 0.01 1")
    plate_collision = mesh(
        armor, "collision", "armor_plate", output / "armor_hit_surface.stl"
    )
    housing = mesh(armor, "collision", "carriage_housing", output / "carriage_housing.stl")
    node(node(node(housing, "surface"), "contact"), "collide_bitmask", 2)
    # 关节约束板与原轨道的相对运动，避免两份原装配接触网格互相挤压；飞机仍与二者碰撞。
    node(node(node(plate_collision, "surface"), "contact"), "collide_bitmask", 2)
    sensor = node(armor, "sensor", name="armor_contact", type="contact")
    node(sensor, "always_on", "true")
    node(sensor, "topic", "/evaluation/enemy_base_moving_armor/contacts")
    node(node(sensor, "contact"), "collision", "armor_plate")
    joint = node(model, "joint", name="armor_slide", type="prismatic")
    node(joint, "parent", "original_rail_anchor")
    node(joint, "child", "armor")
    axis = node(joint, "axis")
    node(axis, "xyz", "0 1 0")
    limit = node(axis, "limit")
    travel = float(config["moving_armor"].get("half_travel_m", 0.28))
    speed = float(config["moving_armor"].get("speed_mps", 0.2))
    if not 0 < travel <= 0.28 or not 0 <= speed <= 1:
        raise ValueError("原滑轨运动参数超出范围")
    for key, value in [
        ("lower", -travel),
        ("upper", travel),
        ("effort", 1000000),
        ("velocity", max(speed, 0.01)),
    ]:
        node(limit, key, value)
    plugin = node(
        model,
        "plugin",
        filename="libmoving_armor_system.so",
        name="drone_sim::MovingArmorSystem",
    )
    node(plugin, "joint_name", "armor_slide")
    node(plugin, "half_travel_m", travel)
    node(plugin, "speed_mps", speed)
    config["terrain_triangles_file"] = str(output / "terrain_fixed.f32")
    config["moving_armor"]["partition"] = report
    return model
