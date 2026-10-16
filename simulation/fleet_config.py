# @file fleet_config.py
# @brief 依据机队数量生成连续编号、出生点和编队位置，配置固定 UWB 锚点。

"""Resolve fleet cardinality and optional fixed UWB geometry."""

import copy
import math


# @brief 深拷贝地图并调整机队配置，原始 config 不被修改。
# @param config 含 spawns 的地图；改变数量时，已有任务编队按其中心重新布置。
# @param vehicle_count 正整数；None 保留现有数量，候选出生点不足或编号不连续时抛出 ValueError。
# @param enable_anchors False 清空锚点，True 在缺少锚点时添加默认锚点，None 保留配置。
# @return 配置副本，包含数量、每机航点以及缺省光流参数。
def configure_fleet(config, vehicle_count=None, enable_anchors=None):
    result = copy.deepcopy(config)
    spawns = result["spawns"]
    count = len(spawns) if vehicle_count is None else vehicle_count
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError("vehicle_count must be a positive integer")
    capacity = result.get("max_vehicle_count")
    if capacity is not None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise ValueError("max_vehicle_count must be a positive integer")
        if count > capacity:
            raise ValueError(f"Map supports at most {capacity} vehicles")
    if count != len(spawns):
        original = list(spawns)
        if count <= len(spawns):
            spawns = spawns[:count]
        else:
            # Add rows within the central platform, maintaining 0.4 m spacing.
            candidates = [(0, -0.4), (0, 0.4)] + [
                (x * 0.4, y * 0.4) for y in range(-2, 3) for x in range(-2, 3)
            ]
            for x, y in candidates:
                if all(math.hypot(x - s["x"], y - s["y"]) >= 0.39 for s in spawns):
                    spawns.append(dict(original[0], id=len(spawns), x=x, y=y, yaw=0))
                    if len(spawns) == count:
                        break
            if len(spawns) != count:
                raise ValueError(
                    "Requested fleet needs an explicit map with enough separated spawn positions"
                )
        result["spawns"] = spawns
        if "team_mission" in result:
            mission = result["team_mission"]
            center_x = sum(p["x"] for p in mission["assembly"]) / len(
                mission["assembly"]
            )
            center_y = sum(p["y"] for p in mission["assembly"]) / len(
                mission["assembly"]
            )
            columns = math.ceil(math.sqrt(count))
            rows = math.ceil(count / columns)
            offsets = [
                (i % columns - (columns - 1) / 2, i // columns - (rows - 1) / 2)
                for i in range(count)
            ]
            mission["assembly"] = [
                {"id": i, "x": center_x + x * 0.65, "y": center_y + y * 0.65}
                for i, (x, y) in enumerate(offsets)
            ]
            mission["formation_offsets"] = [
                {"id": i, "x": x * 0.65, "y": y * 0.65}
                for i, (x, y) in enumerate(offsets)
            ]
            mission["order"] = list(range(count))
            if "completed_holding" in mission:
                holding_y = sum(p["y"] for p in mission["completed_holding"]) / len(
                    mission["completed_holding"]
                )
                mission["completed_holding"] = [
                    {"id": i, "x": center_x + x * 0.65, "y": holding_y + y * 0.65}
                    for i, (x, y) in enumerate(offsets)
                ]
    if sorted(s["id"] for s in spawns) != list(range(count)):
        raise ValueError("SITL vehicle IDs must be contiguous from zero")
    result["vehicle_count"] = count
    result["vehicle_waypoints"] = {
        str(i): result.get("vehicle_waypoints", {}).get(str(i), [])
        for i in range(count)
    }
    if enable_anchors is False:
        result["uwb_anchors"] = []
    elif enable_anchors and not result.get("uwb_anchors"):
        result["uwb_anchors"] = [
            {"id": i, "x": x, "y": y, "z": 2.4}
            for i, (x, y) in enumerate([(-13, -7), (-13, 7), (13, -7), (13, 7)])
        ]
    else:
        result.setdefault("uwb_anchors", [])
    result.setdefault(
        "optical_flow",
        {
            "enabled": True,
            "noise_std_mps": 0.015,
            "scale_error": 0.005,
            "drop_probability": 0.01,
        },
    )
    return result
