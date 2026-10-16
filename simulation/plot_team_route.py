# @file plot_team_route.py
# @brief 根据地图配置绘制全场航路及基地接触、侧方待命位置。

"""Render the configured team route and contact handoff geometry."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle


# 从地图和高度栅格绘制规划几何，保存 team-route.png；图中位置使用世界 XY 米制坐标。
def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=root / "maps/rmuc_2027.json")
    parser.add_argument("--output", type=Path, default=root / "docs/team-route.png")
    args = parser.parse_args()
    path = args.map
    config = json.loads(path.read_text(encoding="utf-8"))
    mission = config["team_mission"]
    with np.load(path.parent / config["terrain_height_grid"]) as grid:
        x, y, z = grid["x"], grid["y"], grid["z"]
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    figure, axes = plt.subplots(
        1, 2, figsize=(13, 5), gridspec_kw={"width_ratios": [2, 1]}
    )
    for axis in axes:
        axis.pcolormesh(x, y, z, shading="nearest", cmap="terrain",
                        vmin=config.get("terrain_height_min_m", 0),
                        vmax=config.get("terrain_height_max_m", 3.7))
        axis.set_aspect("equal")
        axis.set_xlabel("X / m")
        axis.set_ylabel("Y / m")
    axes[0].set_title("全场规划航路")
    bounds = config["bounds"]
    axes[0].set_xlim(bounds["x_min"], bounds["x_max"])
    axes[0].set_ylim(bounds["y_min"], bounds["y_max"])
    route = mission["transit_waypoints"] + [
        {
            "x": sum(p["x"] for p in mission["assembly"]) / len(mission["assembly"]),
            "y": sum(p["y"] for p in mission["assembly"]) / len(mission["assembly"]),
        }
    ]
    axes[0].plot(
        [p["x"] for p in route], [p["y"] for p in route], "o-", color="#ed6800", lw=2
    )
    for home in config["spawns"]:
        axes[0].add_patch(
            Circle((home["x"], home["y"]), mission["contact_radius_m"], fill=False, ec="red")
        )
    axes[0].annotate("四机机库",
        np.mean([[point["x"], point["y"]] for point in config["spawns"]], axis=0),
        xytext=(-42, -24), textcoords="offset points", fontsize=8,
        arrowprops={"arrowstyle": "->", "lw": .8})
    axes[1].set_title("基地接触与侧方待命")
    approach, target = mission["approach"], mission["target"]
    context = [approach, target] + mission["assembly"] + mission.get("completed_holding", [])
    if "retreat_exit" in mission:
        context.append(mission["retreat_exit"])
    axes[1].set_xlim(min(point["x"] for point in context) - .3,
                     max(point["x"] for point in context) + .3)
    axes[1].set_ylim(min(point["y"] for point in context) - .3,
                     max(point["y"] for point in context) + .3)
    axes[1].plot(
        [approach["x"], target["x"]],
        [approach["y"], target["y"]],
        "o-",
        color="#ed6800",
        lw=2,
        label="接触",
    )
    for point in mission["assembly"]:
        axes[1].plot(point["x"], point["y"], "s", color="#005ca9")
        axes[1].annotate(
            str(point["id"]),
            (point["x"], point["y"]),
            xytext=(5, 5),
            textcoords="offset points",
        )
    for point in mission.get("completed_holding", []):
        axes[1].plot(point["x"], point["y"], "^", color="#00853d")
    if "retreat_exit" in mission:
        exit_point = mission["retreat_exit"]
        axes[1].plot([target["x"], approach["x"], exit_point["x"]],
                     [target["y"], approach["y"], exit_point["y"]],
                     "D--", color="#c5222c", lw=1.5, label="退出航点")
        for point in mission.get("completed_holding", []):
            axes[1].plot([exit_point["x"], point["x"]], [exit_point["y"], point["y"]],
                         "--", color="#00853d", lw=1)
    else:
        for side in [1] if mission.get("contact_exit_side") == -1 else [-1, 1]:
            exit_side = mission.get("contact_exit_side") or side
            axes[1].plot(
                approach["x"] - 0.15, approach["y"] + side * 0.25, "D", color="#6747a0"
            )
            axes[1].plot(
                [target["x"], approach["x"] - 0.45, approach["x"] - 0.6],
                [target["y"], approach["y"], approach["y"] + exit_side * 0.4],
                "--", color="#c5222c", lw=1.5,
            )
    axes[1].legend(loc="lower right")
    figure.tight_layout()
    figure.savefig(args.output, dpi=160)
    plt.close(figure)


if __name__ == "__main__":
    main()
