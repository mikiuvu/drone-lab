# @file render_team_map.py
# @brief 以米制等比例绘制地形、编队中心航路、起点标记和机体尺寸。

"""Render configured team routes, home markers and model dimensions."""

import argparse
import json
from pathlib import Path

import numpy as np
from matplotlib import rcParams
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Rectangle


# 读取地图、尺寸核对数据与高度栅格，在全场和起点局部视图中绘制等比例几何并保存图片。
def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=root / "maps/rmuc_2027.json")
    parser.add_argument("--output", type=Path, default=root / "docs/team-map.png")
    args = parser.parse_args()
    config = json.loads(args.map.read_text(encoding="utf-8"))
    scale = json.loads(
        (root / "docs/scale-verification.json").read_text(encoding="utf-8")
    )
    grid = np.load(args.map.parent / config["terrain_height_grid"], allow_pickle=False)
    rcParams["font.family"] = "Microsoft YaHei"
    rcParams["axes.unicode_minus"] = False
    figure = Figure(figsize=(13, 6), layout="constrained")
    FigureCanvasAgg(figure)
    axes = figure.subplots(1, 2, gridspec_kw={"width_ratios": [2, 1]})
    marker = config["home_marker"]["size_m"]
    sx, sy = scale["vehicle_extent_m"][:2]
    for axis in axes:
        axis.imshow(
            grid["z"],
            origin="lower",
            extent=[grid["x"][0], grid["x"][-1], grid["y"][0], grid["y"][-1]],
            cmap="terrain",
            vmin=config.get("terrain_height_min_m", 0),
            vmax=config.get("terrain_height_max_m", 3.7),
        )
        axis.set_aspect("equal")
        axis.set_xlabel("X / m")
        axis.set_ylabel("Y / m")
        for home in config["spawns"]:
            x, y = home["x"], home["y"]
            axis.add_patch(
                Rectangle(
                    (x - marker / 2, y - marker / 2),
                    marker,
                    marker,
                    fill=False,
                    edgecolor="red",
                    linewidth=1.4,
                )
            )
            axis.add_patch(
                Rectangle((x - sx / 2, y - sy / 2), sx, sy, facecolor="black")
            )
            axis.text(
                x + marker / 2 + 0.03,
                y + 0.12,
                f"H{home['id']}",
                color="red",
                fontsize=10,
            )
        axis.grid(alpha=0.25)
    route = config["team_mission"]["transit_waypoints"]
    axes[0].plot(
        [p["x"] for p in route],
        [p["y"] for p in route],
        color="#f35b0c",
        linewidth=1.8,
        marker=".",
        label="编队中心航路",
    )
    base = config["team_mission"]["enemy_base"]
    bx, by = base["center_x_m"], base["center_y_m"]
    axes[0].add_patch(Circle((bx, by), 0.75, fill=False, edgecolor="blue", linewidth=2))
    axes[0].text(bx - 1.6, by - 1.5, "蓝方基地", color="blue")
    axes[0].set_title(config["label"])
    axes[0].legend(loc="upper left")
    homes = config["spawns"]
    axes[1].set_xlim(min(p["x"] for p in homes) - .6, max(p["x"] for p in homes) + .6)
    axes[1].set_ylim(min(p["y"] for p in homes) - .6, max(p["y"] for p in homes) + .6)
    axes[1].set_title(f"起点：红框 {marker:.2f} m，机体 {sx:.3f} m")
    figure.savefig(args.output, dpi=150)
    grid.close()


if __name__ == "__main__":
    main()
