"""Render configured team routes, home markers and model dimensions."""

import json
from pathlib import Path

import numpy as np
from matplotlib import rcParams
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Rectangle


def main():
    root = Path(__file__).resolve().parent
    config = json.loads(
        (root / "maps/rmuc_2026_sentry26.json").read_text(encoding="utf-8")
    )
    scale = json.loads(
        (root / "docs/scale-verification.json").read_text(encoding="utf-8")
    )
    grid = np.load(root / "models/terrain/rmuc_2026_sentry26/terrain_height_grid.npz")
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
            vmin=0,
            vmax=3,
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
    axes[0].add_patch(
        Circle((11.6, 0), 0.75, fill=False, edgecolor="blue", linewidth=2)
    )
    axes[0].text(10, -1.5, "蓝方基地", color="blue")
    axes[0].set_title("比赛区 28 × 15 m · 红方进攻蓝方基地")
    axes[0].legend(loc="upper left")
    axes[1].set_xlim(-1.4, 1.4)
    axes[1].set_ylim(-1.2, 1.4)
    axes[1].set_title(f"中央起点：红框 {marker:.2f} m，机体 {sx:.3f} m")
    figure.savefig(root / "docs/team-route.png", dpi=150)


if __name__ == "__main__":
    main()
