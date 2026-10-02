"""Render the configured team route and contact handoff geometry."""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle


def main():
    root = Path(__file__).resolve().parent
    path = root / "maps/rmuc_2026_sentry26.json"
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
        axis.pcolormesh(x, y, z, shading="nearest", cmap="terrain", vmin=0, vmax=3.5)
        axis.set_aspect("equal")
        axis.set_xlabel("X / m")
        axis.set_ylabel("Y / m")
    axes[0].set_title("全场规划航路")
    axes[0].set_xlim(-14.6, 14.6)
    axes[0].set_ylim(-8.1, 8.1)
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
            Rectangle(
                (home["x"] - 0.15, home["y"] - 0.15), 0.3, 0.3, fill=False, ec="red"
            )
        )
        axes[0].annotate(
            f"H{home['id']}",
            (home["x"], home["y"]),
            xytext=(3, 7),
            textcoords="offset points",
            fontsize=8,
        )
    axes[1].set_title("基地接触与侧方待命")
    axes[1].set_xlim(9.5, 11.65)
    axes[1].set_ylim(-1.3, 1.3)
    approach, target = mission["approach"], mission["target"]
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
    for side in [1] if mission.get("contact_exit_side") == -1 else [-1, 1]:
        exit_side = mission.get("contact_exit_side") or side
        axes[1].plot(
            approach["x"] - 0.15, approach["y"] + side * 0.25, "D", color="#6747a0"
        )
        axes[1].plot(
            [target["x"], approach["x"] - 0.45, approach["x"] - 0.6],
            [target["y"], approach["y"], approach["y"] + exit_side * 0.4],
            "--",
            color="#c5222c",
            lw=1.5,
        )
    axes[1].legend(loc="lower right")
    figure.tight_layout()
    figure.savefig(root / "docs/team-route.png", dpi=160)
    plt.close(figure)


if __name__ == "__main__":
    main()
