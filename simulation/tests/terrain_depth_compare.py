"""Evaluation-only comparison: engine depth against static triangle prior."""

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
data = json.loads((ROOT / "docs/terrain-depth-static.json").read_text(encoding="utf-8"))
tri = (
    np.fromfile(
        ROOT / "models/terrain/rmuc_2026_sentry26/terrain_triangles.f32", dtype="<f4"
    )
    .reshape(-1, 3, 3)
    .astype(float)
)
a = tri[:, 0]
e1 = tri[:, 1] - a
e2 = tri[:, 2] - a


def cast(origin, ray):
    p = np.cross(ray, e2)
    det = np.einsum("ij,ij->i", e1, p)
    s = origin - a
    q = np.cross(s, e1)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.einsum("ij,ij->i", s, p) / det
        v = (q @ ray) / det
        d = np.einsum("ij,ij->i", e2, q) / det
    valid = (
        (abs(det) > 1e-10)
        & (u >= -1e-7)
        & (v >= -1e-7)
        & (u + v <= 1.0000001)
        & (d > 0.0001)
        & (d < 4)
    )
    return float(np.min(d[valid])) if valid.any() else float("inf")


reports = []
for key, depths in data["depth"].items():
    vehicle, sensor = map(int, key.split("_"))
    pose = data["poses"][str(vehicle)]
    w, x, y, z = pose["q_wxyz"]
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    angle = yaw + [0, 2 * np.pi / 3, -2 * np.pi / 3][sensor]
    rot = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1],
        ]
    )
    origin = np.array(pose["xyz"]) + rot @ np.array([0.055, 0, 0])
    expected = []
    for cell in range(64):
        row, col = divmod(cell, 8)
        ray = rot @ np.array(
            [
                1,
                -(col - 3.5) / 4 * np.tan(np.pi / 8),
                -(row - 3.5) / 4 * np.tan(np.pi / 8),
            ]
        )
        expected.append(cast(origin, ray))
    observed = np.array(depths)
    expected = np.array(expected)
    both = np.isfinite(observed) & np.isfinite(expected)
    errors = observed[both] - expected[both]
    reports.append(
        {
            "sensor": key,
            "common_returns": int(both.sum()),
            "max_error_m": float(np.abs(errors).max()) if both.any() else None,
            "rmse_m": float(np.sqrt(np.mean(errors**2))) if both.any() else None,
            "mismatched_hits": int(
                np.count_nonzero(np.isfinite(observed) != np.isfinite(expected))
            ),
        }
    )
(ROOT / "docs/terrain-depth-comparison.json").write_text(
    json.dumps(reports, indent=2), encoding="utf-8"
)
print(json.dumps(reports, indent=2))
