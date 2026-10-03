"""从 Sentry26 原网格分离顶部滑轨中央板件；不新增板或滑轨几何。"""

import hashlib
import json
import struct
from pathlib import Path

import numpy as np


def write_stl(path, triangles):
    normal = np.cross(
        triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    )
    lengths = np.linalg.norm(normal, axis=1)
    normal /= np.maximum(lengths[:, None], 1e-20)
    records = np.zeros(
        len(triangles),
        dtype=np.dtype(
            [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
        ),
    )
    records["normal"] = normal
    records["vertices"] = triangles
    path.write_bytes(
        b"Sentry26 original mesh partition".ljust(80, b" ")
        + struct.pack("<I", len(triangles))
        + records.tobytes()
    )


def partition(source, output):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    triangles = np.fromfile(source, dtype="<f4").reshape(-1, 3, 3)
    low, high = triangles.min(axis=1), triangles.max(axis=1)
    # 已检查的原场地顶部中央滑块包围范围；以完整三角面分组，不裁切三角形。
    selected = (
        (low[:, 0] > 11.45)
        & (high[:, 0] < 11.72)
        & (low[:, 1] > -0.14)
        & (high[:, 1] < 0.145)
        & (low[:, 2] > 1.37)
    )
    moving, fixed = triangles[selected], triangles[~selected]
    if len(moving) != 1062:
        raise ValueError("场地网格版本不匹配，需重新检查原板件分组")
    # 三角面原坐标完整保留，关节零位与静态原模型完全相同。
    write_stl(output / "base_original_carriage.stl", moving)
    write_stl(output / "terrain_fixed.stl", fixed)
    fixed.astype("<f4").tofile(output / "terrain_fixed.f32")
    report = {
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "source_faces": len(triangles),
        "fixed_faces": len(fixed),
        "moving_faces": len(moving),
        "selected_face_indices": np.flatnonzero(selected).tolist(),
        "moving_bounds_m": [
            moving.min(axis=(0, 1)).tolist(),
            moving.max(axis=(0, 1)).tolist(),
        ],
        "zero_pose_geometry_identical": True,
    }
    (output / "partition.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return report
