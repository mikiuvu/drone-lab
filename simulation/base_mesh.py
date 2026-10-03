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
    surfaces = json.loads(
        (Path(__file__).with_name("original_armor_faces.json")).read_text(
            encoding="utf-8"
        )
    )
    if hashlib.sha256(source.read_bytes()).hexdigest() != surfaces["source_sha256"]:
        raise ValueError("原装甲面材质索引与场地版本不一致")
    blue = np.zeros(len(triangles), dtype=bool)
    blue[surfaces["blue_surface_faces"]] = True
    if not np.all(selected[blue]):
        raise ValueError("灯条材质面不属于原滑块")
    front = triangles[blue]
    write_stl(
        output / "carriage_blue_surfaces.stl",
        np.concatenate([front, front[:, [0, 2, 1]]]),
    )
    logo = np.zeros(len(triangles), dtype=bool)
    logo[surfaces["logo_surface_faces"]] = True
    colored = triangles[logo].copy()
    # 在原三角面内细分材质采样，几何曲面和碰撞面保持不变。
    for _ in range(6):
        a, b, c = colored[:, 0], colored[:, 1], colored[:, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        colored = np.concatenate(
            [
                np.stack(v, axis=1)
                for v in [(a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)]
            ]
        )
    mask = np.asarray(
        json.loads(
            (Path(__file__).parent / "assets/base-sticker-mask.json").read_text(
                encoding="utf-8"
            )
        )
    )
    centers = colored.mean(axis=1)
    low, high = triangles[logo].min(axis=(0, 1)), triangles[logo].max(axis=(0, 1))
    u = np.clip(
        ((high[1] - centers[:, 1]) / (high[1] - low[1]) * 127).astype(int), 0, 127
    )
    v = np.clip(
        ((high[2] - centers[:, 2]) / (high[2] - low[2]) * 127).astype(int), 0, 127
    )
    white = mask[v, u] > 0
    write_stl(output / "logo_white.stl", colored[white])
    write_stl(output / "logo_black.stl", colored[~white])
    write_stl(
        output / "carriage_body_surfaces.stl", triangles[selected & ~blue & ~logo]
    )
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
