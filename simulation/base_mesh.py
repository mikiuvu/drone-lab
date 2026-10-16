"""分离基地原滑台，并在原表面划分小装甲灯条和绿色飞镖引导灯。"""

import hashlib
import json
import struct
from pathlib import Path

import numpy as np


def split_plane(triangles, axis, offset):
    """用平面裁分三角形；返回正侧、负侧，交点仍位于原三角面内。"""
    positive, negative = [], []
    for triangle in triangles:
        distances = triangle @ axis - offset
        for sign, destination in ((1, positive), (-1, negative)):
            polygon = []
            for index in range(3):
                a, b = triangle[index], triangle[(index + 1) % 3]
                da, db = distances[index] * sign, distances[(index + 1) % 3] * sign
                if da >= -1e-10:
                    polygon.append(a)
                if (da > 1e-10 and db < -1e-10) or (da < -1e-10 and db > 1e-10):
                    polygon.append(a + (b - a) * da / (da - db))
            for index in range(1, len(polygon) - 1):
                face = np.array([polygon[0], polygon[index], polygon[index + 1]])
                if np.linalg.norm(np.cross(face[1] - face[0], face[2] - face[0])) > 1e-12:
                    destination.append(face)
    return (np.asarray(positive).reshape(-1, 3, 3),
            np.asarray(negative).reshape(-1, 3, 3))


def split_region(triangles, planes):
    """取若干半平面的交集，同时收集外部碎片以保留原表面面积。"""
    inside, outside = triangles, []
    for axis, offset in planes:
        inside, rejected = split_plane(inside, axis, offset)
        outside.append(rejected)
    return inside, np.concatenate(outside)


def target_surfaces(triangles, specification):
    """灯条尺寸使用小装甲正面的米制坐标；不把壳体轮廓当作灯条。"""
    u = np.asarray(specification.get('panel_right', [0., 1., 0.]))
    v = np.asarray(specification['panel_up'])
    center = np.asarray(specification['center_m'])
    panel = triangles[specification['panel_surface_faces']].astype(float)
    strips, remaining = [], panel
    half_height = specification['strip_height_m'] / 2
    half_width = specification['strip_width_m'] / 2
    for side in (-1, 1):
        horizontal = center @ u + side * specification['strip_separation_m'] / 2
        vertical = center @ v
        region, remaining = split_region(remaining, [
            (u, horizontal - half_width), (-u, -horizontal - half_width),
            (v, vertical - half_height), (-v, -vertical - half_height),
        ])
        strips.append(region)
    # 55 mm 绿灯用 48 边形材质区域描绘；裁分后仍与原壳体曲面重合。
    guide = triangles[specification['guide_surface_faces']].astype(float)
    guide_center = np.asarray(specification['guide_center_m'])
    guide_up = np.asarray(specification['guide_up'])
    radius = specification['guide_diameter_m'] / 2
    planes = []
    for angle in np.linspace(0, 2 * np.pi, 48, endpoint=False):
        axis = np.cos(angle) * u + np.sin(angle) * guide_up
        planes.append((-axis, -(guide_center @ axis + radius)))
    green, guide_body = split_region(guide, planes)
    return np.concatenate(strips), remaining, green, guide_body


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
    panel_indices = surfaces['panel_surface_faces']
    guide_indices = surfaces['guide_surface_faces']
    if not np.all(selected[panel_indices + guide_indices]):
        raise ValueError("小装甲板或引导灯不属于原滑台")
    blue, panel_body, green, guide_body = target_surfaces(triangles, surfaces)
    remapped = np.zeros(len(triangles), dtype=bool)
    remapped[panel_indices + guide_indices] = True
    body = np.concatenate([triangles[selected & ~remapped], panel_body, guide_body])
    write_stl(output / 'carriage_blue_surfaces.stl', blue)
    write_stl(output / 'carriage_green_guide.stl', green)
    write_stl(output / 'carriage_body_surfaces.stl', body)
    # 触压只覆盖原小装甲板正面；滑台壳体、背面和绿灯接触单独参与碰撞。
    write_stl(output / 'armor_hit_surface.stl', triangles[panel_indices])
    hit_mask = np.zeros(len(triangles), dtype=bool)
    hit_mask[panel_indices] = True
    write_stl(output / 'carriage_housing.stl', triangles[selected & ~hit_mask])
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
        "target_type": "base_dart_detection_small_armor",
        "material_faces": {"body": len(body), "blue": len(blue), "green": len(green)},
        "target_center_m": surfaces['center_m'],
    }
    (output / "partition.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return report
