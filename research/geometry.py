"""@file geometry.py
@brief 双 8×8 ToF 中心射线与有限墙面的合成观测、局部可观测性和噪声拟合。
@note x 前、y 左、z 上，正俯仰朝上；高度和机体 roll/pitch 已知，只估计 x、y、yaw。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


DEFAULT_CONFIG = {
    "height_m": 2.5,
    "max_range_m": 3.0,
    "mount_pitch_deg": 0.0,
    "yaw_separation_deg": 90.0,
    "noise_mm": 30.0,
}

_OPTIONS = {
    "diagonal_fov_deg": 65.0,
    "fov_model": "diagonal_square",
    "distance_mode": "axial",
    "mount_roll_deg": 0.0,
    "body_roll_deg": 0.0,
    "body_pitch_deg": 0.0,
    "sensor_baseline_m": 0.08,
    "yaw_length_scale_m": 1.0,
    "noise_correlation": 0.6,
    "bias_mm": 0.0,
    "latency_ms": 0.0,
    "sensor_skew_ms": 0.0,
    "velocity_x_m_s": 0.0,
    "velocity_y_m_s": 0.0,
    "yaw_rate_deg_s": 0.0,
    "monte_carlo_trials": 24,
    "seed": 2027,
}


@dataclass(frozen=True)
class Wall:
    """@brief 由平面起止点和高度上下界定义的有限竖直矩形，坐标单位 m。
    """

    name: str
    start: tuple[float, float]  # 水平起点 (x,y)，m。
    end: tuple[float, float]  # 水平终点 (x,y)，m。
    z_max: float  # 上沿高度，m。
    z_min: float = 0.0  # 下沿高度，m。

    def __post_init__(self):
        """@brief 拒绝非有限坐标、零长度墙段及非正高度。
        """
        values = (*self.start, *self.end, self.z_min, self.z_max)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("墙面坐标必须有限")
        if self.start == self.end or self.z_max <= self.z_min:
            raise ValueError("墙段长度和高度必须为正")

    @property
    def normal(self) -> np.ndarray:
        """@brief 返回墙段切线在水平面内旋转 90° 后的单位法向量。
        """
        tangent = np.asarray(self.end) - self.start
        return np.array([-tangent[1], tangent[0], 0.0]) / np.linalg.norm(tangent)


def rotation(yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """@brief 构造 Rz(yaw)·Ry(-pitch)·Rx(roll) 旋转矩阵。
    @param yaw 偏航，rad。
    @param pitch 俯仰，rad，正值将光轴转向上方。
    @param roll 横滚，rad。
    @return 3×3 旋转矩阵；行向量应用时乘矩阵转置。
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def axis_fov_from_diagonal(diagonal_deg: float) -> float:
    """@brief 由理想方形透视视场的对角角度换算轴向角度。
    @param diagonal_deg 对角视场角，度，范围 (0,179)。
    @return 水平与垂直轴向视场角，度；使用 tan(对角半角)/√2 得到轴向半角正切。
    """
    if not math.isfinite(diagonal_deg) or not 0 < diagonal_deg < 179:
        raise ValueError("对角视场必须在 0 到 179 度之间")
    return math.degrees(2 * math.atan(math.tan(math.radians(diagonal_deg / 2)) / math.sqrt(2)))


def zone_rays(diagonal_deg: float = 65.0, *, axis_fov_deg: float | None = None) -> np.ndarray:
    """@brief 在理想投影平面上取 8×8 分区中心并归一化为方向。
    @param diagonal_deg 对角视场角，度。
    @param axis_fov_deg 显式轴向视场角，度；非 None 时优先使用。
    @return 64×3 单位方向，首行在上、首列在左。
    @note 真实分区编号、镜像及光轴需标定，区域脚印并未建模为独立激光。
    """
    axis = axis_fov_from_diagonal(diagonal_deg) if axis_fov_deg is None else axis_fov_deg
    if not math.isfinite(axis) or not 0 < axis < 179:
        raise ValueError("水平/垂直视场必须在 0 到 179 度之间")
    centres = (1 - (np.arange(8) + 0.5) / 4) * math.tan(math.radians(axis / 2))
    horizontal, vertical = np.meshgrid(centres, centres)
    rays = np.column_stack((np.ones(64), horizontal.ravel(), vertical.ravel()))
    return rays / np.linalg.norm(rays, axis=1)[:, None]


def intersect_wall(origin, direction, wall: Wall, max_range_m: float = 3.0) -> float | None:
    """@brief 求单位化射线与有限墙面的有效交点距离。
    @param origin 三维射线起点，m。
    @param direction 非零三维方向，会在内部归一化。
    @param wall 有限竖直墙面。
    @param max_range_m 最大沿射线距离，m。
    @return 沿射线距离，m；平行、背向、超量程或越出墙面时返回 None。
    @note 先求平面交点，再检查线段投影和高度；端点含 1e-10 数值容差。
    """
    origin = np.asarray(origin, dtype=float)
    direction = np.asarray(direction, dtype=float)
    if origin.shape != (3,) or direction.shape != (3,):
        raise ValueError("射线需要三个坐标")
    if not np.isfinite(origin).all() or not np.isfinite(direction).all():
        raise ValueError("射线坐标必须有限")
    norm = float(np.linalg.norm(direction))
    if norm == 0 or not math.isfinite(max_range_m) or max_range_m <= 0:
        raise ValueError("射线方向和最大距离必须有效")
    direction = direction / norm
    normal = wall.normal
    denominator = float(normal @ direction)
    if abs(denominator) < 1e-12:
        return None
    anchor = np.array([*wall.start, wall.z_min])
    distance = float(normal @ (anchor - origin) / denominator)
    if distance <= 1e-10 or distance > max_range_m + 1e-10:
        return None
    point = origin + distance * direction
    segment = np.asarray(wall.end) - wall.start
    fraction = float((point[:2] - wall.start) @ segment / (segment @ segment))
    if -1e-10 <= fraction <= 1 + 1e-10 and wall.z_min - 1e-10 <= point[2] <= wall.z_max + 1e-10:
        return distance
    return None


def _config(config: dict | None) -> dict:
    """@brief 合并公开参数与可选控制，校验角度、距离、噪声、试验数与模型选项。
    @param config 覆盖字典或 None；单位由 _m、_mm、_ms、_deg、_m_s 等后缀标明。
    @return 完整配置，试验次数和随机种子归一化为整数。
    """
    if config is not None and not isinstance(config, dict):
        raise ValueError("config 必须是字典或 None")
    merged = DEFAULT_CONFIG | _OPTIONS
    for key, value in (config or {}).items():
        if key not in merged:
            raise ValueError(f"未知几何参数：{key}")
        merged[key] = value
    for key, value in merged.items():
        if key in ("distance_mode", "fov_model"):
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{key} 必须是有限数值")
    for key in ("height_m", "max_range_m", "yaw_length_scale_m"):
        if not 0 < merged[key] <= 100:
            raise ValueError(f"{key} 必须大于 0 且不超过 100")
    for key in ("noise_mm", "latency_ms", "sensor_skew_ms", "sensor_baseline_m"):
        if not 0 <= merged[key] <= 10000:
            raise ValueError(f"{key} 必须在 0 到 10000 之间")
    if merged["yaw_length_scale_m"] < 0.01:
        raise ValueError("yaw_length_scale_m 至少为 0.01，保持局部差分尺度有效")
    limits = {"mount_pitch_deg": 180, "mount_roll_deg": 180,
              "body_roll_deg": 180, "body_pitch_deg": 180,
              "yaw_separation_deg": 360, "bias_mm": 10000,
              "velocity_x_m_s": 100, "velocity_y_m_s": 100,
              "yaw_rate_deg_s": 3600}
    for key, upper in limits.items():
        if abs(merged[key]) > upper:
            raise ValueError(f"{key} 的绝对值不得超过 {upper}")
    if not 0 <= merged["noise_correlation"] <= 1:
        raise ValueError("noise_correlation 必须在 0 到 1 之间")
    for key, upper in (("monte_carlo_trials", 128), ("seed", 2**32 - 1)):
        if int(merged[key]) != merged[key] or not 0 <= merged[key] <= upper:
            raise ValueError(f"{key} 必须是 0 到 {upper} 的整数")
        merged[key] = int(merged[key])
    if merged["distance_mode"] not in ("axial", "radial"):
        raise ValueError("distance_mode 必须是 axial 或 radial")
    if merged["fov_model"] not in ("diagonal_square", "datasheet_axes"):
        raise ValueError("fov_model 必须是 diagonal_square 或 datasheet_axes")
    axis_fov_from_diagonal(merged["diagonal_fov_deg"])
    return merged


def _sensor_geometry(config: dict):
    """@brief 生成两颗传感器的 128 条机体方向、安装偏移和距离投影比例。
    @param config 完整配置；安装先横滚再俯仰再偏航，随后应用已知机体姿态。
    @return directions、offsets、scale、轴向视场角；偏移 m，视场角度。
    @note axial 使用局部光轴分量缩放沿射线距离，radial 的比例为 1。
    """
    axis = 45.0 if config["fov_model"] == "datasheet_axes" else axis_fov_from_diagonal(config["diagonal_fov_deg"])
    rays = zone_rays(axis_fov_deg=axis)
    pitch, roll = np.radians([config["mount_pitch_deg"], config["mount_roll_deg"]])
    body = rotation(pitch=math.radians(config["body_pitch_deg"]), roll=math.radians(config["body_roll_deg"]))
    sensor_rotations = [rotation(math.radians(yaw), pitch, roll) for yaw in (0, config["yaw_separation_deg"])]
    directions = np.concatenate([rays @ mount.T @ body.T for mount in sensor_rotations])
    baseline = config["sensor_baseline_m"]
    offsets = np.repeat(np.array([[baseline / 2, 0, 0], [-baseline / 2, 0, 0]]) @ body.T, 64, axis=0)
    scale = np.tile(rays[:, 0], 2) if config["distance_mode"] == "axial" else np.ones(128)
    return directions, offsets, scale, axis


def _world_geometry(pose, directions, offsets, height):
    """@brief 将机体系安装偏移与射线按当前偏航转换到世界系。
    @param pose (x,y,yaw)，位置 m、偏航 rad。
    @param directions 机体方向数组。
    @param offsets 安装偏移数组，m。
    @param height 已知高度，m。
    @return 世界系起点和方向数组。
    """
    rz = rotation(yaw=float(pose[2]))
    origins = offsets @ rz.T + np.array([pose[0], pose[1], height])
    return origins, directions @ rz.T


def _observe(walls, config, geometry):
    """@brief 用匀速模型回推两颗的采样位姿，各射线选择量程内最近墙面交点。
    @param walls 合成有限墙面列表。
    @param config 含 latency_ms、sensor_skew_ms 和速度的配置。
    @param geometry 机体系几何。
    @return 命中列表及两颗采样位姿；第二颗年龄比第一颗多 sensor_skew_ms。
    """
    directions, offsets, scale, _ = geometry
    pose_now = np.zeros(3)
    velocity = np.array([config["velocity_x_m_s"], config["velocity_y_m_s"], math.radians(config["yaw_rate_deg_s"])])
    hits, capture_poses = [], []
    for sensor in range(2):
        age = (config["latency_ms"] + sensor * config["sensor_skew_ms"]) / 1000
        pose = pose_now - age * velocity
        capture_poses.append(pose.tolist())
        origins, world_directions = _world_geometry(pose, directions, offsets, config["height_m"])
        for idx in range(sensor * 64, (sensor + 1) * 64):
            candidates = [(distance, wi) for wi, wall in enumerate(walls)
                          if (distance := intersect_wall(origins[idx], world_directions[idx], wall, config["max_range_m"])) is not None]
            if candidates:
                distance, wi = min(candidates)
                hits.append({"ray": idx, "wall": wi, "range_m": distance * scale[idx],
                             "radial_range_m": distance,
                             "point_m": (origins[idx] + distance * world_directions[idx]).tolist()})
    return hits, capture_poses


def _measurement_model(walls, hits, config, geometry):
    """@brief 固定已命中射线的墙面关联，提取平面法向与锚点供连续局部拟合。
    @return 预测函数所需几何元组；后续扰动不重新判断有限端点或切换墙面。
    """
    indices = np.array([hit["ray"] for hit in hits], dtype=int)
    directions, offsets, scale, _ = geometry
    normals = np.array([walls[hit["wall"]].normal for hit in hits]).reshape(-1, 3)
    anchors = np.array([[*walls[hit["wall"]].start, 0] for hit in hits]).reshape(-1, 3)
    return directions[indices], offsets[indices], scale[indices], normals, anchors, config["height_m"]


def _predict(pose, model):
    """@brief 在固定平面关联下预测距离，按 scale 转为 axial 或 radial 输出。
    @param pose (x,y,yaw)，m、m、rad。
    @param model 固定关联的几何元组。
    @return 各命中射线的预测距离，m；近似平行时抛 ValueError。
    """
    directions, offsets, scale, normals, anchors, height = model
    origins, world_directions = _world_geometry(pose, directions, offsets, height)
    denominator = np.einsum("ij,ij->i", normals, world_directions)
    if (np.abs(denominator) < 1e-9).any():
        raise ValueError("固定关联模型发生平行射线")
    return np.einsum("ij,ij->i", normals, anchors - origins) / denominator * scale


def _jacobian(pose, model, length_scale):
    """@brief 对 q=(x,y,L·yaw) 作中心差分，统一三个方向的长度尺度。
    @param pose 位姿，m、m、rad。
    @param model 固定墙面关联。
    @param length_scale L，m；偏航扰动为 1e-5/L rad。
    @return 距离对 q 的雅可比矩阵；差分分母为 2e-5。
    """
    columns = []
    for dim in range(3):
        delta = np.zeros(3)
        delta[dim] = 1e-5 / (length_scale if dim == 2 else 1)
        columns.append((_predict(pose + delta, model) - _predict(pose - delta, model)) / 2e-5)
    return np.column_stack(columns)


def _observability(model, config, hit_count):
    """@brief 用 J/√命中数的奇异值分解评估局部秩和弱约束方向。
    @param model 固定关联模型。
    @param config 提供偏航长度尺度。
    @param hit_count 命中数。
    @return 秩、奇异值、条件数和 q 零空间；秩不足 3 时条件数为 None。
    @note 归一化降低命中数量对幅值的影响；局部满秩不等于全局唯一匹配。
    """
    if not hit_count:
        return {"rank": 0, "singular_values": [0.0, 0.0, 0.0], "smallest_singular_value": 0.0,
                "condition_number": None, "nullspace_q": np.eye(3).tolist(),
                "normalization": "J / sqrt(number_of_hits), q=(x,y,L*yaw)"}
    jacobian = _jacobian(np.zeros(3), model, config["yaw_length_scale_m"])
    _, singular, vt = np.linalg.svd(jacobian / math.sqrt(hit_count), full_matrices=False)
    # All observed walls supply at least three zones in the supplied layouts.
    # Padding also defines the result for externally constructed sparse models.
    if len(singular) < 3:
        _, singular, vt = np.linalg.svd(jacobian / math.sqrt(hit_count), full_matrices=True)
    singular = np.pad(singular, (0, 3 - len(singular)))
    tolerance = max(1e-7, float(singular[0]) * 1e-6)
    rank = int((singular > tolerance).sum())
    return {"rank": rank, "singular_values": singular.tolist(),
            "smallest_singular_value": float(singular[-1]),
            "condition_number": float(singular[0] / singular[-1]) if rank == 3 else None,
            "nullspace_q": vt[rank:].tolist(), "rank_tolerance": tolerance,
            "normalization": "J / sqrt(number_of_hits), q=(x,y,L*yaw)"}


def _fit_ranges(measured, model, config):
    """@brief 从近真值初值进行最多 12 次局部最小二乘迭代。
    @param measured 距离观测，m。
    @param model 固定关联模型。
    @param config 提供偏航长度尺度。
    @return 位姿 (m,m,rad) 和收敛标志。
    @note 先求 q 增量、限制其范数至 0.4，再将偏航增量除以 L；未实现全局搜索。
    """
    pose = np.array([0.08, -0.06, math.radians(3.0)])
    length = config["yaw_length_scale_m"]
    for _ in range(12):
        residual = measured - _predict(pose, model)
        jq = _jacobian(pose, model, length)
        # 在统一长度尺度 q 中求增量，限幅后将偏航分量除以 L 还原为 rad。
        update, _, _, _ = np.linalg.lstsq(jq, residual, rcond=1e-6)
        magnitude = float(np.linalg.norm(update))
        if magnitude > 0.4:
            update *= 0.4 / magnitude
        pose += update / np.array([1, 1, length])
        if magnitude < 1e-8:
            return pose, True
    return pose, False


def _monte_carlo(hits, model, observability, config, seed):
    """@brief 在满秩场景重复噪声生成和局部拟合，统计成功样本误差。
    @param hits 合成命中距离。
    @param model 固定关联模型。
    @param observability 局部秩结果。
    @param config 噪声 mm、共享偏差 mm、同传感器相关系数及试验次数。
    @param seed 当前场景随机种子，使用局部 RNG。
    @return 状态、成功/失败数、平移 RMSE m 和偏航 RMSE 度。
    @note 噪声由分区独立项与每传感器共享项按相关系数组合；真值为当前零位姿，旧帧按当前帧拟合以暴露时延偏差。
    """
    status = "disabled" if not config["monte_carlo_trials"] else "unobservable"
    result = {"status": status, "requested_trials": config["monte_carlo_trials"],
              "successful_trials": 0, "failed_trials": 0,
              "translation_rmse_m": None, "yaw_rmse_deg": None,
              "mean_error_xy_yaw_deg": None,
              "interpretation": "已知地图和墙面关联、理想中心射线下的乐观合成误差；不可观测场景不报告完整位姿精度。"}
    if observability["rank"] < 3 or not config["monte_carlo_trials"]:
        return result
    rng = np.random.default_rng(seed)
    base = np.array([hit["range_m"] for hit in hits])
    sensors = np.array([hit["ray"] // 64 for hit in hits], dtype=int)
    correlation = config["noise_correlation"]
    errors = []
    for _ in range(config["monte_carlo_trials"]):
        # 独立项权重 √(1-ρ)、每颗共享项权重 √ρ，保持每区边际方差不变。
        noise = config["noise_mm"] / 1000 * (math.sqrt(1 - correlation) * rng.normal(size=len(hits))
                + math.sqrt(correlation) * rng.normal(size=2)[sensors])
        measured = base + config["bias_mm"] / 1000 + noise
        if (measured <= 0).any():
            result["failed_trials"] += 1
            continue
        try:
            pose, converged = _fit_ranges(measured, model, config)
        except (ValueError, np.linalg.LinAlgError):
            converged = False
        if converged and np.isfinite(pose).all():
            pose[2] = math.atan2(math.sin(pose[2]), math.cos(pose[2]))
            errors.append(pose)
        else:
            result["failed_trials"] += 1
    result["status"] = "ok" if errors else "estimation_failed"
    result["successful_trials"] = len(errors)
    if errors:
        errors = np.array(errors)
        result["translation_rmse_m"] = float(np.sqrt(np.mean(np.sum(errors[:, :2] ** 2, axis=1))))
        result["yaw_rmse_deg"] = float(math.degrees(np.sqrt(np.mean(errors[:, 2] ** 2))))
        mean_error = np.mean(errors, axis=0)
        mean_error[2] = math.degrees(mean_error[2])
        result["mean_error_xy_yaw_deg"] = mean_error.tolist()
    return result


def _scenarios():
    """@brief 返回单墙、直角、走廊、开阔区与低结构等合成场景及解释。
    """
    def corner(height):
        return [Wall("x_wall", (2.0, -8.0), (2.0, 2.0), height),
                Wall("y_wall", (-8.0, 2.0), (2.0, 2.0), height)]
    return [
        ("single_wall", "单面高墙", [Wall("wall", (2.0, -8.0), (2.0, 8.0), 4.0)],
         "沿墙平移没有距离约束；不把有限端点可见性当连续距离测量。"),
        ("perpendicular_corner", "直角高墙", corner(4.0),
         "覆盖两种墙法向时可局部约束 x、y、yaw，仍依赖已知地图与关联。"),
        ("parallel_corridor", "平行高墙走廊",
         [Wall("left", (-8.0, 1.0), (8.0, 1.0), 4.0),
          Wall("right", (-8.0, -1.0), (8.0, -1.0), 4.0)],
         "理想平行墙中纵向完全不可观测；现实中的端点、纹理或非平行面才可能补充约束。"),
        ("open_area", "开阔区，无范围内墙面", [], "无返回，不能从本帧恢复平面位姿。"),
        ("low_structure_0_3m", "0.3 m 低结构", corner(0.3), "合成低结构，用于检查高度和俯仰覆盖。"),
        ("low_structure_1_1m", "1.1 m 低结构", corner(1.1), "合成低结构，用于检查高度和俯仰覆盖。"),
    ]


def run_geometry(config: dict | None = None) -> dict:
    """@brief 逐场景生成观测、评估局部秩并运行蒙特卡洛拟合。
    @param config 参数覆盖字典或 None，默认公开参数为高度、量程、安装俯仰、两颗夹角及噪声。
    @return 可供 JSON/界面消费的 config、metrics、scenarios、assumptions 与来源。
    @note max_range_m 限制物理沿射线距离，即使选择 axial 输出也如此；高度和姿态误差、反射率、多目标与遮挡未建模。
    """
    cfg = _config(config)
    geometry = _sensor_geometry(cfg)
    results = []
    for index, (key, label, walls, interpretation) in enumerate(_scenarios()):
        hits, capture_poses = _observe(walls, cfg, geometry)
        model = _measurement_model(walls, hits, cfg, geometry)
        obs = _observability(model, cfg, len(hits))
        results.append({"id": key, "name": label, "synthetic": True,
                        "hit_count": len(hits),
                        "sensor_hit_counts": [sum(h["ray"] // 64 == s for h in hits) for s in range(2)],
                        "rank": obs["rank"],
                        "smallest_singular_value": obs["smallest_singular_value"],
                        "observability": obs,
                        "monte_carlo": _monte_carlo(hits, model, obs, cfg, cfg["seed"] + index),
                        "interpretation": interpretation,
                        "scene": {"units": "m", "pose_now_xy_yaw_rad": [0.0, 0.0, 0.0],
                                  "capture_poses_xy_yaw_rad": capture_poses,
                                  "walls": [{"name": w.name, "start": list(w.start), "end": list(w.end),
                                             "z_min_m": w.z_min, "z_max_m": w.z_max} for w in walls],
                                  "hits": hits}})
    axis_fov = geometry[3]
    # Entire ideal FoV envelope, not just zone centres, relative to body centre.
    lowest_direction = min(float(d[2]) for d in geometry[0])
    ideal_level_drop = cfg["max_range_m"] * math.sin(math.radians(axis_fov / 2))
    by_id = {r["id"]: r for r in results}
    corner = by_id["perpendicular_corner"]
    summary = (f"合成双 8×8 观测：单墙秩 {by_id['single_wall']['rank']}/3，直角墙秩 {corner['rank']}/3，"
               f"平行走廊秩 {by_id['parallel_corridor']['rank']}/3，开阔区秩 {by_id['open_area']['rank']}/3。"
               f"当前高度 {cfg['height_m']:.2f} m、安装俯仰 {cfg['mount_pitch_deg']:.1f}°："
               f"0.3 m / 1.1 m 低结构分别命中 {by_id['low_structure_0_3m']['hit_count']} / "
               f"{by_id['low_structure_1_1m']['hit_count']} 个理想分区中心。")
    return {"summary": summary,
            "metrics": {"scenario_count": len(results), "rays_per_sensor": 64,
                        "diagonal_fov_deg": cfg["diagonal_fov_deg"],
                        "horizontal_fov_deg": axis_fov, "vertical_fov_deg": axis_fov,
                        "datasheet_nominal_axis_fov_deg": 45.0,
                        "fov_model": cfg["fov_model"], "distance_mode": cfg["distance_mode"],
                        "yaw_length_scale_m": cfg["yaw_length_scale_m"],
                        "level_fov_envelope_min_height_m": cfg["height_m"] - ideal_level_drop,
                        "level_fov_envelope_note": "仅零机体姿态、零安装俯仰/横滚；全视场理想包络，未计基线旋转。",
                        "lowest_zone_direction_z": lowest_direction,
                        "corner_rank": corner["rank"],
                        "corner_translation_rmse_m": corner["monte_carlo"]["translation_rmse_m"],
                        "corner_yaw_rmse_deg": corner["monte_carlo"]["yaw_rmse_deg"],
                        "hardware_benchmark": False},
            "assumptions": [
                "场景由指定高度的有限墙段组成。",
                "坐标 x 前、y 左、z 上；mount_pitch_deg 正值朝上、负值朝下，安装旋转顺序为偏航、俯仰、横滚。",
                "只估计 x、y、yaw；高度与机体 roll/pitch 已知且不变化，未估计这些量的误差。",
                "65° 是标称对角 FoV。默认理想方形投影换算轴向约 48.5°；数据手册另列轴向 45°，可用 datasheet_axes 对照。真实分区需标定。",
                "8×8 每区只取中心射线，分区不是 64 支独立激光。忽略脚印、混合像素、多目标、信号状态、反射率、环境光、互扰、窗口与遮挡。",
                "默认 axial：输出距离=沿射线距离×局部光轴投影；radial 模式是另一个理想对照。实际固件输出定义和逐区修正必须验证。",
                "max_range_m 是物理射线距离截断；默认实验量程为 3 m。",
                "墙面有限、有高度、最近命中；数值雅可比冻结已知墙面关联，不把命中/失效或端点切换当连续观测。",
                f"雅可比用 q=(x,y,L·yaw)，L={cfg['yaw_length_scale_m']:.3g} m，并除以 √命中数；奇异值描述方向敏感度。",
                f"蒙特卡洛固定随机种子 {cfg['seed']}；噪声标准差 {cfg['noise_mm']:.3g} mm，同一传感器相关系数 {cfg['noise_correlation']:.3g}。",
                "默认相关噪声比 64 区完全独立的假设更保守，但 0.6 不是实测上界；可对比 correlation=1，共同偏差、混合像素和错误关联仍可能更差。",
                "蒙特卡洛对秩 3 场景进行局部拟合，使用已知地图、固定关联和接近真值的初值。",
                "bias_mm 为所有分区共享固定偏差；两传感器年龄为 latency_ms 与 latency_ms+sensor_skew_ms，匀速模型产生旧帧，估计器故意按当前帧拟合。",
            ],
            "config": cfg, "scenarios": results,
            "sources": [
                {"title": "ST VL53L5CX datasheet DS13754", "url": "https://www.st.com/resource/en/datasheet/vl53l5cx.pdf"},
                {"title": "ST staff: zone grid and on-chip distance projection", "url": "https://qa-community.st.com/topic/show?fid=48&tid=7106"},
                {"title": "ST staff: VL53L5CX driver implementation", "url": "https://community.st.com/imaging-sensors-49/vl53l5cx-driver-implementation-129686"},
            ]}
