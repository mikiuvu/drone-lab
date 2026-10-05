"""@file simulation_reader.py
@brief Gazebo 页面后台读取：日志统计、地形着色、相机解码。
读取器由一个工作线程独占；发送的字典和图像作为只读快照交给界面。
"""

import copy
import json
from pathlib import Path

import numpy as np
from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

from app import theme
from simulation.localization_metrics import LocalizationMetrics, read_status


def local_path(value):
    """@brief 将 WSL 挂载盘路径转换为 Windows 路径，普通路径原样保留。"""
    value = str(value)
    if value.startswith("/mnt/") and len(value) > 6:
        value = value[5].upper() + ":" + value[6:]
    return Path(value)


def terrain_image(z):
    """@brief 按 0–3.7 m 色标生成拥有独立像素内存的地形图。
    @param z 二维高度数组，单位 m；数组行方向与屏幕 Y 方向相反。
    """
    ratio = np.clip(np.nan_to_num(z, nan=0.0) / 3.7, 0, 1)
    stops = np.array([[int(color[i:i + 2], 16) for i in (1, 3, 5)]
                      for color in theme.HEIGHT_STOPS[::2]], dtype=float)
    scaled = ratio * 2
    low = np.floor(scaled).astype(int)
    frac = (scaled - low)[..., None]
    pixels = stops[low] * (1 - frac) + stops[np.minimum(low + 1, 2)] * frac
    pixels = np.ascontiguousarray(pixels[::-1].astype(np.uint8))
    return QImage(pixels.data, pixels.shape[1], pixels.shape[0], pixels.strides[0],
                  QImage.Format.Format_RGB888).copy()


class SimulationReader:
    """@brief 保持增量日志游标及文件缓存；调用方保证同一时刻只有一个读取任务。"""

    def __init__(self, root):
        self.root = Path(root)
        self.folder = None
        self.metrics = None
        self.terrain_cache = None
        self.camera_cache = {}

    def read(self, cancelled=lambda: False):
        """@brief 读取一轮快照；阶段之间检查停止请求，退出时不发送半成品。"""
        state_path = self.root / "simulation/runtime/manager.json"
        try:
            value = json.loads(state_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return None
        folder = local_path(value["run_dir"])
        config = dict(value.get("map", {}))
        count = len(config.get("spawns", [])) or None
        if (self.metrics is None or folder != self.folder
                or (count and count != self.metrics.vehicle_count)):
            self.metrics = LocalizationMetrics(folder, manager=value, vehicle_count=count)
            self.folder = folder
            self.terrain_cache = None
            self.camera_cache.clear()
        self.metrics.manager = value
        self.metrics.set_homes(config.get("spawns", []))
        snapshot = self.metrics.poll(include_trails=True)
        if cancelled():
            return None
        audit = read_status(self.root / "simulation/docs/scale-verification.json")
        config["vehicle_extent_m"] = audit.get("vehicle_extent_m", [0.082, 0.082, 0.0285])
        terrain, terrain_error = self.read_terrain(config)
        if cancelled():
            return None
        cameras = {}
        for i in range(self.metrics.vehicle_count):
            if cancelled():
                return None
            path = folder / "camera" / f"camera_{i}.ppm"
            try:
                stat = path.stat()
                key = (stat.st_mtime_ns, stat.st_size)
                cached = self.camera_cache.get(i)
                if cached is None or cached[0] != key:
                    image = QImage(str(path))
                    # 写入未完成或图像损坏时下次再解码，不能缓存失败结果。
                    if image.isNull():
                        self.camera_cache.pop(i, None)
                        continue
                    self.camera_cache[i] = (key, image)
                cameras[i] = self.camera_cache[i][1]
            except OSError:
                self.camera_cache.pop(i, None)
        # 切换运行目录可能发生在慢盘读取期间，旧目录结果不应覆盖新运行。
        current = json.loads(state_path.read_text(encoding="utf-8"))
        if current.get("run_dir") != value["run_dir"]:
            return None
        return dict(folder=folder, manager=value, config=config,
                    snapshot=copy.deepcopy(snapshot), terrain=terrain,
                    terrain_error=terrain_error, cameras=cameras)

    def read_terrain(self, config):
        """@brief 地形只在文件变化时重新加载并着色；失败清空旧图并保留错误信息。"""
        terrain = config.get("terrain_height_grid")
        if not terrain:
            self.terrain_cache = None
            return None, ""
        source = config.get("source_map") or str(
            self.root / "simulation/maps" / (config.get("name", "calibration") + ".json"))
        path = local_path(terrain)
        if not path.is_absolute():
            path = local_path(source).parent / path
        try:
            path = path.resolve()
            stat = path.stat()
            key = (str(path), stat.st_mtime_ns, stat.st_size)
            if self.terrain_cache is None or self.terrain_cache[0] != key:
                with np.load(path, allow_pickle=False) as data:
                    image = terrain_image(data["z"])
                self.terrain_cache = (key, image)
            return self.terrain_cache[1], ""
        except (OSError, KeyError, ValueError, TypeError) as exc:
            self.terrain_cache = None
            return None, f"无法读取地形网格：{type(exc).__name__}：{exc}"


class SimulationRefresh(QThread):
    """@brief 一次后台读取任务。finished 后才能开始下一次，刷新不形成排队积压。"""

    loaded = Signal(object)
    failed = Signal(str)

    def __init__(self, reader, parent=None):
        super().__init__(parent)
        self.reader = reader

    def run(self):
        try:
            result = self.reader.read(self.isInterruptionRequested)
            if not self.isInterruptionRequested():
                self.loaded.emit(result)
        except Exception as exc:
            if not self.isInterruptionRequested():
                self.failed.emit(f"{type(exc).__name__}：{exc}")
