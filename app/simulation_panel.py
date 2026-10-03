"""@file simulation_panel.py
@brief Gazebo 启停入口、多机定位指标与地图显示。
通过 WSL 启动管理脚本，定时读取运行目录中的状态、日志和相机图像；定位统计由 LocalizationMetrics 提供。
"""

import json
import math
from pathlib import Path

try:
    import numpy as np
except ImportError:
    np = None

import itertools

from PySide6.QtCore import QProcess, QRectF, Qt, QTimer, QUrl
from PySide6.QtGui import QColor, QDesktopServices, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from simulation.localization_metrics import (
    LocalizationMetrics,
    home_positions,
    map_bounds,
    read_status,
)


class LocalizationMap(QWidget):
    """@brief 在同一米制平面中绘制地形、起点框、真值轨迹和估计轨迹。
    @note snapshot 为统计快照，config 为地图配置，terrain_image 持有已转换的地形像素。
    """
    def __init__(self):
        """@brief 初始化全场视图和空的地形、定位缓存。
        """
        super().__init__()
        self.setMinimumHeight(250)
        self.snapshot = None
        self.config = {}
        self.terrain = None
        self.terrain_image = None
        self.center_zoom = False

    def toggle_center(self):
        """@brief 在全场范围与原点附近 5 m×5 m 范围之间切换。
        """
        self.center_zoom = not self.center_zoom
        self.update()

    def view_bounds(self):
        """@return (xmin, xmax, ymin, ymax)，单位 m；未提供地图边界时使用默认范围。
        """
        bounds = map_bounds(self.config) or (-6.0, 6.0, -5.0, 5.0)
        if self.center_zoom:
            return -2.5, 2.5, -2.5, 2.5
        return bounds

    def set_terrain(self, terrain):
        """@brief 将高度网格转为可重用的 RGB 地形图。
        @param terrain (x, y, z) 数组元组或 None；z 单位 m，需要 NumPy。
        @note 同一元组对象不重复转换；QImage.copy 使图像拥有像素内存，不依赖局部 NumPy 数组的寿命。
        """
        if terrain is self.terrain:
            return
        self.terrain = terrain
        self.terrain_image = None
        if terrain is not None:
            z = terrain[2]
            # 1. 用 3.7 m 归一化高度，缺测 NaN 按零着色并限幅，使高度对应连续颜色。
            ratio = np.clip(np.nan_to_num(z, nan=0.0) / 3.7, 0, 1)
            pixels = np.stack(
                [45 + 180 * ratio, 110 + 90 * (1 - ratio), 145 + 40 * (1 - ratio)],
                axis=-1,
            ).astype(np.uint8)
            # 2. 数组行序反转以对应屏幕向下的 Y 轴，再转为连续 RGB 字节。
            #    QImage 暂借 NumPy 内存，copy 后像素归图像自身持有。
            pixels = np.ascontiguousarray(pixels[::-1])
            self.terrain_image = QImage(
                pixels.data,
                pixels.shape[1],
                pixels.shape[0],
                pixels.strides[0],
                QImage.Format.Format_RGB888,
            ).copy()
        self.update()

    def set_data(self, snapshot, config):
        """@brief 保存定位快照和地图配置，并请求 Qt 重绘。
        """
        self.snapshot, self.config = snapshot, config
        self.update()

    def paintEvent(self, event):
        """@brief 等比例映射世界 XY 到屏幕，并按地图边界裁剪图层。
        @note 世界 Y 向上而屏幕 Y 向下，因此纵坐标用 y1-y；统一缩放使机体、红框和比例尺尺寸可比较。
        """
        painter = QPainter(self)
        painter.fillRect(self.rect(), self.palette().base())
        x0, x1, y0, y1 = self.view_bounds()
        pad = 28
        # 1. 取两个方向可用缩放率的较小值，保留地图宽高比并居中留白。
        #    scale 单位为像素/m；轨迹、地形、机体和起点框共用这个比例。
        scale = min(
            (self.width() - 2 * pad) / (x1 - x0), (self.height() - 2 * pad) / (y1 - y0)
        )
        left = (self.width() - (x1 - x0) * scale) / 2
        top = (self.height() - (y1 - y0) * scale) / 2

        def xy(point):
            """@brief 把世界 XY 米制位置转为控件像素坐标；反转屏幕 Y 方向。
            """
            return (left + (point[0] - x0) * scale, top + (y1 - point[1]) * scale)

        painter.setPen(QPen(self.palette().mid(), 1))
        painter.drawRect(QRectF(left, top, (x1 - x0) * scale, (y1 - y0) * scale))
        if self.terrain_image is not None:
            terrain_bounds = map_bounds(self.config) or (x0, x1, y0, y1)
            tx0, tx1, ty0, ty1 = terrain_bounds
            painter.setClipRect(QRectF(left, top, (x1 - x0) * scale, (y1 - y0) * scale))
            painter.drawImage(
                QRectF(*xy((tx0, ty1)), (tx1 - tx0) * scale, (ty1 - ty0) * scale),
                self.terrain_image,
            )
            painter.setClipping(False)
        # 2. 地形作底图，起点框按实际米制尺寸覆盖其上；裁剪只限制显示区域。
        marker = self.config.get("home_marker", {})
        size = float(marker.get("size_m", 0.30))
        width = float(marker.get("line_width_m", 0.012))
        painter.setClipRect(QRectF(left, top, (x1 - x0) * scale, (y1 - y0) * scale))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor(220, 30, 30), width * scale))
        for i, home in home_positions(self.config.get("spawns", [])).items():
            px, py = xy(home)
            painter.drawRect(
                QRectF(
                    px - size * scale / 2,
                    py - size * scale / 2,
                    size * scale,
                    size * scale,
                )
            )
            painter.drawText(int(px + size * scale / 2 + 3), int(py + 13), f"H{i}")
        painter.setClipping(False)
        painter.setPen(self.palette().text().color())
        ruler_m = 0.1 if self.center_zoom else 2.0
        ry = self.height() - 12
        painter.drawLine(pad, ry, int(pad + ruler_m * scale), ry)
        painter.drawText(pad, ry - 5, f"{ruler_m:g} m")
        if not self.snapshot:
            painter.drawText(pad, 20, "等待定位数据…")
            return
        colors = [(31, 119, 180), (214, 39, 40), (44, 160, 44), (148, 103, 189)]
        # 3. 同机真值与估计共用颜色，以虚实线区分；当前匹配位置叠加机体尺寸框。
        painter.setClipRect(QRectF(left, top, (x1 - x0) * scale, (y1 - y0) * scale))
        for i, vehicle in enumerate(self.snapshot["vehicles"]):
            color = QColor(*colors[i % len(colors)])
            for kind in ("truth", "estimate"):
                pen = QPen(color, 2)
                pen.setStyle(
                    Qt.PenStyle.DashLine if kind == "truth" else Qt.PenStyle.SolidLine
                )
                painter.setPen(pen)
                trail = self.snapshot["trails"][kind][i]
                for a, b in itertools.pairwise(trail):
                    ax, ay = xy(a)
                    bx, by = xy(b)
                    painter.drawLine(int(ax), int(ay), int(bx), int(by))
            match = vehicle.get("match")
            if match:
                painter.setPen(QPen(color, 0.5))
                px, py = xy(match["estimate"])
                extents = self.config.get("vehicle_extent_m", [0.082, 0.082, 0.0285])
                ex, ey = float(extents[0]) * scale, float(extents[1]) * scale
                painter.drawRect(QRectF(px - ex / 2, py - ey / 2, ex, ey))
                painter.setPen(QPen(color, 1))
                painter.drawPoint(int(px), int(py))
                painter.drawText(int(px) + 7, int(py) - 6, str(i))
        painter.setClipping(False)
        painter.setPen(self.palette().text().color())
        painter.drawText(pad, 20, "虚线=真值评估，实线=估计")


class GazeboPanel(QWidget):
    """@brief 管理 WSL 仿真命令与运行状态展示。
    @note process 跟踪当前管理命令；metrics 持续读取当前运行目录；800 ms 定时器刷新状态与相机画面。
    """
    def __init__(self, root):
        """@param root 项目根目录，用于地图、运行状态和文档路径。
        """
        super().__init__()
        self.root = Path(root)
        self.process = None
        self.metrics = None
        self.current_folder = None
        self.current_map = self.root / "simulation/maps/rmuc_2026_sentry26.json"
        self.map_config = {}
        self._terrain_cache = None
        self.flight_dashboard = None
        outer = QVBoxLayout(self)
        title = QLabel("多机协同定位评估")
        title.setStyleSheet("font-size:16px;font-weight:bold")
        outer.addWidget(title)
        note = QLabel("高度辅助")
        note.setWordWrap(True)
        outer.addWidget(note)
        row = QHBoxLayout()
        self.map_choice = QComboBox()
        self.map_choice.setSizePolicy(
            QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed
        )
        self.map_choice.setMinimumWidth(260)
        self._load_maps()
        row.addWidget(QLabel("地图："))
        row.addWidget(self.map_choice, 1)
        row.addWidget(QLabel("任务："))
        self.mission_choice = QComboBox()
        self.mission_choice.addItem("多机轮流撞击与返航", "team_contact")
        self.mission_choice.addItem("定点悬停", "hover")
        self.mission_choice.addItem("地图航线", "survey")
        row.addWidget(self.mission_choice)
        choose = QPushButton("选择 JSON")
        choose.clicked.connect(self.choose_map)
        row.addWidget(choose)
        guide = QPushButton("地形定位说明")
        guide.clicked.connect(self.open_guide)
        row.addWidget(guide)
        row.addWidget(QLabel("启动数量："))
        self.vehicle_count = QSpinBox()
        self.vehicle_count.setRange(1, 64)
        self.vehicle_count.setValue(4)
        self.vehicle_count.valueChanged.connect(self._set_camera_count)
        row.addWidget(self.vehicle_count)
        outer.addLayout(row)
        row = QHBoxLayout()
        for text, action in (
            ("启动多机环境", "start"),
            ("停止本次仿真", "stop"),
            ("打开 Gazebo", "gui"),
        ):
            button = QPushButton(text)
            # 默认参数保存本次 action，避免点击时所有闭包都取到循环最后的值。
            button.clicked.connect(lambda checked=False, a=action: self.command(a))
            row.addWidget(button)
        report = QPushButton("保存定位报告")
        report.clicked.connect(self.save_report)
        row.addWidget(report)
        plots = QPushButton("飞行数据曲线")
        plots.clicked.connect(self.open_flight_dashboard)
        row.addWidget(plots)
        outer.addLayout(row)
        tof_row = QHBoxLayout()
        tof_row.addWidget(QLabel("ToF 安装："))
        self.tof_mount = QComboBox()
        self.tof_mount.addItem("水平三颗 ToF", "horizontal")
        self.tof_mount.addItem("下视 20°", "down20")
        tof_row.addWidget(self.tof_mount)
        tof_row.addStretch()
        outer.addLayout(tof_row)
        self.height_only = QCheckBox("仅融合高度")
        outer.addWidget(self.height_only)
        self.uwb_anchors = QCheckBox("启用固定 UWB 锚点")
        outer.addWidget(self.uwb_anchors)
        self.fly = QCheckBox("启用多机定位飞行")
        outer.addWidget(self.fly)
        self.status = QLabel("等待选择地图和仿真运行目录。")
        self.status.setWordWrap(True)
        outer.addWidget(self.status)
        self.map_note = QLabel("")
        self.map_note.setWordWrap(True)
        outer.addWidget(self.map_note)
        self.map_view = LocalizationMap()
        zoom = QPushButton("中心放大／全场")
        zoom.clicked.connect(self.map_view.toggle_center)
        outer.addWidget(zoom)
        middle = QHBoxLayout()
        middle.addWidget(self.map_view, 3)
        self.cameras = []
        camera_grid = QGridLayout()
        self.camera_grid = camera_grid
        self.camera_area = QScrollArea()
        self.camera_area.setWidgetResizable(True)
        camera_host = QWidget()
        camera_host.setLayout(camera_grid)
        self.camera_area.setWidget(camera_host)
        middle.addWidget(self.camera_area, 2)
        self._set_camera_count(4)
        outer.addLayout(middle, 1)
        self.cards = QPlainTextEdit()
        self.cards.setReadOnly(True)
        self.cards.setMaximumHeight(180)
        outer.addWidget(self.cards)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(90)
        outer.addWidget(self.log)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(800)

    def open_flight_dashboard(self):
        """@brief 复用已打开的飞行窗口，或为当前运行目录创建窗口。
        @note destroyed 信号清空引用，避免访问已由 Qt 删除的对话框。
        """
        from app.flight_dashboard import FlightDashboard

        if self.flight_dashboard is not None:
            self.flight_dashboard.show()
            self.flight_dashboard.raise_()
            self.flight_dashboard.activateWindow()
            return
        self.refresh()
        folder = self.current_folder
        if folder is None:
            selected = QFileDialog.getExistingDirectory(
                self, "选择仿真运行目录", str(self.root / "simulation/runtime")
            )
            if not selected:
                return
            folder = Path(selected)
        self.flight_dashboard = FlightDashboard(folder, self)
        self.flight_dashboard.destroyed.connect(self._dashboard_closed)
        self.flight_dashboard.show()

    def _dashboard_closed(self):
        """@brief 对话框销毁时释放面板中的引用。
        """
        self.flight_dashboard = None

    def close_flight_dashboard(self):
        """@brief 关闭附属飞行窗口，由其关闭流程结束日志线程。
        """
        if self.flight_dashboard is not None:
            self.flight_dashboard.close()

    def closeEvent(self, event):
        """@brief 先关闭附属飞行窗口，再交给 QWidget 处理关闭事件。
        """
        self.close_flight_dashboard()
        super().closeEvent(event)

    def _set_camera_count(self, count):
        """@brief 按机数重建相机标签，使用至多四列布局。
        @param count 要显示的相机数量；索引与 camera_i.ppm 对应。
        """
        while self.cameras:
            camera = self.cameras.pop()
            self.camera_grid.removeWidget(camera)
            camera.deleteLater()
        columns = max(1, min(4, math.ceil(math.sqrt(count))))
        colors = [(31, 119, 180), (214, 39, 40), (44, 160, 44), (148, 103, 189)]
        for i in range(count):
            camera = QLabel(f"无人机 {i} · 等待 Gazebo 相机图像")
            camera.setMinimumSize(112, 72) if count > 4 else camera.setMinimumSize(
                180, 100
            )
            camera.setToolTip(f"无人机 {i} · GC4653 相机")
            camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
            camera.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
            color = colors[i % len(colors)]
            camera.setStyleSheet(
                f"background:#e5edf4;border:2px solid rgb{color};padding:4px"
            )
            self.camera_grid.addWidget(camera, i // columns, i % columns)
            self.cameras.append(camera)

    def _load_maps(self):
        """@brief 枚举可解析的地图 JSON，将路径保存在选项数据中。
        """
        self.map_choice.clear()
        for path in sorted((self.root / "simulation/maps").glob("*.json")):
            try:
                config = json.loads(path.read_text(encoding="utf-8"))
                label = config.get("label", path.stem)
                if path.name == "rmuc_2026_sentry26.json":
                    label = "2026 全场参考 · Sentry26"
                elif path.name == "calibration.json":
                    label = "标定房间"
                self.map_choice.addItem(label, str(path))
            except (OSError, ValueError):
                continue
        index = self.map_choice.findData(str(self.current_map))
        if index >= 0:
            self.map_choice.setCurrentIndex(index)

    def choose_map(self):
        """@brief 把用户选择的地图加入选项并选中，供下次启动使用。
        """
        path, _ = QFileDialog.getOpenFileName(
            self, "选择地图 JSON", str(self.root / "simulation/maps"), "JSON (*.json)"
        )
        if path:
            if self.map_choice.findData(path) < 0:
                self.map_choice.addItem(Path(path).name, path)
            self.map_choice.setCurrentIndex(self.map_choice.findData(path))

    def command(self, action):
        """@brief 把界面选项转为 WSL 管理脚本参数并异步执行。
        @param action start、stop 或 gui。
        @note Windows 盘符映射到 /mnt/盘符；高度模式优先于飞行选项。process 非空时拒绝并发管理命令。
        """
        if self.process:
            return
        map_path = Path(self.map_choice.currentData() or self.current_map)
        script = (
            "/mnt/"
            + self.root.drive[0].lower()
            + str(self.root)[2:].replace("\\", "/")
            + "/simulation/launch_linux.py"
        )
        map_arg = (
            "/mnt/" + map_path.drive[0].lower() + str(map_path)[2:].replace("\\", "/")
        )
        args = ["-d", "Ubuntu-22.04", "--exec", "python3", script, action]
        if action == "start":
            args += [
                "--gui",
                "--dynamics",
                "micro",
                "--companions",
                "--map",
                map_arg,
                "--mission",
                self.mission_choice.currentData(),
                "--vehicle-count",
                str(self.vehicle_count.value()),
            ]
            args += [
                "--uwb-anchors" if self.uwb_anchors.isChecked() else "--no-uwb-anchors"
            ]
            if self.tof_mount.currentData() == "horizontal":
                args += ["--horizontal-tof"]
            if self.height_only.isChecked():
                args += ["--vertical-only"]
            elif self.fly.isChecked():
                args += ["--fly"]
        process = QProcess(self)
        self.process = process
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: self.log.appendPlainText(
                bytes(process.readAllStandardOutput())
                .decode("utf-8", "replace")
                .replace(chr(0), "")
            )
        )
        process.finished.connect(
            lambda code, state: self._finished(process, action, code)
        )
        process.start("wsl.exe", args)

    def _finished(self, process, action, code):
        """@brief 接收管理命令退出，释放 QProcess 并刷新运行状态。
        """
        self.log.appendPlainText(
            f"{action} 命令退出码 {code}；请查看定位指标和进程状态。"
        )
        self.process = None
        process.deleteLater()
        self.refresh()

    def refresh(self):
        """@brief 读取 manager.json 定位运行目录，再刷新指标、地图和相机。
        @note 运行目录变化时重建统计器；相同目录沿用增量读取状态，避免每次重读完整日志。
        """
        state_path = self.root / "simulation/runtime/manager.json"
        if not state_path.exists():
            return
        try:
            value = json.loads(state_path.read_text(encoding="utf-8"))
            path = value["run_dir"]
            if path.startswith("/mnt/"):
                path = path[5].upper() + ":" + path[6:]
            folder = Path(path)
            # 切换运行目录需要重置增量读取位置，防止把上一轮的轨迹混入新运行。
            if folder != self.current_folder:
                self.current_folder = folder
                self.metrics = LocalizationMetrics(folder, manager=value)
            elif self.metrics is not None:
                self.metrics.manager = value
            self.map_config = dict(value.get("map", {}))
            configured_count = len(self.map_config.get("spawns", []))
            if configured_count >= 1 and configured_count != len(self.cameras):
                self._set_camera_count(configured_count)
            self.metrics.set_homes(self.map_config.get("spawns", []))
            self._update_map_context(self.map_config)
            snapshot = self.metrics.poll(include_trails=True)
            self.map_view.set_data(snapshot, self.map_config)
            self.cards.setPlainText(self._format(snapshot))
            for i, camera in enumerate(self.cameras):
                image = folder / "camera" / f"camera_{i}.ppm"
                if image.exists():
                    pixmap = QPixmap(str(image))
                    if not pixmap.isNull():
                        camera.setPixmap(
                            pixmap.scaled(
                                camera.size(), Qt.AspectRatioMode.KeepAspectRatio
                            )
                        )
            map_label = self.map_config.get("label", "未知")
            if self.map_config.get("name") == "rmuc_2026_sentry26":
                map_label = "Sentry26 2026 全场地形 · 开放边界派生版"
            team = snapshot.get("team_status", {})
            phase = self._phase_text(team)
            self.status.setText(
                ("仿真已请求停止" if value.get("stopped") else "仿真进程已启动")
                + f" · {folder.name} · 地图：{map_label} · {phase}"
            )
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.status.setText(f"等待有效仿真状态与定位日志：{type(exc).__name__}")

    def _update_map_context(self, config):
        """@brief 更新地图尺寸说明并按文件路径及修改时间缓存高度网格。
        @param config 当前地图字典，会补入 vehicle_extent_m 供绘图使用。
        @note 相对网格路径以源地图目录为基准；读取失败时清除地形图，保留其他地图显示。
        """
        bounds = map_bounds(config)
        audit = read_status(self.root / "simulation/docs/scale-verification.json")
        extent = audit.get("vehicle_extent_m", [0.082, 0.082, 0.0285])
        config["vehicle_extent_m"] = extent
        marker = config.get("home_marker", {})
        pieces = [
            f"无人机 {len(config.get('spawns', []))} 架 · UWB 基站 {len(config.get('uwb_anchors', []))} 个"
        ]
        if bounds:
            pieces.append(
                f"网格外包 {bounds[1] - bounds[0]:.2f} × {bounds[3] - bounds[2]:.2f} m"
            )
        pieces.append(
            "机体碰撞尺寸 " + " × ".join(f"{float(v):g}" for v in extent) + " m"
        )
        pieces.append(
            f"红框 {float(marker.get('size_m', 0.30)):g} m · 线宽 {float(marker.get('line_width_m', 0.012)):g} m"
        )
        homes = home_positions(config.get("spawns", []))
        if homes:
            pieces.append(
                "起点高度 "
                + "、".join(f"{v:g}" for v in sorted({p[2] for p in homes.values()}))
                + " m"
            )
        mission = config.get("team_mission", {})
        if "cruise_z_m" in mission:
            pieces.append(f"任务巡航高度 {float(mission['cruise_z_m']):g} m")
        self.map_note.setText(" · ".join(pieces))
        source = config.get("source_map") or str(
            self.root
            / "simulation/maps"
            / (config.get("name", "calibration") + ".json")
        )
        terrain = config.get("terrain_height_grid")
        if not source or not terrain or np is None:
            self.map_view.set_terrain(None)
            return
        try:
            if source.startswith("/mnt/"):
                source = source[5].upper() + ":" + source[6:]
            path = Path(terrain)
            if str(terrain).startswith("/mnt/"):
                path = Path(str(terrain)[5].upper() + ":" + str(terrain)[6:])
            elif not path.is_absolute():
                path = Path(source).parent / path
            path = path.resolve()
            key = (str(path), path.stat().st_mtime_ns)
            # 路径相同但内容更新时修改时间会改变，重新加载；未变时复用数组元组，
            # set_terrain 也能按对象身份跳过昂贵的逐像素转换。
            if self._terrain_cache is None or self._terrain_cache[0] != key:
                with np.load(path, allow_pickle=False) as data:
                    self._terrain_cache = (key, (data["x"], data["y"], data["z"]))
            self.map_view.set_terrain(self._terrain_cache[1])
        except (OSError, KeyError, ValueError) as exc:
            self.map_view.set_terrain(None)
            self.map_note.setText(
                self.map_note.text() + f" · 无法读取地形网格：{type(exc).__name__}"
            )

    def open_guide(self):
        """@brief 打开项目中的多机任务说明。
        """
        path = self.root / "simulation/docs/team-mission.html"
        if path.exists():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))
        else:
            self.status.setText("地形定位说明文件不存在。")

    @staticmethod
    def _phase_text(team):
        """@brief 将任务状态字典格式化为阶段、计时与机号摘要。
        @param team 任务状态，缺少字段时使用等待或空值显示。
        @return 单行界面摘要；未知阶段保留原始名称。
        """
        phase = team.get("phase", "")
        phases = {
            "WAITING": "等待协同起飞",
            "WAIT_READY": "等待多机就绪",
            "LAUNCH": "协同起飞",
            "TAKEOFF": "协同起飞",
            "TRANSIT": "编队航行",
            "ASSEMBLE": "集结",
            "APPROACH": "接近目标",
            "CONTACT": "轮流撞击",
            "RETREAT": "退出接触区",
            "RETURN": "共同返航",
            "RETURN_ALL": "共同返航",
            "LAND": "降落",
            "LANDING": "降落",
            "LAND_ALL": "多机降落",
            "COMPLETE": "任务完成",
            "ABORT": "任务中止",
            "ABORT_RETURN": "中止并返航",
            "ABORTED": "任务已中止",
            "PAUSED": "任务暂停",
        }
        text = phases.get(phase, "等待任务阶段" if not phase else f"任务阶段：{phase}")
        if team.get("flight_elapsed_s", 0) > 0:
            text += f" · 飞行计时 {team['flight_elapsed_s']:.1f} s"
        active = team.get("active_vehicle")
        if type(active) is int and active >= 0:
            text += f" · 当前机 {active}"
        completed = team.get("completed", [])
        if isinstance(completed, list):
            text += " · 已撞击：" + (
                "、".join(str(i) for i in completed if type(i) is int and i >= 0)
                or "无"
            )
        skipped = team.get("skipped", [])
        if skipped:
            text += " · 跳过机号：" + "、".join(map(str, skipped))
        if team.get("telemetry_errors"):
            text += f" · 状态写入重试 {team['telemetry_errors']} 次"
        if team.get("send_errors"):
            text += f" · 指令发送失败 {team['send_errors']} 次"
        return text

    def _format(self, snapshot):
        """@brief 将定位快照格式化为每机与机间统计文本。
        @param snapshot 已由 LocalizationMetrics 计算的指标。
        @return 多行文本；缺失指标显示 --，零值仍按有效数字显示。
        @note 这里只组织显示，不重新计算误差；区分实时回家点偏移与落地/完成时保存的偏移。
        """
        rows = [
            self._phase_text(snapshot.get("team_status", {})),
            f"范围：{'完整日志' if self.metrics and self.metrics.full else '接入后的日志片段'} · 对齐容差：{snapshot['alignment_tolerance_us'] / 1000:.0f} ms",
        ]
        for v in snapshot["vehicles"]:
            validity = v["validity"]

            def pct(key, validity=validity):
                """@brief 将当前机的有效比例转成百分数，None 保留为缺失标记。
                """
                return "--" if validity[key] is None else f"{validity[key] * 100:.0f}%"

            error = (
                "--"
                if v["rmse_m"] is None
                else f"均方根误差 {v['rmse_m']:.2f} m / 最大误差 {v['max_error_m']:.2f} m"
            )
            flow = (
                "--"
                if v["flow_validity"] is None
                else f"{v['flow_validity'] * 100:.0f}%"
            )
            confidence = (
                "--"
                if v["position_confidence"] is None
                else f"{v['position_confidence']:.2f}"
            )
            rows.append(
                f"机 {v['vehicle']} · { {'live': '更新中', 'stopped': '已停止', 'pending': '等待数据', 'stale': '数据过期', 'unmatched': '等待时间匹配'}.get(v['status'], v['status']) } · XY {error} · 有效 {pct('valid')} · 结构定位 {pct('structural')} · 机间定位 {pct('peer')} · 光流 {flow} · 置信度 {confidence} · 模式 {v['localization_mode'] or '--'} · 路程 {v['truth_path_m']:.1f} m"
            )
            team = snapshot.get("team_status", {})
            states = team.get("vehicles", [])
            state = (
                next(
                    (
                        s
                        for s in states
                        if isinstance(s, dict) and s.get("vehicle") == v["vehicle"]
                    ),
                    {},
                )
                if isinstance(states, list)
                else {}
            )
            if state:
                rows.append(
                    "  队伍通信："
                    + (
                        "更新中"
                        if state.get("fresh")
                        else "数据过期"
                        if state.get("seen")
                        else "等待连接"
                    )
                )
                if state.get("degraded"):
                    rows.append(
                        "  当前状态：" + state.get("degraded_reason", "等待恢复")
                    )
            home = v.get("home_error")
            if home:

                def offset_text(offset):
                    """@brief 格式化米制回家点偏移，保留 dx/dy 的正负方向。
                    """
                    return (
                        "--"
                        if offset is None
                        else f"{offset['xy_m']:.3f} m（dx {offset['dx_m']:+.3f}，dy {offset['dy_m']:+.3f} m）"
                    )

                rows.append(
                    f"  实时回家点偏移：估计 {offset_text(home['estimate'])} · 真值评估 {offset_text(home['truth'])}"
                )
                final = home.get("landing") or home.get("final")
                if final:
                    label = "落地误差" if home.get("landing") else "完成时回家点误差"
                    rows.append(
                        f"  {label} · 仿真时间 {final['sim_time_us'] / 1e6:.3f} s：估计 {offset_text(final['estimate'])} · 真值评估 {offset_text(final['truth'])}"
                    )
        pairs = (
            "；".join(
                f"{p['vehicles'][0]}-{p['vehicles'][1]}: {p['distance_error_m']:.2f} m（{'有效' if p['valid'] else '无效'}）"
                for p in snapshot["pairs"]
            )
            or "--"
        )
        common = snapshot["common_translation"]
        drift = (
            "--"
            if common is None
            else f"{common['magnitude_m']:.2f} m（{'有效' if common['valid'] else '无效'}）"
        )
        rows.append(f"机间距离误差：{pairs} · 共同平移漂移：{drift}")
        if snapshot["backlog"]:
            rows.append("日志正在追赶最新数据")
        return "\n".join(rows)

    def save_report(self):
        """@brief 将当前统计快照保存为 UTF-8 JSON，供离线分析使用。
        """
        if not self.metrics:
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "保存定位报告",
            str(self.root / "simulation/runtime/localization_report.json"),
            "JSON (*.json)",
        )
        if path:
            Path(path).write_text(
                json.dumps(self.metrics.snapshot(), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
