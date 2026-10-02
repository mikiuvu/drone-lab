"""Gazebo controls and read-only multivehicle localization assessment."""

import json
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
    QSizePolicy,
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
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(250)
        self.snapshot = None
        self.config = {}
        self.terrain = None
        self.terrain_image = None
        self.center_zoom = False

    def toggle_center(self):
        self.center_zoom = not self.center_zoom
        self.update()

    def view_bounds(self):
        bounds = map_bounds(self.config) or (-6.0, 6.0, -5.0, 5.0)
        if self.center_zoom:
            return -2.5, 2.5, -2.5, 2.5
        return bounds

    def set_terrain(self, terrain):
        if terrain is self.terrain:
            return
        self.terrain = terrain
        self.terrain_image = None
        if terrain is not None:
            z = terrain[2]
            ratio = np.clip(np.nan_to_num(z, nan=0.0) / 3.7, 0, 1)
            pixels = np.stack(
                [45 + 180 * ratio, 110 + 90 * (1 - ratio), 145 + 40 * (1 - ratio)],
                axis=-1,
            ).astype(np.uint8)
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
        self.snapshot, self.config = snapshot, config
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), self.palette().base())
        x0, x1, y0, y1 = self.view_bounds()
        pad = 28
        scale = min(
            (self.width() - 2 * pad) / (x1 - x0), (self.height() - 2 * pad) / (y1 - y0)
        )
        left = (self.width() - (x1 - x0) * scale) / 2
        top = (self.height() - (y1 - y0) * scale) / 2

        def xy(point):
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
        marker = self.config.get("home_marker", {})
        size = float(marker.get("size_m", 0.30))
        width = float(marker.get("line_width_m", 0.012))
        painter.setClipRect(QRectF(left, top, (x1-x0)*scale, (y1-y0)*scale))
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
        painter.setClipRect(QRectF(left, top, (x1-x0)*scale, (y1-y0)*scale))
        for i, vehicle in enumerate(self.snapshot["vehicles"]):
            color = QColor(*colors[i])
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
    def __init__(self, root):
        super().__init__()
        self.root = Path(root)
        self.process = None
        self.metrics = None
        self.current_folder = None
        self.current_map = self.root / "simulation/maps/rmuc_2026_sentry26.json"
        self.map_config = {}
        self._terrain_cache = None
        outer = QVBoxLayout(self)
        title = QLabel("四机协同定位评估")
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
        self.mission_choice.addItem("四机轮流撞击与返航", "team_contact")
        self.mission_choice.addItem("定点悬停", "hover")
        self.mission_choice.addItem("地图航线", "survey")
        row.addWidget(self.mission_choice)
        choose = QPushButton("选择 JSON")
        choose.clicked.connect(self.choose_map)
        row.addWidget(choose)
        guide = QPushButton("地形定位说明")
        guide.clicked.connect(self.open_guide)
        row.addWidget(guide)
        outer.addLayout(row)
        row = QHBoxLayout()
        for text, action in (
            ("启动四机环境", "start"),
            ("停止本次仿真", "stop"),
            ("打开 Gazebo", "gui"),
        ):
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, a=action: self.command(a))
            row.addWidget(button)
        report = QPushButton("保存定位报告")
        report.clicked.connect(self.save_report)
        row.addWidget(report)
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
        self.fly = QCheckBox("启用四机定位飞行")
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
        for i in range(4):
            camera = QLabel(f"无人机 {i} · 等待 Gazebo 相机图像")
            camera.setMinimumSize(220, 110)
            camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
            camera.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
            camera.setStyleSheet(
                "background:#e5edf4;border:1px solid #bdcddc;padding:4px"
            )
            camera_grid.addWidget(camera, i // 2, i % 2)
            self.cameras.append(camera)
        middle.addLayout(camera_grid, 2)
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

    def _load_maps(self):
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
        path, _ = QFileDialog.getOpenFileName(
            self, "选择地图 JSON", str(self.root / "simulation/maps"), "JSON (*.json)"
        )
        if path:
            if self.map_choice.findData(path) < 0:
                self.map_choice.addItem(Path(path).name, path)
            self.map_choice.setCurrentIndex(self.map_choice.findData(path))

    def command(self, action):
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
        self.log.appendPlainText(
            f"{action} 命令退出码 {code}；请查看定位指标和进程状态。"
        )
        self.process = None
        process.deleteLater()
        self.refresh()

    def refresh(self):
        state_path = self.root / "simulation/runtime/manager.json"
        if not state_path.exists():
            return
        try:
            value = json.loads(state_path.read_text(encoding="utf-8"))
            path = value["run_dir"]
            if path.startswith("/mnt/"):
                path = path[5].upper() + ":" + path[6:]
            folder = Path(path)
            if folder != self.current_folder:
                self.current_folder = folder
                self.metrics = LocalizationMetrics(folder, manager=value)
            elif self.metrics is not None:
                self.metrics.manager = value
            self.map_config = dict(value.get("map", {}))
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
        except (OSError, ValueError, KeyError, TypeError):
            self.status.setText("等待有效仿真状态与定位日志。")

    def _update_map_context(self, config):
        bounds = map_bounds(config)
        audit = read_status(self.root / "simulation/docs/scale-verification.json")
        extent = audit.get("vehicle_extent_m", [0.082, 0.082, 0.0285])
        config["vehicle_extent_m"] = extent
        marker = config.get("home_marker", {})
        pieces = []
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
            if self._terrain_cache is None or self._terrain_cache[0] != key:
                with np.load(path, allow_pickle=False) as data:
                    self._terrain_cache = (key, (data["x"], data["y"], data["z"]))
            self.map_view.set_terrain(self._terrain_cache[1])
        except (OSError, KeyError, ValueError):
            self.map_view.set_terrain(None)
            self.map_note.setText(self.map_note.text() + " · 无法读取地形网格")

    def open_guide(self):
        path = self.root / "simulation/docs/team-mission.html"
        if path.exists():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))
        else:
            self.status.setText("地形定位说明文件不存在。")

    @staticmethod
    def _phase_text(team):
        phase = team.get("phase", "")
        phases = {
            "WAITING": "等待协同起飞",
            "WAIT_READY": "等待四机就绪",
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
            "LAND_ALL": "四机降落",
            "COMPLETE": "任务完成",
            "ABORT": "任务中止",
            "ABORT_RETURN": "中止并返航",
            "ABORTED": "任务已中止",
            "PAUSED": "任务暂停",
        }
        text = phases.get(phase, "等待任务阶段" if not phase else f"任务阶段：{phase}")
        active = team.get("active_vehicle")
        if type(active) is int and 0 <= active < 4:
            text += f" · 当前机 {active}"
        completed = team.get("completed", [])
        if isinstance(completed, list):
            text += " · 已撞击：" + (
                "、".join(str(i) for i in completed if type(i) is int and 0 <= i < 4)
                or "无"
            )
        return text

    def _format(self, snapshot):
        rows = [
            self._phase_text(snapshot.get("team_status", {})),
            f"范围：{'完整日志' if self.metrics and self.metrics.full else '接入后的日志片段'} · 对齐容差：{snapshot['alignment_tolerance_us'] / 1000:.0f} ms",
        ]
        for v in snapshot["vehicles"]:
            validity = v["validity"]

            def pct(key, validity=validity):
                return "--" if validity[key] is None else f"{validity[key] * 100:.0f}%"

            error = (
                "--"
                if v["rmse_m"] is None
                else f"均方根误差 {v['rmse_m']:.2f} m / 最大误差 {v['max_error_m']:.2f} m"
            )
            rows.append(
                f"机 {v['vehicle']} · { {'live': '更新中', 'stopped': '已停止', 'pending': '等待数据', 'stale': '数据过期', 'unmatched': '等待时间匹配'}.get(v['status'], v['status']) } · XY {error} · 有效 {pct('valid')} · 结构定位 {pct('structural')} · 机间定位 {pct('peer')} · 路程 {v['truth_path_m']:.1f} m"
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
            home = v.get("home_error")
            if home:

                def offset_text(offset):
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
