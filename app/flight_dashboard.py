"""@file flight_dashboard.py
@brief 飞行日志跟随、曲线绘制与同步时间游标。
FlightLoader 读取历史快照，FlightDashboard 在主线程绘图；横轴使用日志中的仿真秒，播放节奏使用电脑单调时钟。
图表配色与任务阶段定义取自 app.theme，深浅主题随主窗口切换。
"""

import math
import time
from pathlib import Path

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from matplotlib.text import Text
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSlider,
    QVBoxLayout,
)

from app import theme
from simulation.flight_data import FlightHistory, aligned_samples, export_csv

# 阶段中文名与配色由 app.theme 提供，通过同名常量供外部读取。
PHASE_NAMES = theme.PHASE_NAMES
PHASE_COLORS = theme.PHASE_COLORS

# 每个子图的内容描述：标题、纵轴单位与是否需要等比例坐标。
PANELS = [
    ("XY 轨迹", "Y / m"),
    ("高度", "世界 Z / m"),
    ("速度", "m/s"),
    ("水平定位误差", "m"),
    ("航向", "°"),
    ("定位置信度", "0–1"),
]


def smooth(values, window):
    """@brief 对含 NaN 的序列做滑动平均，NaN 不参与均值也不被填平。
    @param values 一维浮点数组，允许含 NaN。
    @param window 窗口长度，单位采样点；小于 2 时原样返回。
    @return 与输入等长的新数组；窗口内全为 NaN 的位置保持 NaN。
    @note 用「求和 ÷ 有效个数」而不是 np.nanmean：后者在整窗都是 NaN 时会发出
    RuntimeWarning 并返回 NaN，在每帧重绘的路径上会刷满日志。
    """
    if window < 2 or len(values) == 0:
        return values
    padded = np.concatenate([np.full(window - 1, np.nan), values])
    stacked = np.lib.stride_tricks.sliding_window_view(padded, window)
    present = ~np.isnan(stacked)
    count = present.sum(axis=1)
    total = np.where(present, stacked, 0.0).sum(axis=1)
    return np.where(count > 0, total / np.maximum(count, 1), np.nan)


class ResponsiveCanvas(FigureCanvasQTAgg):
    """@brief 让图形英寸尺寸跟随控件像素尺寸的画布。
    @note Qt 会按控件大小缩放图形，若图形英寸尺寸固定，字号与线宽会被一起缩小。
    这里在每次尺寸变化时反算英寸尺寸，使 9 pt 标签在任何窗口大小下都渲染成 9 pt。
    """

    def resizeEvent(self, event):
        """@brief 控件尺寸变化后同步图形英寸尺寸。
        @param event Qt 尺寸事件。
        """
        super().resizeEvent(event)
        width, height = self.width(), self.height()
        if width > 80 and height > 80:
            dpi = self.figure.get_dpi()
            self.figure.set_size_inches(width / dpi, height / dpi, forward=False)


class FlightLoader(QThread):
    """@brief 持续读取飞行日志的后台线程，向界面发送独立历史快照。
    """
    loaded = Signal(object)
    failed = Signal(str)

    def __init__(self, folder, parent=None):
        """@param folder 含 companion 等日志的运行目录。
        @param parent Qt 父对象，管理线程对象的归属。
        """
        super().__init__(parent)
        self.folder = folder

    def run(self):
        """@brief 分批追赶历史并继续跟随文件追加。
        @note 有积压时每次最多 poll 两轮，快照节流至约 0.7 秒；追平时立即发出变化并降低轮询频率。
        """
        try:
            history = FlightHistory(self.folder)
            last_emit = 0.0
            while not self.isInterruptionRequested():
                # 增量读取与快照发送分别节流：历史追赶保持吞吐，界面减少重绘次数；
                # 快照与后续累积的数据分离，工作线程继续读取时不改界面正在使用的数据。
                changed = history.poll(rounds=2)
                now = time.monotonic()
                if changed and (now - last_emit > 0.7 or not history.backlog):
                    self.loaded.emit(history.snapshot())
                    last_emit = now
                self.msleep(5 if history.backlog else 350)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.failed.emit(str(exc))


class FlightDashboard(QDialog):
    """@brief 用同一仿真时间游标关联轨迹、高度、速度、误差、有效性与任务阶段。
    @note snapshot 是后台数据快照；plot_data 是本次绘图缓存，cursor 为仿真秒数。控件与 Matplotlib 操作在界面线程执行。
    """
    def __init__(self, folder=None, parent=None):
        """@param folder 可选运行目录，提供时立即开始读取。
        @param parent 所属窗口；对话框关闭后由 Qt 删除。
        """
        super().__init__(parent)
        self.setWindowTitle("飞行数据可视化")
        self.resize(1460, 980)
        self.setMinimumSize(1000, 720)
        self.snapshot = None
        self.worker = None
        self.cursor = 0.0
        self.plot_data = {}  # 机号 -> (估计数组, 真值数组, 对齐误差 m, 速度模 m/s)。
        self.cursor_lines = []
        self.position_markers = []
        self.dark = False
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)
        row = QHBoxLayout()
        row.setSpacing(10)
        title = QLabel("飞行数据可视化")
        title.setProperty("role", "pageTitle")
        row.addWidget(title)
        row.addStretch()
        row.addWidget(QLabel("无人机"))
        self.vehicle = QComboBox()
        self.vehicle.setMinimumWidth(130)
        self.vehicle.currentIndexChanged.connect(self.redraw)
        row.addWidget(self.vehicle)
        self.truth = QCheckBox("显示真实轨迹")
        self.truth.setChecked(True)
        self.truth.toggled.connect(self.redraw)
        row.addWidget(self.truth)
        self.follow = QCheckBox("跟随最新数据")
        self.follow.setChecked(True)
        self.follow.toggled.connect(self.follow_changed)
        row.addWidget(self.follow)
        layout.addLayout(row)

        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(QLabel("平滑窗口"))
        self.smoothing = QComboBox()
        for label, window in (("原始", 1), ("0.25 s", 5), ("0.5 s", 10), ("1 s", 20)):
            self.smoothing.addItem(label, window)
        self.smoothing.setToolTip("仅影响速度与误差曲线的显示，不改变导出的原始数据。")
        self.smoothing.currentIndexChanged.connect(self.redraw)
        row.addWidget(self.smoothing)
        for text, callback, variant in [
            ("选择运行目录", self.choose_folder, ""),
            ("导出图片", self.save_image, ""),
            ("导出 CSV", self.save_csv, ""),
        ]:
            button = QPushButton(text)
            button.setMinimumHeight(32)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            if variant:
                button.setProperty("variant", variant)
            button.clicked.connect(callback)
            row.addWidget(button)
        row.addStretch()
        layout.addLayout(row)

        self.summary = QLabel("选择含 companion 日志的仿真运行目录。")
        self.summary.setWordWrap(True)
        self.summary.setProperty("role", "note")
        layout.addWidget(self.summary)
        self.figure = Figure(figsize=(15, 10), layout="constrained")
        self.canvas = ResponsiveCanvas(self.figure)
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.toolbar.setMaximumHeight(32)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas, 1)
        self.readout = QLabel("拖动时间轴查看同一时刻的位置、高度、速度与误差。")
        self.readout.setWordWrap(True)
        self.readout.setProperty("role", "hint")
        layout.addWidget(self.readout)
        row = QHBoxLayout()
        row.setSpacing(10)
        self.play = QPushButton("▶ 播放")
        self.play.setMinimumHeight(32)
        self.play.setCursor(Qt.CursorShape.PointingHandCursor)
        self.play.clicked.connect(self.toggle_play)
        row.addWidget(self.play)
        self.speed = QComboBox()
        for speed in (0.25, 0.5, 1, 2, 4):
            self.speed.addItem(f"{speed:g}×", speed)
        self.speed.setCurrentIndex(2)
        row.addWidget(self.speed)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 10000)
        self.slider.sliderPressed.connect(lambda: self.follow.setChecked(False))
        self.slider.valueChanged.connect(self.slider_changed)
        row.addWidget(self.slider, 1)
        self.time_label = QLabel("0.00 / 0.00 s")
        self.time_label.setMinimumWidth(140)
        row.addWidget(self.time_label)
        layout.addLayout(row)
        self.status = QLabel(
            "估计采样间隔 ≥0.05 s · 误差时间对齐容差 0.10 s · 实线为估计，虚线为真实值"
        )
        self.status.setWordWrap(True)
        self.status.setProperty("role", "hint")
        layout.addWidget(self.status)
        self.player = QTimer(self)
        self.player.setInterval(40)
        self.player.timeout.connect(self.advance_playback)
        self.canvas.mpl_connect("button_press_event", self.plot_clicked)
        self.canvas.mpl_connect("motion_notify_event", self.hover)
        self.start_time = 0.0
        self.end_time = 0.0
        if folder:
            self.load_folder(folder)

    def set_theme(self, dark):
        """@brief 切换图表深浅主题并重绘。
        @param dark 为真表示深色主题。
        """
        self.dark = dark
        self.figure.set_facecolor(theme.tokens(dark)["surface"])
        self.redraw()

    def stop_worker(self):
        """@brief 请求读取线程中断并等待退出后删除，供换目录和关闭窗口使用。
        """
        if self.worker:
            self.worker.requestInterruption()
            self.worker.wait()
            self.worker.deleteLater()
            self.worker = None

    def load_folder(self, folder):
        """@brief 停止旧读取器与播放，清空画布并启动新目录读取器。
        @param folder 新运行目录。
        """
        self.stop_worker()
        self.player.stop()
        self.play.setText("▶ 播放")
        self.snapshot = None
        self.figure.clear()
        self.canvas.draw_idle()
        self.summary.setText(f"读取飞行日志：{Path(folder).name}")
        self.worker = FlightLoader(Path(folder), self)
        self.worker.loaded.connect(self.accept_snapshot)
        self.worker.failed.connect(self.load_failed)
        self.worker.start()

    def load_failed(self, error):
        """@brief 将读取线程报告的错误显示到摘要栏。
        """
        self.summary.setText("读取失败：" + error)

    def choose_folder(self):
        """@brief 选择运行目录，默认定位到当前快照的目录。
        """
        current = self.snapshot["folder"] if self.snapshot else ""
        folder = QFileDialog.getExistingDirectory(self, "选择仿真运行目录", current)
        if folder:
            self.load_folder(folder)

    def accept_snapshot(self, snapshot):
        """@brief 接收历史快照，更新机号、时间范围和读取状态。
        @param snapshot FlightHistory.snapshot 返回的字典。
        @note 忽略旧读取器迟到的信号；手动回看时仍接收快照，但已有曲线不会随每次追加重建。
        """
        sender = self.sender()
        # 旧线程停止前发出的信号可能仍在 Qt 队列中，换目录后必须识别发送者。
        if isinstance(sender, FlightLoader) and sender is not self.worker:
            return
        had_data = bool(self.plot_data)
        self.snapshot = snapshot
        ids = snapshot["ids"]
        previous_ids = [
            self.vehicle.itemData(i) for i in range(1, self.vehicle.count())
        ]
        if ids != previous_ids:
            selected = self.vehicle.currentData()
            self.vehicle.blockSignals(True)
            self.vehicle.clear()
            self.vehicle.addItem("全部无人机", None)
            for i in ids:
                self.vehicle.addItem(f"无人机 {i}", i)
            index = self.vehicle.findData(selected)
            self.vehicle.setCurrentIndex(max(0, index))
            self.vehicle.blockSignals(False)
        series = [values for values in snapshot["estimates"].values() if values]
        if not series:
            self.summary.setText("等待定位估计数据…")
            return
        self.start_time = min(s[0][0] for s in series)
        self.end_time = max(s[-1][0] for s in series)
        if self.follow.isChecked():
            self.cursor = self.end_time
        else:
            self.cursor = min(max(self.cursor, self.start_time), self.end_time)
        state = (
            "读取历史中"
            if snapshot["backlog"]
            else "运行已结束"
            if set(ids) <= set(snapshot["ended"])
            else "实时跟随"
        )
        counts = snapshot["fusion"]
        accepted = " · ".join(
            f"{name} {sum(n for (i, sensor, ok), n in counts.items() if sensor == key and ok):,}"
            for key, name in [
                ("optical_flow", "光流"),
                ("tof", "ToF"),
                ("uwb", "机间 UWB"),
                ("uwb_anchor", "基站"),
            ]
        )
        self.summary.setText(
            f"{Path(snapshot['folder']).name} · {len(ids)} 架 · {state} · "
            f"{self.start_time:.1f}–{self.end_time:.1f} s\n融合接收：{accepted}"
        )
        self.status.setText(
            "估计采样间隔 ≥0.05 s · 误差时间对齐容差 0.10 s · 实线为估计，虚线为真实值"
            + (
                f" · 早期曲线已移出缓存 {snapshot['dropped']} 点"
                if snapshot["dropped"]
                else ""
            )
        )
        if self.follow.isChecked() or not had_data:
            self.redraw()

    def selected_ids(self):
        """@return 当前选中的机号列表；全部模式返回快照机号，无快照返回空列表。
        """
        if not self.snapshot:
            return []
        vehicle = self.vehicle.currentData()
        return self.snapshot["ids"] if vehicle is None else [vehicle]

    def redraw(self, *_):
        """@brief 在统一的图表主题下重建图形；可直接连接带参数的 Qt 信号。
        """
        if not self.snapshot:
            return
        with theme.mpl_context(self.dark):
            self._draw()

    def _draw(self):
        """@brief 从快照重建六幅曲线、有效性泳道、任务阶段条和游标标记。
        @note 估计列依次为时间 s、XYZ m、VXYZ m/s、航向 rad、置信度及两个有效标志。真值列为时间、XYZ、航向。
        @note 使用 constrained layout 自动排布；机号颜色由全局图例统一说明，各子图不再各画一份，
        否则六份图例会互相挤压并遮住曲线。
        """
        t = theme.tokens(self.dark)
        colors = theme.categorical(self.dark)
        self.figure.clear()
        grid = self.figure.add_gridspec(
            5, 2, height_ratios=[1, 1, 1, 0.45, 0.16]
        )
        self.axes = [
            self.figure.add_subplot(grid[r, c]) for r in range(3) for c in range(2)
        ]
        self.validity = self.figure.add_subplot(grid[3, :])
        self.timeline = self.figure.add_subplot(grid[4, :])
        for axis, (title, ylabel) in zip(self.axes, PANELS):
            axis.set_title(title, loc="left", fontsize=11, pad=6)
            axis.set_ylabel(ylabel, fontsize=9)
        self.validity.set_title("定位有效性", loc="left", fontsize=10, pad=4)
        self.plot_data = {}
        self.position_markers = []
        vehicle_handles = []
        selected = self.selected_ids()
        window = self.smoothing.currentData() or 1
        for vehicle in selected:
            # 1. 把每机历史转成数组，按时间匹配真值得到 XY 误差；未匹配处保留 NaN，
            #    曲线出现缺口，避免把缺失真值绘成零误差。速度模包含 XYZ 三个分量。
            estimate = np.asarray(
                self.snapshot["estimates"].get(vehicle, []), dtype=float
            )
            truth = np.asarray(self.snapshot["truths"].get(vehicle, []), dtype=float)
            if not len(estimate):
                continue
            color = colors[vehicle % len(colors)]
            label = f"机 {vehicle}"
            times = estimate[:, 0]
            errors = np.asarray([v[2] for v in aligned_samples(estimate, truth)])
            speed = np.linalg.norm(estimate[:, 4:7], axis=1)
            self.plot_data[vehicle] = (estimate, truth, errors, speed)
            handle, = self.axes[0].plot(
                estimate[:, 1], estimate[:, 2], color=color, lw=1.4, label=label
            )
            vehicle_handles.append(handle)
            self.axes[1].plot(times, estimate[:, 3], color=color, lw=1.3)
            # 单机视图里同一根轴还叠了 VX/VY/VZ 分量，模长要单独命名才不会被误读成其中一个分量。
            self.axes[2].plot(
                times,
                smooth(speed, window),
                color=color,
                lw=1.3,
                label="速度模" if len(selected) == 1 else label,
            )
            self.axes[3].plot(times, smooth(errors, window), color=color, lw=1.3)
            yaw = np.rad2deg(estimate[:, 7]).copy()
            # 2. 航向在角度边界跳变时用 NaN 断线，避免跨 ±180°画出虚假的长斜线。
            yaw[np.r_[False, np.abs(np.diff(yaw)) > 180]] = np.nan
            self.axes[4].plot(times, yaw, color=color, lw=1.3)
            self.axes[5].plot(times, estimate[:, 8], color=color, lw=1.3)
            if len(selected) == 1:
                for index, name, component_color in [
                    (4, "VX", colors[0]),
                    (5, "VY", colors[1]),
                    (6, "VZ", colors[2]),
                ]:
                    self.axes[2].plot(
                        times,
                        smooth(estimate[:, index], window),
                        lw=1,
                        alpha=0.75,
                        color=component_color,
                        label=name,
                    )
            if self.truth.isChecked() and len(truth):
                self.axes[0].plot(
                    truth[:, 1], truth[:, 2], color=color, lw=1, ls="--", alpha=0.65
                )
                self.axes[1].plot(
                    truth[:, 0], truth[:, 3], color=color, lw=1, ls="--", alpha=0.65
                )
                true_yaw = np.rad2deg(truth[:, 4]).copy()
                true_yaw[np.r_[False, np.abs(np.diff(true_yaw)) > 180]] = np.nan
                self.axes[4].plot(
                    truth[:, 0], true_yaw, color=color, lw=1, ls="--", alpha=0.65
                )
            (marker,) = self.axes[0].plot([], [], "o", color=color, ms=7, mec="white")
            self.position_markers.append((vehicle, marker))

        if vehicle_handles:
            # 多机时只列机号；单机时同一根轴上还有 VX/VY/VZ 分量，需连同分量一起列出。
            if len(selected) == 1:
                speed_handles, speed_labels = self.axes[2].get_legend_handles_labels()
            else:
                speed_handles, speed_labels = vehicle_handles, [
                    f"机 {v}" for v in self.plot_data
                ]
            self.axes[2].legend(
                speed_handles,
                speed_labels,
                fontsize=7,
                loc="upper right",
                ncol=min(5, len(speed_handles)),
                framealpha=0.85,
            )

        # 3. 有效性使用独立泳道：每机一条，XY 有效画实心块，光流有效画半透明叠加，
        #    不再与 0–1 的置信度曲线共用一根纵轴。
        lanes = list(self.plot_data)
        if lanes:
            self.validity.set_ylim(-0.7, len(lanes) - 0.3)
            # 机号写在泳道内部左端：泳道很扁时纵轴刻度标签会互相重叠，内嵌文字更稳。
            self.validity.set_yticks([])
            for i, vehicle in enumerate(lanes):
                y = len(lanes) - 1 - i
                estimate = self.plot_data[vehicle][0]
                times = estimate[:, 0]
                color = colors[vehicle % len(colors)]
                self.validity.fill_between(
                    times, y - 0.34, y + 0.34, where=estimate[:, 10] > 0.5,
                    step="post", color=color, alpha=0.9, linewidth=0,
                )
                self.validity.fill_between(
                    times, y - 0.34, y + 0.34, where=estimate[:, 9] > 0.5,
                    step="post", color=color, alpha=0.35, linewidth=0,
                )
                self.validity.text(
                    0.004,
                    y,
                    f"机 {vehicle}",
                    transform=self.validity.get_yaxis_transform(),
                    fontsize=7,
                    va="center",
                    ha="left",
                    color=theme.readable_on(
                        tuple(int(color[i : i + 2], 16) for i in (1, 3, 5))
                    ),
                )
            self.validity.legend(
                handles=[
                    Rectangle((0, 0), 1, 1, color=t["ink_soft"], alpha=0.9),
                    Rectangle((0, 0), 1, 1, color=t["ink_soft"], alpha=0.35),
                ],
                labels=["XY 有效", "光流有效"],
                fontsize=7,
                loc="upper right",
                ncol=2,
                framealpha=0.85,
            )
        else:
            self.validity.set_yticks([])
        self.validity.set_xlabel("仿真时间 / s", fontsize=8)

        config = self.snapshot["config"]
        for home in config.get("spawns", []):
            if home["id"] in selected:
                size = config.get("home_marker", {}).get("size_m", 0.3)
                self.axes[0].add_patch(
                    Rectangle(
                        (home["x"] - size / 2, home["y"] - size / 2),
                        size,
                        size,
                        fill=False,
                        ec="#ce4d4d",
                        lw=1,
                    )
                )
        self.axes[0].set_xlabel("X / m", fontsize=9)
        self.axes[0].set_aspect("equal", adjustable="datalim")
        self.axes[5].set_ylim(-0.05, 1.08)
        limits = (self.start_time, max(self.start_time + 0.1, self.end_time))
        for axis in self.axes[1:]:
            axis.set_xlim(*limits)
            axis.set_xlabel("仿真时间 / s", fontsize=8)
        self._annotate_errors(selected)
        # 4. 相邻阶段事件围成时间区间；末段延伸到最新采样，并裁剪到曲线时间范围。
        #    阶段名直接写在色块内，不再另画图例，避免与阶段条自身争位置。
        phases = self.snapshot["phases"]
        for index, (stamp, phase, active) in enumerate(phases):
            end = phases[index + 1][0] if index + 1 < len(phases) else self.end_time
            start, end = max(stamp, self.start_time), min(end, self.end_time)
            if end <= start:
                continue
            color = theme.phase_color(phase, self.dark)
            self.timeline.axvspan(start, end, color=color, alpha=0.9)
            if end - start > (limits[1] - limits[0]) / 32:
                name = theme.phase_name(phase)
                if phase == "CONTACT" and active is not None and active >= 0:
                    name += f"{active}"
                self.timeline.text(
                    (start + end) / 2,
                    0.5,
                    name,
                    ha="center",
                    va="center",
                    fontsize=7,
                    color=theme.readable_on(
                        tuple(int(color[i : i + 2], 16) for i in (1, 3, 5))
                    ),
                )
        for stamp, vehicle in self.snapshot["impacts"]:
            if vehicle in selected:
                self.axes[3].axvline(
                    stamp,
                    color=colors[vehicle % len(colors)],
                    alpha=0.45,
                    ls=":",
                    lw=0.9,
                )
        self.timeline.set_xlim(*limits)
        self.timeline.set_yticks([])
        self.timeline.set_xlabel("任务阶段 · 仿真时间 / s", fontsize=8)
        self.cursor_lines = [
            axis.axvline(self.cursor, color=t["ink"], alpha=0.55, lw=0.9)
            for axis in [*self.axes[1:], self.validity, self.timeline]
        ]
        for label in self.figure.findobj(Text):
            label.set_fontfamily(["Microsoft YaHei", "DejaVu Sans"])
        self.update_cursor()
        self.canvas.draw_idle()

    def _annotate_errors(self, selected):
        """@brief 在误差子图上标注均方根、95 分位与最大值。
        @param selected 当前选中的机号列表；多机时按机号分行显示。
        @note 只统计有限值；完全没有有效误差时标注「无有效真值匹配」。
        """
        t = theme.tokens(self.dark)
        lines = []
        for vehicle in selected:
            errors = self.plot_data[vehicle][2]
            finite = errors[np.isfinite(errors)]
            if not len(finite):
                lines.append(f"机 {vehicle}：无有效真值匹配")
                continue
            lines.append(
                f"机 {vehicle}  均方根 {np.sqrt(np.mean(finite ** 2)):.3f}"
                f" · p95 {np.percentile(finite, 95):.3f}"
                f" · 最大 {finite.max():.3f} m"
            )
        if not lines:
            return
        self.axes[3].text(
            0.015,
            0.97,
            "\n".join(lines[:4]),
            transform=self.axes[3].transAxes,
            fontsize=7,
            linespacing=1.35,
            va="top",
            ha="left",
            color=t["ink_soft"],
            bbox=dict(
                boxstyle="round,pad=0.35",
                facecolor=t["surface_alt"],
                edgecolor=t["border"],
                alpha=0.9,
            ),
        )

    def update_cursor(self):
        """@brief 同步各时间轴竖线、XY 位置标记、滑块和文字读数。
        @note XY 标记使用游标之前最后一个估计，超过 0.20 s 则隐藏；滑块更新时阻断信号以避免回调递归。
        """
        for line in self.cursor_lines:
            line.set_xdata([self.cursor, self.cursor])
        for vehicle, marker in self.position_markers:
            estimate = self.plot_data[vehicle][0]
            # 有序时间数组中找第一个大于游标的位置，再退一格，得到不晚于游标的帧；
            # 与文字读数的最近邻策略不同，这里不会提前显示未来位置。
            index = np.searchsorted(estimate[:, 0], self.cursor, side="right") - 1
            if index >= 0 and self.cursor - estimate[index, 0] <= 0.2:
                marker.set_data([estimate[index, 1]], [estimate[index, 2]])
            else:
                marker.set_data([], [])
        self.slider.blockSignals(True)
        # 程序同步进度条时阻断 valueChanged，否则会进入 slider_changed 并关闭跟随。
        self.slider.setValue(
            round(
                10000
                * (self.cursor - self.start_time)
                / max(0.001, self.end_time - self.start_time)
            )
        )
        self.slider.blockSignals(False)
        self.time_label.setText(f"{self.cursor:.2f} / {self.end_time:.2f} s")
        self.show_values(self.cursor)
        self.canvas.draw_idle()

    def show_values(self, stamp):
        """@brief 显示指定仿真时刻附近的高度、速度、误差和任务阶段。
        @param stamp 仿真时间，单位 s。
        @note 读数取距离该时刻最近的估计，超过 0.15 s 显示无数据；误差无有效真值匹配时显示缺失。
        """
        phase = (
            next(
                (
                    event
                    for event in reversed(self.snapshot["phases"])
                    if event[0] <= stamp
                ),
                None,
            )
            if self.snapshot
            else None
        )
        phase_text = (
            f" · {theme.phase_name(phase[1])}"
            if phase
            else ""
        )
        if phase and phase[2] >= 0:
            phase_text += f" · 当前机 {phase[2]}"
        parts = [f"仿真时间 {stamp:.2f} s{phase_text}"]
        for vehicle, (estimate, _truth, errors, speed) in self.plot_data.items():
            index = int(np.argmin(np.abs(estimate[:, 0] - stamp)))
            if abs(estimate[index, 0] - stamp) > 0.15:
                parts.append(f"机 {vehicle}：无对应数据")
                continue
            row = estimate[index]
            error = f"{errors[index]:.3f}" if math.isfinite(errors[index]) else "--"
            parts.append(
                f"机 {vehicle}  Z {row[3]:.2f} m · 速度 {speed[index]:.2f} m/s · XY误差 {error} m"
            )
        self.readout.setText("  |  ".join(parts[:5]))

    def slider_changed(self, value):
        """@brief 将 0–10000 的滑块位置转换为仿真时间，并退出实时跟随。
        """
        self.follow.setChecked(False)
        self.cursor = self.start_time + value / 10000 * (
            self.end_time - self.start_time
        )
        self.update_cursor()

    def follow_changed(self, checked):
        """@brief 启用跟随时停止播放，把游标移到最新数据并重绘。
        """
        if checked:
            self.player.stop()
            self.play.setText("▶ 播放")
            self.cursor = self.end_time
            self.redraw()

    def toggle_play(self):
        """@brief 切换手动播放；从末尾启动时回到起点，并记录电脑单调时钟。
        """
        if self.player.isActive():
            self.player.stop()
            self.play.setText("▶ 播放")
            return
        if not self.snapshot:
            return
        self.follow.setChecked(False)
        if self.cursor >= self.end_time:
            self.cursor = self.start_time
        self.play_clock = time.monotonic()
        self.player.start()
        self.play.setText("⏸ 暂停")

    def advance_playback(self):
        """@brief 按真实经过秒数乘倍率推进仿真游标，到数据末尾自动暂停。
        """
        now = time.monotonic()
        self.cursor = min(
            self.end_time,
            self.cursor + (now - self.play_clock) * self.speed.currentData(),
        )
        self.play_clock = now
        self.update_cursor()
        if self.cursor >= self.end_time:
            self.player.stop()
            self.play.setText("▶ 播放")

    def plot_clicked(self, event):
        """@brief 点击时间曲线或阶段条时定位游标。
        @note XY 图横轴不是时间，且工具栏缩放/平移模式要保留自身鼠标操作，因此不参与定位。
        """
        if (
            event.inaxes
            in [
                *getattr(self, "axes", [])[1:],
                getattr(self, "validity", None),
                getattr(self, "timeline", None),
            ]
            and event.xdata is not None
            and not self.toolbar.mode
        ):
            self.follow.setChecked(False)
            self.cursor = min(self.end_time, max(self.start_time, event.xdata))
            self.update_cursor()

    def hover(self, event):
        """@brief 悬停时间曲线时预览该时刻读数，不改变播放游标。
        """
        axes = [*getattr(self, "axes", [])[1:], getattr(self, "validity", None)]
        if event.inaxes in axes and event.xdata is not None:
            self.show_values(event.xdata)

    def save_image(self):
        """@brief 将当前完整图形导出为 PNG 或 PDF，保存当前曲线选择和游标。
        """
        if not self.snapshot:
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "导出飞行曲线",
            str(Path(self.snapshot["folder"]) / "flight-data.png"),
            "PNG (*.png);;PDF (*.pdf)",
        )
        if path:
            try:
                with theme.mpl_context(self.dark):
                    self.figure.savefig(path, dpi=170)
                self.status.setText("图片已保存：" + path)
            except OSError as exc:
                QMessageBox.warning(self, "导出失败", str(exc))

    def save_csv(self):
        """@brief 导出快照中选定机号的采样与对齐误差，不按当前游标截断数据。
        """
        if not self.snapshot:
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "导出采样曲线数据",
            str(Path(self.snapshot["folder"]) / "flight-data.csv"),
            "CSV (*.csv)",
        )
        if path:
            try:
                export_csv(self.snapshot, path, self.selected_ids())
                self.status.setText("CSV 已保存：" + path)
            except OSError as exc:
                QMessageBox.warning(self, "导出失败", str(exc))

    def closeEvent(self, event):
        """@brief 关闭前停止播放定时器并等待日志线程退出。
        """
        self.player.stop()
        self.stop_worker()
        super().closeEvent(event)


def main():
    """@brief 独立启动飞行曲线窗口，--run-dir 可指定运行目录，--dark 使用深色主题。
    """
    import argparse
    import sys

    from PySide6.QtWidgets import QApplication

    from simulation.localization_metrics import resolve_run_dir

    parser = argparse.ArgumentParser(description="飞行数据曲线与回放")
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--dark", action="store_true")
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    theme.apply_theme(app, args.dark)
    root = Path(__file__).resolve().parents[1]
    folder = args.run_dir or resolve_run_dir(root)
    window = FlightDashboard(folder)
    window.set_theme(args.dark)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
