"""@file simulation_panel.py
@brief Gazebo 启停入口、多机定位指标与地图显示。
通过 WSL 启动管理脚本，定时读取运行目录中的状态、日志和相机图像；定位统计由 LocalizationMetrics 提供。
地图自绘坐标轴与高度色标，指标以表格呈现，配色统一取自 app.theme。
"""

import json
import math
from pathlib import Path

from PySide6.QtCore import QPointF, QProcess, QRectF, Qt, QTimer, QUrl, Signal
from PySide6.QtGui import (
    QColor,
    QDesktopServices,
    QFont,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
)
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from app import theme
from app.simulation_reader import SimulationReader, SimulationRefresh
from simulation.localization_metrics import (
    home_positions,
    map_bounds,
)

# 状态字典到中文与色调的映射：表格用色调给状态列上色，不需要额外图标。
STATUS_LABELS = {
    "live": ("更新中", "ok"),
    "stopped": ("已停止", ""),
    "pending": ("等待数据", "warn"),
    "stale": ("数据过期", "warn"),
    "unmatched": ("等待时间匹配", "warn"),
}


def hint(text):
    """@brief 创建次要说明文字标签。
    @param text 说明文本。
    @return 应用了 hint 角色的 QLabel。
    """
    label = QLabel(text)
    label.setProperty("role", "hint")
    label.setWordWrap(True)
    return label


class ColorbarLegend(QWidget):
    """@brief 地形高度色标图例：连续渐变条加米制刻度。
    @note 与地图共用 theme.HEIGHT_STOPS，使颜色与高度数值的对应关系可读。
    """

    def __init__(self, parent=None):
        """@param parent 所属控件；必须传入，否则会成为独立顶层窗口。"""
        super().__init__(parent)
        self.setFixedHeight(34)
        self.ceiling = 3.7
        self.dark = False

    def configure(self, ceiling, dark):
        """@brief 更新高度上限与主题。
        @param ceiling 色标上端对应的高度，单位 m。
        @param dark 为真表示深色主题。
        """
        self.ceiling, self.dark = float(ceiling), dark
        self.update()

    def paintEvent(self, event):
        """@brief 绘制渐变条、刻度与单位说明。
        @note 用 QLinearGradient 一次填充，避免逐像素画竖线。
        """
        from PySide6.QtGui import QLinearGradient

        t = theme.tokens(self.dark)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        margin = 30
        bar = QRectF(margin, 5, max(10, self.width() - 2 * margin), 10)
        gradient = QLinearGradient(bar.left(), 0, bar.right(), 0)
        for i, stop in enumerate(theme.HEIGHT_STOPS):
            gradient.setColorAt(i / (len(theme.HEIGHT_STOPS) - 1), QColor(stop))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(gradient)
        p.drawRoundedRect(bar, 3, 3)
        p.setPen(QPen(QColor(t["border_strong"]), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(bar, 3, 3)
        p.setFont(QFont("Microsoft YaHei UI", 9))
        p.setPen(QColor(t["ink_muted"]))
        p.drawText(
            QRectF(0, 2, margin - 6, 16),
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            "0",
        )
        p.drawText(
            QRectF(bar.right() + 6, 2, margin, 16),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            f"{self.ceiling:g} m",
        )
        p.drawText(
            QRectF(margin, 17, self.width() - 2 * margin, 14),
            Qt.AlignmentFlag.AlignCenter,
            "地形高度",
        )


class LocalizationMap(QWidget):
    """@brief 在同一米制平面中绘制地形、起点框、真值轨迹和估计轨迹。
    @note snapshot 为统计快照，config 为地图配置，terrain_image 持有已转换的地形像素。
    """

    def __init__(self):
        """@brief 初始化全场视图和空的地形、定位缓存。"""
        super().__init__()
        self.setMinimumHeight(280)
        self.snapshot = None
        self.config = {}
        self.terrain_image = None
        self.center_zoom = False
        self.dark = False
        self.legend = ColorbarLegend(self)

    def set_theme(self, dark):
        """@brief 切换深浅主题并重绘。
        @param dark 为真表示深色主题。
        """
        self.dark = dark
        self.legend.dark = dark
        self.update()

    def toggle_center(self):
        """@brief 在全场范围与原点附近 5 m×5 m 范围之间切换。"""
        self.center_zoom = not self.center_zoom
        self.update()

    def view_bounds(self):
        """@return (xmin, xmax, ymin, ymax)，单位 m；未提供地图边界时使用默认范围。"""
        bounds = map_bounds(self.config) or (-6.0, 6.0, -5.0, 5.0)
        if self.center_zoom:
            return -2.5, 2.5, -2.5, 2.5
        return bounds

    def set_terrain(self, image):
        """@brief 接收后台已着色的地形 QImage；None 清空地图底图。"""
        self.terrain_image = image
        self.update()

    def set_data(self, snapshot, config):
        """@brief 保存定位快照和地图配置，并请求 Qt 重绘。"""
        self.snapshot, self.config = snapshot, config
        self.update()

    def _nice_step(self, span):
        """@brief 为一组坐标轴挑选 1/2/5 系列的整数刻度间隔。
        @param span 轴覆盖的米数。
        @return 刻度间隔，单位 m。
        """
        raw = span / 6.0
        for step in (0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50):
            if raw <= step:
                return step
        return 100.0

    def paintEvent(self, event):
        """@brief 等比例映射世界 XY 到屏幕，绘制网格、刻度、图层与色标。
        @note 世界 Y 向上而屏幕 Y 向下，因此纵坐标用 y1-y；统一缩放使机体、红框和比例尺尺寸可比较。
        """
        t = theme.tokens(self.dark)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(t["surface"]))
        x0, x1, y0, y1 = self.view_bounds()
        pad = 40
        band = 36
        # 1. 取两个方向可用缩放率的较小值，保留地图宽高比并居中留白。
        #    scale 单位为像素/m；轨迹、地形、机体和起点框共用这个比例。
        scale = min(
            (self.width() - 2 * pad) / (x1 - x0),
            (self.height() - 2 * pad - band) / (y1 - y0),
        )
        left = (self.width() - (x1 - x0) * scale) / 2
        top = pad + (self.height() - 2 * pad - band - (y1 - y0) * scale) / 2

        def xy(point):
            """@brief 把世界 XY 米制位置转为控件像素坐标；反转屏幕 Y 方向。"""
            return (left + (point[0] - x0) * scale, top + (y1 - point[1]) * scale)

        frame = QRectF(left, top, (x1 - x0) * scale, (y1 - y0) * scale)

        # 2. 先铺地形底图，再画网格与刻度，保证刻度始终压在图层之上可读。
        if self.terrain_image is not None:
            terrain_bounds = map_bounds(self.config) or (x0, x1, y0, y1)
            tx0, tx1, ty0, ty1 = terrain_bounds
            painter.setClipRect(frame)
            painter.drawImage(
                QRectF(*xy((tx0, ty1)), (tx1 - tx0) * scale, (ty1 - ty0) * scale),
                self.terrain_image,
            )
            painter.setClipping(False)

        step = self._nice_step(x1 - x0)
        painter.setFont(QFont("Microsoft YaHei UI", 9))
        painter.setClipRect(frame)
        grid_pen = QPen(QColor(255, 255, 255, 60) if self.terrain_image is not None else QColor(t["grid"]), 1)
        painter.setPen(grid_pen)
        tick = math.ceil(x0 / step) * step
        while tick <= x1:
            px, _ = xy((tick, y0))
            painter.drawLine(QRectF(px, frame.top(), 0, frame.height()).topLeft(), QRectF(px, frame.bottom(), 0, 0).topLeft())
            tick = round(tick + step, 6)
        tick = math.ceil(y0 / step) * step
        while tick <= y1:
            _, py = xy((x0, tick))
            painter.drawLine(QRectF(frame.left(), py, 0, 0).topLeft(), QRectF(frame.right(), py, 0, 0).topLeft())
            tick = round(tick + step, 6)
        painter.setClipping(False)

        # 3. 坐标轴刻度：标签画在框外，地形和轨迹不会遮挡数值。
        painter.setPen(QColor(t["ink_muted"]))
        tick = math.ceil(x0 / step) * step
        while tick <= x1:
            px, _ = xy((tick, y0))
            painter.drawLine(QRectF(px, frame.bottom(), 0, 4).topLeft(), QRectF(px, frame.bottom() + 4, 0, 0).topLeft())
            painter.drawText(
                QRectF(px - 30, frame.bottom() + 5, 60, 15),
                Qt.AlignmentFlag.AlignCenter,
                f"{tick:g}",
            )
            tick = round(tick + step, 6)
        tick = math.ceil(y0 / step) * step
        while tick <= y1:
            _, py = xy((x0, tick))
            painter.drawLine(QRectF(frame.left() - 4, py, 0, 0).topLeft(), QRectF(frame.left(), py, 0, 0).topLeft())
            painter.drawText(
                QRectF(frame.left() - 44, py - 8, 38, 16),
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                f"{tick:g}",
            )
            tick = round(tick + step, 6)
        painter.drawText(
            QRectF(frame.left() - 44, frame.bottom() + 5, 38, 15),
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            "Y/m",
        )
        painter.drawText(
            QRectF(frame.right() - 40, frame.bottom() + 5, 40, 15),
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            "X / m",
        )

        # 4. 起点框按实际米制尺寸绘制，并在地图边缘裁剪。
        marker = self.config.get("home_marker", {})
        size = float(marker.get("size_m", 0.30))
        width = float(marker.get("line_width_m", 0.012))
        painter.setClipRect(frame)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor(220, 60, 60), max(1.2, width * scale)))
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
            painter.setFont(QFont("Microsoft YaHei UI", 9))
            painter.setPen(QColor(200, 40, 40) if not self.dark else QColor(240, 110, 110))
            painter.drawText(int(px + size * scale / 2 + 4), int(py + 12), f"H{i}")
            painter.setPen(QPen(QColor(220, 60, 60), max(1.2, width * scale)))

        if not self.snapshot:
            painter.setClipping(False)
            painter.setPen(QColor(t["ink_muted"]))
            painter.setFont(QFont("Microsoft YaHei UI", 12))
            painter.drawText(frame, Qt.AlignmentFlag.AlignCenter, "等待定位数据…")
            self.legend.setVisible(False)
            painter.setPen(QPen(QColor(t["border_strong"]), 1))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawRect(frame)
            return
        self.legend.setVisible(self.terrain_image is not None)
        self.legend.setGeometry(
            int(frame.left()), int(frame.bottom() + 22), int(frame.width()), 34
        )
        if self.terrain_image is not None:
            self.legend.configure(3.7, self.dark)

        # 5. 同机真值与估计共用颜色，以虚实线区分；当前匹配位置叠加机体尺寸框。
        #    轨迹用 QPainterPath 一次提交，逐段 drawLine 在长轨迹下会产生上万次调用。
        colors = theme.categorical(self.dark)
        for i, vehicle in enumerate(self.snapshot["vehicles"]):
            color = QColor(colors[i % len(colors)])
            for kind in ("truth", "estimate"):
                pen = QPen(color, 2)
                pen.setStyle(
                    Qt.PenStyle.DashLine if kind == "truth" else Qt.PenStyle.SolidLine
                )
                painter.setPen(pen)
                trail = self.snapshot["trails"][kind][i]
                if len(trail) < 2:
                    continue
                path = QPainterPath()
                path.moveTo(*xy(trail[0]))
                for point in trail[1:]:
                    path.lineTo(*xy(point))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawPath(path)
            match = vehicle.get("match")
            if match:
                px, py = xy(match["estimate"])
                extents = self.config.get("vehicle_extent_m", [0.082, 0.082, 0.0285])
                ex, ey = float(extents[0]) * scale, float(extents[1]) * scale
                painter.setPen(QPen(color, 1.4))
                painter.setBrush(QColor(color.red(), color.green(), color.blue(), 60))
                painter.drawRect(QRectF(px - ex / 2, py - ey / 2, ex, ey))
                painter.setBrush(color)
                painter.drawEllipse(QRectF(px - 2.5, py - 2.5, 5, 5))
                # 6. 机号标签加深色底衬，密集编队时仍能读清属于哪一架。
                label = f"机 {i}"
                painter.setFont(QFont("Microsoft YaHei UI", 9))
                metrics = painter.fontMetrics()
                box = QRectF(
                    px + 7,
                    py - 20,
                    metrics.horizontalAdvance(label) + 10,
                    16,
                )
                painter.setPen(Qt.PenStyle.NoPen)
                painter.setBrush(QColor(color.red(), color.green(), color.blue(), 220))
                painter.drawRoundedRect(box, 4, 4)
                painter.setPen(QColor(theme.readable_on((color.red(), color.green(), color.blue()))))
                painter.drawText(box, Qt.AlignmentFlag.AlignCenter, label)
        painter.setClipping(False)

        # 7. 图例固定在地图左上角，说明虚实线含义与比例尺。
        painter.setFont(QFont("Microsoft YaHei UI", 9))
        legend_items = [("实线 估计", None), ("虚线 真值评估", None)]
        painter.setPen(QPen(QColor(t["border_strong"]), 1))
        painter.setBrush(QColor(t["surface"]))
        box = QRectF(frame.left() + 8, frame.top() + 8, 118, 46)
        painter.drawRoundedRect(box, 6, 6)
        y = box.top() + 14
        for i, (text, _) in enumerate(legend_items):
            pen = QPen(QColor(t["ink_soft"]), 2)
            if i == 1:
                pen.setStyle(Qt.PenStyle.DashLine)
            painter.setPen(pen)
            painter.drawLine(
                QPointF(box.left() + 8, y - 4), QPointF(box.left() + 26, y - 4)
            )
            painter.setPen(QColor(t["ink_soft"]))
            painter.drawText(QPointF(box.left() + 32, y), text)
            y += 17
        ruler_m = 0.1 if self.center_zoom else 2.0
        painter.setPen(QPen(QColor(t["ink"]), 2))
        painter.drawLine(
            QPointF(frame.left() + 8, frame.bottom() - 10),
            QPointF(frame.left() + 8 + ruler_m * scale, frame.bottom() - 10),
        )
        painter.setPen(QColor(t["ink_soft"]))
        painter.drawText(
            QPointF(frame.left() + 12 + ruler_m * scale, frame.bottom() - 6),
            f"{ruler_m:g} m",
        )
        painter.setPen(QPen(QColor(t["border_strong"]), 1))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(frame)


class GazeboPanel(QWidget):
    """@brief 管理 WSL 仿真命令与运行状态展示。
    @note 800 ms 定时器请求后台刷新；界面只绘制快照，读取器独占增量游标。
    """
    shutdown_complete = Signal()

    def __init__(self, root):
        """@param root 项目根目录，用于地图、运行状态和文档路径。"""
        super().__init__()
        self.root = Path(root)
        self.process = None
        self.command_error = ""
        self.snapshot = None
        self.reader = SimulationReader(self.root)
        self.refresh_worker = None
        self._refresh_pending = False
        self._closing = False
        self._shutdown_notified = False
        self.current_folder = None
        self.current_map = self.root / "simulation/maps/rmuc_2026_sentry26.json"
        self.map_config = {}
        self.flight_dashboard = None
        self.dark = False
        outer = QVBoxLayout(self)
        outer.setContentsMargins(18, 16, 18, 16)
        outer.setSpacing(14)
        title = QLabel("Gazebo 多机仿真")
        title.setProperty("role", "pageTitle")
        outer.addWidget(title)

        setup = QGroupBox("仿真配置")
        setup_layout = QVBoxLayout(setup)
        setup_layout.setContentsMargins(16, 14, 16, 16)
        setup_layout.setSpacing(10)
        row = QHBoxLayout()
        row.setSpacing(10)
        self.map_choice = QComboBox()
        self.map_choice.setSizePolicy(
            QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed
        )
        self.map_choice.setMinimumWidth(260)
        self._load_maps()
        row.addWidget(QLabel("地图"))
        row.addWidget(self.map_choice, 1)
        row.addWidget(QLabel("任务"))
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
        row.addWidget(QLabel("启动数量"))
        self.vehicle_count = QSpinBox()
        self.vehicle_count.setRange(1, 64)
        self.vehicle_count.setValue(4)
        self.vehicle_count.setSuffix(" 架")
        self.vehicle_count.valueChanged.connect(self._set_camera_count)
        row.addWidget(self.vehicle_count)
        setup_layout.addLayout(row)

        row = QHBoxLayout()
        row.setSpacing(10)
        for text, action, variant in (
            ("▶ 启动多机环境", "start", "primary"),
            ("■ 停止本次仿真", "stop", "danger"),
            ("打开 Gazebo", "gui", ""),
        ):
            item = QPushButton(text)
            item.setMinimumHeight(34)
            item.setCursor(Qt.CursorShape.PointingHandCursor)
            if variant:
                item.setProperty("variant", variant)
            # 默认参数保存本次 action，避免点击时所有闭包都取到循环最后的值。
            item.clicked.connect(lambda checked=False, a=action: self.command(a))
            row.addWidget(item)
        row.addStretch()
        report = QPushButton("保存定位报告")
        report.clicked.connect(self.save_report)
        row.addWidget(report)
        plots = QPushButton("飞行数据曲线")
        plots.clicked.connect(self.open_flight_dashboard)
        row.addWidget(plots)
        setup_layout.addLayout(row)

        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(QLabel("ToF 安装"))
        self.tof_mount = QComboBox()
        self.tof_mount.addItem("水平三颗 ToF", "horizontal")
        self.tof_mount.addItem("下视 20°", "down20")
        row.addWidget(self.tof_mount)
        row.addWidget(QLabel("视觉处理延迟"))
        self.vision_delay = QSpinBox()
        self.vision_delay.setRange(0, 2000)
        self.vision_delay.setSuffix(" ms")
        self.vision_delay.setToolTip(
            "模拟视觉处理服务时间；单在途任务和最新等待帧，保留曝光时间。"
        )
        row.addWidget(self.vision_delay)
        row.addSpacing(12)
        self.height_only = QCheckBox("仅融合高度")
        self.height_only.setToolTip("只验证高度链路；启用后不执行飞行任务。")
        row.addWidget(self.height_only)
        self.uwb_anchors = QCheckBox("启用固定 UWB 锚点")
        row.addWidget(self.uwb_anchors)
        self.fly = QCheckBox("自动解锁并执行任务（SITL）")
        self.fly.setChecked(True)
        self.fly.setToolTip(
            "启动后自动执行所选任务；取消勾选只启动场景、传感器和定位日志。"
        )
        row.addWidget(self.fly)
        self.height_only.toggled.connect(self._height_only_changed)
        row.addStretch()
        setup_layout.addLayout(row)
        outer.addWidget(setup)

        self.status = QLabel("等待选择地图和仿真运行目录。")
        self.status.setProperty("role", "note")
        self.status.setWordWrap(True)
        outer.addWidget(self.status)
        self.map_note = QLabel("")
        self.map_note.setProperty("role", "hint")
        self.map_note.setWordWrap(True)
        outer.addWidget(self.map_note)

        middle = QHBoxLayout()
        middle.setSpacing(14)
        map_box = QGroupBox("定位地图")
        map_layout = QVBoxLayout(map_box)
        map_layout.setContentsMargins(16, 14, 16, 16)
        map_layout.setSpacing(8)
        self.map_view = LocalizationMap()
        map_tools = QHBoxLayout()
        map_tools.setSpacing(8)
        zoom = QPushButton("中心放大／全场")
        zoom.setCursor(Qt.CursorShape.PointingHandCursor)
        zoom.clicked.connect(self.map_view.toggle_center)
        map_tools.addWidget(zoom)
        map_tools.addStretch()
        map_tools.addWidget(hint("虚线为真值评估，实线为估计；红框为起点"))
        map_layout.addLayout(map_tools)
        map_layout.addWidget(self.map_view, 1)
        middle.addWidget(map_box, 3)

        # 指标表与地图并排：地图受宽高比限制，右侧留白用表格填满比堆在底部更省纵向空间。
        metrics_box = QGroupBox("定位指标")
        metrics_layout = QVBoxLayout(metrics_box)
        metrics_layout.setContentsMargins(16, 14, 16, 16)
        metrics_layout.setSpacing(8)
        self.team_line = QLabel("等待任务阶段。")
        self.team_line.setProperty("role", "sectionTitle")
        self.team_line.setWordWrap(True)
        metrics_layout.addWidget(self.team_line)
        self.table = QTableWidget(0, 9)
        # 列宽按内容自适应而不是等分：等分会在窗口变窄时把表头截断成「均方根 /」这样的残句。
        self.table.setHorizontalHeaderLabels(
            [
                "机号",
                "状态",
                "XY 均方根 / m",
                "最大误差 / m",
                "XY 有效",
                "光流有效",
                "协方差评分",
                "XY 不确定度 / m",
                "定位来源",
            ]
        )
        self.table.setToolTip(
            "XY 均方根与最大误差单位为 m；XY 有效、光流有效、协方差评分和定位来源取最新记录；协方差评分为 exp(-0.5·(Pxx+Pyy))，不是概率；XY 无效时不显示评分和不确定度。"
        )
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setAlternatingRowColors(True)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        # 最后一列吸收剩余宽度，表格右侧不会留下一条突兀的空白。
        header.setSectionResizeMode(8, QHeaderView.ResizeMode.Stretch)
        metrics_layout.addWidget(self.table, 1)
        self.cards = QPlainTextEdit()
        self.cards.setReadOnly(True)
        self.cards.setMaximumHeight(110)
        self.cards.setPlaceholderText("机间距离误差与回家点偏移会显示在这里。")
        metrics_layout.addWidget(self.cards)
        middle.addWidget(metrics_box, 2)
        outer.addLayout(middle, 1)

        camera_box = QGroupBox("机载相机")
        camera_layout = QVBoxLayout(camera_box)
        camera_layout.setContentsMargins(16, 14, 16, 16)
        self.cameras = []
        camera_grid = QGridLayout()
        camera_grid.setSpacing(8)
        self.camera_grid = camera_grid
        self.camera_area = QScrollArea()
        self.camera_area.setWidgetResizable(True)
        self.camera_area.setMinimumHeight(150)
        self.camera_area.setMaximumHeight(210)
        camera_host = QWidget()
        camera_host.setLayout(camera_grid)
        self.camera_area.setWidget(camera_host)
        camera_layout.addWidget(self.camera_area)
        outer.addWidget(camera_box)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(500)
        self.log.setMaximumHeight(76)
        self.log.setPlaceholderText("Gazebo 启动与管理脚本输出会显示在这里。")
        outer.addWidget(self.log)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(800)

    def set_theme(self, dark):
        """@brief 切换深浅主题并同步到地图与已打开的飞行窗口。
        @param dark 为真表示深色主题。
        """
        self.dark = dark
        self.map_view.set_theme(dark)
        if self.flight_dashboard is not None:
            self.flight_dashboard.set_theme(dark)
        self._set_camera_count(max(1, self.vehicle_count.value()))

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
        self.flight_dashboard.set_theme(self.dark)
        self.flight_dashboard.destroyed.connect(self._dashboard_closed)
        self.flight_dashboard.show()

    def _dashboard_closed(self):
        """@brief 对话框销毁时释放面板中的引用。"""
        self.flight_dashboard = None
        if self._closing:
            self._finish_shutdown()

    def close_flight_dashboard(self):
        """@brief 关闭附属飞行窗口，由其关闭流程结束日志线程。"""
        if self.flight_dashboard is not None:
            self.flight_dashboard.close()

    def shutdown(self):
        """@brief 请求后台读取与曲线窗口停止，返回是否已可安全释放本面板。"""
        self._closing = True
        self._refresh_pending = False
        self.timer.stop()
        if self.refresh_worker is not None:
            self.refresh_worker.requestInterruption()
        self.close_flight_dashboard()
        return (self.refresh_worker is None and self.flight_dashboard is None
                and self.process is None)

    def _finish_shutdown(self):
        """@brief 由 finished/destroyed 回调检查资源归零，再通知父窗口关闭。"""
        if (self._closing and not self._shutdown_notified and self.refresh_worker is None
                and self.flight_dashboard is None and self.process is None):
            self._shutdown_notified = True
            self.shutdown_complete.emit()
            self.close()

    def closeEvent(self, event):
        """@brief 保持后台线程及其父控件存活，直到异步收尾完成。"""
        if self.shutdown():
            event.accept()
        else:
            event.ignore()

    def _set_camera_count(self, count):
        """@brief 按机数重建相机标签，使用至多四列布局。
        @param count 要显示的相机数量；索引与 camera_i.ppm 对应。
        @note 边框颜色与地图上的轨迹颜色取自同一分类色板，便于把画面与轨迹对上号。
        """
        while self.cameras:
            camera = self.cameras.pop()
            self.camera_grid.removeWidget(camera)
            camera.deleteLater()
        # 相机在底部按最多四列排布；机数超过四架时通过滚动区查看其余画面。
        columns = max(1, min(4, count))
        colors = theme.categorical(self.dark)
        t = theme.tokens(self.dark)
        for i in range(count):
            camera = QLabel(f"无人机 {i} · 等待图像")
            camera.setMinimumSize(150, 92)
            camera.setToolTip(f"无人机 {i} · GC4653 相机")
            camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
            camera.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
            color = colors[i % len(colors)]
            camera.setStyleSheet(
                f"background:{t['surface_sunken']};color:{t['ink_muted']};"
                f"border:2px solid {color};border-radius:8px;padding:4px"
            )
            self.camera_grid.addWidget(camera, i // columns, i % columns)
            self.cameras.append(camera)

    def _load_maps(self):
        """@brief 枚举可解析的地图 JSON，将路径保存在选项数据中。"""
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
        """@brief 把用户选择的地图加入选项并选中，供下次启动使用。"""
        path, _ = QFileDialog.getOpenFileName(
            self, "选择地图 JSON", str(self.root / "simulation/maps"), "JSON (*.json)"
        )
        if path:
            if self.map_choice.findData(path) < 0:
                self.map_choice.addItem(Path(path).name, path)
            self.map_choice.setCurrentIndex(self.map_choice.findData(path))

    def _height_only_changed(self, checked):
        """@brief 高度链路验证缺少水平融合，选中时关闭并禁用自动飞行。"""
        if checked:
            self.fly.setChecked(False)
        self.fly.setEnabled(not checked)

    def command(self, action):
        """@brief 把界面选项转为 WSL 管理脚本参数并异步执行。
        @param action start、stop 或 gui。
        @note Windows 盘符映射到 /mnt/盘符；高度模式优先于飞行选项。process 非空时拒绝并发管理命令。
        """
        if self.process or self._closing:
            return
        self.command_error = ""
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
                "--vision-delay-ms",
                str(self.vision_delay.value()),
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
        process.errorOccurred.connect(lambda error: self._process_error(process, error))
        process.start("wsl.exe", args)

    def _process_error(self, process, error):
        """@brief 启动失败没有 finished 信号，需在此释放管理命令占用。"""
        if error == QProcess.ProcessError.FailedToStart and self.process is process:
            self.log.appendPlainText(f"无法启动仿真管理命令：{process.errorString()}")
            self.command_error = f"Gazebo 命令启动失败：{process.errorString()}"
            self.status.setText(self.command_error)
            self.process = None
            process.deleteLater()
            self._finish_shutdown()

    def _finished(self, process, action, code):
        """@brief 接收管理命令退出，释放 QProcess 并刷新运行状态。"""
        if self.process is not process:
            return
        action_label = {"start": "启动环境", "stop": "停止仿真", "gui": "打开窗口"}[action]
        self.log.appendPlainText(f"{action_label} · 退出码 {code}")
        if code:
            self.command_error = f"{action_label}失败，退出码 {code}；请查看下方日志。"
            self.status.setText(self.command_error)
        else:
            self.status.setText({"start": "仿真环境已就绪", "stop": "仿真已停止",
                                 "gui": "Gazebo 窗口已就绪"}[action])
        self.process = None
        process.deleteLater()
        self.refresh()
        self._finish_shutdown()

    def refresh(self):
        """@brief 请求一次后台刷新；忙时只保留一次后续请求，避免任务排队。"""
        if self._closing:
            return
        if self.refresh_worker is not None:
            self._refresh_pending = True
            return
        worker = SimulationRefresh(self.reader, self)
        self.refresh_worker = worker
        worker.loaded.connect(self._apply_refresh)
        worker.failed.connect(self._refresh_failed)
        worker.finished.connect(self._refresh_finished)
        worker.start()

    def _refresh_finished(self):
        """@brief 线程退出后释放引用；随后执行合并的刷新或关闭请求。"""
        worker = self.refresh_worker
        self.refresh_worker = None
        if worker is not None:
            worker.deleteLater()
        if self._closing:
            self._finish_shutdown()
        elif self._refresh_pending:
            self._refresh_pending = False
            self.refresh()

    def _refresh_failed(self, message):
        """@brief 在界面显示后台读取异常。"""
        if not self._closing:
            self.status.setText(self.command_error or f"无法读取仿真状态：{message}")

    def _apply_refresh(self, result):
        """@brief 绘制独立快照；此槽只访问内存，不读取日志、地形或相机文件。"""
        if self._closing:
            return
        if result is None:
            self.current_folder = None
            self.snapshot = None
            self.map_config = {}
            self.map_view.set_data(None, {})
            self.map_view.set_terrain(None)
            self.table.setRowCount(0)
            self.cards.clear()
            self.map_note.clear()
            self.team_line.setText("等待任务阶段。")
            for i, camera in enumerate(self.cameras):
                camera.setText(f"无人机 {i} · 等待图像")
            self.status.setText(self.command_error or "等待仿真运行目录。")
            return
        self.current_folder = folder = result["folder"]
        self.map_config = result["config"]
        self.snapshot = snapshot = result["snapshot"]
        value = result["manager"]
        count = len(snapshot["vehicles"])
        if count != len(self.cameras):
            self._set_camera_count(count)
        self._update_map_context(self.map_config)
        if result["terrain_error"]:
            self.map_note.setText(self.map_note.text() + " · " + result["terrain_error"])
        self.map_view.set_terrain(result["terrain"])
        self.map_view.set_data(snapshot, self.map_config)
        self._fill_table(snapshot)
        self.cards.setPlainText(self._format(snapshot))
        for i, camera in enumerate(self.cameras):
            image = result["cameras"].get(i)
            if image is None:
                camera.setText(f"无人机 {i} · 等待图像")
            else:
                camera.setPixmap(QPixmap.fromImage(image).scaled(
                    camera.size(), Qt.AspectRatioMode.KeepAspectRatio))
        map_label = self.map_config.get("label", "未知")
        if self.map_config.get("name") == "rmuc_2026_sentry26":
            map_label = "Sentry26 2026 全场地形 · 开放边界派生版"
        team = snapshot.get("team_status", {})
        phase = self._phase_text(team)
        self.team_line.setText(phase)
        fly_requested = self.map_config.get("fly_requested")
        if fly_requested is None:
            # 旧运行目录从伴随端启动参数恢复模式，不能由当前复选框推断。
            runners = [p for p in value.get("processes", [])
                       if p.get("name", "").startswith("companion_")]
            if runners:
                fly_requested = all("--arm" in p.get("command", [])
                                    and "--offboard" in p.get("command", [])
                                    for p in runners)
        mode = ("自动飞行" if fly_requested is True else
                "仅观察（自动飞行未启用）" if fly_requested is False else "未记录")
        self.status.setText(
            self.command_error or (("仿真已停止" if value.get("stopped") else "仿真运行中")
            + f" · {folder.name} · 地图：{map_label} · 模式：{mode}"
            )
        )

    def _fill_table(self, snapshot):
        """@brief 用定位快照刷新每机指标表。
        @param snapshot LocalizationMetrics 计算的指标快照。
        @note 缺失指标写占位符而不是 0，并按状态给状态列上色，使异常机一眼可见。
        """
        vehicles = snapshot.get("vehicles", [])
        self.table.setRowCount(len(vehicles))
        t = theme.tokens(self.dark)

        def number(value, digits=2):
            """@brief 格式化可缺失的浮点数。"""
            return "—" if value is None else f"{value:.{digits}f}"

        for row, vehicle in enumerate(vehicles):
            latest = vehicle.get("latest", {})
            label, tone = STATUS_LABELS.get(vehicle["status"], (vehicle["status"], ""))

            def latest_flag(value):
                """@brief 显示最新有效位；缺少记录使用占位符。"""
                return "有效" if value is True else "无效" if value is False else "—"

            xy_valid = latest.get("xy_valid")
            score = latest.get("position_confidence") if xy_valid is True else None
            uncertainty = latest.get("xy_uncertainty_m") if xy_valid is True else None
            cells = [
                f"机 {vehicle['vehicle']}",
                label,
                number(vehicle["rmse_m"]),
                number(vehicle["max_error_m"]),
                latest_flag(xy_valid),
                latest_flag(latest.get("flow_valid")),
                number(score),
                number(uncertainty),
                latest.get("localization_mode") or "—",
            ]
            for column, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if column:
                    item.setTextAlignment(
                        Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter
                    )
                if column == 1 and tone:
                    item.setForeground(QColor(t[tone]))
                self.table.setItem(row, column, item)

    def _update_map_context(self, config):
        """@brief 从快照中的地图参数生成米制尺寸与起点说明。"""
        bounds = map_bounds(config)
        extent = config.get("vehicle_extent_m", [0.082, 0.082, 0.0285])
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

    def open_guide(self):
        """@brief 打开项目中的多机任务说明。"""
        path = self.root / "simulation/docs/team-mission.html"
        if path.exists():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))
        else:
            self.status.setText("地形定位说明文件不存在。")

    @staticmethod
    def _id_list(value, limit=8):
        """@brief 把机号序列整理成去重、排序且限长的显示文本。
        @param value 任意序列；非整数项被忽略。
        @param limit 超过该数量时折叠成「前若干项 + 其余 N 架」。
        @return 顿号分隔的文本；空序列返回 "无"。
        @note 任务日志里的失联记录可能重复累积，界面必须去重后再显示，
        否则一条状态栏会被同一批机号反复刷满。
        """
        if not isinstance(value, list):
            return "无"
        ids = sorted({int(i) for i in value if type(i) is int and i >= 0})
        if not ids:
            return "无"
        if len(ids) > limit:
            head = "、".join(str(i) for i in ids[:limit])
            return f"{head} 等 {len(ids)} 架"
        return "、".join(str(i) for i in ids)

    @staticmethod
    def _phase_text(team):
        """@brief 将任务状态字典格式化为阶段、计时与机号摘要。
        @param team 任务状态，缺少字段时使用等待或空值显示。
        @return 单行界面摘要；未知阶段保留原始名称。
        """
        text = theme.phase_name(team.get("phase", ""))
        if team.get("flight_elapsed_s", 0) > 0:
            text += f" · 飞行计时 {team['flight_elapsed_s']:.1f} s"
        active = team.get("active_vehicle")
        if type(active) is int and active >= 0:
            text += f" · 当前机 {active}"
        completed = team.get("completed", [])
        if isinstance(completed, list):
            text += " · 已撞击：" + GazeboPanel._id_list(completed)
        skipped = team.get("skipped", [])
        if isinstance(skipped, list) and skipped:
            text += " · 跳过机号：" + GazeboPanel._id_list(skipped)
        if team.get("telemetry_errors"):
            text += f" · 状态写入重试 {team['telemetry_errors']} 次"
        if team.get("send_errors"):
            text += f" · 指令发送失败 {team['send_errors']} 次"
        return text

    def _format(self, snapshot):
        """@brief 整理表格之外的补充信息：通信状态、降级原因与回家点偏移。
        @param snapshot 已由 LocalizationMetrics 计算的指标。
        @return 多行文本；每机常规指标由 _fill_table 负责，此处不重复。
        @note 这里只组织显示，不重新计算误差；区分实时回家点偏移与落地/完成时保存的偏移。
        """
        rows = [
            f"范围：{'完整日志' if snapshot.get('scope', '').startswith('full') else '接入后的日志片段'}"
            f" · 对齐容差：{snapshot['alignment_tolerance_us'] / 1000:.0f} ms"
        ]
        for v in snapshot["vehicles"]:
            detail = []
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
                detail.append(
                    "队伍通信 "
                    + (
                        "更新中"
                        if state.get("fresh")
                        else "数据过期"
                        if state.get("seen")
                        else "等待连接"
                    )
                )
                if state.get("degraded"):
                    detail.append(state.get("degraded_reason", "等待恢复"))
            home = v.get("home_error")
            if home:

                def offset_text(offset):
                    """@brief 格式化米制回家点偏移，保留 dx/dy 的正负方向。"""
                    return (
                        "--"
                        if offset is None
                        else f"{offset['xy_m']:.3f} m（dx {offset['dx_m']:+.3f}，dy {offset['dy_m']:+.3f} m）"
                    )

                detail.append(
                    f"实时回家点偏移 估计 {offset_text(home['estimate'])} · 真值评估 {offset_text(home['truth'])}"
                )
                final = home.get("landing") or home.get("final")
                if final:
                    label = "落地误差" if home.get("landing") else "完成时回家点误差"
                    detail.append(
                        f"{label} · 仿真时间 {final['sim_time_us'] / 1e6:.3f} s：估计 {offset_text(final['estimate'])} · 真值评估 {offset_text(final['truth'])}"
                    )
            if detail:
                rows.append(f"机 {v['vehicle']} · " + " · ".join(detail))
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
        """@brief 将当前统计快照保存为 UTF-8 JSON，供离线分析使用。"""
        if self.snapshot is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "保存定位报告",
            str(self.root / "simulation/runtime/localization_report.json"),
            "JSON (*.json)",
        )
        if path:
            Path(path).write_text(
                json.dumps(self.snapshot, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
