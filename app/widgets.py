"""@file widgets.py
@brief 实时测距和姿态的 Qt 自绘控件。
接收端更新缓存并调用 update，Qt 随后触发 paintEvent；绘制只读取当前缓存，尺寸变化时重新计算像素坐标。
"""

from collections import deque
import statistics
import time
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QColor, QPainter, QPen, QPainterPath, QFont
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from .data import VALID_STATUS, valid_distances


class Heatmap(QWidget):
    """@brief 绘制单颗传感器的 8×8 距离网格。
    @note frame 保存原始记录；flip 只改变区域到像素的映射，maximum 为色标上限，单位 mm。
    """
    def __init__(self):
        """@brief 初始化空帧、180°显示旋转和 4000 mm 色标。
        """
        super().__init__()
        self.setMinimumSize(280, 280)
        self.frame = None
        self.flip = True
        self.maximum = 4000
        self.setMouseTracking(True)

    def zone_at(self, x, y):
        """@brief 将鼠标像素位置转换为原始区域编号。
        @param x 控件内横坐标，单位像素。
        @param y 控件内纵坐标，单位像素。
        @return 0–63 的原始索引；网格外返回 None。
        @note 旋转时用 63-zone 同时翻转行列，与绘制映射保持一致。
        """
        side = min(self.width(), self.height()) - 16
        # 先扣除网格留白，再减去居中偏移；除以单元格边长得到显示行列。
        left, top = (self.width() - side) / 2, (self.height() - side) / 2
        col, row = int((x - left) / (side / 8)), int((y - top) / (side / 8))
        if x < left or y < top or not (0 <= row < 8 and 0 <= col < 8):
            return None
        zone = row * 8 + col
        return 63 - zone if self.flip else zone

    def mouseMoveEvent(self, event):
        """@brief 从鼠标对应的原始区域读取距离和状态，更新悬停提示。
        """
        zone = self.zone_at(event.position().x(), event.position().y())
        if self.frame and zone is not None:
            self.setToolTip(
                f"区域 {zone} | 距离 {self.frame['distance_mm'][zone]} mm | 状态 {self.frame['status'][zone]}"
            )

    def paintEvent(self, event):
        """@brief 按控件短边居中绘制方形网格；无效区域使用灰色。
        @note 有效距离先除以毫米色标上限并限幅到 0–1，再映射到 HSV 色相；限幅只影响颜色。
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        side = min(self.width(), self.height()) - 16
        left, top = (self.width() - side) / 2, (self.height() - side) / 2
        cell = side / 8
        # 1. 显示行列决定绘制位置；旋转只反转原始索引，不改传感器数组。
        for row in range(8):
            for col in range(8):
                zone = row * 8 + col
                if self.flip:
                    zone = 63 - zone
                valid, value = False, 0
                if self.frame:
                    value = self.frame["distance_mm"][zone]
                    valid = self.frame["status"][zone] in VALID_STATUS and value > 0
                color = QColor("#e5eaf0")
                # 2. 距离归一化后得到连续色相；灰色单元格不参与颜色映射。
                if valid:
                    ratio = max(0, min(1, value / self.maximum))
                    color = QColor.fromHsvF(0.56 - 0.48 * ratio, 0.65, 0.94)
                rect = QRectF(
                    left + col * cell + 1, top + row * cell + 1, cell - 2, cell - 2
                )
                painter.setPen(Qt.PenStyle.NoPen)
                painter.setBrush(color)
                painter.drawRoundedRect(rect, 4, 4)
                painter.setPen(QColor("#123041") if valid else QColor("#718096"))
                painter.setFont(QFont("Segoe UI", max(7, min(11, int(cell / 4.5)))))
                painter.drawText(
                    rect,
                    Qt.AlignmentFlag.AlignCenter,
                    str(int(value)) if valid else "—",
                )


class SensorCard(QWidget):
    """@brief 汇总一颗 ToF 的热图、有效区域、中位数和接收健康状态。
    @note arrivals 保存最多 30 个电脑到达时刻；missed 统计正向序号缺口，不推断回绕或重启。
    """
    def __init__(self, number):
        """@param number 零起始传感器编号，界面名称显示为 number+1。
        """
        super().__init__()
        self.number, self.last = number, None
        self.arrivals = deque(maxlen=30)
        self.total, self.missed, self.last_sequence = 0, 0, None
        layout = QVBoxLayout(self)
        self.title = QLabel(f"VL53L5CX {number + 1}  ·  等待数据")
        self.title.setStyleSheet("font-size:17px;font-weight:600")
        self.map = Heatmap()
        self.stats = QLabel("有效区域 — / 64    中位距离 — mm")
        self.health = QLabel("帧率 — Hz    接收帧 0")
        self.stats.setWordWrap(True)
        layout.addWidget(self.title)
        layout.addWidget(self.map, 1)
        layout.addWidget(self.stats)
        layout.addWidget(self.health)

    def reset(self):
        """@brief 清空帧统计与画面，供切换数据源或重新回放使用。
        """
        self.last = None
        self.total = self.missed = 0
        self.last_sequence = None
        self.arrivals.clear()
        self.map.frame = None
        self.map.update()
        self.title.setText(f"VL53L5CX {self.number + 1}  ·  等待数据")
        self.stats.setText("有效区域 — / 64    中位距离 — mm")
        self.health.setText("帧率 — Hz    接收帧 0")

    def receive(self, frame):
        """@brief 接收一帧并更新有效区域统计及接收频率。
        @param frame 已校验的 ToF 字典，距离单位 mm。
        @note N 个接收时刻包含 N-1 个间隔，频率用 (N-1)/总时长；它反映界面接收节奏。
        """
        self.last = time.monotonic()
        self.arrivals.append(self.last)
        self.total += 1
        sequence = frame.get("sequence")
        if isinstance(sequence, int) and isinstance(self.last_sequence, int):
            if sequence > self.last_sequence + 1:
                self.missed += sequence - self.last_sequence - 1
        self.last_sequence = sequence
        self.map.frame = frame
        self.map.update()
        distances = valid_distances(frame)
        # 中位数取排序后的中心值（偶数个取中间两值的平均），较少受异常远点影响；
        # 所有区域无效时显示缺失，避免用 0 mm 冒充测量结果。
        median = f"{statistics.median(distances):.0f}" if distances else "—"
        self.stats.setText(f"有效区域 {len(distances)} / 64    中位距离 {median} mm")
        self.title.setText(
            f"VL53L5CX {self.number + 1}  ·  {frame.get('source', '未知来源')}"
        )
        hz = (
            (len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])
            if len(self.arrivals) > 1 and self.arrivals[-1] > self.arrivals[0]
            else 0
        )
        self.health.setText(
            f"接收 {hz:.1f} Hz    帧 {self.total}    序号缺口 {self.missed}"
        )

    def mark_stale(self):
        """@brief 超过两秒未收到新帧时标记过期，保留最后一帧供查看。
        """
        if self.last is not None and time.monotonic() - self.last > 2:
            self.title.setText(
                f"VL53L5CX {self.number + 1}  ·  数据已过期（保留最后一帧）"
            )


class AttitudeView(QWidget):
    """@brief 以人工地平线表现横滚和俯仰。
    @note angles 为度制 roll、pitch、yaw；此图只使用前两项，航向由外部文字显示。
    """
    def __init__(self):
        """@brief 初始化等待姿态状态和最小显示尺寸。
        """
        super().__init__()
        self.setMinimumSize(220, 200)
        self.angles = None

    def paintEvent(self, event):
        """@brief 在圆形裁剪区内旋转天空地面，再画固定机体标记。
        @note 横滚对背景取反角；俯仰每 45°移动一个圆半径并限幅，属于示意显示映射。
        """
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) * 0.4
        p.translate(w / 2, h / 2)
        path = QPainterPath()
        path.addEllipse(QPointF(0, 0), r, r)
        # 1. 把坐标原点移到仪表中心并裁剪成圆，背景可以超出仪表而不溢出。
        p.setClipPath(path)
        roll, pitch = self.angles[:2] if self.angles else (0, 0)
        p.rotate(-roll)
        # 2. 机体标记固定，反向旋转地平线表现横滚，再用上下平移表现俯仰。
        horizon = max(-r, min(r, pitch * r / 45))
        p.fillRect(QRectF(-r * 3, -r * 3, r * 6, r * 3 + horizon), QColor("#74b6d7"))
        p.fillRect(QRectF(-r * 3, horizon, r * 6, r * 3), QColor("#b28d66"))
        p.setPen(QPen(QColor("white"), 2))
        p.drawLine(QPointF(-r * 2, horizon), QPointF(r * 2, horizon))
        # 3. 抵消背景旋转后绘制机体标记，使观察者始终以机体为参照。
        p.rotate(roll)
        p.setPen(QPen(QColor("#fce883"), 3))
        p.drawLine(QPointF(-r * 0.6, 0), QPointF(-r * 0.18, 0))
        p.drawLine(QPointF(r * 0.18, 0), QPointF(r * 0.6, 0))
        p.drawEllipse(QPointF(0, 0), 4, 4)
        p.setClipping(False)
        p.setPen(QPen(QColor("#cdd9e5"), 3))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(0, 0), r, r)
        if not self.angles:
            p.setPen(QColor("#17324a"))
            p.drawText(
                QRectF(-r, r * 0.45, r * 2, 30),
                Qt.AlignmentFlag.AlignCenter,
                "等待姿态数据",
            )


class Trend(QWidget):
    """@brief 绘制两颗传感器最近 200 个有效帧的中位距离。
    @note 横轴按样本序号等距排列，两条队列独立更新，不代表设备时间同步。
    """
    def __init__(self):
        """@brief 建立两个限长队列，队列元素的单位为 mm。
        """
        super().__init__()
        self.setMinimumHeight(140)
        self.values = [deque(maxlen=200), deque(maxlen=200)]

    def paintEvent(self, event):
        """@brief 将样本序号映射到横轴，将 0–4000 mm 映射到纵轴。
        @note 绘制时裁剪超量程值，不修改队列中的原始中位距离。
        """
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#f5f8fc"))
        w, h = self.width() - 48, self.height() - 30
        p.setPen(QColor("#9aaec0"))
        p.drawText(5, 18, "4 m")
        p.drawText(5, h, "0 m")
        for i, series in enumerate(self.values):
            if len(series) < 2:
                continue
            p.setPen(QPen(QColor("#168aad" if i == 0 else "#ed8e49"), 2))
            # 固定以 199 个间隔铺满最多 200 个样本；不足 200 个时只占横轴前段，
            # 这样新增帧不会让已有样本的横向间距反复变化。
            points = [
                QPointF(42 + j * w / 199, 12 + h * (1 - max(0, min(4000, v)) / 4000))
                for j, v in enumerate(series)
            ]
            for a, b in zip(points, points[1:]):
                p.drawLine(a, b)
