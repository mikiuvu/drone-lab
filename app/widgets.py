from collections import deque
import statistics
import time
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QColor, QPainter, QPen, QPainterPath, QFont
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from .data import VALID_STATUS, valid_distances


class Heatmap(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(280, 280)
        self.frame = None
        self.flip = True
        self.maximum = 4000
        self.setMouseTracking(True)

    def zone_at(self, x, y):
        side = min(self.width(), self.height()) - 16
        left, top = (self.width() - side) / 2, (self.height() - side) / 2
        col, row = int((x - left) / (side / 8)), int((y - top) / (side / 8))
        if x < left or y < top or not (0 <= row < 8 and 0 <= col < 8):
            return None
        zone = row * 8 + col
        return 63 - zone if self.flip else zone

    def mouseMoveEvent(self, event):
        zone = self.zone_at(event.position().x(), event.position().y())
        if self.frame and zone is not None:
            self.setToolTip(
                f"区域 {zone} | 距离 {self.frame['distance_mm'][zone]} mm | 状态 {self.frame['status'][zone]}"
            )

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        side = min(self.width(), self.height()) - 16
        left, top = (self.width() - side) / 2, (self.height() - side) / 2
        cell = side / 8
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
    def __init__(self, number):
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
        if self.last is not None and time.monotonic() - self.last > 2:
            self.title.setText(
                f"VL53L5CX {self.number + 1}  ·  数据已过期（保留最后一帧）"
            )


class AttitudeView(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(220, 200)
        self.angles = None

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) * 0.4
        p.translate(w / 2, h / 2)
        path = QPainterPath()
        path.addEllipse(QPointF(0, 0), r, r)
        p.setClipPath(path)
        roll, pitch = self.angles[:2] if self.angles else (0, 0)
        p.rotate(-roll)
        horizon = max(-r, min(r, pitch * r / 45))
        p.fillRect(QRectF(-r * 3, -r * 3, r * 6, r * 3 + horizon), QColor("#74b6d7"))
        p.fillRect(QRectF(-r * 3, horizon, r * 6, r * 3), QColor("#b28d66"))
        p.setPen(QPen(QColor("white"), 2))
        p.drawLine(QPointF(-r * 2, horizon), QPointF(r * 2, horizon))
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
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(140)
        self.values = [deque(maxlen=200), deque(maxlen=200)]

    def paintEvent(self, event):
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
            points = [
                QPointF(42 + j * w / 199, 12 + h * (1 - max(0, min(4000, v)) / 4000))
                for j, v in enumerate(series)
            ]
            for a, b in zip(points, points[1:]):
                p.drawLine(a, b)
