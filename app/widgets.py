"""@file widgets.py
@brief 实时测距、趋势与姿态的 Qt 自绘控件。
接收端更新缓存并调用 update，Qt 随后触发 paintEvent；绘制只读取当前缓存，尺寸变化时重新计算像素坐标。
所有颜色取自 app.theme，深浅主题通过 set_theme 统一切换。
"""

from collections import deque
import math
import statistics
import time

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from . import theme
from .data import VALID_STATUS, valid_distances

FONT_SMALL = 11
FONT_TINY = 9


class StatTile(QWidget):
    """@brief 单个关键指标的卡片：标题、数值、单位与可选说明。
    @note 数值为空时显示占位符而不是 0，避免把「没有数据」误读成「测量值为零」。
    """

    def __init__(self, title, unit="", placeholder="—"):
        """@param title 指标名称，显示在数值上方。
        @param unit 单位，显示在数值右侧，可为空。
        @param placeholder 无数值时显示的占位文本。
        """
        super().__init__()
        self.setObjectName("statTile")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self._placeholder = placeholder
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 9, 12, 9)
        layout.setSpacing(1)
        self.title = QLabel(title)
        self.title.setProperty("role", "hint")
        row = QHBoxLayout()
        row.setSpacing(4)
        self.value = QLabel(placeholder)
        self.value.setProperty("role", "metric")
        self.unit = QLabel(unit)
        self.unit.setProperty("role", "metricUnit")
        row.addWidget(self.value)
        row.addWidget(self.unit, 0, Qt.AlignmentFlag.AlignBottom)
        row.addStretch()
        layout.addWidget(self.title)
        layout.addLayout(row)
        self._tone = ""

    def set_value(self, text, unit=None, tone=""):
        """@brief 更新数值文本与语义色调。
        @param text 已格式化的数值文本；None 或空串回退到占位符。
        @param unit 新的单位文本；None 表示保持原单位。
        @param tone 语义色调，取 ok/warn/danger/空字符串。
        """
        self.value.setText(text if text not in (None, "") else self._placeholder)
        if unit is not None:
            self.unit.setText(unit)
        if tone != self._tone:
            self._tone = tone
            self.value.setProperty("tone", tone)
            self.value.style().unpolish(self.value)
            self.value.style().polish(self.value)


class ColorScaleBar(QWidget):
    """@brief 距离热图的色标图例：连续渐变条加刻度，并标出当前帧的取值范围。
    @note 自动模式下刻度跟随本帧最小/最大距离，固定模式下显示 0 到上限。
    """

    def __init__(self, parent=None):
        """@param parent 所属热图控件；必须传入，否则渐变条会成为独立顶层窗口。"""
        super().__init__(parent)
        self.setFixedHeight(30)
        self.stops = list(theme.DISTANCE_STOPS)
        self.low = 0.0
        self.high = 4000.0
        self.auto = True
        self.dark = False
        self.unit = "mm"

    def configure(self, low, high, auto, dark):
        """@brief 更新色标范围和主题。
        @param low 色标下端对应的距离，单位 mm。
        @param high 色标上端对应的距离，单位 mm。
        @param auto 为真表示范围来自当前帧。
        @param dark 为真表示深色主题。
        @note 没有帧时由 Heatmap 直接隐藏本控件，因此这里不处理空数据状态。
        """
        self.low, self.high, self.auto, self.dark = float(low), float(high), auto, dark
        self.update()

    def paintEvent(self, event):
        """@brief 绘制渐变条、刻度与范围说明。
        @note 渐变用 QLinearGradient 一次填充，而不是逐像素画竖线：色标每帧都要重绘，
        逐像素版本在 10 Hz 刷新下会产生数百次绘制调用。
        """
        from PySide6.QtGui import QLinearGradient

        t = theme.tokens(self.dark)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        margin = 34
        bar = QRectF(margin, 6, max(10, self.width() - 2 * margin), 9)
        gradient = QLinearGradient(bar.left(), 0, bar.right(), 0)
        for i, stop in enumerate(self.stops):
            gradient.setColorAt(i / (len(self.stops) - 1), QColor(stop))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(gradient)
        p.drawRoundedRect(bar, 3, 3)
        p.setPen(QPen(QColor(t["border_strong"]), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(bar, 3, 3)
        p.setFont(QFont("Microsoft YaHei UI", FONT_TINY))
        p.setPen(QColor(t["ink_muted"]))
        caption = "近" if self.auto else f"{self.low:.0f}"
        p.drawText(
            QRectF(0, 3, margin - 6, 14),
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            caption,
        )
        p.drawText(
            QRectF(bar.right() + 6, 3, margin, 14),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            "远" if self.auto else f"{self.high:.0f}",
        )
        note = (
            f"自动量程 {self.low:.0f}–{self.high:.0f} {self.unit}（本帧范围）"
            if self.auto
            else f"固定量程 0–{self.high:.0f} {self.unit}"
        )
        p.drawText(
            QRectF(margin, 16, self.width() - 2 * margin, 13),
            Qt.AlignmentFlag.AlignCenter,
            note,
        )


class Heatmap(QWidget):
    """@brief 绘制单颗传感器的 8×8 距离网格，并附色标图例与区域编号。
    @note frame 保存原始记录；flip 只改变区域到像素的映射，maximum 为固定色标上限，单位 mm。
    """

    def __init__(self):
        """@brief 初始化空帧、180°显示旋转和 4000 mm 固定色标。"""
        super().__init__()
        self.setMinimumSize(300, 330)
        self.setMouseTracking(True)
        self.frame = None
        self.flip = True
        self.maximum = 4000
        self.auto_range = True
        self.show_values = True
        self.dark = False
        self._hover = None
        self.scale = ColorScaleBar(self)
        self._low, self._high = 0.0, float(self.maximum)

    def set_theme(self, dark):
        """@brief 切换深浅主题并重绘。
        @param dark 为真表示深色主题。
        """
        self.dark = dark
        self.scale.dark = dark
        self.update()

    def configure(self, flip=None, maximum=None, auto_range=None, show_values=None):
        """@brief 批量更新显示选项，只重绘一次。
        @param flip 是否旋转 180°；None 表示保持。
        @param maximum 固定色标上限，单位 mm；None 表示保持。
        @param auto_range 是否按本帧范围自动量程；None 表示保持。
        @param show_values 是否在单元格内显示距离数值；None 表示保持。
        """
        if flip is not None:
            self.flip = bool(flip)
        if maximum is not None:
            self.maximum = float(maximum)
        if auto_range is not None:
            self.auto_range = bool(auto_range)
        if show_values is not None:
            self.show_values = bool(show_values)
        self.update()

    def grid_rect(self):
        """@return 网格占用的方形 QRectF；顶部与左侧留出行列标号，底部留出色标。
        @note 标号占用的边距在计算正方形边长前扣除，避免小尺寸控件下标号压住单元格。
        """
        band, gutter, header = 34, 30, 22
        side = max(
            40,
            min(self.width() - gutter - 12, self.height() - band - header - 12),
        )
        left = gutter + (self.width() - gutter - 12 - side) / 2
        return QRectF(left, header, side, side)

    def zone_at(self, x, y):
        """@brief 将鼠标像素位置转换为原始区域编号。
        @param x 控件内横坐标，单位像素。
        @param y 控件内纵坐标，单位像素。
        @return 0–63 的原始索引；网格外返回 None。
        @note 旋转时用 63-zone 同时翻转行列，与绘制映射保持一致。
        """
        rect = self.grid_rect()
        if not rect.contains(QPointF(x, y)):
            return None
        col = int((x - rect.left()) / (rect.width() / 8))
        row = int((y - rect.top()) / (rect.height() / 8))
        if not (0 <= row < 8 and 0 <= col < 8):
            return None
        zone = row * 8 + col
        return 63 - zone if self.flip else zone

    def value_range(self):
        """@return (low, high) 当前色标范围，单位 mm；自动模式下取本帧有效距离的极值。"""
        if not self.auto_range or not self.frame:
            return 0.0, float(self.maximum)
        values = valid_distances(self.frame)
        if not values:
            return 0.0, float(self.maximum)
        low, high = min(values), max(values)
        if high - low < 1.0:
            # 单色帧（例如全部测到同一距离）会给零宽量程，人为展开 20 mm 让颜色仍有层次。
            low, high = low - 10.0, high + 10.0
        return float(low), float(high)

    def color_for(self, value):
        """@brief 把距离映射为色标上的颜色。
        @param value 距离，单位 mm。
        @return (r, g, b) 三元组；超出量程时截断到端点。
        """
        low, high = self._low, self._high
        ratio = 0.0 if high <= low else (value - low) / (high - low)
        return theme.lerp_color(theme.DISTANCE_STOPS, max(0.0, min(1.0, ratio)))

    def mouseMoveEvent(self, event):
        """@brief 从鼠标对应的原始区域读取距离和状态，更新悬停提示并高亮。
        """
        zone = self.zone_at(event.position().x(), event.position().y())
        if zone != self._hover:
            self._hover = zone
            self.update()
        if self.frame and zone is not None:
            distance = self.frame["distance_mm"][zone]
            status = self.frame["status"][zone]
            ok = status in VALID_STATUS and distance > 0
            row, col = divmod(zone, 8)
            self.setToolTip(
                f"区域 {zone}（第 {row + 1} 行 · 第 {col + 1} 列）\n"
                f"距离 {distance} mm\n"
                f"状态码 {status} · {'有效' if ok else '无效'}"
            )

    def leaveEvent(self, event):
        """@brief 鼠标离开时清除高亮单元格。
        """
        if self._hover is not None:
            self._hover = None
            self.update()

    def paintEvent(self, event):
        """@brief 绘制网格、单元格数值、行列表头与色标。
        @note 文字颜色按底色亮度自动选择，深色单元格上用浅字，避免出现读不清的数值。
        """
        t = theme.tokens(self.dark)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.grid_rect()
        cell = rect.width() / 8
        self._low, self._high = self.value_range()
        self.scale.configure(self._low, self._high, self.auto_range, self.dark)
        self.scale.setGeometry(0, self.height() - 32, self.width(), 30)
        # 没有数据时隐藏色标：否则会显示一条「自动量程 0–4000 mm（本帧范围）」的空图例，
        # 让人以为界面已经拿到过测量值。
        self.scale.setVisible(self.frame is not None)

        if self.frame is None:
            p.setPen(QColor(t["ink_muted"]))
            p.setFont(QFont("Microsoft YaHei UI", 12))
            p.drawText(rect, Qt.AlignmentFlag.AlignCenter, "等待测距数据")
            p.setPen(QPen(QColor(t["border"]), 1, Qt.PenStyle.DashLine))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRoundedRect(rect, theme.RADIUS["md"], theme.RADIUS["md"])
            return

        # 1. 行列标号写传感器原始区域坐标：旋转只改变显示顺序，标号始终指向原始索引，
        #    使界面上的「区域 N」可以直接与模块数据手册的 8×8 排布对应。
        p.setFont(QFont("Microsoft YaHei UI", FONT_TINY))
        p.setPen(QColor(t["ink_muted"]))
        for i in range(8):
            original = 7 - i if self.flip else i
            p.drawText(
                QRectF(rect.left() - 26, rect.top() + i * cell, 20, cell),
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                str(original),
            )
            p.drawText(
                QRectF(rect.left() + i * cell, rect.top() - 19, cell, 16),
                Qt.AlignmentFlag.AlignCenter,
                str(original),
            )

        font = QFont("Microsoft YaHei UI")
        font.setPixelSize(max(8, min(12, int(cell / 4.2))))
        for row in range(8):
            for col in range(8):
                zone = row * 8 + col
                if self.flip:
                    zone = 63 - zone
                value = self.frame["distance_mm"][zone]
                valid = self.frame["status"][zone] in VALID_STATUS and value > 0
                cell_rect = QRectF(
                    rect.left() + col * cell + 1,
                    rect.top() + row * cell + 1,
                    cell - 2,
                    cell - 2,
                )
                if valid:
                    rgb = self.color_for(value)
                    brush = QColor(int(rgb[0]), int(rgb[1]), int(rgb[2]))
                    text_color = QColor(theme.readable_on(rgb))
                else:
                    brush = QColor(theme.INVALID_CELL["dark" if self.dark else "light"])
                    text_color = QColor(theme.INVALID_TEXT["dark" if self.dark else "light"])
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(brush)
                p.drawRoundedRect(cell_rect, 4, 4)
                # 2. 悬停单元格加一圈描边，比放大或变色更不干扰整体图案。
                if zone == self._hover:
                    p.setPen(QPen(QColor(t["ink"]), 2))
                    p.setBrush(Qt.BrushStyle.NoBrush)
                    p.drawRoundedRect(cell_rect.adjusted(1, 1, -1, -1), 4, 4)
                if self.show_values and cell >= 26:
                    p.setPen(text_color)
                    p.setFont(font)
                    p.drawText(
                        cell_rect,
                        Qt.AlignmentFlag.AlignCenter,
                        str(int(value)) if valid else "—",
                    )
        p.setPen(QPen(QColor(t["border"]), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(rect, theme.RADIUS["sm"], theme.RADIUS["sm"])


class SensorCard(QWidget):
    """@brief 汇总一颗 ToF 的标题、热图与四项运行指标。
    @note 帧率统计使用两秒窗口；演示与回放使用记录时间，实测使用电脑到达时间。
    """

    def __init__(self, number):
        """@param number 零起始传感器编号，界面名称显示为 number+1。
        """
        super().__init__()
        self.number, self.last = number, None
        self.arrivals = deque(maxlen=256)
        self.rate_start = None
        self.rate_clock = None
        self.total, self.missed, self.last_sequence = 0, 0, None
        self.dark = False
        self.setObjectName("sensorCard")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        header = QHBoxLayout()
        header.setSpacing(8)
        self.title = QLabel(f"传感器 {number + 1}")
        self.title.setProperty("role", "sectionTitle")
        self.source = QLabel("等待数据")
        self.source.setProperty("role", "hint")
        header.addWidget(self.title)
        header.addWidget(self.source)
        header.addStretch()
        layout.addLayout(header)
        self.map = Heatmap()
        layout.addWidget(self.map, 1)
        self.tiles = {
            "valid": StatTile("有效区域", "/ 64"),
            "median": StatTile("中位距离", "mm"),
            "rate": StatTile("接收帧率", "Hz"),
            "gap": StatTile("序号缺口", "帧"),
        }
        row = QHBoxLayout()
        row.setSpacing(8)
        for tile in self.tiles.values():
            row.addWidget(tile, 1)
        layout.addLayout(row)

    def set_theme(self, dark):
        """@brief 切换深浅主题并同步到热图。
        @param dark 为真表示深色主题。
        """
        self.dark = dark
        self.map.set_theme(dark)

    def reset(self):
        """@brief 清空帧统计与画面，供切换数据源或重新回放使用。
        """
        self.last = None
        self.total = self.missed = 0
        self.last_sequence = None
        self.arrivals.clear()
        self.rate_start = None
        self.rate_clock = None
        self.map.frame = None
        self.map.update()
        self.source.setText("等待数据")
        self.source.setProperty("tone", "muted")
        self._repolish(self.source)
        self.tiles["valid"].set_value(None)
        self.tiles["median"].set_value(None)
        self.tiles["rate"].set_value(None)
        self.tiles["gap"].set_value(None)

    @staticmethod
    def _repolish(widget):
        """@brief 属性变更后强制重新应用样式表。
        @param widget 需要刷新的控件。
        """
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def receive(self, frame, historical=False):
        """@brief 接收一帧并更新有效区域统计及接收频率。
        @param frame 已校验的 ToF 字典，距离单位 mm。
        @param historical 演示或回放为真，以记录时间计算数据帧率，不受播放倍率影响。
        """
        self.last = time.monotonic()
        self.total += 1
        sequence = frame.get("sequence")
        if isinstance(sequence, int) and isinstance(self.last_sequence, int):
            if sequence > self.last_sequence + 1:
                self.missed += sequence - self.last_sequence - 1
        self.last_sequence = sequence
        self.map.frame = frame
        self.map.update()
        distances = valid_distances(frame)
        # 1. 中位数取排序后的中心值（偶数个取中间两值平均），较少受异常远点影响；
        #    所有区域无效时显示占位符，避免用 0 mm 冒充测量结果。
        self.tiles["valid"].set_value(
            str(len(distances)),
            tone="danger" if not distances else ("warn" if len(distances) < 32 else "ok"),
        )
        self.tiles["median"].set_value(
            f"{statistics.median(distances):.0f}" if distances else None
        )
        self.source.setText(frame.get("source", "未知来源"))
        self.source.setProperty("tone", "")
        self._repolish(self.source)
        self._update_rate(frame, historical)
        self.tiles["gap"].set_value(
            str(self.missed), tone="warn" if self.missed else "ok"
        )

    def _update_rate(self, frame, historical):
        """@brief 按时间窗口估计帧率；至少积累一秒，避免把同批到达间隔外推为 Hz。
        @note 时钟回退、来源变化或时间缺失时重新积累；序号缺口单独统计。
        """
        stamp = self.last
        basis = "host_monotonic"
        if historical:
            stamp = frame.get("source_time_us")
            basis = frame.get("time_basis", "source")
            if type(stamp) in (int, float) and math.isfinite(stamp) and stamp >= 0:
                stamp /= 1e6
            else:
                stamp = frame.get("record_time_s")
                basis = "recording"
        self.tiles["rate"].title.setText("数据帧率" if historical else "接收帧率")
        self.tiles["rate"].setToolTip(
            "按记录时间计算，不受回放倍率影响；至少一秒数据后显示。" if historical else
            "最近两秒电脑收到的帧数；至少接收一秒后显示。")
        clock = (historical, frame.get("source"), basis)
        valid = type(stamp) in (int, float) and math.isfinite(stamp) and stamp >= 0
        if (not valid or clock != self.rate_clock or
                (self.arrivals and stamp < self.arrivals[-1])):
            self.arrivals.clear()
            self.rate_start = None
            self.rate_clock = clock
        if not valid:
            self.tiles["rate"].set_value(None)
            return
        if self.rate_start is None:
            self.rate_start = stamp
        if not historical or not self.arrivals or stamp > self.arrivals[-1]:
            self.arrivals.append(stamp)
        start = max(self.rate_start, stamp - 2.0)
        while self.arrivals and self.arrivals[0] < start:
            self.arrivals.popleft()
        elapsed = stamp - start
        # 窗口起点上的第一帧是间隔基准，不计入到达数量。
        count = len(self.arrivals) - int(bool(self.arrivals) and self.arrivals[0] == start)
        self.tiles["rate"].set_value(f"{count / elapsed:.1f}" if elapsed >= 1 else None)

    def mark_stale(self):
        """@brief 超过两秒未收到新帧时标记过期，保留最后一帧供查看。
        """
        if self.last is not None and time.monotonic() - self.last > 2:
            self.source.setText("数据已过期 · 保留最后一帧")
            self.source.setProperty("tone", "warn")
            self._repolish(self.source)
            self.tiles["rate"].set_value(None)


class AttitudeView(QWidget):
    """@brief 以人工地平线表现横滚和俯仰，并标注俯仰刻度。
    @note angles 为度制 roll、pitch、yaw；此图只使用前两项，航向由外部文字显示。
    """

    def __init__(self):
        """@brief 初始化等待姿态状态和最小显示尺寸。
        """
        super().__init__()
        self.setMinimumSize(230, 210)
        self.angles = None
        self.dark = False

    def set_theme(self, dark):
        """@brief 切换深浅主题并重绘。
        @param dark 为真表示深色主题。
        """
        self.dark = dark
        self.update()

    def paintEvent(self, event):
        """@brief 在圆形裁剪区内旋转天空地面，再画固定机体标记与俯仰刻度。
        @note 横滚对背景取反角；俯仰每 45°移动一个圆半径并限幅，属于示意显示映射。
        """
        t = theme.tokens(self.dark)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) * 0.40
        p.translate(w / 2, h / 2)
        path = QPainterPath()
        path.addEllipse(QPointF(0, 0), r, r)
        # 1. 把坐标原点移到仪表中心并裁剪成圆，背景可以超出仪表而不溢出。
        p.setClipPath(path)
        roll, pitch = self.angles[:2] if self.angles else (0, 0)
        p.rotate(-roll)
        # 2. 机体标记固定，反向旋转地平线表现横滚，再用上下平移表现俯仰。
        sky = QColor("#2f7fb8") if not self.dark else QColor("#20496b")
        ground = QColor("#9a7346") if not self.dark else QColor("#5c4529")
        horizon = max(-r, min(r, pitch * r / 45))
        p.fillRect(QRectF(-r * 3, -r * 3, r * 6, r * 3 + horizon), sky)
        p.fillRect(QRectF(-r * 3, horizon, r * 6, r * 3), ground)
        # 3. 每 10° 画一条俯仰刻度线，20 的倍数加长并在左侧标注角度。
        for step in range(-40, 50, 10):
            offset = horizon + step * r / 45
            if abs(offset) > r - 6:
                continue
            long = step % 20 == 0
            half = r * (0.55 if long else 0.30)
            p.setPen(QPen(QColor(255, 255, 255, 165), 1.4 if long else 1))
            p.drawLine(QPointF(-half, offset), QPointF(half, offset))
            if long and step != 0:
                p.setFont(QFont("Microsoft YaHei UI", FONT_TINY))
                p.setPen(QColor(255, 255, 255, 205))
                p.drawText(
                    QRectF(-r * 0.95, offset - 8, r * 0.36, 16),
                    Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                    f"{abs(step)}",
                )
        p.setPen(QPen(QColor(255, 255, 255), 2))
        p.drawLine(QPointF(-r * 2, horizon), QPointF(r * 2, horizon))
        # 4. 抵消背景旋转后绘制机体标记，使观察者始终以机体为参照。
        p.rotate(roll)
        p.setPen(QPen(QColor("#f2c14e"), 3))
        p.drawLine(QPointF(-r * 0.62, 0), QPointF(-r * 0.20, 0))
        p.drawLine(QPointF(r * 0.20, 0), QPointF(r * 0.62, 0))
        p.drawEllipse(QPointF(0, 0), 4, 4)
        p.setClipping(False)
        # 5. 顶部横滚指针指示当前倾角，刻度每 30° 一格。
        p.setPen(QPen(QColor(t["ink_muted"]), 1.4))
        for angle in range(-60, 61, 30):
            p.save()
            p.rotate(angle)
            p.drawLine(QPointF(0, -r - 4), QPointF(0, -r + (8 if angle == 0 else 4)))
            p.restore()
        p.setPen(QPen(QColor(t["border_strong"]), 3))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(0, 0), r, r)
        if not self.angles:
            p.setPen(QColor(t["ink_muted"]))
            p.setFont(QFont("Microsoft YaHei UI", 11))
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
        self.setMinimumHeight(190)
        self.setMouseTracking(True)
        self.values = [deque(maxlen=200), deque(maxlen=200)]
        self.dark = False
        self._hover = None

    def set_theme(self, dark):
        """@brief 切换深浅主题并重绘。
        @param dark 为真表示深色主题。
        """
        self.dark = dark
        self.update()

    def reset(self):
        """@brief 清空两条曲线。"""
        for series in self.values:
            series.clear()
        self._hover = None
        self.update()

    def plot_rect(self):
        """@return 曲线绘制区，为左侧刻度、底部横轴与顶部图例留出边距。"""
        return QRectF(52, 30, max(20, self.width() - 68), max(20, self.height() - 62))

    def mouseMoveEvent(self, event):
        """@brief 记录悬停位置，用于显示该样本处的读数。
        """
        rect = self.plot_rect()
        count = max(len(s) for s in self.values)
        index = None
        if rect.contains(event.position()) and count > 1:
            ratio = (event.position().x() - rect.left()) / rect.width()
            index = int(round(ratio * (count - 1)))
            index = max(0, min(count - 1, index))
        if index != self._hover:
            self._hover = index
            self.update()

    def leaveEvent(self, event):
        """@brief 鼠标离开时清除悬停读数。
        """
        if self._hover is not None:
            self._hover = None
            self.update()

    def _range(self):
        """@return (low, high) 纵轴范围，单位 mm；无数据时给出默认 0–4000。"""
        values = [v for series in self.values for v in series]
        if not values:
            return 0.0, 4000.0
        low, high = min(values), max(values)
        if high - low < 50:
            mid = (low + high) / 2
            low, high = mid - 25, mid + 25
        pad = (high - low) * 0.12
        return max(0.0, low - pad), high + pad

    def paintEvent(self, event):
        """@brief 绘制坐标轴、网格、两条曲线与图例。
        @note 纵轴按当前数据自动量程，使 1.5–1.9 m 的细微变化也能看出形状。
        """
        t = theme.tokens(self.dark)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(t["surface"]))
        rect = self.plot_rect()
        low, high = self._range()
        p.setFont(QFont("Microsoft YaHei UI", FONT_TINY))

        # 1. 水平网格与纵轴刻度：5 条等分线，标签以米为单位以便与现场直觉一致。
        for i in range(5):
            ratio = i / 4
            y = rect.bottom() - ratio * rect.height()
            p.setPen(QPen(QColor(t["grid"]), 1))
            p.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            p.setPen(QColor(t["ink_muted"]))
            value = low + ratio * (high - low)
            p.drawText(
                QRectF(4, y - 8, 44, 16),
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                f"{value / 1000:.2f} m",
            )
        p.setPen(QPen(QColor(t["border_strong"]), 1))
        p.drawLine(rect.bottomLeft(), rect.bottomRight())
        p.drawLine(rect.topLeft(), rect.bottomLeft())

        count = max(len(s) for s in self.values)
        if count < 2:
            p.setPen(QColor(t["ink_muted"]))
            p.setFont(QFont("Microsoft YaHei UI", 11))
            p.drawText(rect, Qt.AlignmentFlag.AlignCenter, "等待足够的有效帧形成趋势")
        else:
            # 2. 固定以 199 个间隔铺满最多 200 个样本；不足 200 个时只占横轴前段，
            #    这样新增帧不会让已有样本的横向间距反复变化。
            for i, series in enumerate(self.values):
                if len(series) < 2:
                    continue
                color = QColor(theme.sensor_color(i, self.dark))
                points = [
                    QPointF(
                        rect.left() + j * rect.width() / 199,
                        rect.bottom()
                        - (max(low, min(high, v)) - low) / max(1e-6, high - low) * rect.height(),
                    )
                    for j, v in enumerate(series)
                ]
                path = QPainterPath(points[0])
                for point in points[1:]:
                    path.lineTo(point)
                p.setPen(QPen(color, 2))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawPath(path)
                last = points[-1]
                p.setBrush(color)
                p.setPen(Qt.PenStyle.NoPen)
                p.drawEllipse(last, 3.2, 3.2)
            p.setPen(QColor(t["ink_muted"]))
            p.drawText(
                QRectF(rect.left(), rect.bottom() + 4, rect.width(), 16),
                Qt.AlignmentFlag.AlignCenter,
                f"最近 {count} 个有效帧（越靠右越新）",
            )
            # 3. 悬停时画竖直参考线并标出该样本两路的读数。
            if self._hover is not None and 0 <= self._hover < count:
                x = rect.left() + self._hover * rect.width() / 199
                p.setPen(QPen(QColor(t["ink_muted"]), 1, Qt.PenStyle.DashLine))
                p.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
                parts = []
                for i, series in enumerate(self.values):
                    if self._hover < len(series):
                        parts.append(f"{theme.SENSOR_NAMES[i]} {series[self._hover]:.0f} mm")
                if parts:
                    p.setPen(QColor(t["ink"]))
                    p.setFont(QFont("Microsoft YaHei UI", FONT_SMALL))
                    text = " · ".join(parts)
                    width = p.fontMetrics().horizontalAdvance(text) + 16
                    box = QRectF(
                        min(max(rect.left(), x - width / 2), rect.right() - width),
                        rect.top() + 4,
                        width,
                        22,
                    )
                    p.setBrush(QColor(t["surface_alt"]))
                    p.setPen(QPen(QColor(t["border"]), 1))
                    p.drawRoundedRect(box, 5, 5)
                    p.setPen(QColor(t["ink"]))
                    p.drawText(box, Qt.AlignmentFlag.AlignCenter, text)

        # 4. 图例固定在绘图区上方，与曲线颜色一一对应。
        x = rect.left()
        p.setFont(QFont("Microsoft YaHei UI", FONT_SMALL))
        for i in range(2):
            color = QColor(theme.sensor_color(i, self.dark))
            p.setPen(QPen(color, 3))
            p.drawLine(QPointF(x, 16), QPointF(x + 18, 16))
            p.setPen(QColor(t["ink_soft"]))
            label = theme.SENSOR_NAMES[i]
            p.drawText(QPointF(x + 24, 20), label)
            x += 30 + p.fontMetrics().horizontalAdvance(label)
