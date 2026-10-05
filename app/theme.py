"""@file theme.py
@brief Drone Lab 的全局视觉令牌、Qt 样式表与 Matplotlib 主题。
界面与图表共用同一组颜色和间距，避免各面板自行硬编码色值导致风格漂移。
用法：启动时调用 apply_theme(app, dark)，绘图前用 mpl_context(dark) 包裹。
"""

from __future__ import annotations

from contextlib import contextmanager

# 1. 设计令牌：颜色、圆角、间距、字号。浅色与深色两套键名完全一致，
#    面板只引用键名，不关心当前是哪种主题。
LIGHT = {
    "bg": "#eef1f6",
    "surface": "#ffffff",
    "surface_alt": "#f5f8fb",
    "surface_sunken": "#e7ecf3",
    "border": "#dbe3ec",
    "border_strong": "#c2cddb",
    "ink": "#16212e",
    "ink_soft": "#4f6076",
    "ink_muted": "#8595a7",
    "ink_on_accent": "#ffffff",
    "accent": "#1668c4",
    "accent_hover": "#1b7ae0",
    "accent_press": "#11529b",
    "accent_soft": "#e2edfb",
    "accent_border": "#b6d2f2",
    "ok": "#1a8f61",
    "ok_soft": "#e0f3ea",
    "warn": "#b5730d",
    "warn_soft": "#fbf0d9",
    "danger": "#c7364a",
    "danger_soft": "#fbe4e7",
    "grid": "#e6ebf1",
    "grid_strong": "#cfd8e3",
}

DARK = {
    "bg": "#11161d",
    "surface": "#1a212b",
    "surface_alt": "#202936",
    "surface_sunken": "#151b23",
    "border": "#2c3745",
    "border_strong": "#3b4859",
    "ink": "#e7edf5",
    "ink_soft": "#a9b8c9",
    "ink_muted": "#74869a",
    "ink_on_accent": "#ffffff",
    "accent": "#4a9bf0",
    "accent_hover": "#6cb0f5",
    "accent_press": "#3684d8",
    "accent_soft": "#1d2b3d",
    "accent_border": "#2f4d72",
    "ok": "#4cc38a",
    "ok_soft": "#16301f",
    "warn": "#e0a53c",
    "warn_soft": "#33270f",
    "danger": "#f0736f",
    "danger_soft": "#3a1c1f",
    "grid": "#252f3c",
    "grid_strong": "#36434f",
}

# 2. 两套主题共用的几何与排版常量，单位 px。
RADIUS = {"sm": 6, "md": 9, "lg": 14}
SPACE = {"xs": 4, "sm": 8, "md": 14, "lg": 20, "xl": 28}
FONT_STACK = "Microsoft YaHei UI, Microsoft YaHei, Segoe UI, sans-serif"
MONO_STACK = "Cascadia Mono, Consolas, JetBrains Mono, monospace"

# 3. 分类色板：多机曲线按机号取色，机号同时显示在标签和图例中。
CATEGORICAL = [
    "#1668c4",
    "#d9542b",
    "#1a8f61",
    "#8c4fd1",
    "#c08a00",
    "#0f8f9e",
    "#c2418a",
    "#5a6b8c",
]
CATEGORICAL_DARK = [
    "#5aa2f0",
    "#f07a4e",
    "#4cc38a",
    "#b183e8",
    "#e0b03c",
    "#3fb8c4",
    "#e57bb0",
    "#93a6c4",
]

# 4. 距离热图使用「近亮远暗」的连续色标，控制点取自 viridis 并反转，
#    使同一亮度序列在灰度打印和色觉异常下仍保持单调。
DISTANCE_STOPS = ["#fde725", "#5ec962", "#21918c", "#3b528b", "#440154"]
# 高度地形使用「低暗高亮」的连续色标，方向与距离相反。
HEIGHT_STOPS = ["#2b3a4a", "#3d6b7d", "#4f9c8c", "#9dc36a", "#e8d98a"]

INVALID_CELL = {"light": "#dfe5ed", "dark": "#2a333f"}
INVALID_TEXT = {"light": "#93a1b2", "dark": "#5f6d7e"}


def lerp_color(stops, t):
    """@brief 在一组十六进制颜色控制点之间做线性插值。
    @param stops 至少两个 "#rrggbb" 字符串，按 0→1 排列。
    @param t 位置参数，超出 0–1 时截断到端点。
    @return (r, g, b) 三元组，分量为 0–255 的浮点数。
    """
    if t <= 0:
        stops = stops[:2]
        t = 0.0
    elif t >= 1:
        stops = stops[-2:]
        t = 1.0
    span = (len(stops) - 1) * t
    index = min(int(span), len(stops) - 2)
    local = span - index
    low, high = stops[index], stops[index + 1]
    return tuple(
        int(low[i : i + 2], 16) * (1 - local) + int(high[i : i + 2], 16) * local
        for i in (1, 3, 5)
    )


def hex_of(rgb):
    """@brief 把 0–255 的浮点三元组转回 "#rrggbb"。
    @param rgb (r, g, b) 三元组。
    @return 小写十六进制颜色字符串。
    """
    return "#%02x%02x%02x" % tuple(max(0, min(255, int(round(v)))) for v in rgb)


def relative_luminance(rgb):
    """@brief 按 WCAG 公式计算相对亮度，用于选择可读的前景色。
    @param rgb (r, g, b) 三元组，分量 0–255。
    @return 0（黑）到 1（白）之间的相对亮度。
    """
    channels = []
    for value in rgb:
        c = value / 255
        channels.append(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)
    return 0.2126 * channels[0] + 0.7152 * channels[1] + 0.0722 * channels[2]


def readable_on(rgb):
    """@brief 为给定底色选择深色或浅色文字，保证对比度。
    @param rgb 背景色 (r, g, b) 三元组。
    @return 深色或浅色文字用的 "#rrggbb"。
    """
    return "#10161f" if relative_luminance(rgb) > 0.45 else "#ffffff"


def tokens(dark=False):
    """@brief 取得当前主题的令牌字典。
    @param dark 为真时返回深色主题。
    @return 令牌字典，包含颜色键与 RADIUS、SPACE、字体栈。
    """
    value = dict(DARK if dark else LIGHT)
    value["radius"] = RADIUS
    value["space"] = SPACE
    value["dark"] = dark
    return value


def categorical(dark=False):
    """@return 当前主题下按机号循环使用的分类色列表。"""
    return CATEGORICAL_DARK if dark else CATEGORICAL


def stylesheet(dark=False):
    """@brief 生成应用级 Qt 样式表。
    @param dark 为真时使用深色令牌。
    @return 覆盖主要控件类型的 QSS 字符串。
    @note 只使用 Qt 支持的属性；QSS 不支持 box-shadow，卡片阴影靠边框与底色层次表达。
    """
    t = tokens(dark)
    return f"""
QWidget {{
    background: {t['bg']};
    color: {t['ink']};
    font-family: {FONT_STACK};
    font-size: 13px;
}}
QMainWindow, QDialog {{ background: {t['bg']}; }}

/* 卡片：白底、细边框、圆角，作为页面的基本分组单位。 */
QGroupBox {{
    background: {t['surface']};
    border: 1px solid {t['border']};
    border-radius: {RADIUS['lg']}px;
    margin-top: 18px;
    padding: {SPACE['md']}px;
    font-weight: 600;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: {SPACE['md']}px;
    padding: 2px 8px;
    color: {t['ink_soft']};
    background: transparent;
}}

QLabel {{ background: transparent; }}
QWidget#sensorCard {{
    background: {t['surface']};
    border: 1px solid {t['border']};
    border-radius: {RADIUS['lg']}px;
}}
QWidget#statTile {{
    background: {t['surface_alt']};
    border: 1px solid {t['border']};
    border-radius: {RADIUS['md']}px;
}}
QLabel[role="pageTitle"] {{
    font-size: 22px;
    font-weight: 700;
    color: {t['ink']};
}}
QLabel[role="sectionTitle"] {{
    font-size: 15px;
    font-weight: 600;
    color: {t['ink']};
}}
QLabel[role="hint"] {{
    color: {t['ink_muted']};
    font-size: 12px;
}}
QLabel[role="metric"] {{
    font-size: 21px;
    font-weight: 700;
    color: {t['ink']};
}}
QLabel[role="metricUnit"] {{
    font-size: 12px;
    color: {t['ink_muted']};
}}
QLabel[role="card"] {{
    background: {t['surface']};
    border: 1px solid {t['border']};
    border-radius: {RADIUS['lg']}px;
}}
QLabel[role="note"] {{
    background: {t['accent_soft']};
    border: 1px solid {t['accent_border']};
    border-radius: {RADIUS['md']}px;
    padding: 10px 12px;
    color: {t['ink_soft']};
}}
QLabel[tone="ok"]    {{ color: {t['ok']}; font-weight: 600; }}
QLabel[tone="warn"]  {{ color: {t['warn']}; font-weight: 600; }}
QLabel[tone="danger"]{{ color: {t['danger']}; font-weight: 600; }}
QLabel[tone="muted"] {{ color: {t['ink_muted']}; }}

QPushButton {{
    background: {t['surface']};
    color: {t['ink']};
    border: 1px solid {t['border_strong']};
    border-radius: {RADIUS['sm']}px;
    padding: 7px 14px;
    min-height: 20px;
}}
QPushButton:hover  {{ background: {t['accent_soft']}; border-color: {t['accent_border']}; }}
QPushButton:pressed{{ background: {t['surface_sunken']}; }}
QPushButton:disabled {{
    color: {t['ink_muted']};
    background: {t['surface_alt']};
    border-color: {t['border']};
}}
QPushButton[variant="primary"] {{
    background: {t['accent']};
    color: {t['ink_on_accent']};
    border-color: {t['accent']};
    font-weight: 600;
}}
QPushButton[variant="primary"]:hover   {{ background: {t['accent_hover']}; }}
QPushButton[variant="primary"]:pressed {{ background: {t['accent_press']}; }}
QPushButton[variant="danger"] {{
    background: {t['danger']};
    color: {t['ink_on_accent']};
    border-color: {t['danger']};
    font-weight: 600;
}}
QPushButton[variant="danger"]:hover {{ background: {t['danger']}; }}
QPushButton[variant="ghost"] {{
    background: transparent;
    border-color: transparent;
    color: {t['accent']};
    font-weight: 600;
}}
QPushButton[variant="ghost"]:hover {{
    background: {t['accent_soft']};
    border-color: {t['accent_border']};
}}
QPushButton:checked {{
    background: {t['accent_soft']};
    border-color: {t['accent']};
    color: {t['accent']};
    font-weight: 600;
}}

QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QAbstractSpinBox {{
    background: {t['surface']};
    color: {t['ink']};
    border: 1px solid {t['border_strong']};
    border-radius: {RADIUS['sm']}px;
    padding: 6px 8px;
    selection-background-color: {t['accent']};
    selection-color: {t['ink_on_accent']};
}}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
    border-color: {t['accent']};
}}
QLineEdit:disabled, QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled {{
    background: {t['surface_alt']};
    color: {t['ink_muted']};
}}
QComboBox::drop-down {{ border: none; width: 22px; }}
QComboBox QAbstractItemView {{
    background: {t['surface']};
    border: 1px solid {t['border_strong']};
    border-radius: {RADIUS['sm']}px;
    selection-background-color: {t['accent_soft']};
    selection-color: {t['ink']};
    padding: 4px;
}}

QPlainTextEdit, QTextEdit, QTextBrowser {{
    background: {t['surface_sunken']};
    color: {t['ink_soft']};
    border: 1px solid {t['border']};
    border-radius: {RADIUS['md']}px;
    padding: 8px;
    font-family: {MONO_STACK};
    font-size: 12px;
}}
QTextBrowser {{ font-family: {FONT_STACK}; font-size: 13px; }}

QTabWidget::pane {{
    border: 1px solid {t['border']};
    border-radius: {RADIUS['lg']}px;
    background: {t['bg']};
    top: -1px;
}}
QTabBar::tab {{
    background: transparent;
    color: {t['ink_muted']};
    border: 1px solid transparent;
    border-top-left-radius: {RADIUS['sm']}px;
    border-top-right-radius: {RADIUS['sm']}px;
    padding: 10px 18px;
    margin-right: 2px;
    font-weight: 600;
}}
QTabBar::tab:hover {{ color: {t['ink']}; background: {t['surface_alt']}; }}
QTabBar::tab:selected {{
    background: {t['surface']};
    color: {t['accent']};
    border-color: {t['border']};
    border-bottom-color: {t['surface']};
}}

QSlider::groove:horizontal {{
    height: 6px;
    background: {t['surface_sunken']};
    border-radius: 3px;
}}
QSlider::sub-page:horizontal {{ background: {t['accent']}; border-radius: 3px; }}
QSlider::handle:horizontal {{
    background: {t['surface']};
    border: 2px solid {t['accent']};
    width: 14px;
    height: 14px;
    margin: -6px 0;
    border-radius: 8px;
}}

QCheckBox, QRadioButton {{ spacing: 7px; background: transparent; }}
QCheckBox::indicator, QRadioButton::indicator {{
    width: 15px; height: 15px;
    border: 1px solid {t['border_strong']};
    background: {t['surface']};
}}
QCheckBox::indicator {{ border-radius: 4px; }}
QRadioButton::indicator {{ border-radius: 8px; }}
QCheckBox::indicator:checked, QRadioButton::indicator:checked {{
    background: {t['accent']};
    border-color: {t['accent']};
}}

QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{
    background: {t['border_strong']};
    border-radius: 5px;
    min-height: 30px;
}}
QScrollBar::handle:vertical:hover {{ background: {t['ink_muted']}; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle:horizontal {{
    background: {t['border_strong']};
    border-radius: 5px;
    min-width: 30px;
}}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

QStatusBar {{ background: {t['surface']}; border-top: 1px solid {t['border']}; }}
QStatusBar::item {{ border: none; }}
QToolTip {{
    background: {t['ink']};
    color: {t['surface']};
    border: none;
    border-radius: {RADIUS['sm']}px;
    padding: 6px 9px;
}}
QTableWidget, QTableView {{
    background: {t['surface']};
    alternate-background-color: {t['surface_alt']};
    border: 1px solid {t['border']};
    border-radius: {RADIUS['md']}px;
    gridline-color: {t['grid']};
    selection-background-color: {t['accent_soft']};
    selection-color: {t['ink']};
}}
QHeaderView::section {{
    background: {t['surface_alt']};
    color: {t['ink_soft']};
    border: none;
    border-bottom: 1px solid {t['border']};
    padding: 7px 9px;
    font-weight: 600;
}}
QToolBar {{ background: {t["bg"]}; border: none; }}
QToolButton {{ background: transparent; border: none; }}
QToolButton:hover {{ background: {t['accent_soft']}; border-radius: {RADIUS['sm']}px; }}
"""


def apply_theme(app, dark=False):
    """@brief 把样式表、调色板和中文字体应用到 QApplication。
    @param app QApplication 实例。
    @param dark 为真时使用深色主题。
    @return 实际使用的主题令牌字典。
    @note Fusion 样式在两种主题下表现一致，因此固定使用 Fusion 而不是跟随系统样式。
    """
    from PySide6.QtGui import QColor, QFont, QPalette

    t = tokens(dark)
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei UI", 10))
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(t["bg"]))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(t["ink"]))
    palette.setColor(QPalette.ColorRole.Base, QColor(t["surface"]))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(t["surface_alt"]))
    palette.setColor(QPalette.ColorRole.Text, QColor(t["ink"]))
    palette.setColor(QPalette.ColorRole.Button, QColor(t["surface"]))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(t["ink"]))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(t["accent"]))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(t["ink_on_accent"]))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(t["ink"]))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(t["surface"]))
    app.setPalette(palette)
    app.setStyleSheet(stylesheet(dark))
    return t


def matplotlib_rc(dark=False):
    """@brief 生成 Matplotlib rcParams 覆盖项，使图表与 Qt 界面同一套配色。
    @param dark 为真时使用深色令牌。
    @return 可直接交给 rc_context 的字典。
    """
    t = tokens(dark)
    return {
        "font.sans-serif": ["Microsoft YaHei", "Microsoft YaHei UI", "DejaVu Sans"],
        "font.size": 9,
        "axes.unicode_minus": False,
        "figure.facecolor": t["surface"],
        "figure.edgecolor": t["surface"],
        "savefig.facecolor": t["surface"],
        "axes.facecolor": t["surface"],
        "axes.edgecolor": t["border_strong"],
        "axes.labelcolor": t["ink_soft"],
        "axes.titlecolor": t["ink"],
        "axes.titlesize": 10,
        "axes.titleweight": "bold",
        "axes.labelsize": 9,
        "axes.grid": True,
        "axes.axisbelow": True,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "grid.color": t["grid"],
        "grid.linewidth": 0.8,
        "xtick.color": t["ink_muted"],
        "ytick.color": t["ink_muted"],
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "text.color": t["ink"],
        "legend.frameon": True,
        "legend.framealpha": 0.92,
        "legend.facecolor": t["surface"],
        "legend.edgecolor": t["border"],
        "legend.fontsize": 8,
        "lines.linewidth": 1.4,
        "lines.solid_capstyle": "round",
    }


@contextmanager
def mpl_context(dark=False):
    """@brief 在统一的图表主题下执行绘图代码。
    @param dark 为真时使用深色图表主题。
    @yield 无；退出时恢复原有 rcParams。
    """
    from matplotlib import rc_context

    with rc_context(matplotlib_rc(dark)):
        yield


def sensor_color(index, dark=False):
    """@brief 取得传感器对应的固定颜色，保证全应用一致。
    @param index 零起始传感器编号。
    @param dark 为真时返回深色主题下的变体。
    @return "#rrggbb" 颜色字符串。
    """
    pair = ("#e07a1f", "#1668c4") if not dark else ("#f0a04b", "#5aa2f0")
    return pair[index % 2]


SENSOR_NAMES = ("传感器 1", "传感器 2")

# 5. 任务阶段的中文名与配色：状态栏、阶段条和飞行曲线共用同一份定义，
#    避免同一阶段在不同界面显示成不同颜色。
PHASE_NAMES = {
    "WAIT_READY": "等待多机就绪",
    "WAITING": "等待协同起飞",
    "LAUNCH": "协同起飞",
    "TAKEOFF": "起飞",
    "TRANSIT": "航行",
    "ASSEMBLE": "集结",
    "APPROACH": "接近",
    "CONTACT": "轮流撞击",
    "RETREAT": "后撤",
    "RETURN": "返航",
    "RETURN_ALL": "共同返航",
    "LAND": "降落",
    "LANDING": "降落",
    "LAND_ALL": "多机降落",
    "COMPLETE": "完成",
    "ABORT": "中止",
    "ABORT_RETURN": "中止返航",
    "ABORTED": "中止",
    "PAUSED": "任务暂停",
}

PHASE_COLORS = {
    "WAIT_READY": "#94a3b3",
    "TAKEOFF": "#2f9e8f",
    "TRANSIT": "#3f7fbf",
    "ASSEMBLE": "#6aa3d4",
    "APPROACH": "#d29b2c",
    "CONTACT": "#cf4d55",
    "RETREAT": "#a06cb8",
    "RETURN_ALL": "#4f8fca",
    "LAND_ALL": "#3fa88f",
    "COMPLETE": "#3f9e63",
    "ABORT_RETURN": "#b5762a",
    "ABORTED": "#a8444c",
}

PHASE_COLORS_DARK = {
    "WAIT_READY": "#7c8b9c",
    "TAKEOFF": "#41c0ac",
    "TRANSIT": "#5aa2f0",
    "ASSEMBLE": "#8bbdea",
    "APPROACH": "#e8b448",
    "CONTACT": "#f0736f",
    "RETREAT": "#bd8ad4",
    "RETURN_ALL": "#6aa9e0",
    "LAND_ALL": "#4fc4a6",
    "COMPLETE": "#5cbd82",
    "ABORT_RETURN": "#d99a45",
    "ABORTED": "#d4666c",
}


def phase_name(phase):
    """@brief 把阶段枚举名转成界面用中文名。
    @param phase 任务阶段原始名称。
    @return 中文名称；未收录时原样返回。
    """
    if not phase:
        return "等待任务阶段"
    return PHASE_NAMES.get(phase, f"任务阶段：{phase}")


def phase_color(phase, dark=False):
    """@brief 取得任务阶段对应的颜色。
    @param phase 任务阶段原始名称。
    @param dark 为真时返回深色主题下的变体。
    @return "#rrggbb" 颜色字符串；未收录时返回中性灰。
    """
    table = PHASE_COLORS_DARK if dark else PHASE_COLORS
    return table.get(phase, "#94a3b3" if not dark else "#7c8b9c")
