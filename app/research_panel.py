"""@file research_panel.py
@brief 离线预研模型的参数表单、后台计算和结果展示。
模型由独立 Python 进程运行并保存 JSON，Qt 主线程接收结果信号后生成 HTML 报告。
报告配色跟随 app.theme，深浅主题切换时重新渲染已显示的结果。
"""

import html
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QThread, Signal, QUrl
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QGroupBox,
    QDoubleSpinBox,
    QComboBox,
    QPushButton,
    QTextBrowser,
    QLabel,
    QSizePolicy,
)

from . import theme

# 每个模型描述为 (runner 模型名, 参数列表)；参数依次是键、含单位的标签、
# 默认值、下限、上限。同一描述同时决定控件范围与提交给 runner 的配置键。
MODELS = {
    "失效与恢复状态机": ("control_policy", []),
    "I²C 与串口带宽预算": (
        "bandwidth",
        [
            ("i2c_khz", "I²C 时钟 kHz", 400.0, 100.0, 1000.0),
            ("sensor_count", "共享总线传感器数（整数）", 2.0, 1.0, 4.0),
            ("sensor_hz", "每颗传感器 Hz", 15.0, 1.0, 15.0),
            ("payload_bytes", "单帧读取字节数（替换为实测）", 1500.0, 64.0, 4096.0),
            ("chunk_bytes", "每次 I²C 分块字节", 256.0, 32.0, 1024.0),
            ("uart_baud", "UART 波特率", 115200.0, 57600.0, 921600.0),
            ("attitude_hz", "飞控输出姿态 Hz", 100.0, 1.0, 200.0),
            ("odometry_hz", "未来外部里程计输入 Hz", 30.0, 1.0, 100.0),
        ],
    ),
    "任务调度与图像排队": (
        "compute",
        [
            ("camera_fps", "相机帧率 Hz", 30.0, 1.0, 120.0),
            ("cpu_pre_ms", "每帧 CPU 预处理 ms", 4.0, 0.1, 100.0),
            ("npu_ms", "每帧 NPU 服务 ms", 18.0, 0.1, 200.0),
            ("cpu_post_ms", "每帧 CPU 后处理 ms", 3.0, 0.1, 100.0),
            ("nav_ms", "单次定位 CPU 服务 ms", 5.0, 0.1, 100.0),
            ("nav_hz", "定位频率 Hz", 15.0, 1.0, 100.0),
            ("duration_s", "仿真时长 s", 10.0, 1.0, 60.0),
        ],
    ),
    "双 ToF 几何与可观测性": (
        "geometry",
        [
            ("height_m", "飞行高度 m", 2.5, 0.1, 4.0),
            ("max_range_m", "可靠测距上限 m", 3.0, 0.2, 4.0),
            ("mount_pitch_deg", "安装俯仰角 °（正朝上 / 负朝下）", 0.0, -70.0, 70.0),
            ("yaw_separation_deg", "传感器方位间隔 °", 90.0, 0.0, 180.0),
            ("noise_mm", "测距噪声标准差 mm", 30.0, 0.0, 150.0),
        ],
    ),
    "时间错配与接口门控": (
        "integration",
        [
            ("speed_mps", "水平速度 m/s", 1.0, 0.0, 5.0),
            ("yaw_rate_deg_s", "航向转速 °/s", 90.0, 0.0, 360.0),
            ("range_m", "结构距离 m", 3.0, 0.0, 4.0),
            ("delay_ms", "时间错配 ms", 50.0, 0.0, 500.0),
        ],
    ),
}


def result_html(result, dark=False):
    """@brief 将模型结果转换为 QTextBrowser 可显示的 HTML。
    @param result 模型结果字典，可含指标、假设、场景、决策和本地图路径。
    @param dark 为真时使用深色主题的配色。
    @return 完整 HTML 字符串。
    @note 普通动态文本进行 HTML 转义，避免参数中的尖括号被当作标签。
    """
    t = theme.tokens(dark)
    summary = html.escape(str(result.get("summary", "")))
    assumptions = "".join(
        "<li>" + html.escape(str(x)) + "</li>" for x in result.get("assumptions", [])
    )
    metrics = result.get("metrics", {})
    # 先比较 FIFO 与保留最新帧的完成量、延迟和内存，再递归展开完整指标。
    overview = ""
    if "fifo" in metrics and "latest" in metrics:

        def number(value):
            """@brief 格式化可缺失的统计量；None 表示没有完成样本。
            """
            return "无完成样本" if value is None else f"{value:.2f}"

        overview = (
            '<h3>策略对比</h3><table><tr><th>策略</th><th>完成帧数</th><th>主动丢弃</th>'
            "<th>完成帧龄 p95 / ms</th><th>完成但过期</th><th>峰值持有图像帧</th></tr>"
        )
        for key, label in (("fifo", "FIFO 排队"), ("latest", "仅保留最新等待帧")):
            values = metrics[key]
            overview += (
                f"<tr><td>{label}</td><td>{values['frames']['completed']}</td>"
                f"<td>{values['frames']['dropped']}</td>"
                f"<td>{number(values['frames']['age_ms']['p95'])}</td>"
                f"<td>{values['frames']['stale_completed']}</td>"
                f"<td>{values['memory']['max_held_frames']}</td></tr>"
            )
        overview += (
            "</table><p class='hint'>帧龄只对完成的样本统计；丢弃、期末积压和过期必须同时查看。"
            "图像帧内存不包括 SDK/VB/模型/内核。</p>"
        )

    def table(value, depth=0):
        """@brief 递归展开字典和短列表，叶子值转义后作为文本显示。
        @param value 指标树中的任意节点；长列表整体转为文本以限制嵌套。
        @param depth 当前嵌套层数，超过两层改用键值行列表，避免深层表格挤压。
        @return HTML 片段。
        """
        if isinstance(value, dict):
            if depth >= 2:
                return "".join(
                    f"<div class='kv'><span>{html.escape(str(k))}</span>"
                    f"<b>{html.escape(str(v))}</b></div>"
                    for k, v in value.items()
                )
            return "<table>" + "".join(
                "<tr><td>"
                + html.escape(str(k))
                + "</td><td>"
                + table(v, depth + 1)
                + "</td></tr>"
                for k, v in value.items()
            ) + "</table>"
        if isinstance(value, list) and len(value) < 25:
            return "<br>".join(table(x, depth + 1) for x in value)
        return html.escape(str(value))

    plots = ""
    if result.get("_plot_path"):
        plots = (
            '<p><img width="880" src="'
            + html.escape(
                QUrl.fromLocalFile(result["_plot_path"]).toString(), quote=True
            )
            + '"></p>'
        )
    scenes = ""
    if result.get("decisions"):
        mode_labels = {
            "STANDBY": "待机",
            "NAVIGATE": "导航准入（离线）",
            "DEGRADED": "退化",
            "RECOVER": "恢复观察",
            "TERMINATED": "终止锁存",
        }

        def decision_row(i, row):
            """@brief 把一次状态机决策整理为事件表行。
            @param i 零起始事件下标；显示序号从 1 开始。
            @param row 含时间、模式、请求动作和原因的决策字典。
            @return 已转义动态文本的 HTML 表行。
            """
            mode = mode_labels.get(row.get("mode"), row.get("mode", "—"))
            action = (
                "交由已配置飞控失效保护决定"
                if row.get("requested_action") == "REQUEST_CONFIGURED_FC_FAILSAFE"
                else "无实际控制指令"
            )
            reasons = "<br>".join(html.escape(str(r)) for r in row.get("reasons", []))
            return (
                f"<tr><td>{i + 1}</td><td>{html.escape(str(row.get('time_s', '—')))}</td>"
                f"<td>{html.escape(mode)}</td><td>{action}</td><td>{reasons}</td></tr>"
            )

        scenes = (
            '<h3>故障注入事件</h3><p class="hint">输入、标志和决策字段保存在 JSON。</p>'
            "<table><tr><th>步骤</th><th>时间 s</th><th>模式</th><th>处理意图</th>"
            "<th>原因</th></tr>"
            + "".join(decision_row(i, row) for i, row in enumerate(result["decisions"]))
            + "</table>"
        )
    if result.get("scenarios"):
        scenes = (
            '<h3>合成场景逐项结果</h3><table><tr><th>场景</th><th>理想命中区域数</th>'
            "<th>局部秩 / 3</th></tr>"
            + "".join(
                "<tr><td>"
                + html.escape(s["name"])
                + "</td><td>"
                + str(s["hit_count"])
                + "</td><td>"
                + str(s["rank"])
                + "</td></tr>"
                for s in result["scenarios"]
            )
            + "</table>"
        )
    return (
        "<html><head><style>"
        f"body{{font-family:'Microsoft YaHei UI';font-size:13px;color:{t['ink']};}}"
        f"h2{{font-size:17px;color:{t['ink']};margin:2px 0 10px 0;}}"
        f"h3{{font-size:14px;color:{t['ink_soft']};margin:18px 0 8px 0;"
        f"border-left:3px solid {t['accent']};padding-left:8px;}}"
        f"p{{line-height:1.65;color:{t['ink_soft']};}}"
        f"p.hint{{font-size:12px;color:{t['ink_muted']};}}"
        f"table{{border-collapse:collapse;margin:6px 0 12px 0;}}"
        f"th{{background:{t['surface_alt']};color:{t['ink_soft']};font-weight:600;"
        f"text-align:left;padding:7px 10px;border:1px solid {t['border']};}}"
        f"td{{padding:6px 10px;border:1px solid {t['border']};color:{t['ink']};"
        "vertical-align:top;}"
        f"table table{{margin:0;}}"
        f"ul{{color:{t['ink_soft']};line-height:1.7;}}"
        f"div.kv{{display:block;padding:2px 0;}}"
        f"div.kv span{{color:{t['ink_muted']};margin-right:10px;}}"
        f"div.kv b{{color:{t['ink']};}}"
        "</style></head><body>"
        "<h2>离线仿真结果</h2><p>"
        + summary
        + "</p>"
        + plots
        + overview
        + scenes
        + "<h3>指标</h3>"
        + table(metrics)
        + "<h3>输入假设</h3><ul>"
        + assumptions
        + "</ul><p class='hint'>配置、源码摘要和结果保存在 JSON。</p></body></html>"
    )


class SimulationWorker(QThread):
    """@brief 在工作线程内等待独立模型进程，完成后传递结果和文件路径。
    """
    result = Signal(dict, str)
    failure = Signal(str)

    def __init__(self, root, module_name, config):
        """@param root 结果保存目录所属的项目根。
        @param module_name research.runner 识别的模型名。
        @param config 表单生成的输入参数字典。
        """
        super().__init__()
        self.root, self.module_name, self.config = (
            root,
            module_name,
            config,
        )

    def run(self):
        """@brief 启动模型进程、读取保存的 JSON 并发送结果信号。
        @note 独立进程避免复用旧模块缓存，执行超时为 60 s；标准错误末尾用于报告进程失败。
        """
        try:
            folder = self.root / "research" / "results"
            folder.mkdir(parents=True, exist_ok=True)
            path = folder / (
                self.module_name
                + "_"
                + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                + ".json"
            )
            # 1. 使用新进程加载当前源码，避免 Python 导入缓存复用之前的模型。
            code_root = Path(__file__).resolve().parents[1]
            interpreter = code_root / ".venv" / "Scripts" / "python.exe"
            process = subprocess.run(
                [
                    str(interpreter) if interpreter.exists() else sys.executable,
                    "-m",
                    "research.runner",
                    "--model",
                    self.module_name,
                    "--config-json",
                    json.dumps(self.config),
                    "--output",
                    str(path),
                ],
                cwd=code_root,
                capture_output=True,
                timeout=60,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
            if process.returncode:
                raise RuntimeError(process.stderr.decode("utf-8", "replace")[-2000:])
            # 2. 子进程正常退出后读取其保存结果；绘图错误与数值结果分别处理，
            #    JSON 路径随信号送回界面，供用户定位原始输出。
            payload = json.loads(path.read_text(encoding="utf-8"))
            data = payload["result"]
            if payload.get("plot_path"):
                data = dict(data, _plot_path=payload["plot_path"])
            if payload.get("plot_error"):
                data = dict(
                    data, summary=str(data["summary"]) + "（绘图失败，数值结果已保存）"
                )
            self.result.emit(data, str(path))
        except Exception as exc:
            self.failure.emit(f"仿真未完成：{exc}")


class ResearchPanel(QWidget):
    """@brief 根据模型参数描述生成表单，并展示后台模型的结果。
    @note worker 非空表示任务尚未退出；运行时禁用模型选择和启动按钮，防止重复提交。
    """
    def __init__(self, root):
        """@param root 项目根，用于结果与预研说明路径。
        """
        super().__init__()
        self.root = Path(root)
        self.worker = None
        self.spins = {}
        self.dark = False
        self.last_result = None
        outer = QVBoxLayout(self)
        outer.setContentsMargins(18, 16, 18, 16)
        outer.setSpacing(14)
        title = QLabel("预研与仿真")
        title.setProperty("role", "pageTitle")
        outer.addWidget(title)
        note = QLabel(
            "电脑离线仿真：修改输入参数，比较实现策略。模型在独立进程中运行，结果保存为 JSON 后在此渲染。"
        )
        note.setProperty("role", "note")
        note.setWordWrap(True)
        outer.addWidget(note)

        toolbar = QGroupBox("模型")
        row = QHBoxLayout(toolbar)
        row.setContentsMargins(16, 14, 16, 14)
        row.setSpacing(10)
        self.choice = QComboBox()
        self.choice.addItems(MODELS)
        self.choice.setCurrentText("任务调度与图像排队")
        self.choice.setMinimumWidth(260)
        row.addWidget(QLabel("模型"))
        row.addWidget(self.choice)
        self.run_button = QPushButton("▶ 运行并保存结果")
        self.run_button.setProperty("variant", "primary")
        self.run_button.setMinimumHeight(34)
        self.run_button.clicked.connect(self.start)
        row.addWidget(self.run_button)
        row.addStretch()
        open_report = QPushButton("查看预研总览")
        open_report.clicked.connect(self.report)
        row.addWidget(open_report)
        folder = QPushButton("打开结果文件夹")
        folder.clicked.connect(self.open_folder)
        row.addWidget(folder)
        outer.addWidget(toolbar)

        # 参数表单只占左侧固定宽度；结果报告占满剩余空间，避免两者都被拉成一条细带。
        columns = QHBoxLayout()
        columns.setSpacing(14)
        self.params = QGroupBox("输入假设")
        self.params.setMaximumWidth(430)
        self.form = QFormLayout(self.params)
        self.form.setContentsMargins(16, 14, 16, 16)
        self.form.setSpacing(10)
        self.form.setLabelAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        )
        columns.addWidget(self.params, 0)
        result_box = QGroupBox("结果报告")
        result_layout = QVBoxLayout(result_box)
        result_layout.setContentsMargins(16, 14, 16, 16)
        result_layout.setSpacing(8)
        self.status = QLabel("选择模型、设置参数后点击“运行并保存结果”。")
        self.status.setProperty("role", "hint")
        self.status.setWordWrap(True)
        result_layout.addWidget(self.status)
        self.view = QTextBrowser()
        self.view.setOpenExternalLinks(True)
        self.view.setPlaceholderText("运行模型后在此显示指标、假设与图表。")
        result_layout.addWidget(self.view, 1)
        columns.addWidget(result_box, 1)
        outer.addLayout(columns, 1)
        self.choice.currentTextChanged.connect(self.populate)
        self.populate()

    def set_theme(self, dark):
        """@brief 切换深浅主题；已渲染的报告会用新配色重新生成。
        @param dark 为真表示深色主题。
        """
        self.dark = dark
        if self.last_result is not None:
            self.view.setHtml(result_html(self.last_result, dark))

    def populate(self):
        """@brief 根据 MODELS 重建参数表单；spins 将配置键映射到数值控件。
        """
        while self.form.rowCount():
            self.form.removeRow(0)
        self.spins = {}
        for key, label, default, minimum, maximum in MODELS[self.choice.currentText()][
            1
        ]:
            value = QDoubleSpinBox()
            value.setRange(minimum, maximum)
            value.setDecimals(2)
            value.setValue(default)
            value.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            self.form.addRow(label, value)
            self.spins[key] = value

    def start(self):
        """@brief 捕获当前参数为字典，连接结果、失败和结束信号后启动线程。
        """
        if self.worker:
            return
        module, _ = MODELS[self.choice.currentText()]
        config = {k: v.value() for k, v in self.spins.items()}
        # 线程持有本次表单快照；完成结果显示由 result 槽负责，解锁控件由 finished
        # 槽负责，因此成功和失败两条路径都会恢复操作入口。
        self.worker = SimulationWorker(self.root, module, config)
        self.worker.result.connect(self.show_result)
        self.worker.failure.connect(self.failed)
        self.worker.finished.connect(self.finished)
        self.run_button.setEnabled(False)
        self.choice.setEnabled(False)
        self.status.setText("正在计算…")
        self.worker.start()

    def show_result(self, result, path):
        """@brief 在主线程渲染结果 HTML 并显示保存文件名。
        @param result 模型结果字典。
        @param path 后台已保存的 JSON 路径。
        """
        self.last_result = result
        self.view.setHtml(result_html(result, self.dark))
        self.status.setText("结果已保存：" + Path(path).name)

    def failed(self, error):
        """@brief 将后台错误以纯文本显示，避免将错误内容解释成 HTML。
        """
        self.last_result = None
        self.status.setText(error)
        self.view.setPlainText(error)

    def finished(self):
        """@brief 线程退出后延迟销毁对象，并恢复参数选择与运行入口。
        """
        self.worker.deleteLater()
        self.worker = None
        self.run_button.setEnabled(True)
        self.choice.setEnabled(True)

    def report(self):
        """@brief 在结果浏览器中打开本地预研总览。
        """
        path = self.root / "docs" / "research" / "index.html"
        if path.exists():
            self.last_result = None
            self.view.setSource(QUrl.fromLocalFile(str(path)))
        else:
            self.view.setPlainText("预研总览文件不存在。")

    def open_folder(self):
        """@brief 确保结果目录存在，并交给系统文件管理器打开。
        """
        folder = self.root / "research" / "results"
        folder.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder)))
