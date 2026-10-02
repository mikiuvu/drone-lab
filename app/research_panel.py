"""Offline research GUI, isolated from live-device actions."""

import html
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from PySide6.QtCore import QThread, Signal, QUrl
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
)

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


def result_html(result):
    summary = html.escape(str(result.get("summary", "")))
    assumptions = "".join(
        "<li>" + html.escape(str(x)) + "</li>" for x in result.get("assumptions", [])
    )
    metrics = result.get("metrics", {})
    # Present a short Chinese comparison before the full machine-readable fields.
    overview = ""
    if "fifo" in metrics and "latest" in metrics:

        def number(value):
            return "无完成样本" if value is None else f"{value:.2f}"

        overview = '<h3>策略对比</h3><table border="1" cellpadding="6"><tr><th>策略</th><th>完成帧数</th><th>主动丢弃</th><th>完成帧龄 p95 / ms</th><th>完成但过期</th><th>峰值持有图像帧</th></tr>'
        for key, label in (("fifo", "FIFO 排队"), ("latest", "仅保留最新等待帧")):
            values = metrics[key]
            overview += f"<tr><td>{label}</td><td>{values['frames']['completed']}</td><td>{values['frames']['dropped']}</td><td>{number(values['frames']['age_ms']['p95'])}</td><td>{values['frames']['stale_completed']}</td><td>{values['memory']['max_held_frames']}</td></tr>"
        overview += "</table><p>帧龄只对完成的样本统计；丢弃、期末积压和过期必须同时查看。图像帧内存不包括 SDK/VB/模型/内核。</p>"

    def table(value):
        if isinstance(value, dict):
            return (
                '<table border="1" cellpadding="6">'
                + "".join(
                    "<tr><td>"
                    + html.escape(str(k))
                    + "</td><td>"
                    + table(v)
                    + "</td></tr>"
                    for k, v in value.items()
                )
                + "</table>"
            )
        if isinstance(value, list) and len(value) < 25:
            return "<br>".join(table(x) for x in value)
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
            mode = mode_labels.get(row.get("mode"), row.get("mode", "—"))
            action = (
                "交由已配置飞控失效保护决定"
                if row.get("requested_action") == "REQUEST_CONFIGURED_FC_FAILSAFE"
                else "无实际控制指令"
            )
            reasons = "<br>".join(html.escape(str(r)) for r in row.get("reasons", []))
            return f"<tr><td>{i + 1}</td><td>{html.escape(str(row.get('time_s', '—')))}</td><td>{html.escape(mode)}</td><td>{action}</td><td>{reasons}</td></tr>"

        scenes = (
            '<h3>故障注入事件</h3><p>输入、标志和决策字段保存在 JSON。</p><table border="1" cellpadding="6"><tr><th>步骤</th><th>时间 s</th><th>模式</th><th>处理意图</th><th>原因</th></tr>'
            + "".join(decision_row(i, row) for i, row in enumerate(result["decisions"]))
            + "</table>"
        )
    if result.get("scenarios"):
        scenes = (
            '<h3>合成场景逐项结果</h3><table border="1" cellpadding="6"><tr><th>场景</th><th>理想命中区域数</th><th>局部秩 / 3</th></tr>'
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
        "<html><body><h2>离线仿真结果</h2><p>"
        + summary
        + "</p>"
        + plots
        + overview
        + scenes
        + "<h3>指标</h3>"
        + table(metrics)
        + "<h3>输入假设</h3><ul>"
        + assumptions
        + "</ul><p>配置、源码摘要和结果保存在 JSON。</p></body></html>"
    )


class SimulationWorker(QThread):
    result = Signal(dict, str)
    failure = Signal(str)

    def __init__(self, root, module_name, config):
        super().__init__()
        self.root, self.module_name, self.config = (
            root,
            module_name,
            config,
        )

    def run(self):
        try:
            folder = self.root / "research" / "results"
            folder.mkdir(parents=True, exist_ok=True)
            path = folder / (
                self.module_name
                + "_"
                + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                + ".json"
            )
            # A fresh process prevents cached modules being attributed to edited source.
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
    def __init__(self, root):
        super().__init__()
        self.root = Path(root)
        self.worker = None
        self.spins = {}
        outer = QVBoxLayout(self)
        note = QLabel(
            "电脑离线仿真：修改输入参数，比较实现策略。"
        )
        note.setWordWrap(True)
        outer.addWidget(note)
        row = QHBoxLayout()
        self.choice = QComboBox()
        self.choice.addItems(MODELS)
        self.choice.setCurrentText("任务调度与图像排队")
        row.addWidget(self.choice)
        self.run_button = QPushButton("运行并保存结果")
        self.run_button.clicked.connect(self.start)
        row.addWidget(self.run_button)
        open_report = QPushButton("查看预研总览")
        open_report.clicked.connect(self.report)
        row.addWidget(open_report)
        folder = QPushButton("打开结果文件夹")
        folder.clicked.connect(self.open_folder)
        row.addWidget(folder)
        outer.addLayout(row)
        self.params = QGroupBox("输入假设")
        self.form = QFormLayout(self.params)
        outer.addWidget(self.params)
        self.status = QLabel(
            "选择模型、设置参数后点击“运行并保存结果”。"
        )
        outer.addWidget(self.status)
        self.view = QTextBrowser()
        self.view.setOpenExternalLinks(True)
        outer.addWidget(self.view, 1)
        self.choice.currentTextChanged.connect(self.populate)
        self.populate()

    def populate(self):
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
            self.form.addRow(label, value)
            self.spins[key] = value

    def start(self):
        if self.worker:
            return
        module, _ = MODELS[self.choice.currentText()]
        config = {k: v.value() for k, v in self.spins.items()}
        self.worker = SimulationWorker(self.root, module, config)
        self.worker.result.connect(self.show_result)
        self.worker.failure.connect(self.failed)
        self.worker.finished.connect(self.finished)
        self.run_button.setEnabled(False)
        self.choice.setEnabled(False)
        self.status.setText("正在计算…")
        self.worker.start()

    def show_result(self, result, path):
        self.view.setHtml(result_html(result))
        self.status.setText("结果已保存：" + Path(path).name)

    def failed(self, error):
        self.status.setText(error)
        self.view.setPlainText(error)

    def finished(self):
        self.worker.deleteLater()
        self.worker = None
        self.run_button.setEnabled(True)
        self.choice.setEnabled(True)

    def report(self):
        path = self.root / "docs" / "research" / "index.html"
        if path.exists():
            self.view.setSource(QUrl.fromLocalFile(str(path)))
        else:
            self.view.setPlainText("预研总览文件不存在。")

    def open_folder(self):
        folder = self.root / "research" / "results"
        folder.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder)))
