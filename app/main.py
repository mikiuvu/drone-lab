"""Desktop entry point: python -m app.main (or the supplied Windows shortcut)."""

import csv
import json
import os
import statistics
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, QUrl, QProcess
from PySide6.QtGui import QDesktopServices, QFont
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QSpinBox,
    QDoubleSpinBox,
    QLineEdit,
    QCheckBox,
    QTabWidget,
    QGroupBox,
    QFormLayout,
    QPlainTextEdit,
    QTextBrowser,
    QFileDialog,
    QMessageBox,
    QSlider,
)
from serial.tools import list_ports
from .data import (
    demonstration,
    validate_record,
    valid_distances,
    Recorder,
    load_recording,
)
from .widgets import SensorCard, AttitudeView, Trend
from .workers import FCWorker, NanoWorker, sensor_arguments
from .research_panel import ResearchPanel
from .simulation_panel import GazeboPanel

ROOT = Path(__file__).resolve().parents[1]


def button(text, fn):
    result = QPushButton(text)
    result.clicked.connect(fn)
    result.setMinimumHeight(34)
    return result


class LabWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Lab · DM-FC01 / LicheeRV Nano / 双 VL53L5CX")
        self.resize(1250, 900)
        self.workers = {}
        self.recorder = None
        self.mode = "idle"
        self.demo_sequence = 0
        self.replay = []
        self.replay_index = 0
        self.replay_time = 0.0
        self.replay_playing = False
        self.previous_tick = time.monotonic()
        self.attitude_time = None
        self.build_process = None
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.make_live()
        self.make_nano()
        self.make_replay()
        self.make_tools()
        self.make_help()
        self.research_panel = ResearchPanel(ROOT)
        self.tabs.addTab(self.research_panel, "⑥ 预研与仿真")
        self.gazebo_panel = GazeboPanel(ROOT)
        self.tabs.addTab(self.gazebo_panel, "⑦ Gazebo 多机")
        self.load_settings()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.refresh_ports()
        self.log(
            "准备就绪。点击“演示数据”查看界面。"
        )

    def make_live(self):
        page = QWidget()
        outer = QVBoxLayout(page)
        row = QHBoxLayout()
        self.mode_label = QLabel("未连接 · 等待选择数据源")
        self.mode_label.setStyleSheet("font-size:19px;font-weight:600;color:#176a89")
        row.addWidget(self.mode_label, 1)
        self.demo_button = button("▶ 演示数据", self.start_demo)
        row.addWidget(self.demo_button)
        row.addWidget(button("停止数据源", self.stop_sources))
        self.record_button = button("● 开始录制", self.toggle_record)
        row.addWidget(self.record_button)
        outer.addLayout(row)
        controls = QHBoxLayout()
        controls.addWidget(QLabel("DM-FC01 直连电脑："))
        self.fc_port = QComboBox()
        self.fc_port.setEditable(True)
        self.fc_port.setMinimumWidth(200)
        controls.addWidget(self.fc_port)
        controls.addWidget(button("刷新串口", self.refresh_ports))
        self.fc_baud = QComboBox()
        self.fc_baud.addItems(["115200", "57600", "230400", "460800", "921600"])
        controls.addWidget(self.fc_baud)
        controls.addWidget(button("接收飞控数据", self.start_fc))
        controls.addStretch()
        outer.addLayout(controls)
        note = QLabel(
            "串口同一时刻仅供一个软件使用：QGC 占用时请先断开。热力图为区域测距，未做定位或姿态补偿。"
        )
        note.setWordWrap(True)
        outer.addWidget(note)
        body = QHBoxLayout()
        self.cards = [SensorCard(0), SensorCard(1)]
        for card in self.cards:
            body.addWidget(card, 3)
        attitude = QGroupBox("飞控姿态")
        layout = QVBoxLayout(attitude)
        self.attitude = AttitudeView()
        layout.addWidget(self.attitude, 1)
        self.angles = QLabel("Roll —°\nPitch —°\nYaw —°")
        self.angles.setStyleSheet("font-size:18px")
        layout.addWidget(self.angles)
        self.heartbeat = QLabel("心跳：未收到")
        self.heartbeat.setWordWrap(True)
        layout.addWidget(self.heartbeat)
        self.attitude_status = QLabel("等待数据")
        layout.addWidget(self.attitude_status)
        body.addWidget(attitude, 2)
        outer.addLayout(body, 1)
        view = QHBoxLayout()
        self.flip = QCheckBox("热力图旋转 180°（传感器光学反转）")
        self.flip.setChecked(True)
        self.flip.toggled.connect(self.update_map_options)
        view.addWidget(self.flip)
        view.addWidget(QLabel("色标上限 mm"))
        self.maximum = QSpinBox()
        self.maximum.setRange(100, 10000)
        self.maximum.setValue(4000)
        self.maximum.valueChanged.connect(self.update_map_options)
        view.addWidget(self.maximum)
        view.addStretch()
        view.addWidget(QLabel("灰色 = 无有效目标；悬停单元格查看原始状态"))
        outer.addLayout(view)
        outer.addWidget(QLabel("最近 200 帧中位距离：蓝 = 传感器 1，橙 = 传感器 2"))
        self.trend = Trend()
        outer.addWidget(self.trend)
        self.events = QPlainTextEdit()
        self.events.setReadOnly(True)
        self.events.setMaximumHeight(115)
        self.events.document().setMaximumBlockCount(500)
        outer.addWidget(self.events)
        self.tabs.addTab(page, "① 实时数据")

    def make_nano(self):
        page = QWidget()
        outer = QVBoxLayout(page)
        intro = QLabel(
            "Nano 先完成镜像启动与联网。填入地址后可检查设备、上传已编译程序并采集。密码只保留在当前输入框，不写入配置文件。"
        )
        intro.setWordWrap(True)
        outer.addWidget(intro)
        connection = QGroupBox("SSH 连接")
        form = QFormLayout(connection)
        self.nano_host = QLineEdit()
        self.nano_host.setPlaceholderText("Nano 的实际 IP 地址")
        self.nano_port = QSpinBox()
        self.nano_port.setRange(1, 65535)
        self.nano_port.setValue(22)
        self.nano_user = QLineEdit("root")
        self.nano_password = QLineEdit()
        self.nano_password.setEchoMode(QLineEdit.EchoMode.Password)
        form.addRow("地址", self.nano_host)
        form.addRow("SSH 端口", self.nano_port)
        form.addRow("用户名", self.nano_user)
        form.addRow("密码", self.nano_password)
        outer.addWidget(connection)
        row = QHBoxLayout()
        row.addWidget(
            button("检查 Nano 与设备列表", lambda: self.start_nano("inspect"))
        )
        row.addWidget(button("上传采集程序", lambda: self.start_nano("deploy")))
        row.addWidget(button("保存连接配置", self.save_settings))
        row.addStretch()
        outer.addLayout(row)
        settings = QGroupBox("采集设置 · 初次先启用一颗传感器")
        grid = QGridLayout(settings)
        self.sensor_enable = []
        self.bus = []
        self.address = []
        for i in range(2):
            enable = QCheckBox(f"传感器 {i + 1}")
            enable.setChecked(i == 0)
            if i == 0:
                enable.setEnabled(False)
                enable.setToolTip("先使用第一颗；第二颗可按需启用。")
            bus = QComboBox()
            bus.setEditable(True)
            bus.setMinimumWidth(200)
            bus.setPlaceholderText("检查 Nano 后选择 /dev/i2c-*")
            address = QLineEdit("0x29")
            grid.addWidget(enable, i, 0)
            grid.addWidget(bus, i, 1)
            grid.addWidget(QLabel("7 位地址"), i, 2)
            grid.addWidget(address, i, 3)
            self.sensor_enable.append(enable)
            self.bus.append(bus)
            self.address.append(address)
        self.nano_hz = QSpinBox()
        self.nano_hz.setRange(1, 15)
        self.nano_hz.setValue(10)
        grid.addWidget(QLabel("8×8 帧率 Hz"), 2, 0)
        grid.addWidget(self.nano_hz, 2, 1)
        self.nano_tty = QComboBox()
        self.nano_tty.setEditable(True)
        self.nano_tty.setPlaceholderText("留空：不经 Nano 接收飞控")
        self.nano_baud = QComboBox()
        self.nano_baud.addItems(["115200", "57600", "230400", "460800", "921600"])
        grid.addWidget(QLabel("Nano 飞控串口（可选）"), 3, 0)
        grid.addWidget(self.nano_tty, 3, 1)
        grid.addWidget(QLabel("波特率"), 3, 2)
        grid.addWidget(self.nano_baud, 3, 3)
        outer.addWidget(settings)
        note = QLabel(
            "两颗同地址模块请使用独立 I²C 总线，或先在硬件端完成 LPn 地址分配。连接前确认模块供电、3.3 V 信号电平和引脚复用。"
        )
        note.setWordWrap(True)
        outer.addWidget(note)
        row = QHBoxLayout()
        row.addWidget(button("▶ 上传并开始采集", lambda: self.start_nano("stream")))
        row.addWidget(button("停止 Nano 操作", lambda: self.stop_worker("nano")))
        row.addStretch()
        outer.addLayout(row)
        self.nano_events = QPlainTextEdit()
        self.nano_events.setReadOnly(True)
        self.nano_events.document().setMaximumBlockCount(1000)
        outer.addWidget(self.nano_events, 1)
        self.tabs.addTab(page, "② Nano 连接与部署")

    def make_replay(self):
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.addWidget(
            QLabel(
                "打开本程序保存的 JSONL 日志，播放时到“实时数据”页观察热力图与姿态。最大支持 128 MB。"
            )
        )
        row = QHBoxLayout()
        row.addWidget(button("打开日志", self.choose_replay))
        row.addWidget(
            button(
                "打开自带示例",
                lambda: self.open_replay(ROOT / "samples" / "demo.jsonl"),
            )
        )
        self.play_button = button("▶ 播放 / 暂停", self.play_pause)
        row.addWidget(self.play_button)
        row.addWidget(button("导出区域明细 CSV", self.export_csv))
        row.addStretch()
        outer.addLayout(row)
        self.replay_label = QLabel("尚未选择日志")
        outer.addWidget(self.replay_label)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.sliderReleased.connect(self.seek)
        outer.addWidget(self.slider)
        row = QHBoxLayout()
        row.addWidget(QLabel("播放速度"))
        self.speed = QDoubleSpinBox()
        self.speed.setRange(0.1, 4)
        self.speed.setSingleStep(0.25)
        self.speed.setValue(1)
        row.addWidget(self.speed)
        row.addStretch()
        outer.addLayout(row)
        self.replay_info = QPlainTextEdit()
        self.replay_info.setReadOnly(True)
        outer.addWidget(self.replay_info, 1)
        self.tabs.addTab(page, "③ 日志回放")

    def make_tools(self):
        page = QWidget()
        outer = QVBoxLayout(page)
        tools = QGridLayout()
        entries = [
            ("QGroundControl · 飞控配置", lambda: self.launch_qgc()),
            ("MobaXterm · 远程终端与文件", lambda: self.launch_moba()),
            ("Rufus · TF 卡镜像写入", lambda: self.launch_rufus()),
            (
                "打开官方镜像文件夹",
                lambda: self.open_path(ROOT / "images" / "nano-official"),
            ),
            ("打开飞控固件文件夹", lambda: self.open_path(ROOT / "firmware")),
            ("VS Code · 编辑项目", lambda: self.launch_code()),
            ("打开日志文件夹", lambda: self.open_path(ROOT / "logs")),
            (
                "打开飞控使用手册",
                lambda: self.open_path(ROOT / "docs" / "DM-FC01-V1.1.pdf"),
            ),
            ("重新编译 Nano 程序", self.build_nano),
        ]
        for i, (title, fn) in enumerate(entries):
            tools.addWidget(button(title, fn), i // 2, i % 2)
        outer.addLayout(tools)
        outer.addWidget(
            QLabel(
                "图形程序启动后独立运行。Rufus 会打开写卡界面，必须由你核对目标 TF 卡后点击开始。"
            )
        )
        self.tool_log = QPlainTextEdit()
        self.tool_log.setReadOnly(True)
        outer.addWidget(self.tool_log, 1)
        self.tabs.addTab(page, "④ 软件工具")

    def make_help(self):
        self.help = QTextBrowser()
        self.help.setOpenExternalLinks(True)
        guide = ROOT / "docs" / "quickstart.html"
        if guide.exists():
            self.help.setSource(QUrl.fromLocalFile(str(guide)))
        self.tabs.addTab(self.help, "⑤ 从这里开始")

    def log(self, text):
        line = f"[{datetime.now():%H:%M:%S}] {text}"
        self.events.appendPlainText(line)
        self.nano_events.appendPlainText(line)
        self.statusBar().showMessage(str(text).splitlines()[0], 8000)

    def error(self, text):
        self.log(text)
        QMessageBox.warning(self, "操作未完成", text)

    def refresh_ports(self):
        value = self.fc_port.currentData() or self.fc_port.currentText()
        self.fc_port.clear()
        for port in list_ports.comports():
            self.fc_port.addItem(f"{port.device} — {port.description}", port.device)
        if value:
            self.fc_port.setCurrentText(value)

    def reset_views(self):
        for card in self.cards:
            card.reset()
        self.attitude.angles = None
        self.attitude.update()
        self.attitude_time = None
        self.angles.setText("Roll —°\nPitch —°\nYaw —°")
        self.attitude_status.setText("等待数据")
        self.heartbeat.setText("心跳：未收到")
        for series in self.trend.values:
            series.clear()
        self.trend.update()

    def update_map_options(self):
        for card in self.cards:
            card.map.flip = self.flip.isChecked()
            card.map.maximum = self.maximum.value()
            card.map.update()

    def start_demo(self):
        if self.workers:
            self.error("请先停止真实设备连接，再开始演示。")
            return
        self.stop_record()
        self.mode = "demo"
        self.demo_sequence = 0
        self.demo_start = time.monotonic()
        self.replay_playing = False
        self.reset_views()
        self.mode_label.setText("演示模式 · 合成数据")
        self.tabs.setCurrentIndex(0)

    def start_fc(self):
        if "fc" in self.workers:
            return
        if "nano" in self.workers and self.nano_tty.currentText().strip():
            self.error("Nano 已选择飞控串口，请只保留一条姿态数据来源。")
            return
        device = self.fc_port.currentData() or self.fc_port.currentText().strip()
        if not device:
            self.error("未发现串口。请连接飞控 USB，再刷新串口。")
            return
        if self.mode != "live":
            self.stop_record()
            self.reset_views()
        self.mode = "live"
        self.replay_playing = False
        self.mode_label.setText("实测接收 · 数据源状态见各面板")
        self.attach_worker("fc", FCWorker(device, int(self.fc_baud.currentText())))

    def settings(self):
        return dict(
            host=self.nano_host.text(),
            port=self.nano_port.value(),
            user=self.nano_user.text(),
        )

    def sensor_config(self):
        sensors = [
            dict(
                bus=self.bus[i].currentText().strip(),
                address=self.address[i].text().strip(),
            )
            for i in range(2)
            if self.sensor_enable[i].isChecked()
        ]
        return dict(
            sensors=sensors,
            hz=self.nano_hz.value(),
            tty=self.nano_tty.currentText(),
            baud=int(self.nano_baud.currentText()),
        )

    def start_nano(self, action):
        if "nano" in self.workers:
            return
        if not self.nano_host.text().strip():
            self.error("请填写 Nano 的实际 IP 地址。")
            return
        config = self.sensor_config()
        if action == "stream":
            try:
                sensor_arguments(config)
            except (ValueError, KeyError) as exc:
                self.error(str(exc))
                return
            if config["tty"] and "fc" in self.workers:
                self.error("请先停止电脑直连飞控，避免混合两条姿态来源。")
                return
            if self.mode != "live":
                self.stop_record()
                self.reset_views()
            self.mode = "live"
            self.replay_playing = False
            self.mode_label.setText("实测接收 · 等待 Nano 测距")
        worker = NanoWorker(
            ROOT, self.settings(), self.nano_password.text(), action, config
        )
        worker.host_key.connect(
            lambda host, key, w=worker: w.answer_key(
                QMessageBox.question(
                    self,
                    "首次连接 Nano",
                    f"主机：{host}\n密钥指纹：{key}\n请核对这是你的设备。是否信任并保存此密钥？",
                )
                == QMessageBox.StandardButton.Yes
            )
        )
        worker.inspected.connect(self.nano_inspected)
        self.attach_worker("nano", worker)
        self.save_settings(silent=True)

    def nano_inspected(self, info):
        for combo in self.bus:
            previous = combo.currentText()
            combo.clear()
            combo.addItems([d for d in info["devices"] if "/i2c-" in d])
            combo.setCurrentText(previous)
        previous = self.nano_tty.currentText()
        self.nano_tty.clear()
        self.nano_tty.addItem("")
        self.nano_tty.addItems([d for d in info["devices"] if "/tty" in d])
        self.nano_tty.setCurrentText(previous)

    def attach_worker(self, key, worker):
        self.workers[key] = worker
        worker.record.connect(self.receive)
        worker.message.connect(self.log)
        worker.failure.connect(self.error)
        worker.finished.connect(lambda: self.worker_finished(key, worker))
        worker.start()

    def worker_finished(self, key, worker):
        if self.workers.get(key) is worker:
            self.workers.pop(key, None)
        self.log(f"{key} 操作已结束")
        worker.deleteLater()

    def stop_worker(self, key):
        if key in self.workers:
            self.workers[key].stop()
            self.log(f"正在停止 {key}…")

    def stop_sources(self):
        self.mode = "idle"
        self.replay_playing = False
        for worker in self.workers.values():
            worker.stop()
        self.stop_record()
        self.mode_label.setText("已停止 · 面板保留最后数据")

    def receive(self, item):
        if self.mode == "idle":
            return
        try:
            validate_record(item)
            if self.recorder:
                self.recorder.write(item)
        except (ValueError, OSError) as exc:
            self.stop_record()
            self.log(f"数据/录制错误：{exc}")
            return
        if item["type"] == "tof":
            self.cards[item["sensor"]].receive(item)
            values = valid_distances(item)
            if values:
                self.trend.values[item["sensor"]].append(statistics.median(values))
                self.trend.update()
        elif item["type"] == "attitude":
            angles = [item[k] for k in ("roll_deg", "pitch_deg", "yaw_deg")]
            self.attitude.angles = angles
            self.attitude.update()
            self.attitude_time = time.monotonic()
            self.angles.setText(
                f"Roll {angles[0]:.1f}°\nPitch {angles[1]:.1f}°\nYaw {angles[2]:.1f}°"
            )
            self.attitude_status.setText(f"来源：{item.get('source', '未知')}")
        else:
            self.heartbeat.setText(
                f"心跳：系统 {item.get('system_id', '?')} / {'已解锁' if item.get('armed') else '未解锁'}"
            )

    def toggle_record(self):
        if self.recorder:
            self.stop_record()
            return
        if self.mode not in ("live", "demo"):
            self.error("请先开始演示或实测采集。回放数据不重复录制。")
            return
        prefix = "DEMO" if self.mode == "demo" else "LIVE"
        path = ROOT / "logs" / f"{prefix}_{datetime.now():%Y%m%d_%H%M%S_%f}.jsonl"
        try:
            self.recorder = Recorder(path)
        except OSError as exc:
            self.error(str(exc))
            return
        self.record_button.setText("■ 停止录制")
        self.log(f"正在录制：{path.name}")

    def stop_record(self):
        if self.recorder:
            recorder = self.recorder
            self.recorder = None
            recorder.close()
            self.log(f"日志已保存：{recorder.path.name}（{recorder.count} 条）")
        self.record_button.setText("● 开始录制")

    def choose_replay(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "选择日志", str(ROOT / "logs"), "JSONL (*.jsonl)"
        )
        if path:
            self.open_replay(path)

    def open_replay(self, path):
        if self.workers:
            self.error("请先停止设备连接，再打开回放。")
            return
        try:
            records = load_recording(path)
        except (OSError, ValueError) as exc:
            self.error(str(exc))
            return
        self.stop_record()
        self.mode = "replay"
        self.replay = records
        self.replay_index = 0
        self.replay_time = 0
        self.replay_playing = False
        self.reset_views()
        self.mode_label.setText("日志回放 · 历史记录")
        sources = sorted({r.get("source", "未知") for _, r in records})
        self.replay_info.setPlainText(
            f"文件：{path}\n记录：{len(records)}\n时长：{records[-1][0]:.2f} 秒\n来源：{sources}\n回放按电脑接收时间排列；不同设备时钟尚未同步。"
        )
        self.replay_label.setText(f"{Path(path).name} · 0.00 / {records[-1][0]:.2f} 秒")
        self.log("日志已加载，点击播放。")

    def play_pause(self):
        if not self.replay or self.mode != "replay":
            return
        if self.replay_index >= len(self.replay):
            self.replay_index = 0
            self.replay_time = 0
            self.reset_views()
        self.replay_playing = not self.replay_playing

    def seek(self):
        if not self.replay or self.mode != "replay":
            return
        target = self.slider.value() / 1000 * self.replay[-1][0]
        self.replay_time = target
        self.replay_index = 0
        latest = {}
        self.reset_views()
        while (
            self.replay_index < len(self.replay)
            and self.replay[self.replay_index][0] <= target
        ):
            item = self.replay[self.replay_index][1]
            latest[(item["type"], item.get("sensor"))] = item
            self.replay_index += 1
        for item in latest.values():
            self.receive(item)

    def export_csv(self):
        if not self.replay:
            self.error("请先打开一个日志。")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "导出测距明细", str(ROOT / "logs" / "tof_export.csv"), "CSV (*.csv)"
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8-sig", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "record_time_s",
                        "source",
                        "sensor",
                        "zone",
                        "distance_mm",
                        "status",
                        "valid",
                    ]
                )
                for stamp, item in self.replay:
                    if item["type"] == "tof":
                        for zone, (d, s) in enumerate(
                            zip(item["distance_mm"], item["status"])
                        ):
                            writer.writerow(
                                [
                                    stamp,
                                    item.get("source"),
                                    item["sensor"],
                                    zone,
                                    d,
                                    s,
                                    int(s in (5, 9) and d > 0),
                                ]
                            )
            self.log(f"CSV 已导出：{path}")
        except OSError as exc:
            self.error(str(exc))

    def tick(self):
        now = time.monotonic()
        dt = now - self.previous_tick
        self.previous_tick = now
        if self.mode == "demo":
            for item in demonstration(now - self.demo_start, self.demo_sequence):
                self.receive(item)
            self.demo_sequence += 1
        elif self.mode == "replay" and self.replay_playing:
            self.replay_time += dt * self.speed.value()
            while (
                self.replay_index < len(self.replay)
                and self.replay[self.replay_index][0] <= self.replay_time
            ):
                self.receive(self.replay[self.replay_index][1])
                self.replay_index += 1
            end = self.replay[-1][0]
            self.slider.setValue(
                int(min(1, self.replay_time / end) * 1000) if end else 1000
            )
            self.replay_label.setText(
                f"{min(self.replay_time, end):.2f} / {end:.2f} 秒"
            )
            if self.replay_index >= len(self.replay):
                self.replay_playing = False
        if self.mode != "replay":
            for card in self.cards:
                card.mark_stale()
            if self.attitude_time and now - self.attitude_time > 2:
                self.attitude_status.setText("姿态数据已过期")

    def save_settings(self, checked=False, silent=False):
        value = {
            "ssh": self.settings(),
            "sensors": [
                dict(
                    enabled=self.sensor_enable[i].isChecked(),
                    bus=self.bus[i].currentText(),
                    address=self.address[i].text(),
                )
                for i in range(2)
            ],
            "hz": self.nano_hz.value(),
            "tty": self.nano_tty.currentText(),
            "baud": self.nano_baud.currentText(),
        }
        try:
            (ROOT / "config" / "local.json").write_text(
                json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        except OSError as exc:
            self.error(str(exc))
            return
        if not silent:
            self.log("配置已保存（不含密码）")

    def load_settings(self):
        path = ROOT / "config" / "local.json"
        if not path.exists():
            return
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            ssh = value.get("ssh", {})
            self.nano_host.setText(ssh.get("host", ""))
            self.nano_user.setText(ssh.get("user", "root"))
            self.nano_port.setValue(int(ssh.get("port", 22)))
            for i, sensor in enumerate(value.get("sensors", [])[:2]):
                self.sensor_enable[i].setChecked(sensor.get("enabled", False))
                self.bus[i].setCurrentText(sensor.get("bus", ""))
                self.address[i].setText(sensor.get("address", "0x29"))
            self.sensor_enable[0].setChecked(True)
            self.nano_hz.setValue(int(value.get("hz", 10)))
            self.nano_tty.setCurrentText(value.get("tty", ""))
            self.nano_baud.setCurrentText(value.get("baud", "115200"))
        except (ValueError, TypeError, OSError):
            pass

    def open_path(self, path):
        if not Path(path).exists():
            self.error(f"文件不存在：{path}")
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def launch_qgc(self):
        self.open_path(ROOT / "tools" / "QGroundControl" / "bin" / "QGroundControl.exe")

    def launch_moba(self):
        paths = list((ROOT / "tools" / "MobaXterm").glob("MobaXterm*.exe"))
        if paths:
            self.open_path(paths[0])
        else:
            self.error("MobaXterm 可执行文件缺失")

    def launch_rufus(self):
        answer = QMessageBox.information(
            self,
            "写卡工具",
            "将打开 Rufus。写镜像会清空你在 Rufus 中选中的磁盘，请只选择用于 Nano 的 TF 卡。此按钮不会自动写入。",
            QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel,
        )
        if answer == QMessageBox.StandardButton.Ok:
            self.open_path(ROOT / "tools" / "rufus-4.15p.exe")

    def launch_code(self):
        exe = (
            Path(os.environ["LOCALAPPDATA"])
            / "Programs"
            / "Microsoft VS Code"
            / "Code.exe"
        )
        if exe.exists():
            subprocess.Popen([str(exe), str(ROOT)])
        else:
            self.open_path(ROOT)

    def build_nano(self):
        if self.build_process is not None:
            return
        self.build_process = QProcess(self)
        self.build_process.setWorkingDirectory(str(ROOT))
        self.build_process.setProcessChannelMode(
            QProcess.ProcessChannelMode.MergedChannels
        )
        self.build_process.readyReadStandardOutput.connect(
            lambda: self.tool_log.appendPlainText(
                bytes(self.build_process.readAllStandardOutput()).decode(
                    "utf-8", "replace"
                )
            )
        )

        def done(code, status):
            self.tool_log.appendPlainText(f"编译结束，退出码 {code}")
            self.build_process.deleteLater()
            self.build_process = None

        def failed(error):
            self.tool_log.appendPlainText(f"无法启动编译：{error}")
            self.build_process.deleteLater()
            self.build_process = None

        self.build_process.finished.connect(done)
        self.build_process.errorOccurred.connect(failed)
        self.build_process.start(
            str(ROOT / ".venv" / "Scripts" / "python.exe"),
            [str(ROOT / "scripts" / "build_nano.py")],
        )

    def closeEvent(self, event):
        if self.research_panel.worker is not None:
            self.log("离线仿真仍在计算，请待结果保存后关闭。")
            event.ignore()
            return
        if self.workers:
            for worker in self.workers.values():
                worker.stop()
            self.log("正在关闭连接，操作结束后请再次关闭窗口。")
            event.ignore()
            return
        if self.build_process is not None:
            self.error("编译仍在运行，请等待结束后关闭。")
            event.ignore()
            return
        self.stop_record()
        self.save_settings(silent=True)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei UI", 10))
    app.setStyleSheet(
        "QMainWindow,QWidget{background:#f7f9fc;color:#19334b} QGroupBox{border:1px solid #d9e2ec;border-radius:8px;margin-top:12px;padding-top:14px} QGroupBox::title{subcontrol-origin:margin;left:12px} QPushButton{background:#e7eff7;border:1px solid #c8d8e8;border-radius:6px;padding:6px 12px} QPushButton:hover{background:#d4e8f8} QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox,QPlainTextEdit{background:white;border:1px solid #cbd8e4;border-radius:4px;padding:4px} QTabBar::tab{padding:12px 20px} QTabBar::tab:selected{background:#dbeefa;color:#075e7a}"
    )
    window = LabWindow()
    if "--gazebo" in sys.argv:
        window.tabs.setCurrentWidget(window.gazebo_panel)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
