"""@file main.py
@brief Drone Lab 桌面入口与实时采集、录制、回放的界面调度。
通过 python -m app.main 启动。LabWindow 管理数据源状态，后台线程负责 I/O，Qt 主线程负责控件更新。
"""

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
    """@brief 创建统一尺寸按钮并连接 clicked 信号。
    @param text 按钮文字。
    @param fn 点击回调，由 Qt 调度执行。
    @return 已连接信号的 QPushButton。
    """
    result = QPushButton(text)
    result.clicked.connect(fn)
    result.setMinimumHeight(34)
    return result


class LabWindow(QMainWindow):
    """@brief 管理采集状态、显示面板与日志生命周期的主窗口。
    @note mode 为 idle/live/demo/replay；workers 保持线程引用直到 finished，避免运行中的线程被回收。
    """
    def __init__(self):
        """@brief 初始化面板和状态，并启动 100 ms 界面定时器。
        @note replay_time 是日志相对秒数，replay_index 指向下一条未播放记录；previous_tick 用于测量真实刷新间隔。
        """
        super().__init__()
        self.setWindowTitle("Drone Lab · DM-FC01 / LicheeRV Nano / 双 VL53L5CX")
        self.resize(1250, 900)
        self.workers = {}  # 数据源名到线程对象；stop 后仍保留，直到 finished 回调清理。
        self.recorder = None
        self.mode = "idle"
        self.demo_sequence = 0
        self.replay = []
        self.replay_index = 0  # 下一条未分发的回放记录下标。
        self.replay_time = 0.0  # 相对首帧的播放时间，单位 s。
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
        """@brief 建立实时页，将数据源操作、测距热图、姿态和趋势控件连接起来。
        """
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
        """@brief 建立 SSH 与传感器配置表单；按钮将操作类型传给 start_nano。
        """
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
        """@brief 建立日志选择、播放倍率和进度条；拖动释放后才执行 seek。
        """
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
        """@brief 建立外部工具入口和异步编译输出区域。
        """
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
        """@brief 使用本地 quickstart.html 填充帮助页。
        """
        self.help = QTextBrowser()
        self.help.setOpenExternalLinks(True)
        guide = ROOT / "docs" / "quickstart.html"
        if guide.exists():
            self.help.setSource(QUrl.fromLocalFile(str(guide)))
        self.tabs.addTab(self.help, "⑤ 从这里开始")

    def log(self, text):
        """@brief 将带本地时间的消息送到两个日志框，并短暂显示在状态栏。
        @param text 待显示的状态文本。
        """
        line = f"[{datetime.now():%H:%M:%S}] {text}"
        self.events.appendPlainText(line)
        self.nano_events.appendPlainText(line)
        self.statusBar().showMessage(str(text).splitlines()[0], 8000)

    def error(self, text):
        """@brief 记录错误并在主线程显示警告对话框。
        """
        self.log(text)
        QMessageBox.warning(self, "操作未完成", text)

    def refresh_ports(self):
        """@brief 枚举电脑串口；显示说明文字，设备名保存在选项数据中。
        """
        value = self.fc_port.currentData() or self.fc_port.currentText()
        self.fc_port.clear()
        for port in list_ports.comports():
            self.fc_port.addItem(f"{port.device} — {port.description}", port.device)
        if value:
            self.fc_port.setCurrentText(value)

    def reset_views(self):
        """@brief 清除上个数据源的统计、姿态和趋势，防止切换后混用历史。
        """
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
        """@brief 将旋转选项和毫米色标上限传给两幅热图，并请求重绘。
        """
        for card in self.cards:
            card.map.flip = self.flip.isChecked()
            card.map.maximum = self.maximum.value()
            card.map.update()

    def start_demo(self):
        """@brief 在设备线程全部退出后进入演示，重置录制和显示时间轴。
        """
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
        """@brief 创建电脑直连飞控线程，并检查与 Nano 姿态来源的互斥条件。
        """
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
        """@return SSH 地址、端口和用户名组成的字典，不包含密码。
        """
        return dict(
            host=self.nano_host.text(),
            port=self.nano_port.value(),
            user=self.nano_user.text(),
        )

    def sensor_config(self):
        """@return 已启用传感器、Hz 帧率和可选飞控串口配置；后续由 sensor_arguments 校验。
        """
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
        """@brief 校验连接设置并启动指定 Nano 操作。
        @param action inspect、deploy 或 stream；只有 stream 切换到实时数据模式。
        @note 首次密钥确认在界面弹窗中完成，再通过事件通知后台线程继续。
        """
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
        """@brief 用 Nano 实际设备列表更新下拉框，同时保留当前输入。
        @param info 后台 inspected 信号提供的架构和设备字典。
        """
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
        """@brief 持有线程、连接数据与结束信号，然后启动。
        @param key 数据源键，如 fc 或 nano。
        @param worker 尚未启动的 Worker 实例。
        """
        self.workers[key] = worker
        worker.record.connect(self.receive)
        worker.message.connect(self.log)
        worker.failure.connect(self.error)
        # 闭包固定这次的 key 和 worker，结束时能确认删除的仍是同一个实例。
        worker.finished.connect(lambda: self.worker_finished(key, worker))
        worker.start()

    def worker_finished(self, key, worker):
        """@brief 仅移除仍对应此实例的数据源，再延迟销毁 Qt 对象。
        @note 实例比较避免旧线程的结束回调误删后来创建的同名线程。
        """
        if self.workers.get(key) is worker:
            self.workers.pop(key, None)
        self.log(f"{key} 操作已结束")
        worker.deleteLater()

    def stop_worker(self, key):
        """@brief 请求指定数据源停止，保留引用等待 finished 清理。
        """
        if key in self.workers:
            self.workers[key].stop()
            self.log(f"正在停止 {key}…")

    def stop_sources(self):
        """@brief 进入空闲状态、请求设备退出并结束录制，保留最后画面。
        """
        self.mode = "idle"
        self.replay_playing = False
        for worker in self.workers.values():
            worker.stop()
        self.stop_record()
        self.mode_label.setText("已停止 · 面板保留最后数据")

    def receive(self, item):
        """@brief 实时、演示与回放共用的记录入口。
        @param item schema=1 数据字典。
        @note 先校验并按需录制，再按类型分发；空闲状态忽略已排队到达的记录。录制失败会停止该次数据更新。
        """
        if self.mode == "idle":
            return
        # 1. 显示与录制共用一次入口校验；先记录原始区域，再做显示统计，
        #    保证无效区域和完整状态码也能用于离线检查。
        try:
            validate_record(item)
            if self.recorder:
                self.recorder.write(item)
        except (ValueError, OSError) as exc:
            self.stop_record()
            self.log(f"数据/录制错误：{exc}")
            return
        # 2. 按类型分发到界面缓存。update 只请求重绘，不在此处同步执行绘图。
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
        """@brief 仅在实时或演示模式创建新日志；再次点击则关闭录制。
        """
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
        """@brief 先解除窗口的录制引用再关闭文件，后续记录不再写入。
        """
        if self.recorder:
            recorder = self.recorder
            self.recorder = None
            recorder.close()
            self.log(f"日志已保存：{recorder.path.name}（{recorder.count} 条）")
        self.record_button.setText("● 开始录制")

    def choose_replay(self):
        """@brief 从文件对话框取得 JSONL 路径并交给 open_replay。
        """
        path, _ = QFileDialog.getOpenFileName(
            self, "选择日志", str(ROOT / "logs"), "JSONL (*.jsonl)"
        )
        if path:
            self.open_replay(path)

    def open_replay(self, path):
        """@brief 完整校验日志后切换为回放，加载失败保留原状态。
        @param path JSONL 日志路径。
        @note 回放保留文件顺序，以电脑录制时间确定间隔；不直接对齐不同设备的启动时钟。
        """
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
        """@brief 切换回放播放状态；到末尾后再次播放会从头重置。
        """
        if not self.replay or self.mode != "replay":
            return
        if self.replay_index >= len(self.replay):
            self.replay_index = 0
            self.replay_time = 0
            self.reset_views()
        self.replay_playing = not self.replay_playing

    def seek(self):
        """@brief 将 0–1000 进度映射到日志秒数，并恢复目标时刻之前各类型的最新帧。
        @note 按 (type, sensor) 保留最后一条；重建瞬时画面而不补画全部趋势历史。
        """
        if not self.replay or self.mode != "replay":
            return
        target = self.slider.value() / 1000 * self.replay[-1][0]
        self.replay_time = target
        self.replay_index = 0
        latest = {}
        self.reset_views()
        # 1. 从头扫描到目标秒数，按类型和传感器覆盖缓存，留下各路最近的一帧。
        while (
            self.replay_index < len(self.replay)
            and self.replay[self.replay_index][0] <= target
        ):
            item = self.replay[self.replay_index][1]
            latest[(item["type"], item.get("sensor"))] = item
            self.replay_index += 1
        # 2. 通过统一入口恢复画面；索引已指向目标后的首条，继续播放不重复分发。
        for item in latest.values():
            self.receive(item)

    def export_csv(self):
        """@brief 将已加载日志的每个 ToF 区域展开为 CSV 一行。
        @note 时间采用归零后的回放秒数；UTF-8 BOM 便于表格软件识别中文，原始无效距离也保留。
        """
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
        """@brief 推进演示或回放，并检查实时数据是否过期。
        @note 使用实际单调时钟间隔乘播放倍率；一次补发所有到期记录，避免界面调度延迟造成漏帧。
        """
        now = time.monotonic()
        dt = now - self.previous_tick
        self.previous_tick = now
        if self.mode == "demo":
            for item in demonstration(now - self.demo_start, self.demo_sequence):
                self.receive(item)
            self.demo_sequence += 1
        elif self.mode == "replay" and self.replay_playing:
            # 1. Qt 定时器会受界面忙碌影响，所以用两次回调的实际秒差推进时间。
            self.replay_time += dt * self.speed.value()
            # 2. 用 while 补齐本次时间窗中的所有帧，倍率加快也不会只显示一条。
            while (
                self.replay_index < len(self.replay)
                and self.replay[self.replay_index][0] <= self.replay_time
            ):
                self.receive(self.replay[self.replay_index][1])
                self.replay_index += 1
            # 3. 显示进度限制在末尾；全部帧发完后暂停，保留最后画面。
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
        """@brief 将连接和采集表单保存为 config/local.json。
        @param checked 接收按钮 clicked 的布尔参数，本方法不使用。
        @param silent 为真时省略成功日志。
        """
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
        """@brief 从本地配置恢复表单；缺失或解析失败时保留控件当前值。
        """
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
        """@brief 检查路径存在后交给系统关联程序打开。
        """
        if not Path(path).exists():
            self.error(f"文件不存在：{path}")
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def launch_qgc(self):
        """@brief 打开项目附带的 QGroundControl。
        """
        self.open_path(ROOT / "tools" / "QGroundControl" / "bin" / "QGroundControl.exe")

    def launch_moba(self):
        """@brief 查找并打开项目工具目录中的 MobaXterm。
        """
        paths = list((ROOT / "tools" / "MobaXterm").glob("MobaXterm*.exe"))
        if paths:
            self.open_path(paths[0])
        else:
            self.error("MobaXterm 可执行文件缺失")

    def launch_rufus(self):
        """@brief 经界面确认后打开写卡工具，后续磁盘选择由该工具处理。
        """
        answer = QMessageBox.information(
            self,
            "写卡工具",
            "将打开 Rufus。写镜像会清空你在 Rufus 中选中的磁盘，请只选择用于 Nano 的 TF 卡。此按钮不会自动写入。",
            QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel,
        )
        if answer == QMessageBox.StandardButton.Ok:
            self.open_path(ROOT / "tools" / "rufus-4.15p.exe")

    def launch_code(self):
        """@brief 使用本机 VS Code 打开项目；程序缺失时打开项目目录。
        """
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
        """@brief 通过 QProcess 异步编译 Nano，并将合并输出送到工具日志。
        @note process 引用同时作为并发编译门控；结束或启动错误回调释放该引用。
        """
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
            """@brief 编译进程退出后显示退出码，解除并发编译门控。
            @param code 子进程退出码。
            @param status Qt 退出状态，由信号提供，此处不使用。
            """
            self.tool_log.appendPlainText(f"编译结束，退出码 {code}")
            self.build_process.deleteLater()
            self.build_process = None

        def failed(error):
            """@brief 接收 QProcess 错误并释放编译进程引用。
            @param error Qt 进程错误枚举。
            """
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
        """@brief 在后台任务结束后关闭窗口、日志和飞行曲线对话框。
        @param event Qt 关闭事件；仍有计算、连接或编译时忽略本次关闭。
        """
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
        self.gazebo_panel.close_flight_dashboard()
        self.save_settings(silent=True)
        event.accept()


def main():
    """@brief 创建 QApplication、配置外观并进入 Qt 事件循环。
    @note --gazebo 仅选择启动时显示的页签。
    """
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
