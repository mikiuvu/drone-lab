"""@file workers.py
@brief 串口和 SSH 后台采集线程。
阻塞 I/O 在 run 中执行，记录、状态和错误经信号送到界面；停止使用事件协作退出，资源在 finally 中释放。
"""

import base64
import hashlib
import json
import re
import shlex
import threading
import time
from pathlib import Path
import paramiko
from PySide6.QtCore import QThread, Signal
from .data import validate_record


class Worker(QThread):
    """@brief 采集线程公共接口。record/message/failure 分别传递记录、状态和错误。
    @note QThread 对象在创建它的线程中，只有 run 在工作线程执行；不要在 run 中直接操作控件。
    """
    record = Signal(dict)  # schema=1 记录；界面槽负责显示及录制。
    message = Signal(str)  # 连接进度或设备诊断，不作为数据帧解析。
    failure = Signal(str)  # 操作失败原因；线程退出另由 QThread.finished 通知。

    def __init__(self):
        """@brief 建立跨线程可见的停止事件，供 I/O 循环检查。
        """
        super().__init__()
        self.stop_event = threading.Event()

    def stop(self):
        """@brief 请求协作停止；返回时线程可能仍在等待 I/O，完成以 finished 信号为准。
        """
        self.stop_event.set()


class FCWorker(Worker):
    """@brief 从 MAVLink 串口接收姿态和心跳，转换为统一记录。
    """
    def __init__(self, device, baud):
        """@param device 串口设备名。
        @param baud 串口波特率。
        """
        super().__init__()
        self.device, self.baud = device, baud

    def run(self):
        """@brief 接收 MAVLink 并发出数据；连接错误转为 failure 信号。
        @note ATTITUDE 的毫秒启动时间转为微秒，欧拉角弧度转为度，角速度保留 MAVLink 的 rad/s。
        """
        from pymavlink import mavutil

        connection = None
        try:
            connection = mavutil.mavlink_connection(
                self.device, baud=self.baud, autoreconnect=False, source_system=254
            )
            self.message.emit(
                "飞控端口已打开；等待 MAVLink 数据。此连接只接收，不发送控制指令。"
            )
            last = time.monotonic()
            while not self.stop_event.is_set():
                # 有限等待让循环能定期检查停止事件，即使串口没有新数据也能退出。
                packet = connection.recv_match(
                    type=["HEARTBEAT", "ATTITUDE"], blocking=True, timeout=0.3
                )
                if packet is None:
                    if time.monotonic() - last > 5:
                        self.message.emit(
                            "5 秒未收到姿态/心跳：检查端口、波特率、MAVLink 输出及 QGC 是否占用串口"
                        )
                        last = time.monotonic()
                    continue
                last = time.monotonic()
                common = dict(
                    schema=1,
                    source="FC_USB",
                    system_id=packet.get_srcSystem(),
                    component_id=packet.get_srcComponent(),
                    host_rx_monotonic_us=int(last * 1e6),
                )
                # 电脑接收时刻和飞控启动时刻属于两个时钟；保留 time_basis，
                # 使后续消费者能辨认时间来源，而不会把两者直接相减。
                if packet.get_type() == "ATTITUDE":
                    import math

                    common.update(
                        type="attitude",
                        source_time_us=int(packet.time_boot_ms) * 1000,
                        time_basis="fc_boot",
                        roll_deg=math.degrees(packet.roll),
                        pitch_deg=math.degrees(packet.pitch),
                        yaw_deg=math.degrees(packet.yaw),
                        rollspeed=packet.rollspeed,
                        pitchspeed=packet.pitchspeed,
                        yawspeed=packet.yawspeed,
                    )
                else:
                    common.update(
                        type="heartbeat",
                        autopilot=packet.autopilot,
                        armed=bool(packet.base_mode & 128),
                    )
                self.record.emit(validate_record(common))
        except Exception as exc:
            self.failure.emit(f"飞控连接失败：{exc}")
        finally:
            if connection:
                connection.close()


def sensor_arguments(config):
    """@brief 校验 Nano 设备配置并生成采集程序参数列表。
    @param config sensors 含总线和 7 位地址；hz 为帧率，tty 非空时添加飞控串口参数。
    @return 尚未进行 shell 引号拼接的字符串列表。
    @note 同总线地址不可重复，8×8 频率限定 1–15 Hz；vehicle 为 0–245，输入不合要求抛出 ValueError。
    """
    sensors = config.get("sensors", [])
    if not 1 <= len(sensors) <= 2:
        raise ValueError("请选择 1 或 2 颗传感器")
    args, seen = [], set()
    vehicle = config.get("vehicle", 0)
    if type(vehicle) is not int or not 0 <= vehicle <= 245:
        raise ValueError("无人机编号应为 0–245")
    args += ["--vehicle", str(vehicle)]
    for sensor in sensors:
        bus = sensor["bus"]
        if not re.fullmatch(r"/dev/i2c-[0-9]+", bus):
            raise ValueError("I²C 路径必须形如 /dev/i2c-1，按 Nano 检查结果填写")
        address = int(sensor["address"], 0)
        if not 0x08 <= address <= 0x77:
            raise ValueError("请填写 7 位 I²C 地址，例如 0x29")
        if (bus, address) in seen:
            raise ValueError(
                "同总线两颗传感器不能同地址。请用独立总线，或先在硬件端完成 LPn 地址分配。"
            )
        seen.add((bus, address))
        args += ["--sensor", f"{bus}:{address:#x}"]
    hz = int(config.get("hz", 10))
    if not 1 <= hz <= 15:
        raise ValueError("8×8 模式频率应为 1–15 Hz")
    args += ["--hz", str(hz)]
    tty = config.get("tty", "").strip()
    if tty:
        if not re.fullmatch(r"/dev/tty[A-Za-z0-9]+", tty):
            raise ValueError("飞控串口路径格式错误")
        baud = int(config.get("baud", 115200))
        if baud not in (57600, 115200, 230400, 460800, 921600):
            raise ValueError("飞控波特率不受支持")
        args += ["--mavlink", tty, "--baud", str(baud)]
    return args


class InteractiveHostKey(paramiko.MissingHostKeyPolicy):
    """@brief 将 Paramiko 首次主机密钥确认转交界面。
    @note 工作线程等待事件，界面通过 NanoWorker.answer_key 回答；等待期间仍响应停止请求。
    """
    def __init__(self, owner):
        """@param owner 持有 host_key 信号、确认事件和 known_hosts 路径的 NanoWorker。
        """
        self.owner = owner

    def missing_host_key(self, client, hostname, key):
        """@brief 计算 SHA256 指纹，经确认后保存到项目 known_hosts。
        @param client 当前 SSH 客户端。
        @param hostname 待确认的主机名称。
        @param key 服务器提供的主机公钥。
        @note 拒绝或取消会抛出异常，使连接终止。
        """
        fingerprint = (
            base64.b64encode(hashlib.sha256(key.asbytes()).digest())
            .decode()
            .rstrip("=")
        )
        self.owner.key_decision = None
        # 先清除上一次回答再发信号；事件只承担唤醒，信任决定单独保存。
        self.owner.key_event.clear()
        self.owner.host_key.emit(hostname, "SHA256:" + fingerprint)
        while not self.owner.key_event.wait(0.2):
            if self.owner.stop_event.is_set():
                raise RuntimeError("连接已取消")
        if not self.owner.key_decision:
            raise RuntimeError("未信任该 Nano 主机密钥")
        client.get_host_keys().add(hostname, key.get_name(), key)
        client.save_host_keys(str(self.owner.known_hosts))


class NanoWorker(Worker):
    """@brief 执行 Nano 检查、部署或持续采集的 SSH 工作线程。
    @note host_key 请求界面确认；inspected 提供架构和设备列表；记录沿用 Worker.record。
    """
    host_key = Signal(str, str)
    inspected = Signal(dict)

    def __init__(self, root, settings, password, action, config=None):
        """@param root 项目目录，提供本地二进制和主机密钥文件。
        @param settings host、port、user 连接配置。
        @param password 本次连接密码，连接后及退出时清空线程持有的引用。
        @param action inspect 检查，deploy 上传，stream 上传并采集。
        @param config 传感器和可选飞控串口配置。
        """
        super().__init__()
        self.root = Path(root)
        self.settings, self.password, self.action = settings, password, action
        self.config = config or {}
        self.known_hosts = self.root / "config" / "known_hosts"
        self.key_event = threading.Event()
        self.key_decision = None

    def answer_key(self, decision):
        """@brief 写入界面的信任决定，再唤醒等待密钥确认的工作线程。
        @param decision 是否信任此主机公钥。
        """
        self.key_decision = decision
        self.key_event.set()

    def run(self):
        """@brief 连接、检查平台、上传校验并按行接收采集输出。
        @note stdout 为 PID 标记和 JSONL，stderr 为诊断文本；退出时尝试终止本次远程进程并关闭 SSH。
        """
        client = paramiko.SSHClient()
        channel, pid = None, None
        try:
            self.known_hosts.parent.mkdir(exist_ok=True)
            if self.known_hosts.exists():
                client.load_host_keys(str(self.known_hosts))
            client.set_missing_host_key_policy(InteractiveHostKey(self))
            client.connect(
                self.settings["host"].strip(),
                port=int(self.settings["port"]),
                username=self.settings["user"].strip(),
                password=self.password or None,
                timeout=8,
                auth_timeout=10,
                banner_timeout=10,
                look_for_keys=False,
                allow_agent=False,
            )
            self.password = ""
            client.get_transport().set_keepalive(10)
            self.message.emit("SSH 连接成功")
            if self.stop_event.is_set():
                return
            _, stdout, stderr = client.exec_command(
                "uname -m; cat /etc/os-release; ls /dev/i2c-* /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null",
                timeout=8,
            )
            info = stdout.read().decode("utf-8", "replace")
            arch = info.splitlines()[0] if info else ""
            self.message.emit(info)
            self.inspected.emit(
                {
                    "arch": arch,
                    "devices": re.findall(r"/dev/(?:i2c-[0-9]+|tty[A-Za-z0-9]+)", info),
                }
            )
            if self.action == "inspect":
                return
            if arch != "riscv64":
                raise RuntimeError(
                    f"预编译程序适用于 RISC-V 官方镜像；检测到 {arch}，请核对系统版本"
                )
            binary = self.root / "build" / "nano-riscv64" / "drone-nano"
            if not binary.exists():
                raise RuntimeError("缺少 Nano 可执行文件，请在工具页面重新编译")
            remote = "/tmp/drone-lab/drone-nano"
            # 1. 先上传到 .new，上传过程中不覆盖可执行文件；完整上传后再改名。
            _, stdout, stderr = client.exec_command(
                "mkdir -p /tmp/drone-lab", timeout=8
            )
            if stdout.channel.recv_exit_status() != 0:
                raise RuntimeError(stderr.read().decode("utf-8", "replace"))
            with client.open_sftp() as sftp:
                sftp.get_channel().settimeout(15)
                sftp.put(str(binary), remote + ".new")
                sftp.chmod(remote + ".new", 0o755)
            _, stdout, stderr = client.exec_command(
                "mv /tmp/drone-lab/drone-nano.new /tmp/drone-lab/drone-nano; sha256sum /tmp/drone-lab/drone-nano",
                timeout=8,
            )
            checksum = stdout.read().decode().split()
            # 2. 比较远端和本地文件摘要，确认传输内容一致后才允许开始采集。
            expected = hashlib.sha256(binary.read_bytes()).hexdigest()
            if not checksum or checksum[0] != expected:
                raise RuntimeError("上传后 SHA256 校验失败")
            self.message.emit(
                "采集程序上传并校验成功：/tmp/drone-lab/drone-nano（重启后可能需重新部署）"
            )
            if self.action == "deploy":
                return
            if self.stop_event.is_set():
                return
            args = sensor_arguments(self.config)
            # 3. shell 先报告自身 PID，exec 用采集程序替换 shell，保持 PID 不变；
            #    退出清理便能终止本次进程。shlex.join 为各参数添加远端 shell 引号。
            cmd = "echo DRONELAB_PID:$$; exec " + shlex.join([remote] + args)
            channel = client.get_transport().open_session(timeout=8)
            channel.exec_command(cmd)
            pending, errors = b"", b""
            # SSH 一次 recv 可能包含半行或多行。两个字节缓冲分别累积 stdout/stderr，
            # 只有完整换行记录才解析，跨 recv 的半行留到下一轮，避免拆坏 UTF-8 或 JSON。
            newline = bytes([10])
            last_received = time.monotonic()
            while not self.stop_event.is_set():
                if not client.get_transport().is_active():
                    raise RuntimeError("SSH 连接已断开")
                if channel.recv_ready():
                    pending += channel.recv(32768)
                    last_received = time.monotonic()
                    if len(pending) > 262144:
                        raise RuntimeError("Nano 数据帧过大或缺少换行")
                    while newline in pending:
                        line, pending = pending.split(newline, 1)
                        if line.startswith(b"DRONELAB_PID:"):
                            pid = int(line.split(b":")[1])
                            continue
                        if line.strip():
                            self.record.emit(validate_record(json.loads(line)))
                if channel.recv_stderr_ready():
                    errors += channel.recv_stderr(16384)
                    last_received = time.monotonic()
                    while newline in errors:
                        line, errors = errors.split(newline, 1)
                        self.message.emit(line.decode("utf-8", "replace"))
                if (
                    # 先排空两个输出通道再读取退出码，保留进程结束前最后的诊断。
                    channel.exit_status_ready()
                    and not channel.recv_ready()
                    and not channel.recv_stderr_ready()
                ):
                    code = channel.recv_exit_status()
                    if errors:
                        self.message.emit(errors.decode("utf-8", "replace"))
                    if code != 0:
                        raise RuntimeError(
                            f"Nano 采集退出，状态码 {code}；检查上方错误"
                        )
                    break
                if time.monotonic() - last_received > 15:
                    raise RuntimeError(
                        "15 秒未收到 Nano 输出，采集已停止；检查网络、驱动和设备连接"
                    )
                time.sleep(0.01)
        except Exception as exc:
            self.failure.emit(f"Nano 操作失败：{exc}")
        finally:
            self.password = ""
            # 即使解析失败也执行清理；远程终止失败不妨碍关闭本地通道和 SSH 客户端。
            if channel is not None:
                if pid is not None and not channel.exit_status_ready():
                    try:
                        _, out, _ = client.exec_command(
                            f"kill -TERM {int(pid)}", timeout=3
                        )
                        out.channel.recv_exit_status()
                    except Exception:
                        pass
                channel.close()
            client.close()
