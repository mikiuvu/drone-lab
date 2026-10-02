"""I/O workers keep network and serial operations off the GUI thread."""

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
    record = Signal(dict)
    message = Signal(str)
    failure = Signal(str)

    def __init__(self):
        super().__init__()
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()


class FCWorker(Worker):
    def __init__(self, device, baud):
        super().__init__()
        self.device, self.baud = device, baud

    def run(self):
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
    sensors = config.get("sensors", [])
    if not 1 <= len(sensors) <= 2:
        raise ValueError("请选择 1 或 2 颗传感器")
    args, seen = [], set()
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
    def __init__(self, owner):
        self.owner = owner

    def missing_host_key(self, client, hostname, key):
        fingerprint = (
            base64.b64encode(hashlib.sha256(key.asbytes()).digest())
            .decode()
            .rstrip("=")
        )
        self.owner.key_decision = None
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
    host_key = Signal(str, str)
    inspected = Signal(dict)

    def __init__(self, root, settings, password, action, config=None):
        super().__init__()
        self.root = Path(root)
        self.settings, self.password, self.action = settings, password, action
        self.config = config or {}
        self.known_hosts = self.root / "config" / "known_hosts"
        self.key_event = threading.Event()
        self.key_decision = None

    def answer_key(self, decision):
        self.key_decision = decision
        self.key_event.set()

    def run(self):
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
            cmd = "echo DRONELAB_PID:$$; exec " + shlex.join([remote] + args)
            channel = client.get_transport().open_session(timeout=8)
            channel.exec_command(cmd)
            pending, errors = b"", b""
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
