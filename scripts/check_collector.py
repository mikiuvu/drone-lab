"""Native WSL collector fault injection; no board, packages, or device writes."""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
WSL = ["wsl", "-d", "Ubuntu-22.04", "--exec"]


def run(args, **kwargs):
    result = subprocess.run(WSL + args, capture_output=True, timeout=60, **kwargs)
    if result.returncode:
        raise RuntimeError(f"{args}: {result.returncode}\n{result.stdout.decode('utf-8', 'replace')}\n{result.stderr.decode('utf-8', 'replace')}")
    return result.stdout


def linux_tests(root):
    # Executed by existing WSL Python, with argv preserving workspace spaces.
    import fcntl
    import pty
    import select
    import termios
    import time

    binary = root + "/build/native/native-collector"
    sensor = ["--sensor", "/dev/i2c-0:0x29"]

    def case(mode, args=None, code=0, init=None, stop=None):
        proc = subprocess.run([binary, mode] + (sensor if args is None else args), capture_output=True, timeout=8)
        assert proc.returncode == code, (mode, proc.returncode, proc.stderr)
        if init is not None:
            assert f"init={init} ".encode() in proc.stderr, proc.stderr
        if stop is not None:
            assert f"stop={stop} ".encode() in proc.stderr, proc.stderr
        return proc

    unit = case("units", args=[])
    maximum = json.loads(unit.stdout)
    assert len(maximum["distance_mm"]) == 64
    print(f"PASS maximal C storage JSON line: {len(unit.stdout)} bytes (<4096-byte Linux pipe atomic limit)")
    case("good", args=sensor + ["--sensor", "/dev/i2c-1:0x29"], init=2, stop=2)
    case("slow-init", args=sensor + ["--sensor", "/dev/i2c-1:0x29"], init=2, stop=2)
    case("init-fail", code=1, init=1, stop=0)
    case("init-stop", args=sensor + ["--sensor", "/dev/i2c-1:0x29"], code=1, init=1, stop=0)
    case("start-fail", code=1, stop=1)
    case("ready-fail", code=4, stop=1)
    bad_read = case("read-fail", code=4, stop=1)
    assert not bad_read.stdout
    case("stop-fail", code=4, stop=1)
    for flag, value in [("--hz", "10junk"), ("--hz", "-1"), ("--hz", "4294967306"),
                        ("--baud", "115200x"), ("--baud", "9600"), ("--mavlink", ""),
                        ("--mavlink", "relative/path")]:
        case("good", args=sensor + [flag, value], code=2, init=0)
    case("good", args=sensor + sensor, code=2, init=0)
    case("good", args=["--sensor", "/dev/i2c-0:0x100000029"], code=2, init=0)
    case("good", args=sensor + ["--mavlink", "/dev/drone-test-no-such-tty"], code=5, init=0)
    fault_master, fault_slave = pty.openpty()
    try:
        case("read-error", args=sensor + ["--mavlink", os.ttyname(fault_slave)], code=5, stop=1)
    finally:
        os.close(fault_master);os.close(fault_slave)

    # An unread small pipe must fail instead of hanging or dropping a partial line.
    read_fd, write_fd = os.pipe()
    fcntl.fcntl(write_fd, fcntl.F_SETPIPE_SZ, 4096)
    proc = subprocess.Popen([binary, "output"] + sensor, stdout=write_fd, stderr=subprocess.PIPE)
    os.close(write_fd)
    _, error = proc.communicate(timeout=8)
    assert proc.returncode == 6, (proc.returncode, error)
    raw = os.read(read_fd, 65536)
    os.close(read_fd)
    assert raw and raw.endswith(b"\n")
    for line in raw.splitlines():
        json.loads(line)
    assert b"stop=1" in error

    # Broken output pipe also propagates failure, despite SIGPIPE handling.
    read_fd, write_fd = os.pipe()
    os.close(read_fd)
    proc = subprocess.Popen([binary, "output"] + sensor, stdout=write_fd, stderr=subprocess.PIPE)
    os.close(write_fd)
    _, error = proc.communicate(timeout=8)
    assert proc.returncode == 6 and b"stop=1" in error, error

    def receive_line(proc):
        assert select.select([proc.stdout], [], [], 3)[0], "UART JSON timed out"
        return json.loads(proc.stdout.readline())

    master, slave = pty.openpty()
    path = os.ttyname(slave)
    proc = subprocess.Popen([binary, "uart"] + sensor + ["--mavlink", path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for _ in range(100):
            if not termios.tcgetattr(slave)[3] & termios.ICANON:
                break
            time.sleep(0.01)
        else:
            raise AssertionError("UART was not configured raw")
        # MAVLink v1 ATTITUDE built locally, using standard CRC-extra 39.
        import math
        import struct
        def attitude(sequence, boot, roll=0.1):
            payload = struct.pack("<Iffffff", boot, roll, 0.2, 0.3, 0.0, 0.0, 0.0)
            packet = bytes([len(payload), sequence, 1, 1, 30]) + payload
            crc = 0xffff
            for byte in packet + bytes([39]):
                tmp = byte ^ (crc & 0xff)
                tmp ^= (tmp << 4) & 0xff
                crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
            return b"\xfe" + packet + struct.pack("<H", crc)
        frame = attitude(10, 120000)
        os.write(master, frame[:9])
        time.sleep(0.02)
        os.write(master, frame[9:])
        first = receive_line(proc)
        assert first["source_time_us"] == 120000000 and first["mavlink_sequence"] == 10
        assert first["uart_bytes_total"] == len(frame)
        assert abs(first["roll_deg"] - 5.72958) < 0.0001
        # Malformed CRC and nonfinite angles are consumed but never emitted.
        corrupted = bytearray(attitude(11, 120100)); corrupted[-1] ^= 0xff
        os.write(master, bytes(corrupted) + attitude(12, 120200, math.nan) + attitude(13, 100))
        reset = receive_line(proc)
        assert reset["source_time_us"] == 100000 and reset["mavlink_sequence"] == 13
        assert reset["nano_rx_monotonic_us"] >= first["nano_rx_monotonic_us"]
        assert reset["uart_bytes_total"] == 4 * len(frame)
        assert not select.select([master], [], [], 0)[0], "Collector wrote UART bytes"
        os.close(master); master = -1
        _, error = proc.communicate(timeout=8)
        assert proc.returncode == 5 and b"stop=1" in error, error
    finally:
        if proc.poll() is None:
            proc.kill(); proc.communicate()
        if master >= 0:
            os.close(master)
        os.close(slave)
    print("PASS collector: strict CLI, dual sensor, fault exit codes, init stop, cleanup, atomic full/broken pipe, PTY split/CRC/NaN/boot reset/hangup")


def main():
    root = run(["wslpath", "-a", str(ROOT)]).decode("utf-8").strip()
    run(["mkdir", "-p", root + "/build/native"])
    includes = ["-I" + root + "/nano", "-I" + root + "/vendor/stm32-vl53l5cx/modules",
                "-isystem", root + "/vendor/mavlink"]
    for name, extra in [("collector", [root + "/nano/platform.c", "-lm"]), ("platform", [])]:
        run(["gcc", "-std=c11", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
                      "-fsanitize=undefined", "-fno-sanitize-recover=all"] + includes +
                     [root + f"/tests/native_{name}.c"] + extra + ["-o", root + f"/build/native/native-{name}"])
    print(run([root + "/build/native/native-platform"]).decode("utf-8").strip())
    print(run(["python3", root + "/scripts/check_collector.py", "--linux", root]).decode("utf-8").strip())


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--linux":
        linux_tests(sys.argv[2])
    else:
        main()
