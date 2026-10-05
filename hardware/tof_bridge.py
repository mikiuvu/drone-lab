#!/usr/bin/env python3
"""Nano JSONL to companion UDP bridge with optional MAVLink serial transport.

The Nano timestamp is retained as provenance. It is never used as a flight
clock because Nano and the flight controller have no shared epoch here.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional

import serial


TOF_CELLS = 64
TIMESYNC_MSG_ID = 111
TIMESYNC_CRC_EXTRA = 34
MAVLINK_V1_MAGIC = 0xFE
MAVLINK_V2_MAGIC = 0xFD


def monotonic_ns() -> int:
    return time.monotonic_ns()


def x25_crc(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        tmp = value ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def mavlink_frame_length(buffer: bytearray) -> Optional[int]:
    if not buffer:
        return None
    if buffer[0] == MAVLINK_V1_MAGIC:
        if len(buffer) < 2:
            return None
        return 8 + buffer[1]
    if buffer[0] == MAVLINK_V2_MAGIC:
        if len(buffer) < 3:
            return None
        signature = 13 if buffer[2] & 0x01 else 0
        return 12 + buffer[1] + signature
    del buffer[0]
    return None


def decode_frame(frame: bytes) -> Optional[dict[str, Any]]:
    """Check framing, validate TIMESYNC CRC, and decode TIMESYNC when present.

    Unknown message CRC extras are not available in this bridge, so complete
    unknown frames are retained for transparent serial forwarding.
    """
    if not frame or frame[0] not in (MAVLINK_V1_MAGIC, MAVLINK_V2_MAGIC):
        return None
    if frame[0] == MAVLINK_V1_MAGIC:
        if len(frame) < 8 or len(frame) != 8 + frame[1]:
            return None
        payload_len = frame[1]
        header = frame[1:6]
        msg_id = frame[5]
        payload = frame[6 : 6 + payload_len]
        crc_at = 6 + payload_len
    else:
        if len(frame) < 12 or len(frame) != 12 + frame[1] + (13 if frame[2] & 1 else 0):
            return None
        payload_len = frame[1]
        header = frame[1:10]
        msg_id = int.from_bytes(frame[7:10], "little")
        payload = frame[10 : 10 + payload_len]
        crc_at = 10 + payload_len
    result: dict[str, Any] = {"msgid": msg_id, "payload": payload}
    if msg_id == TIMESYNC_MSG_ID:
        if x25_crc(header + payload + bytes([TIMESYNC_CRC_EXTRA])) != int.from_bytes(
            frame[crc_at : crc_at + 2], "little"
        ):
            return None
    if msg_id == TIMESYNC_MSG_ID and len(payload) >= 18:
        tc1, ts1, target_system, target_component = struct.unpack_from("<qqBB", payload)
        result.update(
            tc1_ns=tc1,
            ts1_ns=ts1,
            target_system=target_system,
            target_component=target_component,
        )
    return result


def make_timesync_request(system_id: int, component_id: int, timestamp_ns: int) -> bytes:
    """Build a MAVLink 2 TIMESYNC request using the generated message contract."""
    payload = struct.pack("<qqBB", 0, timestamp_ns, 0, 0)
    sequence = make_timesync_request.sequence & 0xFF
    make_timesync_request.sequence += 1
    header = struct.pack("<BBBBBB", len(payload), 0, 0, sequence, system_id, component_id)
    header += TIMESYNC_MSG_ID.to_bytes(3, "little")
    crc = x25_crc(header + payload + bytes([TIMESYNC_CRC_EXTRA]))
    return bytes([MAVLINK_V2_MAGIC]) + header + payload + struct.pack("<H", crc)


make_timesync_request.sequence = 0


def make_timesync_response(
    system_id: int,
    component_id: int,
    target_system: int,
    target_component: int,
    remote_request_ts1_ns: int,
    bridge_now_ns: int,
) -> bytes:
    payload = struct.pack(
        "<qqBB", bridge_now_ns, remote_request_ts1_ns, target_system, target_component
    )
    sequence = make_timesync_response.sequence & 0xFF
    make_timesync_response.sequence += 1
    header = struct.pack("<BBBBBB", len(payload), 0, 0, sequence, system_id, component_id)
    header += TIMESYNC_MSG_ID.to_bytes(3, "little")
    crc = x25_crc(header + payload + bytes([TIMESYNC_CRC_EXTRA]))
    return bytes([MAVLINK_V2_MAGIC]) + header + payload + struct.pack("<H", crc)


make_timesync_response.sequence = 0


@dataclass
class TimeSample:
    offset_ns: int
    rtt_ns: int
    received_ns: int


@dataclass
class ClockModel:
    mode: str
    max_rtt_us: int = 20_000
    stale_s: float = 5.0
    receive_error_us: int = 2_000
    samples: list[TimeSample] = field(default_factory=list)

    def add_timesync_response(self, sent_ns: int, received_ns: int, remote_ns: int) -> bool:
        rtt_ns = received_ns - sent_ns
        if rtt_ns <= 0 or rtt_ns > self.max_rtt_us * 1000:
            return False
        midpoint_ns = sent_ns + rtt_ns // 2
        self.samples.append(TimeSample(remote_ns - midpoint_ns, rtt_ns, received_ns))
        self.samples = self.samples[-16:]
        return True

    def _best(self, now_ns: int) -> Optional[TimeSample]:
        fresh = [s for s in self.samples if now_ns - s.received_ns <= self.stale_s * 1e9]
        return min(fresh, key=lambda s: s.rtt_ns) if fresh else None

    def stamp(self, rx_ns: int) -> Optional[dict[str, Any]]:
        if self.mode in ("monotonic", "receive"):
            return {
                "sim_time_us": max(1, rx_ns // 1000),
                "time_basis": "bridge_monotonic_rx",
                "clock_offset_us": 0,
                "clock_error_us": 0,
                "time_error_limit_us": self.receive_error_us,
            }
        sample = self._best(rx_ns)
        if sample is None:
            return None
        offset_us = round(sample.offset_ns / 1000)
        sync_error_us = (sample.rtt_ns + 1999) // 2000
        return {
            "sim_time_us": max(1, (rx_ns + sample.offset_ns) // 1000),
            "time_basis": "fc_boot_timesync",
            "clock_offset_us": offset_us,
            "clock_error_us": sync_error_us,
            "time_error_limit_us": sync_error_us + self.receive_error_us,
            "timesync_rtt_us": (sample.rtt_ns + 999) // 1000,
        }


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def convert_nano_record(
    record: dict[str, Any],
    vehicle: int,
    sensor_count: int,
    allowed_status: set[int],
    rx_ns: int,
    clock: ClockModel,
) -> Optional[dict[str, Any]]:
    if record.get("schema") != 1 or record.get("type") != "tof":
        return None
    if record.get("source") not in (None, "NANO"):
        return None
    sensor = record.get("sensor")
    source_time_us = record.get("source_time_us")
    distances = record.get("distance_mm")
    statuses = record.get("status")
    if not isinstance(sensor, int) or isinstance(sensor, bool) or not 0 <= sensor < sensor_count:
        return None
    if not isinstance(source_time_us, int) or isinstance(source_time_us, bool) or source_time_us < 0:
        return None
    if not isinstance(distances, list) or len(distances) != TOF_CELLS:
        return None
    if not isinstance(statuses, list) or len(statuses) != TOF_CELLS:
        return None
    if any(not isinstance(status, int) or isinstance(status, bool) for status in statuses):
        return None
    if any(not _finite_number(distance) for distance in distances):
        return None
    stamp = clock.stamp(rx_ns)
    if stamp is None:
        return None
    distance_m = [float(distance) * 0.001 for distance in distances]
    valid = [
        int(status in allowed_status and float(distance) > 0.0)
        for distance, status in zip(distances, statuses)
    ]
    return {
        "schema": 1,
        "kind": "tof",
        "vehicle": vehicle,
        "sensor": sensor,
        **stamp,
        "distance_m": distance_m,
        "valid": valid,
        "status": statuses,
        "source": "NANO_TOF_BRIDGE",
        "source_time_us": source_time_us,
        "source_time_basis": record.get("time_basis", "nano_data_ready_poll"),
        "bridge_rx_monotonic_us": rx_ns // 1000,
    }


class MavlinkParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> Iterable[tuple[bytes, Optional[dict[str, Any]]]]:
        self.buffer.extend(data)
        while self.buffer:
            if self.buffer[0] not in (MAVLINK_V1_MAGIC, MAVLINK_V2_MAGIC):
                del self.buffer[0]
                continue
            length = mavlink_frame_length(self.buffer)
            if length is None or len(self.buffer) < length:
                return
            frame = bytes(self.buffer[:length])
            del self.buffer[:length]
            decoded = decode_frame(frame)
            if decoded is not None:
                yield frame, decoded


class SerialLock:
    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self.handle: Any = None

    def __enter__(self) -> "SerialLock":
        self.handle = open(self.path, "a+b")
        if os.name == "nt":
            import msvcrt

            self.handle.seek(0)
            try:
                msvcrt.locking(self.handle.fileno(), msvcrt.LK_NBLCK, 1)
            except OSError as exc:
                self.handle.close()
                raise RuntimeError(f"serial lock busy: {self.path}") from exc
        else:
            import fcntl

            try:
                fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as exc:
                self.handle.close()
                raise RuntimeError(f"serial lock busy: {self.path}") from exc
        return self

    def __exit__(self, *_: Any) -> None:
        if self.handle is not None:
            if os.name == "nt":
                import msvcrt

                self.handle.seek(0)
                try:
                    msvcrt.locking(self.handle.fileno(), msvcrt.LK_UNLCK, 1)
                except OSError:
                    pass
            else:
                import fcntl

                fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
            self.handle.close()


class Bridge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.stop = threading.Event()
        self.lines: queue.Queue[tuple[bytes, int]] = queue.Queue(maxsize=256)
        self.clock = ClockModel(
            args.time_mode,
            args.timesync_max_rtt_us,
            args.timesync_stale_s,
            args.receive_time_error_us,
        )
        self.pending: dict[int, int] = {}
        self.serial_port: Optional[serial.Serial] = None
        self.mav_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mav_udp.bind((args.mavlink_udp_listen_host, args.mavlink_udp_listen_port))
        self.tof_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0

    def close(self) -> None:
        self.stop.set()
        self.mav_udp.close()
        self.tof_udp.close()
        if self.serial_port is not None:
            self.serial_port.close()

    def read_jsonl(self) -> None:
        for line in sys.stdin.buffer:
            if self.stop.is_set():
                break
            if not line.strip():
                continue
            try:
                self.lines.put((line, monotonic_ns()), timeout=0.2)
            except queue.Full:
                print("Nano JSONL queue full; dropping record", file=sys.stderr)
        self.stop.set()

    def process_jsonl(self) -> None:
        while not self.stop.is_set() or not self.lines.empty():
            try:
                line, rx_ns = self.lines.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                record = json.loads(line.decode("utf-8"))
                output = convert_nano_record(
                    record,
                    self.args.vehicle,
                    self.args.sensor_count,
                    set(self.args.allowed_status),
                    rx_ns,
                    self.clock,
                )
                if output is None:
                    if self.args.time_mode == "timesync" and self.clock.stamp(rx_ns) is None:
                        print("waiting for valid MAVLink TIMESYNC; ToF record withheld", file=sys.stderr)
                    else:
                        print("rejected Nano ToF JSONL record", file=sys.stderr)
                    continue
                output["sequence"] = self.sequence
                self.sequence += 1
                payload = json.dumps(output, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                self.tof_udp.sendto(payload, (self.args.tof_udp_host, self.args.tof_udp_port))
                if self.args.mirror_jsonl:
                    with open(self.args.mirror_jsonl, "ab") as mirror:
                        mirror.write(payload + b"\n")
            except (UnicodeDecodeError, json.JSONDecodeError, OSError, TypeError, ValueError) as exc:
                print(f"Nano JSONL error: {exc}", file=sys.stderr)

    def serial_loop(self) -> None:
        if not self.args.mavlink_serial:
            return
        lock_path = self.args.serial_lock or self.args.mavlink_serial + ".lock"
        try:
            with SerialLock(lock_path):
                self.serial_port = serial.Serial(
                    self.args.mavlink_serial,
                    self.args.mavlink_baud,
                    timeout=0.05,
                    exclusive=(os.name != "nt"),
                )
                parser = MavlinkParser()
                next_sync = 0.0
                while not self.stop.is_set():
                    now = time.monotonic()
                    if self.args.time_mode == "timesync" and now >= next_sync:
                        stamp = monotonic_ns()
                        self.pending[stamp] = stamp
                        self.serial_port.write(make_timesync_request(200, 191, stamp))
                        next_sync = now + self.args.timesync_period_s
                    data = self.serial_port.read(4096)
                    if data:
                        for frame, decoded in parser.feed(data):
                            self.mav_udp.sendto(frame, self.args.mavlink_udp_target)
                            if decoded and decoded.get("msgid") == TIMESYNC_MSG_ID:
                                self.handle_timesync(decoded)
                    self.forward_udp_to_serial()
        except (OSError, serial.SerialException, RuntimeError) as exc:
            print(f"MAVLink serial transport stopped: {exc}", file=sys.stderr)
            self.stop.set()

    def handle_timesync(self, decoded: dict[str, Any]) -> None:
        tc1 = decoded.get("tc1_ns")
        ts1 = decoded.get("ts1_ns")
        if not isinstance(tc1, int) or not isinstance(ts1, int):
            return
        if tc1 == 0:
            if self.serial_port is not None:
                response = make_timesync_response(
                    200,
                    191,
                    decoded.get("target_system", 0),
                    decoded.get("target_component", 0),
                    ts1,
                    monotonic_ns(),
                )
                self.serial_port.write(response)
            return
        sent = self.pending.pop(ts1, None)
        if sent is not None:
            received = monotonic_ns()
            if self.clock.add_timesync_response(sent, received, tc1):
                sample = self.clock.samples[-1]
                print(
                    f"TIMESYNC valid offset_us={sample.offset_ns / 1000:.0f} "
                    f"rtt_us={sample.rtt_ns / 1000:.0f}",
                    file=sys.stderr,
                )

    def forward_udp_to_serial(self) -> None:
        if self.serial_port is None:
            return
        self.mav_udp.settimeout(0.0)
        for _ in range(64):
            try:
                payload, _ = self.mav_udp.recvfrom(65535)
            except BlockingIOError:
                return
            if payload:
                self.serial_port.write(payload)

    def run(self) -> int:
        json_thread = threading.Thread(target=self.read_jsonl, daemon=True)
        json_thread.start()
        processor = threading.Thread(target=self.process_jsonl, daemon=True)
        processor.start()
        serial_thread = threading.Thread(target=self.serial_loop, daemon=True)
        serial_thread.start()
        try:
            while not self.stop.is_set():
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()
        return 0


def load_config(path: Optional[str]) -> dict[str, Any]:
    if not path:
        return {}
    with open(path, encoding="utf-8") as source:
        value = json.load(source)
    if not isinstance(value, dict):
        raise ValueError("bridge config must be a JSON object")
    return value


def build_parser(config: dict[str, Any]) -> argparse.ArgumentParser:
    serial_cfg = config.get("serial", {})
    parser = argparse.ArgumentParser(description="Nano JSONL and MAVLink serial/UDP bridge")
    parser.add_argument("--config")
    parser.add_argument("--vehicle", type=int, default=config.get("vehicle", 0))
    parser.add_argument("--sensor-count", type=int, default=config.get("sensor_count", 3))
    parser.add_argument("--allowed-status", type=int, nargs="+", default=config.get("allowed_status", [5, 9]))
    parser.add_argument("--tof-udp-host", default=config.get("tof_udp_host", "127.0.0.1"))
    parser.add_argument("--tof-udp-port", type=int, default=config.get("tof_udp_port", 16000))
    parser.add_argument("--time-mode", choices=("monotonic", "receive", "timesync"), default=config.get("time_mode", "timesync"))
    parser.add_argument("--receive-time-error-us", type=int, default=config.get("receive_time_error_us", 2000))
    parser.add_argument("--timesync-period-s", type=float, default=config.get("timesync_period_s", 1.0))
    parser.add_argument("--timesync-max-rtt-us", type=int, default=config.get("timesync_max_rtt_us", 20000))
    parser.add_argument("--timesync-stale-s", type=float, default=config.get("timesync_stale_s", 5.0))
    parser.add_argument("--mavlink-serial", default=serial_cfg.get("port"))
    parser.add_argument("--mavlink-baud", type=int, default=serial_cfg.get("baud", 115200))
    parser.add_argument("--serial-lock", default=config.get("serial_lock"))
    parser.add_argument("--mavlink-udp-listen-host", default=config.get("mavlink_udp_listen_host", "127.0.0.1"))
    parser.add_argument("--mavlink-udp-listen-port", type=int, default=config.get("mavlink_udp_listen_port", 14580))
    parser.add_argument("--mavlink-udp-target-host", default=config.get("mavlink_udp_target_host", "127.0.0.1"))
    parser.add_argument("--mavlink-udp-target-port", type=int, default=config.get("mavlink_udp_target_port", 14540))
    parser.add_argument("--mirror-jsonl")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    preliminary = argparse.ArgumentParser(add_help=False)
    preliminary.add_argument("--config")
    known, _ = preliminary.parse_known_args(argv)
    config = load_config(known.config)
    args = build_parser(config).parse_args(argv)
    if not 0 <= args.vehicle <= 255 or args.sensor_count < 1 or args.sensor_count > 3:
        raise SystemExit("vehicle must be 0..255 and sensor-count must be 1..3")
    if args.time_mode == "timesync" and not args.mavlink_serial:
        raise SystemExit("--time-mode timesync requires --mavlink-serial")
    if args.receive_time_error_us < 0 or args.timesync_max_rtt_us <= 0 or args.timesync_stale_s <= 0:
        raise SystemExit("invalid time error or TIMESYNC limit")
    return Bridge(args).run()


if __name__ == "__main__":
    raise SystemExit(main())
