"""Versioned records for live acquisition, demonstration and replay."""

import json
import math
import time
from pathlib import Path

VALID_STATUS = {5, 9}


def validate_record(item):
    if not isinstance(item, dict) or item.get("schema") != 1:
        raise ValueError("记录格式错误：需要 schema=1 的 JSON 对象")
    kind = item.get("type")
    if kind not in ("tof", "attitude", "heartbeat"):
        raise ValueError(f"不支持的数据类型：{kind}")
    if kind == "tof":
        if item.get("sensor") not in (0, 1):
            raise ValueError("传感器编号应为 0 或 1")
        for key in ("distance_mm", "status"):
            values = item.get(key)
            if not isinstance(values, list) or len(values) != 64:
                raise ValueError(f"{key} 必须包含 64 个区域")
            if any(
                not isinstance(v, (float, int)) or not math.isfinite(v) for v in values
            ):
                raise ValueError(f"{key} 包含非法数值")
        if any(v < 0 or v > 255 or int(v) != v for v in item["status"]):
            raise ValueError("测距状态应为 0–255 的整数")
    elif kind == "attitude":
        for key in ("roll_deg", "pitch_deg", "yaw_deg"):
            if not isinstance(item.get(key), (int, float)) or not math.isfinite(
                item[key]
            ):
                raise ValueError(f"姿态 {key} 无效")
    return item


def valid_distances(item):
    return [
        d
        for d, s in zip(item["distance_mm"], item["status"])
        if s in VALID_STATUS and d > 0
    ]


def demonstration(t, sequence=0):
    """UI sample data, not a physics or flight simulation."""
    records = []
    for sensor in (0, 1):
        base = 1450 + 450 * math.sin(t * 0.6 + sensor)
        distances, statuses = [], []
        for zone in range(64):
            row, col = divmod(zone, 8)
            distances.append(
                round(base + (col - 3.5) * 48 * math.sin(t * 0.3) + (row - 3.5) * 12)
            )
            statuses.append(0 if (zone + sequence + sensor * 7) % 53 == 0 else 5)
        records.append(
            dict(
                schema=1,
                type="tof",
                sensor=sensor,
                sequence=sequence,
                source="DEMO",
                source_time_us=int(t * 1e6),
                time_basis="synthetic",
                distance_mm=distances,
                status=statuses,
            )
        )
    records.append(
        dict(
            schema=1,
            type="attitude",
            source="DEMO",
            source_time_us=int(t * 1e6),
            time_basis="synthetic",
            roll_deg=12 * math.sin(t * 0.8),
            pitch_deg=8 * math.cos(t * 0.5),
            yaw_deg=(t * 8) % 360,
        )
    )
    return records


class Recorder:
    def __init__(self, path):
        self.path = Path(path)
        self.handle = self.path.open("x", encoding="utf-8")
        self.start = time.monotonic()
        self.count = 0

    def write(self, item):
        value = validate_record(dict(item))
        value["record_time_s"] = time.monotonic() - self.start
        value["received_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self.handle.write(
            json.dumps(value, ensure_ascii=False, allow_nan=False) + chr(10)
        )
        self.handle.flush()
        self.count += 1

    def close(self):
        self.handle.close()


def load_recording(path, max_bytes=128 * 1024 * 1024):
    path = Path(path)
    if path.stat().st_size > max_bytes:
        raise ValueError("本版回放支持 128 MB 以内的日志，请分段录制")
    records, previous = [], 0.0
    with path.open(encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            if not line.strip():
                continue
            try:
                item = validate_record(json.loads(line))
                stamp = float(item["record_time_s"])
                if not math.isfinite(stamp) or stamp < previous or stamp < 0:
                    raise ValueError("回放时间必须非负且单调递增")
                previous = stamp
                records.append(item)
            except (ValueError, KeyError, TypeError) as exc:
                raise ValueError(f"日志第 {line_no} 行：{exc}") from exc
    if not records:
        raise ValueError("日志为空")
    start = records[0]["record_time_s"]
    return [(r["record_time_s"] - start, r) for r in records]
