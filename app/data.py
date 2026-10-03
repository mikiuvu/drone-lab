"""@file data.py
@brief 统一实时采集、演示和回放使用的 schema=1 字典记录。
ToF 距离以毫米表示，姿态角以度表示；设备源时间保留原时间基准，录制时间由电脑单调时钟生成。
"""

import json
import math
import time
from pathlib import Path

VALID_STATUS = {5, 9}  # 本界面接受的 ToF 状态码；统计还需满足距离 > 0 mm。


def validate_record(item):
    """@brief 检查显示和录制所需的最小记录结构。
    @param item schema=1 字典；ToF 含 64 个距离和状态，姿态含三个有限角度。
    @return 原字典对象，不复制、不补齐字段。
    @note 格式错误抛出 ValueError；心跳只检查公共类型，时间和来源字段不在此处验证。
    """
    if not isinstance(item, dict) or item.get("schema") != 1:
        raise ValueError("记录格式错误：需要 schema=1 的 JSON 对象")
    kind = item.get("type")
    if kind not in ("tof", "attitude", "heartbeat"):
        raise ValueError(f"不支持的数据类型：{kind}")
    if kind == "tof":
        # 先保证区域数和数值有限，绘制端才能安全按 8×8 原始索引访问；
        # 有效目标筛选留给 valid_distances，负距离也保留在原始日志中。
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
    """@brief 提取状态为 5 或 9 且距离大于零的有效区域。
    @param item 已校验的 ToF 记录。
    @return 有效距离列表，单位 mm；可能为空。
    """
    return [
        d
        for d, s in zip(item["distance_mm"], item["status"])
        if s in VALID_STATUS and d > 0
    ]


def demonstration(t, sequence=0):
    """@brief 生成两颗 ToF 和一组姿态的界面演示记录。
    @param t 合成时间，单位 s。
    @param sequence 帧序号，也用于移动无效区域的位置。
    @return 三条 schema=1 记录；距离、角度由周期函数构造。
    """
    records = []
    for sensor in (0, 1):
        # 1. 正弦项产生随时间变化的基础距离，两颗传感器使用不同相位。
        base = 1450 + 450 * math.sin(t * 0.6 + sensor)
        distances, statuses = [], []
        for zone in range(64):
            # 2. 区域号拆成行列，以网格中心为零添加横向倾斜和纵向梯度，
            #    并随序号移动无效区域，使热图和有效区域统计都有可见变化。
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
    # 3. 补入度制姿态记录，与测距共用同一个合成时间基准。
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
    """@brief JSONL 逐条录制器，每行保存一个独立 JSON 对象。
    @note 文件由调用方关闭；使用独占创建模式，已有同名文件会报错。
    """
    def __init__(self, path):
        """@param path 新日志路径；父目录须已存在。
        @note start 为电脑单调时钟起点，count 为成功写入并刷新后的记录数。
        """
        self.path = Path(path)
        self.handle = self.path.open("x", encoding="utf-8")
        self.start = time.monotonic()
        self.count = 0

    def write(self, item):
        """@brief 校验记录并添加电脑接收时间后写入。
        @param item 待录制字典；使用浅拷贝，新增字段不会写回调用方字典。
        @note record_time_s 用于回放间隔；received_utc 用于日期定位。每条 flush 将缓冲交给系统，但不是磁盘同步保证。
        """
        value = validate_record(dict(item))
        value["record_time_s"] = time.monotonic() - self.start
        value["received_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self.handle.write(
            json.dumps(value, ensure_ascii=False, allow_nan=False) + chr(10)
        )
        self.handle.flush()
        self.count += 1

    def close(self):
        """@brief 关闭日志句柄；调用后不能继续写入。
        """
        self.handle.close()


def load_recording(path, max_bytes=128 * 1024 * 1024):
    """@brief 校验整份 JSONL 并建立回放时间轴。
    @param path 日志路径。
    @param max_bytes 允许读取的最大文件字节数。
    @return (相对首条记录的秒数, 原记录) 列表；记录顺序保持不变。
    @note 允许相同时间戳，拒绝倒退、负数和非有限时间；格式错误附带行号，空日志也会报错。
    """
    path = Path(path)
    if path.stat().st_size > max_bytes:
        raise ValueError("本版回放支持 128 MB 以内的日志，请分段录制")
    records, previous = [], 0.0
    # 1. 按文件顺序校验接收时间；允许相同时间代表同一次刷新中的多条记录，
    #    但不对倒序记录重新排序，以免掩盖日志中的时间错误。
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
    # 2. 全部验证成功后以首条为零点，消除开始录制到首帧之间的等待。
    start = records[0]["record_time_s"]
    return [(r["record_time_s"] - start, r) for r in records]
