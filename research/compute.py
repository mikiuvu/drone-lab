"""Synthetic CPU/NPU scheduling study; no board I/O and no import side effects.

One Linux CPU, one NPU, one admitted vision frame through the entire pipeline.
Only navigation can overlap vision's NPU stage. Camera arrivals represent frames
made available by a driver, not modeled exposure/ISP operations.
"""

from collections import deque
from copy import deepcopy
import heapq
import math


DEFAULT_CONFIG = {
    "camera_fps": 30,
    "cpu_pre_ms": 4,
    "npu_ms": 18,
    "cpu_post_ms": 3,
    "nav_ms": 5,
    "nav_hz": 15,
    "duration_s": 10,
    "cpu_capture_ms": 0.5,
    "cpu_tracking_ms": 1,
    "cpu_copy_ms": 0,
    "frame_deadline_ms": 100,
    "nav_deadline_ms": None,  # default: one navigation period
    "frame_bytes": 640 * 480 * 3 // 2,  # illustrative tightly packed NV12
    "fixed_memory_bytes": 0,  # caller's driver/model/tensor pools; not auto-estimated
    "npu_ms_pattern": [],  # cyclic by admitted camera frame ID; [] uses npu_ms
    "bursts": [],  # start_s/end_s/cpu_factor/npu_factor; latch at stage start
    "trace_limit": 0,  # bounded diagnostic trace; not a modeled frame buffer
}

OVERLOAD_CONFIG = {"npu_ms": 60, "cpu_pre_ms": 6, "cpu_post_ms": 4}
EPS = 1e-8  # milliseconds, only for ordering/roundoff


def _number(value, name, minimum=0, strict=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限数值")
    if not math.isfinite(value) or value < minimum or (strict and value == minimum):
        raise ValueError(f"{name} 超出允许范围")
    return value


def _config(config):
    if config is not None and not isinstance(config, dict):
        raise ValueError("config 必须是 dict 或 None")
    cfg = deepcopy(DEFAULT_CONFIG)
    for key, value in (config or {}).items():
        if key not in cfg:
            raise ValueError(f"未知参数：{key}")
        cfg[key] = deepcopy(value)
    for key in ("camera_fps", "nav_hz", "duration_s", "frame_deadline_ms"):
        _number(cfg[key], key, strict=True)
    for key in ("cpu_pre_ms", "npu_ms", "cpu_post_ms", "nav_ms",
                "cpu_capture_ms", "cpu_tracking_ms", "cpu_copy_ms"):
        _number(cfg[key], key)
    if cfg["nav_deadline_ms"] is None:
        cfg["nav_deadline_ms"] = 1000 / cfg["nav_hz"]
    _number(cfg["nav_deadline_ms"], "nav_deadline_ms", strict=True)
    for key in ("frame_bytes", "fixed_memory_bytes", "trace_limit"):
        _number(cfg[key], key)
        if not isinstance(cfg[key], int):
            raise ValueError(f"{key} 必须是整数")
    if cfg["frame_bytes"] == 0 or cfg["trace_limit"] > 10000:
        raise ValueError("frame_bytes 必须大于零；trace_limit 不能超过 10000")
    if (cfg["camera_fps"] + cfg["nav_hz"]) * cfg["duration_s"] > 100000:
        raise ValueError("释放事件上限为 100000，请缩短仿真窗口")
    pattern = cfg["npu_ms_pattern"]
    if not isinstance(pattern, list) or len(pattern) > 10000:
        raise ValueError("npu_ms_pattern 必须是最多 10000 项的列表")
    for value in pattern:
        _number(value, "npu_ms_pattern")
    if not isinstance(cfg["bursts"], list) or len(cfg["bursts"]) > 100:
        raise ValueError("bursts 必须是最多 100 项的列表")
    normalized = []
    for burst in cfg["bursts"]:
        if not isinstance(burst, dict) or set(burst) - {
            "start_s", "end_s", "cpu_factor", "npu_factor"
        } or not {"start_s", "end_s"} <= set(burst):
            raise ValueError("burst 需 start_s、end_s，可选 cpu_factor、npu_factor")
        entry = dict(cpu_factor=1, npu_factor=1, **{
            k: v for k, v in burst.items() if k not in ("cpu_factor", "npu_factor")
        })
        entry.update(burst)
        for key in ("start_s", "end_s"):
            _number(entry[key], key)
        for key in ("cpu_factor", "npu_factor"):
            _number(entry[key], key, strict=True)
        if entry["end_s"] <= entry["start_s"]:
            raise ValueError("burst 的 end_s 必须大于 start_s")
        normalized.append(entry)
    cfg["bursts"] = normalized
    # Prevent finite inputs whose products would overflow the event clock.
    max_cpu = max(cfg[k] for k in ("cpu_capture_ms", "cpu_copy_ms",
                                  "cpu_pre_ms", "cpu_post_ms",
                                  "cpu_tracking_ms", "nav_ms"))
    max_npu = max([cfg["npu_ms"]] + pattern)
    for key, base in (("cpu_factor", max_cpu), ("npu_factor", max_npu)):
        for burst in normalized:
            base *= burst[key]
            if not math.isfinite(base):
                raise ValueError("服务耗时乘积过大")
    horizon = cfg["duration_s"] * 1000
    if (not math.isfinite(horizon) or not math.isfinite(1000 / cfg["camera_fps"])
            or not math.isfinite(1000 / cfg["nav_hz"])):
        raise ValueError("时间范围过大")
    if (horizon < 1e-5 or horizon > 1e12 or cfg["camera_fps"] > 1e9
            or cfg["nav_hz"] > 1e9 or max_cpu > 1e12 or max_npu > 1e12):
        raise ValueError("仿真时间或服务参数超出数值稳定范围")
    cpu_factor = npu_factor = 1.0
    for burst in normalized:
        cpu_factor *= burst["cpu_factor"]
        npu_factor *= burst["npu_factor"]
    if (not math.isfinite(cpu_factor) or not math.isfinite(npu_factor)
            or cpu_factor * max_cpu > 1e12 or npu_factor * max_npu > 1e12):
        raise ValueError("突发服务时间过大")
    return cfg


def _stats(values):
    """Nearest-rank quantiles; no completed samples returns null, never zero."""
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None}
    ordered = sorted(values)
    return {
        "count": len(values),
        "p50": round(ordered[math.ceil(0.50 * len(values)) - 1], 6),
        "p95": round(ordered[math.ceil(0.95 * len(values)) - 1], 6),
        "max": round(ordered[-1], 6),
    }


def _simulate(cfg, policy):
    horizon = cfg["duration_s"] * 1000
    events = []
    serial = 0

    def event(t, kind, payload=None):
        nonlocal serial
        # Completion events win ties; dispatch only after the whole timestamp.
        priority = 0 if kind.endswith("done") else 1
        heapq.heappush(events, (t, priority, serial, kind, payload))
        serial += 1

    # Arrivals are scheduled lazily, so the heap also stays small.
    event(0, "camera", 0)
    event(0, "nav", 0)
    queue = deque()
    active = None
    cpu_ready = None
    cpu = None
    nav_pending = None
    frame_ages, nav_ages, queue_waits, trace = [], [], [], []
    frames = dict(generated=0, completed=0, dropped=0, deadline_misses_completed=0)
    navigation = dict(generated=0, completed=0, dropped=0,
                      deadline_misses_completed=0, deadline_misses_dropped=0)
    max_queue = max_buffers = 0
    cpu_busy = npu_busy = overlap = 0.0
    npu_interval = None

    def record(t, kind, ident):
        if len(trace) < cfg["trace_limit"]:
            trace.append({"time_ms": round(t, 6), "event": kind, "id": ident})

    def service(base, t, resource):
        factor = 1.0
        for burst in cfg["bursts"]:
            if burst["start_s"] * 1000 <= t < burst["end_s"] * 1000:
                factor *= burst[resource + "_factor"]
        return base * factor

    def peaks():
        nonlocal max_queue, max_buffers
        max_queue = max(max_queue, len(queue))
        max_buffers = max(max_buffers, len(queue) + (active is not None))

    def start_cpu(t, stage, item):
        nonlocal cpu, cpu_busy, overlap
        key = "nav_ms" if stage == "nav" else "cpu_" + stage + "_ms"
        duration = service(cfg[key], t, "cpu")
        end = t + duration
        cpu = (stage, item)
        cpu_busy += max(0, min(end, horizon) - t)
        if npu_interval is not None:
            overlap += max(0, min(end, npu_interval[1], horizon)
                           - max(t, npu_interval[0]))
        record(t, "cpu_" + stage + "_start", item["id"])
        event(end, "cpu_done")

    def dispatch(t):
        nonlocal active, cpu_ready, nav_pending
        if cpu is not None:
            return
        if nav_pending is not None:
            item, nav_pending = nav_pending, None
            start_cpu(t, "nav", item)
            return
        if active is None and queue:
            active = queue.popleft()
            queue_waits.append(t - active["created"])
            cpu_ready = "capture"
        if cpu_ready is not None:
            stage, cpu_ready = cpu_ready, None
            start_cpu(t, stage, active)

    while events and events[0][0] <= horizon + EPS:
        t = events[0][0]
        # Handle every simultaneous completion/arrival before choosing CPU work.
        while events and abs(events[0][0] - t) <= EPS:
            _, _, _, kind, payload = heapq.heappop(events)
            if kind == "camera":
                frames["generated"] += 1
                item = {"id": payload, "created": t}
                record(t, "camera", payload)
                if policy == "latest" and queue:
                    old = queue.pop()
                    frames["dropped"] += 1
                    record(t, "drop", old["id"])
                queue.append(item)
                next_t = (payload + 1) * 1000 / cfg["camera_fps"]
                if next_t < horizon - EPS:
                    event(next_t, "camera", payload + 1)
                peaks()
            elif kind == "nav":
                navigation["generated"] += 1
                if nav_pending is not None:
                    navigation["dropped"] += 1
                    if t - nav_pending["created"] > cfg["nav_deadline_ms"] + EPS:
                        navigation["deadline_misses_dropped"] += 1
                    record(t, "nav_drop", nav_pending["id"])
                nav_pending = {"id": payload, "created": t}
                next_t = (payload + 1) * 1000 / cfg["nav_hz"]
                if next_t < horizon - EPS:
                    event(next_t, "nav", payload + 1)
            elif kind == "npu_done":
                cpu_ready = "post"
                npu_interval = None
                record(t, "npu_done", active["id"])
            else:
                stage, item = cpu
                cpu = None
                record(t, "cpu_" + stage + "_done", item["id"])
                if stage == "nav":
                    age = t - item["created"]
                    nav_ages.append(age)
                    navigation["completed"] += 1
                    navigation["deadline_misses_completed"] += (
                        age > cfg["nav_deadline_ms"] + EPS)
                elif stage in ("capture", "copy", "post"):
                    cpu_ready = {"capture": "copy", "copy": "pre",
                                 "post": "tracking"}[stage]
                elif stage == "pre":
                    pattern = cfg["npu_ms_pattern"]
                    base = pattern[item["id"] % len(pattern)] if pattern else cfg["npu_ms"]
                    end = t + service(base, t, "npu")
                    npu_interval = (t, end)
                    npu_busy += max(0, min(end, horizon) - t)
                    record(t, "npu_start", item["id"])
                    event(end, "npu_done")
                else:  # tracking completes the admitted vision frame
                    age = t - item["created"]
                    frame_ages.append(age)
                    frames["completed"] += 1
                    frames["deadline_misses_completed"] += (
                        age > cfg["frame_deadline_ms"] + EPS)
                    record(t, "frame_done", item["id"])
                    active = None
        # At the boundary, only finish zero-duration continuations of an
        # already admitted frame. Never admit a new frame/navigation job.
        if t < horizon - EPS:
            dispatch(t)
            peaks()
        elif cpu is None and cpu_ready is not None:
            base = cfg["cpu_" + cpu_ready + "_ms"]
            if service(base, t, "cpu") == 0:
                stage, cpu_ready = cpu_ready, None
                start_cpu(t, stage, active)

    outstanding = list(queue) + ([active] if active is not None else [])
    overdue = sum(horizon - item["created"] > cfg["frame_deadline_ms"] + EPS
                  for item in outstanding)
    frames.update(
        pending=len(queue), in_flight=int(active is not None),
        outstanding=len(outstanding), overdue_at_end=overdue,
        deadline_misses_observed=frames["deadline_misses_completed"] + overdue,
        deadline_miss_fraction_generated=(frames["deadline_misses_completed"] + overdue)
        / frames["generated"],
        age_ms=_stats(frame_ages),
        queue_wait_ms=_stats(queue_waits),
        stale_completed=frames["deadline_misses_completed"],
        oldest_outstanding_age_ms=round(max(
            (horizon - item["created"] for item in outstanding), default=0), 6),
    )
    nav_active = cpu[1] if cpu is not None and cpu[0] == "nav" else None
    nav_outstanding = ([nav_pending] if nav_pending is not None else []) + (
        [nav_active] if nav_active is not None else [])
    nav_overdue = sum(horizon - item["created"] > cfg["nav_deadline_ms"] + EPS
                      for item in nav_outstanding)
    navigation.update(
        pending=int(nav_pending is not None), in_flight=int(nav_active is not None),
        overdue_at_end=nav_overdue, age_ms=_stats(nav_ages),
        deadline_misses_observed=navigation["deadline_misses_completed"]
        + navigation["deadline_misses_dropped"] + nav_overdue,
    )
    memory = dict(
        frame_bytes=cfg["frame_bytes"], max_queue_frames=max_queue,
        queue_frames_end=len(queue), max_held_frames=max_buffers,
        max_queue_bytes=max_queue * cfg["frame_bytes"],
        queue_bytes_end=len(queue) * cfg["frame_bytes"],
        max_held_frame_bytes=max_buffers * cfg["frame_bytes"],
        held_frame_bytes_end=len(outstanding) * cfg["frame_bytes"],
        fixed_memory_bytes=cfg["fixed_memory_bytes"],
        modeled_peak_bytes=max_buffers * cfg["frame_bytes"] + cfg["fixed_memory_bytes"],
        held_frame_bound=2 if policy == "latest" else None,
        in_flight_frame_bound=1,
        waiting_frame_bound=1 if policy == "latest" else None,
        bounded=policy == "latest",
    )
    return {
        "policy": policy, "frames": frames, "navigation": navigation,
        "memory": memory,
        "resources": {"cpu_busy_ms": round(cpu_busy, 6),
                      "npu_busy_ms": round(npu_busy, 6),
                      "cpu_utilization": cpu_busy / horizon,
                      "npu_utilization": npu_busy / horizon,
                      "cpu_npu_overlap_ms": round(overlap, 6)},
        "trace": trace,
    }


def run_compute(config: dict | None = None) -> dict:
    """Compare FIFO vs latest-frame admission using synthetic milliseconds.

    No file access or dependencies. The horizon is half-open for releases and
    closed for completions. No post-window drain: unfinished frames and their
    expired deadlines are explicitly reported, never hidden in age quantiles.
    """
    cfg = _config(config)
    fifo = _simulate(cfg, "fifo")
    latest = _simulate(cfg, "latest")
    cpu_vision = sum(cfg[k] for k in ("cpu_capture_ms", "cpu_copy_ms",
                                    "cpu_pre_ms", "cpu_post_ms",
                                    "cpu_tracking_ms"))
    npu_mean = (sum(cfg["npu_ms_pattern"]) / len(cfg["npu_ms_pattern"])
                if cfg["npu_ms_pattern"] else cfg["npu_ms"])
    metrics = {
        "fifo": fifo, "latest": latest,
        "nominal_offered_cpu_utilization":
            (cfg["camera_fps"] * cpu_vision + cfg["nav_hz"] * cfg["nav_ms"]) / 1000,
        "nominal_offered_npu_utilization": cfg["camera_fps"] * npu_mean / 1000,
        "nominal_serial_vision_service_ms": cpu_vision + npu_mean,
        "nominal_serial_vision_load": cfg["camera_fps"] * (cpu_vision + npu_mean) / 1000,
    }
    summary = (f"合成仿真：FIFO 积压 {fifo['frames']['outstanding']} 帧；"
               f"最新帧队列丢弃 {latest['frames']['dropped']} 帧，"
               f"峰值持有 {latest['memory']['max_held_frames']} 帧。")
    return {
        "summary": summary, "metrics": metrics, "config": cfg,
        "assumptions": [
            "全部服务耗时由输入参数指定。",
            "资源模型包含一个 Linux CPU 服务台和一个 NPU 服务台。",
            "单视觉帧依次采集、复制、预处理、NPU、后处理、跟踪；NPU 期间 CPU 可执行定位。",
            "定位优先，在 CPU 阶段边界调度；CPU 阶段不抢占，等待定位槽容量一并覆盖旧任务。",
            "FIFO 无界；latest 只覆盖等待帧，不取消已接纳帧，整个视觉管线最多持有两帧。",
            "相机周期到达；时间戳表示驱动可提供帧的时刻，未建模曝光、ISP、驱动内排队或总线。",
            "突发倍率在服务阶段开始时锁定；重叠突发相乘。NPU 周期耗时按原始帧 ID 选取。",
            "只统计窗口内完成帧的 nearest-rank 帧龄；未完成、到期、丢弃数另报，窗口后不排空。",
            "期限违约指年龄严格大于期限；视觉丢帧单独统计。",
            "视觉期限从驱动帧可用到跟踪完成；定位期限从周期任务释放到 CPU 计算完成。",
            "queue_wait_ms 截止进入采集阶段，age_ms 截止跟踪完成；stale_completed 为完成时已超过视觉期限的输出。",
            "仿真仍统计过期完成输出；下游必须拒绝超龄或重复 target/frame ID，并按无新鲜目标状态退化。",
            "frame_bytes 为紧密 NV12 示例，不含实际 stride；fixed_memory_bytes 默认为零表示尚未填写。",
            "两帧上界约束应用帧引用；SDK/VB/ION/模型/张量/内核和观测记录另计。",
            "名义负载按平均服务时间计算；突发影响见帧龄、积压和期限违约指标。",
        ],
        "model_version": 1,
        "hardware_measured": False,
    }
