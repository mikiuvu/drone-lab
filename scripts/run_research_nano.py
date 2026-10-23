"""在 Nano 板端以真实进程运行算法回放负载，采集设备级资源并汇总为机器可读结果。

覆盖范围：算法内核的定时回放（三路 ToF 定位、光流、视觉、导航、撞击任务调度）。
不覆盖：相机采集、ToF 硬件 I/O、MAVLink 传输、执行机构动作；
板端 ToF 帧来自地形射线投射，非真实传感器输入。
"""
import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import time

import paramiko

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_KNOWN_HOSTS = ROOT / "downloads/nano-ram-recovery/known_hosts"
DEFAULT_BINARY = ROOT / "build/nano-algorithms/algorithm_load_test"
HISTORICAL_RUN = ROOT / "simulation/runtime/run_20261010_201647"
ARMOR_IMAGE = ROOT / "simulation/runtime/benchmark-armor.ppm"
GUIDE_IMAGE = HISTORICAL_RUN / "camera/camera_0.ppm"
HISTORICAL_TERRAIN = HISTORICAL_RUN / "original_base/terrain_fixed.f32"
RMUC2027_TERRAIN = ROOT / "simulation/models/terrain/rmuc_2027/terrain_triangles.f32"

# 各工况：相机频率、时长来源（seconds/sustained）、地形、视觉分支策略。
PROFILES = {
    "camera5": dict(camera_hz=5, duration="seconds", terrain="historical",
                    vision_branches="fallback"),
    "camera15": dict(camera_hz=15, duration="seconds", terrain="historical",
                     vision_branches="fallback"),
    "camera30": dict(camera_hz=30, duration="seconds", terrain="historical",
                     vision_branches="fallback"),
    "sustained15": dict(camera_hz=15, duration="sustained", terrain="historical",
                        vision_branches="fallback"),
    "allbranches15": dict(camera_hz=15, duration="seconds", terrain="historical",
                          vision_branches="all"),
    "rmuc2027_15": dict(camera_hz=15, duration="seconds", terrain="rmuc2027",
                        vision_branches="fallback"),
}
DEFAULT_ORDER = ["camera5", "camera15", "camera30", "sustained15",
                 "allbranches15", "rmuc2027_15"]

# 每个工况都必须产生样本的算法阶段，缺任一即判定覆盖失败。
REQUIRED_STAGES = ("imu_predict", "flow_height", "vision_submit", "vision_poll",
                   "target_tracking", "navigation", "periodic_service", "periodic_learning",
                   "intercept_guidance", "team_impact_schedule", "vision_pixel",
                   "vision_detect_pnp", "tof_frame")

MONITOR_SCRIPT = r"""#!/bin/sh
# 每 1 秒采样设备级资源；出现停止文件即退出。由 run_research_nano.py 生成。
OUT="$1"; STOP="$2"; MONPID="$3"; TESTPID="$4"
echo $$ > "$MONPID"
i=0
while [ ! -f "$STOP" ]; do
  i=$((i+1))
  {
    echo "SAMPLE $i $(date +%s)"
    echo "STAT $(sed -n '1p' /proc/stat)"
    echo "LOAD $(cat /proc/loadavg)"
    echo "MEM $(awk '/^(MemTotal|MemFree|MemAvailable|Buffers|Cached|SwapTotal|SwapFree|Dirty|Slab|SReclaimable|SUnreclaim|Active|Inactive|Shmem|Committed_AS):/{gsub(/[ \t]+/,"",$0); printf "%s ", $0}' /proc/meminfo)"
    echo "UPTIME $(cat /proc/uptime)"
    th=""
    for z in /sys/class/thermal/thermal_zone*; do
      [ -d "$z" ] || continue
      th="$th${z##*/}:$(cat $z/type 2>/dev/null):$(cat $z/temp 2>/dev/null) "
    done
    echo "THERM $th"
    cl=""
    for z in /sys/class/thermal/cooling_device*; do
      [ -d "$z" ] || continue
      cl="$cl${z##*/}:$(cat $z/type 2>/dev/null):$(cat $z/cur_state 2>/dev/null) "
    done
    echo "COOL $cl"
    fr=""
    for d in /sys/devices/system/cpu/cpu[0-9]*; do
      [ -d "$d/cpufreq" ] || continue
      fr="$fr${d##*/}:$(cat $d/cpufreq/scaling_cur_freq 2>/dev/null) "
    done
    echo "FREQ $fr"
    if [ -f "$TESTPID" ]; then
      p=$(cat "$TESTPID" 2>/dev/null)
      if [ -n "$p" ] && [ -d "/proc/$p" ]; then
        echo "PROC $p $(awk '{print $14" "$15" "$24}' /proc/$p/stat 2>/dev/null)"
        echo "PROCSTAT $(awk '/^(VmPeak|VmSize|VmHWM|VmRSS|VmData|VmStk|Threads):/{gsub(/[ \t]+/,"",$0); printf "%s ", $0}' /proc/$p/status 2>/dev/null)"
      fi
    fi
    echo "END"
  } >> "$OUT" 2>&1
  sleep 1
done
"""

LAUNCHER_SCRIPT = r"""#!/bin/sh
# 记录自身 PID 后原地替换为目标程序，PID 在 exec 后保持不变。
PIDFILE="$1"; shift
echo $$ > "$PIDFILE"
exec "$@"
"""


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare_poses(run, output):
    """导出历史 run 中所有被接受的 visual_pose，供板端回放。"""
    poses, sources = [], []
    for vehicle in range(4):
        path = run / f"companion_{vehicle}.jsonl"
        accepted = 0
        with path.open(encoding="utf-8") as stream:
            for line in stream:
                line = line.strip()
                if not line:
                    continue
                row = json.loads(line)
                if row.get("kind") != "visual_pose" or not row.get("accepted"):
                    continue
                poses.append(dict(vehicle=vehicle, **{k: row[k] for k in
                    ("sim_time_us", "x_m", "y_m", "z_m")}))
                accepted += 1
        sources.append(dict(path=str(path.relative_to(ROOT)), sha256=digest(path), accepted=accepted))
    poses.sort(key=lambda p: (p["sim_time_us"], p["vehicle"]))
    path = output / "poses.json"
    path.write_text(json.dumps(dict(source="Historical Gazebo accepted visual_pose records",
        sources=sources, poses=poses), indent=2), encoding="utf-8")
    return path, sources, len(poses)


def remote_command(client, command, limit=120):
    """执行短命令，返回 (stdout, stderr, exit_status)。"""
    _, stdout, stderr = client.exec_command(command)
    channel = stdout.channel
    out, err = bytearray(), bytearray()
    started = time.monotonic()
    while True:
        while channel.recv_ready():
            out.extend(channel.recv(65536))
        while channel.recv_stderr_ready():
            err.extend(channel.recv_stderr(65536))
        if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
            break
        if time.monotonic() - started > limit:
            channel.close()
            raise TimeoutError(f"remote command exceeded {limit}s: {command[:80]}")
        time.sleep(0.05)
    return out.decode("utf-8", "replace"), err.decode("utf-8", "replace"), channel.recv_exit_status()


def stream_command(client, command, log_path, limit, progress_prefix="elapsed="):
    """流式执行命令：完整输出写入 log_path，只在终端打印进度行。"""
    _, stdout, stderr = client.exec_command(command)
    channel = stdout.channel
    started = time.monotonic()
    out_tail, err_tail = "", ""
    out_bytes, err_bytes = 0, 0
    timed_out = False
    with log_path.open("wb") as log:
        while True:
            progressed = False
            while channel.recv_ready():
                data = channel.recv(65536)
                progressed = True
                out_bytes += len(data)
                log.write(data)
                log.flush()
                out_tail += data.decode("utf-8", "replace")
                lines = out_tail.split("\n")
                out_tail = lines.pop()
                for line in lines:
                    if line.startswith(progress_prefix):
                        print("  " + line, flush=True)
            while channel.recv_stderr_ready():
                data = channel.recv_stderr(65536)
                progressed = True
                err_bytes += len(data)
                log.write(data)
                log.flush()
                err_tail += data.decode("utf-8", "replace")
                if len(err_tail) > 4096:
                    err_tail = err_tail[-4096:]
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                break
            if time.monotonic() - started > limit:
                timed_out = True
                break
            if not progressed:
                time.sleep(0.05)
    if timed_out:
        channel.close()
        return dict(status=None, timed_out=True, stdout_bytes=out_bytes, stderr_bytes=err_bytes,
                    stderr_tail=err_tail[-2000:])
    return dict(status=channel.recv_exit_status(), timed_out=False, stdout_bytes=out_bytes,
                stderr_bytes=err_bytes, stderr_tail=err_tail[-2000:])


def kill_remote_pid(client, pidfile, log):
    """按 PID 文件精确终止测试进程；先用 cmdline 核对身份，不使用 pkill。"""
    command = (
        f"p=$(cat {shlex.quote(pidfile)} 2>/dev/null); "
        "if [ -n \"$p\" ] && [ -d \"/proc/$p\" ] && "
        "tr '\\0' ' ' < \"/proc/$p/cmdline\" 2>/dev/null | grep -q 'algorithm_load_test'; then "
        "kill -TERM \"$p\" 2>/dev/null; sleep 3; "
        "[ -d \"/proc/$p\" ] && kill -KILL \"$p\" 2>/dev/null; "
        "echo \"terminated $p\"; else echo \"no-matching-pid\"; fi")
    try:
        out, err, _ = remote_command(client, command, limit=20)
        log.append(dict(action="kill", output=(out + err).strip()))
    except Exception as error:  # noqa: BLE001 - 终止失败必须记录而非中断
        log.append(dict(action="kill", error=type(error).__name__ + ": " + str(error)[:200]))


def start_monitor(client, remote, name, monpid, testpid, reslog, stop):
    command = ("cd " + shlex.quote(remote) + " && rm -f " + shlex.quote(stop) +
               " && exec /bin/sh " + shlex.quote(remote + "/monitor.sh") + " " +
               " ".join(shlex.quote(x) for x in (reslog, stop, monpid, testpid)))
    _, stdout, stderr = client.exec_command(command)
    channel = stdout.channel
    for _ in range(50):
        if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
            raise RuntimeError("monitor exited early: " + stderr.read().decode("utf-8", "replace")[:300])
        try:
            pid_text = remote_command(client, "cat " + shlex.quote(monpid) + " 2>/dev/null")[0].strip()
        except Exception:  # noqa: BLE001
            pid_text = ""
        if pid_text.isdigit():
            return channel, int(pid_text)
        time.sleep(0.1)
    channel.close()
    raise RuntimeError("monitor did not publish a PID")


def terminate_monitor(client, monpid):
    """核对 cmdline 后按 PID 精确终止本任务监视脚本，不使用 pkill。"""
    check = ("p=$(cat " + shlex.quote(monpid) + " 2>/dev/null); "
             "if [ -n \"$p\" ] && [ -d \"/proc/$p\" ] && "
             "tr '\\0' ' ' < \"/proc/$p/cmdline\" 2>/dev/null | grep -q 'monitor.sh'; then "
             "kill -KILL \"$p\" 2>/dev/null; echo killed; else echo gone; fi")
    try:
        return remote_command(client, check, limit=20)[0].strip()
    except Exception:  # noqa: BLE001
        return "unknown"


def stop_monitor(client, remote, stop, monpid, channel):
    """停止本任务监视：写停止文件后仅等待退出状态，超时则按 PID 精确终止再关闭通道。"""
    remote_command(client, "touch " + shlex.quote(stop))
    exited = False
    for _ in range(100):
        if channel.exit_status_ready():
            exited = True
            break
        time.sleep(0.1)
    exit_status = None
    if exited:
        try:
            exit_status = channel.recv_exit_status()
        except Exception:  # noqa: BLE001
            exit_status = None
        state = "gone"
    else:
        state = terminate_monitor(client, monpid)
    channel.close()
    return dict(exited=exited, exit_status=exit_status, state=state)


def cleanup_active(client, active):
    """确保本任务已启动的监视与测试进程都被精确停止，再清空活动记录。"""
    if active.get("stop"):
        try:
            remote_command(client, "touch " + shlex.quote(active["stop"]), limit=15)
        except Exception:  # noqa: BLE001
            pass
    if active.get("monitor_channel") is not None:
        channel = active["monitor_channel"]
        for _ in range(50):
            if channel.exit_status_ready():
                break
            time.sleep(0.1)
        try:
            channel.close()
        except Exception:  # noqa: BLE001
            pass
        if active.get("monpid"):
            terminate_monitor(client, active["monpid"])
    if active.get("test_pid"):
        kill_remote_pid(client, active["test_pid"], [])
    active.update(monitor_channel=None, stop=None, monpid=None, test_pid=None)


def parse_cpu_line(line):
    parts = line.split()
    if not parts or parts[0] != "cpu":
        return None
    values = [int(x) for x in parts[1:]]
    while len(values) < 10:
        values.append(0)
    user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice = values[:10]
    # guest 已计入 user，guest_nice 已计入 nice；合计时排除以避免重复计数。
    total = user + nice + system + idle + iowait + irq + softirq + steal
    return dict(user=user, nice=nice, system=system, idle=idle, iowait=iowait, irq=irq,
                softirq=softirq, steal=steal, guest=guest, guest_nice=guest_nice,
                total=total, busy=total - idle - iowait)


def parse_kv(line):
    result = {}
    for token in line.split():
        if ":" in token:
            key, value = token.split(":", 1)
            result[key] = value
    return result


def meminfo_value(text, key):
    match = re.search(rf"^{re.escape(key)}:\s+(\d+)", text, re.MULTILINE)
    return int(match.group(1)) if match else None


def parse_resources(text, cpu_count, clk_tck=None):
    """解析板端监视日志；缺失字段显式标记为不可用。clk_tck 来自远端 getconf CLK_TCK。"""
    samples = []
    current = None
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("SAMPLE "):
            if current is not None:
                samples.append(current)
            parts = line.split()
            current = dict(index=int(parts[1]) if len(parts) > 1 else None,
                           epoch_s=int(parts[2]) if len(parts) > 2 else None)
            continue
        if current is None:
            continue
        if line.startswith("STAT "):
            current["cpu"] = parse_cpu_line(line[5:])
        elif line.startswith("LOAD "):
            fields = line[5:].split()
            current["loadavg"] = [float(x) for x in fields[:3]] if len(fields) >= 3 else None
            current["loadavg_tasks"] = fields[3] if len(fields) > 3 else None
        elif line.startswith("MEM "):
            current["mem_kib"] = {k: int(v.replace("kB", "")) for k, v in parse_kv(line[4:]).items()
                                  if v.replace("kB", "").isdigit()}
        elif line.startswith("UPTIME "):
            fields = line[7:].split()
            current["uptime_s"] = float(fields[0]) if fields else None
        elif line.startswith("THERM "):
            zones = {}
            for token in line[6:].split():
                parts = token.split(":")
                if len(parts) >= 3 and parts[2].lstrip("-").isdigit():
                    zones[parts[0]] = dict(type=parts[1], temp_c=int(parts[2]) / 1000)
            current["thermal"] = zones
        elif line.startswith("COOL "):
            coolers = {}
            for token in line[5:].split():
                parts = token.split(":")
                if len(parts) >= 3:
                    coolers[parts[0]] = dict(type=parts[1], state=parts[2])
            current["cooling"] = coolers
        elif line.startswith("FREQ "):
            freq = {}
            for token in line[5:].split():
                parts = token.split(":")
                if len(parts) >= 2 and parts[1].isdigit():
                    freq[parts[0]] = int(parts[1])
            current["cpufreq_khz"] = freq
        elif line.startswith("PROC "):
            fields = line[5:].split()
            if len(fields) >= 4:
                current["proc"] = dict(pid=int(fields[0]), utime=int(fields[1]),
                                       stime=int(fields[2]), rss_pages=int(fields[3]))
        elif line.startswith("PROCSTAT "):
            current["proc_status"] = {k: int(v.replace("kB", "")) for k, v in
                                      parse_kv(line[9:]).items() if v.replace("kB", "").isdigit()}
    if current is not None:
        samples.append(current)

    def cpu_percent(a, b):
        if not a or not b or a.get("cpu") is None or b.get("cpu") is None:
            return None
        dt_ = b["cpu"]["total"] - a["cpu"]["total"]
        db = b["cpu"]["busy"] - a["cpu"]["busy"]
        if dt_ <= 0:
            return None
        return 100.0 * db / dt_

    intervals = [cpu_percent(samples[i - 1], samples[i]) for i in range(1, len(samples))]
    intervals = [v for v in intervals if v is not None]
    overall = cpu_percent(samples[0], samples[-1]) if len(samples) >= 2 else None
    rss_series = [s["proc_status"]["VmRSS"] for s in samples
                  if s.get("proc_status", {}).get("VmRSS") is not None]
    proc_series = [s["proc"] for s in samples if s.get("proc")]
    proc_ticks_delta = (proc_series[-1]["utime"] + proc_series[-1]["stime"] -
                        proc_series[0]["utime"] - proc_series[0]["stime"]) if len(proc_series) >= 2 else None
    return dict(
        sample_count=len(samples),
        window_epoch_s=[samples[0].get("epoch_s"), samples[-1].get("epoch_s")] if samples else None,
        device_cpu_percent=overall,
        device_cpu_percent_scope="whole-machine busy fraction from /proc/stat delta (0..100, no per-core scaling)",
        device_cpu_percent_interval_mean=(sum(intervals) / len(intervals)) if intervals else None,
        device_cpu_percent_interval_max=max(intervals) if intervals else None,
        device_cpu_percent_interval_min=min(intervals) if intervals else None,
        loadavg_1min_max=max((s["loadavg"][0] for s in samples if s.get("loadavg")), default=None),
        mem_available_kib_min=min((s["mem_kib"]["MemAvailable"] for s in samples
                                   if s.get("mem_kib", {}).get("MemAvailable") is not None), default=None),
        mem_free_kib_min=min((s["mem_kib"]["MemFree"] for s in samples
                              if s.get("mem_kib", {}).get("MemFree") is not None), default=None),
        swap_total_kib=(samples[0]["mem_kib"].get("SwapTotal") if samples and samples[0].get("mem_kib") else None),
        thermal_c_max=max((z["temp_c"] for s in samples for z in s.get("thermal", {}).values()), default=None),
        thermal_c_min=min((z["temp_c"] for s in samples for z in s.get("thermal", {}).values()), default=None),
        cpufreq_available=any(s.get("cpufreq_khz") for s in samples),
        cpufreq_unavailable_reason=None if any(s.get("cpufreq_khz") for s in samples)
        else "no /sys/devices/system/cpu/*/cpufreq on device",
        proc_rss_kib_max=max((s["proc_status"]["VmRSS"] for s in samples
                              if s.get("proc_status", {}).get("VmRSS") is not None), default=None),
        proc_rss_kib_last=rss_series[-1] if rss_series else None,
        proc_rss_kib_first=rss_series[0] if rss_series else None,
        proc_rss_kib_series=rss_series,
        proc_hwm_kib_max=max((s["proc_status"]["VmHWM"] for s in samples
                              if s.get("proc_status", {}).get("VmHWM") is not None), default=None),
        proc_vmpeak_kib_max=max((s["proc_status"]["VmPeak"] for s in samples
                                 if s.get("proc_status", {}).get("VmPeak") is not None), default=None),
        proc_samples=sum(1 for s in samples if s.get("proc")),
        proc_ticks_delta=proc_ticks_delta,
        proc_cpu_seconds=(proc_ticks_delta / clk_tck) if (proc_ticks_delta is not None and clk_tck) else None,
        first_proc_sample_index=next((i for i, s in enumerate(samples) if s.get("proc")), None),
        clk_tck=clk_tck,
        clk_tck_source="remote getconf CLK_TCK" if clk_tck else "unavailable",
        cpu_count=cpu_count,
        samples=samples)


def stage_summary(result, elapsed):
    stages = {}
    for name, stat in result.get("stages", {}).items():
        row = dict(stat)
        row["achieved_hz"] = (stat.get("samples", 0) / elapsed) if elapsed else None
        stages[name] = row
    return stages


def profile_summary(name, spec, argv, seconds, camera_hz, binary_result, resources, wall_s, run_meta):
    elapsed = binary_result.get("elapsed_s") if binary_result else None
    summary = dict(
        profile=name, spec=spec, camera_hz=camera_hz, requested_seconds=seconds,
        argv=argv, wall_seconds=wall_s, exit_status=run_meta.get("status"),
        timed_out=run_meta.get("timed_out", False), stderr_tail=run_meta.get("stderr_tail", ""),
        kernel_oom=run_meta.get("kernel_oom"), kernel_evidence=run_meta.get("kernel_tail"),
        binary=dict(
            present=binary_result is not None,
            host_arch=binary_result.get("host_arch") if binary_result else None,
            host_name=binary_result.get("host_name") if binary_result else None,
            elapsed_s=elapsed,
            requested_duration_s=binary_result.get("requested_duration_s") if binary_result else None,
            completed_ticks=binary_result.get("completed_ticks") if binary_result else None,
            skipped_ticks=binary_result.get("skipped_ticks") if binary_result else None,
            deadline_misses=binary_result.get("deadline_misses") if binary_result else None,
            stopped_early=binary_result.get("stopped_early") if binary_result else None,
            process_cpu_percent=binary_result.get("process_cpu_percent") if binary_result else None,
            process_cpu_capacity_percent=(binary_result.get("process_cpu_percent") / resources.get("cpu_count", 1))
            if binary_result and binary_result.get("process_cpu_percent") is not None else None,
            process_cpu_seconds=binary_result.get("process_cpu_seconds") if binary_result else None,
            peak_rss_kib=binary_result.get("peak_rss_kib") if binary_result else None,
            steady_rss_kib=(binary_result.get("resources", [{}])[-1].get("rss_kib")
                            if binary_result and binary_result.get("resources") else None),
            opencv_runtime=binary_result.get("opencv_runtime") if binary_result else None,
            opencv_threads=binary_result.get("opencv_threads") if binary_result else None,
            terrain_storage_bytes=binary_result.get("terrain_storage_bytes") if binary_result else None,
            image_bytes=binary_result.get("image_bytes") if binary_result else None,
            recorded_pose_count=binary_result.get("recorded_pose_count") if binary_result else None,
            valid_tof_cells=binary_result.get("valid_tof_cells") if binary_result else None,
            navigation_intervals=binary_result.get("navigation_intervals") if binary_result else None,
            tof_queue=binary_result.get("tof_queue") if binary_result else None,
            vision_pipeline=binary_result.get("vision_pipeline") if binary_result else None,
            periodic_model_ready=binary_result.get("periodic_model_ready") if binary_result else None,
            periodic_samples=binary_result.get("periodic_samples") if binary_result else None,
            periodic_period_s=binary_result.get("periodic_period_s") if binary_result else None,
            periodic_reject=binary_result.get("periodic_reject") if binary_result else None,
            periodic_background=binary_result.get("periodic_background") if binary_result else None,
            mission_phase_counts=binary_result.get("mission_phase_counts") if binary_result else None,
            outcome_reasons=binary_result.get("outcome_reasons") if binary_result else None,
            tick_work=binary_result.get("tick_work") if binary_result else None,
            tick_completion_lag=binary_result.get("tick_completion_lag") if binary_result else None,
            resources_trace=binary_result.get("resources") if binary_result else None),
        stages=stage_summary(binary_result, elapsed) if binary_result else {},
        device=resources)
    summary["verification"] = verify(summary)
    return summary


def verify(summary):
    checks = {}
    binary = summary["binary"]
    stages = summary["stages"]
    spec = summary.get("spec", {})
    checks["binary_present"] = bool(binary["present"])
    checks["exit_status_ok"] = summary.get("exit_status") == 0 and not summary.get("timed_out")
    checks["host_arch_riscv64"] = binary.get("host_arch") == "riscv64"
    seconds = summary["requested_seconds"]
    elapsed = binary.get("elapsed_s")
    checks["duration_complete"] = bool(elapsed is not None and elapsed >= seconds * 0.98)
    checks["stopped_early_false"] = binary.get("stopped_early") is False
    checks["stages_present"] = bool(stages)
    missing = [name for name in REQUIRED_STAGES if not stages.get(name, {}).get("samples")]
    checks["missing_required_stages"] = missing
    checks["stages_with_samples"] = sorted(n for n, s in stages.items() if s.get("samples"))
    checks["stages_without_samples"] = sorted(n for n, s in stages.items() if not s.get("samples"))
    finite = True
    for key in ("elapsed_s", "process_cpu_percent", "peak_rss_kib"):
        value = binary.get(key)
        if value is None or not math.isfinite(float(value)):
            finite = False
    for stat in stages.values():
        for value in stat.values():
            if isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(float(value)):
                finite = False
    checks["finite_values"] = finite
    tof = binary.get("tof_queue") or {}
    valid_cells = binary.get("valid_tof_cells")
    checks["tof_valid_cells"] = valid_cells
    checks["tof_taken"] = tof.get("taken")
    checks["tof_submitted"] = tof.get("submitted")
    checks["tof_replaced"] = tof.get("replaced")
    # ToF 覆盖判定：有有效像素但接受数为零，或没有任何有效像素，都算覆盖失败。
    accepted = stages.get("tof_frame", {}).get("accepted")
    if not binary["present"]:
        checks["tof_coverage"] = "unavailable_no_result"
    elif not valid_cells or sum(valid_cells) == 0:
        checks["tof_coverage"] = "no_valid_cells"
    elif not accepted:
        checks["tof_coverage"] = "no_acceptance"
    else:
        checks["tof_coverage"] = "ok"
    # 分支覆盖：all 必须真正跑到绿灯导引。
    branch = {}
    if spec.get("vision_branches") == "all":
        guide = stages.get("green_guide", {})
        branch["green_guide_samples"] = guide.get("samples")
        branch["green_guide_accepted"] = guide.get("accepted")
        branch["all_branches_exercised"] = bool(guide.get("accepted"))
    checks["branch_coverage"] = branch
    failures = []
    for key in ("binary_present", "exit_status_ok", "host_arch_riscv64", "duration_complete",
                "stopped_early_false", "stages_present", "finite_values"):
        if not checks[key]:
            failures.append(key)
    if missing:
        failures.append("missing_required_stages:" + ",".join(missing))
    if checks["tof_coverage"] != "ok":
        failures.append("tof_coverage:" + checks["tof_coverage"])
    if branch.get("all_branches_exercised") is False:
        failures.append("branch_coverage:all_branches")
    checks["failures"] = failures
    checks["ok"] = not failures
    return checks


def ordered_results(results):
    ordered = [results[name] for name in DEFAULT_ORDER if name in results]
    ordered += [results[name] for name in results if name not in DEFAULT_ORDER]
    return ordered


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="10.132.85.1")
    parser.add_argument("--known-hosts", type=Path, default=DEFAULT_KNOWN_HOSTS)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--output", type=Path, required=True, help="结果目录")
    parser.add_argument("--seconds", type=float, default=60, help="基础工况时长（默认 60）")
    parser.add_argument("--sustained-seconds", type=float, default=180, help="持续工况时长（默认 180）")
    parser.add_argument("--profile", choices=tuple(PROFILES), action="append",
                        help="只运行指定工况，可重复；默认按固定顺序全部运行")
    parser.add_argument("--idle-seconds", type=float, default=10, help="测试前的空闲基线时长")
    parser.add_argument("--timeout-margin", type=float, default=150, help="命令超时相对时长的额外余量")
    parser.add_argument("--resume", action="store_true",
                        help="复用已有结果目录，只重跑 --profile 指定工况并合并汇总")
    parser.add_argument("--password-env", default="NANO_SSH_PASSWORD")
    args = parser.parse_args()

    if not args.binary.is_file():
        parser.error("binary not found: " + str(args.binary))
    for path in (ARMOR_IMAGE, GUIDE_IMAGE, HISTORICAL_TERRAIN, RMUC2027_TERRAIN, args.known_hosts):
        if not path.is_file():
            parser.error("input not found: " + str(path))
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    existing = (output / "manifest.json").exists() or (output / "summary.json").exists()
    if existing and not args.resume:
        parser.error("output already contains results; pass --resume with --profile: " + str(output))
    if args.resume and not args.profile:
        parser.error("--resume requires at least one --profile")
    profiles = args.profile or DEFAULT_ORDER

    prior, prior_idle = {}, None
    if args.resume:
        manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        stamp, remote = manifest["stamp"], manifest["remote"]
        prior_summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
        prior = {p["profile"]: p for p in prior_summary.get("profiles", [])}
        prior_idle = prior_summary.get("idle")
        manifest["commands"] = [c for c in manifest.get("commands", []) if c.get("profile") not in profiles]
        manifest["profiles"] = [n for n in DEFAULT_ORDER if n in set(manifest.get("profiles", [])) | set(profiles)]
    else:
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        remote = f"/root/research-nano/{stamp}"

    poses_path, pose_sources, pose_count = prepare_poses(HISTORICAL_RUN, output)
    local_inputs = {
        "algorithm_load_test": args.binary,
        "armor.ppm": ARMOR_IMAGE,
        "guide.ppm": GUIDE_IMAGE,
        "terrain_historical.f32": HISTORICAL_TERRAIN,
        "terrain_rmuc2027.f32": RMUC2027_TERRAIN,
        "poses.json": poses_path,
    }
    inputs = [dict(name=name, path=str(path), bytes=path.stat().st_size, sha256=digest(path))
              for name, path in local_inputs.items()]
    if not args.resume:
        manifest = dict(
            stamp=stamp, host=args.host, remote=remote,
            local_binary=str(args.binary.relative_to(ROOT)) if args.binary.is_relative_to(ROOT)
            else str(args.binary),
            profiles=profiles,
            coverage_excluded=["camera capture", "ToF hardware I/O", "MAVLink transport",
                               "actuator execution"],
            notes="Algorithm-kernel replay only; ToF frames are terrain raycasts, not sensor input.")
    manifest["date"] = dt.datetime.now().astimezone().isoformat()
    manifest["seconds"] = args.seconds
    manifest["sustained_seconds"] = args.sustained_seconds
    manifest["pose_count"] = pose_count
    manifest["pose_sources"] = pose_sources
    manifest["inputs"] = inputs
    manifest.setdefault("runs", []).append(dict(at=dt.datetime.now().astimezone().isoformat(),
                                                profiles=profiles, resume=bool(args.resume)))
    build_manifest = args.binary.parent / "build-manifest.json"
    if build_manifest.is_file():
        build = json.loads(build_manifest.read_text(encoding="utf-8"))
        if build.get("binary_sha256") != digest(args.binary):
            raise ValueError("build manifest hash mismatch: " + str(build_manifest))
        manifest["build"] = build
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    client = paramiko.SSHClient()
    client.load_host_keys(str(args.known_hosts))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    results = dict(prior)
    active = dict(monitor_channel=None, stop=None, monpid=None, test_pid=None)
    try:
        client.connect(args.host, username="root",
                       password=os.environ.get(args.password_env, "root"),
                       timeout=10, banner_timeout=15, auth_timeout=15,
                       look_for_keys=False, allow_agent=False)
        client.get_transport().set_keepalive(15)
        inspection = remote_command(client,
            "uname -a; uname -m; nproc; cat /proc/cpuinfo; cat /proc/meminfo; "
            "cat /proc/sys/kernel/random/boot_id; cat /proc/sys/kernel/hostname; "
            "cat /proc/uptime; cat /proc/stat; cat /proc/loadavg; free; df -h /; "
            "for z in /sys/class/thermal/thermal_zone*; do echo \"$z $(cat $z/type) $(cat $z/temp)\"; done; "
            "ls -d /sys/devices/system/cpu/cpu[0-9]*/cpufreq 2>/dev/null || echo cpufreq-absent")[0]
        (output / "device.txt").write_text(inspection, encoding="utf-8")
        arch = remote_command(client, "uname -m")[0].strip()
        if arch != "riscv64":
            raise RuntimeError("unexpected remote arch: " + arch)
        manifest["remote_arch"] = arch
        manifest["device"] = dict(uname=remote_command(client, "uname -a")[0].strip(),
                                  nproc=int(remote_command(client, "nproc")[0].strip() or "0"),
                                  mem_total_kib=int(remote_command(client,
                                      "grep MemTotal /proc/meminfo | tr -dc 0-9")[0] or "0"))
        cpu_count = manifest["device"]["nproc"] or 1
        clk_tck_text = remote_command(client, "getconf CLK_TCK 2>/dev/null || true")[0].strip()
        clk_tck = int(clk_tck_text) if clk_tck_text.isdigit() else None
        manifest["clk_tck"] = clk_tck
        manifest["clk_tck_source"] = "remote getconf CLK_TCK" if clk_tck else "unavailable"
        memory_budget = dict(
            mem_total_kib=manifest["device"]["mem_total_kib"],
            commit_limit_kib=meminfo_value(remote_command(client, "grep CommitLimit /proc/meminfo")[0],
                                           "CommitLimit"),
            swap_total_kib=meminfo_value(remote_command(client, "grep SwapTotal /proc/meminfo")[0],
                                         "SwapTotal"),
            overcommit_memory=int(remote_command(client, "cat /proc/sys/vm/overcommit_memory")[0].strip()
                                  or "-1"),
            overcommit_ratio=int(remote_command(client, "cat /proc/sys/vm/overcommit_ratio")[0].strip()
                                 or "-1"),
            full_terrain_triangles=RMUC2027_TERRAIN.stat().st_size // 36)
        manifest["memory_budget"] = memory_budget
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

        remote_command(client, "mkdir -p " + shlex.quote(remote))
        for script_name, script_text in (("monitor.sh", MONITOR_SCRIPT), ("run_test.sh", LAUNCHER_SCRIPT)):
            with client.open_sftp() as sftp:
                with sftp.file(remote + "/" + script_name, "w") as handle:
                    handle.write(script_text)
        with client.open_sftp() as sftp:
            for name, path in local_inputs.items():
                print(f"upload {name} ({path.stat().st_size} bytes)", flush=True)
                sftp.put(str(path), remote + "/" + name)
            sftp.chmod(remote + "/algorithm_load_test", 0o755)

        checksums = remote_command(client, "cd " + shlex.quote(remote) + " && sha256sum " +
                                   " ".join(shlex.quote(n) for n in local_inputs))[0]
        verified = {}
        for line in checksums.splitlines():
            checksum, name = line.split(maxsplit=1)
            name = name.strip()
            if digest(local_inputs[name]) != checksum:
                raise RuntimeError("upload checksum mismatch: " + name)
            verified[name] = checksum
        manifest["remote_checksums"] = verified
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

        # 空闲基线：测试开始前单独采样。
        if prior_idle is not None:
            idle_resources = prior_idle
            print("idle baseline reused from prior run", flush=True)
        else:
            print(f"idle baseline {args.idle_seconds}s", flush=True)
            idle_stop = remote + "/idle.stop"
            idle_log = remote + "/idle.resources.log"
            channel, mon_pid = start_monitor(client, remote, "idle", remote + "/idle.monpid",
                                             remote + "/idle.pid", idle_log, idle_stop)
            time.sleep(args.idle_seconds)
            state = stop_monitor(client, remote, idle_stop, remote + "/idle.monpid", channel)
            idle_text = remote_command(client, "cat " + shlex.quote(idle_log))[0]
            (output / "idle.resources.log").write_text(idle_text, encoding="utf-8")
            idle_resources = parse_resources(idle_text, cpu_count, clk_tck)
            idle_resources["monitor_state"] = state
            (output / "idle.summary.json").write_text(json.dumps(idle_resources, indent=2),
                                                      encoding="utf-8")
            print("idle device_cpu_percent=" + str(idle_resources["device_cpu_percent"]), flush=True)

        for name in profiles:
            spec = PROFILES[name]
            seconds = args.sustained_seconds if spec["duration"] == "sustained" else args.seconds
            terrain_name = "terrain_rmuc2027.f32" if spec["terrain"] == "rmuc2027" else "terrain_historical.f32"
            out_json = remote + f"/{name}.json"
            argv = [remote + "/algorithm_load_test",
                    "--image", remote + "/armor.ppm",
                    "--guide-image", remote + "/guide.ppm",
                    "--terrain", remote + "/" + terrain_name,
                    "--poses", remote + "/poses.json",
                    "--output", out_json,
                    "--seconds", str(seconds),
                    "--camera-hz", str(spec["camera_hz"]),
                    "--tof-hz", "15",
                    "--periodic-async", "1",
                    "--vision-async", "1",
                    "--tof-bounded", "1",
                    "--tof-navigation-slot", "1",
                    "--vision-branches", spec["vision_branches"]]
            manifest.setdefault("commands", []).append(dict(profile=name, seconds=seconds,
                                                            camera_hz=spec["camera_hz"], argv=argv))
            (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
            print(f"run {name}: {seconds}s camera={spec['camera_hz']}Hz terrain={terrain_name}", flush=True)

            test_pid = remote + f"/{name}.pid"
            stop = remote + f"/{name}.stop"
            monpid = remote + f"/{name}.monpid"
            reslog = remote + f"/{name}.resources.log"
            try:
                remote_command(client, "rm -f " +
                               " ".join(shlex.quote(x) for x in (test_pid, out_json, reslog)))
                channel, mon_pid = start_monitor(client, remote, name, monpid, test_pid, reslog, stop)
                active.update(monitor_channel=channel, stop=stop, monpid=monpid, test_pid=test_pid)
                run_command = ("cd " + shlex.quote(remote) + " && exec /bin/sh " +
                               shlex.quote(remote + "/run_test.sh") + " " + shlex.quote(test_pid) + " " +
                               " ".join(shlex.quote(x) for x in argv))
                started = time.monotonic()
                run_meta = stream_command(client, run_command, output / f"{name}.test.log",
                                          seconds + args.timeout_margin)
                wall_s = time.monotonic() - started
                if run_meta["timed_out"]:
                    kill_log = []
                    kill_remote_pid(client, test_pid, kill_log)
                    run_meta["kill"] = kill_log
                    print("  timeout -> terminated remote test process", flush=True)
                run_meta["monitor_state"] = stop_monitor(client, remote, stop, monpid, channel)
                active.update(monitor_channel=None, stop=None, monpid=None, test_pid=None)

                binary_result = None
                with client.open_sftp() as sftp:
                    try:
                        sftp.get(out_json, str(output / f"{name}.json"))
                        binary_result = json.loads((output / f"{name}.json").read_text(encoding="utf-8"))
                    except Exception as error:  # noqa: BLE001 - 结果缺失按失败记录
                        run_meta["result_fetch_error"] = type(error).__name__ + ": " + str(error)[:200]
                    try:
                        sftp.get(reslog, str(output / f"{name}.resources.log"))
                    except Exception:  # noqa: BLE001
                        pass
                resource_text = ""
                if (output / f"{name}.resources.log").exists():
                    resource_text = (output / f"{name}.resources.log").read_text(encoding="utf-8",
                                                                                errors="replace")
                if binary_result is None or run_meta.get("status") != 0:
                    kernel_text = remote_command(client, "dmesg 2>/dev/null | tail -40 || true")[0]
                    (output / f"{name}.kernel.log").write_text(kernel_text, encoding="utf-8")
                    run_meta["kernel_oom"] = ("Out of memory" in kernel_text) or ("oom-kill" in kernel_text)
                    run_meta["kernel_tail"] = kernel_text.strip().splitlines()[-8:]
                    match = re.search(r"anon-rss:(\d+)kB", kernel_text)
                    run_meta["kernel_peak_anon_rss_kib"] = int(match.group(1)) if match else None
                    match = re.search(r"total-vm:(\d+)kB", kernel_text)
                    run_meta["kernel_total_vm_kib"] = int(match.group(1)) if match else None
                resources = parse_resources(resource_text, cpu_count, clk_tck)
                summary = profile_summary(name, spec, argv, seconds, spec["camera_hz"],
                                          binary_result, resources, wall_s, run_meta)
                summary["memory_budget"] = memory_budget
                if spec["terrain"] == "rmuc2027":
                    summary["cold_start"] = dict(
                        startup_seconds=(wall_s - (binary_result.get("elapsed_s") if binary_result else 0))
                        if binary_result else wall_s,
                        process_peak_rss_kib=binary_result.get("peak_rss_kib") if binary_result else None,
                        steady_rss_kib=binary_result.get("resources", [{}])[-1].get("rss_kib")
                        if binary_result and binary_result.get("resources") else None,
                        external_hwm_kib_max=resources.get("proc_hwm_kib_max"),
                        external_rss_kib_max=resources.get("proc_rss_kib_max"),
                        mem_available_kib_min=resources.get("mem_available_kib_min"),
                        kernel_oom=run_meta.get("kernel_oom"),
                        kernel_peak_anon_rss_kib=run_meta.get("kernel_peak_anon_rss_kib"),
                        kernel_total_vm_kib=run_meta.get("kernel_total_vm_kib"),
                        note="full-map terrain load: kernel high-water and OOM evidence")
            except Exception as error:  # noqa: BLE001 - 单个工况失败不阻塞其余工况
                summary = dict(profile=name, spec=spec, seconds=seconds,
                               memory_budget=memory_budget,
                               error=type(error).__name__ + ": " + str(error)[:300],
                               verification=dict(ok=False, failures=["exception"]))
                print(f"  {name}: exception {summary['error']}", flush=True)
            finally:
                cleanup_active(client, active)

            results[name] = summary
            (output / f"{name}.summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            (output / "summary.json").write_text(json.dumps(
                dict(stamp=stamp, host=args.host, cpu_count=cpu_count, clk_tck=clk_tck,
                     device=manifest["device"], memory_budget=memory_budget, build=manifest.get("build"),
                     idle=idle_resources, profiles=ordered_results(results)),
                indent=2), encoding="utf-8")
            checks = summary.get("verification", {})
            binary = summary.get("binary", {})
            device = summary.get("device", {}) or {}
            print(f"  {name}: ok={checks.get('ok')} elapsed={binary.get('elapsed_s')} "
                  f"dev_cpu%={device.get('device_cpu_percent')} proc_cpu%={binary.get('process_cpu_percent')} "
                  f"peak_rss_kib={binary.get('peak_rss_kib')} tof={checks.get('tof_coverage')} "
                  f"failures={checks.get('failures')}", flush=True)
        print("results: " + str(output), flush=True)
    finally:
        try:
            cleanup_active(client, active)
        except Exception:  # noqa: BLE001
            pass
        client.close()


if __name__ == "__main__":
    main()
