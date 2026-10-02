"""Dependency-free, read-only Linux resource sampler for later board measurements.

Run on the Nano alongside the actual workload. Not a CPU/NPU benchmark.
"""

import argparse
import json
import platform
import time
from pathlib import Path


def text(path):
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return None


def collect(pid=None):
    thermal = {
        str(p): text(p) for p in Path("/sys/class/thermal").glob("thermal_zone*/temp")
    }
    cpufreq = {
        str(p): text(p)
        for p in Path("/sys/devices/system/cpu").glob("cpu*/cpufreq/scaling_cur_freq")
    }
    item = {
        "schema": 1,
        "type": "resource_profile",
        "host": platform.node(),
        "machine": platform.machine(),
        "monotonic_ns": time.monotonic_ns(),
        "utc_ns": time.time_ns(),
        "meminfo": text("/proc/meminfo"),
        "cpu_stat": text("/proc/stat"),
        "loadavg": text("/proc/loadavg"),
        "thermal_raw": thermal,
        "cpufreq_raw": cpufreq,
        "pid": pid,
        "process_status": text(f"/proc/{pid}/status") if pid else None,
        "process_stat": text(f"/proc/{pid}/stat") if pid else None,
    }
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid", type=int)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--interval", type=float, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if (
        args.seconds <= 0
        or args.interval < 0.1
        or (args.pid is not None and args.pid <= 0)
    ):
        parser.error("positive duration/pid and interval >=0.1 required")
    if platform.system() != "Linux":
        parser.error("Run this sampler on the Linux device being profiled, not Windows")
    with args.output.open("x", encoding="utf-8") as stream:
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            stream.write(json.dumps(collect(args.pid), ensure_ascii=False) + chr(10))
            stream.flush()
            time.sleep(args.interval)
    print(
        f"Saved {args.output}. Resource readings do not measure NPU utilization or image latency."
    )


if __name__ == "__main__":
    main()
