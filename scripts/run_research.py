"""Reproduce the PC pre-research bundle; no hardware or network access."""

import argparse
import hashlib
import importlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
if sys.stdout is not None:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=Path, default=ROOT / "research" / "results" / "baseline.json"
    )
    args = parser.parse_args()
    specifications = [
        ("bandwidth_400khz", "bandwidth", "run_bandwidth", {}),
        ("control_faults", "control_policy", "run_control", {}),
        ("compute_baseline", "compute", "run_compute", {}),
        (
            "compute_overload",
            "compute",
            "run_compute",
            {"npu_ms": 60, "cpu_pre_ms": 6, "cpu_post_ms": 4},
        ),
        ("geometry_horizontal", "geometry", "run_geometry", {}),
        ("geometry_downward", "geometry", "run_geometry", {"mount_pitch_deg": -30}),
        ("integration_50ms", "integration", "run_integration", {}),
    ]
    runs = {}
    for name, module_name, function, config in specifications:
        module = importlib.import_module("research." + module_name)
        source = Path(module.__file__)
        data = getattr(module, function)(config)
        runs[name] = {
            "input_override": config,
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "source": source.relative_to(ROOT).as_posix(),
            "result": data,
        }
        from research.plots import save_plot

        plot_folder = ROOT / "docs" / "research" / "figures"
        plot_folder.mkdir(parents=True, exist_ok=True)
        save_plot(module_name, data, plot_folder / (name + ".png"))
        print(name + ": " + data["summary"])
    payload = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "evidence_type": "offline_simulation",
        "python": sys.version,
        "host_arch": platform.machine(),
        "runs": runs,
        "warning": "Offline simulation with configured service times, noise and geometry.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    print("Saved " + str(args.output))


if __name__ == "__main__":
    main()
