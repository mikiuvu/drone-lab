"""Real four-process SITL trial; independent truth log for evaluation only."""

import argparse
import bisect
import json
import math
import os
from pathlib import Path
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--no-arm", action="store_true")
    args = parser.parse_args()
    state = json.loads((ROOT / "runtime/manager.json").read_text())
    run = Path(state["run_dir"])
    if (run / "flight_truth.jsonl").exists():
        raise RuntimeError("Trial already recorded for this run; start a fresh world")
    env = dict(os.environ, GZ_PARTITION="drone-lab-four")
    elog = (run / "evaluator.log").open("w")
    evaluator = subprocess.Popen(
        [
            "python3",
            str(ROOT / "evaluate_linux.py"),
            "--world",
            state["map"]["name"],
            "--output",
            str(run / "flight_truth.jsonl"),
        ],
        env=env,
        stdout=elog,
        stderr=elog,
        start_new_session=True,
    )
    companions = []
    try:
        for i in range(4):
            log = (run / f"flight_companion_{i}.log").open("w")
            command = [
                "/home/devcontainers/drone-sim/build-companion/drone_companion_runner",
                "--vehicle",
                str(i),
                "--config",
                str(run / "companion_config.json"),
                "--map",
                str(ROOT / "maps/calibration.json"),
                "--runtime",
                str(run),
                "--duration",
                str(args.seconds),
                "--takeoff-height",
                "1.8",
            ]
            if not args.no_arm:
                command += ["--offboard", "--arm"]
            companions.append(
                subprocess.Popen(
                    command, stdout=log, stderr=log, start_new_session=True
                )
            )
        exit_codes = [p.wait(timeout=args.seconds + 30) for p in companions]
    finally:
        for p in companions + [evaluator]:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(timeout=5)
    truth = [
        json.loads(x) for x in (run / "flight_truth.jsonl").read_text().splitlines()
    ]
    result = {
        "evidence": "actual_gazebo_px4_sitl",
        "height_assisted": True,
        "official_map": False,
        "runtime": str(run),
        "seconds_wall": args.seconds,
        "companion_exit_codes": exit_codes,
        "vehicles": [],
    }
    for i in range(4):
        rows = [
            json.loads(x)
            for x in (run / f"companion_{i}.jsonl").read_text().splitlines()
        ]
        estimates = [r for r in rows if r.get("kind") == "estimate"]
        controls = [r for r in rows if r.get("kind") == "control_intention"]
        actual = [r for r in truth if r["vehicle"] == i]
        stamps = [r["sim_time_us"] for r in estimates]
        errors = []
        for t in actual:
            idx = bisect.bisect_left(stamps, t["sim_time_us"])
            if idx < len(estimates) and abs(stamps[idx] - t["sim_time_us"]) < 50000:
                e = estimates[idx]
                errors.append(math.hypot(e["x_m"] - t["x_m"], e["y_m"] - t["y_m"]))
        result["vehicles"].append(
            {
                "id": i,
                "truth_samples": len(actual),
                "estimates": len(estimates),
                "armed_records": sum(r.get("armed", False) for r in controls),
                "true_max_height_m": max((r["z_m"] for r in actual), default=None),
                "true_final_pose": actual[-1] if actual else None,
                "xy_rmse_m": math.sqrt(sum(e * e for e in errors) / len(errors))
                if errors
                else None,
                "guidance_records": sum(r.get("mode") == "GUIDANCE" for r in controls),
                "invalid_records": sum(
                    r.get("mode") == "HOLD_INVALID" for r in controls
                ),
            }
        )
    (run / "flight_trial.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
