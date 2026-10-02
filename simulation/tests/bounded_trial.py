"""Bounded real SITL trial, supervised by simulation clock (no pose injection)."""

import argparse
import json
import os
import subprocess
import threading
from pathlib import Path

from gz.msgs10.clock_pb2 import Clock
from gz.transport13 import Node

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--sim-seconds", type=float, default=30)
parser.add_argument("--vehicle-count",type=int)
parser.add_argument("--uwb-anchors",action="store_true")
parser.add_argument("--flow-disable",action="store_true")
parser.add_argument(
    "--mission", choices=["hover", "survey", "team_contact"], default="hover"
)
args = parser.parse_args()
if not 5 <= args.sim_seconds <= 600:
    parser.error("sim-seconds must be between 5 and 600")
os.environ["GZ_PARTITION"] = "drone-lab-four"
command = [
        "python3",
        str(root / "launch_linux.py"),
        "start",
        "--gui",
        "--companions",
        "--fly",
        "--horizontal-tof",
        "--mission",
        args.mission,
    ]
if args.vehicle_count is not None:
    command += ["--vehicle-count",str(args.vehicle_count)]
if args.uwb_anchors:
    command += ["--uwb-anchors"]
if args.flow_disable:
    command += ["--flow-disable"]
subprocess.run(command,check=True,stdout=subprocess.DEVNULL)
state = json.loads((root / "runtime/manager.json").read_text(encoding="utf-8"))
done = threading.Event()
last = [0.0]
node = Node()


def clock(message):
    last[0] = message.sim.sec + message.sim.nsec / 1e9
    if last[0] >= args.sim_seconds:
        done.set()


try:
    if not node.subscribe(Clock, f"/world/{state['map']['name']}/clock", clock):
        raise RuntimeError("Clock unavailable")
    import time

    deadline = time.monotonic() + args.sim_seconds * 4 + 20
    aborted = False
    complete_seen = None
    while not done.wait(0.2):
        status_path = Path(state["run_dir"]) / "team_status.json"
        if args.mission == "team_contact" and status_path.exists():
            try:
                team = json.loads(status_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            if team.get("aborted") and team.get("paused"):
                aborted = True
                break
            if team.get("complete"):
                complete_seen = complete_seen or time.monotonic()
                if time.monotonic() - complete_seen > 2.0:
                    break
        if time.monotonic() > deadline:
            raise RuntimeError("Simulation stalled")
finally:
    active = json.loads((root / "runtime/manager.json").read_text(encoding="utf-8"))
    if active.get("run_dir") == state["run_dir"]:
        subprocess.run(["python3", str(root / "launch_linux.py"), "stop"], check=True)
run = Path(state["run_dir"])
report = {
    "run_dir": str(run),
    "sim_seconds": last[0],
    "mission": args.mission,
    "vehicle_count":len(state["map"]["spawns"]),
    "uwb_anchor_count":len(state["map"].get("uwb_anchors",[])),
    "real_physics": True,
    "height_assisted": True,
    "flight_success": "requires metric review",
    "stopped_on_team_failure": aborted,
}
(run / "bounded_trial.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report))
