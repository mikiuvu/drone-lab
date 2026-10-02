"""Independent evaluator; the companion never imports or reads this truth file."""

import argparse
import json
import math
import signal
import threading
from pathlib import Path

from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    output = args.output.open("x", encoding="utf-8")
    event_output = args.output.with_name(args.output.stem + "_contacts.jsonl").open(
        "x", encoding="utf-8"
    )
    config_path = args.output.parent / "map.json"
    config = (
        json.loads(config_path.read_text(encoding="utf-8"))
        if config_path.exists()
        else {}
    )
    ground_z = config.get("home_marker", {}).get("ground_z_m", 0.0)
    target = config.get("team_mission", {}).get("target")
    body_clearance = config.get("team_mission", {}).get("body_clearance_m", 0.0125)
    homes = {s["id"]: s for s in config.get("spawns", [])}
    last_pose = {}
    airborne = set()
    contact_times = {}
    ground_contact_until = {}
    mutex = threading.Lock()
    stop = threading.Event()
    last = -1

    def callback(message):
        nonlocal last
        stamp = message.header.stamp.sec * 1000000 + message.header.stamp.nsec // 1000
        if stamp - last < 20000:
            return
        last = stamp
        for pose in message.pose:
            if pose.name not in {f"micro_{i}" for i in range(4)}:
                continue
            q = pose.orientation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            row = {
                "kind": "truth_evaluation_only",
                "vehicle": int(pose.name[-1]),
                "sim_time_us": stamp,
                "x_m": pose.position.x,
                "y_m": pose.position.y,
                "z_m": pose.position.z,
                "yaw_rad": yaw,
            }
            vehicle = row["vehicle"]
            if row["z_m"] > ground_z + 0.15:
                airborne.add(vehicle)
            row["landed"] = vehicle in airborne and stamp <= ground_contact_until.get(
                vehicle, 0
            )
            last_pose[vehicle] = row
            with mutex:
                output.write(json.dumps(row) + chr(10))
                output.flush()

    def contacts(vehicle, message):
        if not message.contact:
            return
        pose = last_pose.get(vehicle)
        if pose is None:
            return
        stamp = message.header.stamp.sec * 1000000 + message.header.stamp.nsec // 1000
        ground_contact = any(
            any(abs(n.z) > 0.7 for n in contact.normal)
            and any(p.z < pose["z_m"] - body_clearance * 0.5 for p in contact.position)
            for contact in message.contact
        )
        if ground_contact:
            ground_contact_until[vehicle] = stamp + 100000
        if stamp - contact_times.get(vehicle, -1000000) < 200000:
            return
        contact_times[vehicle] = stamp
        home = homes.get(vehicle)
        row = {
            "kind": "physical_contact_evaluation",
            "vehicle": vehicle,
            "sim_time_us": stamp,
            "x_m": pose["x_m"],
            "y_m": pose["y_m"],
            "z_m": pose["z_m"],
            "enemy_base_contact": bool(
                target
                and math.dist(
                    [pose["x_m"], pose["y_m"], pose["z_m"]],
                    [target["x"], target["y"], target["z"]],
                )
                < 0.4
            ),
            "home_error_m": math.hypot(pose["x_m"] - home["x"], pose["y_m"] - home["y"])
            if home
            else None,
            "contacts": len(message.contact),
            "ground_contact": ground_contact,
        }
        with mutex:
            event_output.write(json.dumps(row) + chr(10))
            event_output.flush()

    node = Node()
    if not node.subscribe(Pose_V, f"/world/{args.world}/dynamic_pose/info", callback):
        raise RuntimeError("Truth evaluation subscription failed")
    for vehicle in range(4):
        node.subscribe(
            Contacts,
            f"/world/{args.world}/model/micro_{vehicle}/link/base_link/sensor/body_contact/contact",
            lambda msg, i=vehicle: contacts(i, msg),
        )
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    while not stop.wait(0.2):
        pass
    node.unsubscribe(f"/world/{args.world}/dynamic_pose/info")
    for vehicle in range(4):
        node.unsubscribe(
            f"/world/{args.world}/model/micro_{vehicle}/link/base_link/sensor/body_contact/contact"
        )
    with mutex:
        output.close()
        event_output.close()


if __name__ == "__main__":
    main()
