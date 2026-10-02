"""Send simulated world height to four SITL EKFs for height-fusion verification.

Requires launcher --vertical-only and no companions using ports 16000/14540.
The adapter sends height observations without arming or motion commands.
"""

import argparse
import json
import select
import socket
import time
import os
import subprocess
import threading
import re
from pathlib import Path

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    # Refuse to inject placeholder x/y if the live EKF would fuse them.
    param = "/home/devcontainers/drone-sim/PX4-Autopilot/build/px4_sitl_default/bin/px4-param"
    for vehicle in range(4):
        check = subprocess.run(
            [param, "--instance", str(vehicle), "show", "EKF2_EV_CTRL"],
            capture_output=True,
            timeout=5,
        )
        if check.returncode or not re.search(
            r"EKF2_EV_CTRL.*:\s*2\b", check.stdout.decode()
        ):
            raise RuntimeError(
                "Height-only diagnostic requires EKF2_EV_CTRL=2 on all instances"
            )
    observations = []
    links = []
    expected = {}
    measurements = {}
    counts = {i: 0 for i in range(4)}
    for i in range(4):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 16000 + i))
        sock.setblocking(False)
        observations.append(sock)
        links.append(
            mavutil.mavlink_connection(
                f"udpout:127.0.0.1:{14580 + i}",
                source_system=240 + i,
                source_component=197,
            )
        )
    started = time.monotonic()
    last = {i: 0.0 for i in range(4)}
    snapshots = {}
    snapshot_thread = None

    def inspect_filters():
        px4 = "/home/devcontainers/drone-sim/PX4-Autopilot/build/px4_sitl_default/bin/px4-listener"
        for vehicle in range(4):
            results = {}
            for topic in (
                "estimator_status_flags",
                "vehicle_local_position",
                "vehicle_visual_odometry",
            ):
                proc = subprocess.run(
                    [px4, "--instance", str(vehicle), topic, "-n", "1"],
                    capture_output=True,
                    timeout=5,
                )
                results[topic] = proc.stdout.decode("utf-8", "replace")
            snapshots[vehicle] = results

    try:
        while time.monotonic() - started < args.seconds:
            if (
                snapshot_thread is None
                and time.monotonic() - started > args.seconds / 2
            ):
                snapshot_thread = threading.Thread(target=inspect_filters)
                snapshot_thread.start()
            ready, _, _ = select.select(observations, [], [], 0.02)
            for sock in ready:
                i = observations.index(sock)
                while True:
                    try:
                        packet, _ = sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    data = json.loads(packet)
                    if data.get("kind") != "height_assist":
                        continue
                    if set(data) != {
                        "schema",
                        "kind",
                        "vehicle",
                        "sim_time_us",
                        "z_world_m",
                        "source",
                    }:
                        raise RuntimeError("Height truth boundary violation")
                    assert (
                        data["source"] == "SIM_HEIGHT_ASSIST" and data["vehicle"] == i
                    )
                    z = -float(data["z_world_m"])
                    covariance = [0.0] * 21
                    # This diagnostic requires EV_CTRL=2: x/y and attitude are
                    # unobserved placeholders and MUST NOT be fused. PX4 v1.15
                    # rotates the position covariance using q_error, so huge
                    # unused x/y/RP variances would contaminate the height axis.
                    for index in (0, 6, 11, 15, 18, 20):
                        covariance[index] = 0.0001
                    links[i].mav.vision_position_estimate_send(
                        data["sim_time_us"], 0, 0, z, 0, 0, 0, covariance, 0
                    )
                    expected[i] = z
                    counts[i] += 1
            for i, link in enumerate(links):
                now = time.monotonic()
                if now - last[i] > 1:
                    link.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        4,
                    )
                    link.mav.command_long_send(
                        i + 1,
                        1,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0,
                        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                        100000,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                    last[i] = now
                for _ in range(100):
                    msg = link.recv_match(blocking=False)
                    if msg is None:
                        break
                    if msg.get_type() == "LOCAL_POSITION_NED":
                        measurements[i] = float(msg.z)
        if snapshot_thread:
            snapshot_thread.join(timeout=10)
        report = {
            "height_assist_only": True,
            "armed": False,
            "sent_counts": counts,
            "sent_ned_z": expected,
            "reported_fc_z": measurements,
            "filter_snapshots": snapshots,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(
            json.dumps(
                {k: v for k, v in report.items() if k != "filter_snapshots"}, indent=2
            )
        )
        if min(counts.values()) < 10:
            raise RuntimeError("Missing height observations")
        for vehicle in range(4):
            flags = snapshots[vehicle]["estimator_status_flags"]
            assert re.search(r"cs_ev_hgt:\s+True", flags), (
                "PX4 not fusing assisted height"
            )
            assert re.search(r"cs_gps:\s+False", flags), "GPS must remain disabled"
            assert re.search(r"cs_ev_pos:\s+False", flags), (
                "Height-only adapter cannot be used with horizontal EV fusion"
            )
            assert re.search(r"cs_ev_yaw:\s+False", flags), (
                "Height-only adapter cannot be used with EV yaw fusion"
            )
            position = snapshots[vehicle]["vehicle_local_position"]
            assert re.search(r"z_valid:\s+True", position), "Height not valid"
            z = float(re.search(r"\n\s+z:\s+([-+0-9.eE]+)", position).group(1))
            assert abs(z - expected[vehicle]) < 0.1, (vehicle, z, expected[vehicle])
        print(
            "PASS all four PX4 filters fuse height assist; GPS disabled; height error <0.1m at sample"
        )
    finally:
        for sock in observations:
            sock.close()
        for link in links:
            link.close()


if __name__ == "__main__":
    main()
