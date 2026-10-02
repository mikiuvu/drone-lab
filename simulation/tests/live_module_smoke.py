"""Verify live processed-flow, ToF, peer and anchor observation streams."""

import argparse
import json
import select
import socket
import time
from collections import Counter
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle-count", type=int, required=True)
    parser.add_argument("--anchors", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    sockets = []
    counts = Counter()
    peers = set()
    anchors = set()
    forbidden = []
    try:
        for i in range(args.vehicle_count):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("127.0.0.1", 16000 + i))
            s.setblocking(False)
            sockets.append(s)
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            readable, _, _ = select.select(sockets, [], [], 0.2)
            for s in readable:
                row = json.loads(s.recv(65535))
                i = row["vehicle"]
                assert 0 <= i < args.vehicle_count
                kind = row["kind"]
                counts[i, kind] += 1
                if kind == "uwb":
                    peers.add((i, row["peer"]))
                if kind == "uwb_anchor":
                    anchors.add((i, row["anchor_id"]))
                if kind == "optical_flow":
                    assert 0 <= row["quality"] <= 1 and row["variance_m2s2"] > 0
                    assert abs(row["vx_body_mps"]) < 1 and abs(row["vy_body_mps"]) < 1
                    forbidden.extend(
                        k for k in ("x_m", "y_m", "yaw_rad", "z_world_m") if k in row
                    )
        assert not forbidden
        for i in range(args.vehicle_count):
            for kind in ("tof", "height_assist", "optical_flow"):
                assert counts[i, kind] >= 10, (i, kind, counts[i, kind])
        assert len(peers) == args.vehicle_count * (args.vehicle_count - 1)
        assert len(anchors) == (args.vehicle_count * 4 if args.anchors else 0)
        report = {
            "vehicles": args.vehicle_count,
            "anchors_enabled": args.anchors,
            "sensor_source": "live Gazebo module publisher",
            "counts": {f"{i}:{kind}": v for (i, kind), v in counts.items()},
            "directed_peer_links": len(peers),
            "anchor_links": len(anchors),
            "passed": True,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps({k: v for k, v in report.items() if k != "counts"}))
    finally:
        for s in sockets:
            s.close()


if __name__ == "__main__":
    main()
