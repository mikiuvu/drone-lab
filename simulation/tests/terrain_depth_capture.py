"""Diagnostic only: static actual engine depth and pose, never an estimator input."""

import json
import os
import struct
import threading
from pathlib import Path

from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

os.environ["GZ_PARTITION"] = "drone-lab-four"
node = Node()
data = {"depth": {}, "poses": {}}
ready = threading.Event()


def depth(vehicle, sensor, message):
    if message.width == 8 and len(message.data) == 256:
        data["depth"][f"{vehicle}_{sensor}"] = list(struct.unpack("<64f", message.data))
        if len(data["depth"]) == 12 and len(data["poses"]) == 4:
            ready.set()


def pose(message):
    for item in message.pose:
        if item.name in {f"micro_{i}" for i in range(4)}:
            data["poses"][item.name[-1]] = {
                "xyz": [item.position.x, item.position.y, item.position.z],
                "q_wxyz": [
                    item.orientation.w,
                    item.orientation.x,
                    item.orientation.y,
                    item.orientation.z,
                ],
            }


for v in range(4):
    for s in range(3):
        node.subscribe(
            Image,
            f"/simulation/vehicle_{v}/tof_{s}/depth_image",
            lambda msg, v=v, s=s: depth(v, s, msg),
        )
node.subscribe(Pose_V, "/world/rmuc_2026_sentry26/dynamic_pose/info", pose)
if not ready.wait(20):
    raise RuntimeError("Depth capture missing streams")
out = Path(__file__).resolve().parents[1] / "docs/terrain-depth-static.json"
out.write_text(json.dumps(data), encoding="utf-8")
print(str(out))
