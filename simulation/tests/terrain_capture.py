"""Capture actual Gazebo image and dynamic poses for operator-only inspection."""

import argparse
import json
import threading
from pathlib import Path

from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="rmuc_2027")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    node = Node()
    complete = threading.Event()
    captured = set()
    poses = {}

    def image(message):
        if (
            "image" in captured
            or len(message.data) != message.width * message.height * 3
        ):
            return
        header = chr(10).join(["P6", f"{message.width} {message.height}", "255", ""])
        (args.output / "terrain-gazebo.ppm").write_bytes(header.encode() + message.data)
        captured.add("image")
        if len(poses) == 4:
            complete.set()

    def pose(message):
        for item in message.pose:
            if item.name in {f"micro_{i}" for i in range(4)}:
                poses[item.name] = [item.position.x, item.position.y, item.position.z]
        if len(poses) == 4 and captured:
            complete.set()

    node.subscribe(Image, "/operator/terrain_overview", image)
    node.subscribe(Pose_V, f"/world/{args.world}/dynamic_pose/info", pose)
    if not complete.wait(25):
        raise RuntimeError(f"Capture timed out: image={captured}, poses={poses}")
    (args.output / "terrain-poses.json").write_text(
        json.dumps(poses, indent=2), encoding="utf-8"
    )
    print(json.dumps(poses))


if __name__ == "__main__":
    main()
