"""Status export failures must not stop the mission command stream."""

import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest


BINARY = Path(os.environ.get(
    "TEAM_COORDINATOR_BIN",
    "/home/devcontainers/drone-sim/build-companion/team_coordinator",
))


@unittest.skipUnless(BINARY.is_file(), "team_coordinator binary required")
class CoordinatorIOTest(unittest.TestCase):
    def test_status_export_failure_recovers_without_stopping_commands(self):
        with tempfile.TemporaryDirectory() as directory, socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        ) as receiver:
            root = Path(directory)
            receiver.bind(("127.0.0.1", 18000))
            receiver.settimeout(3)
            config = {
                "spawns": [{"id": 0, "x": 0, "y": 0}],
                "team_mission": {
                    "assembly": [{"id": 0, "x": 0, "y": 1}],
                    "approach": {"x": 0, "y": 2},
                    "target": {"x": 0, "y": 2.2},
                    "order": [0],
                },
            }
            path = root / "map.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            status = root / "team_status.json"
            status.mkdir()
            with (root / "stderr.log").open("w", encoding="utf-8") as errors:
                process = subprocess.Popen(
                    [str(BINARY), "--map", str(path), "--runtime", str(root)],
                    stdout=subprocess.DEVNULL, stderr=errors,
                )
                try:
                    sequences = []
                    for _ in range(5):
                        packet = json.loads(receiver.recv(8192))
                        sequences.append(packet["sequence"])
                    self.assertEqual(sequences, sorted(set(sequences)))
                    self.assertIsNone(process.poll())
                    status.rmdir()
                    deadline = time.monotonic() + 3
                    while not status.is_file() and time.monotonic() < deadline:
                        time.sleep(0.02)
                    exported = json.loads(status.read_text(encoding="utf-8"))
                    self.assertGreater(exported["telemetry_errors"], 0)
                    self.assertIsNone(process.poll())
                finally:
                    process.terminate()
                    process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
