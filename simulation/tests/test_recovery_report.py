"""Recovery reports reject restart requests before the landing phase."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import tempfile

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("recovery_report", ROOT / "scripts/summarize_recovery_ulog.py")
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class RecoveryReportTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name)
        (self.run / "map.json").write_text(json.dumps({"spawns": [{"id": 0}]}), encoding="utf-8")
        log = self.run / "px4_0/log"
        log.mkdir(parents=True)
        (log / "test.ulg").touch()
        self.native = {
            "vehicle_status": {"timestamp": np.array([1, 6_000_000, 12_000_000]),
                               "arming_state": np.array([1, 2, 1])},
            "vehicle_land_detected": {"timestamp": np.array([1, 7_000_000, 11_000_000]),
                                      "landed": np.array([1, 0, 1])},
        }
        self.loader = patch.object(REPORT, "load_segments", side_effect=lambda paths: (
            self.native, [(paths[0], SimpleNamespace(dropouts=[], logged_messages=[]))],
            [{"path": str(paths[0]), "sha256": "test"}], np.array([])))
        self.loader.start()
        self.addCleanup(self.loader.stop)

    def write_rows(self, name, rows):
        (self.run / name).write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")

    def report(self, recovery="ABORT_RETURN", restart=None, armed=True):
        self.write_rows("team_mission.jsonl", [
            {"phase": "TAKEOFF", "sim_time_us": 5_000_000},
            {"phase": recovery, "sim_time_us": 8_000_000},
            {"phase": "LAND_ALL", "sim_time_us": 10_000_000},
            {"phase": "ABORTED", "sim_time_us": 13_000_000},
        ])
        requests = [{"kind": "command_request", "command": "ARM", "sim_time_us": 5_000_000}]
        if restart is not None:
            requests.append({"kind": "command_request", "command": restart, "sim_time_us": 9_000_000})
        requests += [{"kind": "command_request", "command": "LAND", "sim_time_us": 10_000_000},
                     {"kind": "stop", "armed_at_stop": False, "sim_time_us": 14_000_000}]
        self.write_rows("companion_0.jsonl", requests)
        if not armed:
            self.native["vehicle_status"]["arming_state"][:] = 1
            self.native["vehicle_land_detected"]["landed"][:] = 1
        return REPORT.summarize(self.run, self.run)

    def require_stopped(self):
        argv = ["report", str(self.run), "--ulog-root", str(self.run),
                "--output", str(self.run / "recovery.json"), "--require-stopped"]
        with patch("sys.argv", argv), contextlib.redirect_stdout(io.StringIO()):
            REPORT.main()

    def test_abort_restart_before_landing_is_rejected(self):
        for command in ("ARM", "OFFBOARD"):
            with self.subTest(command=command):
                result = self.report(restart=command)
                self.assertTrue(result["all_stopped"])
                self.assertEqual(result["vehicles"][0]["arm_or_offboard_after_landing"], [])
                self.assertEqual(result["vehicles"][0]["arm_or_offboard_after_recovery"][0]["command"], command)
                with self.assertRaises(SystemExit):
                    self.require_stopped()

    def test_normal_return_restart_is_rejected(self):
        self.report(recovery="RETURN_ALL", restart="ARM")
        with self.assertRaises(SystemExit):
            self.require_stopped()

    def test_takeoff_and_native_shutdown_pass(self):
        result = self.report()
        self.assertEqual(result["recovery_start_time_s"], 8.0)
        self.assertEqual(result["vehicles"][0]["first_armed_time_s"], 6.0)
        self.assertEqual(result["vehicles"][0]["first_airborne_time_s"], 7.0)
        self.require_stopped()

    def test_grounded_member_has_no_native_arm_or_airborne_time(self):
        result = self.report(armed=False)
        self.assertIsNone(result["vehicles"][0]["first_armed_time_s"])
        self.assertIsNone(result["vehicles"][0]["first_airborne_time_s"])
        self.require_stopped()

    def test_native_armed_at_end_is_rejected(self):
        self.report()
        self.native["vehicle_status"]["arming_state"][-1] = 2
        with self.assertRaises(SystemExit):
            self.require_stopped()


if __name__ == "__main__":
    unittest.main()
