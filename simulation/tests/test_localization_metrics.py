import json
import tempfile
import unittest
import os
import time
import contextlib
import io
from pathlib import Path

from simulation.localization_metrics import JsonlTail, LocalizationMetrics, map_bounds, main


def write_rows(path, rows):
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")


def row(kind, vehicle, stamp, x, y, **extra):
    return {"kind": kind, "vehicle": vehicle, "sim_time_us": stamp, "x_m": x, "y_m": y, **extra}


class LocalizationMetricsTests(unittest.TestCase):
    def test_tail_is_incremental_and_bounded(self):
        with tempfile.TemporaryDirectory() as folder:
            tmp_path = Path(folder)
            path = tmp_path / "tail.jsonl"
            write_rows(path, [row("x", 0, i, i, 0) for i in range(20)])
            tail = JsonlTail(path, budget=120)
            first = tail.read()
            second = tail.read()
            self.assertTrue(first)
            self.assertFalse(second)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(row("x", 0, 21, 21, 0)) + "\n")
            self.assertEqual(tail.read()[0]["sim_time_us"], 21)

    def test_metrics_distinguish_travel_and_compute_errors(self):
        with tempfile.TemporaryDirectory() as folder:
            tmp_path = Path(folder)
            truth = [row("truth_evaluation_only", 0, 1_000_000, 0, 0), row("truth_evaluation_only", 0, 2_000_000, 2, 0)]
            estimates = [row("estimate", 0, 1_000_000, 0.1, 0, source="ESTIMATE", xy_valid=True, xy_structural_valid=True, xy_peer_valid=False), row("estimate", 0, 2_000_000, 2.1, 0, source="ESTIMATE", xy_valid=True, xy_structural_valid=True, xy_peer_valid=True)]
            write_rows(tmp_path / "truth_evaluation_only.jsonl", truth)
            write_rows(tmp_path / "companion_0.jsonl", estimates)
            metrics = LocalizationMetrics(tmp_path)
            snapshot = metrics.poll(now=0.0)
            vehicle = snapshot["vehicles"][0]
            self.assertEqual(vehicle["trajectory"], "traveled")
            self.assertEqual(vehicle["matched_samples"], 2)
            self.assertGreater(vehicle["rmse_m"], 0)
            self.assertEqual(vehicle["validity"]["valid"], 1)
            self.assertEqual(vehicle["validity"]["peer"], 0.5)

    def test_bounds_prefer_explicit_contract(self):
        self.assertEqual(map_bounds({"bounds": {"x_min": -1, "x_max": 2, "y_min": -3, "y_max": 4}}), (-1.0, 2.0, -3.0, 4.0))
        self.assertEqual(map_bounds({"walls": [{"x1": -2, "x2": 2, "y1": -1, "y2": 1}]}), (-2.0, 2.0, -1.0, 1.0))

    def test_future_estimate_waits_then_aligns_without_losing_initial_matches(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            truth_path = root / "truth_evaluation_only.jsonl"
            write_rows(truth_path, [row("truth_evaluation_only", 0, 1000000, 0, 0)])
            write_rows(root / "companion_0.jsonl", [
                row("estimate", 0, 1000000, 1, 0, source="ESTIMATE", xy_valid=True),
                row("estimate", 0, 1100000, 3, 0, source="ESTIMATE", xy_valid=True)])
            metrics = LocalizationMetrics(root)
            self.assertEqual(metrics.poll()["vehicles"][0]["matched_samples"], 1)
            with truth_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(row("truth_evaluation_only", 0, 1120000, 2, 0)) + "\n")
            result = metrics.poll()["vehicles"][0]
            self.assertEqual(result["matched_samples"], 2)
            self.assertEqual(result["rmse_m"], 1)
            self.assertEqual(result["match"]["truth_stamp"], 1120000)

    def test_old_mtime_stopped_and_dead_process_are_not_live(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            write_rows(root / "truth_evaluation_only.jsonl", [row("truth_evaluation_only", 0, 1000000, 0, 0)])
            write_rows(root / "companion_0.jsonl", [row("estimate", 0, 1000000, 0, 0, source="ESTIMATE")])
            old = time.time() - 60
            for path in root.iterdir():
                os.utime(path, (old, old))
            report = LocalizationMetrics(root).poll()
            self.assertEqual(report["vehicles"][0]["status"], "stale")
            self.assertGreater(report["vehicles"][0]["ages_s"]["estimate"], 59)
            self.assertEqual(LocalizationMetrics(root, manager={"stopped": True}).poll()["vehicles"][0]["status"], "stopped")
            self.assertEqual(LocalizationMetrics(root, manager={"pid_status": {"companion_0": False}}).poll()["vehicles"][0]["status"], "stale")

    def test_invalid_estimates_still_show_pairs_and_common_drift(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            write_rows(root / "truth_evaluation_only.jsonl", [row("truth_evaluation_only", i, 1000000, i, 0) for i in range(4)])
            for i in range(4):
                write_rows(root / f"companion_{i}.jsonl", [row("estimate", i, 1000000, i + 3, 4, source="ESTIMATE", xy_valid=False)])
            report = LocalizationMetrics(root, manager={"stopped": True}).poll()
            self.assertEqual(len(report["pairs"]), 6)
            self.assertTrue(all(p["distance_error_m"] == 0 and not p["valid"] for p in report["pairs"]))
            self.assertEqual(report["common_translation"]["magnitude_m"], 5)
            self.assertFalse(report["common_translation"]["valid"])

    def test_full_stream_covers_multiple_chunks_and_compact_cli(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            n = 4000
            write_rows(root / "truth_evaluation_only.jsonl", [row("truth_evaluation_only", 0, i * 20000, i / 1000, 0) for i in range(n)])
            write_rows(root / "companion_0.jsonl", [row("estimate", 0, i * 20000, i / 1000 + 2, 0, source="ESTIMATE", xy_valid=False) for i in range(n)])
            report = LocalizationMetrics(root, full=True).read_full()
            self.assertEqual(report["vehicles"][0]["matched_samples"], n)
            self.assertAlmostEqual(report["vehicles"][0]["rmse_m"], 2)
            self.assertNotIn("trails", report)
            self.assertEqual(report["skipped_bytes"], 0)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                main(["--run-dir", str(root), "--json", "--full"])
            self.assertNotIn("trails", json.loads(output.getvalue()))


if __name__ == "__main__":
    unittest.main()
