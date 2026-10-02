import json
import tempfile
import unittest
from pathlib import Path

from simulation.localization_metrics import LocalizationMetrics


def row(kind, vehicle, stamp, x, y, **extra):
    return {
        "kind": kind,
        "vehicle": vehicle,
        "sim_time_us": stamp,
        "x_m": x,
        "y_m": y,
        **extra,
    }


class VariableFleetMetricsTests(unittest.TestCase):
    def _write_run(self, root, count):
        truth = [
            row("truth_evaluation_only", i, 1_000_000, float(i), 0.0)
            for i in range(count)
        ]
        (root / "truth_evaluation_only.jsonl").write_text(
            "".join(json.dumps(item) + "\n" for item in truth), encoding="utf-8"
        )
        for i in range(count):
            estimate = row(
                "estimate",
                i,
                1_000_000,
                float(i) + 1.0,
                2.0,
                source="ESTIMATE",
                xy_valid=True,
                xy_structural_valid=True,
                xy_peer_valid=True,
                flow_valid=True,
                position_confidence=0.8,
                localization_mode="continuousodom",
            )
            (root / f"companion_{i}.jsonl").write_text(
                json.dumps(estimate) + "\n", encoding="utf-8"
            )

    def test_n1_report_has_one_vehicle_and_no_pairs(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self._write_run(root, 1)
            report = LocalizationMetrics(root, vehicle_count=1).poll()
            self.assertEqual(len(report["vehicles"]), 1)
            self.assertEqual(report["pairs"], [])
            self.assertAlmostEqual(
                report["common_translation"]["magnitude_m"], 2.23606797749979
            )
            self.assertEqual(
                report["vehicles"][0]["localization_mode"], "continuousodom"
            )

    def test_n6_report_has_fifteen_pairs_and_common_translation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self._write_run(root, 6)
            report = LocalizationMetrics(root, vehicle_count=6).poll()
            self.assertEqual(len(report["vehicles"]), 6)
            self.assertEqual(len(report["pairs"]), 15)
            self.assertAlmostEqual(
                report["common_translation"]["magnitude_m"], 2.23606797749979
            )
            self.assertTrue(
                all(item["flow_validity"] == 1 for item in report["vehicles"])
            )


try:
    from PySide6.QtWidgets import QApplication

    from app.simulation_panel import GazeboPanel
except ModuleNotFoundError:
    QApplication = None
    GazeboPanel = None


@unittest.skipUnless(QApplication is not None, "PySide6 is required for UI checks")
class VariableFleetUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_camera_count_follows_n1_and_n6(self):
        with tempfile.TemporaryDirectory() as folder:
            panel = GazeboPanel(Path(folder))
            panel.vehicle_count.setValue(1)
            self.assertEqual(len(panel.cameras), 1)
            panel.vehicle_count.setValue(6)
            self.assertEqual(len(panel.cameras), 6)
            panel.close()


if __name__ == "__main__":
    unittest.main()
