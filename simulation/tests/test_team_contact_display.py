import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from app.simulation_panel import GazeboPanel, LocalizationMap
from simulation.localization_metrics import (
    LocalizationMetrics,
    home_positions,
    read_status,
)

HOMES = [
    {"id": i, "x": p[0], "y": p[1], "z": 0.55}
    for i, p in enumerate([(-0.6, -0.2), (-0.2, -0.6), (0.2, 0.6), (0.6, 0.2)])
]
CONFIG = {
    "spawns": HOMES,
    "home_marker": {"size_m": 0.30, "line_width_m": 0.012, "ground_z_m": 0.5},
    "bounds": {"x_min": -14.5, "x_max": 14.5, "y_min": -8, "y_max": 8},
}


def append(path, rows):
    with path.open("a", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")


def samples(root, stamp, dx=0.3, dy=0.4, **truth_extra):
    for home in HOMES:
        i = home["id"]
        base = {"vehicle": i, "sim_time_us": stamp}
        append(
            root / "truth_evaluation_only.jsonl",
            [
                dict(
                    base,
                    kind="truth_evaluation_only",
                    x_m=home["x"] + dx,
                    y_m=home["y"] + dy,
                    z_m=0.52,
                    **truth_extra,
                )
            ],
        )
        append(
            root / f"companion_{i}.jsonl",
            [
                dict(
                    base,
                    kind="estimate",
                    source="ESTIMATE",
                    x_m=home["x"] + dx + 0.1,
                    y_m=home["y"] + dy,
                    xy_valid=True,
                )
            ],
        )


class TeamHomeMetricsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.metrics = LocalizationMetrics(self.root, homes=HOMES)

    def team(self, **value):
        (self.root / "team_status.json").write_text(json.dumps(value), encoding="utf-8")

    def test_four_signed_offsets_and_truth_height(self):
        samples(self.root, 1_000_000)
        report = self.metrics.poll()
        for v in report["vehicles"]:
            home = v["home_error"]
            self.assertAlmostEqual(home["truth"]["xy_m"], 0.5)
            self.assertAlmostEqual(home["estimate"]["dx_m"], 0.4)
            self.assertAlmostEqual(home["estimate"]["dy_m"], 0.4)
            self.assertEqual(home["truth_z_m"], 0.52)
            self.assertEqual(home["label"], "realtime_home_offset")
            self.assertIsNone(home["landing"])

    def test_initial_home_and_landed_truth_are_not_return(self):
        samples(self.root, 1_000_000, dx=0, dy=0, landed=True)
        self.team(phase="TAKEOFF", completed=[])
        home = self.metrics.poll()["vehicles"][0]["home_error"]
        self.assertEqual(home["label"], "realtime_home_offset")
        self.assertIsNone(home["landing"])

    def test_complete_uses_final_matched_truth_without_claiming_touchdown(self):
        samples(self.root, 1_000_000)
        self.team(phase="COMPLETE", completed=[0, 1, 2, 3])
        home = self.metrics.poll()["vehicles"][0]["home_error"]
        self.assertEqual(home["label"], "final_matched_home_error")
        self.assertIsNone(home["landing"])
        self.assertEqual(home["final"]["truth_stamp_us"], 1_000_000)

    def test_transit_ground_contact_is_not_return_landing(self):
        samples(self.root, 1_000_000, dx=0, dy=0)
        self.metrics.poll()
        samples(self.root, 2_000_000, dx=1, dy=0, landed=True)
        self.team(phase="TRANSIT")
        self.assertIsNone(self.metrics.poll()["vehicles"][0]["home_error"]["landing"])

    def test_complete_does_not_relabel_obsolete_match(self):
        samples(self.root, 1_000_000)
        self.metrics.poll()
        append(
            self.root / "truth_evaluation_only.jsonl",
            [
                {
                    "kind": "truth_evaluation_only",
                    "vehicle": 0,
                    "sim_time_us": 2_000_000,
                    "x_m": 10,
                    "y_m": 10,
                }
            ],
        )
        self.team(phase="COMPLETE")
        home = self.metrics.poll()["vehicles"][0]["home_error"]
        self.assertEqual(home["label"], "realtime_home_offset")

    def test_delayed_contact_landing_keeps_historical_truth_and_estimate(self):
        samples(self.root, 1_000_000)
        self.metrics.poll()
        samples(self.root, 2_000_000, dx=2, dy=2)
        append(
            self.root / "companion_0.jsonl",
            [
                {
                    "kind": "contact_final",
                    "vehicle": 0,
                    "sim_time_us": 1_000_000,
                    "source": "ESTIMATE",
                    "landed": True,
                }
            ],
        )
        home = self.metrics.poll()["vehicles"][0]["home_error"]
        self.assertEqual(home["label"], "final_landing_error")
        self.assertAlmostEqual(home["landing"]["truth"]["xy_m"], 0.5)
        self.assertAlmostEqual(home["landing"]["estimate"]["dx_m"], 0.4)
        samples(self.root, 3_000_000, dx=3, dy=3)
        self.assertEqual(
            self.metrics.poll()["vehicles"][0]["home_error"]["landing"], home["landing"]
        )

    def test_contact_event_waits_for_nearest_truth(self):
        append(
            self.root / "companion_0.jsonl",
            [
                {
                    "kind": "contact_final",
                    "vehicle": 0,
                    "sim_time_us": 1_000_000,
                    "source": "ESTIMATE",
                    "landed": True,
                }
            ],
        )
        self.assertIsNone(self.metrics.poll()["vehicles"][0]["home_error"]["landing"])
        samples(self.root, 1_020_000)
        landing = self.metrics.poll()["vehicles"][0]["home_error"]["landing"]
        self.assertEqual(landing["truth_stamp_us"], 1_020_000)
        self.assertIsNotNone(landing["estimate"])

    def test_nonlanded_contact_final_and_low_height_do_not_mark_landing(self):
        samples(self.root, 1_000_000)
        append(
            self.root / "companion_0.jsonl",
            [
                {
                    "kind": "contact_final",
                    "vehicle": 0,
                    "sim_time_us": 1_000_000,
                    "source": "ESTIMATE",
                    "landed": False,
                }
            ],
        )
        self.assertIsNone(self.metrics.poll()["vehicles"][0]["home_error"]["landing"])

    def test_backward_compatible_constructor_and_set_homes(self):
        samples(self.root, 1_000_000)
        metrics = LocalizationMetrics(self.root)
        report = metrics.poll()
        self.assertIsNone(report["vehicles"][0]["home_error"])
        metrics.set_homes(HOMES)
        self.assertIsNotNone(metrics.snapshot()["vehicles"][0]["home_error"])
        self.assertEqual(metrics.snapshot()["vehicles"][0]["matched_samples"], 1)

    def test_status_is_bounded_and_rejects_other_run(self):
        self.team(phase="COMPLETE", run_dir=str(self.root / "other"))
        self.assertEqual(self.metrics.poll()["team_status"], {})
        (self.root / "team_status.json").write_text(" " * 65537, encoding="utf-8")
        self.assertEqual(read_status(self.root / "team_status.json"), {})

    def test_invalid_home_records_are_ignored(self):
        self.assertEqual(
            home_positions([None, {"id": 0, "x": float("nan"), "y": 0, "z": 0}]), {}
        )

    def test_home_metadata_and_run_status_take_precedence(self):
        shared = self.root / "shared.json"
        shared.write_text(json.dumps({"phase": "CONTACT"}), encoding="utf-8")
        self.team(phase="RETURN_ALL")
        metrics = LocalizationMetrics(
            self.root, homes=HOMES, team_status_path=shared, manager={"map": CONFIG}
        )
        report = metrics.poll()
        self.assertEqual(report["team_status"]["phase"], "RETURN_ALL")
        self.assertEqual(report["homes"], HOMES)
        self.assertEqual(report["home_marker"]["size_m"], 0.30)
        self.assertEqual(report["home_error_reference"], "configured_spawn_world_xy")


class TeamContactQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.panel = GazeboPanel(self.root)
        self.panel.timer.stop()
        self.addCleanup(self.panel.close)

    def test_default_mission_and_command_arguments(self):
        self.assertEqual(self.panel.mission_choice.currentData(), "team_contact")
        self.assertEqual(self.panel.mission_choice.currentText(), "多机轮流撞击与返航")
        with patch("app.simulation_panel.QProcess") as process:
            self.panel.command("start")
            args = process.return_value.start.call_args.args[1]
            self.assertEqual(args[args.index("--mission") + 1], "team_contact")
            self.assertEqual(args[args.index("--dynamics") + 1], "micro")
            self.assertEqual(args[args.index("--vehicle-count") + 1], "4")
            self.assertEqual(args[args.index("--vision-delay-ms") + 1], "0")
            self.assertNotIn("--uwb-anchors", args)
            self.assertIn("--fly", args)

    def test_observation_mode_does_not_request_flight(self):
        self.panel.fly.setChecked(False)
        with patch("app.simulation_panel.QProcess") as process:
            self.panel.command("start")
            self.assertNotIn("--fly", process.return_value.start.call_args.args[1])

    def test_height_only_disables_automatic_flight(self):
        self.panel.height_only.setChecked(True)
        self.assertFalse(self.panel.fly.isChecked())
        self.assertFalse(self.panel.fly.isEnabled())
        with patch("app.simulation_panel.QProcess") as process:
            self.panel.command("start")
            args = process.return_value.start.call_args.args[1]
            self.assertIn("--vertical-only", args)
            self.assertNotIn("--fly", args)
        self.panel.height_only.setChecked(False)
        self.assertTrue(self.panel.fly.isEnabled())

    def test_home_zoom_and_red_markers_render_before_logs(self):
        view = LocalizationMap()
        view.resize(700, 400)
        view.set_data(None, CONFIG)
        image = view.grab().toImage()
        reds = sum(
            image.pixelColor(x, y).red() > 180 and image.pixelColor(x, y).green() < 70
            for y in range(image.height())
            for x in range(image.width())
        )
        self.assertGreater(reds, 0)
        view.toggle_center()
        bounds = view.view_bounds()
        self.assertEqual(bounds, (-2.5, 2.5, -2.5, 2.5))
        self.assertLessEqual(bounds[0], min(h["x"] for h in HOMES) - 0.175)
        self.assertGreaterEqual(bounds[1], max(h["x"] for h in HOMES) + 0.175)
        view.close()

    def test_live_refresh_reads_phase_four_states_and_home_errors(self):
        run = self.root / "run"
        run.mkdir()
        samples(run, 1_000_000)
        runtime = self.root / "simulation/runtime"
        runtime.mkdir(parents=True)
        (runtime / "manager.json").write_text(
            json.dumps({"run_dir": str(run), "map": CONFIG}), encoding="utf-8"
        )
        (run / "team_status.json").write_text(
            json.dumps({"phase": "CONTACT", "active_vehicle": 2, "completed": [0, 1]}),
            encoding="utf-8",
        )
        self.panel.refresh()
        text = self.panel.cards.toPlainText()
        phase_text = self.panel.team_line.text()
        self.assertIn("轮流撞击", phase_text)
        self.assertIn("当前机 2", phase_text)
        self.assertIn("已撞击：0、1", phase_text)
        self.assertEqual(self.panel.table.rowCount(), 4)
        for i in range(4):
            self.assertIn(f"机 {i}", text)
        self.assertEqual(text.count("实时回家点偏移"), 4)
        self.assertIn("真值评估", text)
        self.assertNotIn("落地误差", text)
        self.panel.resize(1300, 900)
        self.panel.show()
        self.app.processEvents()
        self.assertFalse(self.panel.grab().isNull())

    def test_optional_audit_label_and_final_display(self):
        docs = self.root / "simulation/docs"
        docs.mkdir(parents=True)
        (docs / "scale-verification.json").write_text(
            json.dumps(
                {"field_size_m": [29, 16], "vehicle_extent_m": [0.082, 0.082, 0.025]}
            ),
            encoding="utf-8",
        )
        self.panel._update_map_context(CONFIG)
        self.assertIn("机体碰撞尺寸 0.082 × 0.082 × 0.025 m", self.panel.map_note.text())
        self.assertIn("红框 0.3 m", self.panel.map_note.text())
        self.assertIn("起点高度 0.55 m", self.panel.map_note.text())
        self.assertNotIn("0.35", self.panel.map_note.text())
        samples(self.root, 1_000_000)
        (self.root / "team_status.json").write_text(
            json.dumps({"phase": "COMPLETE"}), encoding="utf-8"
        )
        self.panel.metrics = LocalizationMetrics(self.root, homes=HOMES)
        text = self.panel._format(self.panel.metrics.poll())
        self.assertEqual(text.count("完成时回家点误差"), 4)
        self.assertNotIn("落地误差", text)

    def test_actual_phase_names_and_configured_cruise_height(self):
        for phase, text in [
            ("WAIT_READY", "等待多机就绪"),
            ("RETURN_ALL", "共同返航"),
            ("LAND_ALL", "多机降落"),
            ("PAUSED", "任务暂停"),
        ]:
            self.assertIn(text, self.panel._phase_text({"phase": phase}))
        config = dict(CONFIG, team_mission={"cruise_z_m": 1.2})
        self.panel._update_map_context(config)
        self.assertIn("任务巡航高度 1.2 m", self.panel.map_note.text())
        self.assertNotIn("1.8", self.panel.map_note.text())


if __name__ == "__main__":
    unittest.main()
