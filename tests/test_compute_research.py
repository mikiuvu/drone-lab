"""Hand-calculated schedules, conservation, and overload bounds (no hardware)."""

from copy import deepcopy
import json
import unittest

from research.compute import DEFAULT_CONFIG, OVERLOAD_CONFIG, run_compute


def simple(**updates):
    cfg = dict(camera_fps=10, duration_s=0.2, nav_hz=5, nav_ms=0,
               cpu_capture_ms=0, cpu_copy_ms=0, cpu_pre_ms=0,
               cpu_post_ms=0, cpu_tracking_ms=0, npu_ms=0, trace_limit=200)
    cfg.update(updates)
    return cfg


def done_ids(policy):
    return [row["id"] for row in policy["trace"] if row["event"] == "frame_done"]


class ComputeResearchTests(unittest.TestCase):
    def test_two_independent_resources_hand_schedule(self):
        # At 0: nav 0..2, pre 2..12, NPU 12..32, frame done 32.
        # Next frame at 100: pre 100..110, NPU 110..130; age 30.
        metrics = run_compute(simple(nav_ms=2, cpu_pre_ms=10, npu_ms=20))["metrics"]
        for policy in (metrics["fifo"], metrics["latest"]):
            self.assertEqual(done_ids(policy), [0, 1])
            self.assertEqual(policy["frames"]["age_ms"],
                             dict(count=2, p50=30, p95=32, max=32))
            self.assertEqual(policy["resources"]["cpu_busy_ms"], 22)
            self.assertEqual(policy["resources"]["npu_busy_ms"], 40)

    def test_fifo_vs_latest_exact_overload_and_boundary(self):
        # Camera 0,100,200,300,400; NPU service 250 ms.
        # FIFO finishes IDs 0 and 1 at 250 and 500; ages 250,400.
        # Latest finishes IDs 0 and 2 at 250 and 500; ages 250,300.
        # Latest replaces 1 at 200 and 3 at 400; ID 4 remains pending.
        metrics = run_compute(simple(duration_s=0.5, npu_ms=250))["metrics"]
        fifo, latest = metrics["fifo"], metrics["latest"]
        self.assertEqual(done_ids(fifo), [0, 1])
        self.assertEqual(done_ids(latest), [0, 2])
        self.assertEqual(fifo["frames"]["age_ms"]["max"], 400)
        self.assertEqual(latest["frames"]["age_ms"]["max"], 300)
        self.assertEqual(latest["frames"]["dropped"], 2)
        self.assertEqual(fifo["frames"]["pending"], 3)
        self.assertEqual(latest["frames"]["pending"], 1)
        self.assertEqual(fifo["frames"]["in_flight"], 0)
        self.assertEqual(latest["memory"]["max_held_frames"], 2)
        self.assertEqual(fifo["frames"]["generated"], 5)
        self.assertEqual(fifo["frames"]["queue_wait_ms"]["max"], 150)
        self.assertEqual(latest["frames"]["queue_wait_ms"]["max"], 50)
        self.assertEqual(latest["frames"]["stale_completed"], 2)

    def test_nonpreemptive_cpu_blocks_nav_but_npu_does_not(self):
        cfg = simple(camera_fps=1, duration_s=0.04, nav_hz=100, nav_ms=2,
                     cpu_pre_ms=15, npu_ms=20, nav_deadline_ms=5)
        policy = run_compute(cfg)["metrics"]["latest"]
        # nav 0..2; pre 2..17 blocks release at 10; nav 17..19 age9.
        # NPU 17..37 overlaps nav 17..19,20..22,30..32 (six ms).
        self.assertEqual(policy["navigation"]["deadline_misses_completed"], 1)
        self.assertEqual(policy["navigation"]["age_ms"]["max"], 9)
        self.assertEqual(policy["resources"]["cpu_npu_overlap_ms"], 6)
        self.assertEqual(policy["frames"]["age_ms"]["max"], 37)

    def test_unfinished_frames_not_hidden_and_no_sample_is_null(self):
        policy = run_compute(simple(duration_s=0.3, npu_ms=1000,
                                    frame_deadline_ms=50))["metrics"]["fifo"]
        self.assertEqual(policy["frames"]["age_ms"]["p95"], None)
        self.assertEqual(policy["frames"]["outstanding"], 3)
        self.assertEqual(policy["frames"]["overdue_at_end"], 3)
        self.assertEqual(policy["frames"]["oldest_outstanding_age_ms"], 300)
        self.assertEqual(policy["resources"]["npu_busy_ms"], 300)

    def test_burst_latches_at_start_and_deadline_equality_is_not_miss(self):
        # First NPU doubles 100 to 200 despite burst ending at 50.
        # Second starts at 200 outside burst, and runs its ordinary 100.
        cfg = simple(duration_s=0.3, npu_ms=100, frame_deadline_ms=200,
                     bursts=[dict(start_s=0, end_s=0.05, npu_factor=2)])
        fifo = run_compute(cfg)["metrics"]["fifo"]
        self.assertEqual(fifo["frames"]["completed"], 2)
        self.assertEqual(fifo["frames"]["age_ms"]["max"], 200)
        self.assertEqual(fifo["frames"]["deadline_misses_completed"], 0)

    def test_conservation_bounds_under_resource_and_burst_overload(self):
        cases = [dict(), OVERLOAD_CONFIG, dict(cpu_pre_ms=80),
                 dict(nav_ms=100), dict(cpu_copy_ms=25),
                 dict(npu_ms_pattern=[3, 180, 18, 500]),
                 dict(bursts=[dict(start_s=0.1, end_s=0.5, cpu_factor=4,
                                  npu_factor=8)]),
                 dict(cpu_pre_ms=0, cpu_post_ms=0, npu_ms=0, nav_ms=0,
                      cpu_capture_ms=0, cpu_tracking_ms=0),
                 dict(npu_ms=10000, camera_fps=120)]
        for case in cases:
            for duration in (0.7, 2):
                with self.subTest(case=case, duration=duration):
                    result = run_compute(dict(case, duration_s=duration))
                    json.dumps(result, ensure_ascii=False, allow_nan=False)
                    for name in ("fifo", "latest"):
                        policy = result["metrics"][name]
                        for counts in (policy["frames"], policy["navigation"]):
                            self.assertEqual(counts["generated"], counts["completed"]
                                             + counts["dropped"] + counts["pending"]
                                             + counts["in_flight"])
                        resources = policy["resources"]
                        self.assertLessEqual(resources["cpu_utilization"], 1 + 1e-9)
                        self.assertLessEqual(resources["npu_utilization"], 1 + 1e-9)
                        self.assertLessEqual(resources["cpu_npu_overlap_ms"],
                                             min(resources["cpu_busy_ms"],
                                                 resources["npu_busy_ms"]) + 1e-6)
                    memory = result["metrics"]["latest"]["memory"]
                    self.assertLessEqual(memory["max_queue_frames"], 1)
                    self.assertLessEqual(memory["max_held_frames"], 2)
                    self.assertLessEqual(memory["max_held_frame_bytes"],
                                         2 * result["config"]["frame_bytes"])

    def test_fifo_grows_with_horizon_latest_does_not(self):
        short = run_compute(dict(OVERLOAD_CONFIG, duration_s=1))["metrics"]
        long = run_compute(dict(OVERLOAD_CONFIG, duration_s=3))["metrics"]
        self.assertGreater(long["fifo"]["memory"]["max_queue_frames"],
                           short["fifo"]["memory"]["max_queue_frames"])
        self.assertEqual(long["latest"]["memory"]["max_held_frames"], 2)
        self.assertGreater(long["latest"]["frames"]["dropped"], 0)

    def test_config_not_mutated_deterministic_and_invalid_values(self):
        initial = deepcopy(DEFAULT_CONFIG)
        cfg = dict(bursts=[dict(start_s=0.1, end_s=0.2, cpu_factor=2)])
        saved = deepcopy(cfg)
        self.assertEqual(run_compute(cfg), run_compute(cfg))
        self.assertEqual(cfg, saved)
        self.assertEqual(DEFAULT_CONFIG, initial)
        for bad in (dict(camera_fps=0), dict(nav_ms=-1), dict(npu_ms=float('nan')),
                    dict(nav_hz=float('inf')), dict(camera_fps=True),
                    dict(frame_bytes=1.5), dict(unknown=1),
                    dict(bursts=[dict(start_s=2, end_s=1)]),
                    dict(bursts=[dict(start_s=0, end_s=1, npu_factor=0)]),
                    dict(duration_s=10000), dict(npu_ms_pattern=[False])):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                run_compute(bad)

    def test_gui_float_parameters_and_json_contract(self):
        result = run_compute(dict(duration_s=10.0, nav_hz=15.0, camera_fps=30.0,
                                  cpu_pre_ms=4.0, npu_ms=18.0, nav_ms=5.0))
        self.assertIsInstance(result["summary"], str)
        self.assertTrue(all(isinstance(a, str) for a in result["assumptions"]))
        json.dumps(result, ensure_ascii=False, allow_nan=False)
        self.assertEqual(result["metrics"]["latest"]["frames"]["completed"], 300)


if __name__ == "__main__":
    unittest.main()
