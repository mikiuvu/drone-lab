"""Landing-gate accounting uses preceding samples and clips intervals to the window."""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from summarize_landing_delay import blocked_seconds, latest


class LandingDelaySummaryTest(unittest.TestCase):
    def setUp(self):
        self.data = {"timestamp": np.array([1000000, 2000000, 4000000]),
                     "horizontal_movement": np.array([0, 1, 0])}

    def test_irregular_intervals_are_clipped_and_weighted_by_time(self):
        self.assertAlmostEqual(blocked_seconds(self.data, "horizontal_movement", 1.5, 4.5, 1), 2.0)
        self.assertAlmostEqual(blocked_seconds(self.data, "horizontal_movement", 2.5, 3.0, 1), 0.5)
        self.assertAlmostEqual(blocked_seconds(self.data, "horizontal_movement", 4.0, 5.0, 1), 0.0)

    def test_future_samples_do_not_change_earlier_decisions(self):
        self.assertEqual(latest(self.data, 1.999), 0)
        self.assertEqual(latest(self.data, 2.0), 1)
        self.assertEqual(latest(self.data, 3.9), 1)
        self.assertEqual(latest(self.data, 4.0), 2)

    def test_missing_coverage_and_invalid_windows_are_rejected(self):
        with self.assertRaises(ValueError):
            latest(self.data, 0.9)
        with self.assertRaises(ValueError):
            blocked_seconds(self.data, "horizontal_movement", 0.9, 1.5, 1)
        with self.assertRaises(ValueError):
            blocked_seconds(self.data, "horizontal_movement", 2.0, 1.5, 1)


if __name__ == "__main__":
    unittest.main()
