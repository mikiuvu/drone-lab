"""Check exposure-time ego error decomposition and missing-data handling."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

SPEC = importlib.util.spec_from_file_location(
    'projection', Path(__file__).resolve().parents[1] / 'scripts/audit_vision_projection.py')
projection = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(projection)


class ProjectionAuditTests(unittest.TestCase):
    def test_common_translation_bias_does_not_become_image_error(self):
        ego = projection.ScalarSeries([0, 20000], [0, .02])
        target = projection.ScalarSeries([0, 20000], [.4, .4])
        request = dict(sim_time_us=10000, pose_time_us=10000, pose_valid=True,
                       pose_y_m=.11, pose_variance_y_m2=.0025)
        observed = dict(sim_time_us=10000, y_m=.5, measurement='blue_pnp', accepted=True)
        row = projection.sample_error(request, observed, ego, target)
        self.assertAlmostEqual(row['ego_translation_error_y_m'], .1)
        self.assertAlmostEqual(row['measurement_error_y_m'], .1)
        self.assertAlmostEqual(row['residual_after_ego_translation_y_m'], 0)

    def test_future_selected_pose_separates_time_shift_from_state_error(self):
        ego = projection.ScalarSeries([0, 20000], [0, .02])
        target = projection.ScalarSeries([0, 20000], [.4, .4])
        request = dict(sim_time_us=10000, pose_time_us=20000, pose_valid=True,
                       pose_y_m=.12, pose_variance_y_m2=.0025)
        observed = dict(sim_time_us=10000, y_m=.51, measurement='green_rail_plane', accepted=True)
        row = projection.sample_error(request, observed, ego, target)
        self.assertAlmostEqual(row['ego_translation_error_y_m'], .11)
        self.assertAlmostEqual(row['ego_state_error_y_m'], .1)
        self.assertAlmostEqual(row['ego_time_shift_y_m'], .01)
        self.assertEqual(row['pose_time_offset_us'], 10000)
        self.assertEqual(row['measurement'], 'green_rail_plane')

    def test_truth_never_extrapolates_or_bridges_missing_intervals(self):
        series = projection.ScalarSeries([0, 50000], [0, 1])
        self.assertEqual(series.at(25000), .5)
        for stamp in (-1, 50001):
            with self.assertRaises(ValueError):
                series.at(stamp)
        with self.assertRaises(ValueError):
            projection.ScalarSeries([0, 50001], [0, 1]).at(25000)
        for stamps, values in [([1, 1], [0, 1]), ([1, 0], [0, 1]), ([0, 1], [0, np.nan])]:
            with self.assertRaises(ValueError):
                projection.ScalarSeries(stamps, values)

    def test_ambiguous_requests_are_excluded_instead_of_taking_the_latest(self):
        rows = [dict(kind='vision_projection_input', sim_time_us=t) for t in (1, 2, 2)]
        unique, duplicates = projection.unique_projections(rows)
        self.assertEqual(set(unique), {1})
        self.assertEqual(duplicates, {2})

    def test_invalid_or_unmatched_pose_is_rejected(self):
        ego = target = projection.ScalarSeries([0, 20000], [0, 0])
        request = dict(sim_time_us=10000, pose_time_us=10000, pose_valid=True,
                       pose_y_m=0., pose_variance_y_m2=.01)
        observed = dict(sim_time_us=10000, y_m=0., measurement='blue_pnp', accepted=True)
        for key, value in [('pose_valid', False), ('pose_variance_y_m2', -1),
                           ('pose_y_m', np.nan), ('sim_time_us', 10001), ('pose_time_us', 30000)]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                projection.sample_error(dict(request, **{key: value}), observed, ego, target)

    def test_constant_series_has_no_manufactured_correlation(self):
        row = {key:0. for key in ('ego_translation_error_y_m', 'ego_state_error_y_m',
                                 'ego_time_shift_y_m', 'measurement_error_y_m',
                                 'residual_after_ego_translation_y_m')}
        row['pose_time_offset_us'] = 0
        result = projection.summarize([row]*3)
        self.assertIsNone(result['ego_measurement_error_correlation'])
        self.assertEqual(result['measurement_error_y_m']['rmse'], 0)
        self.assertEqual(projection.summarize([]), dict(samples=0))


if __name__ == '__main__':
    unittest.main()
