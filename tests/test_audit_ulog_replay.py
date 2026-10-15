"""EKF2 输入索引及日志丢失诊断。"""

from pathlib import Path
from types import SimpleNamespace
import importlib.util
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/audit_ulog_replay.py'
SPEC = importlib.util.spec_from_file_location('audit_ulog_replay', SCRIPT)
audit_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit_module)


def dataset(name, instance=0, **fields):
    return SimpleNamespace(name=name, multi_id=instance, data=fields)


class ReplayInputAuditTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'flight.ulg'
        self.path.write_bytes(b'ulog-fixture')
        self.clock = {'timestamp': np.array([1000000, 1004000], dtype=np.uint64)}
        for field in audit_module.RELATIVE_TOPICS:
            self.clock[field] = np.array([32767, 32767], dtype=np.int16)
        self.records = [dataset('ekf2_timestamps', **self.clock),
                        dataset('sensor_combined', timestamp=self.clock['timestamp'])]
        self.log = SimpleNamespace(data_list=self.records, initial_parameters={'SDLOG_PROFILE': 3}, dropouts=[])

    def audit(self, begin=1.0):
        with patch.object(audit_module, 'ULog', return_value=self.log):
            return audit_module.audit(self.path, begin)

    def test_signed_relative_timestamp_matches_unsigned_input_clock(self):
        self.clock['visual_odometry_timestamp_rel'][0] = -2
        self.records.append(dataset('vehicle_visual_odometry', timestamp=np.array([999899], dtype=np.uint64)))
        report = self.audit()
        self.assertTrue(report['timestamp_inputs_complete'])
        self.assertEqual(report['topics']['vehicle_visual_odometry']['missing_total'], 0)

    def test_missing_input_has_actual_neighbours_and_no_write_dropout(self):
        self.clock['visual_odometry_timestamp_rel'][0] = 10
        self.records.append(dataset('vehicle_visual_odometry', timestamp=np.array([1000800, 1001200], dtype=np.uint64)))
        report = self.audit()
        self.assertFalse(report['timestamp_inputs_complete'])
        missing = report['topics']['vehicle_visual_odometry']['missing_references'][0]
        self.assertEqual(missing, {'filter_time_us': 1000000, 'requested_time_100us': 10010,
                                  'nearest_logged_before_us': 1000800, 'nearest_logged_after_us': 1001200})
        self.assertEqual(report['logger_diagnostics']['write_dropout_events'], 0)
        self.assertIsNone(report['logger_diagnostics']['full_log_message_gaps_max'])

    def test_missing_topic_has_no_fabricated_neighbours(self):
        self.clock['visual_odometry_timestamp_rel'][0] = 10
        missing = self.audit()['topics']['vehicle_visual_odometry']['missing_references'][0]
        self.assertIsNone(missing['nearest_logged_before_us'])
        self.assertIsNone(missing['nearest_logged_after_us'])

    def test_window_excludes_earlier_missing_reference_without_erasing_it(self):
        self.clock['visual_odometry_timestamp_rel'][0] = 10
        report = self.audit(1.001)
        self.assertTrue(report['timestamp_inputs_complete'])
        self.assertEqual(report['topics']['vehicle_visual_odometry']['missing_total'], 1)
        self.assertEqual(report['topics']['vehicle_visual_odometry']['missing_in_window'], 0)

    def test_write_overflow_and_full_log_subscription_gaps_remain_independent(self):
        self.log.dropouts = [SimpleNamespace(duration=5), SimpleNamespace(duration=7)]
        self.records.extend([
            dataset('logger_status', message_gaps=np.array([2, 4], dtype=np.uint32)),
            dataset('logger_status', instance=1, message_gaps=np.array([99], dtype=np.uint32)),
        ])
        report = self.audit()
        self.assertTrue(report['timestamp_inputs_complete'])
        self.assertEqual(report['logger_diagnostics'], {'write_dropout_events': 2,
                         'write_dropout_duration_ms': 12, 'full_log_message_gaps_max': 4})

    def test_complete_input_does_not_imply_zero_subscription_gaps(self):
        self.records.append(dataset('logger_status', message_gaps=np.array([7], dtype=np.uint32)))
        report = self.audit()
        self.assertTrue(report['timestamp_inputs_complete'])
        self.assertEqual(report['logger_diagnostics']['full_log_message_gaps_max'], 7)

    def test_neighbours_are_chronological_even_if_log_records_are_out_of_order(self):
        self.clock['visual_odometry_timestamp_rel'][0] = 10
        self.records.append(dataset('vehicle_visual_odometry', timestamp=np.array([1001200, 1000800], dtype=np.uint64)))
        reference = self.audit()['topics']['vehicle_visual_odometry']['missing_references'][0]
        self.assertEqual(reference['nearest_logged_before_us'], 1000800)
        self.assertEqual(reference['nearest_logged_after_us'], 1001200)

    def test_requested_input_with_multiple_instances_is_rejected(self):
        self.clock['visual_odometry_timestamp_rel'][0] = 10
        self.records.extend([dataset('vehicle_visual_odometry', timestamp=np.array([1001000], dtype=np.uint64)),
                             dataset('vehicle_visual_odometry', instance=1, timestamp=np.array([1001000], dtype=np.uint64))])
        with self.assertRaisesRegex(ValueError, '非单实例'):
            self.audit()


if __name__ == '__main__':
    unittest.main()
