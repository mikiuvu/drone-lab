"""Analytic checks for IMU interval integration and velocity endpoint coverage."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

SPEC = importlib.util.spec_from_file_location(
    'impulse', Path(__file__).resolve().parents[1] / 'scripts/audit_contact_impulse.py')
impulse = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(impulse)


def imu(stamps=(10000, 20000), force=(0, 0, -9.80665), duration=10000):
    rows = dict(timestamp=np.array(stamps),
                accelerometer_integral_dt=np.full(len(stamps), duration),
                accelerometer_timestamp_relative=np.zeros(len(stamps), dtype=int),
                accelerometer_clipping=np.zeros(len(stamps), dtype=int))
    rows.update({f'accelerometer_m_s2[{axis}]': np.full(len(stamps), force[axis])
                 for axis in range(3)})
    return rows


def attitude(stamps=(0, 20000), quaternion=(1, 0, 0, 0)):
    rows = dict(timestamp=np.array(stamps))
    rows.update({f'q[{axis}]': np.full(len(stamps), quaternion[axis]) for axis in range(4)})
    return rows


def velocity(stamps=(0, 20000)):
    return dict(timestamp=np.array(stamps), vx=np.array([0., 2.]),
                vy=np.array([0., 0.]), vz=np.array([0., -1.]),
                v_xy_valid=np.array([True, True]), v_z_valid=np.array([True, True]))


class ContactImpulseTests(unittest.TestCase):
    def test_stationary_gravity_cancels(self):
        result = impulse.integrate(imu(), attitude(), 0, 20000)
        np.testing.assert_allclose(result['delta_velocity_ned_mps'], [0, 0, 0], atol=1e-12)
        self.assertTrue(result['complete'])
        self.assertEqual(result['covered_us'], 20000)

    def test_freefall_retains_gravity_and_constant_horizontal_acceleration(self):
        for force, expected in [((0, 0, 0), [0, 0, .196133]),
                                ((2, 0, -9.80665), [.04, 0, 0])]:
            result = impulse.integrate(imu(force=force), attitude(), 0, 20000)
            np.testing.assert_allclose(result['delta_velocity_ned_mps'], expected, atol=1e-12)

    def test_pitched_body_projects_specific_force_to_ned(self):
        half = np.sqrt(.5)
        result = impulse.integrate(imu(force=(9.80665, 0, 0)),
                                   attitude(quaternion=(half, 0, half, 0)), 0, 20000)
        np.testing.assert_allclose(result['delta_velocity_ned_mps'], [0, 0, 0], atol=1e-12)

    def test_clipped_intervals_use_only_window_duration_and_count_edge_clipping(self):
        rows = imu(force=(1, 0, -9.80665))
        rows['accelerometer_clipping'][1] = 1
        result = impulse.integrate(rows, attitude(), 5000, 15000)
        self.assertEqual(result['covered_us'], 10000)
        self.assertEqual(result['samples'], 2)
        self.assertEqual(result['accelerometer_clipping_samples'], 1)
        self.assertAlmostEqual(result['delta_velocity_ned_mps'][0], .01)

    def test_gaps_and_empty_coverage_are_explicit(self):
        result = impulse.integrate(imu(stamps=(10000, 30000)), attitude(stamps=(0, 30000)), 0, 30000)
        self.assertFalse(result['complete'])
        self.assertEqual(result['missing_intervals_us'], [[10000, 20000]])
        result = impulse.integrate(imu(), attitude(), 30000, 40000)
        self.assertFalse(result['complete'])
        self.assertEqual(result['coverage_fraction'], 0)
        self.assertEqual(result['missing_intervals_us'], [[30000, 40000]])

    def test_overlap_nonpositive_duration_and_relative_time_are_rejected(self):
        cases = [imu(stamps=(10000, 15000)), imu(duration=0), imu()]
        cases[2]['accelerometer_timestamp_relative'][0] = -1000
        for rows in cases:
            with self.assertRaises(ValueError):
                impulse.integrate(rows, attitude(), 0, 20000)

    def test_invalid_attitude_and_nonfinite_force_are_rejected(self):
        for q in ((0, 0, 0, 0), (np.nan, 0, 0, 0), (np.inf, 0, 0, 0)):
            with self.assertRaises(ValueError):
                impulse.integrate(imu(), attitude(quaternion=q), 0, 20000)
        with self.assertRaises(ValueError):
            impulse.integrate(imu(force=(np.nan, 0, 0)), attitude(), 0, 20000)

    def test_slerp_uses_short_arc_across_sign_and_yaw_wrap(self):
        q = np.array([np.sqrt(.5), 0, 0, np.sqrt(.5)])
        np.testing.assert_allclose(impulse.interpolate([0, 20000], [q, -q], 10000, True), q)
        angles = np.deg2rad([170, -170]) / 2
        values = np.stack([np.cos(angles), np.zeros(2), np.zeros(2), np.sin(angles)], axis=1)
        middle = impulse.interpolate([0, 20000], values, 10000, True)
        np.testing.assert_allclose(impulse.quaternion_rotation(middle) @ [1, 0, 0], [-1, 0, 0], atol=1e-12)

    def test_interpolation_rejects_missing_nonfinite_or_unordered_samples(self):
        for stamps, values, stamp in [
            ([0, 60000], [[0], [1]], 30000), ([0, 20000], [[0], [1]], -1),
            ([0, 20000], [[np.nan], [1]], 0), ([0, 20000], [[0], [np.inf]], 10000),
            ([0, 0], [[0], [1]], 0), ([2, 1], [[0], [1]], 1),
        ]:
            with self.assertRaises(ValueError):
                impulse.interpolate(stamps, values, stamp)
        with self.assertRaises(ValueError):
            impulse.interpolate([0, 20000], [[0]*4, [1, 0, 0, 0]], 0, True)

    def test_attitude_sample_clock_takes_precedence_over_publication(self):
        rows = attitude(stamps=(100000, 120000))
        rows['timestamp_sample'] = np.array([0, 20000])
        result = impulse.integrate(imu(), rows, 0, 20000)
        self.assertTrue(result['complete'])
        self.assertEqual(result['attitude_time_field'], 'timestamp_sample')

    def test_native_velocity_requires_both_valid_endpoints_and_sample_time(self):
        rows = velocity(stamps=(100000, 120000))
        rows['timestamp_sample'] = np.array([0, 20000])
        result = impulse.velocity_change(rows, 5000, 15000, require_valid=True)
        np.testing.assert_allclose(result['delta_velocity_ned_mps'], [1, 0, -.5])
        rows['v_z_valid'][1] = False
        with self.assertRaises(ValueError):
            impulse.velocity_change(rows, 5000, 15000, require_valid=True)

    def test_partial_integral_is_not_compared_with_full_window_velocity(self):
        class Topic:
            multi_id = 0

            def __init__(self, name, data):
                self.name, self.data = name, data

        class Log:
            data_list = [Topic('sensor_combined', imu(stamps=(10000,))),
                         Topic('vehicle_attitude', attitude()),
                         Topic('vehicle_attitude_groundtruth', attitude()),
                         Topic('vehicle_local_position', velocity()),
                         Topic('vehicle_local_position_groundtruth', velocity())]

        with patch.object(impulse, 'ULog', return_value=Log()):
            result = impulse.analyze(Path(__file__), 0, 20000)
        self.assertFalse(result['complete'])
        self.assertNotIn('native_difference_norm_mps', result['estimated_attitude'])
        self.assertNotIn('gazebo_pose_velocity_difference_norm_mps', result['truth_attitude'])


if __name__ == '__main__':
    unittest.main()
