"""外部里程计与原生垂向速度的坐标、时间匹配检查。"""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

SPEC = importlib.util.spec_from_file_location('recovery', Path(__file__).resolve().parents[1] /
                                             'scripts/summarize_recovery_ulog.py')
recovery = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recovery)


def external(stamps, vectors, frame=1, quaternion=(1,0,0,0), pose_frame=1):
    rows = {'timestamp_sample': np.array(stamps, dtype=np.uint64),
            'velocity_frame': np.full(len(stamps),frame), 'pose_frame': np.full(len(stamps),pose_frame)}
    rows.update({f'velocity[{axis}]': np.array([v[axis] for v in vectors]) for axis in range(3)})
    rows.update({f'q[{axis}]': np.full(len(stamps),quaternion[axis]) for axis in range(4)})
    return rows


class PostContactVelocityTests(unittest.TestCase):
    def test_local_down_frames_have_same_vertical_sign(self):
        for frame in (1,2):
            self.assertEqual(recovery.vertical_velocity_up(frame,[4,-3,-2]),2)

    def test_body_velocity_rotates_pitch_and_roll(self):
        h = np.sqrt(.5)
        self.assertAlmostEqual(recovery.vertical_velocity_up(3,[2,0,0],[h,0,h,0],1),2)
        self.assertAlmostEqual(recovery.vertical_velocity_up(3,[0,2,0],[h,h,0,0],2),-2)

    def test_yaw_and_quaternion_scale_do_not_change_vertical_projection(self):
        h = np.sqrt(.5)
        self.assertAlmostEqual(recovery.vertical_velocity_up(3,[4,5,-3],[2*h,0,0,2*h],1),3)

    def test_invalid_or_unknown_frames_are_not_used(self):
        for frame,velocity,q,pose in [(0,[1,2,3],[1,0,0,0],1), (3,[1,2,3],[1,0,0,0],0),
                                     (3,[1,2,3],[0,0,0,0],1), (3,[1,2,3],[np.nan,0,0,0],1),
                                     (1,[0,np.inf,0],None,1)]:
            self.assertIsNone(recovery.vertical_velocity_up(frame,velocity,q,pose))

    def test_pairing_uses_sample_time_and_never_future_native_sample(self):
        ev = external([1000000,1020000],[[0,0,-2],[0,0,-3]])
        ev['timestamp'] = np.array([1300000,1320000],dtype=np.uint64)
        native = {'timestamp':np.array([999999,1010000,1030000]),'vz':np.array([-2.,-1.,100.])}
        result = recovery.velocity_consistency(ev,native,1000000,1020000)
        self.assertEqual(result['paired_samples'],2)
        self.assertEqual(result['absolute_difference_max_mps'],2)
        self.assertEqual(result['samples_over_1_mps'],1)

    def test_native_age_boundary_and_window_are_explicit(self):
        ev = external([999999,1050000,1050001,1100000],[[0,0,-2]]*4)
        native = {'timestamp':np.array([1000000]),'vz':np.array([-2.])}
        result = recovery.velocity_consistency(ev,native,1000000,1050001)
        self.assertEqual(result['paired_samples'],1)
        self.assertEqual(result['missing_native_samples'],1)
        self.assertEqual(result['absolute_difference_max_mps'],0)

    def test_invalid_native_or_external_values_are_counted(self):
        ev = external([1000000,1020000],[[0,np.nan,0],[0,0,-2]])
        native = {'timestamp':np.array([999999]),'vz':np.array([np.nan])}
        result = recovery.velocity_consistency(ev,native,1000000,1040000)
        self.assertEqual(result['invalid_or_unsupported_samples'],1)
        self.assertEqual(result['invalid_native_samples'],1)
        self.assertEqual(result['paired_samples'],0)
        self.assertNotIn('absolute_difference_max_mps',result)

    def test_native_validity_flag_excludes_finite_invalid_velocity(self):
        ev = external([1000000],[[0,0,-2]])
        native = {'timestamp':np.array([1000000]), 'vz':np.array([-2.]), 'v_z_valid':np.array([False])}
        result = recovery.velocity_consistency(ev,native,1000000,1040000)
        self.assertEqual(result['paired_samples'],0)
        self.assertEqual(result['invalid_native_samples'],1)

    def test_missing_topics_are_unavailable_not_zero_error(self):
        result = recovery.velocity_consistency({}, {}, 0, 1000000)
        self.assertFalse(result['available'])
        self.assertEqual(result['paired_samples'],0)


if __name__ == '__main__':
    unittest.main()
