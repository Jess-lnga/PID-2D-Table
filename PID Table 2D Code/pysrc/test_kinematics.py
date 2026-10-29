"""python -m unittest discover -s pysrc -p 'test_*.py' -v"""
import math
import os
import subprocess
import unittest
import numpy as np
from kinematics_model import compute_kinematics, load_parameters


class KinematicsTests(unittest.TestCase):
    def test_constraints_and_both_valid_and_invalid_poses(self):
        p = load_parameters()
        valid, invalid = 0, 0
        for z in (0, 10, 30, 60):
            for roll in range(-30, 31, 10):
                for pitch in range(-30, 31, 10):
                    result = compute_kinematics(p, z, roll, pitch)
                    if not result.valid:
                        invalid += 1
                        self.assertIsNone(result.angles)
                        continue
                    valid += 1
                    np.testing.assert_allclose(np.linalg.norm(result.elbows-result.bases, axis=1), p.L1, atol=1e-8)
                    np.testing.assert_allclose(np.linalg.norm(result.joints-result.elbows, axis=1), p.L2, atol=1e-8)
                    alpha = np.arange(3)*2*math.pi/3
                    residual = -np.sin(alpha)*result.joints[:, 0]+np.cos(alpha)*result.joints[:, 1]
                    np.testing.assert_allclose(residual, 0, atol=1e-8)
                    self.assertTrue(np.all(result.angles >= p.q_min))
                    self.assertTrue(np.all(result.angles <= p.q_max))
        self.assertGreater(valid, 0)
        self.assertGreater(invalid, 0)

    def test_level_and_errors(self):
        p = load_parameters()
        result = compute_kinematics(p, 0, 0, 0)
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.angles, result.angles[0], atol=1e-9)
        self.assertEqual(result.yaw_deg, 0)
        self.assertEqual(compute_kinematics(p, -1, 0, 0).error, 'InvalidInput')
        self.assertEqual(compute_kinematics(p, 0, float('nan'), 0).error, 'InvalidInput')
        self.assertEqual(compute_kinematics(p, 0, 0, 180).error, 'SingularOrientation')
        self.assertEqual(compute_kinematics(p, 1000, 0, 0).error, 'Unreachable')

    @unittest.skipUnless(os.environ.get('KINEMATICS_REFERENCE'), 'Optional native C++ reference executable')
    def test_cpp_parity(self):
        p = load_parameters()
        poses = [(z, r, t) for z in (0, 10, 30, 60) for r in range(-40, 41, 5) for t in range(-40, 41, 5)]
        poses += [(0, 0, 180), (-1, 0, 0), (1000, 0, 0), (0, 370, -355)]
        output = subprocess.run([os.environ['KINEMATICS_REFERENCE']],
                                input=''.join(f'{z} {r} {t}\n' for z, r, t in poses),
                                text=True, capture_output=True, check=True).stdout.splitlines()
        self.assertEqual(len(output), len(poses))
        errors = ['None', 'InvalidParameters', 'InvalidInput', 'SingularOrientation',
                  'SingularLeg', 'Unreachable', 'JointLimit']
        for pose, line in zip(poses, output):
            fields = line.split()
            result = compute_kinematics(p, *pose)
            self.assertEqual(result.error, errors[int(fields[0])], str(pose))
            self.assertEqual(result.failed_leg, int(fields[1]), str(pose))
            if result.valid:
                expected = np.array([float(v) for v in fields[2:]])
                actual = np.r_[result.angles, result.center[:2], result.yaw_deg]
                np.testing.assert_allclose(actual, expected, atol=1e-8, rtol=1e-10, err_msg=str(pose))

    def test_viewer_controls(self):
        import matplotlib
        matplotlib.use('Agg')
        from kinematics_viewer import Viewer
        import matplotlib.pyplot as plt
        from kinematics_model import DEFAULT_HEADER
        viewer = Viewer(DEFAULT_HEADER)
        viewer.sliders[1].set_val(5)
        viewer.sliders[2].set_val(-5)
        self.assertIn('q1', viewer.info.get_text())
        viewer.sliders[0].set_val(viewer.sliders[0].valmax)
        viewer.sliders[1].set_val(40)
        self.assertIn('PAS DE SOLUTION', viewer.info.get_text())
        viewer.toggle_planes(None)
        viewer.fig.canvas.draw()
        viewer.reset_pose(None)
        self.assertIn('q1', viewer.info.get_text())
        viewer.reload_parameters(None)
        viewer.fig.canvas.draw()
        # Pose updates must not accumulate artists or alter the camera.
        objects = tuple(viewer.ax.get_children())
        viewer.ax.view_init(elev=17, azim=33)
        viewer.fig.canvas.draw()
        for i in range(10):
            viewer.sliders[1].set_val(i-5)
        self.assertEqual(tuple(viewer.ax.get_children()), objects)
        self.assertEqual((viewer.ax.elev, viewer.ax.azim), (17, 33))
        viewer.sliders[1].set_val(7)
        from types import SimpleNamespace
        viewer._flush_update(SimpleNamespace(name='button_release_event'))
        self.assertIn('7.000', viewer.info.get_text())
        # Multiple cursor events are coalesced into one update of the latest pose.
        viewer._interactive = True
        viewer.sliders[1].set_val(2)
        viewer.sliders[1].set_val(4)
        self.assertTrue(viewer._dirty)
        viewer._flush_update(SimpleNamespace(name='button_release_event'))
        self.assertFalse(viewer._dirty)
        self.assertIn('4.000', viewer.info.get_text())
        viewer._interactive = False
        viewer.fig.set_size_inches(12, 8)
        viewer.fig.canvas.draw()
        viewer.update(force_info=True)
        plt.close(viewer.fig)

    def test_viewer_fallback(self):
        import matplotlib
        matplotlib.use('Agg')
        from kinematics_viewer import Viewer
        from kinematics_model import DEFAULT_HEADER
        import matplotlib.pyplot as plt
        viewer = Viewer(DEFAULT_HEADER, use_blit=False)
        viewer.sliders[1].set_val(5)
        viewer.update(force_info=True)
        self.assertIn('5.000', viewer.info.get_text())
        self.assertFalse(any(a.get_animated() for a in viewer._dynamic_artists))
        plt.close(viewer.fig)


if __name__ == '__main__':
    unittest.main()
