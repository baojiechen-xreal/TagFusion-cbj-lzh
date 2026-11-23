import math
import unittest

from scripts import eval_large_scene_stationary as large_scene
from scripts import eval_stationary_drift as core


class StationaryDriftFormulaTest(unittest.TestCase):
    def test_revisit_drift_uses_identity_gt(self):
        identity = ((1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0))
        rz10 = core.so3_exp((0.0, 0.0, math.radians(10.0)))
        gt_windows = [
            core.Window(0, 0.0, 3.0, 90, identity, (0.0, 0.0, 0.0), 0.0),
            core.Window(1, 10.0, 13.0, 90, identity, (9.9, 0.0, 0.0), 0.0),
        ]
        est_windows = [
            core.Window(0, 0.0, 3.0, 90, identity, (1.0, 2.0, 3.0), 0.0),
            core.Window(1, 10.0, 13.0, 90, rz10, (1.5, 2.0, 3.0), 0.0),
        ]

        errors = large_scene.errors_against_gt(est_windows, gt_windows)

        self.assertEqual(len(errors), 1)
        self.assertAlmostEqual(errors[0].trans_vs_gt_m, 0.5, places=12)
        self.assertAlmostEqual(errors[0].rot_vs_gt_deg, 10.0, places=9)

    def test_adaptive_core_keeps_long_window_and_shrinks_short_window(self):
        # Long 8s window keeps full 3.0s core
        t0, t1 = large_scene.adaptive_core_bounds(
            10.0, 18.0, core_duration=3.0, min_trim=0.75, min_core=0.5)
        self.assertAlmostEqual(t1 - t0, 3.0, places=9)
        self.assertAlmostEqual(t0, 12.5, places=9)

        # Short ~3.2s window trims at least 0.75s from both ends -> 1.7s core
        s0, s1 = large_scene.adaptive_core_bounds(
            10.0, 13.2, core_duration=3.0, min_trim=0.75, min_core=0.5)
        self.assertAlmostEqual(s0, 10.75, places=9)
        self.assertAlmostEqual(s1, 12.45, places=9)
        self.assertAlmostEqual(s1 - s0, 1.7, places=9)

        # Short 2.0s initial window keeps central 50% (1.0s core, trims 0.5s each side)
        i0, i1 = large_scene.adaptive_core_bounds(
            0.0, 2.0, core_duration=3.0, min_trim=0.75, min_core=0.5)
        self.assertAlmostEqual(i0, 0.5, places=9)
        self.assertAlmostEqual(i1, 1.5, places=9)

        # Very short 1.4s initial window still works (central 0.7s core, trims 0.35s each side)
        v0, v1 = large_scene.adaptive_core_bounds(
            0.0, 1.4, core_duration=3.0, min_trim=0.75, min_core=0.5)
        self.assertAlmostEqual(v0, 0.35, places=9)
        self.assertAlmostEqual(v1, 1.05, places=9)

    def test_static_jitter_skips_window0_and_computes_five_metrics(self):
        # Build a 4.0s stationary segment [10.0, 14.0]; unified core bounds -> [10.75, 13.25] (2.5s >= 1.0s)
        poses = []
        for k in range(41):
            t = 10.0 + 0.1 * k
            # Linear drift 1 mm/s in x, plus a +2 mm jump at t=12.0s
            x = 0.001 * (t - 10.75)
            if abs(t - 12.0) < 1e-6:
                x += 0.002
            ang_deg = 0.1 * (t - 10.75)
            rot = core.so3_exp((0.0, 0.0, math.radians(ang_deg)))
            poses.append(core.Pose(t, rot, (x, 0.0, 0.0)))

        # Window 0 (seg_index == 0) must be skipped
        self.assertIsNone(
            large_scene.compute_static_jitter(
                poses, 10.75, 13.25, seg_index=0, min_duration=1.0
            )
        )

        # Core window < 1.0s (e.g. 0.7s core) -> rejected for static jitter
        too_short = large_scene.compute_static_jitter(
            poses, 10.75, 11.45, seg_index=1, min_duration=1.0)
        self.assertIsNone(too_short)

        ct0, ct1 = large_scene.adaptive_core_bounds(10.0, 14.0, core_duration=3.0, min_trim=0.75)
        jitter = large_scene.compute_static_jitter(
            poses, ct0, ct1, seg_index=1, min_duration=1.0)
        self.assertIsNotNone(jitter)
        self.assertEqual(jitter.index, 1)
        self.assertGreaterEqual(jitter.duration_s, 2.39)
        # P2P from first sample inside [10.75, 13.25] at t=10.8s (x=0.05mm) to t=12.0s (x=3.25mm) -> 3.20 mm
        self.assertAlmostEqual(jitter.p2p_trans_mm, 3.20, places=4)
        # Max single-step jump around t=12.0s: from 1.15mm (11.9s) to 3.25mm (12.0s) = 2.10 mm
        self.assertAlmostEqual(jitter.max_jump_trans_mm, 2.10, places=2)
        self.assertGreater(jitter.rms_trans_mm, 0.5)
        self.assertGreater(jitter.p95_trans_mm, jitter.rms_trans_mm)
        # Net drift from 10.8s (0.05mm) to 13.2s (2.45mm) = 2.40mm over 2.40s = 1.00 mm/s
        self.assertAlmostEqual(jitter.drift_rate_mm_s, 1.00, places=2)
        self.assertAlmostEqual(jitter.drift_rate_deg_s, 0.10, places=2)


if __name__ == "__main__":
    unittest.main()
