"""Run with:  python -m unittest discover -s tests -v   (from the code/ folder)"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ftqc.allocation import fault_aware_mixer, mixer_matrix, stock_mixer  # noqa: E402
from ftqc.params import QuadParams  # noqa: E402
from ftqc.sim import simulate  # noqa: E402


class TestAllocation(unittest.TestCase):
    def setUp(self):
        self.p = QuadParams(mass=0.029)
        self.hover = np.array([self.p.weight, 0.0, 0.0, 0.0])

    def test_healthy_hover_even_split(self):
        f, info = fault_aware_mixer(self.hover, self.p, np.full(4, self.p.f_max))
        np.testing.assert_allclose(f, self.p.weight / 4, atol=1e-9)
        self.assertFalse(info["yaw_dropped"])

    def test_matches_stock_when_unsaturated(self):
        v = np.array([self.p.weight, 1e-4, -5e-5, 2e-5])
        f_ftc, _ = fault_aware_mixer(v, self.p, np.full(4, self.p.f_max))
        np.testing.assert_allclose(f_ftc, stock_mixer(v, self.p), atol=1e-9)

    def test_fault_drops_yaw_and_pairs_motors(self):
        upper = np.full(4, self.p.f_max)
        upper[0] = 0.25 * self.p.f_max
        f, info = fault_aware_mixer(self.hover, self.p, upper)
        self.assertTrue(info["yaw_dropped"])
        self.assertLessEqual(f[0], upper[0] + 1e-12)
        self.assertAlmostEqual(f[0], f[2], places=6)      # diagonal partner matches
        w = mixer_matrix(self.p) @ f
        self.assertAlmostEqual(w[1], 0.0, places=9)       # no roll torque
        self.assertAlmostEqual(w[2], 0.0, places=9)       # no pitch torque

    def test_overloaded_fault_gives_all_the_thrust_the_motors_can(self):
        # 31 g drone after the fault: weight 0.304 N, but at most 2 * 0.25 f_max + 2 f_max = 0.300 N
        p = QuadParams(mass=0.031)
        upper = np.full(4, p.f_max)
        upper[0] = 0.25 * p.f_max
        f, info = fault_aware_mixer(np.array([p.weight, 0.0, 0.0, 0.0]), p, upper)
        self.assertAlmostEqual(f.sum(), 2.5 * p.f_max, places=9)
        self.assertAlmostEqual(info["thrust"], 2.5 * p.f_max, places=9)
        self.assertEqual(info["att_scale"], 1.0)

    def test_thrust_request_within_reach_is_met_exactly_after_fault(self):
        upper = np.full(4, self.p.f_max)
        upper[1] = 0.25 * self.p.f_max
        v = np.array([0.27, 2e-4, -1e-4, 0.0])
        f, info = fault_aware_mixer(v, self.p, upper)
        w = mixer_matrix(self.p) @ f
        np.testing.assert_allclose(w[:3], v[:3], atol=1e-9)    # thrust, roll and pitch all met
        self.assertTrue(np.all(f <= upper + 1e-12) and np.all(f >= 0.0))


class TestClosedLoop(unittest.TestCase):
    def test_ftc_lands_upright_and_pid_does_not(self):
        p = QuadParams(mass=0.029)
        _, ftc = simulate(mode="ftc", fault_motor=1, p=p)
        _, pid = simulate(mode="pid", fault_motor=1, p=p)
        self.assertEqual(ftc["result"], "landed")
        self.assertLess(ftc["max_tilt_after_fault"], 20.0)
        self.assertNotEqual(pid["result"], "landed")


if __name__ == "__main__":
    unittest.main()
