"""Regression checks for timed eligibility and end-of-trial seating."""

import unittest

import numpy as np

from geometry import PHYSICS_HZ, REST_Z, TOOLS
from scoring import HOLD_INTERVALS, Score, measure_seat


class HoldScoringTest(unittest.TestCase):
    def setUp(self):
        self.poses = np.tile([0.30, 0.0, REST_Z + 0.15, 0, 0, 0, 1], (6, 1))
        self.forces = np.zeros((6, 12, 3))
        for i, tool in enumerate(TOOLS):
            barrel, cap = 2 * i, 2 * i + 1
            self.poses[cap, 1] = self.poses[barrel, 1] = i * 0.10
            self.poses[barrel, 2] += (tool.mouth_z - tool.shoulder_z) / 1000
            self.forces[barrel, 0:2, 1] = [-1, 1]
        self.score = Score()
        self.seat_with_both_hands()

    def seat_with_both_hands(self):
        self.forces[1::2, 2:4, 1] = [-1, 1]
        self.run_steps(1)
        self.forces[1::2, 2:4, 1] = 0

    def run_steps(self, steps):
        for _ in range(steps):
            self.score.sample(self.poses, self.forces)

    def test_partial_stays_on_without_credit(self):
        self.poses[3, 2] -= 0.005
        self.run_steps(HOLD_INTERVALS + 1)
        marker = self.score.metrics["marker"]
        self.assertTrue(marker["barrel_only_cap_down"])
        self.assertFalse(marker["seated"])
        self.assertAlmostEqual(marker["gap_mm"], 5.0)
        self.assertAlmostEqual(self.score.score, 2 / 3)

    def test_separation_interrupts_hold_and_revokes_completed_credit(self):
        self.run_steps(PHYSICS_HZ)
        self.poses[3, 2] -= 0.015
        self.run_steps(1)
        self.poses[3, 2] += 0.015
        self.seat_with_both_hands()
        self.run_steps(HOLD_INTERVALS - PHYSICS_HZ)
        self.assertFalse(self.score.metrics["marker"]["credited"])
        self.run_steps(HOLD_INTERVALS + 1)
        self.assertEqual(self.score.score, 1.0)
        self.poses[3, 2] -= 0.015
        self.run_steps(1)
        self.assertAlmostEqual(self.score.score, 2 / 3)

    def test_opposed_cap_contacts_invalidate_hold(self):
        self.forces[1, 2:4, 1] = [-1, 1]
        self.run_steps(HOLD_INTERVALS + 1)
        self.assertAlmostEqual(self.score.score, 2 / 3)
        self.assertFalse(self.score.metrics["pen"]["barrel_only_cap_down"])

    def test_seated_without_a_cap_grasp_cannot_start_proof(self):
        self.score = Score()
        self.run_steps(HOLD_INTERVALS + 1)
        self.assertEqual(self.score.score, 0.)

    def test_all_around_gap_and_highlighter_roll(self):
        self.poses[5, 3:] = [0, 0, 2**-0.5, 2**-0.5]
        self.assertFalse(measure_seat(2, self.poses[4], self.poses[5])["seated"])
        self.poses[5, 3:] = [0, np.sin(0.015), 0, np.cos(0.015)]
        # Centre gap is 0.4 mm, but the high corner exceeds 0.6 mm.
        self.poses[5, 2] -= 0.0004
        m = measure_seat(2, self.poses[4], self.poses[5])
        self.assertLess(m["gap_mm"], 0.6)
        self.assertGreater(m["rim_gap_max_mm"], 0.6)
        self.assertFalse(m["seated"])


if __name__ == "__main__":
    unittest.main()
