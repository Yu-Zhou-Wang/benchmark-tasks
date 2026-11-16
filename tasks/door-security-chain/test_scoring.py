"""Rubric counterexamples; these do not replace the physical controls."""
import unittest
import numpy as np
from task import evaluate

class Counterexamples(unittest.TestCase):
    def result(self,p=(-.013,-.325,.245),angle=22.3,speed=0.,head_speed=0.,clear=True,intact=True,r=None):
        return evaluate(p,np.eye(3) if r is None else r,angle,speed,head_speed,clear,intact)
    def test_initial_latched_is_zero(self):
        self.assertEqual(self.result()['partial_score'],0.)
    def test_closed_but_latched_is_zero(self):
        self.assertEqual(self.result(angle=0)['partial_score'],0.)
    def test_open_angle_alone_cannot_succeed(self):
        s=self.result(angle=60)
        self.assertFalse(s['complete']);self.assertEqual(s['partial_score'],0.)
    def test_wide_end_still_captive_is_quarter(self):
        s=self.result(p=(-.013,-.260,.245),angle=0)
        self.assertEqual(s['partial_score'],.25);self.assertFalse(s['released'])
    def test_exit_credit_does_not_require_a_prescribed_door_angle(self):
        self.assertEqual(self.result(p=(-.013,-.260,.245),angle=10)['partial_score'],.25)
    def test_near_exit_but_outside_tolerance_is_zero(self):
        self.assertEqual(self.result(p=(-.013,-.270,.245),angle=0)['partial_score'],0.)
    def test_withdrawal_without_door_opening_is_half(self):
        s=self.result(p=(-.032,-.260,.245),angle=0)
        self.assertEqual(s['partial_score'],.5);self.assertFalse(s['complete'])
    def test_open_and_released_but_moving_cannot_finish(self):
        self.assertFalse(self.result(p=(-.032,-.260,.245),angle=55,speed=3)['complete'])
        self.assertFalse(self.result(p=(-.032,-.260,.245),angle=55,head_speed=.1)['complete'])
    def test_holding_the_fitting_cannot_finish(self):
        self.assertFalse(self.result(p=(-.032,-.260,.245),angle=55,clear=False)['complete'])
    def test_broken_continuity_erases_all_credit(self):
        s=self.result(p=(-.032,-.260,.245),angle=55,intact=False)
        self.assertEqual(s['partial_score'],0.);self.assertFalse(s['complete'])
    def test_full_geometry_can_qualify_for_hold(self):
        self.assertTrue(self.result(p=(-.032,-.260,.245),angle=55)['complete'])
    def test_head_center_outside_does_not_mean_whole_head_clear(self):
        # Rotate the thin head edge-on: its centre is outside, its wide edge
        # still intersects the housing. The scorer must use the whole head.
        r=np.array([[0,0,1.],[0,1.,0],[-1.,0,0]])
        s=self.result(p=(-.025,-.260,.245),angle=55,r=r)
        self.assertFalse(s['released']);self.assertFalse(s['complete'])

if __name__=='__main__':unittest.main()
