"""Counterexamples for the contact zones, latched failure and placement scoring."""
import unittest
from types import SimpleNamespace
import numpy as np
import monitor as m
from specification import *

def state(center=(.32,0.,PAD_TOP+.0135),q=(1.,0.,0.,0.),v=(0.,0.,0.),w=(0.,0.,0.)):
    return SimpleNamespace(pos=np.array(center)-m.rotation(q)@np.array([0,0,.0135]),
        quat_xyzw=np.array(q),lin_vel=np.array(v),ang_vel=np.array(w))

class Scoring(unittest.TestCase):
    def setUp(self):m.ERROR=None;m.reset(None);m.M['polls']=20
    def removed(self):m.evaluate(state(center=(.32,0.,TOP+.2)),False)
    def hold(self,s=None,clear=True,n=40):
        for _ in range(n):m.evaluate(s or state(),clear)
    def test_idle_zero(self):
        self.hold(state(center=(*BOTTLE_XY,TOP+.09),q=(0,0,0,1)));self.assertEqual(m.M['score'],0)
    def test_clean_removal_partial(self):self.removed();self.assertEqual(m.M['score'],.25)
    def test_complete_requires_two_seconds(self):
        self.removed();self.hold(n=39);self.assertEqual(m.M['score'],.25)
        self.hold(n=1);self.assertEqual(m.M['score'],1)
    def test_no_removal_no_full_score(self):self.hold();self.assertEqual(m.M['score'],0)
    def test_contamination_latches(self):
        self.removed();m.M['contaminated']=True;self.hold();self.assertEqual(m.M['score'],0)
    def test_held_wrong_place_moving_wrong_side_fail_full(self):
        for s,clear in [(state(),False),(state(center=(.4,0.,PAD_TOP+.0135)),True),
            (state(v=(.02,0,0)),True),(state(q=(0,0,0,1)),True),(state(w=(0,0,.3)),True)]:
            self.setUp();self.removed();self.hold(s,clear);self.assertEqual(m.M['score'],.25)
    def test_off_table_latches(self):
        self.removed();m.evaluate(state(center=(.32,0.,TOP-.1)),True);self.hold();self.assertEqual(m.M['score'],0)
    def test_error_and_missing_contacts_fail_closed(self):
        self.removed();m.M['error']='sensor failure';self.hold();self.assertEqual(m.M['score'],0)
        self.setUp();self.removed();m.M['polls']=0;self.hold();self.assertEqual(m.M['score'],0)
    def test_nonfinite_fails_closed(self):
        self.removed();m.evaluate(state(v=(float('nan'),0,0)),True);self.hold();self.assertEqual(m.M['score'],0)
    def test_hold_resets_on_motion(self):
        self.removed();self.hold(n=30);self.hold(state(center=(.323,0,PAD_TOP+.0135)),n=20);self.assertEqual(m.M['score'],.25)
    def test_full_credit_removed_if_moved(self):
        self.removed();self.hold();self.assertEqual(m.M['score'],1)
        m.evaluate(state(center=(.42,0,PAD_TOP+.0135)),True);self.assertEqual(m.M['score'],.25)
    def test_reset_clears_history(self):
        self.removed();m.M['contaminated']=True;m.reset(None)
        self.assertFalse(m.M['contaminated']);self.assertFalse(m.M['removed']);self.assertEqual(m.M['score'],0)
    def test_protected_interior_and_rim(self):
        for p in [(0,0,.024),(.0233,0,.012),(.029,0,0)]:self.assertTrue(m.protected(np.array(p)))
    def test_allowed_exterior(self):
        for p in [(.029,0,.012),(0,0,.027),(.029,0,.025)]:self.assertFalse(m.protected(np.array(p)))
if __name__=='__main__':unittest.main()
