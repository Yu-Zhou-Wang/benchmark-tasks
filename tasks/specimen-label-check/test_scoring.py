"""Counterexamples for identity, routing, release and continuous stability."""
from dataclasses import replace
from types import SimpleNamespace
import math
import unittest

import numpy as np
from scoring import Monitor, in_processing, on_floor_in_tray
from specification import FLOOR_Z, SIZE, SPECIMENS, TOP, TRAYS


def state(xy, bottom=FLOOR_Z, quat=(0,0,0,1), vel=(0,0,0), spin=(0,0,0)):
    return SimpleNamespace(pos=np.array([*xy, bottom+SIZE[2]/2],float),
        quat_xyzw=np.array(quat,float), lin_vel=np.array(vel,float), ang_vel=np.array(spin,float))


CLEAR = np.array([[[0,0,1.1], [0.1,0.1,1.2]]])


def initial():
    return {s.key: state(s.xy,bottom=TOP) for s in SPECIMENS}


def finished():
    return {s.key: state(TRAYS[s.destination]) for s in SPECIMENS}


class ScoringTest(unittest.TestCase):
    def setUp(self):
        self.monitor = Monitor()

    def advance(self, objects, steps=40, bounds=CLEAR):
        for _ in range(steps):
            self.monitor.update(objects,bounds,0.05)
        return self.monitor.value

    def test_both_identifiers_determine_routing(self):
        a=SPECIMENS[0]
        self.assertEqual(a.destination,"PROCESS")
        self.assertEqual(replace(a,name="C. LEE").destination,"QUARANTINE")
        self.assertEqual(replace(a,sample_id="S71").destination,"QUARANTINE")
        self.assertEqual(SPECIMENS[1].destination,"QUARANTINE")

    def test_idle_never_scores(self):
        self.assertEqual(self.advance(initial(),100),0)

    def test_two_seconds_are_required_for_each_credit(self):
        self.assertEqual(self.advance(finished(),39),0)
        self.assertEqual(self.advance(finished(),1),1)

    def test_partial_credit_either_specimen(self):
        for s in SPECIMENS:
            self.monitor.reset()
            objects=initial()
            objects[s.key]=finished()[s.key]
            self.assertEqual(self.advance(objects),0.5)

    def test_mismatch_in_processing_cancels_other_credit(self):
        objects=finished()
        objects["specimen_b"]=state((TRAYS["PROCESS"][0]+0.025,TRAYS["PROCESS"][1]))
        self.assertEqual(self.advance(objects),0)
        self.assertTrue(self.monitor.unsafe)

    def test_both_quarantined_only_earns_half(self):
        objects=finished()
        objects["specimen_a"]=state(TRAYS["QUARANTINE"])
        self.assertEqual(self.advance(objects),0.5)

    def test_swapped_trays_scores_zero(self):
        objects={s.key: state(TRAYS["QUARANTINE" if s.destination=="PROCESS" else "PROCESS"]) for s in SPECIMENS}
        self.assertEqual(self.advance(objects),0)

    def test_held_motionless_and_open_gripper_nearby_do_not_count(self):
        objects=finished()
        p=objects["specimen_a"].pos
        for bounds in (np.array([[p-0.010,p+0.010]]), np.array([[p+[-0.01,0.023,-0.01],p+[0.01,0.027,0.01]]])):
            self.monitor.reset()
            self.assertEqual(self.advance(objects,bounds=bounds),0.5)

    def test_missing_or_invalid_robot_bounds_fail_closed(self):
        for bounds in ([],np.zeros((2,3)),np.full((1,2,3),np.nan),[[[1,1,1],[0,0,0]]]):
            self.monitor.reset()
            self.assertEqual(self.advance(finished(),bounds=bounds),0)

    def test_airborne_walltop_sideways_upside_down_overhang_and_motion(self):
        xy=TRAYS["PROCESS"]
        examples=(state(xy,bottom=FLOOR_Z+0.010),state(xy,bottom=FLOOR_Z+0.025),
            state(xy,quat=(math.sqrt(0.5),0,0,math.sqrt(0.5))),state(xy,quat=(1,0,0,0)),
            state((xy[0]+0.05,xy[1])),state(xy,vel=(0.02,0,0)),state(xy,spin=(0,0,0.3)))
        for bad in examples:
            self.monitor.reset()
            objects=finished()
            objects["specimen_a"]=bad
            self.assertEqual(self.advance(objects),0.5)

    def test_low_held_or_sideways_mismatch_still_triggers_gate(self):
        self.assertTrue(in_processing(state(TRAYS["PROCESS"],bottom=FLOOR_Z+0.005)))
        self.assertTrue(in_processing(state(TRAYS["PROCESS"],quat=(math.sqrt(0.5),0,0,math.sqrt(0.5)))))

    def test_stability_break_restarts_hold_and_removes_credit(self):
        objects=finished()
        self.advance(objects,39)
        objects["specimen_a"].lin_vel[0]=0.02
        self.advance(objects,1)
        objects["specimen_a"].lin_vel[0]=0
        self.assertEqual(self.advance(objects,39),0.5)
        self.assertEqual(self.advance(objects,1),1)
        objects["specimen_a"]=initial()["specimen_a"]
        self.assertEqual(self.advance(objects,1),0.5)

    def test_slow_drift_restarts_hold(self):
        objects=finished()
        self.advance(objects,39)
        objects["specimen_a"].pos[0]+=0.003
        self.assertEqual(self.advance(objects,1),0.5)
        self.assertEqual(self.advance(objects,39),1)

    def test_invalid_and_missing_states_remove_all_credit(self):
        self.advance(finished())
        objects=finished()
        objects["specimen_b"].pos[0]=math.nan
        self.assertEqual(self.advance(objects,1),0)
        self.assertFalse(self.monitor.valid)
        objects=finished()
        del objects["specimen_b"]
        self.assertEqual(self.advance(objects,1),0)

    def test_reset_clears_all_history(self):
        self.advance(finished())
        self.monitor.reset()
        self.assertEqual(self.monitor.value,0)
        self.assertEqual(self.advance(finished(),39),0)

    def test_position_and_yaw_within_wide_tray_are_irrelevant(self):
        objects=finished()
        for s in SPECIMENS:
            objects[s.key].pos[:2]+=(0.012,0.009)
            objects[s.key].quat_xyzw[:]=(0,0,math.sin(0.3),math.cos(0.3))
            self.assertTrue(on_floor_in_tray(objects[s.key],s.destination))
        self.assertEqual(self.advance(objects),1)


if __name__ == "__main__":
    unittest.main()
