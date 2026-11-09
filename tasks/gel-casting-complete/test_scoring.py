"""Pure-Python scorer invariants; no simulator, USD or robot assets are imported.

Run beside task.py: python test_scoring.py
Synthetic states describe physical counterexamples, including an oversized gel
cavity and glass floating above its sealing pad. They are not a physics test.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass,field
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
sys.path.append(str(HERE.parent))


class _Spec:
    def __init__(self,name,**kwargs):
        self.name=name
        self.__dict__.update(kwargs)


def _decorator(fn):return fn


def _set_joint_targets(environment,name,targets):
    environment.unwrapped.scene[name].last_targets=dict(targets)

def _set_joint_gains(environment,name,stiffness,damping):
    environment.unwrapped.scene[name].last_gains=(dict(stiffness),dict(damping))


def _module(name,**attrs):
    result=types.ModuleType(name)
    result.__dict__.update(attrs)
    return result


@dataclass
class FakeBodyState:
    pos: np.ndarray
    quat_xyzw: np.ndarray=field(default_factory=lambda:np.array([0.,0.,1.,0.]))
    lin_vel: np.ndarray=field(default_factory=lambda:np.zeros(3))
    ang_vel: np.ndarray=field(default_factory=lambda:np.zeros(3))
    joints: dict=field(default_factory=dict)

    def resting(self,max_speed=.01,max_spin=.5):
        return bool(np.linalg.norm(self.lin_vel)<max_speed and np.linalg.norm(self.ang_vel)<max_spin)


def rot(q):
    x,y,z,w=np.asarray(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def load_task():
    """Only replace import-time scene interfaces, leaving the actual scorer intact."""
    task_file=_module('yam_arena.task_file',TABLE_TOP={'station':.7475,'tabletop':.75})
    arena=_module('yam_arena')
    arena.__path__=[]
    scene=_module('scenekit',UsdBody=_Spec,on_reset=_decorator,on_step=_decorator,
                  arm_joints=lambda environment,side,names:np.zeros(6),set_joint_targets=_set_joint_targets,
                  set_joint_gains=_set_joint_gains)
    with patch.dict(sys.modules,{'scenekit':scene,'yam_arena':arena,'yam_arena.task_file':task_file}):
        spec=importlib.util.spec_from_file_location('caster_task_under_test',HERE/'task.py')
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


TASK=load_task()


def assembly(*,at_start=False,cams=(0.,0.),lever=0.,quat=(0.,0.,1.,0.)):
    """Reference dimensions from the real asset specification, in metres."""
    orientation=np.asarray(quat,dtype=float)
    rotation=rot(orientation)
    origin=np.asarray(TASK.FRAME_START if at_start else TASK.STAND_START).copy()
    stand=np.asarray(TASK.STAND_START).copy()
    states={
        'frame':FakeBodyState(origin,orientation.copy(),joints=dict(zip(('left_hinge','right_hinge'),map(math.radians,cams)))),
        'rear':FakeBodyState(origin+rotation@np.array([0.,0.,.053]),orientation.copy()),
        'front':FakeBodyState(origin+rotation@np.array([.004,0.,.0485]),orientation.copy()),
        'stand':FakeBodyState(stand,orientation.copy(),joints={'clip_hinge':math.radians(lever)}),
    }
    return states


@contextmanager
def fake_runtime(objects,tcp_positions=None):
    """A minimal measured-state environment for the real monitor callback."""
    names=['left_hinge','right_hinge']
    q=np.array([[objects['frame'].joints.get(name,0.) for name in names]])
    data=types.SimpleNamespace(joint_pos=types.SimpleNamespace(torch=q),
                               joint_vel=types.SimpleNamespace(torch=np.zeros_like(q)))
    asset=types.SimpleNamespace(data=data,joint_names=names,last_effort=None,last_targets={},last_gains=None)
    def effort(*,value):asset.last_effort=np.asarray(value).copy()
    asset.actuators=types.SimpleNamespace(target_command=types.SimpleNamespace(set_effort_index=effort))
    environment=types.SimpleNamespace(unwrapped=types.SimpleNamespace(scene={'frame':asset},step_dt=.05))
    tcps=tcp_positions or {'left':np.array([0.,.45,1.]),'right':np.array([0.,-.45,1.])}
    station=_module('yam_arena.station',BASES={'left':'left','right':'right'})
    ik=_module('yam_arena.ik',tcp_pose=lambda joints,base:(tcps[base],np.eye(3)))
    scene=_module('yam_arena.scene',object_states=lambda environment,names:{name:objects[name] for name in names})
    constants=_module('yam_arena.constants',ARM_JOINTS=())
    arena=_module('yam_arena',ik=ik,station=station)
    arena.__path__=[]
    torch=_module('torch',zeros_like=np.zeros_like,clamp=lambda x,lo,hi:np.clip(x,lo,hi))
    with patch.dict(sys.modules,{'yam_arena':arena,'yam_arena.ik':ik,'yam_arena.station':station,
                                  'yam_arena.scene':scene,'yam_arena.constants':constants,'torch':torch}):
        yield environment,asset


class ScoringInvariants(unittest.TestCase):
    def setUp(self):TASK.reset_monitor()

    def observe(self,objects,**kwargs):
        with fake_runtime(objects,**kwargs) as (environment,asset):
            TASK.update(environment)
            return asset

    def history_completed(self):
        TASK.STATE['seen_open']=[True,True]
        TASK.STATE['transported']=True

    def test_open_detent_allows_manual_rotation_without_clamping_preload(self):
        for sign in (1.,-1.):
            target,stiffness,damping=TASK.cam_drive_parameters(sign*math.radians(-60.),sign)
            self.assertAlmostEqual(target,sign*math.radians(-100.))
            self.assertLess(stiffness*math.radians(51.),.014)
            self.assertAlmostEqual(damping,.00035)

    def test_cam_contact_preload_ramps_only_near_closed_position(self):
        for sign in (1.,-1.):
            samples=[TASK.cam_drive_parameters(sign*math.radians(a),sign) for a in (-30.,-10.,-5.,0.,2.)]
            self.assertTrue(all(abs(target-sign*math.radians(2.))<1e-12 for target,_,_ in samples))
            gains=[stiffness for _,stiffness,_ in samples]
            self.assertEqual(gains,sorted(gains))
            self.assertAlmostEqual(gains[0],.015)
            self.assertAlmostEqual(gains[-1],.30)
            self.assertAlmostEqual(samples[-1][2],.003)

    def test_runtime_gain_update_depends_only_on_measured_cam_angle(self):
        asset=self.observe(assembly(at_start=True,cams=(-100.,100.)))
        self.assertEqual(asset.last_gains[0],{'left_hinge':.015,'right_hinge':.015})
        # Histories and target location cannot strengthen a cam that is still open.
        self.history_completed()
        docked_asset=self.observe(assembly(cams=(-100.,100.)))
        self.assertEqual(docked_asset.last_gains,asset.last_gains)

    def test_idle_open_cams_cannot_complete_or_close_themselves(self):
        objects=assembly(at_start=True,cams=(-100.,100.))
        asset=self.observe(objects)
        self.assertFalse(TASK.success(objects))
        self.assertEqual(TASK.score(objects),0.)
        self.assertFalse(TASK.STATE['transported'])
        self.assertTrue(all(TASK.STATE['seen_open']))
        if asset.last_targets:
            self.assertAlmostEqual(asset.last_targets['left_hinge'],math.radians(-100.))
            self.assertAlmostEqual(asset.last_targets['right_hinge'],math.radians(100.))
        else:
            # Also supports the previous explicit-effort monitor during rollout.
            self.assertIsNotNone(asset.last_effort)
            np.testing.assert_allclose(asset.last_effort,0.,atol=1e-12)

    def test_closing_cams_earns_only_preparation_credit(self):
        self.observe(assembly(at_start=True,cams=(-100.,100.)))
        one=assembly(at_start=True,cams=(0.,100.))
        self.observe(one)
        self.assertAlmostEqual(TASK.score(one),.15)
        self.assertFalse(TASK.success(one))
        both=assembly(at_start=True)
        self.observe(both)
        self.assertAlmostEqual(TASK.score(both),.30)
        self.assertFalse(TASK.success(both))

    def test_closed_reset_cannot_earn_unperformed_cam_credit(self):
        objects=assembly(at_start=True)
        self.observe(objects)
        self.assertEqual(TASK.score(objects),0.)
        # Seeing only the right cam open must not earn left-cam credit.
        self.observe(assembly(at_start=True,cams=(0.,100.)))
        self.assertEqual(TASK.score(objects),0.)

    def test_correct_docked_released_assembly_succeeds_after_valid_history(self):
        objects=assembly()
        self.history_completed()
        for _ in range(21):self.observe(objects)
        self.assertTrue(TASK.geometry_metrics(objects)['docked'])
        self.assertTrue(TASK.success(objects))
        # The standard task_score scorer turns sustained success into1.0.
        self.assertGreaterEqual(TASK.score(objects),.9)

    def test_loose_frame_in_front_of_rear_supports_is_not_docked(self):
        objects=assembly()
        delta=rot(objects['stand'].quat_xyzw)@np.array([.0012,0.,0.])
        for name in ('frame','rear','front'):objects[name].pos+=delta
        self.history_completed();self.observe(objects)
        self.assertTrue(TASK.STATE['aligned'])
        self.assertFalse(TASK.STATE['registered'])
        self.assertFalse(TASK.STATE['docked'])
        self.assertAlmostEqual(TASK.score(objects),.3)

    def test_frame_penetrating_rear_supports_is_not_docked(self):
        objects=assembly()
        delta=rot(objects['stand'].quat_xyzw)@np.array([-.0006,0.,0.])
        for name in ('frame','rear','front'):objects[name].pos+=delta
        self.history_completed();self.observe(objects)
        self.assertFalse(TASK.STATE['registered'])
        self.assertFalse(TASK.STATE['docked'])

    def test_frame_outside_lateral_registration_is_not_docked(self):
        objects=assembly()
        delta=rot(objects['stand'].quat_xyzw)@np.array([0.,.001,0.])
        for name in ('frame','rear','front'):objects[name].pos+=delta
        self.history_completed();self.observe(objects)
        self.assertFalse(TASK.STATE['registered'])
        self.assertFalse(TASK.STATE['docked'])

    def test_whole_cassette_floating_above_gasket_is_not_docked(self):
        objects=assembly()
        for name in ('frame','rear','front'):objects[name].pos[2]+=.00055
        self.history_completed();self.observe(objects)
        self.assertTrue(TASK.STATE['aligned'])
        self.assertFalse(TASK.STATE['docked'])

    def test_near_zero_clip_angle_without_shoe_contact_is_not_closed(self):
        objects=assembly(lever=-2.)
        self.history_completed();self.observe(objects)
        self.assertTrue(TASK.STATE['docked'])
        self.assertFalse(TASK.STATE['clip_shoe_seated'])
        self.assertFalse(TASK.STATE['clip_closed'])

    def test_finishing_pose_alone_cannot_forge_cam_and_transport_history(self):
        objects=assembly()
        for _ in range(81):self.observe(objects)
        self.assertTrue(TASK.STATE['stable'])
        self.assertFalse(TASK.success(objects))
        TASK.STATE['seen_open']=[True,True]
        self.assertFalse(TASK.success(objects))
        TASK.STATE['seen_open']=[True,False]
        TASK.STATE['transported']=True
        self.assertFalse(TASK.success(objects))

    def test_oversized_gel_gap_is_not_a_valid_cassette(self):
        objects=assembly()
        # 1.6mm gap cannot be a correctly assembled1mm spacer cassette.
        objects['front'].pos += rot(objects['front'].quat_xyzw)@np.array([.0006,0.,0.])
        self.history_completed();self.observe(objects)
        self.assertFalse(TASK.geometry_metrics(objects)['docked'])
        self.assertFalse(TASK.success(objects))

    def test_interpenetrating_glass_cannot_score_as_aligned(self):
        objects=assembly()
        # Translate inward1.5mm: the1mm cavity becomes a0.5mm overlap.
        objects['front'].pos += rot(objects['front'].quat_xyzw)@np.array([-.0015,0.,0.])
        self.history_completed();self.observe(objects)
        self.assertFalse(TASK.success(objects))
        self.assertEqual(TASK.score(objects),0.)

    def test_glass_floating_above_gasket_cannot_count_as_docked(self):
        objects=assembly()
        objects['rear'].pos[2]+=.001
        objects['front'].pos[2]+=.001
        self.history_completed();self.observe(objects)
        self.assertFalse(TASK.geometry_metrics(objects)['docked'])
        self.assertFalse(TASK.success(objects))

    def test_plate_yaw_that_crosses_the_cavity_is_rejected(self):
        objects=assembly()
        angle=math.pi+math.radians(2.)
        objects['front'].quat_xyzw=np.array([0.,0.,math.sin(angle/2),math.cos(angle/2)])
        # The origin remains nominal, but a2degree yaw over101mm width crosses
        # the nominal1mm cavity at one edge. Centre-only checks would miss it.
        self.history_completed();self.observe(objects)
        self.assertFalse(TASK.success(objects))

    def test_large_plate_misalignment_loses_all_credit(self):
        objects=assembly()
        objects['front'].pos[1]+=.008
        self.history_completed();self.observe(objects)
        self.assertFalse(TASK.success(objects))
        self.assertEqual(TASK.score(objects),0.)

    def test_open_lever_or_open_cam_prevents_success(self):
        for objects in (assembly(lever=-60.),assembly(cams=(-80.,0.))):
            with self.subTest(objects=objects):
                TASK.reset_monitor();self.history_completed();self.observe(objects)
                self.assertFalse(TASK.success(objects))

    def test_nearly_closed_cams_without_pad_contact_do_not_count_as_clamped(self):
        objects=assembly(cams=(-7.,7.))
        self.history_completed();self.observe(objects)
        self.assertFalse(any(TASK.geometry_metrics(objects)['cams_closed']))
        self.assertFalse(TASK.success(objects))
        self.assertEqual(TASK.score(objects),0.)

    def test_motion_or_hands_near_work_prevents_success(self):
        objects=assembly()
        objects['rear'].lin_vel=np.array([.06,0.,0.])
        self.history_completed()
        for _ in range(81):self.observe(objects)
        self.assertFalse(TASK.success(objects))
        objects['rear'].lin_vel[:]=0.
        target=objects['stand'].pos+np.array([0.,0.,.060])
        self.observe(objects,tcp_positions={'left':target,'right':np.array([0.,-.45,1.])})
        self.assertFalse(TASK.success(objects))

    def test_a_drop_is_irreversible_even_if_pose_is_later_restored(self):
        dropped=assembly()
        dropped['rear'].pos[2]=TASK.TOP-.04
        self.history_completed();self.observe(dropped)
        self.assertTrue(TASK.STATE['violations'])
        self.assertEqual(TASK.score(dropped),0.)
        restored=assembly()
        self.observe(restored)
        self.assertFalse(TASK.success(restored))
        self.assertEqual(TASK.score(restored),0.)

    def test_missing_joint_measurement_does_not_invent_a_closed_cam(self):
        objects=assembly()
        objects['frame'].joints.pop('left_hinge')
        self.history_completed();self.observe(objects)
        self.assertFalse(TASK.success(objects))


class StabilityInvariants(unittest.TestCase):
    def setUp(self):TASK.reset_monitor()

    def trajectory(self,transform=lambda objects,t:None,seconds=6.,dt=.05):
        history=[]
        for i in range(round(seconds/dt)+1):
            t=i*dt
            objects=assembly()
            transform(objects,t)
            history.append((t,TASK.pose_stability(objects,dt,True)))
        return history

    def first_full_hold(self,history):
        start=None
        for time,measurement in history:
            if not measurement['stable']:start=None
            elif start is None:start=time
            if start is not None and time-start>=TASK.SUCCESS_HOLD_S-1e-9:return time
        return None

    def test_positive_requires_a_full_second_then_three_more_seconds(self):
        history=self.trajectory(seconds=4.)
        self.assertFalse(any(result['stable'] for time,result in history if time<1.))
        self.assertTrue(next(result['stable'] for time,result in history if time==1.))
        self.assertAlmostEqual(self.first_full_hold(history),4.)
        self.assertEqual(history[20][1]['stability_samples'],21)

    def test_slow_drift_cannot_accumulate_through_the_success_hold(self):
        # 0.1 mm/s is far below the former 10 mm/s velocity limit. A separate
        # one-second window would accept it, but it moves 0.4 mm over 1+3 s.
        def drift(objects,t):
            for body in objects.values():body.pos[0]+=.0001*t
        self.assertIsNone(self.first_full_hold(self.trajectory(drift)))

    def test_slow_whole_assembly_rotation_cannot_pass(self):
        def turn(objects,t):
            angle=math.radians(.1*t)
            pivot=objects['stand'].pos.copy()
            matrix=rot(np.array([0.,0.,math.sin(angle/2),math.cos(angle/2)]))
            for body in objects.values():
                body.pos=pivot+matrix@(body.pos-pivot)
                body.quat_xyzw=np.array([0.,0.,math.sin((math.pi+angle)/2),math.cos((math.pi+angle)/2)])
        self.assertIsNone(self.first_full_hold(self.trajectory(turn)))

    def test_slow_joint_drift_cannot_pass(self):
        def drift(objects,t):objects['frame'].joints['left_hinge']=math.radians(.2*t)
        self.assertIsNone(self.first_full_hold(self.trajectory(drift)))

    def test_joint_oscillation_inside_closed_angle_range_still_fails_stability(self):
        def oscillate(objects,t):objects['frame'].joints['left_hinge']=math.radians(.35*math.sin(2*math.pi*t))
        self.assertIsNone(self.first_full_hold(self.trajectory(oscillate)))

    def test_post_release_displacement_restarts_the_full_qualification(self):
        def movement(objects,t):
            if t>=3.:
                for body in objects.values():body.pos[0]+=.0005
        history=self.trajectory(movement,seconds=8.)
        self.assertFalse(dict(history)[3.]['stable'])
        self.assertAlmostEqual(self.first_full_hold(history),7.)

    def test_pairwise_range_rejects_opposite_displacements_from_an_anchor(self):
        def oscillate(objects,t):
            for body in objects.values():body.pos[0]+=.00011*math.sin(2*math.pi*t)
        self.assertIsNone(self.first_full_hold(self.trajectory(oscillate)))

    def test_quaternion_sign_changes_and_residual_velocity_do_not_invent_motion(self):
        def unchanged(objects,t):
            for body in objects.values():
                body.quat_xyzw*= -1. if round(t/.05)%2 else 1.
                body.lin_vel=np.array([.013,0.,0.])
                body.ang_vel=np.array([0.,.5,0.])
        self.assertAlmostEqual(self.first_full_hold(self.trajectory(unchanged)),4.)

    def test_coarse_motion_guards_reject_fast_aliased_samples(self):
        for linear,angular in ((.06,0.),(0.,2.1)):
            TASK.reset_monitor()
            def reported_motion(objects,t):
                objects['rear'].lin_vel=np.array([linear,0.,0.])
                objects['rear'].ang_vel=np.array([0.,angular,0.])
            self.assertIsNone(self.first_full_hold(self.trajectory(reported_motion)))

    def test_missing_sample_or_reset_clears_qualification(self):
        self.assertTrue(self.trajectory(seconds=1.)[-1][1]['stable'])
        missing=TASK.pose_stability(None,.05,True)
        self.assertFalse(missing['stable'])
        self.assertEqual(missing['stability_reason'],'missing_or_invalid_sample')
        self.assertFalse(TASK.pose_stability(assembly(),.05,True)['stable'])
        self.trajectory(seconds=1.)
        TASK.reset_monitor()
        self.assertFalse(TASK.pose_stability(assembly(),.05,True)['stable'])

    def test_missing_joint_or_invalid_quaternion_cannot_qualify(self):
        for invalid in ('joint','quaternion'):
            TASK.reset_monitor()
            self.trajectory(seconds=1.)
            objects=assembly()
            if invalid=='joint':objects['stand'].joints.clear()
            else:objects['rear'].quat_xyzw[:]=0.
            result=TASK.pose_stability(objects,.05,True)
            self.assertFalse(result['stable'])
            self.assertEqual(result['stability_reason'],'missing_or_invalid_sample')

    def test_losing_released_geometry_clears_qualification(self):
        self.trajectory(seconds=1.)
        self.assertFalse(TASK.pose_stability(assembly(),.05,False)['stable'])
        self.assertFalse(TASK.pose_stability(assembly(),.05,True)['stable'])

    def test_qualification_uses_actual_control_time(self):
        for dt in (.025,.1):
            TASK.reset_monitor()
            history=self.trajectory(seconds=1.,dt=dt)
            self.assertFalse(any(result['stable'] for time,result in history if time<1.-1e-9))
            self.assertTrue(history[-1][1]['stable'])


if __name__=='__main__':unittest.main(verbosity=2)
