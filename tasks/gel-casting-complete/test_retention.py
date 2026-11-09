"""Physical retention bench with an open-cam control and a torsion disturbance.

    python test_retention.py --seconds 2

The frame alone is explicitly bolted to a test fixture above empty space. There
are no glass attachments, cradles, floors, pose corrections, or robotic grasps.
All copies use the task's implicit two-well cam detent. One begins open; a third
closed copy receives measured torque pulses on its front plate. This is a
retention bench, not a robot demonstration.
Preload is established for 0.25 s in zero gravity before the gravity load is
applied, representing closure while the task's alignment cradle bears weight.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds',type=float,default=2.)
parser.add_argument('--preload-seconds',type=float,default=.25)
parser.add_argument('--disturbance-torque',type=float,default=.001,
                    help='Front-plate yaw torque in N m, applied in two opposite 0.2 s pulses.')
parser.add_argument('--report',type=Path,default=Path(__file__).with_name('retention_check.json'))
args=parser.parse_args()
HERE=Path(__file__).resolve().parent
from source_fingerprint import capture
source_before=capture(HERE,extra_files=(Path(__file__).name,),include_demo=False)
from isaaclab.app import AppLauncher
launcher=AppLauncher(headless=True)
app=launcher.app

import numpy as np
import torch
from pxr import Gf,Sdf,UsdGeom,UsdPhysics
import omni.usd
import omni.physics.tensors as physics_tensors
import isaaclab.sim as sim_utils
import geometry as g
from task import cam_drive_parameters

DT=1/240
sim=sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT,gravity=(0.,0.,0.),use_fabric=False))
stage=omni.usd.get_context().get_stage()
origins={'Closed':np.array([0.,-.13,.20]),'Open':np.array([0.,.13,.20]),
         'Torsion':np.array([0.,.39,.20])}


def reference(path,filename,position):
    prim=stage.DefinePrim(path,'Xform')
    prim.GetReferences().AddReference(str(HERE/'task_assets'/filename))
    xf=UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    return prim


for name,origin in origins.items():
    root='/World/'+name
    frame_prim=reference(root+'Frame','casting_frame.usda',origin)
    frame_prim.GetAttribute('physxArticulation:solverPositionIterationCount').Set(64)
    frame_prim.GetAttribute('physxArticulation:solverVelocityIterationCount').Set(8)
    for joint in ('left_hinge','right_hinge'):
        prim=stage.GetPrimAtPath(root+'Frame/'+joint)
        drive=UsdPhysics.DriveAPI.Apply(prim,'angular')
        gain=.015 if name=='Open' else .30
        damping=.00035 if name=='Open' else .003
        drive.CreateStiffnessAttr(gain*math.pi/180.)
        drive.CreateDampingAttr(damping*math.pi/180.)
        drive.CreateMaxForceAttr(.08)
    reference(root+'Rear','rear_plate.usda',origin+g.REAR_POS)
    reference(root+'Front','front_plate.usda',origin+g.FRONT_POS)
    fixture=UsdPhysics.FixedJoint.Define(stage,root+'Frame/bench_fixture')
    fixture.CreateBody1Rel().SetTargets([Sdf.Path(root+'Frame/body')])
    fixture.CreateLocalPos0Attr(Gf.Vec3f(*origin))
    fixture.CreateLocalPos1Attr(Gf.Vec3f(0.))
    fixture.CreateLocalRot0Attr(Gf.Quatf(1.))
    fixture.CreateLocalRot1Attr(Gf.Quatf(1.))
    fixture.CreateCollisionEnabledAttr(False)

sim.reset()
view=physics_tensors.create_simulation_view('torch')
view.set_subspace_roots('/')
indices=torch.tensor([0],dtype=torch.int32,device=sim.device)
benches={}
for name in origins:
    root='/World/'+name
    art=view.create_articulation_view(root+'Frame*')
    if art.count!=1 or art.max_dofs!=2:
        raise RuntimeError(f'{name}: expected one two-joint frame, got {art.count}/{art.max_dofs}')
    names=list(art.shared_metatype.dof_names)
    if set(names)!={'left_hinge','right_hinge'}:raise RuntimeError(f'Unexpected joints: {names}')
    # Test initialization only. Subsequent steps use drive targets, never poses.
    positions=tuple(0. if name!='Open' else math.radians(-100. if joint=='left_hinge' else 100.) for joint in names)
    art.set_dof_positions(torch.tensor([positions],dtype=torch.float32,device=sim.device),indices)
    art.set_dof_velocities(torch.zeros((1,2),dtype=torch.float32,device=sim.device),indices)
    bodies={part:view.create_rigid_body_view(root+part.title()) for part in ('rear','front')}
    if any(body.count!=1 for body in bodies.values()):raise RuntimeError(f'{name}: glass body view missing')
    benches[name]={'frame':art,'bodies':bodies,'joint_names':names,'initial_angles_degrees':list(np.degrees(positions)),'trace':[]}


def rotation(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def measure(name,bench):
    poses={part:body.get_transforms()[0].cpu().numpy().astype(float) for part,body in bench['bodies'].items()}
    errors={part:(pose[:3]-origins[name]-np.asarray(g.REAR_POS if part=='rear' else g.FRONT_POS))
            for part,pose in poses.items()}
    rear,front=poses['rear'],poses['front']
    nr=rotation(rear[3:])[:,0]
    fr=rotation(front[3:])
    gaps=[float(nr@(front[:3]+fr@np.array([-.0015,y,z])-rear[:3])-.0015)
          for y in (-.044,.044) for z in (-.035,.035)]
    bottoms=[]
    for part,pose in poses.items():
        rr=rotation(pose[3:]);halfheight=(g.REAR_SIZE if part=='rear' else g.FRONT_SIZE)[2]/2
        bottoms.extend(float((pose[:3]+rr@np.array([0.,y,-halfheight])-origins[name])[2]-.012) for y in (-.044,.044))
    degrees=torch.rad2deg(bench['frame'].get_dof_positions()[0]).cpu().tolist()
    return {'cam_degrees':[degrees[bench['joint_names'].index(joint)] for joint in ('left_hinge','right_hinge')],
            'plate_position_error_m':{part:error.tolist() for part,error in errors.items()},
            'gap_samples_m':gaps,'glass_bottom_error_m':bottoms,
            'max_plate_position_error_m':max(float(np.linalg.norm(error)) for error in errors.values()),
            'max_downward_drop_m':max(float(-error[2]) for error in errors.values())}


def retained(measurement):
    return bool(measurement['max_plate_position_error_m']<.002
        and max(abs(v) for v in measurement['glass_bottom_error_m'])<.0005
        and all(abs(v-.001)<.00025 for v in measurement['gap_samples_m']))


try:
    gravity_scene=next(UsdPhysics.Scene(prim) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene))
    preload_steps=max(0,round(args.preload_seconds/DT))
    initial_loaded=None
    for step in range(preload_steps+max(1,round(args.seconds/DT))):
        if step==preload_steps:
            initial_loaded={name:measure(name,bench) for name,bench in benches.items()}
            gravity_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.,0.,-1.))
            gravity_scene.CreateGravityMagnitudeAttr(9.81)
        if step%12==0:  # Same 20 Hz target selection as the task; physics at 240 Hz.
            for name,bench in benches.items():
                art=bench['frame'];q=art.get_dof_positions()
                parameters=[cam_drive_parameters(q[0,j],1. if joint=='left_hinge' else -1.)
                            for j,joint in enumerate(bench['joint_names'])]
                target,gain,damping=(torch.tensor([[v[k] for v in parameters]],dtype=q.dtype,device=q.device) for k in range(3))
                art.set_dof_stiffnesses(gain.cpu(),indices.cpu())
                art.set_dof_dampings(damping.cpu(),indices.cpu())
                art.set_dof_position_targets(target,indices)
        loaded_time=(step-preload_steps)*DT
        pulse=1. if .4<=loaded_time<.6 else -1. if 1.<=loaded_time<1.2 else 0.
        torque=torch.tensor([[0.,0.,args.disturbance_torque*pulse]],dtype=torch.float32,device=sim.device)
        benches['Torsion']['bodies']['front'].apply_forces_and_torques_at_position(None,torque,None,indices,True)
        sim.step(render=False)
        if step>=preload_steps and (step-preload_steps)%24==0:
            for name,bench in benches.items():bench['trace'].append({'time_s':(step-preload_steps+1)*DT,**measure(name,bench)})
    report={'test':'independent_glass_retention_with_open_control_and_torsion','duration_s':args.seconds,'dt_s':DT,
            'frame_solver_iterations':{'position':64,'velocity':8},
            'detent':{'law':'task.cam_drive_parameters','open_stiffness_Nm_per_rad':.015,
                      'closed_stiffness_Nm_per_rad':.30,'open_damping_Nm_s_per_rad':.00035,
                      'closed_damping_Nm_s_per_rad':.003,'closed_gain_ramp_deg':[-10,0],
                      'effort_limit_Nm':.08,'target_selection_hz':20,'barrier_deg':-49,'open_target_deg':-100,'closed_target_deg':2},
            'fixture':'Only the frame body is fixed to world; glass bodies have no joints or pose corrections.',
            'preload_s':args.preload_seconds,'gravity_load_m_s2':9.81,'before_gravity_load':initial_loaded,
            'closed':measure('Closed',benches['Closed']),'open_negative_control':measure('Open',benches['Open']),
            'torsion':measure('Torsion',benches['Torsion']),
            'disturbance':{'front_plate_yaw_torque_Nm':args.disturbance_torque,
                           'positive_pulse_s':[.4,.6],'negative_pulse_s':[1.,1.2]},
            'traces':{name:bench['trace'] for name,bench in benches.items()},
            'limitations':['Nominal torque/friction model, not calibrated to manufacturer hardware.',
                           'Small specified yaw disturbance only; no robot transport, compliant seal, or liquid test.']}
    closed=report['closed'];opened=report['open_negative_control']
    report['closed_retention_passed']=all(retained(v) for v in [closed,*benches['Closed']['trace']])
    report['torsion_retention_passed']=all(retained(v) for v in [report['torsion'],*benches['Torsion']['trace']])
    report['open_negative_control_passed']=bool(opened['max_downward_drop_m']>.020
        and max(abs(v-goal) for v,goal in zip(opened['cam_degrees'],(-100.,100.)))<5.)
    report['passed']=report['closed_retention_passed'] and report['open_negative_control_passed'] and report['torsion_retention_passed']
    source_after=capture(HERE,extra_files=(Path(__file__).name,),include_demo=False)
    report.update(source_fingerprint=source_before,source_fingerprint_after=source_after,
                  source_unchanged=(source_before==source_after))
    report['passed']=bool(report['passed'] and report['source_unchanged'])
    args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('RETENTION_RESULT',json.dumps({key:report[key] for key in ('passed','closed_retention_passed','open_negative_control_passed','torsion_retention_passed','closed','open_negative_control','torsion')}),flush=True)
finally:
    app.close()
raise SystemExit(0 if report['passed'] else 1)
