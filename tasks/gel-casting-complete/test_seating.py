"""Physical stand seating with identical disturbances and a held-open control.

All bodies are free on a bench; only the control's lever has a holding servo.
No cassette joint, pose correction, friction adjustment, or artificial latch is
created. Initial poses and joint states are authored before simulation reset.
The specified loads are design checks, not manufacturer hardware load ratings.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--report',type=Path,default=Path(__file__).with_name('seating_check.json'))
args=parser.parse_args()
HERE=Path(__file__).resolve().parent
from source_fingerprint import capture
before=capture(HERE,extra_files=(Path(__file__).name,),include_demo=False)
from isaaclab.app import AppLauncher
launcher=AppLauncher(headless=True)
app=launcher.app

import numpy as np
import torch
from pxr import Gf,Usd,UsdGeom,UsdPhysics,PhysxSchema
import omni.usd
import omni.physics.tensors as tensors
import isaaclab.sim as sim_utils
import geometry as g
from task import cam_drive_parameters,geometry_metrics

DT=1/240
DURATION=4.5
origins={'Closed':np.array([0.,-.15,.20]),'Open':np.array([0.,.15,.20])}


def reference(stage,path,filename,pos):
    prim=stage.DefinePrim(path,'Xform')
    prim.GetReferences().AddReference(str(HERE/'task_assets'/filename))
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*pos))
    return prim


def rotation(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def lift_check():
    stage=Usd.Stage.CreateInMemory()
    reference(stage,'/Stand','casting_stand.usda',(0.,0.,0.))
    stage.GetPrimAtPath('/Stand/pressure_clip').GetAttribute('xformOp:rotateY').Set(-75.)
    reference(stage,'/Frame','casting_frame.usda',(0.,0.,0.))
    reference(stage,'/Rear','rear_plate.usda',g.REAR_POS)
    reference(stage,'/Front','front_plate.usda',g.FRONT_POS)
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_],useExtentsHint=False,ignoreVisibility=True)
    def bounds(root):
        result=[]
        for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if not p.HasAPI(UsdPhysics.CollisionAPI):continue
            if not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
            box=cache.ComputeWorldBound(p).ComputeAlignedRange()
            result.append((str(p.GetPath()),np.array(box.GetMin()),np.array(box.GetMax())))
        return result
    fixed=bounds('/Stand');moving=sum((bounds(p) for p in ('/Frame','/Rear','/Front')),[])
    overlaps=[]
    for h in np.linspace(0.,.140,281):
        shift=np.array([0.,0.,h])
        for name,lo,hi in moving:
            for fixed_name,flo,fhi in fixed:
                depth=np.minimum(hi+shift,fhi)-np.maximum(lo+shift,flo)
                if np.all(depth>1e-6):
                    overlaps.append({'height_m':float(h),'cassette':name,'stand':fixed_name,'overlap_m':depth.tolist()})
    return {'passed':not overlaps,'method':'Conservative world AABB sweep of actual collision geometry; no physical extraction claim.',
            'open_clip_degrees':-75.,'lift_m':.140,'step_m':.0005,'overlap_count':len(overlaps),'overlaps':overlaps[:12]}


extraction=lift_check()
sim=sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT,use_fabric=False))
stage=omni.usd.get_context().get_stage()
floor=UsdGeom.Cube.Define(stage,'/World/Bench')
floor.CreateSizeAttr(1.)
floor.AddTranslateOp().Set(Gf.Vec3d(0.,0.,.19))
floor.AddScaleOp().Set(Gf.Vec3f(.5,.8,.02))
UsdPhysics.CollisionAPI.Apply(floor.GetPrim()).CreateCollisionEnabledAttr(True)
for name,origin in origins.items():
    root='/World/'+name
    for suffix,file,pos in [('Frame','casting_frame.usda',origin),('Stand','casting_stand.usda',origin),
                            ('Rear','rear_plate.usda',origin+g.REAR_POS),('Front','front_plate.usda',origin+g.FRONT_POS)]:
        prim=reference(stage,root+suffix,file,pos)
        if suffix in ('Frame','Stand'):
            prim.GetAttribute('physxArticulation:solverPositionIterationCount').Set(64)
            prim.GetAttribute('physxArticulation:solverVelocityIterationCount').Set(8)
    for joint in ('left_hinge','right_hinge'):
        prim=stage.GetPrimAtPath(root+'Frame/'+joint)
        state=PhysxSchema.JointStateAPI.Apply(prim,'angular')
        state.CreatePositionAttr(0.);state.CreateVelocityAttr(0.)
        d=UsdPhysics.DriveAPI.Apply(prim,'angular')
        d.CreateTargetPositionAttr(2. if joint=='left_hinge' else -2.)
        d.CreateStiffnessAttr(.30*math.pi/180);d.CreateDampingAttr(.003*math.pi/180);d.CreateMaxForceAttr(.08)
    angle=-75. if name=='Open' else 0.
    stage.GetPrimAtPath(root+'Stand/pressure_clip').GetAttribute('xformOp:rotateY').Set(angle)
    prim=stage.GetPrimAtPath(root+'Stand/clip_hinge')
    state=PhysxSchema.JointStateAPI.Apply(prim,'angular')
    state.CreatePositionAttr(angle);state.CreateVelocityAttr(0.)
    if name=='Open':
        # Explicit lever-holding actuator for the negative control, not a weld.
        d=UsdPhysics.DriveAPI.Apply(prim,'angular')
        d.CreateTargetPositionAttr(-75.);d.CreateStiffnessAttr(1.*math.pi/180)
        d.CreateDampingAttr(.02*math.pi/180);d.CreateMaxForceAttr(.10)

sim.reset()
view=tensors.create_simulation_view('torch');view.set_subspace_roots('/')
indices=torch.tensor([0],dtype=torch.int32,device=sim.device)
benches={}
for name in origins:
    root='/World/'+name
    frame=view.create_articulation_view(root+'Frame*')
    stand=view.create_articulation_view(root+'Stand*')
    benches[name]={'frame':frame,'stand':stand,'joints':list(frame.shared_metatype.dof_names),
        'body_index':list(frame.shared_metatype.link_names).index('body'),
        'rear':view.create_rigid_body_view(root+'Rear'),'front':view.create_rigid_body_view(root+'Front'),'trace':[]}


def measure(b):
    f=b['frame'].get_root_transforms()[0].cpu().numpy().astype(float)
    s=b['stand'].get_root_transforms()[0].cpu().numpy().astype(float)
    rf,rs=rotation(f[3:]),rotation(s[3:])
    rel=rs.T@(f[:3]-s[:3]);r=rs.T@rf
    gaps=[float((rel+r@np.array([-.012,y,z]))[0]+.0123) for y in (-.038,.038) for z in (.019,.081)]
    rear=b['rear'].get_transforms()[0].cpu().numpy().astype(float)
    front=b['front'].get_transforms()[0].cpu().numpy().astype(float)
    plate_errors={p:(rf.T@(pose[:3]-f[:3])-np.array(g.REAR_POS if p=='rear' else g.FRONT_POS)).tolist()
                  for p,pose in [('rear',rear),('front',front)]}
    clip=float(b['stand'].get_dof_positions()[0,0].item())
    cam_values=b['frame'].get_dof_positions()[0].cpu().tolist()
    objects={name:SimpleNamespace(pos=pose[:3],quat_xyzw=pose[3:],joints={})
             for name,pose in [('frame',f),('stand',s),('rear',rear),('front',front)]}
    objects['frame'].joints=dict(zip(b['joints'],cam_values))
    objects['stand'].joints={'clip_hinge':clip}
    geometry=geometry_metrics(objects)
    angle=math.degrees(math.acos(float(np.clip((np.trace(r)-1)/2,-1,1))))
    return {'frame_relative_m':rel.tolist(),'frame_relative_rotation_deg':angle,'rear_pad_gaps_m':gaps,
            'cam_degrees':torch.rad2deg(b['frame'].get_dof_positions()[0]).cpu().tolist(),
            'clip_degrees':math.degrees(clip),'plate_relative_error_m':plate_errors,
            'geometry':geometry}


try:
    reference_state=None
    for step in range(round(DURATION/DT)):
        t=step*DT
        if step>0 and step%12==0:
            for b in benches.values():
                art=b['frame'];q=art.get_dof_positions()
                params=[cam_drive_parameters(q[0,j],1 if n=='left_hinge' else -1) for j,n in enumerate(b['joints'])]
                target,gain,damping=(torch.tensor([[v[k] for v in params]],dtype=q.dtype,device=q.device) for k in range(3))
                art.set_dof_stiffnesses(gain.cpu(),indices.cpu());art.set_dof_dampings(damping.cpu(),indices.cpu())
                art.set_dof_position_targets(target,indices)
        horizontal=-.3 if 1.<=t<1.5 else 0.
        upward=2.2 if 3.<=t<3.35 else 0.
        for b in benches.values():
            if step==0:continue  # Physics pose buffers become valid after the first step.
            art=b['frame'];force=torch.zeros((1,art.max_links,3),dtype=torch.float32,device=sim.device)
            points=torch.zeros_like(force)
            pose=art.get_root_transforms()[0].cpu().numpy().astype(float)
            point_local=np.array([.001337,0.,.050324]) if upward else np.array([0.,0.,.081])
            point=pose[:3]+rotation(pose[3:])@point_local
            force[0,b['body_index']]=torch.tensor([horizontal,0.,upward],device=sim.device)
            points[0,b['body_index']]=torch.tensor(point,dtype=torch.float32,device=sim.device)
            art.apply_forces_and_torques_at_position(force,None,points,indices,True)
        sim.step(render=False)
        if step==round(1./DT)-1:reference_state={name:measure(b) for name,b in benches.items()}
        for name,b in benches.items():b['trace'].append({'time_s':(step+1)*DT,**measure(b)})
    summaries={}
    for name,b in benches.items():
        base=np.array(reference_state[name]['frame_relative_m'])
        loaded=[v for v in b['trace'] if v['time_s']>=1.]
        upward=[v for v in loaded if 3.<=v['time_s']<=3.6]
        rearward=[v for v in loaded if 1.<=v['time_s']<=2.]
        summaries[name]={'max_upward_displacement_m':max(v['frame_relative_m'][2]-base[2] for v in upward),
                         'max_rearward_translation_m':max(base[0]-v['frame_relative_m'][0] for v in rearward),
                         'max_rearward_rotation_deg':max(v['frame_relative_rotation_deg'] for v in rearward),
                         'rear_load_pad_gap_range_m':[min(min(v['rear_pad_gaps_m']) for v in rearward),max(max(v['rear_pad_gaps_m']) for v in rearward)],
                         'clip_range_degrees':[min(v['clip_degrees'] for v in loaded),max(v['clip_degrees'] for v in loaded)],
                         'final':measure(b)}
    closed,opened=summaries['Closed'],summaries['Open']
    checks={'closed_upward_restraint':closed['max_upward_displacement_m']<.0005,
            'open_upward_negative_control':opened['max_upward_displacement_m']>.005,
            'open_lever_actually_held':max(abs(v+75.) for v in opened['clip_range_degrees'])<2.,
            'closed_rear_registration':closed['max_rearward_translation_m']<.0007 and closed['max_rearward_rotation_deg']<1.,
            'open_rear_registration':opened['max_rearward_translation_m']<.0007 and opened['max_rearward_rotation_deg']<1.,
            'closed_glass_alignment_after_load':closed['final']['geometry']['aligned'],
            'closed_gasket_and_shoe_seated_after_load':closed['final']['geometry']['docked'] and closed['final']['geometry']['clip_closed'],
            'straight_extraction_geometry':extraction['passed']}
    checks={name:bool(value) for name,value in checks.items()}
    after=capture(HERE,extra_files=(Path(__file__).name,),include_demo=False)
    report={'test':'free_cassette_seating_and_spring_restraint','dt_s':DT,'duration_s':DURATION,
            'solver_iterations':{'position':64,'velocity':8},'source_fingerprint':before,'source_fingerprint_after':after,'source_unchanged':before==after,
            'initialization':'JointStateAPI and USD transforms before reset only; no object attachments or pose writes after reset.',
            'disturbances':{'identical_in_both_cases':True,'rearward_force_N':.3,'rearward_pulse_s':[1.,1.5],
                           'rearward_application_local_m':[0.,0.,.081],'upward_force_N':2.2,'upward_pulse_s':[3.,3.35],
                           'upward_application_local_m':[.001337,0.,.050324],
                           'application':'Frame body, world force direction; point follows frame and approximates assembled cassette COM for upward load.'},
            'closed_spring':{'target_deg':8.,'stiffness_Nm_per_rad':.045,'maximum_torque_Nm':.05},
            'open_control':'Lever held at -75 degrees by an explicit position servo; all cassette and stand bodies remain free.',
            'reference_at_1s':reference_state,'cases':summaries,'checks':checks,'geometric_extraction':extraction,
            'traces':{name:b['trace'] for name,b in benches.items()},
            'limitations':['Nominal fit, spring, friction and loads; not manufacturer-calibrated or a hardware load rating.',
                           'Rigid gasket; no seal or liquid validation. No robot motion in this bench.',
                           'Rear supports tested in both lever states; no guide-removal ablation.']}
    report['passed']=all(checks.values()) and report['source_unchanged']
    args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('SEATING_RESULT',json.dumps({k:report[k] for k in ('passed','source_unchanged','checks','cases','geometric_extraction')}),flush=True)
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
raise SystemExit(0 if report['passed'] else 1)
