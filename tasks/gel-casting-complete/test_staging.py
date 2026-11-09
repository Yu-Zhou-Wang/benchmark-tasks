"""Five-second idle test of the open-cam staging jig and a geometric lift check.

The current jig is compared with the original low-guide fixture. Frame and
glass remain dynamic on an ordinary bench. All initial transforms and cam
angles are authored before reset; there are no pose writes or object joints
after reset. The separate lift check examines collision bounding boxes and
does not move simulated objects or claim a physical transport demonstration.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--report',type=Path,default=Path(__file__).with_name('staging_check.json'))
args=parser.parse_args()
HERE=Path(__file__).resolve().parent
from source_fingerprint import capture
source_before=capture(HERE,extra_files=(Path(__file__).name,),include_demo=False)
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
from task import cam_drive_parameters

DT=1/240
DURATION=5.


def reference(stage,path,filename,pos):
    prim=stage.DefinePrim(path,'Xform')
    prim.GetReferences().AddReference(str(HERE/'task_assets'/filename))
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*pos))
    return prim


def geometric_lift_check():
    """Conservative AABB sweep of the nominal CLOSED cassette, not simulation."""
    check=Usd.Stage.CreateInMemory()
    reference(check,'/Fixture','staging_cradle.usda',(0.,0.,0.))
    reference(check,'/Frame','casting_frame.usda',(0.,0.,0.))
    reference(check,'/Rear','rear_plate.usda',g.REAR_POS)
    reference(check,'/Front','front_plate.usda',g.FRONT_POS)
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_],useExtentsHint=False)

    def bounds(root):
        result=[]
        for prim in Usd.PrimRange(check.GetPrimAtPath(root)):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
            if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():continue
            bbox=cache.ComputeWorldBound(prim).ComputeAlignedRange()
            result.append((str(prim.GetPath()),np.array(bbox.GetMin()),np.array(bbox.GetMax())))
        return result

    fixed=bounds('/Fixture')
    moving=[b for root in ('/Frame','/Rear','/Front') for b in bounds(root)]
    overlaps=[]
    for height in np.linspace(0.,.045,91):
        shift=np.array([0.,0.,height])
        for moving_name,lo,hi in moving:
            for fixed_name,flo,fhi in fixed:
                depth=np.minimum(hi+shift,fhi)-np.maximum(lo+shift,flo)
                if np.all(depth>1e-6):
                    overlaps.append({'lift_m':float(height),'moving':moving_name,'fixture':fixed_name,
                                     'aabb_overlap_m':depth.tolist()})
    lowest_moving=min(float(lo[2]) for _,lo,_ in moving)
    highest_fixture=max(float(hi[2]) for _,_,hi in fixed)
    full_clearance=lowest_moving+.045-highest_fixture
    return {'passed':not overlaps and full_clearance>0.,'method':'Conservative collision-AABB sweep; no physics transport.',
            'nominal_cam_degrees':[0.,0.],'lift_m':.045,'height_sample_spacing_m':.0005,
            'lowest_moving_collision_z_m':lowest_moving,'highest_fixture_collision_z_m':highest_fixture,
            'full_vertical_clearance_at_45mm_m':full_clearance,'overlap_count':len(overlaps),
            'overlaps':overlaps[:20]}


lift_check=geometric_lift_check()
sim=sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT,use_fabric=False))
stage=omni.usd.get_context().get_stage()
origins={'legacy_low_guides':np.array([0.,-.12,.20]),'current_jig':np.array([0.,.12,.20])}
floor=UsdGeom.Cube.Define(stage,'/World/Bench')
floor.CreateSizeAttr(1.)
floor.AddTranslateOp().Set(Gf.Vec3d(0.,0.,.199))
floor.AddScaleOp().Set(Gf.Vec3f(.5,.6,.02))
UsdPhysics.CollisionAPI.Apply(floor.GetPrim()).CreateCollisionEnabledAttr(True)

for name,origin in origins.items():
    root='/World/'+name
    frame_prim=reference(stage,root+'Frame','casting_frame.usda',origin)
    frame_prim.GetAttribute('physxArticulation:solverPositionIterationCount').Set(64)
    frame_prim.GetAttribute('physxArticulation:solverVelocityIterationCount').Set(8)
    for joint,part,angle in (('left_hinge','left_cam',-100.),('right_hinge','right_cam',100.)):
        # Author initial transforms before reset; no joint-position setter later.
        stage.GetPrimAtPath(root+'Frame/'+part).GetAttribute('xformOp:rotateZ').Set(angle)
        joint_prim=stage.GetPrimAtPath(root+'Frame/'+joint)
        state=PhysxSchema.JointStateAPI.Apply(joint_prim,'angular')
        state.CreatePositionAttr(angle)
        state.CreateVelocityAttr(0.)
        drive=UsdPhysics.DriveAPI.Apply(joint_prim,'angular')
        drive.CreateStiffnessAttr(.015*math.pi/180)
        drive.CreateDampingAttr(.00035*math.pi/180)
        drive.CreateMaxForceAttr(.08)
        drive.CreateTargetPositionAttr(angle)
    reference(stage,root+'Rear','rear_plate.usda',origin+g.REAR_POS)
    reference(stage,root+'Front','front_plate.usda',origin+g.FRONT_POS)
    reference(stage,root+'Cradle','staging_cradle.usda',origin)
    if name=='legacy_low_guides':
        # Reconstruct the previous fixture explicitly, so this control remains
        # reproducible after task_assets has been regenerated with the new jig.
        for i,y in enumerate((-.020,.020)):
            prim=stage.GetPrimAtPath(root+f'Cradle/front_guide_{i}')
            prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(.0082,y,.019))
            prim.GetAttribute('xformOp:scale').Set(Gf.Vec3f(.005,.006,.020))
        for i in range(2):
            for j in range(4):stage.GetPrimAtPath(root+f'Cradle/foot_nest_{i}_{j}').SetActive(False)

sim.reset()
view=tensors.create_simulation_view('torch')
view.set_subspace_roots('/')
indices=torch.tensor([0],dtype=torch.int32,device=sim.device)
cases={}
for name in origins:
    root='/World/'+name
    art=view.create_articulation_view(root+'Frame*')
    names=list(art.shared_metatype.dof_names)
    if art.count!=1 or set(names)!={'left_hinge','right_hinge'}:
        raise RuntimeError(f'Unexpected frame articulation: {name}/{names}')
    cases[name]={'frame':art,'joint_names':names,
                 'bodies':{part:view.create_rigid_body_view(root+part.title()) for part in ('rear','front')},'trace':[]}


def rotation(q):
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def measure(name,case):
    poses={part:body.get_transforms()[0].cpu().numpy().astype(float) for part,body in case['bodies'].items()}
    rear,front=poses['rear'],poses['front']
    normal=rotation(rear[3:])[:,0]
    front_rotation=rotation(front[3:])
    gaps=[float(normal@(front[:3]+front_rotation@np.array([-.0015,y,z])-rear[:3])-.0015)
          for y in (-.044,.044) for z in (-.035,.035)]
    bottoms=[]
    for part,pose in poses.items():
        matrix=rotation(pose[3:])
        height=(g.REAR_SIZE if part=='rear' else g.FRONT_SIZE)[2]
        bottoms.extend(float((pose[:3]+matrix@np.array([0.,y,-height/2])-origins[name])[2]-.012)
                       for y in (-.044,.044))
    angles=torch.rad2deg(case['frame'].get_dof_positions()[0]).cpu().tolist()
    return {'gap_samples_m':gaps,'glass_bottom_error_m':bottoms,
            'cams_deg':[angles[case['joint_names'].index(j)] for j in ('left_hinge','right_hinge')],
            'plate_pose':{part:pose.tolist() for part,pose in poses.items()}}


try:
    for step in range(round(DURATION/DT)):
        # Tensor joint-state buffers populate after the first physics step.
        # That step already has the correct authored open targets/gains; do not
        # mistake the initial unpopulated zero buffer for a closed mechanism.
        if step>0 and step%12==0:
            for case in cases.values():
                art=case['frame'];q=art.get_dof_positions()
                parameters=[cam_drive_parameters(q[0,j],1. if joint=='left_hinge' else -1.)
                            for j,joint in enumerate(case['joint_names'])]
                target,gain,damping=(torch.tensor([[v[k] for v in parameters]],dtype=q.dtype,device=q.device) for k in range(3))
                art.set_dof_stiffnesses(gain.cpu(),indices.cpu())
                art.set_dof_dampings(damping.cpu(),indices.cpu())
                art.set_dof_position_targets(target,indices)
        sim.step(render=False)
        if step%24==0 or step==round(DURATION/DT)-1:
            for name,case in cases.items():case['trace'].append({'time_s':(step+1)*DT,**measure(name,case)})
    report={'test':'open_cam_staging_idle','duration_s':DURATION,'physics_dt_s':DT,'measurement_interval_s':.1,
            'frame_solver_iterations':{'position':64,'velocity':8},
            'fixture':'Cradle fixed; frame and both plates free. Ordinary bench supports the feet.',
            'initialization':'All poses and initial cam angles authored before reset. Only drive targets change after reset.',
            'geometric_lift_check':lift_check,'cases':{},
            'limitations':['Idle test only; no robot cam closure, physical lift, or transport.',
                           'The lift sweep checks nominal collision bounding boxes; it is not a contact dynamics test.']}
    for name,case in cases.items():
        trace=case['trace']
        valid_gaps=all(abs(v-.001)<.00025 for sample in trace for v in sample['gap_samples_m'])
        valid_bottoms=all(abs(v)<.0005 for sample in trace for v in sample['glass_bottom_error_m'])
        open_cams=all(abs(v-goal)<5. for sample in trace for v,goal in zip(sample['cams_deg'],(-100.,100.)))
        report['cases'][name]={'passed':valid_gaps and valid_bottoms and open_cams,
            'all_gaps_valid':valid_gaps,'all_bottoms_valid':valid_bottoms,'cams_remain_open':open_cams,
            'gap_range_m':[min(v for s in trace for v in s['gap_samples_m']),max(v for s in trace for v in s['gap_samples_m'])],
            'final':trace[-1],'trace':trace}
    # The retired fixture is an observational comparison, not the task's
    # acceptance subject. Its result is preserved even when it loses alignment.
    report['acceptance_subject']='current_jig; legacy_low_guides is an observational comparison'
    report['passed']=lift_check['passed'] and report['cases']['current_jig']['passed']
    source_after=capture(HERE,extra_files=(Path(__file__).name,),include_demo=False)
    report.update(source_fingerprint=source_before,source_fingerprint_after=source_after,
                  source_unchanged=(source_before==source_after))
    report['passed']=bool(report['passed'] and report['source_unchanged'])
    args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('STAGING_RESULT',json.dumps({'passed':report['passed'],'geometric_lift_check':lift_check,
          'cases':{n:{k:v[k] for k in ('passed','all_gaps_valid','all_bottoms_valid','cams_remain_open','gap_range_m','final')}
                   for n,v in report['cases'].items()}}),flush=True)
finally:
    app.close()
raise SystemExit(0 if report['passed'] else 1)
