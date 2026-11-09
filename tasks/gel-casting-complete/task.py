"""Prepare the mini PAGE cassette for casting. Independent pre-aligned glass plates
rest in an open green casting frame on a low alignment cradle. Close both side
cams, lift the cassette out, and seat it on the separate clear stand's gray pad.
Press and hold the stand's spring lever open with one hand while the other docks
the cassette. Release the lever and withdraw both hands. No liquid is simulated.
"""
from pathlib import Path
from collections import deque
import sys,math
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from scenekit import UsdBody,on_reset,on_step,arm_joints,set_joint_targets,set_joint_gains
from yam_arena.task_file import TABLE_TOP
import geometry as g

SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=3.0
TOP=TABLE_TOP[SCENE]
FRAME_START=np.array([.30,0.,TOP-.009])
STAND_START=np.array([.16,0.,TOP])
ROT=np.diag([-1.,-1.,1.])
QUAT=(0.,0.,1.,0.)
STAND_LEVER_TARGET_DEG=8.0
REAR_SUPPORT_FRONT_X=-.0123
INSTRUCTION=('Close both green pressure cams to secure the aligned glass plates. Lift the cassette from its '
             'alignment cradle, hold the clear stand lever open, and place the cassette on the gray sealing pad. '
             'Release the lever onto the tall plate and withdraw both grippers, leaving the assembly upright and stable.')
def placed(local,origin=FRAME_START):return tuple(origin+ROT@np.array(local))
OBJECTS=[
    UsdBody('frame',usd='task_assets/casting_frame.usda',pos=tuple(FRAME_START),rot_xyzw=QUAT,
        size=(.04,.15,.09),mass=.086,articulation=True,solver_iters=64,solver_velocity_iters=8,
        joint_pos=(('left_hinge',math.radians(-100)),('right_hinge',math.radians(100))),
        joint_stiffness=.015,joint_damping=.00035,joint_friction=.0005,joint_effort_limit=.08,
        joint_targets=(('left_hinge',math.radians(-100)),('right_hinge',math.radians(100)))),
    UsdBody('rear',usd='task_assets/rear_plate.usda',pos=placed(g.REAR_POS),rot_xyzw=QUAT,
        size=g.REAR_SIZE,mass=g.REAR_MASS,friction=(.9,.8),solver_iters=64,solver_velocity_iters=8),
    UsdBody('front',usd='task_assets/front_plate.usda',pos=placed(g.FRONT_POS),rot_xyzw=QUAT,
        size=g.FRONT_SIZE,mass=g.FRONT_MASS,friction=(.9,.8),solver_iters=64,solver_velocity_iters=8),
    UsdBody('stand',usd='task_assets/casting_stand.usda',pos=tuple(STAND_START),rot_xyzw=QUAT,
        size=g.STAND_SIZE,mass=g.STAND_MASS,articulation=True,solver_iters=64,solver_velocity_iters=8,
        joint_pos=(('clip_hinge',0.),),joint_targets=(('clip_hinge',math.radians(STAND_LEVER_TARGET_DEG)),)),
    UsdBody('cradle',usd='task_assets/staging_cradle.usda',pos=tuple(FRAME_START),rot_xyzw=QUAT,
        size=(.0351,.1301,.050),solver_iters=64,solver_velocity_iters=8),
]
STATE={}
_POSE_HISTORY=deque()
_POSE_TIME=0.
STABILITY_QUALIFY_S=1.
STABILITY_HISTORY_S=STABILITY_QUALIFY_S+SUCCESS_HOLD_S
def rotation(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def point(obj,local):return obj.pos+rotation(obj.quat_xyzw)@np.asarray(local)
def angle_between(a,b):
    return 2*math.acos(float(np.clip(abs(np.dot(a.quat_xyzw,b.quat_xyzw)),0,1)))
def geometry_metrics(objects):
    frame,stand=objects['frame'],objects['stand']
    rot=rotation(frame.quat_xyzw)
    local={n:rot.T@(objects[n].pos-frame.pos) for n in ('rear','front')}
    errors={n:float(np.linalg.norm(local[n]-p)) for n,p in [('rear',g.REAR_POS),('front',g.FRONT_POS)]}
    aligned=all(e<.002 for e in errors.values()) and all(angle_between(frame,objects[n])<math.radians(4) for n in errors)
    rear,front=objects['rear'],objects['front']
    normal=rotation(rear.quat_xyzw)[:,0]
    gaps=[float(np.dot(normal,point(front,(-.0015,y,z))-rear.pos)-.0015)
          for y in (-.044,.044) for z in (-.035,.035)]
    bottom_errors=[float((rot.T@(point(objects[n],(0,y,-size[2]/2))-frame.pos))[2]-.012)
                   for n,size in [('rear',g.REAR_SIZE),('front',g.FRONT_SIZE)] for y in (-.044,.044)]
    aligned=aligned and all(abs(v-.001)<.00025 for v in gaps) and all(abs(v)<.0005 for v in bottom_errors)
    cams=[float(frame.joints.get('left_hinge',-math.pi)),float(frame.joints.get('right_hinge',math.pi))]
    closed=[math.radians(-1)<cams[0]<math.radians(4),math.radians(-4)<cams[1]<math.radians(1)]
    offset=rotation(stand.quat_xyzw).T@(frame.pos-stand.pos)
    # Four visible stand pads register the frame's load-bearing rear ribs.
    # Clearance is measured in the stand frame, not inferred from stability.
    stand_rotation=rotation(stand.quat_xyzw)
    rear_clearances=[float((stand_rotation.T@(point(frame,(-.012,y,z))-stand.pos))[0]-REAR_SUPPORT_FRONT_X)
                     for y in (-.038,.038) for z in (.019,.081)]
    registered=bool(all(-.00015<=gap<=.001 for gap in rear_clearances) and abs(offset[1])<=.0007)
    gasket_gaps=[float((stand_rotation.T@(point(objects[name],(0,y,-size[2]/2))-stand.pos))[2]-.012)
                 for name,size in (('rear',g.REAR_SIZE),('front',g.FRONT_SIZE)) for y in (-.044,.044)]
    docked=bool(aligned and all(closed) and np.linalg.norm(offset[:2])<.0025 and abs(offset[2])<.0006
                and angle_between(frame,stand)<math.radians(3) and registered
                and all(abs(gap)<=.0005 for gap in gasket_gaps))
    lever=float(stand.joints.get('clip_hinge',-math.pi))
    c,s=math.cos(lever),math.sin(lever)
    lever_rotation=np.array([[c,0,s],[0,1,0],[-s,0,c]])
    shoe_points=[point(stand,np.asarray(g.CLIP_PIVOT)+lever_rotation@np.array([.025,y,-.011])) for y in (-.009,.009)]
    shoe_local=[rotation(rear.quat_xyzw).T@(p-rear.pos) for p in shoe_points]
    shoe_gaps=[float(p[2]-g.REAR_SIZE[2]/2) for p in shoe_local]
    shoe_seated=bool(all(abs(gap)<=.0005 for gap in shoe_gaps)
                     and all(abs(p[0])<=g.REAR_SIZE[0]/2 for p in shoe_local))
    clip_closed=abs(lever)<math.radians(6) and shoe_seated
    upright=rotation(stand.quat_xyzw)[2,2]>math.cos(math.radians(5))
    return dict(aligned=bool(aligned),plate_error_m=errors,glass_gap_samples_m=gaps,bottom_error_m=bottom_errors,cam_degrees=np.degrees(cams).tolist(),cams_closed=closed,
                rear_support_clearances_m=rear_clearances,registered=registered,
                glass_gasket_gaps_m=gasket_gaps,clip_shoe_gaps_m=shoe_gaps,clip_shoe_seated=shoe_seated,
                docked=docked,dock_error_m=offset.tolist(),clip_degrees=math.degrees(lever),clip_closed=bool(clip_closed),
                upright=bool(upright))

@on_reset
def reset_monitor(environment=None):
    global _POSE_TIME
    _POSE_TIME=0.
    _POSE_HISTORY.clear()
    STATE.clear();STATE.update(steps=0,seen_open=[False,False],transported=False,violations=[],hands_clear=False,
                               aligned=False,cams_closed=[False,False],docked=False,clip_closed=False,stable=False,upright=True)

def pose_stability(objects,step_dt,eligible):
    """Measure released-assembly settling, without trusting residual velocities.

    A candidate qualifies after one full second. Its history keeps expanding
    through the framework's additional three-second hold, capped at four
    seconds. Thus slow cumulative drift cannot pass independent short windows.
    Only measurements are stored; this function never changes simulated state.
    """
    global _POSE_TIME
    result=dict(stable=False,stability_duration_s=0.,stability_samples=0,
                stability_position_diameter_m={},stability_rotation_diameter_deg={},
                stability_joint_ranges_deg={},stability_reason='awaiting_released_geometry')
    if not math.isfinite(step_dt) or step_dt<=0.:
        _POSE_HISTORY.clear();result['stability_reason']='invalid_sample_interval';return result
    _POSE_TIME+=step_dt
    if not eligible:
        _POSE_HISTORY.clear();return result
    try:
        poses=[]
        for name in ('frame','rear','front','stand'):
            body=objects[name]
            position=np.asarray(body.pos,dtype=float)
            quaternion=np.asarray(body.quat_xyzw,dtype=float)
            linear=np.asarray(body.lin_vel,dtype=float)
            angular=np.asarray(body.ang_vel,dtype=float)
            if (position.shape!=(3,) or quaternion.shape!=(4,) or linear.shape!=(3,) or angular.shape!=(3,)
                or not all(np.all(np.isfinite(v)) for v in (position,quaternion,linear,angular))
                or np.linalg.norm(quaternion)<1e-12):raise ValueError('invalid pose sample')
            if np.linalg.norm(linear)>=.05 or np.linalg.norm(angular)>=2.:
                _POSE_HISTORY.clear();result['stability_reason']='coarse_motion_guard';return result
            poses.append((position.copy(),quaternion/np.linalg.norm(quaternion)))
        joints=np.array([objects['frame'].joints['left_hinge'],objects['frame'].joints['right_hinge'],
                         objects['stand'].joints['clip_hinge']],dtype=float)
        if not np.all(np.isfinite(joints)):raise ValueError('invalid joint sample')
    except (KeyError,TypeError,ValueError,AttributeError):
        _POSE_HISTORY.clear();result['stability_reason']='missing_or_invalid_sample';return result
    sample=(_POSE_TIME,poses,joints.copy())
    _POSE_HISTORY.append(sample)
    while _POSE_HISTORY[0][0]<_POSE_TIME-STABILITY_HISTORY_S-1e-9:_POSE_HISTORY.popleft()
    for index,name in enumerate(('frame','rear','front','stand')):
        positions=np.stack([entry[1][index][0] for entry in _POSE_HISTORY])
        quaternions=np.stack([entry[1][index][1] for entry in _POSE_HISTORY])
        distances=np.linalg.norm(positions[:,None,:]-positions[None,:,:],axis=-1)
        quaternion_dot=np.clip(np.abs(quaternions@quaternions.T),0.,1.)
        result['stability_position_diameter_m'][name]=float(np.max(distances))
        result['stability_rotation_diameter_deg'][name]=float(np.degrees(2*np.arccos(np.min(quaternion_dot))))
    joint_ranges=np.degrees(np.ptp(np.stack([entry[2] for entry in _POSE_HISTORY]),axis=0))
    result['stability_joint_ranges_deg']=dict(zip(('left_cam','right_cam','top_clip'),map(float,joint_ranges)))
    excursion=(max(result['stability_position_diameter_m'].values())>.0002
               or max(result['stability_rotation_diameter_deg'].values())>.2
               or float(np.max(joint_ranges))>.5)
    if excursion:
        _POSE_HISTORY.clear();_POSE_HISTORY.append(sample)
        result['stability_samples']=1;result['stability_reason']='pose_or_joint_excursion';return result
    duration=_POSE_TIME-_POSE_HISTORY[0][0]
    result.update(stable=bool(duration>=STABILITY_QUALIFY_S-1e-9),stability_duration_s=float(duration),
                  stability_samples=len(_POSE_HISTORY),stability_reason='qualified' if duration>=STABILITY_QUALIFY_S-1e-9 else 'collecting')
    return result

def cam_drive_parameters(joint_angle_rad,sign):
    """Nominal cam spring: weak free travel, stronger preload near pad contact.

    Returns (target radians, stiffness N m/rad, damping N m s/rad). Only the
    mechanical angle enters this law; no robot, plate, score, or pose gating.
    """
    oriented_degrees=math.degrees(sign*float(joint_angle_rad))
    open_well=oriented_degrees < -49.
    blend=0. if open_well else min(1.,max(0.,(oriented_degrees+10.)/10.))
    return (sign*math.radians(-100. if open_well else 2.),
            .015+(.30-.015)*blend,.00035+(.003-.00035)*blend)

@on_step
def update(environment):
    from yam_arena.scene import object_states
    from yam_arena import ik,station
    from yam_arena.constants import ARM_JOINTS
    base=environment.unwrapped
    frame=base.scene['frame']
    # The implicit solver handles the cam's small inertia. Free travel uses a
    # weak detent; the closed well stiffens over its final ten degrees.
    # This never inspects robot/score/plate position and never sets an object pose.
    q=frame.data.joint_pos.torch[0]
    targets={};stiffness={};damping={}
    for joint,sign in [('left_hinge',1.),('right_hinge',-1.)]:
        j=frame.joint_names.index(joint)
        targets[joint],stiffness[joint],damping[joint]=cam_drive_parameters(q[j],sign)
    set_joint_gains(environment,'frame',stiffness,damping)
    set_joint_targets(environment,'frame',targets)
    objects=object_states(environment,['frame','rear','front','stand'])
    metrics=geometry_metrics(objects)
    STATE['steps']+=1
    STATE['seen_open']=[old or abs(a)>60 for old,a in zip(STATE['seen_open'],metrics['cam_degrees'])]
    if objects['frame'].pos[2]>FRAME_START[2]+.025:STATE['transported']=True
    # Falling off the work surface is irreversible; orientation/fit must recover
    # for any score. Small contact-settling errors alone do not trigger failure.
    if any(objects[n].pos[2]<TOP-.025 for n in ('rear','front','frame','stand')):
        if 'part_fell_off_table' not in STATE['violations']:STATE['violations'].append('part_fell_off_table')
    target=point(objects['stand'],(0,0,.060))
    tcps={}
    for side in ('left','right'):
        p,_=ik.tcp_pose(arm_joints(environment,side,ARM_JOINTS),station.BASES[side])
        tcps[side]=p.tolist()
    metrics['hands_clear']=all(np.linalg.norm(np.asarray(p)-target)>.145 for p in tcps.values())
    eligible=all(metrics[name] for name in ('aligned','docked','clip_closed','upright','hands_clear'))
    metrics.update(pose_stability(objects,float(base.step_dt),eligible))
    STATE.update(metrics,tcps=tcps)

def success(objects):
    return bool(not STATE['violations'] and all(STATE['seen_open']) and STATE['transported'] and
                STATE['docked'] and STATE['clip_closed'] and STATE['upright'] and STATE['stable'] and STATE['hands_clear'])
def score(objects):
    if STATE['violations'] or not STATE['aligned']:return 0.
    value=.15*sum(closed and opened for closed,opened in zip(STATE['cams_closed'],STATE['seen_open']))
    if STATE['docked'] and STATE['transported'] and all(STATE['seen_open']):
        value=.6
        if STATE['clip_closed'] and STATE['upright']:value=.9
    return value
reset_monitor()
