"""Release the chain and open the bench door.

A 36 cm wide hinged plywood panel carries a full-size brass slotted chain
plate. Its steel chain runs to the fixed jamb. The loaded fitting starts at
the closed end of the narrow slot; the wide opening is toward the hinge.
A light, idealized hinge spring supplies the opening load. The black vertical
door handle is below and toward the hinge from the chain fitting.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import math
import numpy as np
import geometry as G
from scenekit import UsdBody, body_link_poses, on_reset, on_step

SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=1.5
INSTRUCTION=('Release the door security chain and open the door at least 50 degrees. '
             'Finish with the chain fitting free of the slotted plate and both grippers clear of the fitting. '
             'You may keep holding the door handle.')
OBJECTS=[UsdBody('latch',usd='assets/latch.usda',pos=tuple(G.HINGE),size=(.20,.47,.34),articulation=True,
                 joint_pos=(("door_hinge",G.INITIAL_ANGLE),),
                 joint_stiffness={'door_hinge':G.SPRING_K,'bend.*|end.*':0.0},
                 joint_damping={'door_hinge':G.SPRING_DAMPING,'bend.*|end.*':.0003},
                 joint_targets=(("door_hinge",G.SPRING_TARGET),),
                 joint_friction=0.0,solver_iters=64,solver_velocity_iters=4,
                 friction=(.45,.35))]

STATE={}
_prev_head=None

def evaluate(head_local,head_rotation,angle_deg,angular_speed_deg,head_speed,gripper_clear,integrity=True):
    """Geometry-only rubric; metres, degrees, seconds. No unlocking state machine."""
    p=np.asarray(head_local); half=np.abs(head_rotation)@G.HEAD_HALF
    lo,hi=p-half,p+half
    plate_lo=np.array([-.020,-.345,.226]); plate_hi=np.array([-.009,-.240,.264])
    separation=float(max(np.max(plate_lo-hi),np.max(lo-plate_hi)))
    free=separation>=.004
    at_exit=abs(p[1]-G.EXIT_Y)<=.006 and abs(p[2]-.245)<=.005 and -.017<=p[0]<=-.010
    stable=abs(angular_speed_deg)<=2.0 and head_speed<=.06
    complete=bool(integrity and free and angle_deg>=50 and stable and gripper_clear)
    partial=(.5 if free else .25 if at_exit else 0.) if integrity else 0.
    return {'head_local_m':p.tolist(),'head_bounds_local_m':[lo.tolist(),hi.tolist()],
            'head_plate_separation_m':separation,'released':bool(free),'at_exit':bool(at_exit),
            'door_angle_deg':float(angle_deg),'door_speed_deg_s':float(angular_speed_deg),
            'head_speed_m_s':float(head_speed),'grippers_clear':bool(gripper_clear),
            'integrity':bool(integrity),'stable':bool(stable),'complete':complete,'partial_score':partial}

@on_reset
def reset_monitor(env):
    global _prev_head
    STATE.clear(); _prev_head=None

@on_step
def monitor(env):
    global _prev_head
    poses=body_link_poses(env,'latch'); door=poses['door']; head=poses['fitting']
    dr=G.rotation(door[3:]); hr=G.rotation(head[3:]); local=dr.T@(head[:3]-door[:3])
    asset=env.unwrapped.scene['latch']
    q=asset.data.joint_pos[0].detach().cpu().numpy(); dq=asset.data.joint_vel[0].detach().cpu().numpy()
    idx=asset.joint_names.index('door_hinge')
    angle=-math.degrees(float(q[idx])); speed=-math.degrees(float(dq[idx]))
    hs=0. if _prev_head is None else float(np.linalg.norm(head[:3]-_prev_head)*20)
    _prev_head=head[:3].copy()
    chainpoints=[]
    authored=G.chain_points()
    gaps=[]
    for k in range(G.N_LINKS):
        p=poses[f'link{k:02d}']; length=np.linalg.norm(authored[k+1]-authored[k])
        a=p[:3]; b=a+G.rotation(p[3:])@np.array([length,0,0]); chainpoints.append(a)
        nxt=poses[f'link{k+1:02d}'][:3] if k+1<G.N_LINKS else head[:3]+hr@G.ATTACH
        gaps.append(float(np.linalg.norm(b-nxt)))
    chainpoints.append(head[:3]+hr@G.ATTACH)
    anchor=G.HINGE+G.ANCHOR
    gaps.append(float(np.linalg.norm(chainpoints[0]-anchor)))
    integrity=max(gaps)<.005 and np.linalg.norm(poses['frame'][:3]-G.HINGE)<.002
    tab=head[:3]+hr@G.TAB
    tcp=[]
    for side in ('left','right'):
        gp=body_link_poses(env,f'{side}_robot')['gripper']
        tcp.append(gp[:3]+G.rotation(gp[3:])@np.array([0.,0.,.134]))
    clear=all(min(np.linalg.norm(p-head[:3]),np.linalg.norm(p-tab))>=.055 for p in tcp)
    STATE.update(evaluate(local,dr.T@hr,angle,speed,hs,clear,integrity))
    endpoint=np.array(chainpoints[-1]); distance=float(np.linalg.norm(endpoint-anchor))
    STATE.update(chain_endpoint_distance_m=distance,chain_length_m=G.CHAIN_LENGTH,
                 chord_allowance_m=G.CHAIN_LENGTH-distance,max_joint_gap_m=max(gaps),
                 head_world_m=head[:3].tolist(),tab_world_m=tab.tolist(),
                 hinge_spring_torque_nm=G.SPRING_K*(G.SPRING_TARGET-float(q[idx])),
                 door_handle_world_m=(door[:3]+dr@G.HANDLE).tolist())

def success(objects): return bool(STATE.get('complete',False))
def score(objects): return float(STATE.get('partial_score',0.))
