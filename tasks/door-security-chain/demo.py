"""Robot-only fixed-waypoint controller and evidence runner.

Controller inputs: setup dimensions, elapsed action index and YAM inverse
kinematics. No object pose, scorer metric or segmentation is read by script().
The runner records privileged geometry separately for auditable diagnostics.
"""
from pathlib import Path
import argparse, hashlib, json, math, os, time, traceback
import numpy as np
import geometry as G

HERE=Path(__file__).resolve().parent
HZ=20

class Arm:
    def __init__(self,side):
        from yam_arena import ik,station
        from yam_arena.constants import READY
        self.side=side; self.base=np.array(station.BASES[side]); self.q=np.array(READY,float); self.grip=1.
        self.pos,self.rot=ik.tcp_pose(self.q,self.base); self.home=(self.pos.copy(),self.rot.copy()); self.worst_mm=0.; self.worst_rot_deg=0.
    def aim(self,p,r):
        from yam_arena import ik
        q,err,rot_err=ik.solve(p,ik.matrix_to_quat_xyzw(r),q_init=self.q,base_pos=self.base,iters=80)
        self.q=q; self.pos=np.array(p,float); self.rot=np.array(r,float); self.worst_mm=max(self.worst_mm,err*1000); self.worst_rot_deg=max(self.worst_rot_deg,math.degrees(rot_err))
    def move(self,p,r=None,seconds=1.,grip=None):
        from yam_arena import ik
        p=np.array(p,float); r=self.rot if r is None else np.array(r,float)
        p0,r0,g0=self.pos.copy(),self.rot.copy(),self.grip; g1=g0 if grip is None else grip
        for k in range(1,max(1,round(seconds*HZ))+1):
            s=ik.smoothstep(k/max(1,round(seconds*HZ)))
            self.grip=g0+s*(g1-g0); self.aim(p0+s*(p-p0),ik.slerp_matrix(r0,r,s)); yield
    def action(self): return np.r_[self.q,self.grip]

def wait(t):
    for _ in range(round(t*HZ)): yield

def together(*gens):
    active=list(map(iter,gens))
    while active:
        for g in active[:]:
            try: next(g)
            except StopIteration: active.remove(g)
        if active: yield

FITTING_ROT=np.array([[0,0,1.],[0,-1.,0],[1.,0,0]])
HANDLE_ROT=np.array([[0,0,1.],[-1.,0,0],[0,-1.,0]])

def script(left,right,variant,stage):
    """Fixed layout waypoints only; deliberately has no environment parameter."""
    if variant=='idle':
        stage['name']='idle'; yield from wait(20.); return
    q=G.INITIAL_ANGLE
    stage['name']='approach_handle'
    handle=G.point(G.HANDLE,q); rot=G.rz(q)@HANDLE_ROT
    yield from left.move(handle+G.rz(q)@[-.10,0,0],rot,3.)
    yield from left.move(handle,rot,2.)
    yield from left.move(handle,rot,1.,grip=0.)
    yield from wait(.5)
    if variant=='pull_latched':
        stage['name']='pull_latched'
        for qa in np.linspace(q,math.radians(-55),120):
            left.aim(G.point(G.HANDLE,qa),G.rz(qa)@HANDLE_ROT); yield
        yield from wait(3); return
    if variant!='no_slack':
        stage['name']='close_for_slack'
        for u,qa in zip(np.linspace(0,1,120),np.linspace(q,0,120)):
            left.aim(G.point(G.HANDLE+[.006*u,0,0],qa),G.rz(qa)@HANDLE_ROT); yield
        q=0.
    yield from wait(.75)
    stage['name']='grasp_fitting'
    tab=G.point(G.HEAD+G.TAB,q); rr=G.rz(q)@FITTING_ROT
    yield from right.move(tab+G.rz(q)@[-.09,0,0],rr,3.)
    yield from right.move(tab,rr,2.)
    yield from right.move(tab,rr,1.,grip=0.)
    yield from wait(.5)
    if variant!='no_slide':
        stage['name']='slide_to_wide_exit'
        target=G.HEAD+G.TAB; target=target.copy(); target[1]=G.EXIT_Y
        yield from right.move(G.point(target,q),rr,4.)
    stage['name']='withdraw_fitting'
    yield from right.move(right.pos+G.rz(q)@[-.045,0,0],rr,3.)
    yield from wait(1.)
    if variant in ('no_slack','no_slide'):
        yield from wait(2); return
    stage['name']='release_chain'
    yield from right.move(right.pos,seconds=1.,grip=1.)
    yield from right.move(right.pos+[-.10,-.10,.08],seconds=2.)
    yield from wait(2.)
    stage['name']='open_door'
    for qa in np.linspace(0,math.radians(-58),180):
        left.aim(G.point(G.HANDLE+[.006,0,0],qa),G.rz(qa)@HANDLE_ROT); yield
    stage['name']='hold_open'
    yield from wait(6.)

def capture_sources():
    return {p.relative_to(HERE).as_posix():{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}
            for p in sorted(HERE.rglob('*')) if p.is_file() and '__pycache__' not in str(p) and p.suffix in ('.py','.usda','.json')}

def run(args):
    out=Path(args.out).resolve()
    if out.exists() and any(out.iterdir()): raise ValueError('Require a fresh output directory')
    out.mkdir(parents=True,exist_ok=True); start=time.monotonic()
    manifest={'source_before':capture_sources(),'physical_gpu':os.environ.get('ROBOCURVE_GPU'),
              'controller_observation_contract':'Fixed setup waypoints and robot IK only; no object-state feedback',
              'variant':args.variant,'seed':0,'control_hz':HZ}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    result={'success':False,'task_score':None,'steps':0,'variant':args.variant}; writers={}; emb=None
    try:
        from inspect_robots import Scene
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        from PIL import Image
        import imageio.v2 as imageio
        emb=yam_arena_embodiment(task=str(HERE/'task.py'),object_jitter=0,control_hz=HZ,cam_width=args.width,cam_height=args.height)
        obs=emb.reset(Scene(id='door-security-chain',instruction=emb.task.module.INSTRUCTION),seed=0)
        task=emb.task.module
        task.monitor(emb._env)
        asset=emb._base.scene['latch']
        manifest['runtime']={'physics_dt_s':float(emb._base.sim.get_physics_dt()),
            'control_dt_s':float(emb._base.step_dt),
            'joint_stiffness':dict(zip(asset.joint_names,asset.data.joint_stiffness[0].detach().cpu().tolist())),
            'joint_damping':dict(zip(asset.joint_names,asset.data.joint_damping[0].detach().cpu().tolist()))}
        result['initial']=dict(task.STATE)
        print('RESET_METRICS',json.dumps(task.STATE),flush=True)
        left,right=Arm('left'),Arm('right'); stage={'name':'initial'}
        gen=script(left,right,args.variant,stage)
        for name,img in obs.images.items(): Image.fromarray(np.asarray(img)).save(out/f'setup_{name}.png')
        def record(obs):
            if args.no_record:return
            for name,img in obs.images.items():
                if name not in writers:
                    writers[name]=imageio.get_writer(out/f'{name}.mp4',fps=HZ,codec='libx264',macro_block_size=1,
                                                     output_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
                writers[name].append_data(np.asarray(img))
        record(obs); ended=None; info={}
        original_observation=emb._observation
        if args.no_record:
            from inspect_robots.types import Observation
            emb._base.render_enabled=False
            manager=emb._base.observation_manager
            def state_only(update_history=False):
                observations={'policy':manager.compute_group('policy',update_history=update_history)}
                manager._obs_buffer=observations
                return observations
            manager.compute=state_only
            emb._observation=lambda:Observation(images={},state={},instruction=emb._instruction)
        def fresh_observation():
            for _ in range(3):emb._base.sim.render()
            for name in ('top_cam','left_cam','right_cam'):emb._base.scene[name].update(0.,force_recompute=True)
            return original_observation()
        with (out/'trace.jsonl').open('w') as trace, (out/'actions.jsonl').open('w') as acts:
            for step in range(1,args.max_steps+1):
                try:next(gen)
                except StopIteration:ended=step if ended is None else ended
                action=Action(data=pack(left.action(),right.action()))
                acts.write(json.dumps({'step':step,'left':left.action().tolist(),'right':right.action().tolist()})+'\n')
                res=emb.step(action); info=res.info; record(res.observation)
                metrics=dict(task.STATE)
                if not np.isfinite(metrics.get('door_angle_deg',float('nan'))): raise RuntimeError('Nonfinite articulation state; failed physics probe')
                trace.write(json.dumps({'step':step,'stage':stage['name'],'score':info.get('score'),**metrics})+'\n');trace.flush()
                result.update(steps=step,task_score=float(info.get('score') or 0.),final=metrics)
                if step%40==0:
                    print(json.dumps({'step':step,'stage':stage['name'],'score':info.get('score'),'metrics':metrics}),flush=True)
                if args.snapshots and step%40==0:
                    snap=fresh_observation() if args.no_record else res.observation
                    for name,img in snap.images.items():Image.fromarray(np.asarray(img)).save(out/f'{step:04d}_{name}.png')
                if res.terminated:
                    result['success']=res.termination_reason=='success'
                    if result['success']:result['task_score']=1.
                    break
                if ended is not None and step-ended>=20:break
        result['worst_ik_mm']={'left':left.worst_mm,'right':right.worst_mm}
        result['worst_ik_rotation_deg']={'left':left.worst_rot_deg,'right':right.worst_rot_deg}
        final_obs=fresh_observation() if args.no_record else res.observation
        for name,img in final_obs.images.items():Image.fromarray(np.asarray(img)).save(out/f'final_{name}.png')
        if args.reset_check and result['success']:
            from yam_arena.constants import READY
            ready=np.r_[np.array(READY,float),1.]
            idle=Action(data=pack(ready,ready))
            checks=[]
            with (out/'reset_trace.jsonl').open('w') as rt:
                for episode in range(2):
                    emb.reset(Scene(id=f'reset-check-{episode}',instruction=task.INSTRUCTION),seed=0)
                    valid=True
                    for j in range(200):
                        res=emb.step(idle); m=dict(task.STATE)
                        valid=valid and res.info.get('score')==0 and not m['released'] and m['integrity'] and 20<=m['door_angle_deg']<=24
                        rt.write(json.dumps({'reset':episode+1,'step':j+1,**m})+'\n');rt.flush()
                    checks.append({'reset':episode+1,'steps':200,'passed':bool(valid),'final':dict(task.STATE)})
                    print('RESET_CHECK',json.dumps(checks[-1]),flush=True)
            result['reset_checks']=checks

    except BaseException:
        result['error']=traceback.format_exc();raise
    finally:
        for w in writers.values():w.close()
        result['wall_seconds']=time.monotonic()-start
        manifest['source_after']=capture_sources(); manifest['unchanged']=manifest['source_before']==manifest['source_after']
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
        (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        if emb is not None:
            emb._env.close()
            emb.close()
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--variant',choices=['full','idle','pull_latched','no_slack','no_slide'],default='full')
    p.add_argument('--max-steps',type=int,default=1800);p.add_argument('--no-record',action='store_true');p.add_argument('--snapshots',action='store_true')
    p.add_argument('--reset-check',action='store_true');p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=352)
    args=p.parse_args()
    result=run(args)
    expected=result['success'] if args.variant=='full' else result['task_score']==0. and 'error' not in result
    raise SystemExit(0 if expected else 1)
