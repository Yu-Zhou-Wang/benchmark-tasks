"""Record genuine robot-only trials on Brev, including idle and fresh-reset checks."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time


def fingerprint():
    root=Path(__file__).resolve().parent
    return {p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file() and (p.suffix=='.py' or p.parent.name=='assets')}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--episodes',type=int,default=3)
    parser.add_argument('--max-steps',type=int,default=1200)
    parser.add_argument('--dump-stage',action='store_true')
    args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    import imageio.v2 as imageio
    import numpy as np
    from PIL import Image
    from yam_arena.constants import READY
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.scripted import YamScriptedPolicy
    from specification import INSTRUCTION, SPECIMENS
    from scoring import geometry, on_floor_in_tray, released
    started=time.monotonic()
    before=fingerprint()
    emb=yam_arena_embodiment(task=str(Path(__file__).with_name('task.py')),device='cpu',
        object_jitter=0,control_hz=20,cam_width=1280,cam_height=720)
    policy=YamScriptedPolicy(hold_s=5)
    policy.bind(emb.info)
    results=[]
    writers={}
    try:
        for episode in range(args.episodes):
            scene=Scene(id=f'labels-{episode}',instruction=INSTRUCTION)
            obs=emb.reset(scene,seed=0)
            from yam_arena.scene import object_states
            import labelkit
            if args.dump_stage and episode==0:
                import omni.usd
                omni.usd.get_context().get_stage().Export(str(args.out/'scene.usda'))
            Image.fromarray(obs.images['top_cam']).save(args.out/f'setup_{episode}.png')
            idle=[]
            for _ in range(40):
                result=emb.step(Action(data=pack(np.r_[READY,1.],np.r_[READY,1.])))
                obs=result.observation
                idle.append(result.info.get('score'))
            if any(score != 0 for score in idle):
                raise RuntimeError(f'Nonzero idle score: {idle}')
            # Reset again so the demonstrated policy starts at its own t=0.
            obs=emb.reset(scene,seed=0)
            policy.reset(scene)
            for camera, frame in obs.images.items():
                Image.fromarray(frame).save(args.out/f'{camera}_setup_{episode}.png')
                writers[camera]=imageio.get_writer(args.out/f'{camera}_{episode}.mp4',fps=5,codec='libx264',
                    macro_block_size=1,output_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
                writers[camera].append_data(frame)
            with (args.out/f'trace_{episode}.jsonl').open('w') as trace:
                for step in range(1,args.max_steps+1):
                    action=policy.act(obs).actions[0]
                    result=emb.step(action)
                    obs=result.observation
                    states=object_states(emb._env,emb.task.free_objects)
                    row={'step':step,'info':result.info,'objects':{n:{'pos':s.pos.tolist(),'quat_xyzw':s.quat_xyzw.tolist(),
                        'lin_vel':s.lin_vel.tolist(),'ang_vel':s.ang_vel.tolist()} for n,s in states.items()}}
                    trace.write(json.dumps(row,allow_nan=False)+'\n')
                    if step%4==0:
                        for camera,frame in obs.images.items():
                            writers[camera].append_data(frame)
                    if result.terminated or action.meta.get('request_stop'):
                        break
            bounds=labelkit._robot_bounds(emb._env)
            diagnostics={s.key:{'bounds':[a.tolist() for a in geometry(states[s.key])[:2]],
                'correct_tray':on_floor_in_tray(states[s.key],s.destination),
                'released':released(states[s.key],bounds)} for s in SPECIMENS}
            summary={'episode':episode,'idle_score_zero':True,'steps':step,'simulated_s':step/20,
                'score':result.info.get('score'),'success':bool(result.terminated),
                'termination_reason':result.termination_reason,'routing':result.info.get('routing'),
                'diagnostics':diagnostics}
            results.append(summary)
            print(json.dumps(summary),flush=True)
            for camera,frame in obs.images.items():
                Image.fromarray(frame).save(args.out/f'{camera}_final_{episode}.png')
            for writer in writers.values():
                writer.close()
            writers.clear()
            report={'backend':'CPU PhysX on Brev L40S; GPU rendering','control_hz':20,'seed':0,'object_jitter':0,
                'max_steps':args.max_steps,'native_camera_size':[1280,720],
                'episodes':results,'wall_s':time.monotonic()-started,'source_sha256':before,
                'sources_unchanged':before==fingerprint()}
            (args.out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    finally:
        for writer in writers.values():
            writer.close()
        emb.close()


if __name__=='__main__':
    main()
