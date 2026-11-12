"""Run robot-only demonstrations and preserve every-step evidence."""
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np

def fingerprint():
    root=Path(__file__).resolve().parent
    return {p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*'))
      if p.is_file() and ('runs' not in p.parts) and (p.suffix=='.py' or p.parent.name=='assets')}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);parser.add_argument('--episodes',type=int,default=1)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.constants import READY
    from specification import INSTRUCTION,MAX_STEPS
    import imageio.v2 as imageio
    from PIL import Image
    before=fingerprint();start=time.monotonic();results=[]
    emb=yam_arena_embodiment(task=str(Path(__file__).with_name('task.py')),device='cpu',object_jitter=0,control_hz=20,cam_width=1280,cam_height=720)
    import monitor
    from motion import plans
    writers={}
    try:
      for ep in range(args.episodes):
        obs=emb.reset(Scene(id=f'cap-{ep}',instruction=INSTRUCTION),seed=0)
        for _ in range(40):result=emb.step(Action(data=pack(np.r_[READY,1.],np.r_[READY,1.])))
        print('IDLE',dict(monitor.M),flush=True)
        obs=emb.reset(Scene(id=f'cap-{ep}',instruction=INSTRUCTION),seed=0);ps=plans()
        for c,f in obs.images.items():
            Image.fromarray(f).save(args.out/f'{c}_setup_{ep}.png')
            writers[c]=imageio.get_writer(args.out/f'{c}_{ep}.mp4',fps=5,codec='libx264',macro_block_size=1,output_params=['-pix_fmt','yuv420p','-movflags','+faststart']);writers[c].append_data(f)
        with (args.out/f'trace_{ep}.jsonl').open('w') as out:
          for step in range(1,MAX_STEPS+1):
            a=[];labels=[]
            for side in ('left','right'):
                v,label=ps[side].action(step/20);a.append(v);labels.append(label)
            result=emb.step(Action(data=pack(*a)));obs=result.observation
            row={'step':step,'score':result.info.get('score'),'monitor':dict(monitor.M),'labels':labels,'action':np.concatenate(a).tolist()}
            from yam_arena.scene import object_states
            s=object_states(emb._env,['cap'])['cap'];row['cap']={'pos':s.pos.tolist(),'quat_xyzw':s.quat_xyzw.tolist(),'lin_vel':s.lin_vel.tolist(),'ang_vel':s.ang_vel.tolist()}
            out.write(json.dumps(row,allow_nan=False)+'\n')
            if step%40==0:
                print(json.dumps(row),flush=True)
                Image.fromarray(obs.images['top_cam']).save(args.out/'latest.png')
            if step%4==0:
                for c,f in obs.images.items():writers[c].append_data(f)
            if result.terminated:break
        for c,f in obs.images.items():Image.fromarray(f).save(args.out/f'{c}_final_{ep}.png')
        for w in writers.values():w.close()
        writers.clear()
        summary={'episode':ep,'steps':step,'score':result.info.get('score'),'success':bool(result.terminated),'monitor':dict(monitor.M),'worst_ik':{s:p.worst.tolist() for s,p in ps.items()}}
        results.append(summary);print('RESULT',json.dumps(summary),flush=True)
        (args.out/'report.json').write_text(json.dumps({'episodes':results,'wall_s':time.monotonic()-start,'source_sha256':before,'sources_unchanged':before==fingerprint()},indent=2)+'\n')
    finally:
        for w in writers.values():w.close()
        emb.close()
if __name__=='__main__':main()
