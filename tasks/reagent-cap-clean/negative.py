"""Physical counterexample: lower the cap mouth-down onto the rest."""
import argparse,json
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.constants import READY,FINGER_OPEN
    from specification import INSTRUCTION,TOP,BOTTLE_XY,CAP_START_Z,PAD_XY,PAD_TOP
    from motion import W,Plan,D,GRIP
    from validate import fingerprint
    import imageio.v2 as imageio
    from PIL import Image
    before=fingerprint();x,y=BOTTLE_XY
    emb=yam_arena_embodiment(task=str(Path(__file__).with_name('task.py')),device='cpu',object_jitter=0,control_hz=20)
    import monitor
    obs=emb.reset(Scene(id='rim-down-counterexample',instruction=INSTRUCTION),seed=0)
    moves=[W(2,(x,y,TOP+.16),D,FINGER_OPEN,'approach'),W(2,(x,y,CAP_START_Z+.0195),D,FINGER_OPEN,'descend'),
      W(1.2,(x,y,CAP_START_Z+.0195),D,GRIP,'grasp exterior'),W(2,(x,y,TOP+.16),D,GRIP,'lift'),
      W(2,(*PAD_XY,TOP+.16),D,GRIP,'carry mouth-down'),W(3,(*PAD_XY,PAD_TOP+.015),D,GRIP,'wrong placement'),
      W(1.5,(*PAD_XY,PAD_TOP+.015),D,FINGER_OPEN,'release'),W(2,(*PAD_XY,TOP+.16),D,FINGER_OPEN,'withdraw')]
    plan=Plan('left',moves);writer=imageio.get_writer(args.out/'negative.mp4',fps=5,codec='libx264',macro_block_size=1,output_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
    try:
      writer.append_data(obs.images['top_cam'])
      with (args.out/'trace.jsonl').open('w') as out:
        for step in range(1,401):
          a,label=plan.action(step/20);r=emb.step(Action(data=pack(a,np.r_[READY,1.])))
          out.write(json.dumps({'step':step,'label':label,'monitor':monitor.M})+'\n')
          if step%4==0:writer.append_data(r.observation.images['top_cam'])
      summary={'steps':step,'score':r.info['score'],'monitor':monitor.M.copy(),'source_sha256':before,'sources_unchanged':fingerprint()==before}
      (args.out/'report.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary),flush=True)
      Image.fromarray(r.observation.images['top_cam']).save(args.out/'final.png')
      assert monitor.M['contaminated'] and r.info['score']==0
    finally:writer.close();emb.close()
if __name__=='__main__':main()
