"""Run the hosted Dream-1 policy with the exact submitted instruction."""
import argparse,subprocess,json,hashlib
from pathlib import Path
from specification import INSTRUCTION,MAX_STEPS

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    from validate import fingerprint
    before=fingerprint();(args.out/'source_sha256.json').write_text(json.dumps(before,indent=2)+'\n')
    command=['inspect-robots','run','--embodiment','yam_arena','--policy','dream',
      '-E','device=cpu','-E',f'task={Path(__file__).with_name("task.py").resolve()}',
      '-E','object_jitter=0','-E','control_hz=20','-E','cam_width=320','-E','cam_height=176',
      '--instruction',INSTRUCTION,'--scorer','task_score','--max-steps',str(MAX_STEPS),
      '--seed','0','--epochs','1','--no-prompt','--store-frames','--log-dir',str(args.out)]
    subprocess.run(command,check=True)
    if fingerprint()!=before:raise RuntimeError('Sources changed during model run')
if __name__=='__main__':main()
