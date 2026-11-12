"""Run a scored scripted or Dream-1 trial with the exact task instruction."""
import argparse
from pathlib import Path
import subprocess
from specification import INSTRUCTION


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('policy',choices=('demo','dream'))
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--epochs',type=int,default=1)
    args=parser.parse_args()
    width,height=(320,176) if args.policy=='dream' else (1280,720)
    command=['inspect-robots','run','--embodiment','yam_arena','--policy',
        'dream' if args.policy=='dream' else 'yam_scripted',
        '-E','device=cpu','-E',f'task={Path(__file__).with_name("task.py").resolve()}',
        '-E','object_jitter=0','-E','control_hz=20','-E',f'cam_width={width}','-E',f'cam_height={height}',
        '--instruction',INSTRUCTION,'--scorer','task_score','--max-steps','1200',
        '--seed','0','--epochs',str(args.epochs),'--no-prompt','--store-frames','--log-dir',str(args.out)]
    subprocess.run(command,check=True)


if __name__=='__main__':
    main()
