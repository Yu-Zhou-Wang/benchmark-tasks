"""Run one unmodified task through the installed official Inspect Robots CLI."""
from datetime import datetime, timezone
import hashlib
import importlib.metadata as md
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

root=Path('/home/ubuntu/gel-casting-clamp-work/tasks/gel-casting-complete').resolve()
run=root/'runs/dream1-eval-20261109-01'
run.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(root))
from source_fingerprint import capture

config=run/'explicit-config.ini'
config.write_text('# All trial settings are supplied explicitly on the command line.\n')
instruction=json.loads((root/'task.json').read_text())['instruction']
cmd=['inspect-robots','run','--config',str(config),'--embodiment','yam_arena','--policy','dream',
     '-E','task='+str(root/'task.py'),'-E','object_jitter=0','-E','control_hz=20',
     '-E','cam_width=320','-E','cam_height=176','-E','top_cam=calibrated',
     '-E','wrist_cam_mount=link','-E','headless=true',
     '-P','model=dreamlabs/dream-1-beta','-P','rows_per_request=48','-P','preroll_steps=32',
     '-P','dry_run=false','-P','record=false','-P','timeout_s=300',
     '--instruction',instruction,'--scorer','task_score','--max-steps','2400',
     '--seed','0','--epochs','1','--no-prompt','--grader','none','--fail-on-error','1',
     '--store-frames','--no-rerun','--no-rerun-save','--log-dir',str(run/'eval')]
started=time.monotonic()
before=capture(root)
manifest={'schema_version':1,'started_utc':datetime.now(timezone.utc).isoformat(),
          'source_before':before,'instruction':instruction,'command':cmd,'command_display':shlex.join(cmd),
          'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          'versions':{name:md.version(name) for name in ('inspect-robots','inspect-robots-dream','dreamlabs-client','yam-arena')},
          'settings':{'model':'dreamlabs/dream-1-beta','seed':0,'hosted_model_seeded':False,
                      'epochs':1,'object_jitter_m':0,'control_hz':20,'max_steps':2400,
                      'camera_pixels':[320,176],'cameras':['top_cam','left_cam','right_cam'],
                      'top_camera':'calibrated','wrist_mount':'link','rows_per_request':48,
                      'preroll_steps':32,'default_guardrails':True,'hosted_recording':False},
          'status':'running'}
(run/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'run':str(run),'status':'starting','model':manifest['settings']['model']}),flush=True)
with (run/'console.log').open('w') as log:
    result=subprocess.run(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT,env={**os.environ,'PYTHONUNBUFFERED':'1'})
after=capture(root)
manifest.update(source_after=after,source_unchanged=(before==after),returncode=result.returncode,
                wall_seconds=time.monotonic()-started,finished_utc=datetime.now(timezone.utc).isoformat(),
                status='completed' if result.returncode==0 else 'error')
(run/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({k:manifest[k] for k in ('status','returncode','source_unchanged','wall_seconds')}),flush=True)
sys.exit(result.returncode)
