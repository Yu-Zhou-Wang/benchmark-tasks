"""Remove a loose cap while protecting its inside, then park it inside-up."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import scenekit
from scenekit import UsdBody
from yam_arena.task_file import Box
from specification import *
import monitor
SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=0.
OBJECTS=[
    UsdBody('bottle',pos=(*BOTTLE_XY,TOP),usd='assets/body.usda',size=(.056,.056,.100)),
    UsdBody('cap',pos=(*BOTTLE_XY,CAP_START_Z),usd='assets/cap.usda',size=(.058,.058,.027),mass=.0272838,friction=(.8,.7),torsional_patch=.006),
    Box('bottle_holder',pos=(*BOTTLE_XY,TOP+.002),size=(.07,.07,.004),color=(.3,.3,.3)),
    Box('pad_base',pos=(*PAD_XY,TOP+.003),size=(.12,.12,.006),color=(.2,.65,.35)),
    Box('parking_pad',pos=(*PAD_XY,TOP+PAD_SIZE[2]/2),size=PAD_SIZE,color=(.2,.65,.35)),
]
scenekit.on_reset(monitor.reset)
scenekit.on_step(monitor.step)
def score(objects):return monitor.M['score']
def success(objects):return monitor.M['score']==1. and monitor.M['error'] is None
