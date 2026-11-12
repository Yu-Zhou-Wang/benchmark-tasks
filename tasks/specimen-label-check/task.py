"""Compare each specimen's two identifiers with its adjacent request card.

Two identical closed rectangular specimen containers wait beside two wide,
fixed trays labelled PROCESS and QUARANTINE. A fixed request card lies beside
each container, aligned in the same row. Names and sample IDs are printed on the container
lids and the request cards. Read both fields to decide each destination.
Keep the containers upright and closed, then leave both hands clear.
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import labelkit
from labelkit import PrintedBox
from scoring import Monitor
from specification import (FLOOR_THICKNESS, FLOOR_Z, INSTRUCTION, MASS, SIZE,
                           SPECIMENS, TOP, TRAYS, TRAY_SIZE, WALL_HEIGHT, WALL_THICKNESS)
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # Monitor already requires two continuous seconds per placement.
MONITOR = Monitor()
labelkit.UPDATE = MONITOR.update
labelkit.RESET = MONITOR.reset
labelkit.REPORT = MONITOR.report

OBJECTS = []
for sample in SPECIMENS:
    OBJECTS.append(PrintedBox(sample.key, pos=(*sample.xy, TOP + SIZE[2]/2),
        size=SIZE, mass=MASS, color=(0.75,0.77,0.78), texture=f"assets/{sample.key}.png"))
    OBJECTS.append(PrintedBox(sample.key + "_request", pos=(sample.xy[0],0.070,TOP+0.001),
        size=(0.110,0.075,0.002), color=(0.98,0.98,0.96), texture=f"assets/{sample.key}_request.png"))

for name, (x,y) in TRAYS.items():
    prefix = name.lower()
    # Print the tray name in a front strip that remains visible after placement.
    OBJECTS.append(PrintedBox(prefix + "_floor", pos=(x,y,TOP+FLOOR_THICKNESS/2),
        size=(*TRAY_SIZE,FLOOR_THICKNESS), color=(0.65,0.67,0.68),
        texture=f"assets/{prefix}.png", print_size=(0.026,0.112), print_offset=(-0.052,0)))
    for index, side in enumerate((-1,1)):
        OBJECTS.append(Box(f"{prefix}_xwall_{index}", pos=(x+side*(TRAY_SIZE[0]+WALL_THICKNESS)/2,y,FLOOR_Z+WALL_HEIGHT/2),
            size=(WALL_THICKNESS,TRAY_SIZE[1]+2*WALL_THICKNESS,WALL_HEIGHT),color=(0.50,0.52,0.54)))
        OBJECTS.append(Box(f"{prefix}_ywall_{index}", pos=(x,y+side*(TRAY_SIZE[1]+WALL_THICKNESS)/2,FLOOR_Z+WALL_HEIGHT/2),
            size=(TRAY_SIZE[0],WALL_THICKNESS,WALL_HEIGHT),color=(0.50,0.52,0.54)))


def score(objects):
    return MONITOR.value


def success(objects):
    return MONITOR.valid and MONITOR.value == 1.0


def _move(sample):
    x,y = sample.xy
    qx,qy = TRAYS[sample.destination]
    # Pads grip the 44 mm width with finite 20 N nominal squeeze per finger.
    grip = -(SIZE[1]-0.008)/2
    hover = TOP + 0.140
    pick = TOP + 0.030
    place = FLOOR_Z + 0.030 + 0.004
    return [
        Waypoint(1.5,(x,y,hover),FINGER_OPEN,"approach specimen"),
        Waypoint(1.5,(x,y,pick),FINGER_OPEN,"descend"),
        Waypoint(0.6,(x,y,pick),FINGER_OPEN,"settle"),
        Waypoint(1.5,(x,y,pick),grip,"grasp closed container"),
        Waypoint(1.5,(x,y,hover),grip,"lift"),
        Waypoint(2.0,(qx,qy,hover),grip,"carry to identified tray"),
        Waypoint(0.6,(qx,qy,hover),grip,"settle"),
        Waypoint(2.0,(qx,qy,place),grip,"lower"),
        Waypoint(0.8,(qx,qy,place),grip,"settle"),
        Waypoint(1.5,(qx,qy,place),FINGER_OPEN,"release"),
        Waypoint(1.5,(qx,qy,hover),FINGER_OPEN,"withdraw"),
        Waypoint(2.5,(qx,qy,hover),FINGER_OPEN,"observe released stability"),
    ]


def demo():
    moves = [w for sample in SPECIMENS for w in _move(sample)]
    moves += [Waypoint(2.0,(0.28,0.13,TOP+0.14),FINGER_OPEN,"park clear of both trays"),
              Waypoint(3.0,(0.28,0.13,TOP+0.14),FINGER_OPEN,"observe final released stability")]
    return {"left": moves}
