"""Isolated docking diagnostic. NOT a complete task demonstration.

Starts with cams closed at reset to debug grasp/lever contacts without repeating
the already-verified clamp motions. Full-task scoring is deliberately disabled.
"""
from dataclasses import replace
import math
from task import *

OBJECTS=[replace(obj,joint_pos=(('left_hinge',0.),('right_hinge',0.)),
                 joint_targets=(('left_hinge',math.radians(2)),('right_hinge',math.radians(-2))),
                 joint_stiffness=.30,joint_damping=.003)
         if obj.name=='frame' else obj for obj in OBJECTS]

def success(objects):return False
def score(objects):return 0.
