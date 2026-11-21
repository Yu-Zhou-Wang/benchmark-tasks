"""Recap the blue slim pen, rose whiteboard marker and yellow rectangular highlighter.

From between the robot bases, x is forward and y is left. The tools lie on
open rests in x order, at the 180, 300 and 420 mm grasp rows. Each loose cap
on the right matches the same-color writing tool immediately to its left.
The barrel nibs and cap mouths face in opposite directions along x. The
highlighter cap starts a quarter-turn out of alignment.

Push each matching cap all the way to the barrel shoulder. For each tool,
let go of its cap and hold the tool cap-down by its barrel alone for two
continuous seconds with the cap touching nothing else. Keep every cap on
and fully seated through the end of the trial.
"""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from yam_arena import ik
from yam_arena.task_file import Box

from arena_adapter import install
from demo import RecapController
from geometry import BARREL_MASS, MM, PARTS, TABLE_Z

SCENE = "tabletop"
CONTROL = os.environ.get("RECAP_CONTROL", "normal")
if CONTROL not in ("normal", "partial", "separation"):
    raise ValueError("RECAP_CONTROL must be normal, partial or separation")
INSTRUCTION = json.loads((HERE / "task.json").read_text())["instruction"]


@dataclass(frozen=True)
class PartAsset(Box):
    usd: str = ""
    rot: tuple[float, ...] = (0., 0., 0., 1.)


OBJECTS = [
    PartAsset(
        tool.part_name(cap), tool.reset_origin(cap), mass=tool.cap_mass if cap else BARREL_MASS,
        size=(tool.grasp_width(cap), tool.thickness(cap), tool.mouth_z * MM if cap else 0.120),
        rot=tuple(ik.matrix_to_quat_xyzw(tool.reset_rotation(cap)).tolist()),
        usd=tool.part_name(cap) + (
            "_low_friction" if CONTROL == "separation" and tool.name == "marker" and not cap else ""
        ) + ".usda",
    )
    for tool, cap in PARTS
]
OBJECTS.append(PartAsset("pickup_tray", (0., 0., TABLE_Z), size=(.43, .26, .035), usd="pickup_tray.usda"))


def demo():
    return RecapController(CONTROL)


def success(_objects):
    """The adapter supplies success using physics-substep history."""
    return False


RECAP_WRITING_TOOLS = True
install()
