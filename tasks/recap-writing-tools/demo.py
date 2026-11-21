"""Scripted bimanual YAM controller using the advertised layout and joint observations."""

import numpy as np
from inspect_robots import Action, ActionChunk
from scipy.spatial.transform import Rotation

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import grip_rotation
from yam_arena.rig import TABLETOP

from geometry import BARREL_GRASP, CONTROL_HZ, MM, TABLE_Z, TOOLS


DOWN = grip_rotation()
OPEN = 0.60
ALIGN_XY = np.array([0.355, 0.0])
CAP_GRASP_Z = 0.937
BASES = tuple(TABLETOP.bases.values())


def around(axis, degrees):
    return Rotation.from_euler(axis, degrees, degrees=True).as_matrix()


def target(position, rotation, grip):
    return np.asarray(position), rotation, grip


def grip_for(tool, cap):
    return (tool.grasp_width(cap) - 0.0022) / (-2 * FINGER_OPEN)


def script(control="normal"):
    tools = TOOLS if control == "normal" else (TOOLS[1],)
    for tool in tools:
        label = tool.name
        barrel, cap = (tool.grasp_position(kind) for kind in (False, True))
        picked = (barrel, cap)
        grips = tuple(grip_for(tool, kind) for kind in (False, True))
        yaws = (-22.5, 67.5 if tool.rectangular else 22.5)
        grip_level = tuple(around("z", yaw) @ around("y", -90) @ DOWN for yaw in yaws)
        rim_z = CAP_GRASP_Z + tool.mouth_z * MM - tool.cap_grasp
        tip_clear = rim_z + BARREL_GRASP + 0.007
        cap_level = np.r_[ALIGN_XY, CAP_GRASP_Z]
        barrel_level = np.r_[ALIGN_XY, tip_clear]
        overtravel_mm = 0.90 if tool.name == "pen" else 0.65
        insertion = (tool.shoulder_z + 7.0 + overtravel_mm) * MM
        if control == "partial":
            insertion -= 5.65 * MM
        seated = barrel_level - [0., 0., insertion]

        def pair(positions, rotations, openings):
            return tuple(target(*args) for args in zip(positions, rotations, openings, strict=True))

        def pickup(height, openings):
            return pair([p + [0., 0., height] for p in picked], [DOWN, DOWN], openings)

        def at_seam(left, right=cap_level, right_grip=grips[1]):
            return pair((left, right), grip_level, (grips[0], right_grip))

        yield f"{label} approach", 30, pickup(0.060, (OPEN, OPEN))
        yield f"{label} descend", 22, pickup(0, (OPEN, OPEN))
        yield f"{label} close", 15, pickup(0, grips)
        yield f"{label} lift", 18, pickup(0.048, grips)
        yield f"{label} turn upright", 30, at_seam(
            barrel_level + [-0.025, 0.105, 0.006], cap_level + [-0.025, -0.130, 0.006]
        )
        yield f"{label} align", 28, at_seam(barrel_level)
        yield f"{label} insert", 43, at_seam(seated)
        yield f"{label} settle", 14, at_seam(seated)
        yield f"{label} release cap", 9, at_seam(seated, right_grip=OPEN)
        right_clear = cap_level + [-0.075, -0.105, 0.0]
        yield f"{label} withdraw cap hand", 18, at_seam(seated, right_clear, OPEN)
        yield f"{label} hold", 49, at_seam(seated, right_clear, OPEN)
        if tool is tools[-1]:
            return
        right_park = target((0.27, -0.19, 0.880), DOWN, OPEN)
        storage = np.array([barrel[0], 0.215 if tool.name == "pen" else 0.265, TABLE_Z + 0.018])
        yield f"{label} turn for storage", 35, (
            target(storage + [0., 0.0, 0.103], DOWN, grips[0]), right_park
        )
        yield f"{label} lower for storage", 31, (target(storage, DOWN, grips[0]), right_park)
        yield f"{label} settle above tabletop", 12, (target(storage, DOWN, grips[0]), right_park)
        yield f"{label} release barrel", 13, (target(storage, DOWN, OPEN), right_park)
        yield f"{label} clear tabletop", 20, (target(storage + [0., 0., 0.090], DOWN, OPEN), right_park)


class RecapController:
    """Joint-space tracking trim cancels steady offsets in the public arm state."""

    def __init__(self, control):
        self.commands = iter(script(control))
        self.q_ref = None
        self.stage = ""
        self.plan_step = 0
        self.trim = np.zeros((2, 6))

    def act(self, observation):
        measured = np.asarray(observation.state["joint_pos"]).reshape(2, 7)
        if self.q_ref is None:
            self.q_ref = measured[:, :6].copy()
            self.end = tuple(
                target(*ik.tcp_pose(q[:6], base), q[6]) for q, base in zip(measured, BASES, strict=True)
            )
        if self.plan_step == 0:
            next_stage = next(self.commands, None)
            if next_stage is None:
                return ActionChunk(
                    actions=[Action(data=self.last, meta={"request_stop": True})], control_hz=CONTROL_HZ
                )
            self.stage, self.duration, next_end = next_stage
            self.start, self.end = self.end, next_end
        self.plan_step += 1
        u = ik.smoothstep(self.plan_step / self.duration)
        command = []
        for i, ((p0, r0, g0), (p1, r1, g1)) in enumerate(zip(self.start, self.end, strict=True)):
            pose = p0 + u * (p1 - p0)
            rot = ik.slerp_matrix(r0, r1, u)
            q, _, _ = ik.solve(
                pose, ik.matrix_to_quat_xyzw(rot), q_init=self.q_ref[i],
                base_pos=BASES[i], iters=32, damping=0.035, tol=1e-6,
            )
            self.q_ref[i] = q
            self.trim[i] = np.clip(self.trim[i] + 0.23 * (q - measured[i, :6]), -0.033, 0.033)
            command.extend(np.r_[np.clip(q + self.trim[i], ik.LOWER, ik.UPPER), g0 + u * (g1 - g0)])
        self.plan_step %= self.duration
        self.last = np.array(command)
        return ActionChunk(
            actions=[Action(data=self.last, meta={"stage": self.stage})], control_hz=CONTROL_HZ
        )
