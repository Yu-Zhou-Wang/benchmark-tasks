"""RoboCurve microwave-turntable embodiment and continuous loaded-turn scorer."""

import math
import os

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MUJOCO_EGL_DEVICE_ID", "0")

import mujoco
import numpy as np
from inspect_robots import Observation
from inspect_robots.embodiment import (
    EmbodimentBase,
    EmbodimentInfo,
    RESETTABLE,
    SEEDABLE,
    PRIVILEGED_SUCCESS,
)
from inspect_robots.types import StepResult
from inspect_robots_yam.config import DEFAULT_CAMERAS, action_box, observation_space
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN

from scene import FLOOR, FRONT, HUB, MOTOR_SPEED, OPEN, build
from scoring import HZ, LoadedTurn

HOME_POSE = [0.0, 0.65, 1.2, -0.3, 0.0, 0.0]
LIMIT = 1400
INSTRUCTION = (
    "Reinstall the microwave turntable after cleaning. Center the roller ring in the unheated fixture, "
    "seat the clear tray on the three drive pins, and place the orange mug off center. "
    "Close the door, press the green Start button, and let the marked tray and mug ride one full turn together."
)


def tilt(body):
    return math.degrees(math.acos(np.clip(body.xmat[8], -1, 1)))


class Microwave(EmbodimentBase):
    def __init__(self, cameras=True):
        self.model = mujoco.MjModel.from_xml_string(build())
        self.data = mujoco.MjData(self.model)
        self.cameras = cameras
        self.renderers = {}
        self.steps_per_tick = round(1 / HZ / self.model.opt.timestep)
        self.arm_joints = np.array(
            [[self.model.joint(f"{s}_joint{i}").qposadr[0] for i in range(1, 9)] for s in ("left", "right")]
        )
        self.arm_drives = np.array(
            [[self.model.actuator(f"{s}_joint{i}").id for i in range(1, 9)] for s in ("left", "right")]
        )
        self.low = np.tile(np.r_[ik.LOWER, 0], 2)
        self.high = np.tile(np.r_[ik.UPPER, 1], 2)
        self.info = EmbodimentInfo(
            name="microwave_yam",
            action_space=action_box(self.low, self.high),
            observation_space=observation_space(176, 320, DEFAULT_CAMERAS),
            control_hz=HZ,
            is_simulated=True,
            capabilities=frozenset({RESETTABLE, SEEDABLE, PRIVILEGED_SUCCESS}),
            environment_id="microwave-turntable",
        )
        bodies = [self.model.body(i).name for i in self.model.geom_bodyid]
        solid = self.model.geom_contype | self.model.geom_conaffinity
        self.arm_geoms = np.array([i for i, n in enumerate(bodies) if solid[i] and n.startswith(("left_", "right_"))])
        self.part_geoms = {
            part: np.array([i for i, n in enumerate(bodies) if solid[i] and
                            (n == part or (part == "roller_ring" and n.startswith("wheel_")))])
            for part in ("roller_ring", "tray", "mug")
        }
        self.pads = [self.model.geom(f"{side}_tip_{finger}_pad").id
                     for side in ("left", "right") for finger in ("left", "right")]
        self.lamps = [self.model.geom(n).id for n in ("lamp_face", "lamp_plan")]
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[3] = 0
        self.opt.sitegroup[:] = 0

    def reset(self, scene, *, seed=None):
        mujoco.mj_resetData(self.model, self.data)
        self.instruction = scene.instruction or INSTRUCTION
        arm = np.r_[HOME_POSE, FINGER_OPEN, FINGER_OPEN]
        self.data.qpos[self.arm_joints] = arm
        self.data.ctrl[self.arm_drives] = arm
        self.data.joint("hinge").qpos[0] = OPEN
        self.data.ctrl[self.model.actuator("turntable_motor").id] = 0
        mujoco.mj_forward(self.model, self.data)
        mujoco.mj_step(self.model, self.data, nstep=200)
        self.steps, self.motor_ticks, self.previous_press = 0, 0, False
        self.turn = LoadedTurn()
        self.set_lamp(False)
        self.metrics = self.measure()
        return self.observe()

    def render(self, name, width=320, height=176):
        if (width, height) not in self.renderers:
            self.renderers[width, height] = mujoco.Renderer(self.model, height, width)
        renderer = self.renderers[width, height]
        renderer.update_scene(self.data, camera=name, scene_option=self.opt)
        return renderer.render().copy()

    def observe(self):
        joints = self.data.qpos[self.arm_joints]
        opening = np.clip(joints[:, 6:].mean(axis=1) / FINGER_OPEN, 0, 1)
        state = np.column_stack((joints[:, :6], opening)).ravel()
        images = {n: self.render(n) for n in DEFAULT_CAMERAS} if self.cameras else {}
        return Observation(
            images=images,
            state={"joint_pos": state},
            instruction=self.instruction,
            state_time=self.steps / HZ,
            image_times={n: self.steps / HZ for n in images},
        )

    def set_lamp(self, on):
        self.model.geom_rgba[self.lamps] = (0.20, 1.0, 0.22, 1) if on else (0.12, 0.15, 0.13, 1)

    def released(self, part):
        # Visible air gap from robot surfaces. Bounding spheres only prune distant
        # pairs; the final test measures exterior surface separation in metres.
        ids = self.part_geoms[part]
        arms = self.arm_geoms
        pos, radius = self.data.geom_xpos, self.model.geom_rbound
        separations = np.linalg.norm(pos[ids, None] - pos[arms], axis=2)
        close = np.argwhere(separations <= radius[ids, None] + radius[arms] + 0.005)
        for i, j in close:
            gap = mujoco.mj_geomDistance(self.model, self.data, ids[i], arms[j], 0.006, None)
            if gap < 0.005:
                return False
        return True

    def measure(self):
        d = self.data
        ring, tray, mug = (d.body(n) for n in ("roller_ring", "tray", "mug"))
        ring_error = float(np.linalg.norm(ring.xpos[:2] - HUB))
        tray_error = float(np.linalg.norm(tray.xpos[:2] - HUB))
        ring_height, tray_height = ring.xpos[2] - FLOOR, tray.xpos[2] - FLOOR
        ring_tilt, tray_tilt, mug_tilt = tilt(ring), tilt(tray), tilt(mug)
        mug_dot = d.geom("mug_dot").xpos[:2] - HUB
        radius = float(np.linalg.norm(mug_dot))
        # Perpendicular distance from the mug foot center to the actual tray top.
        normal = tray.xmat.reshape(3, 3)[:, 2]
        foot_gap = float((mug.xpos - tray.xpos) @ normal - 0.004)
        ring_ok = bool(ring_error <= 0.012 and abs(ring_height - 0.010) <= 0.003
                       and ring_tilt <= 4 and self.released("roller_ring"))
        tray_ok = bool(ring_ok and tray_error <= 0.005 and abs(tray_height - 0.024) <= 0.0025
                      and tray_tilt <= 2 and self.released("tray"))
        mug_ok = bool(tray_ok and 0.035 <= radius <= 0.078 and mug_tilt <= 5
                      and abs(foot_gap) <= 0.002 and self.released("mug"))
        stripe = d.geom("tray_stripe").xpos[:2] - HUB
        bearings = [math.degrees(math.atan2(y, x)) for x, y in (stripe, mug_dot)]
        latch_zero = d.geom("latch_target").xpos[:2] + [0, 0.020]
        latch_gap = float(np.linalg.norm(d.geom("latch_mark").xpos[:2] - latch_zero))
        pads = d.geom_xpos[self.pads]
        inside = ((pads[:, 0] >= FRONT - 0.010) & (pads[:, 0] <= HUB[0] + 0.16)
                  & (np.abs(pads[:, 1]) <= 0.155) & (pads[:, 2] >= FLOOR) & (pads[:, 2] <= FLOOR + 0.22))
        clear = not inside.any()
        on = self.motor_ticks > 0
        complete = self.turn.sample(*bearings, mug_ok and on and latch_gap <= 0.010 and clear)
        score = 1.0 if complete else 0.75 if mug_ok else 0.5 if tray_ok else 0.25 if ring_ok else 0.0
        return dict(
            score=score, turn_deg=max(0.0, float(self.turn.travel.min())),
            tray_mug_travel_deg=self.turn.travel.tolist(), tray_mug_bearing_deg=bearings,
            best_loaded_turn_deg=self.turn.best_deg, turn_resets=self.turn.resets,
            ring_seated=ring_ok, ring_error_mm=ring_error*1000, ring_height_mm=ring_height*1000,
            ring_tilt_deg=ring_tilt, tray_seated=tray_ok, tray_error_mm=tray_error*1000,
            tray_height_mm=tray_height*1000, tray_tilt_deg=tray_tilt, mug_loaded=mug_ok,
            mug_radius_mm=radius*1000, mug_tilt_deg=mug_tilt, mug_foot_gap_mm=foot_gap*1000,
            latch_gap_mm=latch_gap*1000, grippers_clear=bool(clear),
            start_button_mm=float(d.joint("button").qpos[0])*1000, motor_on=on,
        )

    def step(self, action):
        target = np.clip(np.asarray(action.data, dtype=float), self.low, self.high).reshape(2, 7)
        self.data.ctrl[self.arm_drives[:, :6]] = target[:, :6]
        self.data.ctrl[self.arm_drives[:, 6:]] = target[:, 6, None] * FINGER_OPEN
        # Only the independent shaft actuator is powered. The ring, tray and mug are always free.
        cut_power = False
        for _ in range(self.steps_per_tick):
            closed = abs(self.data.joint("hinge").qpos[0]) <= 2 * math.asin(0.010 / (2 * 0.300))
            pressed = self.data.joint("button").qpos[0] >= 0.002
            if not closed:
                cut_power |= self.motor_ticks > 0
                self.motor_ticks = 0
            elif pressed and not self.previous_press and self.motor_ticks == 0:
                self.motor_ticks = round(30 / self.model.opt.timestep)
            self.previous_press = pressed
            self.data.actuator("turntable_motor").ctrl[0] = MOTOR_SPEED if self.motor_ticks > 0 else 0.0
            self.motor_ticks = max(0, self.motor_ticks - 1)
            mujoco.mj_step(self.model, self.data)
        if cut_power and self.turn.previous is not None:
            self.turn.resets += 1
            self.turn.clear()
        self.set_lamp(self.motor_ticks > 0)
        self.steps += 1
        self.metrics = self.measure()
        success = self.metrics["score"] == 1
        return StepResult(
            observation=self.observe(),
            terminated=success,
            termination_reason="success" if success else None,
            info={"success": success, **self.metrics},
        )

    def close(self):
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()
