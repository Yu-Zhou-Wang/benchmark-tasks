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

from scene import FLOOR, HUB, MOTOR_SPEED, OPEN, build

HOME_POSE = [0.0, 0.65, 1.2, -0.3, 0.0, 0.0]
HZ, LIMIT = 20, 1400
INSTRUCTION = (
    "Reinstall the microwave turntable after cleaning. Center the loose roller ring in the oven, "
    "seat the glass tray on the drive hub, and put the orange mug off center on the tray. "
    "Close the door, press the green start button, and let the mug ride one smooth full turn."
)


def delta(curr, prev):
    return math.atan2(math.sin(curr - prev), math.cos(curr - prev))


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
        self.body_names = [self.model.body(i).name for i in range(self.model.nbody)]
        self.geom_names = [self.model.geom(i).name for i in range(self.model.ngeom)]
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
        self.progress = np.zeros(3)
        self.last = None
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

    def contacts(self):
        d = self.data
        wheels, tracked = set(), set()
        key_force = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist > 0.0002:
                continue
            a, b = (self.geom_names[j] for j in c.geom)
            ba, bb = (self.body_names[self.model.geom_bodyid[j]] for j in c.geom)
            tracked.add(frozenset((ba, bb)))
            if (
                {ba, bb} == {"spindle", "tray"}
                and (a.startswith("drive_pin") or b.startswith("drive_pin"))
                and (a.startswith("socket") or b.startswith("socket"))
            ):
                wrench = np.zeros(6)
                mujoco.mj_contactForce(self.model, d, i, wrench)
                key_force += wrench[0]
            if a == "glass_disk" and b.startswith("roller_"):
                wheels.add(b)
            if b == "glass_disk" and a.startswith("roller_"):
                wheels.add(a)
        return wheels, tracked, key_force

    def measure(self):
        d = self.data
        ring, tray, mug = (d.body(n) for n in ("roller_ring", "tray", "mug"))
        wheels, touch, key_force = self.contacts()
        ring_error = float(np.linalg.norm(ring.xpos[:2] - HUB))
        tray_error = float(np.linalg.norm(tray.xpos[:2] - HUB))
        rel = mug.xpos[:2] - HUB
        radius = float(np.linalg.norm(rel))
        tray_yaw = math.atan2(tray.xmat[3], tray.xmat[0])
        shaft = float(d.joint("shaft").qpos[0])
        phase = abs(delta(3 * tray_yaw, 3 * shaft)) / 3
        dz = float(mug.xpos[2] - tray.xpos[2])
        touching_arm = {
            name for pair in touch if any(n.startswith(("left_", "right_")) for n in pair) for name in pair
        }
        ring_held = any(n == "roller_ring" or n.startswith("wheel_") for n in touching_arm)
        ring_ok = (
            ring_error <= 0.012
            and abs(ring.xpos[2] - FLOOR - 0.010) <= 0.003
            and tilt(ring) <= 4
            and not ring_held
        )
        tray_ok = (
            ring_ok
            and tray_error <= 0.005
            and abs(tray.xpos[2] - FLOOR - 0.024) <= 0.0025
            and tilt(tray) <= 2
            and phase <= math.radians(22)
            and len(wheels) >= 2
            and "tray" not in touching_arm
        )
        mug_ok = (
            tray_ok
            and 0.035 <= radius <= 0.078
            and tilt(mug) <= 5
            and abs(dz - 0.004) <= 0.003
            and frozenset(("mug", "tray")) in touch
            and "mug" not in touching_arm
        )
        door = math.degrees(d.joint("hinge").qpos[0])
        angles = np.array([shaft, tray_yaw, math.atan2(rel[1], rel[0])])
        eligible = mug_ok and self.motor_ticks > 0 and abs(door) <= 2 and key_force >= 0.02
        if eligible and self.last is not None:
            advances = np.array([delta(a, b) for a, b in zip(angles, self.last)])
            expected = MOTOR_SPEED / HZ
            # Every 50 ms all three angles must progress at 55–145% of rated speed.
            if np.all((advances >= 0.55 * expected) & (advances <= 1.45 * expected)):
                self.progress += advances
            else:
                self.progress[:] = 0
        else:
            self.progress[:] = 0
        self.last = angles if eligible else None
        turn = float(np.degrees(self.progress.min()))
        score = 1.0 if turn >= 360 else 0.75 if mug_ok else 0.5 if tray_ok else 0.25 if ring_ok else 0.0
        return dict(
            score=score,
            turn_deg=turn,
            shaft_tray_mug_deg=np.degrees(self.progress).tolist(),
            ring_error_mm=ring_error * 1000,
            tray_error_mm=tray_error * 1000,
            ring_seated=bool(ring_ok),
            tray_seated=bool(tray_ok),
            mug_loaded=bool(mug_ok),
            wheel_contacts=len(wheels),
            key_phase_deg=math.degrees(phase),
            key_contact_N=float(key_force),
            mug_radius_mm=1000 * radius,
            mug_tilt_deg=tilt(mug),
            door_deg=door,
            button_mm=float(d.joint("button").qpos[0]) * 1000,
            motor_on=self.motor_ticks > 0,
        )

    def step(self, action):
        target = np.clip(np.asarray(action.data, dtype=float), self.low, self.high).reshape(2, 7)
        self.data.ctrl[self.arm_drives[:, :6]] = target[:, :6]
        self.data.ctrl[self.arm_drives[:, 6:]] = target[:, 6, None] * FINGER_OPEN
        # Only the independent shaft actuator is powered. The ring, tray and mug are always free.
        for _ in range(self.steps_per_tick):
            closed = abs(self.data.joint("hinge").qpos[0]) <= math.radians(2)
            pressed = self.data.joint("button").qpos[0] >= 0.002
            if not closed:
                self.motor_ticks = 0
            elif pressed and not self.previous_press:
                self.motor_ticks = round(30 / self.model.opt.timestep)
            self.previous_press = pressed
            self.data.actuator("turntable_motor").ctrl[0] = MOTOR_SPEED if self.motor_ticks > 0 else 0.0
            self.motor_ticks = max(0, self.motor_ticks - 1)
            mujoco.mj_step(self.model, self.data)
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
