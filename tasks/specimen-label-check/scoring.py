"""Pose-only scoring, with continuously measured release and stability."""
from __future__ import annotations

import math
import numpy as np

from specification import CLEARANCE, FLOOR_Z, HOLD_S, SIZE, SPECIMENS, TRAYS, TRAY_SIZE, WALL_HEIGHT

HALF = np.asarray(SIZE) / 2
SIGNS = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])


def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def geometry(state):
    arrays = [np.asarray(getattr(state, field), float) for field in ("pos", "quat_xyzw", "lin_vel", "ang_vel")]
    if [a.shape for a in arrays] != [(3,), (4,), (3,), (3,)]:
        raise ValueError("Invalid state dimensions")
    if not all(np.isfinite(a).all() for a in arrays) or not np.isclose(np.linalg.norm(arrays[1]), 1, atol=1e-4):
        raise ValueError("Missing or non-finite pose/velocity")
    pos, quat, vel, spin = arrays
    rot = rotation(quat)
    corners = pos + (SIGNS * HALF) @ rot.T
    return corners.min(axis=0), corners.max(axis=0), rot, vel, spin


def on_floor_in_tray(state, tray):
    low, high, rot, vel, spin = geometry(state)
    center = np.asarray(TRAYS[tray])
    inside = np.all(low[:2] >= center - np.asarray(TRAY_SIZE)/2 - 1e-6) and np.all(high[:2] <= center + np.asarray(TRAY_SIZE)/2 + 1e-6)
    return bool(inside and abs(low[2] - FLOOR_Z) <= 0.003
                and rot[2, 2] >= math.cos(math.radians(10))
                and np.linalg.norm(vel) < 0.010 and np.linalg.norm(spin) < 0.2)


def in_processing(state):
    """Conservative safety gate, including sideways/held/partly overhanging containers."""
    low, high, *_ = geometry(state)
    center = np.asarray(TRAYS["PROCESS"])
    overlaps = np.all(high[:2] > center - np.asarray(TRAY_SIZE)/2) and np.all(low[:2] < center + np.asarray(TRAY_SIZE)/2)
    return bool(overlaps and low[2] <= FLOOR_Z + WALL_HEIGHT + 0.003 and high[2] >= FLOOR_Z - 0.003)


def released(state, robot_bounds):
    low, high, *_ = geometry(state)
    bounds = np.asarray(robot_bounds, float)
    if bounds.ndim != 3 or bounds.shape[1:] != (2, 3) or not len(bounds) or not np.isfinite(bounds).all():
        return False
    if np.any(bounds[:, 0] > bounds[:, 1]):
        return False
    separation = np.maximum(np.maximum(low - bounds[:, 1], bounds[:, 0] - high), 0)
    return bool(np.all(np.linalg.norm(separation, axis=1) >= CLEARANCE))


class Monitor:
    def __init__(self):
        self.reset()

    def reset(self):
        self.elapsed = {s.key: 0.0 for s in SPECIMENS}
        self.anchors = {}
        self.credited = []
        self.unsafe = False
        self.valid = False
        self.value = 0.0

    def update(self, objects, robot_bounds, dt):
        self.credited = []
        self.value = 0.0
        self.valid = False
        try:
            if not math.isfinite(dt) or not 0 < dt <= 0.1:
                raise ValueError("Invalid control interval")
            # Check every input before granting any credit.
            for sample in SPECIMENS:
                geometry(objects[sample.key])
            self.unsafe = any(s.destination != "PROCESS" and in_processing(objects[s.key]) for s in SPECIMENS)
            for sample in SPECIMENS:
                state = objects[sample.key]
                ok = on_floor_in_tray(state, sample.destination) and released(state, robot_bounds)
                if not ok:
                    self.elapsed[sample.key] = 0.0
                    self.anchors.pop(sample.key, None)
                    continue
                anchor = self.anchors.get(sample.key)
                rot = rotation(state.quat_xyzw)
                if anchor is None or np.linalg.norm(state.pos - anchor[0]) > 0.002 or np.trace(anchor[1].T @ rot) < 1 + 2*math.cos(math.radians(3)):
                    self.anchors[sample.key] = (np.asarray(state.pos, float).copy(), rot)
                    self.elapsed[sample.key] = 0.0
                self.elapsed[sample.key] += dt
                if self.elapsed[sample.key] + 1e-9 >= HOLD_S:
                    self.credited.append(sample.key)
            self.valid = True
            self.value = 0.0 if self.unsafe else len(self.credited)/len(SPECIMENS)
        except (KeyError, ValueError, TypeError, AttributeError):
            self.reset()
        return self.value

    def report(self):
        return {"score": self.value, "unsafe_processing": self.unsafe, "valid": self.valid,
                "released_stable_s": dict(self.elapsed), "credited": list(self.credited)}
