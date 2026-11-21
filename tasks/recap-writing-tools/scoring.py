"""Seat and continuous cap-down hold scoring from read-only physics measurements."""

import math

import numpy as np

from yam_arena.urdf import quat_xyzw_to_matrix

from geometry import MM, PHYSICS_HZ, REST_Z, TOOLS, cap_rim, cap_vertices


GAP = 0.60 * MM
OFFSET = 0.90 * MM
CLEARANCE = 10 * MM
TILT = 4.0
ROLL = 5.0
VERTICAL = 12.0
HOLD_SECONDS = 2.0
HOLD_INTERVALS = round(HOLD_SECONDS * PHYSICS_HZ)
GRIP_FORCE = 0.08
OTHER_FORCE = 0.003
RIMS = tuple(cap_rim(tool) for tool in TOOLS)
VERTICES = tuple(cap_vertices(tool) for tool in TOOLS)
PADS = (slice(0, 2), slice(2, 4))


def angle(cosine):
    return math.degrees(math.acos(float(np.clip(cosine, -1, 1))))


def measure_seat(i, barrel_pose, cap_pose):
    tool = TOOLS[i]
    barrel_rot, cap_rot = (quat_xyzw_to_matrix(p[3:]) for p in (barrel_pose, cap_pose))
    axis, cap_axis = barrel_rot[:, 2], cap_rot[:, 2]
    shoulder = barrel_pose[:3] + axis * (tool.shoulder_z * MM)
    rim = RIMS[i] @ cap_rot.T + cap_pose[:3]
    delta = shoulder - (cap_pose[:3] + cap_axis * (tool.mouth_z * MM))
    gap = float(delta @ axis)
    rim_gap = (shoulder - rim) @ axis
    offset = float(np.linalg.norm(delta - gap * axis))
    tilt = angle(axis @ cap_axis)
    roll = angle(abs(float(barrel_rot[:, 0] @ cap_rot[:, 0]))) if tool.rectangular else 0.
    vertical = max(angle(axis[2]), angle(cap_axis[2]))
    bottom = float(np.min((VERTICES[i] @ cap_rot.T + cap_pose[:3])[:, 2]))
    # Positive gap is space between the shoulder plane and cap rim. A negative
    # tolerance accommodates the contact solver's small interpenetration.
    seated = bool(rim_gap.min() > -GAP and rim_gap.max() < GAP
                  and offset <= OFFSET and tilt <= TILT and roll <= ROLL)
    return dict(
        seated=seated, gap_mm=gap / MM, rim_gap_max_mm=float(rim_gap.max() / MM),
        rim_gap_min_mm=float(rim_gap.min() / MM), coaxial_mm=offset / MM,
        relative_tilt_deg=tilt, roll_deg=roll, from_vertical_deg=vertical,
        cap_above_rests_mm=(bottom - REST_Z) / MM,
    )


class Hold:
    """Credit accrues during one uninterrupted barrel-only hold and is revocable."""

    def __init__(self):
        self.samples = 0
        self.best_samples = 0
        self.bimanual_seat = False
        self.proved = False

    @property
    def seconds(self):
        return max(0, self.samples - 1) / PHYSICS_HZ

    def sample(self, seated, eligible, two_grasps=False):
        self.bimanual_seat = bool(seated and (self.bimanual_seat or two_grasps))
        self.samples = self.samples + 1 if eligible and self.bimanual_seat else 0
        self.best_samples = max(self.samples, self.best_samples)
        self.proved = bool(seated and (self.proved or self.samples > HOLD_INTERVALS))
        return self.proved


def pinches(forces, magnitudes):
    """True for each gripper with opposed, load-bearing fingertip contacts."""
    result = []
    for pads in PADS:
        v, n = forces[pads], magnitudes[pads]
        result.append(bool(v[0] @ v[1] <= -0.35 * np.prod(n) and np.all(n >= GRIP_FORCE)))
    return result


class Score:
    def __init__(self):
        self.holds = [Hold() for _ in TOOLS]
        self.metrics = {}
        self.score = 0.
        self.ticks = 0

    def sample(self, poses, normal_forces):
        """The first four filter columns are the two fingers of each hand."""
        self.ticks += 1
        if not np.isfinite(poses).all() or not np.isfinite(normal_forces).all():
            # A corrupt sample cannot continue an existing hold or establish credit.
            for hold in self.holds:
                hold.sample(False, False)
            self.metrics = {}
            self.score = 0.
            return
        magnitudes = np.linalg.norm(normal_forces, axis=2)
        results = []
        for i, (tool, hold) in enumerate(zip(TOOLS, self.holds, strict=True)):
            b, c = 2 * i, 2 * i + 1
            m = measure_seat(i, poses[b], poses[c])
            barrel = magnitudes[b].copy()
            cap = magnitudes[c].copy()
            # The filter list is the four fingertips, then all six parts, then
            # the other robot links and static surroundings.
            barrel[4 + c] = 0.
            cap[4 + b] = 0.
            barrel_grasps = pinches(normal_forces[b], barrel)
            cap_grasps = pinches(normal_forces[c], cap)
            clear = m["cap_above_rests_mm"] >= CLEARANCE / MM
            two_grasps = clear and (
                barrel_grasps[0] and cap_grasps[1] or barrel_grasps[1] and cap_grasps[0]
            )
            hand = None
            for arm, side in enumerate(("left", "right")):
                if (barrel_grasps[arm] and np.max(barrel[PADS[1 - arm]]) < OTHER_FORCE
                        and np.max(barrel[4:]) < OTHER_FORCE):
                    hand = side
            no_cap_contact = bool(np.max(cap) < OTHER_FORCE)
            eligible = bool(hand and no_cap_contact and m["from_vertical_deg"] <= VERTICAL and clear)
            credited = hold.sample(m["seated"], eligible, two_grasps)
            m.update(
                barrel_hand=hand, barrel_pad_normals_N=barrel[:4].tolist(),
                cap_pad_normals_N=cap[:4].tolist(), cap_external_force_N=float(np.max(cap)),
                two_grasps=bool(two_grasps), bimanual_seat=hold.bimanual_seat,
                barrel_only_cap_down=eligible, hold_seconds=hold.seconds,
                longest_hold_seconds=max(0, hold.best_samples - 1) / PHYSICS_HZ, credited=credited,
            )
            self.metrics[tool.name] = m
            results.append(credited)
        self.score = sum(results) / len(TOOLS)
