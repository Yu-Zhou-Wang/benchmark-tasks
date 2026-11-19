"""Terminal-only physical checks. Every lift starts from a copy of the same released state."""

from itertools import combinations

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from scene import MASS, NAMES, RAIL_RADIUS, RAIL_Z, THICKNESS, profile

# A ruler beside the midpoint of the other trouser bar resolves this displacement.
MOTION_TOLERANCE = 0.010
CLEARANCE = 0.003
POINTS = np.array([(x, 0, z, r) for _, points, r in profile() for x, z in points])


def bounds(data, name, axis):
    """Support interval of the full molded profile along any unit world direction."""
    body = data.body(name)
    local = body.xmat.reshape(3, 3).T @ axis
    centers = POINTS[:, :3] @ local + body.xpos @ axis
    radii = POINTS[:, 3] * np.hypot(local[0], local[2]) + abs(local[1]) * THICKNESS / 2
    return float((centers-radii).min()), float((centers+radii).max())


def separating_plane(data, first, second):
    # Disjoint support intervals certify separation for this pair of complete solids.
    axes = [*np.eye(3)]
    for name in (first, second):
        axes.extend(data.body(name).xmat.reshape(3, 3).T)
    for axis in axes:
        a, b = (bounds(data, n, axis) for n in (first, second))
        if a[1] + 0.0001 < b[0] or b[1] + 0.0001 < a[0]:
            return True
    return False


def loads(model, data):
    hanger_ids = [model.body(n).id for n in NAMES]
    rail = model.geom("rail").id
    answer = {n: dict(hook_on_rail_N=0.0, from_other_N=0.0, robot_contact=False) for n in NAMES}
    touching = set()
    wrench = np.zeros(6)
    for i, c in enumerate(data.contact):
        g1, g2 = c.geom
        b1, b2 = model.geom_bodyid[[g1, g2]]
        if b1 not in hanger_ids and b2 not in hanger_ids:
            continue
        mujoco.mj_contactForce(model, data, i, wrench)
        if wrench[0] <= 1e-6:
            continue
        fz = float(c.frame.reshape(3, 3)[:, 2] @ wrench[:3])
        for body, its_geom, other, other_geom, force in ((b1, g1, b2, g2, -fz), (b2, g2, b1, g1, fz)):
            if body not in hanger_ids:
                continue
            n = NAMES[hanger_ids.index(body)]
            if other in hanger_ids:
                touching.add(n)
                answer[n]["from_other_N"] += force
            elif other_geom == rail and model.geom(its_geom).name.startswith(n+"_hook_"):
                answer[n]["hook_on_rail_N"] += force
            elif model.body(other).name.startswith(("left_", "right_")):
                answer[n]["robot_contact"] = True
    return answer, touching


def directly_hung(reading):
    # A freely hanging 47 g hanger weighs 0.461 N.
    return reading["hook_on_rail_N"] >= 0.80 * MASS * 9.81


def copy_state(model, data):
    target = mujoco.MjData(model)
    mujoco.mj_copyData(target, model, data)
    return target


def physical_lift(model, start, name, on_rail, capture=None):
    """A compliant evaluator hand lifts by applying a limited wrench, never writing a pose."""
    d = copy_state(model, start)
    peer_origins = {n: d.body(n).xpos.copy() for n in NAMES if n != name}
    hand = d.body(name)
    origin = hand.xipos.copy()
    bar0 = hand.xpos.copy()
    initial_rot = hand.xmat.reshape(3, 3).copy()
    # Move the hook away from its open tip, in whichever direction it faces on the rail.
    side = 1 if initial_rot[0, 0] >= 0 else -1
    lift_height = 0.047 if on_rail else 0.065
    targets = [np.array([0, 0, lift_height])]
    if on_rail:
        targets.extend((np.array([side * .085, 0, lift_height]), np.array([side * .085, 0, -.048])))
    force = d.xfrc_applied[hand.id]
    dt = model.opt.timestep
    # 4 ms servo update, 2 ms physics. All untested objects remain passive under gravity.
    repeat = round(.004 / dt)
    period = repeat * dt
    ticks = round(.68 / period)
    peer_travel = dict.fromkeys(peer_origins, 0.0)
    largest_rise = 0.0
    previous = np.zeros(3)
    velocity = np.zeros(6)
    if capture:
        capture(name, "start", d)
    for stage, target in enumerate(targets):
        for k in range(1, ticks + 1 + round(.14 / period)):
            u = min(k / ticks, 1.0)
            s = u*u*(3-2*u)
            target_pos = origin + previous + s * (target - previous)
            target_vel = (6*u*(1-u)/(ticks*period)) * (target - previous)
            rot = hand.xmat.reshape(3, 3)
            mujoco.mj_objectVelocity(model, d, mujoco.mjtObj.mjOBJ_BODY, hand.id, velocity, 0)
            # mjOBJ_BODY gives the inertial frame velocity at the centre of mass.
            requested = 175*(target_pos-hand.xipos) + 5.6*(target_vel-velocity[3:])
            requested[2] += MASS * 9.81
            force[:3] = requested / max(1, np.linalg.norm(requested)/3.0)
            torque = .42*Rotation.from_matrix(initial_rot @ rot.T).as_rotvec() - .022*velocity[:3]
            force[3:] = torque / max(1, np.linalg.norm(torque)/.12)
            mujoco.mj_step(model, d, nstep=repeat)
            for peer, start_pos in peer_origins.items():
                travel = float(np.linalg.norm(d.body(peer).xpos - start_pos))
                peer_travel[peer] = max(peer_travel[peer], travel)
            largest_rise = max(largest_rise, float(hand.xpos[2]-bar0[2]))
        previous = target
        if capture:
            capture(name, ("lift", "withdraw", "off_rail")[stage], d)
    end_loads, touching = loads(model, d)
    highest = bounds(d, name, np.array([0., 0., 1.]))[1]
    below = RAIL_Z - RAIL_RADIUS - highest
    taken_off = on_rail and below >= CLEARANCE and not directly_hung(end_loads[name])
    farthest_peer = max(peer_travel.values())
    uncoupled = (largest_rise >= lift_height-.007 and farthest_peer <= MOTION_TOLERANCE
                 and name not in touching and not end_loads[name]["robot_contact"])
    return dict(lifted=name, rise_mm=round(largest_rise*1000, 2),
                other_motion_mm=round(farthest_peer*1000, 2),
                peer_motion_mm={n: round(v*1000, 2) for n, v in peer_travel.items()},
                touching_other_at_end=name in touching,
                independent=bool(uncoupled), off_rail=bool(taken_off),
                crown_below_rail_mm=round(below*1000, 2),
                others_still_on_rail=all(directly_hung(end_loads[n]) for n in peer_origins))


def evaluate(model, endpoint, capture=None):
    """Ruler/independent-lift rubric; the policy's live world is never altered."""
    d = copy_state(model, endpoint)
    release_loads, _ = loads(model, d)
    mujoco.mj_step(model, d, nstep=round(0.75/model.opt.timestep))
    readings, _ = loads(model, d)
    released = all(not (release_loads[n]["robot_contact"] or readings[n]["robot_contact"]) for n in NAMES)
    plane = all(separating_plane(d, a, b) for a, b in combinations(NAMES, 2))
    tests = [physical_lift(model, d, n, directly_hung(readings[n]), capture) for n in NAMES] if released else []
    separated = plane or (released and all(t["independent"] for t in tests))
    all_hung = all(directly_hung(readings[n]) for n in NAMES)
    full = released and all_hung and all(t["independent"] and t["off_rail"] and t["others_still_on_rail"]
                                         for t in tests)
    return dict(score=1.0 if full else 0.5 if separated else 0.0, released=released,
                separated=bool(separated), clear_plane=bool(plane),
                support={n: {k: round(v, 5) if type(v) is float else v for k, v in reading.items()}
                         for n, reading in readings.items()}, lifts=tests)
