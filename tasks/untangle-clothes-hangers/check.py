"""Score the threaded reset, an opposed robot pull and a dangling hanger."""

import argparse
import json
from pathlib import Path

from inspect_robots import Action, Scene
from PIL import Image

from task import Hangers, INSTRUCTION
import mujoco
import numpy as np

from controller import straight_pull
from geometry import bar_height_difference, crossings, hook_engagement, table_angle
from planner import Planner
from scene import MASS, NAMES, THICKNESS
from scoring import MOTION_TOLERANCE, directly_hung, evaluate


def pads_in_contact(env, side, hanger):
    pads = {side + "_tip_" + finger + "_pad" for finger in ("left", "right")}
    held = set()
    for contact in env.data.contact:
        names = [env.model.geom(g).name for g in contact.geom]
        if contact.dist <= 0 and any(n.startswith(hanger + "_") for n in names):
            held.update(pads.intersection(names))
    return held == pads


def bar_contact_force(env, first, second):
    bars = {env.model.geom(name + "_bar_0").id for name in (first, second)}
    wrench = np.zeros(6)
    normal_force = 0.0
    for i, contact in enumerate(env.data.contact):
        if set(contact.geom) == bars:
            mujoco.mj_contactForce(env.model, env.data, i, wrench)
            normal_force += float(wrench[0])
    return normal_force


def run(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    env = Hangers(cameras=False, near_miss=args.case == "near-miss")
    try:
        env.reset(Scene(id=args.case, instruction=INSTRUCTION), seed=0)
        reset = args.case == "reset"
        pull = args.case == "straight-pull"
        outer_a, middle, outer_c = NAMES
        grasped = {}
        if pull:
            for label, action in straight_pull(Planner(env.model)):
                env.step(Action(data=action))
                if label in ("grip outer bars", "opposed straight pull"):
                    arms = (("left", outer_a), ("right", outer_c))
                    grasped[label] = [pads_in_contact(env, side, name) for side, name in arms]
                    positions = [env.data.site(side + "_tcp").xpos[1] for side, _ in arms]
                    if label == "grip outer bars":
                        start_y = positions
                    else:
                        travel = [round(float(sign*(y-y0)*1000), 2)
                                  for y, y0, sign in zip(positions, start_y, (1, -1))]
        camera = "top_cam" if reset or pull else "overview"
        Image.fromarray(env.render(camera, 640, 360)).save(out / "view.png")
        if reset:
            Image.fromarray(env.render("crossing_view", 960, 540)).save(out / "crossing.png")

        def capture(name, phase, data):
            if phase == "lift" and name == (middle if reset else outer_a):
                Image.fromarray(env.render(camera, 960, 540, data=data)).save(out / "lift.png")

        report = dict(case=args.case, robot_actions=env.steps, **evaluate(env.model, env.data, capture))
        lifts = {trial["lifted"]: trial for trial in report["lifts"]}
        support = report["support"]
        if reset:
            hits = {n: crossings(env.data, middle, n) for n in (outer_a, outer_c)}
            report["middle_frame_crossings"] = hits
            engagement = hook_engagement(env.data, outer_a, outer_c)
            angle = table_angle(env.data, outer_a, middle)
            height = bar_height_difference(env.data, outer_a, middle)
            contact_force = bar_contact_force(env, outer_a, middle)
            report.update(orange_purple=engagement, orange_blue_angle_deg=round(angle, 2),
                          orange_above_blue_mm=None if height is None else round(height*1000, 2),
                          orange_blue_bar_contact_N=round(contact_force, 4))
            # Both edges of the same end pass through each opening, on opposite ends of B.
            threaded = all(len(hits[n]) == 2 and all(sign * h["inserted_span_mm"] > 0 for h in hits[n])
                           for n, sign in ((outer_a, 1), (outer_c, -1)))
            moved = lifts.get(middle, {}).get("peer_motion_mm", {})
            caught = all(moved.get(n, 0) > 1000 * MOTION_TOLERANCE for n in (outer_a, outer_c))
            bar_caught = any(hit["part"] == "trouser_bar" for hit in engagement["purple_rim_in_crook"])
            valid = (threaded and caught and engagement["hook_inside_opening_mm"] and bar_caught and 40 < angle < 80
                     and height is not None and height >= THICKNESS and contact_force > 0
                     and all(not directly_hung(load) for load in support.values()))
        elif pull:
            engagement = hook_engagement(env.data, outer_a, outer_c)
            report.update(orange_purple=engagement, both_pads_on_each_bar=grasped,
                          commanded_pull_mm_per_arm=130, outward_gripper_travel_mm=travel)
            valid = (len(grasped) == 2 and all(all(pair) for pair in grasped.values())
                     and all(engagement.values()) and not lifts[outer_a]["independent"])
        else:
            loaded = support[middle]["from_other_N"] >= .8 * MASS * 9.81
            valid = (loaded and not directly_hung(support[middle]) and directly_hung(support[outer_a])
                     and directly_hung(support[outer_c]) and not lifts.get(outer_a, {}).get("independent", True)
                     and lifts.get(outer_c, {}).get("independent", False))
        report["passed"] = bool(report["score"] == 0 and report["released"] and valid)
        (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report), flush=True)
        return int(not report["passed"])
    finally:
        env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=("near-miss", "reset", "straight-pull"))
    parser.add_argument("--out", required=True)
    raise SystemExit(run(parser.parse_args()))
