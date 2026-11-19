"""Run the bimanual controller and record the calibrated YAM top camera."""

import argparse
import gzip
import json
from pathlib import Path
import time

from inspect_robots import Action, Scene
from PIL import Image

from task import Hangers, HZ, INSTRUCTION, LIMIT
from planner import Planner
from controller import script
from geometry import hook_engagement
from scene import NAMES
from scoring import evaluate


def trace_row(env, stage, row):
    # Diagnostics go only to disk. The controller does not receive this dictionary.
    data, model = env.data, env.model
    pose = {n: [*data.body(n).xpos.round(6), *data.body(n).xquat.round(6)] for n in NAMES}
    tcp = {s: data.site(s+"_tcp").xpos.round(6).tolist() for s in ("left", "right")}
    contacts = set()
    for c in data.contact:
        if c.dist > 0:
            continue
        a, b = (model.geom(i).name for i in c.geom)
        if any(n and n.startswith("hanger_") for n in (a, b)):
            contacts.add((a, b))
    # Replaying rounded targets can change which side of a narrow rim the tip follows.
    return dict(step=env.steps, stage=stage, action=row.tolist(), poses=pose, tcp=tcp,
                contacts=sorted(contacts), joints=env.data.qpos[env.arm_joints].round(6).tolist())


def run(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    env = Hangers(cameras=False)
    start = time.monotonic()
    writer = None
    try:
        env.reset(Scene(id="untangle-clothes-hangers", instruction=INSTRUCTION), seed=0)
        planner = Planner(env.model)
        if args.record:
            import imageio.v2 as imageio

            writer = imageio.get_writer(str(out/"demo.mp4"), fps=HZ, codec="libx264", macro_block_size=1,
                                        pixelformat="yuv420p", output_params=["-movflags", "+faststart"])
            setup = env.render("top_cam", 640, 360)
            Image.fromarray(setup).save(out/"setup.png")
            writer.append_data(setup)
        previous = None
        hook_release = None
        with gzip.open(out/"trace.jsonl.gz", "wt", encoding="utf-8") as trace:
            for label, action in script(planner, lambda: env.render("top_cam")):
                if label != previous:
                    if label == "withdraw orange with purple held":
                        hook_release = dict(step=env.steps, **hook_engagement(env.data, NAMES[0], NAMES[2]))
                        if writer:
                            Image.fromarray(env.render("setup_view", 960, 540)).save(out/"hook-clear.png")
                    if label == "lift outer hangers" and writer:
                        Image.fromarray(env.render("setup_view", 960, 540)).save(out/"after-slide.png")
                    print(env.steps + 1, label, flush=True)
                    previous = label
                env.step(Action(data=action))
                trace.write(json.dumps(trace_row(env, label, action)) + "\n")
                if writer:
                    writer.append_data(env.render("top_cam", 640, 360))
                if env.steps >= LIMIT:
                    break

        def capture(name, phase, data):
            if phase == "off_rail" and writer:
                Image.fromarray(env.render("overview", 960, 540, data=data)).save(out/(name+"-removed.png"))

        if writer:
            Image.fromarray(env.render("overview", 960, 540)).save(out/"released.png")
        final = evaluate(env.model, env.data, capture)
        result = dict(policy="scripted", seed=0, steps=env.steps, action_limit=LIMIT,
                      recording_camera="top_cam" if writer else None, hook_release=hook_release,
                      task_score=final["score"], simulated_seconds=env.steps/HZ,
                      wall_seconds=round(time.monotonic()-start, 3), **final)
        (out/"result.json").write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(result), flush=True)
        return int(final["score"] != 1)
    finally:
        if writer:
            writer.close()
        env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--record", action="store_true")
    raise SystemExit(run(parser.parse_args()))
