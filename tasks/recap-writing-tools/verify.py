"""Summarize and check three normal Inspect Robots logs and their 120 Hz traces."""

import argparse
from itertools import groupby
import json
from pathlib import Path

from geometry import ACTION_LIMIT, TOOLS
from scoring import HOLD_INTERVALS


def load_run(directory):
    path, = sorted(directory.glob("adhoc_*.json"))
    log = json.loads(path.read_text())
    with (directory / "trace.jsonl").open() as traces:
        header = json.loads(next(traces))
        rows = [json.loads(line) for line in traces]
    assert log["status"] == "success" and log["results"]["errored_trials"] == 0, path
    assert log["eval"]["embodiment"] == "yam_arena" and log["eval"]["policy"] == "yam_scripted", path
    score = log["results"]["metrics"]["task_score"]
    count = log["stats"]["total_steps"]
    assert rows[-1]["action"] == count <= ACTION_LIMIT and rows[-1]["score"] == score, path
    ticks = {tool.name: [] for tool in TOOLS}
    substeps = header["physics_hz"] // header["control_hz"]
    for row in rows:
        assert len(row["ticks"]) == substeps, path
        for idx, (tick, parts) in enumerate(row["ticks"], start=1):
            assert (row["action"] - 1) * substeps + idx == tick, path
            for values, frames in zip(parts, ticks.values(), strict=True):
                frame = dict(zip(header["tick_fields"], values, strict=True))
                frames.append(dict(frame, tick=tick, action=row["action"], stage=row["stage"]))
    record = dict(
        log=path.name, run_directory=directory.name, started_utc=log["stats"]["started_at"],
        physical_gpu=header["physical_gpu"], device=header["device"], control=header["control"],
        score=score, actions=count, seconds=count / header["control_hz"],
        wall_seconds=round(log["stats"]["duration_s"], 3),
        termination=log["samples"][0]["termination_reasons"][0],
        inspect_robots_version=log["eval"]["inspect_robots_version"],
        environment_revision=log["eval"]["environment_revision"],
        seed=log["eval"].get("seed"), errored_trials=log["results"]["errored_trials"],
        physics_hz=header["physics_hz"], control_hz=header["control_hz"],
    )
    return record, header, rows[-1]["tools"], ticks


def observed_range(frames):
    range_keys = (
        "gap_mm", "rim_gap_min_mm", "rim_gap_max_mm", "coaxial_mm", "relative_tilt_deg", "roll_deg",
        "from_vertical_deg", "cap_above_rests_mm", "cap_external_force_N",
    )
    return {
        key: [round(min(f[key] for f in frames), 4), round(max(f[key] for f in frames), 4)]
        for key in range_keys
    }


def span(frames, hz):
    return {
        "start_tick": frames[0]["tick"], "end_tick": frames[-1]["tick"],
        "start_action": frames[0]["action"], "end_action": frames[-1]["action"],
        "seconds": round((frames[-1]["tick"] - frames[0]["tick"]) / hz, 4),
        "ranges": observed_range(frames),
    }


def verify_full(record, header, final, ticks):
    assert record["control"] == "normal" and record["score"] == 1.0
    assert record["termination"] == "success"
    record["reset_poses_xyz_xyzw"] = dict(zip(header["parts"], header["poses"], strict=True))
    record["tools"] = {}
    for name, frames in ticks.items():
        assert final[name]["credited"] and final[name]["seated"]
        proof_end = next(i for i, f in enumerate(frames) if f["credited"])
        proof = frames[proof_end - HOLD_INTERVALS:proof_end + 1]
        assert len(proof) == HOLD_INTERVALS + 1
        assert all(f["barrel_only_cap_down"] and f["seated"] and f["bimanual_seat"] for f in proof)
        assert all(f["seated"] for f in frames[proof_end:])
        both = [f for f in frames[:proof_end] if f["two_grasps"] and f["seated"]]
        record["tools"][name] = {
            "bimanual_seat": span(both, record["physics_hz"]),
            "first_passing_hold": span(proof, record["physics_hz"]),
            "final": final[name],
        }
    return record


def verify_partial(record, _, final, ticks):
    assert record["control"] == "partial" and record["score"] == 0.
    frames = ticks["marker"]
    physical_holds = [
        list(group) for keep, group in groupby(frames, lambda f: f["barrel_only_cap_down"]) if keep
    ]
    longest = max(physical_holds, key=len)
    assert len(longest) > HOLD_INTERVALS
    assert all(.6 < f["gap_mm"] < 8.0 and not f["seated"] for f in longest)
    assert not any(f["credited"] for f in frames) and not final["marker"]["seated"]
    record.update(marker_unsupported_retention=span(longest, record["physics_hz"]), final=final["marker"])
    return record


def verify_separation(record, _, final, ticks):
    assert record["control"] == "separation" and record["score"] == 0.
    frames = ticks["marker"]
    both = [f for f in frames if f["two_grasps"] and f["seated"]]
    dropped = final["marker"]["cap_above_rests_mm"] < 0 and final["marker"]["gap_mm"] > 10
    assert both and dropped and not final["marker"]["seated"] and not any(f["credited"] for f in frames)
    first_free = next(i for i, f in enumerate(frames) if f["barrel_only_cap_down"] and f["seated"])
    failure = next(i for i, f in enumerate(frames[first_free:], first_free) if not f["seated"])
    assert failure - first_free < HOLD_INTERVALS and frames[failure]["barrel_only_cap_down"]
    record.update(
        last_bimanual_seat=both[-1], first_barrel_only_seated=frames[first_free],
        first_unseated=frames[failure], seconds_before_unseat=round((failure - first_free) / record["physics_hz"], 4),
        final=final["marker"],
    )
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--normal", required=True, type=Path)
    parser.add_argument("--partial", required=True, type=Path)
    parser.add_argument("--separation", required=True, type=Path)
    parser.add_argument("--repeat", type=Path, help="optional second full rollout")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    summary = {
        "action_limit": ACTION_LIMIT,
        "normal": verify_full(*load_run(args.normal)),
        "partial": verify_partial(*load_run(args.partial)),
        "separation": verify_separation(*load_run(args.separation)),
    }
    if args.repeat is not None:
        repeated = verify_full(*load_run(args.repeat))
        summary["repeat"] = {k: v for k, v in repeated.items() if k not in ("tools", "reset_poses_xyz_xyzw")}
    args.out.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    for mode in ("normal", "partial", "separation", "repeat"):
        if mode in summary:
            print(mode, summary[mode]["score"], summary[mode]["actions"])


if __name__ == "__main__":
    main()
