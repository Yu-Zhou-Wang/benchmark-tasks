"""Check the final four contiguous seconds of a 240 Hz physics pose trace.

    python analyze_settling.py runs/demo/physics_trace.json
    python analyze_settling.py runs/demo/physics_trace.json --report runs/demo/settling_check.json

Reads the trace without modifying it. The pose format must explicitly be
[x,y,z,qx,qy,qz,qw]. Exact pairwise pose diameters are computed over the entire
selected tail, without filtering phases or dropping inconvenient samples.
Raw velocity maxima are reported separately and do not determine pose_passed.
This is a sampled physics-rate settling check, not a proof of zero motion
between samples or a substitute for the full task's manipulation history.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

BODY_NAMES = ("frame", "rear", "front", "stand")
JOINTS = (("frame", "left_hinge"), ("frame", "right_hinge"), ("stand", "clip_hinge"))
POSE_FORMAT = ["x", "y", "z", "qx", "qy", "qz", "qw"]
VELOCITY_FORMAT = ["vx", "vy", "vz", "wx", "wy", "wz"]
WINDOW_SECONDS = 4.0
PHYSICS_HZ = 240.0
POSITION_LIMIT_M = 0.0002
ORIENTATION_LIMIT_DEG = 0.2
JOINT_LIMIT_DEG = 0.5


def vector(value, size, label):
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f"{label}: expected {size} finite numbers")
    return array


def analyze(trace, source_name, source_sha256):
    report = {
        "test": "physics_rate_settling",
        "passed": False,
        "source": {"name": source_name, "sha256": source_sha256},
        "pose_format": POSE_FORMAT,
        "criteria": {"last_contiguous_seconds": WINDOW_SECONDS, "physics_hz": PHYSICS_HZ,
                     "position_diameter_m": POSITION_LIMIT_M,
                     "orientation_diameter_deg": ORIENTATION_LIMIT_DEG,
                     "joint_range_deg": JOINT_LIMIT_DEG},
        "sampling_passed": False, "pose_passed": False,
        "bodies": {}, "joints": {}, "errors": [],
        "limitations": [
            "Sampled at the reported physics rate; motion between samples is not measured.",
            "No phase filtering, sample thinning, smoothing, or endpoint-only displacement test.",
            "Raw linear/angular velocity maxima are diagnostic and do not determine the pose pass.",
            "Settling alone does not establish a successful full manipulation task."
        ],
    }
    try:
        if not isinstance(trace, dict):
            raise ValueError("Trace JSON must be an object")
        if trace.get("pose_format") != POSE_FORMAT:
            raise ValueError("Trace must explicitly declare pose format [x,y,z,qx,qy,qz,qw]")
        if trace.get("velocity_format") != VELOCITY_FORMAT:
            raise ValueError("Trace must explicitly declare velocity format [vx,vy,vz,wx,wy,wz]")
        dt = float(trace["physics_dt"])
        if not math.isfinite(dt) or dt <= 0 or not math.isclose(1.0 / dt, PHYSICS_HZ, rel_tol=1e-6):
            raise ValueError("Reported physics_dt must correspond to 240 Hz")
        samples = trace["samples"]
        if not isinstance(samples, list) or not samples:
            raise ValueError("Trace has no physics samples")
        # Include the sample at or immediately before end-4s and every later
        # sample. Never sort, deduplicate, or select a different passing window.
        end_time = float(samples[-1]["time_s"])
        if not math.isfinite(end_time):
            raise ValueError("Final sample timestamp is not finite")
        cutoff = end_time - WINDOW_SECONDS
        start_index = len(samples) - 1
        while start_index > 0 and float(samples[start_index]["time_s"]) > cutoff:
            start_index -= 1
        selected = samples[start_index:]
        if len(selected) < 2:
            raise ValueError("Insufficient samples for a four-second tail")
        steps = [sample["physics_step"] for sample in selected]
        if any(isinstance(step, bool) or not isinstance(step, int) for step in steps):
            raise ValueError("Physics-step identifiers must be integers")
        times = np.asarray([float(sample["time_s"]) for sample in selected], dtype=np.float64)
        if not np.isfinite(times).all():
            raise ValueError("Selected sample timestamps must be finite")
        span = float(times[-1] - times[0])
        consecutive = all(b == a + 1 for a, b in zip(steps, steps[1:]))
        constant_dt = bool(np.allclose(np.diff(times), dt, rtol=1e-7, atol=1e-9))
        step_time_consistent = bool(np.allclose(times, np.asarray(steps, dtype=np.float64) * dt,
                                               rtol=0.0, atol=1e-7))
        report["coverage"] = {"input_samples": len(samples), "selected_samples": len(selected),
                              "start_index": start_index, "first_physics_step": steps[0],
                              "last_physics_step": steps[-1], "start_time_s": float(times[0]),
                              "end_time_s": float(times[-1]), "span_s": span,
                              "physics_dt_s": dt, "reported_hz": 1.0 / dt,
                              "unique_consecutive_steps": consecutive,
                              "constant_time_increment": constant_dt,
                              "time_matches_physics_steps": step_time_consistent}
        if span < WINDOW_SECONDS - 1e-9:
            raise ValueError(f"Trace tail spans only {span:.9f} seconds; at least four are required")
        if not consecutive or not constant_dt or not step_time_consistent:
            raise ValueError("The final four-second tail has duplicate, missing, reordered or mistimed physics samples")
        report["sampling_passed"] = True

        for name in BODY_NAMES:
            poses = np.stack([vector(sample["bodies"][name]["pose_w_xyzw"], 7,
                                     f"{name} pose at step {step}") for sample, step in zip(selected, steps)])
            velocities = np.stack([vector(sample["bodies"][name]["velocity_w"], 6,
                                          f"{name} velocity at step {step}") for sample, step in zip(selected, steps)])
            positions = poses[:, :3]
            norms = np.linalg.norm(poses[:, 3:7], axis=1)
            if np.any(norms < 1e-12):
                raise ValueError(f"{name}: zero-length quaternion")
            quaternions = poses[:, 3:7] / norms[:, None]
            # Exact all-pairs Euclidean diameter, not axis-wise extents.
            delta = positions[:, None, :] - positions[None, :, :]
            squared_distance = np.einsum("ijk,ijk->ij", delta, delta)
            position_pair = np.unravel_index(int(np.argmax(squared_distance)), squared_distance.shape)
            position_diameter = math.sqrt(float(squared_distance[position_pair]))
            # abs(dot) makes q and -q equivalent; normalization precedes this.
            absolute_dot = np.clip(np.abs(quaternions @ quaternions.T), 0.0, 1.0)
            orientation_pair = np.unravel_index(int(np.argmin(absolute_dot)), absolute_dot.shape)
            orientation_diameter = math.degrees(2.0 * math.acos(float(absolute_dot[orientation_pair])))
            report["bodies"][name] = {
                "passed": position_diameter <= POSITION_LIMIT_M and orientation_diameter <= ORIENTATION_LIMIT_DEG,
                "position_diameter_m": position_diameter,
                "orientation_diameter_deg": orientation_diameter,
                "position_extreme_physics_steps": [steps[int(i)] for i in position_pair],
                "orientation_extreme_physics_steps": [steps[int(i)] for i in orientation_pair],
                "quaternion_norm_range": [float(norms.min()), float(norms.max())],
                "max_raw_speed_m_s": float(np.linalg.norm(velocities[:, :3], axis=1).max()),
                "max_raw_spin_rad_s": float(np.linalg.norm(velocities[:, 3:6], axis=1).max()),
            }

        for body, joint in JOINTS:
            angles = np.asarray([float(sample["joints"][body][joint]) for sample in selected])
            if not np.isfinite(angles).all():
                raise ValueError(f"{body}/{joint}: nonfinite joint angle")
            low, high = float(angles.min()), float(angles.max())
            range_deg = math.degrees(high - low)
            report["joints"][f"{body}/{joint}"] = {
                "passed": range_deg <= JOINT_LIMIT_DEG,
                "min_rad": low, "max_rad": high, "range_deg": range_deg,
            }
        report["pose_passed"] = all(item["passed"] for item in report["bodies"].values()) and all(
            item["passed"] for item in report["joints"].values())
        report["passed"] = report["sampling_passed"] and report["pose_passed"]
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        report["errors"].append(str(error))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="physics_trace.json produced by demo.py --physics-trace")
    parser.add_argument("--report", type=Path, help="Output JSON; defaults to settling_check.json beside the trace")
    args = parser.parse_args()
    source = args.trace.resolve()
    output = (args.report or source.with_name("settling_check.json")).resolve()
    if output == source or (output.exists() and output.samefile(source)):
        parser.error("The report path must not overwrite the input trace")
    raw = source.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    try:
        trace = json.loads(raw.decode("utf-8-sig"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        trace = {}
        report = analyze(trace, source.name, digest)
        report["errors"].insert(0, f"Cannot decode input JSON: {error}")
    else:
        report = analyze(trace, source.name, digest)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
