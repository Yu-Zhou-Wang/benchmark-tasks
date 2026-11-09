"""Prepare an allowlisted development archive; never upload or alter source files.

From the complete task directory:
  python package_task.py --output /path/to/gel-casting-manipulation.zip
  python package_task.py --output /path/to/gel-casting-manipulation.zip --write

The default only checks and prints the proposed contents. An optional
--demo-dir must name a successful full rollout with actual recording files.
Failed/debug runs are excluded. Existing output archives are never overwritten.
The preclamped docking fixture is included only as diagnostic source code; its
runs cannot qualify as a complete demonstration. Passing staging evidence is
required in addition to scoring and retention checks. Bench and rollout source
fingerprints must match the exact packaged task sources and generated assets.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import zipfile


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def source_file(root, relative):
    rel = PurePosixPath(relative)
    if rel.is_absolute() or ".." in rel.parts or "\\" in relative:
        raise ValueError(f"Unsafe package path: {relative}")
    path = (root / Path(*rel.parts)).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise FileNotFoundError(f"Missing or out-of-root required file: {relative}")
    return path


def verify_fingerprint(root, fingerprint, required, packaged_files, label):
    if not isinstance(fingerprint, dict) or fingerprint.get("algorithm") != "sha256":
        raise ValueError(f"{label}: missing SHA-256 source fingerprint")
    recorded = fingerprint.get("files")
    if not isinstance(recorded, dict) or not required.issubset(recorded):
        raise ValueError(f"{label}: fingerprint omits a required executable source or asset")
    for name, digest in recorded.items():
        if name not in packaged_files or not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError(f"{label}: invalid or unpackaged fingerprint entry {name}")
        current = hashlib.sha256(source_file(root, name).read_bytes()).hexdigest()
        if current != digest:
            raise ValueError(f"{label}: evidence does not match current file {name}")


def verify_bench_sources(root, report, driver, common_inputs, files, label):
    before, after = report.get("source_fingerprint"), report.get("source_fingerprint_after")
    if report.get("source_unchanged") is not True or before != after:
        raise ValueError(f"{label}: sources were not verified unchanged across the bench")
    verify_fingerprint(root, before, common_inputs | {driver}, files, label)


def collect(root, manifest, demo_dir=None):
    files = {name: source_file(root, name) for name in manifest["required_files"]}
    if len(files) != len(manifest["required_files"]):
        raise ValueError("The required-file allowlist contains duplicates")
    metadata = load_json(files["task.json"])
    required_fields = {"title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"}
    if not all(isinstance(metadata.get(key), str) and metadata[key].strip() for key in required_fields):
        raise ValueError("Development task metadata is missing a required text field")
    if metadata["category"] not in {"Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relevance", "Other"}:
        raise ValueError("Unknown repository task category")
    common_inputs = {"task.py", "scenekit.py", "geometry.py", "build_task_assets.py", "source_fingerprint.py"}
    common_inputs.update(name for name in files if name.startswith("task_assets/"))
    retention = load_json(files["retention_check.json"])
    if retention.get("passed") is not True:
        raise ValueError("The selected retention report does not record a pass")
    verify_bench_sources(root, retention, "test_retention.py", common_inputs, files, "Retention bench")
    staging = load_json(files["staging_check.json"])
    if not (staging.get("passed") is True
            and staging.get("geometric_lift_check", {}).get("passed") is True
            and staging.get("cases", {}).get("current_jig", {}).get("passed") is True):
        raise ValueError("The selected staging report must pass both current-jig idle and geometric lift checks")
    verify_bench_sources(root, staging, "test_staging.py", common_inputs, files, "Staging bench")
    seating = load_json(files["seating_check.json"])
    verify_bench_sources(root, seating, "test_seating.py", common_inputs, files, "Seating bench")
    seating_policy = manifest["seating_evidence_policy"]
    physical_checks = seating_policy["required_physical_checks"]
    checks = seating.get("checks", {})
    diagnostic = seating_policy["retained_diagnostic"]
    if set(checks) != set(physical_checks) | {diagnostic}:
        raise ValueError("Seating report has missing or unreviewed checks")
    if not all(checks[name] is True for name in physical_checks):
        raise ValueError("At least one required physical seating check failed")
    if seating.get("disturbances", {}).get("identical_in_both_cases") is not True:
        raise ValueError("Seating report does not establish identical closed/open disturbance profiles")
    if seating.get("passed") is not all(checks.values()):
        raise ValueError("Seating report overall pass disagrees with its preserved check results")
    seating_assessment = {
        "original_report_passed": seating["passed"],
        "required_physical_checks_passed": True,
        "physical_check_names": physical_checks,
        "straight_extraction_geometry_passed": checks[diagnostic],
        "interpretation": seating_policy["interpretation"],
    }
    path_report_name = seating_policy["corrected_path_report"]
    path_check = load_json(files[path_report_name])
    if (path_check.get("check") != "revised_cassette_insertion_corridor"
            or path_check.get("passed") is not True or path_check.get("source_unchanged") is not True
            or path_check.get("source_fingerprint") != path_check.get("source_fingerprint_after")
            or path_check.get("path_function") != "demo.dock_forward_offset"):
        raise ValueError("The corrected insertion-path report is missing, failed or not source-bound")
    path_inputs = common_inputs | {"demo.py", "check_cassette_path.py", "contact_geometry_audit.py", "test_seating.py"}
    verify_fingerprint(root, path_check["source_fingerprint"], path_inputs, files, "Corrected insertion path")
    angle_checks = path_check.get("angles", [])
    if (len(angle_checks) != 2 or {row.get("clip_degrees") for row in angle_checks} != {-85.0, -82.0}
            or any(row.get("sample_count") != 141 or len(row.get("samples", [])) != 141
                   or row.get("overlap_sample_count") != 0
                   or any(sample.get("intersections") for sample in row["samples"])
                   for row in angle_checks)):
        raise ValueError("Corrected path evidence does not cover the required 141 poses at each checked lever angle")
    seating_assessment["corrected_path"] = {
        "report": path_report_name, "passed": True,
        "clip_degrees": sorted(row["clip_degrees"] for row in angle_checks),
        "samples_per_angle": 141, "scope": path_check.get("scope"), "limits": path_check.get("limits"),
    }
    scoring_log = files["scoring_check.log"].read_text(encoding="utf-8", errors="replace")
    if not re.search(r"Ran \d+ tests?", scoring_log) or not re.search(r"^OK\s*$", scoring_log, re.MULTILINE):
        raise ValueError("The selected scoring log does not contain a passing unittest result")
    if re.search(r"^(?:FAILED|ERROR:|FAIL:)", scoring_log, re.MULTILINE):
        raise ValueError("The selected scoring log contains a failed test")

    demo_result = None
    if demo_dir is None and checks[diagnostic] is not True:
        raise ValueError("The retained extraction diagnostic failed; a new verified full robot assembly run is required")
    if demo_dir is not None:
        directory = (root / demo_dir).resolve()
        if not directory.is_relative_to(root) or not directory.is_dir():
            raise ValueError("--demo-dir must be a directory within --root")
        demo_spec = manifest["optional_verified_demo"]
        if directory != (root / demo_spec["required_source_directory"]).resolve():
            raise ValueError("This refined package only accepts the explicitly designated runs/refined-final rollout")
        selected = {name: source_file(directory, name) for name in demo_spec["required_files"]}
        demo_result = load_json(selected["result.json"])
        if not (demo_result.get("variant") == "full" and demo_result.get("success") is True
                and demo_result.get("task_score") == 1.0 and 0 < demo_result.get("steps", 0) <= 2400):
            raise ValueError("The selected run is not a successful full demonstration within the task limit")
        closeup = demo_result.get("seating_closeup", {})
        if (closeup.get("image") != "seating_closeup.png"
                or not isinstance(closeup.get("physics_step_before"), int)
                or closeup.get("physics_step_after") != closeup["physics_step_before"]):
            raise ValueError("The seating close-up must record unchanged physics-step counts")
        run_manifest = load_json(selected["run_manifest.json"])
        before, after = run_manifest.get("source_before"), run_manifest.get("source_after")
        if (run_manifest.get("schema_version") != 1 or run_manifest.get("variant") != "full"
                or run_manifest.get("source_unchanged") is not True or before != after
                or demo_result.get("run_manifest") != "run_manifest.json"
                or demo_result.get("source_unchanged") is not True):
            raise ValueError("The selected rollout lacks matching unchanged before/after source evidence")
        verify_fingerprint(root, before, common_inputs | {"demo.py"}, files, "Full rollout")
        settling = load_json(selected["settling_check.json"])
        trace_hash = hashlib.sha256(selected["physics_trace.json"].read_bytes()).hexdigest()
        if not (settling.get("test") == "physics_rate_settling" and settling.get("passed") is True
                and settling.get("sampling_passed") is True and settling.get("pose_passed") is True
                and settling.get("source", {}).get("sha256") == trace_hash):
            raise ValueError("The selected demonstration needs a passing settling report for its exact physics trace")
        videos = sorted(directory.glob("*.mp4"))
        if not videos:
            raise ValueError("A selected demonstration needs an actual MP4 recording")
        for path in videos + sorted(directory.glob("*_final.png")):
            selected[path.name] = source_file(directory, path.name)
        prefix = demo_spec["archive_directory"]
        for name, path in selected.items():
            files[f"{prefix}/{name}"] = path

    # Only exact, explicitly declared built-in MDL source assets may be supplied
    # by the renderer. All other references must resolve to packaged files.
    runtime_modules = {}
    for module in manifest.get("allowed_runtime_modules", []):
        name = module["name"]
        if (not isinstance(name, str) or PurePosixPath(name).name != name
                or "\\" in name or not name.endswith(".mdl") or module.get("type") != "MDL"):
            raise ValueError(f"Invalid exact runtime MDL module name: {name}")
        if name in runtime_modules:
            raise ValueError(f"Duplicate runtime module declaration: {name}")
        runtime_modules[name] = module
    runtime_uses = {}
    included_sources = set(files.values())
    for name, path in files.items():
        if path.suffix == ".usda":
            source = path.read_text(encoding="utf-8")
            for match in re.finditer(r"@([^@\r\n]+)@", source):
                reference = match.group(1)
                dependency = (path.parent / reference).resolve()
                if dependency in included_sources:
                    continue
                declaration = source[source.rfind("\n", 0, match.start()) + 1:match.start()]
                if (reference in runtime_modules
                        and re.search(r"\binfo:mdl:sourceAsset\s*=\s*$", declaration)):
                    runtime_uses.setdefault(reference, set()).add(name)
                    continue
                raise ValueError(f"USD dependency is outside the allowlist: {name} -> {reference}")
    runtime_dependencies = [{**runtime_modules[name], "referenced_by": sorted(users)}
                            for name, users in sorted(runtime_uses.items())]
    return files, metadata, demo_result, runtime_dependencies, seating_assessment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--demo-dir", type=Path, help="Explicit successful recorded full-run folder inside --root")
    parser.add_argument("--write", action="store_true", help="Write the checked archive; otherwise only show the plan")
    args = parser.parse_args()
    root = args.root.resolve()
    manifest = load_json(root / "package_manifest.json")
    files, metadata, demo_result, runtime_dependencies, seating_assessment = collect(root, manifest, args.demo_dir)
    prefix = manifest["task_directory"]
    if PurePosixPath(prefix).name != prefix or prefix in {"", ".", ".."}:
        raise ValueError("Invalid archive task directory")
    inventory = [{"path": name, "bytes": path.stat().st_size,
                  "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                 for name, path in sorted(files.items())]
    record = {"format_version": 1, "status": manifest["status"],
              "created_utc": datetime.now(timezone.utc).isoformat(),
              "title": metadata["title"], "metadata_has_media": bool(metadata.get("media")),
              "robot_demo_included": demo_result is not None,
              "diagnostic_sources": manifest.get("diagnostic_sources", {}),
              "external_runtime_dependencies": runtime_dependencies,
              "seating_evidence_assessment": seating_assessment,
              "evidence_source_binding": "Bench and rollout fingerprints were checked against the exact packaged sources and generated assets; docs are finalized separately.",
              "robot_demo_result": None if demo_result is None else
                  {key: demo_result[key] for key in ("variant", "success", "task_score", "steps")},
              "files": inventory,
              "note": "Development artifact; not uploaded or submitted. Hashes identify packaged bytes, not validation freshness."}
    print(json.dumps(record, indent=2))
    if args.write:
        output = args.output.resolve()
        if output.exists():
            raise FileExistsError(f"Output exists; choose a new path: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
            for name, path in sorted(files.items()):
                archive.write(path, f"{prefix}/{name}")
            archive.writestr(f"{prefix}/PACKAGE_CONTENTS.json", json.dumps(record, indent=2) + "\n")
        print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
