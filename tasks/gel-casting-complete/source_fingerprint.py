"""Hash executable task sources and generated assets without modifying them.

Docs are intentionally excluded because results are documented after a run.
The original read-only trace/report hashes serve a separate purpose: this
fingerprint binds evidence to the geometry and code it actually exercised.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

COMMON_SOURCES = (
    "task.py", "scenekit.py", "geometry.py", "build_task_assets.py",
    "source_fingerprint.py",
)


def capture(root, extra_files=(), include_demo=True):
    root = Path(root).resolve()
    names = set(COMMON_SOURCES)
    if include_demo:
        names.add("demo.py")
    names.update(extra_files)
    assets = root / "task_assets"
    if not assets.is_dir():
        raise FileNotFoundError(f"Missing generated assets directory: {assets}")
    names.update(path.relative_to(root).as_posix() for path in assets.rglob("*") if path.is_file())
    hashes = {}
    for name in sorted(names):
        path = (root / name).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise FileNotFoundError(f"Missing or out-of-root fingerprint input: {name}")
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"algorithm": "sha256", "files": hashes}
