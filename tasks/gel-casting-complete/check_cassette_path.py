"""Verify the revised cassette insertion corridor using exact convex colliders.

This CPU-only check reads authored USD geometry. It does not run physics, move
live objects, or establish a dynamic/swept-collision guarantee.
"""
import argparse
import importlib
import json
from pathlib import Path
import sys

import numpy as np
from contact_geometry_audit import Geometry, intersections


def check(task_root):
    root = Path(task_root).resolve()
    sys.path.insert(0, str(root))
    from source_fingerprint import capture
    demo = importlib.import_module('demo')
    if Path(demo.__file__).resolve().parent != root:
        raise RuntimeError('The checker must use demo.py from the fingerprinted task root.')
    extra = ('check_cassette_path.py', 'contact_geometry_audit.py', 'test_seating.py')
    before = capture(root, extra_files=extra, include_demo=True)
    # asset() requires no robot-instance state; avoid unrelated robot meshes.
    geometry = Geometry.__new__(Geometry)
    assets = root / 'task_assets'
    origin = np.array([.16, 0., .7475])
    rotation = np.diag([-1., -1., 1.])
    cassette = geometry.asset(assets / 'casting_frame.usda', origin, rotation,
        joint_transforms={'/CastingFrame/left_cam': ('Z', 0.),
                          '/CastingFrame/right_cam': ('Z', 0.)})
    cassette += geometry.asset(assets / 'rear_plate.usda',
        origin + rotation @ np.array([0., 0., .053]), rotation)
    cassette += geometry.asset(assets / 'front_plate.usda',
        origin + rotation @ np.array([.004, 0., .0485]), rotation)
    angles = []
    for angle in (-85., -82.):
        stand = geometry.asset(assets / 'casting_stand.usda', origin, rotation,
            joint_transforms={'/CastingStand/pressure_clip': ('Y', angle)})
        samples = []
        for millimetres in range(141):
            height = millimetres / 1000
            forward = demo.dock_forward_offset(height)
            shift = rotation @ np.array([forward, 0., height])
            moved = []
            for name, low, high, equations in cassette:
                translated = equations.copy()
                translated[:, 3] -= translated[:, :3] @ shift
                moved.append((name, low + shift, high + shift, translated))
            hits = intersections(moved, stand)
            samples.append({'height_m': height, 'forward_offset_m': forward,
                            'intersections': hits})
        angles.append({'clip_degrees': angle, 'sample_count': len(samples),
                       'overlap_sample_count': sum(bool(s['intersections']) for s in samples),
                       'samples': samples})
    after = capture(root, extra_files=extra, include_demo=True)
    unchanged = before == after
    return {
        'check': 'revised_cassette_insertion_corridor',
        'passed': unchanged and all(a['overlap_sample_count'] == 0 for a in angles),
        'source_fingerprint': before, 'source_fingerprint_after': after,
        'source_unchanged': unchanged,
        'path_function': 'demo.dock_forward_offset',
        'height_step_m': .001, 'strict_penetration_depth_m': .00005,
        'scope': 'Cassette versus all enabled stand colliders, including invisible proxies.',
        'limits': 'Nominal frame orientation and grasp; discrete poses, not a continuous or dynamic guarantee.',
        'angles': angles,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-root', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = check(args.task_root)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'angles'}, indent=2))
    print('overlap samples:', [a['overlap_sample_count'] for a in result['angles']])
    raise SystemExit(0 if result['passed'] else 1)
