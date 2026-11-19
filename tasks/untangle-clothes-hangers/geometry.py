"""Check where solid frame edges pierce the actual triangular opening."""

import numpy as np

from scene import BAR_RADIUS, HOOK_RADIUS, HOOK_TIP, HOOK_X, profile


def trouser_frame():
    runs = {name: points for name, points, _ in profile()}
    boundary = [*runs["shoulder_l"], *runs["shoulder_r"][1:], runs["shoulder_l"][0]]
    return np.array([(u, 0, w) for u, w in boundary])


FRAME = trouser_frame()
OUTLINE = FRAME[:, [0, 2]]
HOOK = np.array([(x, 0, z) for name, points, _ in profile() if name == "hook" for x, z in points])
# Close the open hook at its mouth to measure a bar passing through its crook.
CROOK = np.vstack(((HOOK_X, HOOK_TIP), HOOK[1:, [0, 2]], (HOOK_X, HOOK_TIP)))


def interior_distance(point, outline=OUTLINE):
    a, b = outline[:-1], outline[1:]
    edge = b-a
    t = np.clip(np.sum((point-a)*edge, axis=1)/np.sum(edge*edge, axis=1), 0, 1)
    distance = float(np.linalg.norm(point-a-edge*t[:, None], axis=1).min())
    crosses = (a[:, 1] > point[1]) != (b[:, 1] > point[1])
    x = a[crosses, 0] + (point[1]-a[crosses, 1])*edge[crosses, 0]/edge[crosses, 1]
    return distance if np.count_nonzero(x > point[0]) % 2 else -distance


def piercings(data, inserted, line, opening, outline, rim):
    b, a = data.body(inserted), data.body(opening)
    world = line @ b.xmat.reshape(3, 3).T + b.xpos
    points = (world-a.xpos) @ a.xmat.reshape(3, 3)
    for i, (start, end) in enumerate(zip(points[:-1], points[1:])):
        if (start[1] > 0) == (end[1] > 0):
            continue
        t = start[1]/(start[1]-end[1])
        point = start+t*(end-start)
        # Distance from the piercing centreline to the inner edge of the receiving rim.
        margin = interior_distance(point[[0, 2]], outline) - rim
        if margin > 0.001:
            u = line[i, 0] + t*(line[i+1, 0]-line[i, 0])
            yield i, round(float(1000*u), 2), round(1000*margin, 2)


def crossings(data, inserted, opening):
    """Intersect the inserted frame centreline with the receiver's open interior."""
    return [dict(part="trouser_bar" if i == len(FRAME)-2 else "shoulder",
                 inserted_span_mm=u, inside_rim_mm=margin)
            for i, u, margin in piercings(data, inserted, FRAME, opening, OUTLINE, BAR_RADIUS)]


def hook_engagement(data, hooked, receiver):
    """Measure the hook in the aperture and the reciprocal crossing inside its crook."""
    tip = [margin for _, _, margin in piercings(data, hooked, HOOK, receiver, OUTLINE, BAR_RADIUS)]
    rim = [dict(part="trouser_bar" if i == len(FRAME)-2 else "shoulder",
                inside_crook_mm=margin)
           for i, _, margin in piercings(data, receiver, FRAME, hooked, CROOK, HOOK_RADIUS)]
    return dict(hook_inside_opening_mm=tip, purple_rim_in_crook=rim)


def bar_height_difference(data, upper, lower):
    """Signed vertical distance between bar centrelines at their plan-view intersection."""
    a, b = data.body(upper), data.body(lower)
    u, v = (body.xmat.reshape(3, 3)[:, 0] for body in (a, b))
    axes = np.column_stack((u[:2], -v[:2]))
    if abs(np.linalg.det(axes)) < 1e-8:
        return None
    s, t = np.linalg.solve(axes, b.xpos[:2] - a.xpos[:2])
    if max(abs(s), abs(t)) > abs(FRAME[-1, 0]):
        return None
    return float(a.xpos[2] + s*u[2] - b.xpos[2] - t*v[2])


def table_angle(data, first, second):
    axes = [data.body(n).xmat.reshape(3, 3)[:2, 0] for n in (first, second)]
    cosine = abs(axes[0] @ axes[1]) / (np.linalg.norm(axes[0]) * np.linalg.norm(axes[1]))
    return float(np.rad2deg(np.arccos(np.clip(cosine, 0, 1))))
