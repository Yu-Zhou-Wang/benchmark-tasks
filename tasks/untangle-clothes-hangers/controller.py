"""YAM waypoints with a tabletop pickup localized from the public top camera."""

import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation

from planner import frame
from scene import MIDDLE_FLAT, RAIL_RADIUS, RAIL_TO_BAR, RAIL_X, RAIL_Z, TABLE, THICKNESS


def blue_bar_on_table(image, model):
    """Locate the two blue tape bands on the worktop using the fixed camera calibration."""
    red, green, blue = image.astype(float).transpose(2, 0, 1)
    mask = (green > red + 60) & (green > .68 * blue) & (blue > 1.05 * green)
    labels, count = ndimage.label(mask, structure=np.ones((3, 3)))
    if count < 2:
        raise ValueError("Both blue tape bands must be visible for the tabletop pickup")
    bands = np.argsort(np.bincount(labels.ravel())[1:])[-2:] + 1
    pixels = np.array(ndimage.center_of_mass(mask, labels, bands))[:, ::-1] + .5
    height, width = image.shape[:2]
    cam = model.camera("top_cam").id
    pitch = model.cam_sensorsize[cam, 1] / height
    focal = model.cam_intrinsic[cam, :2] / pitch
    principal = np.array([width, height]) / 2 - model.cam_intrinsic[cam, 2:] / pitch
    uv = (pixels - principal) / focal
    directions = np.column_stack((uv[:, 0], -uv[:, 1], -np.ones(2)))
    rays = Rotation.from_quat(model.cam_quat[cam], scalar_first=True).apply(directions)
    eye = model.cam_pos[cam]
    points = eye + rays * ((TABLE + THICKNESS - eye[2]) / rays[:, 2, None])
    midpoint = points.mean(axis=0)
    midpoint[2] = TABLE + THICKNESS / 2
    axis = points[np.argmax(points[:, 1])] - points[np.argmin(points[:, 1])]
    axis /= np.linalg.norm(axis)
    normal = np.array([0., 0., 1.])
    return midpoint, np.column_stack((axis, normal, np.cross(axis, normal)))


def rotation(heading, pitch, roll):
    return Rotation.from_euler("ZYX", (heading, pitch, roll), degrees=True).as_matrix()


def paired(planner, label, points, jaws, openings, ticks):
    paths = (planner.move(i, point, jaw, opening, ticks)
             for i, (point, jaw, opening) in enumerate(zip(points, jaws, openings)))
    for _ in zip(*paths, strict=True):
        yield label, planner.q.ravel().copy()


def outer_grasps():
    # Fixed, reachable bar poses measured on the declared reset.
    orange = rotation(23, -8.5, 101)
    purple = rotation(32, 4, 82)
    points = np.array([[.343, .071, TABLE+.038], [.325, -.126, TABLE+.015]])
    points += [orange[:, 0] * .058, purple[:, 0] * -.074]
    jaws = np.array([rotation(yaw, 0, 0) @ frame((.75, across, -.65), (0, -1, 0))
                     for yaw, across in ((23, -.10), (32, .10))])
    return points, jaws


def grip_outer_bars(planner, points, jaws):
    yield from paired(planner, "approach orange and purple", points+[0, 0, .084], jaws, [.59]*2, 26)
    yield from paired(planner, "straddle outer bars", points, jaws, [.59]*2, 20)
    yield from paired(planner, "grip outer bars", points, jaws, [0]*2, 13)


def straight_pull(planner):
    points, jaws = outer_grasps()
    yield from grip_outer_bars(planner, points, jaws)
    points += [[0, .130, 0], [0, -.130, 0]]
    yield from paired(planner, "opposed straight pull", points, jaws, [0]*2, 35)
    yield from paired(planner, "release after straight pull", points, jaws, [.62]*2, 10)
    yield from paired(planner, "withdraw after straight pull", points+[0, 0, .09], jaws, [.62]*2, 15)


def script(planner, top_image=None):
    def move(label, arm, point, orientation, opening, ticks):
        for action in planner.move(arm, np.asarray(point), orientation, opening, ticks):
            yield label, action

    def mount(arm, y, offset, jaw, ticks, center_grasp=False):
        entry = np.array([RAIL_X, y, RAIL_Z - .126]) + offset
        seat = np.array([RAIL_X, y, RAIL_Z - RAIL_TO_BAR]) + offset
        seat_jaw = jaw
        if center_grasp:
            # Level the bar held at the pad tips, transfer its weight to the rail,
            # and open a gap in the direction perpendicular to the hanger.
            seat_jaw = Rotation.from_euler("yx", (-2, -2), degrees=True).as_matrix() @ jaw
            seat[2] += .0015
            clear_point = seat + [0, .065, .006]
        elif arm == 0:
            clear_point = seat + [0, .075, .010]
        else:
            # Set purple at its resting pitch about the rail before opening the jaws.
            pitch = Rotation.from_euler("y", 1.5, degrees=True).as_matrix()
            crown = np.array([RAIL_X, y, RAIL_Z + RAIL_RADIUS])
            seat = crown + pitch @ (seat - crown)
            seat_jaw = pitch @ jaw
            clear_point = seat - .075 * seat_jaw[:, 2]
        lift, cross, lower, release, clear = ticks
        yield from move("raise beside rail", arm, entry + [.073, 0, 0], jaw, 0, lift)
        yield from move("pass open hook over rail", arm, entry, jaw, 0, cross)
        yield from move("lower onto rail", arm, seat, seat_jaw, 0, lower)
        yield from move("release bar", arm, seat, seat_jaw, .56, release)
        yield from move("clear bar", arm, clear_point, seat_jaw, .56, clear)

    def park(arm, sign, ticks=24):
        yield from move("withdraw arm", arm, [.18, sign * .30, TABLE + .20],
                        frame((.85, 0, -.52), (0, -1, 0)), .55, ticks)

    points, jaws = outer_grasps()
    yield from grip_outer_bars(planner, points, jaws)
    # Feed the hook toward the purple opening before withdrawing either frame.
    # The right hand supports the rim as the left rolls the open tip through it.
    axis = rotation(23, -8.5, 101)[:, 0]
    for label, orange, purple, roll, ticks in (
        ("feed orange hook into slack", [.010, -.023, .018], [0, 0, .012], 8, 22),
        ("turn orange tip past purple rim", [.018, -.018, .021], [-.008, 0, .012], 15, 25),
        ("lift orange tip through opening", [-.005, 0, .046], [0, 0, 0], 10, 22),
    ):
        points += [orange, purple]
        jaws[0] = Rotation.from_rotvec(np.deg2rad(roll)*axis).as_matrix() @ jaws[0]
        yield from paired(planner, label, points, jaws, [0]*2, ticks)
    points[0] += [0, .217, 0]
    yield from paired(planner, "withdraw orange with purple held", points, jaws, [0]*2, 42)
    points[1] += [0, -.119, 0]
    yield from paired(planner, "withdraw purple", points, jaws, [0]*2, 28)
    points += [[0, 0, .073], [0, 0, .120]]
    yield from paired(planner, "lift outer hangers", points, jaws, [0]*2, 30)

    # A fingertip pinch can turn as the hanger takes its own weight. These
    # fixed grasp transforms describe the carried bars in the rail orientation.
    upright = [frame((.83, -.56, 0), (0, 0, 1)), frame((.932, -.364, 0), (0, 0, 1))]
    offsets = [rotation(15, 0, 0) @ [.0564, .005, 0], np.array([-.0745, .002, 0])]
    rail_ys = [.215, -.335]
    yield from paired(planner, "turn outer hooks upright", points, upright, [0]*2, 55)
    beside = [np.array([RAIL_X+.073, y, RAIL_Z-.246])+offset for y, offset in zip(rail_ys, offsets)]
    yield from paired(planner, "align outside rail", beside, upright, [0]*2, 24)
    paths = [mount(i, y, offset, jaw, ticks=(25, 20, 23, 16, 15))
             for i, (y, offset, jaw) in enumerate(zip(rail_ys, offsets, upright))]
    for (label, _), _ in zip(*paths, strict=True):
        yield label, planner.q.ravel().copy()
    yield from park(1, -1, ticks=20)
    arm = 1
    if top_image is None:
        # Nominal landing pose for open-loop replay of the fixed reset.
        bar = np.array([.425, -.068, TABLE + THICKNESS / 2])
        start = rotation(3.75, 0, 0) @ MIDDLE_FLAT
    else:
        bar, start = blue_bar_on_table(top_image(), planner.model)
    # Centre the grasp along the bar so gravity does not spin it between the pad tips.
    offset = np.array([0., .011, 0])
    grasp = bar + start @ offset
    across = Rotation.from_euler("x", 90, degrees=True).as_matrix()
    flat_jaw = frame((.76, .05, -.65), (0, -1, 0))
    jaw = start @ across.T @ flat_jaw
    yield from move("approach blue bar", arm, grasp + [0, 0, .075], jaw, .64, 23)
    yield from move("straddle blue bar", arm, grasp, jaw, .64, 19)
    yield from move("grip blue bar", arm, grasp, jaw, 0, 12)
    low = np.array([.30, -.095, TABLE + .045])
    yield from move("bring blue clear of table", arm, low, jaw, 0, 24)
    yield from move("turn blue along table", arm, low, flat_jaw, 0, 27)
    upright_jaw = across.T @ flat_jaw
    yield from move("roll blue hook upright", arm, [.29, -.183, TABLE + .122], upright_jaw, 0, 44)
    rail_y = -.185
    beside = np.array([RAIL_X + .073, rail_y, TABLE + .19]) + offset
    yield from move("align middle hanger", arm, beside, upright_jaw, 0, 22)
    yield from mount(arm, rail_y, offset, upright_jaw, ticks=(24, 20, 23, 16, 15), center_grasp=True)
    # The final hanger must settle before either of its neighbors is removed.
    for _ in range(60):
        yield "let all three settle", planner.q.ravel().copy()
