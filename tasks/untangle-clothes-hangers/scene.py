"""Full-size molded hangers. Local x follows the trouser bar and local z points up the hook."""

import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import station

from station_model import add_arms, numbers, quat

TABLE = station.TABLE_TOP
MASS = 0.047
THICKNESS = 0.004
RAIL_X, RAIL_Z, RAIL_RADIUS = 0.405, TABLE + 0.435, 0.0125
HOOK_X, HOOK_TOP, HOOK_TIP = 0.026, 0.184, 0.154
BAR_RADIUS = 0.0045
HOOK_RADIUS = 0.0040
UPRIGHT = np.eye(3)
CROSSWISE = Rotation.from_euler("z", 90, degrees=True).as_matrix()
MIDDLE_FLAT = np.column_stack(([0., 1., 0.], [0., 0., 1.], [1., 0., 0.]))
RAIL_TO_BAR = HOOK_TOP - HOOK_RADIUS - RAIL_RADIUS
NAMES = ("hanger_a", "hanger_b", "hanger_c")
# Bar midpoints above the tabletop (m) and intrinsic Z-Y-X angles (degrees).
# Blue pierces both frames. Orange's open hook loops around the purple bar
# with its tip through the purple opening. The closed triangular frames can slide free.
TABLETOP_POSES = (
    ((.339, .074, .059), (24., -8.5, 100.5)),
    ((.335, .009, .052), (87.9, 5., 83.)),
    ((.326, -.117, .054), (29.5, 15., 68.5)),
)
FRAME_COLORS = (".94 .36 .06 1", ".08 .28 .69 1", ".57 .20 .68 1")
TAPE_COLORS = (".96 .50 .08 1", ".04 .70 .96 1", ".78 .13 .54 1")


def initial_poses(near_miss=False):
    if near_miss:
        upper = np.array([RAIL_X, -.175, RAIL_Z - RAIL_TO_BAR])
        lower = upper + [0, 0, BAR_RADIUS - (HOOK_TOP - HOOK_RADIUS)]
        third = np.array([RAIL_X, .215, RAIL_Z - RAIL_TO_BAR])
        return ((upper, UPRIGHT), (lower, CROSSWISE), (third, UPRIGHT))
    return tuple((np.array(pos) + [0, 0, TABLE],
                  Rotation.from_euler("ZYX", angles, degrees=True).as_matrix())
                 for pos, angles in TABLETOP_POSES)


def profile():
    """Photograph-scaled centrelines in metres; each named run has a separate physical tip."""
    edge = [
        (-0.159, 0.000), (-0.173, -0.0015), (-0.182, 0.001), (-0.189, 0.009),
        (-0.1905, 0.020), (-0.189, 0.032), (-0.184, 0.043), (-0.177, 0.049),
        (-0.040, 0.101), (-0.034, 0.100), (-0.031, 0.094), (-0.027, 0.085),
        (-0.019, 0.079), (-0.008, 0.077), (0.004, 0.078), (0.015, 0.083),
        (0.023, 0.091), (HOOK_X, 0.103),
    ]
    far_edge = [
        (HOOK_X, 0.103), (0.177, 0.049), (0.184, 0.043), (0.189, 0.032),
        (0.1905, 0.020), (0.189, 0.009), (0.182, 0.001), (0.173, -0.0015),
        (0.159, 0.000),
    ]
    hook = [(HOOK_X, 0.103), (HOOK_X, 0.157)]
    for angle in np.linspace(0, math.pi, 15)[1:]:
        hook.append((HOOK_X * math.cos(angle), 0.157 + (HOOK_TOP - 0.157) * math.sin(angle)))
    hook.append((-0.027, HOOK_TIP))
    runs = [
        ("bar", [(-0.159, 0), (0.159, 0)], BAR_RADIUS),
        ("shoulder_l", edge, BAR_RADIUS), ("shoulder_r", far_edge, BAR_RADIUS),
        ("hook", hook, HOOK_RADIUS),
    ]
    for side, sign in (("l", -1), ("r", 1)):
        start = 0.068 if sign < 0 else 0.066
        loop = [(sign * x, z) for x, z in (
            (0.129, start), (0.129, start - 0.024), (0.132, start - 0.030),
            (0.138, start - 0.032), (0.144, start - 0.029), (0.147, start - 0.021))]
        runs.append(("skirt_" + side, loop, 0.003))
    return runs


def hanger_meshes(assets):
    # A convex, round-ended strip for each span. Compound collision preserves every opening.
    specs = []
    for kind, points, radius in profile():
        for i, (a, b) in enumerate(zip(points, points[1:])):
            a, b = np.array(a), np.array(b)
            direction = b - a
            angle = math.atan2(direction[1], direction[0])
            outline = []
            for end, base in ((b, angle - math.pi / 2), (a, angle + math.pi / 2)):
                for theta in np.linspace(base, base + math.pi, 7):
                    outline.append(end + radius * np.array([math.cos(theta), math.sin(theta)]))
            mid = (a + b) / 2
            vertices = [(x - mid[0], y, z - mid[1]) for y in (-THICKNESS/2, THICKNESS/2)
                        for x, z in outline]
            name = f"{kind}_{i}"
            ET.SubElement(assets, "mesh", name=name, vertex=numbers(np.asarray(vertices).ravel()))
            specs.append((name, (mid[0], 0, mid[1]), np.linalg.norm(direction) * 2 * radius))
    return specs


def look_at(world, name, eye, target, fovy):
    forward = np.subtract(target, eye)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    ET.SubElement(world, "camera", name=name, pos=numbers(eye),
                  xyaxes=numbers((*right, *np.cross(right, forward))), fovy=str(fovy))


def build(near_miss=False):
    root = ET.fromstring("""<mujoco model="Crossed clothes hangers">
      <compiler angle="radian" autolimits="true"/>
      <option timestep=".002" integrator="implicitfast" cone="elliptic" iterations="60"/>
      <visual><global offwidth="960" offheight="540"/><quality shadowsize="1536"/>
       <headlight ambient=".48 .48 .48" diffuse=".32 .32 .32" specular=".09 .09 .09"/>
       <map znear=".004"/></visual>
      <default><geom friction=".55 .002 .0001" solref=".005 1" solimp=".96 .995 .0005"/></default>
      <asset><material name="plastic" rgba=".115 .12 .13 1" specular=".38" shininess=".42"/>
       <material name="wood" rgba=".74 .65 .52 1"/>
       <material name="tube" rgba=".63 .68 .71 1" specular=".55" shininess=".65"/>
       <texture name="sky" type="skybox" builtin="gradient" rgb1=".71 .80 .85" rgb2=".94 .94 .92"
          width="128" height="768"/></asset>
      <worldbody/><contact/><actuator/></mujoco>""")
    world, assets = root.find("worldbody"), root.find("asset")
    ET.SubElement(world, "geom", name="table", type="box", pos=numbers((.30, 0, TABLE-.025)),
                  size=".55 .55 .025", material="wood")
    ET.SubElement(world, "geom", name="floor", type="plane", pos=numbers((0, 0, station.FLOOR_Z)),
                  size="0 0 .05", rgba=".68 .69 .68 1")
    ET.SubElement(world, "geom", name="rail", type="cylinder", material="tube",
                  size=numbers((RAIL_RADIUS, .412)), pos=numbers((RAIL_X, 0, RAIL_Z)),
                  zaxis="0 1 0", friction=".35 .001 .0001")
    for side, sign in (("left", 1), ("right", -1)):
        ET.SubElement(world, "geom", name="upright_" + side, type="box", material="wood",
                      pos=numbers((RAIL_X, sign*.421, TABLE+.2325)), size=".050 .009 .2325")
    specs = hanger_meshes(assets)
    weight = sum(s[2] for s in specs)
    for name, (pos, orientation), pigment, tape in zip(NAMES, initial_poses(near_miss), FRAME_COLORS, TAPE_COLORS):
        body = ET.SubElement(world, "body", name=name, pos=numbers(pos), quat=quat(orientation))
        ET.SubElement(body, "freejoint", name=name)
        for part, offset, volume in specs:
            ET.SubElement(body, "geom", name=name+"_"+part, type="mesh", mesh=part,
                          material="plastic", rgba=pigment, pos=numbers(offset), mass=str(MASS*volume/weight))
        # Two visible tape bands mark the bar midpoint for camera-based pickup.
        for j, u in enumerate((-.075, .075)):
            ET.SubElement(body, "geom", name=f"{name}_tape_{j}", type="box", pos=numbers((u, 0, 0)),
                          size=".009 .00212 .00462", rgba=tape, contype="0", conaffinity="0", mass="0")
    ET.SubElement(world, "light", pos="-.25 -.60 2.1", dir=".35 .40 -1", directional="true",
                  diffuse=".52 .52 .50", specular=".15 .15 .15")
    look_at(world, "overview", (.92, -.72, TABLE+.72), (.31, .0, TABLE+.155), 43)
    look_at(world, "setup_view", (.82, -.025, TABLE+.80), (.32, 0., TABLE+.012), 43)
    look_at(world, "crossing_view", (.56, -.30, TABLE+.260), (.390, -.083, TABLE+.018), 34)
    add_arms(root)
    return ET.tostring(root, encoding="unicode")
