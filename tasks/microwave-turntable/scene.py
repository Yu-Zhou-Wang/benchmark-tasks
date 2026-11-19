"""Unheated turntable fixture, roller ring, transparent tray and mug. Dimensions in metres."""

import math
import xml.etree.ElementTree as ET

import numpy as np
from yam_arena import station
from station_model import add_arms, numbers

TABLE = station.TABLE_TOP
HUB = np.array([0.525, 0.0])
FRONT = 0.359
FLOOR = TABLE + 0.115
HINGE = np.array([0.340, 0.161])
OPEN = math.radians(-170)
RING_START = np.array([0.202, 0.136])
TRAY_START = np.array([0.226, -0.085])
MUG_START = np.array([0.365, -0.322])
RING_R = 0.070
TRAY_R = 0.115
RING_PICK_Z = TABLE + 0.087
TRAY_PICK_Z = TABLE + 0.093
MUG_R = 0.031
MOTOR_SPEED = math.tau / 10
BUTTON = np.array([FRONT - 0.009, -0.205, FLOOR + 0.059])
OFFSET = np.array([-0.063, -0.016])


def geom(parent, name, shape, pos, size, mat, **kw):
    attrs = dict(name=name, type=shape, pos=numbers(pos), size=numbers(size), material=mat)
    return ET.SubElement(parent, "geom", **attrs, **kw)


def decor(parent, name, shape, pos, size, mat, **kw):
    return geom(parent, name, shape, pos, size, mat, contype="0", conaffinity="0", mass="0", **kw)


def circ(parent, name, radius, z, tube, mat, segments=32, visual=False):
    make = decor if visual else geom
    for i in range(segments):
        a, b = np.array([i, i + 1]) * math.tau / segments
        ends = (
            radius * math.cos(a),
            radius * math.sin(a),
            z,
            radius * math.cos(b),
            radius * math.sin(b),
            z,
        )
        make(parent, f"{name}_{i}", "capsule", (0, 0, 0), (tube,), mat, fromto=numbers(ends))


def cabinet(w, drives):
    steel = ET.SubElement(w, "body", name="cabinet", pos=numbers((*HUB, FLOOR)))
    # The clear polycarbonate guard encloses an independent low-voltage drive.
    geom(steel, "oven_floor", "box", (0, -0.026, -0.014), (0.166, 0.196, 0.014), "enamel")
    for label, radius in (("inner_track", 0.057), ("outer_track", 0.084)):
        circ(steel, label, radius, 0, 0.0038, "enamel", 48)
    geom(steel, "roof_window", "box", (0, -0.026, 0.222), (0.166, 0.196, 0.002), "clear")
    geom(steel, "back_window", "box", (0.159, -0.026, 0.108), (0.002, 0.196, 0.108), "clear")
    geom(steel, "side_window", "box", (0, 0.156, 0.108), (0.163, 0.002, 0.108), "clear")
    geom(steel, "right_guard", "box", (0, -0.154, 0.108), (0.163, 0.007, 0.108), "enamel")
    geom(steel, "control_housing", "box", (0.001, -0.19, 0.108), (0.165, 0.036, 0.108), "metal")
    geom(steel, "fascia", "box", (-0.162, -0.19, 0.108), (0.006, 0.036, 0.111), "graphite")
    geom(steel, "counter_riser", "box", (0, -0.026, -0.077), (0.164, 0.191, 0.038), "counter")
    for dx in (-0.157, 0.157):
        for dy in (-0.151, 0.154):
            geom(steel, f"post_{dx}_{dy}", "box", (dx, dy, 0.110), (0.003, 0.003, 0.110), "metal")
    for dx in (-0.125, 0.125):
        for dy in (-0.178, 0.124):
            geom(steel, f"foot_{dx}_{dy}", "cylinder", (dx, dy, -0.033), (0.014, 0.006), "rubber")

    # Printed floor template and roof reticle share the drive axis in the plan view.
    for radius, ink in ((58, "ring_limit"), (82, "ring_limit"), (107, "navy"), (117, "navy"),
                        (35, "load_ink"), (78, "load_ink")):
        z = 0.004 if radius in (58, 82) else 0.0004
        circ(steel, f"template_{radius}", radius / 1000, z, 0.00065, ink, 64, visual=True)
    circ(steel, "center_target", 0.005, 0.2243, 0.00055, "navy", 24, visual=True)
    for i in range(4):
        a = i * math.pi / 2
        c, s = math.cos(a), math.sin(a)
        decor(steel, f"quadrant_{i}", "box", (0.129*c, 0.129*s, 0.0006),
              (0.009, 0.0020, 0.0005), "navy", euler=numbers((0, 0, a)))
    # Identical vertical rulers in the back and left elevations. Bars show the
    # entire allowed height interval. Minor ticks are spaced 5 mm apart.
    for side in ("back", "left"):
        ruler_pos = (0.147, -0.125) if side == "back" else (-0.125, 0.146)
        ruler_size = (0.0004, 0.016, 0.052) if side == "back" else (0.016, 0.0004, 0.052)
        decor(steel, f"{side}_ruler", "box", (*ruler_pos, 0.050), ruler_size, "enamel")
        def bar(name, z, width, height, ink):
            xy = (0.148, -0.125) if side == "back" else (-0.125, 0.147)
            size = (0.0005, width/2, height/2) if side == "back" else (width/2, 0.0005, height/2)
            decor(steel, f"{side}_{name}", "box", (*xy, z), size, ink)
        for mm in range(0, 101, 5):
            bar(f"tick_{mm}", mm/1000, 0.028 if mm % 10 == 0 else 0.020, 0.00065, "navy")
        bar("axle_band", 0.010, 0.014, 0.006, "ring_ink")
        bar("tray_band", 0.024, 0.014, 0.005, "stripe")
    # Two faces of the same motor lamp let the plan and operator see power.
    for name, pos, size in (
        ("lamp_face", (-0.169, -0.184, 0.112), (0.001, 0.012, 0.007)),
        ("lamp_plan", (-0.131, -0.169, 0.218), (0.010, 0.010, 0.001)),
    ):
        decor(steel, name + "_bezel", "box", pos, size, "graphite")
        loc = np.array(pos) + ([-0.0015, 0, 0] if name == "lamp_face" else [0, 0, 0.0015])
        lens = np.array(size) * 0.74
        ET.SubElement(steel, "geom", name=name, type="box", pos=numbers(loc), size=numbers(lens),
                      rgba=".12 .15 .13 1", contype="0", conaffinity="0", mass="0")
    # Latch end crosses this 10 mm witness strip at door closure.
    decor(steel, "latch_bracket", "box", (-0.178, -0.167, 0.219), (0.022, 0.015, 0.002), "graphite")
    decor(steel, "latch_target", "box", (-0.185, -0.159, 0.222), (0.010, 0.006, 0.0006), "load_ink")
    panel = ET.SubElement(w, "body", name="start_button", pos=numbers(BUTTON))
    ET.SubElement(
        panel,
        "joint",
        name="button",
        type="slide",
        axis="1 0 0",
        range="0 .005",
        stiffness="110",
        damping="1.0",
        armature=".002",
    )
    geom(panel, "start", "box", (0, 0, 0), (0.004, 0.018, 0.010), "teal", mass=".012")
    decor(panel, "start_mark", "ellipsoid", (-0.0041, 0, 0), (0.0004, 0.004, 0.003), "enamel")
    door = ET.SubElement(w, "body", name="door", pos=numbers((*HINGE, FLOOR)))
    ET.SubElement(
        door,
        "joint",
        name="hinge",
        type="hinge",
        axis="0 0 1",
        range=numbers((OPEN - 0.02, 0.009)),
        damping=".13",
        frictionloss=".095",
        armature=".003",
    )
    ET.SubElement(door, "inertial", mass=".68", pos="0 -.155 .105", diaginertia=".0082 .0028 .0055")
    geom(door, "window", "box", (0, -0.154, 0.109), (0.002, 0.130, 0.079), "clear")
    for name, pos, size in (
        ("sill", (0, -0.154, 0.016), (0.010, 0.155, 0.016)),
        ("top", (0, -0.154, 0.204), (0.010, 0.155, 0.017)),
        ("hinge_edge", (0, -0.011, 0.111), (0.010, 0.012, 0.081)),
        ("door_edge", (0, -0.298, 0.111), (0.010, 0.012, 0.081)),
    ):
        geom(door, name, "box", pos, size, "graphite")
    decor(door, "latch_mark", "box", (0, -0.300, 0.222), (0.002, 0.006, 0.0006), "stripe")
    for z in (0.050, 0.168):
        geom(
            door,
            f"handle_mount_{z}",
            "capsule",
            (0, 0, 0),
            (0.007,),
            "metal",
            fromto=numbers((-0.010, -0.290, z, -0.035, -0.290, z)),
        )
    geom(door, "handle", "capsule", (0, 0, 0), (0.009,), "metal", fromto="-.035 -.29 .05 -.035 -.29 .168")
    shaft = ET.SubElement(w, "body", name="spindle", pos=numbers((*HUB, FLOOR)))
    ET.SubElement(
        shaft, "joint", name="shaft", type="hinge", axis="0 0 1", damping=".002", armature=".0005"
    )
    geom(shaft, "hub_base", "cylinder", (0, 0, 0.0025), (0.023, 0.0025), "coupler")
    for i in range(3):
        a = i * math.tau / 3
        geom(
            shaft,
            f"drive_pin_{i}",
            "cylinder",
            (0.016 * math.cos(a), 0.016 * math.sin(a), 0.010),
            (0.004, 0.006),
            "coupler",
        )
    ET.SubElement(
        drives, "velocity", joint="shaft", name="turntable_motor", kv=".23", forcerange="-.20 .20"
    )


def cup_surface(assets):
    # A continuous glazed shell for rendering. The separate physical walls leave the cup open.
    profile = [
        (0.026, 0),
        (0.031, 0.004),
        (0.031, 0.062),
        (0.030, 0.0675),
        (0.028, 0.0695),
        (0.0255, 0.067),
        (0.0255, 0.008),
        (0.023, 0.006),
    ]
    vertices, faces = [], []
    count = 64
    for radius, z in profile:
        for i in range(count):
            a = math.tau * i / count
            vertices.extend((radius * math.cos(a), radius * math.sin(a), z))
    for j in range(len(profile) - 1):
        for i in range(count):
            a = j * count + i
            b = j * count + (i + 1) % count
            faces.extend((a, b, a + count, b, b + count, a + count))
    ET.SubElement(
        assets, "mesh", name="ceramic_shell", vertex=numbers(vertices), face=" ".join(map(str, faces))
    )


def loose_parts(w):
    # Three short silicone drying rests leave the front edge of the tray accessible.
    for i, angle in enumerate((0, 125, 235)):
        a = math.radians(angle)
        p = TRAY_START + 0.069 * np.array([math.cos(a), math.sin(a)])
        geom(
            w,
            f"drying_rest_{i}",
            "cylinder",
            (*p, (TRAY_PICK_Z - 0.004 + TABLE) / 2),
            (0.012, (TRAY_PICK_Z - 0.004 - TABLE) / 2),
            "rubber",
        )
    for i, angle in enumerate((0, 110, 250)):
        a = math.radians(angle)
        p = RING_START + RING_R * np.array([math.cos(a), math.sin(a)])
        h = RING_PICK_Z - TABLE - 0.0034
        geom(w, f"guide_rest_{i}", "cylinder", (*p, TABLE + h / 2), (0.012, h / 2), "rubber")
    ring = ET.SubElement(w, "body", name="roller_ring", pos=numbers((*RING_START, RING_PICK_Z)))
    ET.SubElement(ring, "freejoint", name="ring_pose")
    ET.SubElement(ring, "inertial", pos="0 0 0", mass=".044", diaginertia=".00012 .00012 .00022")
    circ(ring, "guide", RING_R, 0, 0.0034, "roller", 30)
    circ(ring, "axle_line", RING_R, 0, 0.0035, "ring_ink", 30, visual=True)
    for i in range(3):
        a = math.pi + i * math.tau / 3
        axis = np.array([math.cos(a), math.sin(a), 0.0])
        wheel = ET.SubElement(ring, "body", name=f"wheel_{i}", pos=numbers(axis * RING_R))
        ET.SubElement(wheel, "joint", name=f"axle_{i}", type="hinge", axis=numbers(axis), damping=".000004")
        geom(
            wheel,
            f"roller_{i}",
            "cylinder",
            (0, 0, 0),
            (0.010, 0.005),
            "wheel",
            zaxis=numbers(axis),
            mass=".004",
            friction=".45 .001 .0001",
        )
    tray = ET.SubElement(w, "body", name="tray", pos=numbers((*TRAY_START, TRAY_PICK_Z)))
    ET.SubElement(tray, "freejoint", name="tray_pose")
    ET.SubElement(tray, "inertial", pos="0 0 0", mass=".400", diaginertia=".00133 .00133 .00265")
    geom(tray, "glass_disk", "cylinder", (0, 0, 0), (TRAY_R, 0.004), "glass", friction=".55 .003 .0001")
    circ(tray, "tray_edge_mark", TRAY_R, 0, 0.0009, "navy", 72, visual=True)
    decor(tray, "tray_center", "cylinder", (0, 0, 0.0043), (0.0021, 0.00025), "stripe")
    decor(tray, "tray_stripe", "box", (0.105, 0, 0.0045), (0.010, 0.004, 0.0004), "stripe")
    decor(tray, "stripe_edge", "box", (0.1145, 0, 0), (0.0005, 0.004, 0.004), "stripe")
    # Recesses beneath the glass enclose each spindle pin on all four sides.
    for i in range(3):
        a = i * math.tau / 3
        tangent = np.array([-math.sin(a), math.cos(a)])
        radial = np.array([math.cos(a), math.sin(a)])
        for j, (dr, dt, sr, st) in enumerate(
            (
                (0, 0.009, 0.009, 0.002),
                (0, -0.009, 0.009, 0.002),
                (0.009, 0, 0.002, 0.008),
                (-0.009, 0, 0.002, 0.008),
            )
        ):
            p = (0.016 + dr) * radial + dt * tangent
            geom(
                tray,
                f"socket_{i}_{j}",
                "box",
                (*p, -0.008),
                (sr, st, 0.005),
                "edge",
                euler=numbers((0, 0, a)),
                friction=".45 .002 .0001",
            )
    mug = ET.SubElement(w, "body", name="mug", pos=numbers((*MUG_START, TABLE)))
    ET.SubElement(mug, "freejoint", name="mug_pose")
    ET.SubElement(mug, "inertial", pos="0 0 .032", mass=".19", diaginertia=".00015 .00016 .00010")
    geom(mug, "mug_bottom", "cylinder", (0, 0, 0.003), (MUG_R, 0.003), "glaze", friction=".75 .002 .0001")
    for i in range(24):
        a = i * math.tau / 24
        geom(
            mug,
            f"cup_wall_{i}",
            "box",
            (0.028 * math.cos(a), 0.028 * math.sin(a), 0.036),
            (0.003, 0.00385, 0.031),
            "glaze",
            euler=numbers((0, 0, a)),
            group="3",
        )
    ET.SubElement(
        mug,
        "geom",
        name="glazed_shell",
        mesh="ceramic_shell",
        type="mesh",
        material="glaze",
        contype="0",
        conaffinity="0",
        mass="0",
    )
    circ(mug, "cup_lip", 0.028, 0.067, 0.0031, "cream", 32, visual=True)
    decor(mug, "mug_dot", "cylinder", (0, 0, 0.007), (0.005, 0.00035), "yellow")
    circ(mug, "mug_foot", 0.026, 0.0006, 0.00075, "cream", 40, visual=True)
    # The open loop faces the back of the oven when the mug is set down.
    for i in range(10):
        a, b = np.array([i, i + 1]) * math.pi / 10
        ends = (
            0.027 + 0.023 * math.sin(a),
            0,
            0.037 + 0.020 * math.cos(a),
            0.027 + 0.023 * math.sin(b),
            0,
            0.037 + 0.020 * math.cos(b),
        )
        # Orient the handle into the three-quarter view.
        ends = (ends[0] * 0.67, -ends[0] * 0.742, ends[2], ends[3] * 0.67, -ends[3] * 0.742, ends[5])
        geom(mug, f"cup_handle_{i}", "capsule", (0, 0, 0), (0.0043,), "glaze", fromto=numbers(ends))


def build():
    root = ET.fromstring("""<mujoco model="Microwave turntable">
      <compiler angle="radian" autolimits="true"/>
      <option timestep=".002" integrator="implicitfast" cone="elliptic" iterations="40"/>
      <visual><global offwidth="960" offheight="576"/><quality shadowsize="1536"/>
        <headlight ambient=".48 .48 .48" diffuse=".20 .20 .20" specular=".09 .09 .09"/>
        <map znear=".006"/></visual>
      <default><geom friction=".65 .003 .0001" solref=".006 1" solimp=".94 .99 .001"/></default>
      <asset/><worldbody/><contact/><actuator/></mujoco>""")
    assets, w, drives = (root.find(n) for n in ("asset", "worldbody", "actuator"))
    colors = {
        "enamel": " .83 .84 .81 1",
        "metal": ".56 .60 .62 1",
        "graphite": ".105 .116 .13 1",
        "clear": ".77 .90 .94 .045",
        "navy": ".065 .105 .16 1",
        "stripe": ".02 .16 .93 1",
        "ring_ink": ".65 .07 .48 1",
        "ring_limit": ".38 .22 .39 1",
        "load_ink": ".02 .45 .33 1",
        "yellow": "1 .90 .04 1",
        "rubber": ".23 .25 .26 1",
        "teal": ".09 .43 .38 1",
        "glaze": ".88 .30 .13 1",
        "cream": ".97 .85 .71 1",
        "glass": ".64 .85 .88 .13",
        "edge": ".49 .68 .68 1",
        "coupler": ".42 .31 .22 1",
        "roller": ".40 .37 .33 1",
        "wheel": ".83 .74 .54 1",
        "counter": ".68 .63 .55 1",
        "tile": ".74 .77 .75 1",
    }
    for name, rgba in colors.items():
        ET.SubElement(
            assets,
            "material",
            name=name,
            rgba=rgba.strip(),
            specular=".58" if name in ("glass", "glaze", "metal") else ".21",
            shininess=".65" if name in ("glass", "glaze", "metal") else ".20",
        )
    ET.SubElement(
        assets,
        "texture",
        name="sky",
        type="skybox",
        builtin="gradient",
        rgb1=".78 .84 .87",
        rgb2=".93 .92 .88",
        width="256",
        height="1536",
    )
    # Neutral kitchen backsplash and counter run behind the working area.
    for row in range(5):
        for col in range(17):
            y = (col - 8) * 0.16 + (row % 2) * 0.08
            decor(
                w,
                f"tile_{row}_{col}",
                "box",
                (0.866, y, TABLE + 0.05 + row * 0.102),
                (0.009, 0.0796, 0.0506),
                "tile",
            )
    geom(w, "table", "box", (0.30, 0, TABLE - 0.025), (0.55, 0.55, 0.025), "counter")
    ET.SubElement(
        w,
        "light",
        pos="-.45 -.65 2",
        dir=".50 .40 -1",
        directional="true",
        diffuse=".39 .39 .38",
        ambient=".06 .06 .06",
        specular=".14 .14 .14",
        castshadow="false",
    )
    ET.SubElement(
        w,
        "light",
        pos=numbers((FRONT + 0.12, 0.10, FLOOR + 0.20)),
        dir=".4 -.3 -1",
        diffuse=".21 .20 .18",
        specular=".05 .05 .05",
        castshadow="false",
    )
    # Evidence camera looks through the front window and sees the loose parts.
    eye = np.array([-0.38, -0.052, TABLE + 0.50])
    target = np.array([0.32, -0.014, TABLE + 0.074])
    forward = (target - eye) / np.linalg.norm(target - eye)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    ET.SubElement(
        w,
        "camera",
        name="overview",
        pos=numbers(eye),
        xyaxes=numbers((*right, *np.cross(right, forward))),
        fovy="48",
    )
    # Independent inspection cameras through physical windows. Orthographic
    # elevations share one mm scale at every depth; the policy keeps its three cameras.
    for name, eye, axes, height in (
        ("plan", (HUB[0]-0.016, -0.010, FLOOR+0.40), "0 -1 0 1 0 0", 0.38),
        ("back", (HUB[0]+0.27, 0, FLOOR+0.048), "0 1 0 0 0 1", 0.12),
        ("side", (HUB[0], 0.34, FLOOR+0.048), "-1 0 0 0 0 1", 0.12),
    ):
        ET.SubElement(w, "camera", name=name, pos=numbers(eye), xyaxes=axes,
                      projection="orthographic", fovy=str(height))
    cup_surface(assets)
    cabinet(w, drives)
    loose_parts(w)
    add_arms(root)
    return ET.tostring(root, encoding="unicode")
