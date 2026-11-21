"""Dimensions, reset poses and matching split-line geometry for the writing tools."""

from dataclasses import dataclass
import math

import numpy as np


MM = 0.001
PHYSICS_HZ = 120
CONTROL_HZ = 20
ACTION_LIMIT = 1200
TABLE_Z = 0.750
REST_Z = TABLE_Z + 0.035
LANE_Y = 0.100
RESET_CLEARANCE = 0.0001
BARREL_GRASP = 0.092
BARREL_LENGTH = 120.0
BARREL_MASS = 0.015
CAP_WALL = 2.35             # exterior expansion from the nominal land, mm
BARREL_SHOULDER = 2.15      # barrel expansion from the nominal land, mm
STIFFNESS = 1600.0           # N/m
DAMPING = 1.0                # N s/m
FRICTION = 0.45
CONTACT_OFFSET = 0.00025
PICKUP_FRAME = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
QUARTER_TURN = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])


@dataclass(frozen=True)
class Tool:
    name: str
    title: str
    pair_color: tuple[float, float, float]
    barrel_color: tuple[float, float, float]
    pick_x: float
    land_x: float
    land_y: float
    mouth_z: float
    shoulder_z: float
    cap_mass: float
    overlap: float
    sides: int = 16
    rectangular: bool = False

    @property
    def cap_center(self):
        return self.mouth_z * MM / 2

    @property
    def cap_grasp(self):
        # Pinch below the midpoint of the two shorter caps so the upper
        # edges of the fingertips clear the barrel shoulder during seating.
        offset = {"pen": 4.5, "marker": 0., "highlighter": 4.0}[self.name]
        return self.cap_center - offset * MM

    def reset_rotation(self, cap):
        if self.rectangular and cap:
            return PICKUP_FRAME @ QUARTER_TURN
        return PICKUP_FRAME

    def grasp_width(self, cap):
        span = self.land_y if self.rectangular and cap else self.land_x
        return 2 * (span + (CAP_WALL if cap else BARREL_SHOULDER)) * MM

    def thickness(self, cap):
        span = self.land_x if self.rectangular and cap else self.land_y
        return 2 * (span + (CAP_WALL if cap else BARREL_SHOULDER)) * MM

    def reset_origin(self, cap):
        return (
            self.pick_x - (self.cap_center if cap else BARREL_GRASP),
            -LANE_Y if cap else LANE_Y,
            REST_Z + self.thickness(cap) / 2 + RESET_CLEARANCE,
        )

    def grasp_position(self, cap):
        station = self.cap_grasp if cap else BARREL_GRASP
        return np.array(self.reset_origin(cap)) + [station, 0., 0.]

    def part_name(self, cap):
        return f"{self.name}_{'cap' if cap else 'barrel'}"


TOOLS = (
    Tool("pen", "Slim pen", (0.18, 0.48, 0.96), (0.15, 0.31, 0.55), 0.180,
         4.1, 4.1, 25., 21., 0.0015, 0.18, sides=8),
    Tool("marker", "Whiteboard marker", (0.74, 0.17, 0.35), (0.94, 0.88, 0.79), 0.300,
         6.2, 6.2, 32., 27., 0.0033, 0.30),
    Tool("highlighter", "Rectangular highlighter", (0.92, 0.69, 0.08), (0.96, 0.73, 0.13), 0.420,
         10.2, 5.6, 29., 24., 0.0032, 0.18, rectangular=True),
)
PARTS = tuple((tool, cap) for tool in TOOLS for cap in (False, True))


def outline(tool, expansion=0.0, scale=1.0):
    """Counterclockwise cross-section in metres, with the probe's flat faces."""
    x = (tool.land_x * scale + expansion) * MM
    y = (tool.land_y * scale + expansion) * MM
    if tool.rectangular:
        chamfer = min(x, y) * 0.22
        return [
            (x, -y + chamfer), (x, y - chamfer), (x - chamfer, y), (-x + chamfer, y),
            (-x, y - chamfer), (-x, -y + chamfer), (-x + chamfer, -y), (x - chamfer, -y),
        ]
    angle = math.pi / tool.sides
    return [
        (x * math.cos((2 * i - 1) * angle) / math.cos(angle),
         y * math.sin((2 * i - 1) * angle) / math.cos(angle))
        for i in range(tool.sides)
    ]


def cap_rim(tool):
    return np.array([(x, y, tool.mouth_z * MM) for x, y in outline(tool, CAP_WALL)])


def cap_vertices(tool):
    ring = cap_rim(tool)
    return np.concatenate((ring, ring - [0, 0, tool.mouth_z * MM]))
