"""One fixed layout, in metres; all patient details are fictional."""
from dataclasses import dataclass

TOP = 0.7475
SIZE = (0.060, 0.044, 0.040)
MASS = 0.080
TRAY_SIZE = (0.136, 0.120)
FLOOR_THICKNESS = 0.003
WALL_THICKNESS = 0.004
WALL_HEIGHT = 0.010
FLOOR_Z = TOP + FLOOR_THICKNESS
TRAYS = {"PROCESS": (0.320, 0.320), "QUARANTINE": (0.160, 0.320)}
HOLD_S = 2.0
CLEARANCE = 0.015


@dataclass(frozen=True)
class Specimen:
    key: str
    name: str
    sample_id: str
    request_name: str
    request_id: str
    xy: tuple[float, float]

    @property
    def destination(self):
        return "PROCESS" if (self.name, self.sample_id) == (self.request_name, self.request_id) else "QUARANTINE"


SPECIMENS = (
    Specimen("specimen_a", "A. LEE", "S17", "A. LEE", "S17", (0.160, 0.165)),
    Specimen("specimen_b", "B. RAY", "S42", "B. RAY", "S24", (0.320, 0.165)),
)
INSTRUCTION = (
    "All names and IDs on this simulated training bench are fictional. "
    "Check each closed specimen container against the request card beside it. "
    "Both the patient name and sample ID must match. Put matching specimens in the PROCESS tray "
    "and any specimen with a mismatch in the QUARANTINE tray. Keep the containers upright and closed. "
    "Release both containers and move both hands clear."
)
