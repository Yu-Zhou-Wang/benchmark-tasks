"""Read PhysX poses and contact forces at each 120 Hz post-step event."""

import json
import os
from pathlib import Path

import numpy as np
import omni.physics.tensors as tensors
from pxr import Usd, UsdPhysics, UsdUtils

from isaaclab.managers import RecorderTerm

from geometry import CONTROL_HZ, PARTS, PHYSICS_HZ
from scoring import Score


TICK_FIELDS = (
    "gap_mm", "rim_gap_min_mm", "rim_gap_max_mm", "coaxial_mm", "relative_tilt_deg",
    "roll_deg", "from_vertical_deg", "cap_above_rests_mm", "cap_external_force_N",
    "barrel_only_cap_down", "seated", "two_grasps", "bimanual_seat", "hold_seconds", "credited",
)


class RecapRecorder(RecorderTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._env.recap = self
        self.active = False
        self.result = Score()

    def start(self):
        env = self._env
        stage = env.sim.stage
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.view = tensors.create_simulation_view("torch", stage_id=stage_id)
        view = self.view
        prefix, = env.scene.env_prim_paths
        part_paths = [f"{prefix}/{tool.part_name(cap)}" for tool, cap in PARTS]
        prims = list(stage.Traverse(Usd.TraverseInstanceProxies()))
        links = []
        for side in ("LeftArm", "RightArm"):
            tips = ("tip_left", "tip_right")
            found = {
                str(p.GetName()): str(p.GetPath())
                for p in prims if str(p.GetPath()).startswith(f"{prefix}/{side}/")
                and p.HasAPI(UsdPhysics.RigidBodyAPI)
            }
            if not set(tips) <= found.keys():
                raise RuntimeError(f"{side} fingertip links missing: {sorted(found)}")
            links.extend(found[k] for k in tips)

        def is_static_collider(prim):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                return False
            while prim and not prim.IsPseudoRoot():
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    return False
                prim = prim.GetParent()
            return True

        other_links = [
            str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)
            and str(p.GetPath()) not in {*part_paths, *links}
        ]
        static = [str(p.GetPath()) for p in prims if is_static_collider(p)]
        self.filters = [*links, *part_paths, *other_links, *static]
        self.bodies = view.create_rigid_body_view(part_paths)
        self.contacts = view.create_rigid_contact_view(part_paths, filter_patterns=[self.filters] * len(PARTS))
        if (self.bodies.count != len(PARTS) or self.contacts.sensor_count != len(PARTS)
                or self.contacts.filter_count != len(self.filters)):
            raise RuntimeError("PhysX writing-tool pose and contact views failed to initialize.")
        if list(map(str, self.bodies.prim_paths)) != part_paths:
            raise RuntimeError("Unexpected PhysX body order.")
        self.result = Score()
        self.label = ""
        self.tick_log = []
        self.active = True
        out = os.environ.get("RECAP_TRACE")
        self.trace = Path(out).resolve() if out else None
        if self.trace is not None:
            self.trace.parent.mkdir(parents=True, exist_ok=True)
            self.trace.write_text(json.dumps({
                "action": 0, "poses": self.read_poses().tolist(),
                "parts": [tool.part_name(cap) for tool, cap in PARTS],
                "filters": self.filters, "physics_hz": PHYSICS_HZ,
                "control_hz": CONTROL_HZ, "tick_fields": TICK_FIELDS,
                "device": str(env.device), "control": os.environ.get("RECAP_CONTROL", "normal"),
                "physical_gpu": int(os.environ["ROBOCURVE_GPU"]) if "ROBOCURVE_GPU" in os.environ else None,
            }) + "\n")

    def stop(self):
        self.active = False

    def read_poses(self):
        return self.bodies.get_transforms().cpu().numpy().astype(np.float64)

    def record_post_physics_decimation_step(self):
        if self.active:
            self.poses = self.read_poses()
            self.forces = self.contacts.get_contact_force_matrix(1 / PHYSICS_HZ).cpu().numpy()
            self.result.sample(self.poses, self.forces)
            if self.trace is not None:
                self.tick_log.append([
                    self.result.ticks,
                    [[m[field] for field in TICK_FIELDS] for m in self.result.metrics.values()],
                ])
        return None, None

    def record_action(self, count, joint_pos):
        if self.result.ticks != count * (PHYSICS_HZ // CONTROL_HZ):
            raise RuntimeError("Expected six physical substeps per writing-tool action.")
        if self.trace is not None:
            row = dict(
                action=count, stage=self.label, score=self.result.score, tools=self.result.metrics,
                poses=self.poses.tolist(), joint_pos=np.asarray(joint_pos).tolist(), ticks=self.tick_log,
            )
            with self.trace.open("a") as out:
                out.write(json.dumps(row, separators=(",", ":")) + "\n")
            self.tick_log.clear()
