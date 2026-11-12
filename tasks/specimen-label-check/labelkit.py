"""Task-local printed surfaces and read-only scoring hooks for YAM Arena.

The cube's original collision model, mass and robot controller are preserved.
The printed plane is visual geometry parented to its physical object.
Adapted from the scene-hook pattern in tasks/lock-before-loading/scenekit.py.
"""
from __future__ import annotations

import importlib.abc
import importlib.util
from pathlib import Path
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box

HERE = Path(__file__).resolve().parent
UPDATE = None
RESET = None
REPORT = None
_LOCAL_BOUNDS = {}


@dataclass(frozen=True)
class PrintedBox(Box):
    texture: str = ""
    print_size: tuple[float, float] | None = None
    print_offset: tuple[float, float] = (0., 0.)

    def _check(self):
        super()._check()
        if not (HERE / self.texture).is_file():
            raise FileNotFoundError(HERE / self.texture)


def _print_surface(prim, spec):
    from pxr import Sdf, UsdGeom, UsdShade, Vt
    stage = prim.GetStage()
    # The spawned cuboid has a unit-scale root; geometry scaling is on its child mesh.
    root = str(prim.GetPath())
    mesh = UsdGeom.Mesh.Define(stage, root + "/printed_label")
    sx, sy = spec.print_size or (spec.size[0]*0.94, spec.size[1]*0.94)
    x, y = spec.print_offset
    z = spec.size[2]/2 + 0.0002
    mesh.CreatePointsAttr([(x-sx/2,y-sy/2,z), (x+sx/2,y-sy/2,z),
                           (x+sx/2,y+sy/2,z), (x-sx/2,y+sy/2,z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    # Looking from the arms: text runs left to right along -y; its top is +x.
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray([(1,0), (1,1), (0,1), (0,0)]))
    material = UsdShade.Material.Define(stage, root + "/label_material")
    surface = UsdShade.Shader.Define(stage, root + "/label_material/surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    texture = UsdShade.Shader.Define(stage, root + "/label_material/texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(HERE / spec.texture)))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    reader = UsdShade.Shader.Define(stage, root + "/label_material/uv")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    if spec.free:
        # A dark printed seam marks the sealed lid; all physical dimensions and
        # collision faces remain the original closed rectangular envelope.
        seam = UsdGeom.Mesh.Define(stage, root + "/lid_seam")
        a,b = spec.size[0]/2+0.00005, spec.size[1]/2+0.00005
        vertices=[]
        for p,q in (((-a,-b),(a,-b)),((a,-b),(a,b)),((a,b),(-a,b)),((-a,b),(-a,-b))):
            vertices.extend([(p[0],p[1],z-0.004),(q[0],q[1],z-0.004),
                             (q[0],q[1],z-0.003),(p[0],p[1],z-0.003)])
        seam.CreatePointsAttr(vertices)
        seam.CreateFaceVertexCountsAttr([4]*4)
        seam.CreateFaceVertexIndicesAttr(list(range(16)))
        seam.CreateSubdivisionSchemeAttr("none")
        seam.CreateDoubleSidedAttr(True)
        seam.CreateDisplayColorAttr([(0.10,0.10,0.10)])


def _patch_scene(module):
    original = module.task_object

    def task_object(spec):
        obj = original(spec)
        if not isinstance(spec, PrintedBox):
            return obj
        class PrintedObject(type(obj)):
            def _get_spawn_cfg(self, activate_contact_sensors=False):
                cfg = super()._get_spawn_cfg(activate_contact_sensors).copy()
                spawn = cfg.func

                def spawn_with_print(*args, **kwargs):
                    prim = spawn(*args, **kwargs)
                    _print_surface(prim, spec)
                    return prim

                cfg.func = spawn_with_print
                return cfg

        # Object builds its asset config during construction, so the override
        # must exist before __init__, including for fixed request cards.
        return PrintedObject(name=obj.name, object_type=obj.object_type,
                             spawner_cfg=obj.spawner_cfg, initial_pose=obj.initial_pose)

    module.task_object = task_object


def _np(value):
    value = getattr(value, "torch", value)
    return value.detach().cpu().numpy().astype(float)


def _robot_bounds(environment):
    """Conservative link AABBs, transformed by current physics poses, both arms."""
    from yam_arena import require_usd
    from scoring import rotation
    if not _LOCAL_BOUNDS:
        from collision_bounds import read_link_corners
        names = environment.unwrapped.scene["left_robot"].body_names
        _LOCAL_BOUNDS.update(read_link_corners(require_usd(),names))
    base = environment.unwrapped
    origin = _np(base.scene.env_origins)[0]
    bounds = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        poses = _np(robot.data.body_link_pose_w)[0]
        for name, pose in zip(robot.body_names, poses, strict=True):
            corners = _LOCAL_BOUNDS[name] @ rotation(pose[3:7]).T + pose[:3] - origin
            bounds.append([corners.min(axis=0), corners.max(axis=0)])
    return np.asarray(bounds)


def _patch_embodiment(module):
    cls = module.YamArenaEmbodiment
    reset, step = cls.reset, cls._task_step

    def reset_with_monitor(self, *args, **kwargs):
        obs = reset(self, *args, **kwargs)
        if RESET:
            RESET()
        return obs

    def step_with_monitor(self, *args, **kwargs):
        if UPDATE:
            from yam_arena.scene import object_states
            UPDATE(object_states(self._env, self.task.names), _robot_bounds(self._env), self._base.step_dt)
        result = step(self, *args, **kwargs)
        if REPORT:
            result.info["routing"] = REPORT()
        return result

    cls.reset, cls._task_step = reset_with_monitor, step_with_monitor


def _patch_cameras(module):
    original = module.BimanualYamEmbodiment.__init__

    def init(self, *args, **kwargs):
        kwargs["cam_width"], kwargs["cam_height"] = 1280, 720
        original(self, *args, **kwargs)

    module.BimanualYamEmbodiment.__init__ = init


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
               "yam_arena.embodiment": _patch_cameras}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]

        class Loader(importlib.abc.Loader):
            def create_module(self, spec):
                return loader.create_module(spec)

            def exec_module(self, module):
                loader.exec_module(module)
                patch(module)

        spec.loader = Loader()
        return spec


_finder = _Finder()
for _name, _patch in _finder.targets.items():
    if _name in sys.modules:
        _patch(sys.modules[_name])
sys.meta_path.insert(0, _finder)
