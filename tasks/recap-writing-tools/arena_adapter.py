"""Task-local integration with the pinned Arena YAM task-file entry point.

The stock adapter handles actions, reset, and public observations. These
hooks add posed compound USD bodies, a post-physics recorder, and this file's
Cartesian scripted controller. Other Arena task files keep their usual paths.
"""

from dataclasses import replace
from functools import wraps
import importlib.abc
import importlib.util
import math
import sys

from geometry import ACTION_LIMIT, CONTROL_HZ, PHYSICS_HZ


def handles(task):
    return task is not None and getattr(task.module, "RECAP_WRITING_TOOLS", False)


def build_environment(embodiment, task, control_hz):
    from isaaclab.managers import RecorderTermCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
    from yam_arena import scene

    from physics_monitor import RecapRecorder

    if control_hz != CONTROL_HZ:
        raise ValueError(f"Recap three writing tools requires -E control_hz={CONTROL_HZ}")
    for side in ("left_robot", "right_robot"):
        spawn = getattr(embodiment.scene_config, side).spawn
        spawn.collision_props = PhysxCollisionPropertiesCfg(contact_offset=0.0005, rest_offset=0.)
        spawn.make_uninstanceable = True

    def configure(cfg):
        cfg = scene.env_cfg_callback(control_hz)(cfg)
        cfg.sim.dt = 1 / PHYSICS_HZ
        cfg.decimation = PHYSICS_HZ // CONTROL_HZ
        cfg.sim.render_interval = cfg.decimation
        cfg.recorders.recap = RecorderTermCfg(class_type=RecapRecorder)
        return cfg

    props = [
        Object(
            name=obj.name, object_type=ObjectType.RIGID if obj.free else ObjectType.BASE,
            usd_path=str(task.path.parent / "assets" / obj.usd),
            initial_pose=Pose(position_xyz=tuple(obj.pos), rotation_xyzw=tuple(obj.rot)),
        )
        for obj in task.objects
    ]
    return IsaacLabArenaEnvironment(
        name="yam_bimanual_recap_writing_tools",
        scene=Scene([*scene.scene_assets(objects=False, d435_collider=False), *props]),
        embodiment=embodiment,
        env_cfg_callback=configure,
    )


def patch_scene(module):
    original = module.build_task_environment

    @wraps(original)
    def build(embodiment, task, control_hz=None):
        if handles(task):
            return build_environment(embodiment, task, control_hz)
        return original(embodiment, task, control_hz)

    module.build_task_environment = build


class SceneImport(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        load = spec.loader.exec_module

        def load_and_patch(module):
            load(module)
            patch_scene(module)

        spec.loader.exec_module = load_and_patch
        return spec


def install():
    from inspect_robots.types import StepResult
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    from yam_arena.rig import Camera, Lens
    from yam_arena.scripted import YamScriptedPolicy

    if getattr(YamArenaEmbodiment, "_recap_adapter_installed", False):
        return
    if "yam_arena.scene" in sys.modules:
        patch_scene(sys.modules["yam_arena.scene"])
    else:
        sys.meta_path.insert(0, SceneImport())

    reset = YamArenaEmbodiment.reset
    step = YamArenaEmbodiment.step
    task_step = YamArenaEmbodiment._task_step
    policy_reset, policy_act = YamScriptedPolicy.reset, YamScriptedPolicy.act

    @wraps(reset)
    def recap_reset(self, scene, *, seed=None):
        if not handles(self.task):
            return reset(self, scene, seed=seed)
        if not self.device.startswith("cuda") or self.object_jitter:
            raise ValueError("Use GPU PhysX and -E object_jitter=0 for the fixed reset.")
        self.rig = replace(self.rig, top_cam=Camera(
            (0.280, 0., 1.475), (0., math.sqrt(0.5), 0., math.sqrt(0.5)), "world", Lens.from_focal(15.0)
        ))
        if self._base is not None:
            self._base.recap.stop()
        obs = reset(self, scene, seed=seed)
        self._base.recap.start()
        self._recap_actions = 0
        return obs

    @wraps(step)
    def recap_step(self, action):
        if handles(self.task):
            self._base.recap.label = action.meta.get("stage", "")
        return step(self, action)

    @wraps(task_step)
    def recap_outcome(self):
        if not handles(self.task):
            return task_step(self)
        self._recap_actions += 1
        recorder = self._base.recap
        score = float(recorder.result.score)
        success = score == 1.
        ended = self._recap_actions >= ACTION_LIMIT
        obs = self._observation()
        recorder.record_action(self._recap_actions, obs.state["joint_pos"])
        return StepResult(
            observation=obs, terminated=success, truncated=ended and not success,
            termination_reason="success" if success else "action_limit" if ended else None,
            info={"score": score, "success": success},
        )

    @wraps(policy_reset)
    def recap_policy_reset(self, scene):
        if handles(self._task):
            self._recap_controller = self._task.demo()
            return None
        return policy_reset(self, scene)

    @wraps(policy_act)
    def recap_policy_act(self, observation):
        if handles(self._task):
            return self._recap_controller.act(observation)
        return policy_act(self, observation)

    YamArenaEmbodiment.reset = recap_reset
    YamArenaEmbodiment.step = recap_step
    YamArenaEmbodiment._task_step = recap_outcome
    YamScriptedPolicy.reset = recap_policy_reset
    YamScriptedPolicy.act = recap_policy_act
    YamArenaEmbodiment._recap_adapter_installed = True
