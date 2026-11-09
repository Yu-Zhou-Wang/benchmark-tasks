"""CPU-only convex collision audits; does not start or modify a simulation."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
from pxr import Usd, UsdGeom, UsdPhysics
from yam_arena import YAM_URDF
from yam_arena.urdf import forward_kinematics, axis_angle_matrix
from yam_arena.constants import TCP_OFFSET


def hull(name, vertices):
    v = np.asarray(vertices)
    return (name, v.min(0), v.max(0), np.unique(np.round(ConvexHull(v).equations, 10), axis=0))


def transform_hull(name, vertices, equations, rotation, position):
    v = vertices @ rotation.T + position
    n = equations[:, :3] @ rotation.T
    return (name, v.min(0), v.max(0), np.c_[n, equations[:, 3] - n @ position])


def intersections(first, second, depth=0.00005):
    result = set()
    for an, alo, ahi, ae in first:
        for bn, blo, bhi, be in second:
            if (an, bn) in result or not np.all(np.minimum(ahi, bhi)-np.maximum(alo, blo) > 2*depth):
                continue
            e = np.vstack([ae, be])
            answer = linprog(np.zeros(3), A_ub=e[:, :3], b_ub=-e[:, 3]-depth,
                             bounds=[(None, None)]*3, method="highs")
            if answer.success:
                result.add((an, bn))
    return sorted(result)


class Geometry:
    def __init__(self):
        self.robot = []
        root = ET.parse(YAM_URDF).getroot()
        for link in root.findall("link"):
            name = link.get("name")
            if name not in ("link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right"):
                continue
            for shape in link.findall("collision"):
                mesh = shape.find("geometry/mesh")
                if mesh is None:
                    continue
                m = trimesh.load_mesh(Path(YAM_URDF).parent/mesh.get("filename"), process=False)
                o = shape.find("origin")
                xyz = np.fromstring(o.get("xyz", "0 0 0"), sep=" ")
                rpy = np.fromstring(o.get("rpy", "0 0 0"), sep=" ")
                r = (axis_angle_matrix(np.array([0.,0,1]),rpy[2]) @
                     axis_angle_matrix(np.array([0.,1,0]),rpy[1]) @
                     axis_angle_matrix(np.array([1.,0,0]),rpy[0]))
                v = np.asarray(m.vertices) @ r.T + xyz
                self.robot.append((name, v, hull(name, v)[3]))
        self.zero_fk = forward_kinematics(YAM_URDF, {f"joint{i}": 0. for i in range(1,9)})
        self.tcp_from_world = np.linalg.inv(self.zero_fk["gripper"])
        self.tcp_from_world[:3, 3] -= TCP_OFFSET

    def arm(self, hand):
        fk = forward_kinematics(YAM_URDF, {
            **{f"joint{i+1}": v for i,v in enumerate(hand.q)},
            "joint7": -.04695*hand.grip, "joint8": -.04695*hand.grip})
        return [transform_hull(n,v,e,fk[n][:3,:3],fk[n][:3,3]+hand.base) for n,v,e in self.robot]

    def tool(self, tcp, rotation, opening):
        result = []
        for name, v, e in self.robot:
            if name not in ("gripper", "tip_left", "tip_right"):
                continue
            tf = self.tcp_from_world @ self.zero_fk[name]
            local = v @ tf[:3,:3].T + tf[:3,3]
            local[:,0] += -opening if name == "tip_left" else opening if name == "tip_right" else 0.
            result.append(hull(name, local @ rotation.T + tcp))
        return result

    def asset(self, path, position, rotation, prefix=None, joint_transforms=None):
        stage = Usd.Stage.Open(str(path))
        for prim_path, (axis, degrees) in (joint_transforms or {}).items():
            stage.GetPrimAtPath(prim_path).GetAttribute("xformOp:rotate"+axis).Set(degrees)
        cache = UsdGeom.XformCache()
        result = []
        prims = Usd.PrimRange(stage.GetPrimAtPath(prefix)) if prefix else stage.Traverse()
        for p in prims:
            if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():
                continue
            if p.IsA(UsdGeom.Mesh):
                v = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())
            elif p.IsA(UsdGeom.Cube):
                h = UsdGeom.Cube(p).GetSizeAttr().Get()/2
                v = np.array([(x,y,z) for x in (-h,h) for y in (-h,h) for z in (-h,h)])
            elif p.IsA(UsdGeom.Cylinder):
                obj = UsdGeom.Cylinder(p)
                r,h = obj.GetRadiusAttr().Get(), obj.GetHeightAttr().Get()/2
                v = np.array([(r*math.cos(a),r*math.sin(a),z)
                    for a in np.linspace(0,2*math.pi,24,endpoint=False) for z in (-h,h)])
                if obj.GetAxisAttr().Get() == "Y": v = v[:,[0,2,1]]
                if obj.GetAxisAttr().Get() == "X": v = v[:,[2,0,1]]
            else:
                continue
            tf = np.array(cache.GetLocalToWorldTransform(p)).T
            v = (np.c_[v,np.ones(len(v))] @ tf.T)[:,:3]
            result.append(hull(Path(path).stem+"/"+p.GetName(), v @ rotation.T + position))
        return result


def box(name, position, size):
    v = np.array([np.asarray(position)+np.array([x,y,z])*np.asarray(size)/2
                  for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    return hull(name,v)
