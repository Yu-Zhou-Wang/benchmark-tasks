"""Read collision bounds per rigid link, excluding nested links and visual meshes."""
from __future__ import annotations

import numpy as np


def read_link_corners(usd_path, names):
    from pxr import Usd, UsdGeom, UsdPhysics
    stage=Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise ValueError(f'Cannot open robot collision asset: {usd_path}')
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'],
                           useExtentsHint=False,ignoreVisibility=True)
    parts={name:[] for name in names}
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        enabled=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        if enabled is False:
            continue
        owner=prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner=owner.GetParent()
        if not owner or owner.GetName() not in parts:
            continue
        # A link's recursive visual bounds include its child rigid links in the
        # URDF hierarchy. Assign each collider only to its nearest rigid parent.
        box=cache.ComputeRelativeBound(prim,owner).ComputeAlignedRange()
        if box.IsEmpty():
            raise ValueError(f'Empty collision bounds: {prim.GetPath()}')
        low,high=box.GetMin(),box.GetMax()
        parts[owner.GetName()].extend((x,y,z) for x in (low[0],high[0])
            for y in (low[1],high[1]) for z in (low[2],high[2]))
    missing=[name for name,points in parts.items() if not points]
    if missing:
        raise ValueError(f'Cannot measure collision geometry for robot links: {missing}')
    return {name:np.asarray(points,float) for name,points in parts.items()}
