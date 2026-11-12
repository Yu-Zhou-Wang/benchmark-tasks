"""Regression: recursive USD hierarchy and visual meshes are not rigid-link bounds."""
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from collision_bounds import read_link_corners


class CollisionBoundsTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.path=Path(self.temp.name)/'robot.usda'
        self.stage=Usd.Stage.CreateNew(str(self.path))
        root=UsdGeom.Xform.Define(self.stage,'/base')
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        self.cube('/base/collision',0.1)

    def tearDown(self):
        self.stage=None
        self.temp.cleanup()

    def cube(self,path,size,collide=True):
        geom=UsdGeom.Cube.Define(self.stage,path)
        geom.CreateSizeAttr(size)
        if collide:
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        return geom

    def read(self,names):
        self.stage.GetRootLayer().Save()
        return read_link_corners(self.path,names)

    def test_nested_link_does_not_enlarge_parent(self):
        child=UsdGeom.Xform.Define(self.stage,'/base/child')
        child.AddTranslateOp().Set(Gf.Vec3d(0,0,0.5))
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        self.cube('/base/child/collision',0.02)
        parts=self.read(['base','child'])
        np.testing.assert_allclose(parts['base'].min(0),[-.05]*3)
        np.testing.assert_allclose(parts['base'].max(0),[.05]*3)
        np.testing.assert_allclose(parts['child'].min(0),[-.01]*3)
        np.testing.assert_allclose(parts['child'].max(0),[.01]*3)

    def test_visual_meshes_and_repeated_names_are_ignored(self):
        self.cube('/base/base',10,collide=False)
        parts=self.read(['base'])
        np.testing.assert_allclose(parts['base'].max(0),[.05]*3)

    def test_invisible_guide_colliders_still_count(self):
        geom=self.cube('/base/extra_collision',0.02)
        geom.AddTranslateOp().Set(Gf.Vec3d(.1,0,0))
        geom.CreateVisibilityAttr('invisible')
        geom.CreatePurposeAttr('guide')
        parts=self.read(['base'])
        self.assertAlmostEqual(parts['base'][:,0].max(),.11)

    def test_disabled_colliders_are_ignored(self):
        geom=self.cube('/base/disabled',10)
        UsdPhysics.CollisionAPI(geom).CreateCollisionEnabledAttr(False)
        parts=self.read(['base'])
        np.testing.assert_allclose(parts['base'].max(0),[.05]*3)

    def test_missing_collision_geometry_fails_closed(self):
        child=UsdGeom.Xform.Define(self.stage,'/base/no_collision')
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        with self.assertRaises(ValueError):
            self.read(['base','no_collision'])


if __name__=='__main__':
    unittest.main()
