"""Convert the supplied ChemLab visual mesh and primitive proxies to portable USD."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from isaacsim import SimulationApp
APP=SimulationApp({"headless":True})
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
HERE=Path(__file__).resolve().parent

def build(name):
    root=ET.parse(HERE/'source_assets'/f'{name}.urdf').getroot().find('link')
    stage=Usd.Stage.CreateNew(str(HERE/'assets'/f'{name}.usda'))
    UsdGeom.SetStageMetersPerUnit(stage,1)
    UsdGeom.SetStageUpAxis(stage,'Z')
    body=UsdGeom.Xform.Define(stage,'/Body');stage.SetDefaultPrim(body.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    PhysxSchema.PhysxContactReportAPI.Apply(body.GetPrim()).CreateThresholdAttr(0.0)
    v=root.find('visual');meshfile=Path(v.find('geometry/mesh').get('filename')).name
    points=[];faces=[]
    for line in (HERE/'source_assets'/meshfile).read_text().splitlines():
        items=line.split()
        if not items:continue
        if items[0]=='v':points.append(tuple(map(float,items[1:4])))
        elif items[0]=='f':faces.append([int(s.split('/')[0])-1 for s in items[1:]])
    visual=UsdGeom.Mesh.Define(stage,'/Body/visual')
    visual.CreatePointsAttr(points);visual.CreateFaceVertexCountsAttr([len(f) for f in faces]);visual.CreateFaceVertexIndicesAttr(sum(faces,[]))
    visual.CreateSubdivisionSchemeAttr('none');visual.CreateDoubleSidedAttr(True)
    color=(0.04,0.22,0.65) if name=='cap' else (0.57,0.72,0.79)
    visual.CreateDisplayColorAttr([color])
    # Opaque glass surrogate keeps the neck visible without renderer-dependent transparency.
    for i,c in enumerate(root.findall('collision')):
        node=c.find('geometry')[0];attrs=node.attrib
        path=f'/Body/collision_{i:02}'
        if node.tag=='box':
            shape=UsdGeom.Cube.Define(stage,path);shape.CreateSizeAttr(1.)
            size=tuple(map(float,attrs['size'].split()))
        elif node.tag=='cylinder':
            shape=UsdGeom.Cylinder.Define(stage,path);shape.CreateRadiusAttr(float(attrs['radius']));shape.CreateHeightAttr(float(attrs['length']));shape.CreateAxisAttr('Z');size=None
        else:raise ValueError(node.tag)
        origin=c.find('origin');xyz=tuple(map(float,origin.get('xyz','0 0 0').split()));rpy=tuple(map(float,origin.get('rpy','0 0 0').split()))
        shape.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        shape.AddRotateXYZOp().Set(Gf.Vec3f(*np.degrees(rpy)))
        if size:shape.AddScaleOp().Set(Gf.Vec3f(*size))
        shape.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        props=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());props.CreateContactOffsetAttr(.001);props.CreateRestOffsetAttr(0.)
    stage.GetRootLayer().Save()
    p=np.array(points);print(name,p.min(0),p.max(0),len(faces))

if __name__=='__main__':
    (HERE/'assets').mkdir(exist_ok=True)
    for name in ('body','cap'):build(name)
    APP.close()
