"""Generate original primitive CAD. CPU only; needs usd-core, numpy and scipy.

The head is retained solely by solid slot lips. No joint connects it to the door.
The chain's permanent pin joints approximate interlocked steel rings.
"""
from pathlib import Path
import math
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
import geometry as G

HERE=Path(__file__).resolve().parent

def quat(r):
    x,y,z,w=Rotation.from_matrix(r).as_quat()
    return Gf.Quatf(float(w),float(x),float(y),float(z))

def body(stage,path,p,r,mass,inertia,com=(0,0,0)):
    b=UsdGeom.Xform.Define(stage,path)
    b.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)))
    b.AddOrientOp().Set(quat(r))
    UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
    m=UsdPhysics.MassAPI.Apply(b.GetPrim())
    m.CreateMassAttr(mass); m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia)); m.CreatePrincipalAxesAttr(Gf.Quatf(1,0,0,0))
    return b

def material(stage,name,rgb,metal=0.):
    m=UsdShade.Material.Define(stage,f'/latch/looks/{name}')
    sh=UsdShade.Shader.Define(stage,f'{m.GetPath()}/shader')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.38 if metal else .75)
    sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
    return m

def finish(prim,mat,collision):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim)
        prim.AddAppliedSchema('PhysxCollisionAPI')
        prim.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.0005)
        prim.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)

def box(st,path,center,size,mat,collision=True):
    b=UsdGeom.Cube.Define(st,path); b.CreateSizeAttr(1.)
    b.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
    b.AddScaleOp().Set(Gf.Vec3f(*size)); finish(b.GetPrim(),mat,collision)

def span(st,path,x,y,z,mat,collision=True):
    box(st,path,[(v[0]+v[1])/2 for v in (x,y,z)],[v[1]-v[0] for v in (x,y,z)],mat,collision)

def capsule(st,path,a,b,r,mat,collision=True):
    a,b=np.array(a),np.array(b); d=b-a; n=np.linalg.norm(d); x=d/n
    ref=np.array([0.,0,1.]) if abs(x[2])<.95 else np.array([0.,1.,0])
    y=np.cross(ref,x); y/=np.linalg.norm(y); z=np.cross(x,y)
    c=UsdGeom.Capsule.Define(st,path); c.CreateAxisAttr('X'); c.CreateRadiusAttr(r); c.CreateHeightAttr(float(n))
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float,(a+b)/2))); c.AddOrientOp().Set(quat(np.stack([x,y,z],1)))
    finish(c.GetPrim(),mat,collision)

def joint(st,name,b0,p0,r0,b1,p1,r1,p,axis='Y',limits=(-175,175),k=0.,d=.00003,target=0.):
    j=UsdPhysics.RevoluteJoint.Define(st,f'/latch/joints/{name}')
    j.CreateBody0Rel().SetTargets([Sdf.Path(b0)]); j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
    frame=r1
    for pp,rr,pa,ra in ((p0,r0,j.CreateLocalPos0Attr,j.CreateLocalRot0Attr),(p1,r1,j.CreateLocalPos1Attr,j.CreateLocalRot1Attr)):
        pa(Gf.Vec3f(*map(float,rr.T@(p-pp)))); ra(quat(rr.T@frame))
    j.GetPrim().AddAppliedSchema('PhysxJointAPI')
    j.GetPrim().CreateAttribute('physxJoint:maxJointVelocity',Sdf.ValueTypeNames.Float).Set(360.)
    j.CreateAxisAttr(axis); j.CreateLowerLimitAttr(limits[0]); j.CreateUpperLimitAttr(limits[1])
    drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular')
    drive.CreateStiffnessAttr(k*math.pi/180); drive.CreateDampingAttr(d*math.pi/180)
    drive.CreateTargetPositionAttr(target); drive.CreateMaxForceAttr(100.)
    return j

def build():
    path=HERE/'assets/latch.usda'; path.parent.mkdir(exist_ok=True)
    st=Usd.Stage.CreateNew(str(path)); UsdGeom.SetStageUpAxis(st,'Z'); UsdGeom.SetStageMetersPerUnit(st,1.)
    root=UsdGeom.Xform.Define(st,'/latch').GetPrim(); st.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root); root.AddAppliedSchema('PhysxArticulationAPI')
    root.CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(True)
    root.CreateAttribute('physxArticulation:solverPositionIterationCount',Sdf.ValueTypeNames.Int).Set(64)
    root.CreateAttribute('physxArticulation:solverVelocityIterationCount',Sdf.ValueTypeNames.Int).Set(16)
    mats={n:material(st,n,c,m) for n,c,m in [('wood',(.57,.30,.12),0),('frame',(.73,.72,.67),0),('brass',(.70,.48,.15),.65),('steel',(.50,.53,.58),.65),('dark',(.055,.06,.07),.2)]}
    I=np.eye(3); O=np.zeros(3)
    body(st,'/latch/frame',O,I,5.,(.1,.1,.1))
    fixed=UsdPhysics.FixedJoint.Define(st,'/latch/joints/world_fixed'); fixed.CreateBody1Rel().SetTargets(['/latch/frame'])
    box(st,'/latch/frame/foot',(.018,-.18,.008),(.18,.47,.016),mats['frame'])
    box(st,'/latch/frame/hinge_post',(.014,.020,.18),(.036,.04,.34),mats['frame'])
    box(st,'/latch/frame/strike_post',(.014,-.399,.18),(.036,.046,.34),mats['frame'])
    box(st,'/latch/frame/anchor_plate',(-.014,-.393,.245),(.010,.030,.046),mats['brass'])
    capsule(st,'/latch/frame/anchor_eye',G.ANCHOR+[0,0,-.006],G.ANCHOR+[0,0,.006],.003,mats['brass'])
    # The hinge's zero frame is the closed position. Initial angle is written by reset.
    body(st,'/latch/door',O,I,.70,(.008,.006,.008),com=(0,-.18,.17))
    box(st,'/latch/door/panel',(0,-.18,.175),(.018,.36,.31),mats['wood'])
    for h in (.065,.28):
        capsule(st,f'/latch/door/hinge{int(h*1000)}',(0,0,h-.019),(0,0,h+.019),.009,mats['steel'],False)
    joint(st,'door_hinge','/latch/frame',O,I,'/latch/door',O,I,O,'Z',(-75,0),G.SPRING_K,G.SPRING_DAMPING,math.degrees(G.SPRING_TARGET))
    # Door handle: two standoffs plus a graspable vertical bar.
    for idx,z in enumerate((.100,.160)):
        capsule(st,f'/latch/door/handle_stem{idx}',(-.013,-.155,z),(-.055,-.155,z),.006,mats['dark'])
    capsule(st,'/latch/door/handle',(-.055,-.155,.100),(-.055,-.155,.160),.008,mats['dark'])
    # Plate: hollow channel behind two lips, a closed narrow end, and a genuinely wide exit.
    z=.245
    for i,s in enumerate((-1,1)):
        span(st,f'/latch/door/rail{i}',(-.020,-.010),(-.345,-.240),(z+s*.017-.002,z+s*.017+.002),mats['brass'])
        zz=(z-.015,z-.004) if s<0 else (z+.004,z+.015)
        span(st,f'/latch/door/lip{i}',(-.020,-.017),(-.345,-.274),zz,mats['brass'])
    span(st,'/latch/door/closed_end',(-.020,-.010),(-.345,-.337),(z-.015,z+.015),mats['brass'])
    span(st,'/latch/door/exit_end',(-.020,-.010),(-.246,-.240),(z-.015,z+.015),mats['brass'])
    # Backing is the wood face; front screws are visual discs (no fictitious collider).
    for i,y in enumerate((-.340,-.243)):
        for j,zz in enumerate((z-.017,z+.017)):
            capsule(st,f'/latch/door/screw{i}{j}',(-.0205,y,zz),(-.021,y,zz),.0025,mats['steel'],False)
    pts=G.chain_points(); prev='/latch/frame'; pp=O; pr=I
    bodies=[]
    for k in range(G.N_LINKS):
        p=pts[k]; x=pts[k+1]-p; length=float(np.linalg.norm(x)); x/=length
        y=np.cross([0,0,1],x); y/=np.linalg.norm(y); r=np.stack([x,y,np.cross(x,y)],1)
        if k%2: r=r@np.array([[1.,0,0],[0,0,-1],[0,1,0]])
        # Alternating pin axes approximate the two-link bend of an oval chain.
        path=f'/latch/link{k:02d}'
        body(st,path,p,r,.004,(2e-7,4e-7,4e-7),com=(length/2,0,0))
        joint(st,f'bend{k:02d}',prev,pp,pr,path,p,r,p,'Y',(-120,120),d=.00005)
        off=.0045
        for side in (-1,1): capsule(st,f'{path}/side{side+1}',(0,side*off,0),(length,side*off,0),G.WIRE_R,mats['steel'])
        for end in (0,1): capsule(st,f'{path}/end{end}',(end*length,-off,0),(end*length,off,0),G.WIRE_R,mats['steel'])
        # Neighbouring rings' intersecting end bars represent permanent interlinking.
        UsdPhysics.FilteredPairsAPI.Apply(st.GetPrimAtPath(path)).CreateFilteredPairsRel().AddTarget(bodies[-1] if bodies else '/latch/frame')
        bodies.append(path); prev,pp,pr=path,p,r
    # End swivel lets the thumb fitting orient without making the chain rigid.
    hp=G.rz(G.INITIAL_ANGLE)@G.HEAD; hr=G.rz(G.INITIAL_ANGLE); p=pts[-1]
    body(st,'/latch/fitting',hp,hr,.022,(2e-6,4e-6,4e-6),com=(-.014,0,0))
    joint(st,'end_twist',prev,pp,pr,'/latch/fitting',hp,hr,p,'Y',(-140,140),d=.00005)
    box(st,'/latch/fitting/head',(0,0,0),(.004,.016,.018),mats['brass'])
    box(st,'/latch/fitting/neck',(-.013,0,0),(.024,.006,.006),mats['brass'])
    box(st,'/latch/fitting/tab',G.TAB,(.014,.022,.022),mats['brass'])
    capsule(st,'/latch/fitting/eye',G.ATTACH+[0,0,-.004],G.ATTACH+[0,0,.004],.0022,mats['brass'])
    UsdPhysics.FilteredPairsAPI.Apply(st.GetPrimAtPath('/latch/fitting')).CreateFilteredPairsRel().AddTarget(bodies[-1])
    # Empty pivot bodies need a finite visual bound for the adapter's AABB reader.
    for prim in st.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and ('pivot' in prim.GetName() or 'swivel' in prim.GetName()):
            box(st,str(prim.GetPath())+'/marker',(0,0,0),(.0001,)*3,mats['steel'],False)
    st.GetRootLayer().Save()
    print(f'Wrote latch.usda; chain length {G.CHAIN_LENGTH:.6f} m')

if __name__=='__main__': build()
