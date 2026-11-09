"""Derive manipulation assets from the reference-matched gel-casting kit.

    python build_task_assets.py --source-assets assets --output task_assets

Source assets are never edited. Glass remains independent and free; no plate,
frame, robot or stand attachment is created. The installed sealing pad belongs
to the stand's rigid body. Cams have zero drive gains in the base asset; the
task configures an angle-dependent implicit bistable spring for their nominal
detent and preload. The stand lever retains its passive return spring.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

CONTACT_OFFSET = 0.00005
CAM_FRICTION = 0.0005
CAM_OPEN_RAD = (-math.radians(100), math.radians(100))
CAM_CLOSED_TARGET_RAD = (math.radians(2), -math.radians(2))
POST_TORSIONAL_PATCH_RADIUS = .004
POST_MIN_TORSIONAL_PATCH_RADIUS = .0032
STAND_LEVER_STIFFNESS_SI = 0.045
STAND_LEVER_DAMPING_SI = 0.002
STAND_LEVER_MAX_TORQUE = 0.05
STAND_LEVER_TARGET_DEG = 8.0
REAR_SUPPORT_FACE_X = -.0123
FIT_CLEARANCE = .0003
REAR_RIB_SPAN = .088
FRAME_COM = (-0.003, 0.0, 0.0495)
STAND_COM = (-0.012, 0.0, 0.040)
CLIP_COM = (0.005, 0.0, -0.002)


def _attr(prim, name, value):
    kind = (Sdf.ValueTypeNames.Bool if isinstance(value, bool) else
            Sdf.ValueTypeNames.Int if isinstance(value, int) else
            Sdf.ValueTypeNames.Token if isinstance(value, str) else Sdf.ValueTypeNames.Float)
    prim.CreateAttribute(name, kind, custom=False).Set(value)


def _collision(prim):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    prim.AddAppliedSchema('PhysxCollisionAPI')
    _attr(prim, 'physxCollision:contactOffset', CONTACT_OFFSET)
    _attr(prim, 'physxCollision:restOffset', 0.0)


def _bind(prim, material):
    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    binding.Bind(material)
    binding.Bind(material, materialPurpose='physics')


def _material(stage, path, color, friction=0.8):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.8)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    physical = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physical.CreateStaticFrictionAttr(friction)
    physical.CreateDynamicFrictionAttr(friction*0.8)
    physical.CreateRestitutionAttr(0.)
    return material


def _cube(stage, path, pos, size, material=None, visible=True):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.0)
    shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if material:
        _bind(shape.GetPrim(), material)
    if not visible:
        shape.CreateVisibilityAttr('invisible')
    _collision(shape.GetPrim())
    return shape


def _mass(prim, mass, com, size):
    api = UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(float(mass))
    api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    x, y, z = size
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(mass*(y*y+z*z)/12,
                                        mass*(x*x+z*z)/12,
                                        mass*(x*x+y*y)/12))
    api.CreatePrincipalAxesAttr(Gf.Quatf(1.))
    prim.AddAppliedSchema('PhysxRigidBodyAPI')
    _attr(prim, 'physxRigidBody:solverPositionIterationCount', 64)
    _attr(prim, 'physxRigidBody:solverVelocityIterationCount', 8)
    _attr(prim, 'physxRigidBody:enableCCD', True)


def _friction(stage, root, static=0.9, dynamic=0.75):
    """Set nominal contact friction, retaining authored shader appearances."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(UsdPhysics.MaterialAPI):
            api = UsdPhysics.MaterialAPI(prim)
            api.CreateStaticFrictionAttr(static)
            api.CreateDynamicFrictionAttr(dynamic)
            api.CreateRestitutionAttr(0.)


def _clone(source, destination):
    if source.resolve() == destination.resolve():
        raise ValueError('Task output must be separate from the visual source assets')
    shutil.copy2(source, destination)
    stage = Usd.Stage.Open(str(destination))
    if not stage:
        raise RuntimeError(f'Cannot open {destination}')
    return stage


def _frame(source, output):
    stage = _clone(source/'casting_frame.usda', output/'casting_frame.usda')
    root = '/CastingFrame'
    _mass(stage.GetPrimAtPath(root+'/body'), .062, FRAME_COM, (.025, .123, .084))
    green = UsdShade.Material(stage.GetPrimAtPath(root+'/Looks/GreenPolysulfone'))
    # Central ribs carry the stand reaction without entering the side-post
    # grasp corridors. Shorten the visible rounded ends and matching proxies
    # together; preserve the source bevel instead of scaling its radius.
    for i,z in enumerate((.019,.081)):
        name=('lower_rail_rib','upper_rail_rib')[i]
        mesh=UsdGeom.Mesh(stage.GetPrimAtPath(root+'/body/'+name))
        points=mesh.GetPointsAttr().Get()
        trim=max(abs(float(p[1])) for p in points)-REAR_RIB_SPAN/2
        adjusted=[Gf.Vec3f(float(p[0]),float(p[1])-math.copysign(trim,float(p[1])),float(p[2])) for p in points]
        mesh.GetPointsAttr().Set(adjusted)
        mesh.CreateExtentAttr([Gf.Vec3f(*(min(float(p[k]) for p in adjusted) for k in range(3))),
                               Gf.Vec3f(*(max(float(p[k]) for p in adjusted) for k in range(3)))])
        # Native box fills the visible 0.5 mm rounded bevel, avoiding slender
        # convex-mesh cooking fallback. It has the same 88 mm overall span.
        _cube(stage,root+f'/body/rear_rib_collision_{i}',
              (-.011,0.,z),(.002,REAR_RIB_SPAN,.002),green,visible=False)
    # A finite-area jaw contact resists twist while gripping a 10 mm-wide post.
    # Keep this nominal patch on the two actual grasp faces only; applying it
    # to the whole articulation would also alter the cam/glass mechanism.
    for i in range(2):
        post=stage.GetPrimAtPath(root+f'/body/side_post_{i}')
        post.AddAppliedSchema('PhysxCollisionAPI')
        _attr(post,'physxCollision:torsionalPatchRadius',POST_TORSIONAL_PATCH_RADIUS)
        _attr(post,'physxCollision:minTorsionalPatchRadius',POST_MIN_TORSIONAL_PATCH_RADIUS)
    for name, sign in [('left_cam', 1), ('right_cam', -1)]:
        _mass(stage.GetPrimAtPath(root+'/'+name), .012, (.001, sign*.013, 0.), (.009,.029,.073))
    for name in ('left_hinge', 'right_hinge'):
        prim = stage.GetPrimAtPath(root+'/'+name)
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateTypeAttr('force')
        drive.CreateTargetPositionAttr(0.)
        drive.CreateTargetVelocityAttr(0.)
        drive.CreateStiffnessAttr(0.)
        drive.CreateDampingAttr(0.)
        # Zero gains remove motor closure; this bound does not command torque.
        drive.CreateMaxForceAttr(.20)
        prim.AddAppliedSchema('PhysxJointAPI')
        _attr(prim, 'physxJoint:jointFriction', CAM_FRICTION)
    _friction(stage, root)
    stage.GetDefaultPrim().SetCustomDataByKey('taskMechanics',
        'Free frame and two cam links. Zero drive stiffness/damping. '
        'Runtime angle-dependent implicit bistable spring, not an achievement-triggered latch. '
        'Only the two frame side posts have a nominal 4 mm torsional contact patch (3.2 mm minimum). '
        'No glass attachments. Nominal COM/inertias/contact friction require dynamic verification.')
    stage.GetRootLayer().Save()


def _stand(source, output):
    stage = _clone(source/'casting_stand.usda', output/'casting_stand.usda')
    root = '/CastingStand'
    # The installed pad is part of this rigid body, not a manipulated loose part.
    _mass(stage.GetPrimAtPath(root+'/body'), .146, STAND_COM, (.082,.151,.108))
    _mass(stage.GetPrimAtPath(root+'/pressure_clip'), .010, CLIP_COM, (.039,.027,.026))
    rubber = UsdShade.Material(stage.GetPrimAtPath(root+'/Looks/Rubber'))
    clear = UsdShade.Material(stage.GetPrimAtPath(root+'/Looks/ClearPolycarbonate'))
    _cube(stage, root+'/body/sealing_pad', (.002,0.,.0105), (.014,.105,.003), rubber)
    # Visible molded pads bridge the old oversized rear gap while leaving
    # both side-post gripper corridors open. All contacts are ordinary solids.
    for i,y in enumerate((-.038,.038)):
        for j,z in enumerate((.019,.081)):
            _cube(stage,root+f'/body/rear_registration_pad_{i}_{j}',
                  (-.0199,y,z),(.0152,.006,.008),clear)
    # Low, open-top L guides bound lateral and forward foot travel. Their
    # 0.3 mm insertion clearance and 12 mm top permit straight-up extraction.
    for i,sign in enumerate((-1,1)):
        _cube(stage,root+f'/body/foot_lateral_guide_{i}',
              (.003,sign*.0648,.0105),(.031,.003,.003),clear)
        _cube(stage,root+f'/body/foot_front_guide_{i}',
              (.0203,sign*.058,.0105),(.003,.0106,.003),clear)
    # Native box proxies avoid the slender guides' GPU convex cooking fallback.
    # Visual rounded guides remain unchanged; the proxy fills only their 0.3mm bevel.
    for i,x in enumerate((-.0065,.0105)):
        guide = stage.GetPrimAtPath(root+f'/body/gasket_guide_{i}')
        UsdPhysics.CollisionAPI(guide).CreateCollisionEnabledAttr(False)
        _cube(stage, root+f'/body/gasket_guide_collision_{i}',
              (x,0.,.0096), (.003,.105,.0012), rubber, visible=False)
    joint = stage.GetPrimAtPath(root+'/clip_hinge')
    drive = UsdPhysics.DriveAPI.Apply(joint,'angular')
    drive.CreateTypeAttr('force')
    drive.CreateTargetPositionAttr(STAND_LEVER_TARGET_DEG)
    drive.CreateTargetVelocityAttr(0.)
    drive.CreateStiffnessAttr(STAND_LEVER_STIFFNESS_SI*math.pi/180.)
    drive.CreateDampingAttr(STAND_LEVER_DAMPING_SI*math.pi/180.)
    drive.CreateMaxForceAttr(STAND_LEVER_MAX_TORQUE)
    _friction(stage,root)
    stage.GetDefaultPrim().SetCustomDataByKey('taskMechanics',
        'Free stand with installed rigid sealing pad. Genuine spring-return top lever starts closed. '
        'Nominal +8 degree spring target beyond the +2 degree stop supplies seating preload. '
        'Four visible rear pads and low open-top foot guides have 0.3 mm nominal insertion clearance. '
        'One gripper must hold it open while the other docks the cassette. '
        'No attachment between cassette and stand. No liquid or compliant seal model.')
    stage.GetRootLayer().Save()


def _plates(source, output):
    for filename in ('rear_plate.usda','front_plate.usda'):
        stage = _clone(source/filename, output/filename)
        root = stage.GetDefaultPrim()
        UsdPhysics.RigidBodyAPI(root).CreateKinematicEnabledAttr(False)
        _friction(stage, str(root.GetPath()))
        root.SetCustomDataByKey('taskMechanics','Independent free glass plate; no hidden attachments.')
        stage.GetRootLayer().Save()


def _cradle(output):
    stage = Usd.Stage.CreateNew(str(output/'staging_cradle.usda'))
    UsdGeom.SetStageUpAxis(stage,'Z')
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    root=UsdGeom.Xform.Define(stage,'/StagingCradle').GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root).CreateKinematicEnabledAttr(True)
    _mass(root,.080,(.003,0.,.025),(.016,.105,.050))
    _attr(root,'physxRigidBody:enableCCD',False)
    mat=_material(stage,'/StagingCradle/Looks/GrayRubber',(.17,.19,.21))
    _cube(stage,'/StagingCradle/leveling_pad',(.002,0.,.0105),(.014,.105,.003),mat)
    for i,y in enumerate((-.020,.020)):
        # Front contact is 0.05 mm beyond the glass face, 38 mm above its
        # bottom. These visible fingers support upright loose glass while
        # leaving both cam sweeps and the straight vertical extraction free.
        _cube(stage,f'/StagingCradle/front_guide_{i}',(.00805,y,.0295),(.005,.006,.041),mat)
    for i,y in enumerate((-.058,.058)):
        # Shallow collars brace the frame feet while staying below the lower
        # hinge ears. All four walls end at z=.0115, so vertical lift releases
        # them without unlatching or changing any object attachment.
        walls=[((x,y,.01025),(.002,.0141,.0025)) for x in (-.01355,.01955)]
        walls += [((.003,yy,.01025),(.0311,.002,.0025)) for yy in (y-.00605,y+.00605)]
        for j,(pos,size) in enumerate(walls):
            _cube(stage,f'/StagingCradle/foot_nest_{i}_{j}',pos,size,mat)
    root.SetDocumentation(
        'Visible fixed preparation cradle, located at the initial frame origin. '
        'Place its origin at table height minus 9 mm. The 3 mm leveling pad contacts '
        'the flush glass bottoms at local Z 12 mm. Front alignment fingers rise '
        'to local Z 50 mm with 0.05 mm face clearance and do not contact the cams. '
        'Shallow 2.5 mm collars brace the two frame feet with 0.05 mm clearance; '
        'the frame rear rails support the rear plate. All parts remain independent. '
        'Lift the clamped cassette at least 45 mm to clear the open-top fixture. '
        'This fixture prevents loose upright plates tipping during initialization; '
        'it does not attach or retain the cassette during transfer.')
    stage.GetRootLayer().Save()


def build(source, output):
    source,output=Path(source).resolve(),Path(output).resolve()
    output.mkdir(parents=True,exist_ok=True)
    for name in ('rear_plate.usda','front_plate.usda','casting_frame.usda','casting_stand.usda'):
        if not (source/name).is_file(): raise FileNotFoundError(source/name)
    _frame(source,output)
    _stand(source,output)
    _plates(source,output)
    _cradle(output)
    manifest={
        'scope':'Secure pre-aligned plates, transfer cassette, hold spring lever open and dock, release lever.',
        'assets':['rear_plate.usda','front_plate.usda','casting_frame.usda','casting_stand.usda','staging_cradle.usda'],
        'frame_joint_names':['left_hinge','right_hinge'],
        'frame_initial_joint_rad':list(CAM_OPEN_RAD),
        'frame_nominal_closed_targets_rad':list(CAM_CLOSED_TARGET_RAD),
        'cam_drive_stiffness_si':0.,'cam_drive_damping_si':0.,'cam_joint_friction_coefficient':CAM_FRICTION,
        'frame_grasp_contact':{'collision_prims':['/CastingFrame/body/side_post_0','/CastingFrame/body/side_post_1'],
                               'torsional_patch_radius_m':POST_TORSIONAL_PATCH_RADIUS,
                               'minimum_torsional_patch_radius_m':POST_MIN_TORSIONAL_PATCH_RADIUS,
                               'scope':'Nominal finite-area jaw contact on 10 mm-wide posts only; cam and glass contacts unchanged.'},
        'stand_joint_name':'clip_hinge','stand_initial_joint_rad':0.,
        'stand_return_spring':{'stiffness_Nm_per_rad':STAND_LEVER_STIFFNESS_SI,
                               'damping_Nm_s_per_rad':STAND_LEVER_DAMPING_SI,
                               'max_torque_Nm':STAND_LEVER_MAX_TORQUE,'target_rad':math.radians(STAND_LEVER_TARGET_DEG)},
        'stand_registration':{'nominal_clearance_m':FIT_CLEARANCE,
                              'rear_pad_front_x_m':REAR_SUPPORT_FACE_X,
                              'rear_pad_centres_m':[[-.0199,y,z] for y in (-.038,.038) for z in (.019,.081)],
                              'rear_pad_size_m':[.0152,.006,.008],
                              'frame_rear_rib_span_m':REAR_RIB_SPAN,
                              'frame_rear_rib_collision':'Native boxes match central 88 mm visible ribs; fill 0.5 mm bevel. Side-post grasp corridors stay clear.',
                              'lateral_guide_centres_m':[[.003,y,.0105] for y in (-.0648,.0648)],
                              'lateral_guide_size_m':[.031,.003,.003],
                              'front_guide_centres_m':[[.0203,y,.0105] for y in (-.058,.058)],
                              'front_guide_size_m':[.003,.0106,.003],
                              'extraction':'Open-top guides, no hook or cassette attachment; spring lever must be held open.',
                              'provenance':'Functional design approximation, not manufacturer CAD or measured tolerances.'},
        'geometry':{'frame_floor_z':.009,'glass_bottom_z':.012,'rear_plate_position':[0.,0.,.053],
                    'front_plate_position':[.004,0.,.0485],'staging_cradle_same_origin_as_initial_frame':True,
                    'initial_frame_z_relative_to_table':-.009,'staging_front_guide_top_z':.050,
                    'staging_front_face_clearance_m':.00005,
                    'staging_foot_nest_height_m':.0025,'staging_foot_nest_clearance_m':.00005,
                    'minimum_lift_before_transfer_m':.045,'stand_origin_at_table':True},
        'centres_of_mass':{'frame_body':FRAME_COM,'stand_body':STAND_COM,'stand_clip':CLIP_COM},
        'external_mechanics_required':'An angle-dependent implicit bistable cam spring; no milestone-triggered pose writes or fixed joints.',
        'validation':'Asset authoring only; retention and robot manipulation must be demonstrated separately.',
        'limitations':['Rigid rubber proxy; no sealing compliance or fluid.','Nominal friction, detents, masses and inertias.',
                       'Two rounded gasket guide meshes use native box proxies filling their 0.3 mm bevels.',
                       'Two rear rib collision proxies fill their visible 0.5 mm bevels.',
                       'Stand registration and spring preload are nominal design choices, not manufacturer-calibrated geometry or a seal model.'],
    }
    (output/'task_mechanics.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-assets',type=Path,default=Path(__file__).with_name('assets'))
    parser.add_argument('--output',type=Path,default=Path(__file__).with_name('task_assets'))
    args=parser.parse_args()
    result=build(args.source_assets,args.output)
    print(json.dumps({'output':str(args.output),'assets':result['assets']},indent=2))
