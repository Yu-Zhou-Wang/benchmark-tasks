"""Reference-informed mini PAGE caster; metres/kg/radians. +X front, Y width, Z up.
Plate dimensions and casting order follow the manufacturer manual. Other shapes,
glass thicknesses, spring constants and masses are nominal research geometry.
"""
from math import pi

REAR_SIZE = (.003, .101, .082)
FRONT_SIZE = (.003, .101, .073)
SPACER_SIZE = (.001, .006, .073)
SPACER_LOCAL_POSITIONS = ((.002, -.0475, -.0045), (.002, .0475, -.0045))
REAR_MASS = .064
FRONT_MASS = .055
PLATE_BOTTOM = .012
REAR_POS = (0., 0., .053)
FRONT_POS = (.004, 0., .0485)
GAP_X = (.0015, .0025)
PACK_X = (-.0015, .0055)
PACK_THICKNESS = .007
CONTACT_OFFSET = .00005

FRAME_MASS = .062
CAM_MASS = .012
CAM_PIVOTS = ((.009, -.054, .0495), (.009, .054, .0495))
CAM_OPEN_DEGREES = (-100., 100.)
CAM_CLOSED_DEGREES = (0., 0.)
CAM_SPRING_STIFFNESS = .015
CAM_SPRING_DAMPING = .0008
CAM_MAX_TORQUE = .03
CAM_HINGE_PATHS = ('/CastingFrame/left_hinge', '/CastingFrame/right_hinge')
CAM_BODY_PATHS = ('/CastingFrame/left_cam', '/CastingFrame/right_cam')
FRAME_BODY_PATH = '/CastingFrame/body'

CLIP_PIVOT = (-.025, 0., .105)
CLIP_OPEN_DEGREES = -75.
CLIP_MASS = .010
CLIP_SPRING_STIFFNESS = .045
CLIP_SPRING_DAMPING = .002
CLIP_MAX_TORQUE = .05
CLIP_HINGE_PATH = '/CastingStand/clip_hinge'
CLIP_BODY_PATH = '/CastingStand/pressure_clip'
STAND_BODY_PATH = '/CastingStand/body'
STAND_MASS = .140
STAND_SIZE = (.082, .151, .108)

GASKET_POS = (.002, 0., .0105)
GASKET_BOTTOM_SIZE = (.014, .105, .003)
GASKET_MASS = .006
COMB_POS = (.002, 0., .0915)
COMB_BLADE_THICKNESS = .0008
COMB_TOOTH_COUNT = 10
COMB_TOOTH_WIDTH = .0045
COMB_TOOTH_LENGTH = .012
COMB_TOOTH_PITCH = .008
COMB_HEADER_SIZE = (.0008, .087, .012)
COMB_MASS = .003
TRAY_SIZE = (.120, .155, .012)
TRAY_MASS = .080

ASSET_PATHS = {n: n+'.usda' for n in ('rear_plate','front_plate','casting_frame','gasket','comb','casting_stand','parts_tray')}
DEFAULT_PRIMS = dict(zip(ASSET_PATHS,('/RearPlate','/FrontPlate','/CastingFrame','/Gasket','/Comb','/CastingStand','/PartsTray')))

def pose(asset, position=(0.,0.,0.), **kwargs):
    return dict(asset=asset, position=position, yaw_degrees=0., **kwargs)

ASSEMBLY_PLACEMENTS = {
    'casting_stand': pose('casting_stand', clip_degrees=0.),
    'gasket': pose('gasket', GASKET_POS),
    'casting_frame': pose('casting_frame', cam_degrees=CAM_CLOSED_DEGREES),
    'rear_plate': pose('rear_plate', REAR_POS),
    'front_plate': pose('front_plate', FRONT_POS),
}
FINAL_WITH_COMB_PLACEMENTS = dict(ASSEMBLY_PLACEMENTS, comb=pose('comb', COMB_POS))
EXPLODED_PLACEMENTS = {
    'casting_stand': pose('casting_stand', (-.085,0.,0.), clip_degrees=CLIP_OPEN_DEGREES),
    'gasket': pose('gasket', (-.047,0.,.025)),
    'casting_frame': pose('casting_frame', (.015,0.,.030), cam_degrees=CAM_OPEN_DEGREES),
    'rear_plate': pose('rear_plate', (.065,0.,.100)),
    'front_plate': pose('front_plate', (.105,0.,.0955)),
    'comb': pose('comb', (.105,0.,.175)),
}

def shift(pos, x=0., z=0.):
    return (pos[0]+x,pos[1],pos[2]+z)

STEP_LAYOUTS = {
    '01_align_plates': {
        'rear_plate': pose('rear_plate',REAR_POS),
        'front_plate': pose('front_plate',shift(FRONT_POS,x=.022)),
    },
    '02_insert_in_frame': {
        'casting_frame': pose('casting_frame',cam_degrees=CAM_OPEN_DEGREES),
        'rear_plate': pose('rear_plate',shift(REAR_POS,z=.032)),
        'front_plate': pose('front_plate',shift(FRONT_POS,z=.032)),
    },
    '03_close_cams': {
        'casting_frame': pose('casting_frame',cam_degrees=CAM_CLOSED_DEGREES),
        'rear_plate': pose('rear_plate',REAR_POS),
        'front_plate': pose('front_plate',FRONT_POS),
    },
    '04_dock_on_gasket': dict(ASSEMBLY_PLACEMENTS,casting_stand=pose('casting_stand',clip_degrees=CLIP_OPEN_DEGREES)),
    '05_engage_top_clip': dict(ASSEMBLY_PLACEMENTS),
    '06_insert_comb': dict(FINAL_WITH_COMB_PLACEMENTS),
}
STEP_TITLES = [
    'Align plate bottoms; spacer plate behind the short plate',
    'Slide the glass sandwich into the frame with both cams open',
    'Rotate both side cams inward to clamp the glass',
    'Seat the framed cassette on the horizontal sealing pad',
    'Engage the top spring lever: ready to cast',
    'After gel filling, insert the comb between the plates',
]
FIDELITY = (
    'Original reference-informed single-bay mini PAGE casting kit. Rear 101x82mm and front 101x73mm plates, '
    'nominal3mm glass thickness, bonded1mm spacers. Separate green frame with two vertical-axis pressure cams '
    'and transparent stand with horizontal-axis top spring clip. Nominal dimensions, masses and springs outside '
    'the manufacturer-specified plate and cavity sizes are visually inferred research approximations, not CAD. '
    'Glass and gasket are rigid. No liquid, leakage, polymerization, fracture or compliant contact is simulated. '
    'The assembled layout is casting-ready and excludes the comb; final_with_comb is a later illustrative state. '
    'Assembly steps and exploded stages are static explanatory layouts, not robot demonstrations. '
    'No hidden plate attachments. Cam closure and stand retention require dynamic validation before robotics claims.'
)
