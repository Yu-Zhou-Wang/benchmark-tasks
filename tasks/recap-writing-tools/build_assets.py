"""Regenerate the self-contained USD collision and visual assets using OpenUSD/PhysX."""

from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from geometry import (
    BARREL_LENGTH, BARREL_MASS, BARREL_SHOULDER, CAP_WALL, CONTACT_OFFSET, DAMPING, FRICTION,
    MM, PARTS, REST_Z, STIFFNESS, TABLE_Z, TOOLS, outline,
)

ASSETS = Path(__file__).resolve().parent / "assets"


class Asset:
    def __init__(self):
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1)
        self.root = UsdGeom.Xform.Define(self.stage, "/Part").GetPrim()
        self.stage.SetDefaultPrim(self.root)

    def material(self, name, color, friction=FRICTION, compliant=False):
        path = f"/Part/Materials/{name}"
        mat = UsdShade.Material.Define(self.stage, path)
        pbr = UsdShade.Shader.Define(self.stage, f"{path}/Surface")
        pbr.CreateIdAttr("UsdPreviewSurface")
        pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(c ** 2.2 for c in color)))
        pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.37)
        mat.CreateSurfaceOutput().ConnectToSource(pbr.CreateOutput("surface", Sdf.ValueTypeNames.Token))
        props = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        props.CreateStaticFrictionAttr(friction)
        props.CreateDynamicFrictionAttr(friction * 0.8)
        props.CreateRestitutionAttr(0)
        prim = mat.GetPrim()
        prim.AddAppliedSchema("PhysxMaterialAPI")
        prim.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
        if compliant:
            prim.CreateAttribute("physxMaterial:compliantContactStiffness", Sdf.ValueTypeNames.Float).Set(STIFFNESS)
            prim.CreateAttribute("physxMaterial:compliantContactDamping", Sdf.ValueTypeNames.Float).Set(DAMPING)
        return mat

    def rigid_body(self, mass):
        UsdPhysics.RigidBodyAPI.Apply(self.root)
        UsdPhysics.MassAPI.Apply(self.root).CreateMassAttr(mass)
        self.root.AddAppliedSchema("PhysxRigidBodyAPI")
        self.root.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(24)
        self.root.CreateAttribute("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(4)
        self.root.CreateAttribute("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0)
        self.root.CreateAttribute("physxRigidBody:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0)

    def surface(self, prim, material, collision=True):
        api = UsdShade.MaterialBindingAPI.Apply(prim)
        api.Bind(material)
        if collision:
            UsdPhysics.CollisionAPI.Apply(prim)
            prim.AddAppliedSchema("PhysxCollisionAPI")
            prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(CONTACT_OFFSET)
            prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.)
            api.Bind(material, materialPurpose="physics")

    def mesh(self, name, points, faces, material, collision=True):
        mesh = UsdGeom.Mesh.Define(self.stage, f"/Part/{name}")
        mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
        mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
        mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
        mesh.CreateSubdivisionSchemeAttr("none")
        prim = mesh.GetPrim()
        self.surface(prim, material, collision)
        if collision:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")

    def loft(self, name, sections, material, collision=True):
        """Closed convex mesh, built from successive XY rings at Z in mm."""
        n = len(sections[0][1])
        points = [(x, y, z * MM) for z, ring in sections for x, y in ring]
        faces = [list(reversed(range(n)))]
        for start in range(0, len(points) - n, n):
            for i in range(n):
                j = (i + 1) % n
                faces.append([start + i, start + j, start + n + j, start + n + i])
        faces.append(list(range(len(points) - n, len(points))))
        self.mesh(name, points, faces, material, collision)

    def box(self, name, center, dims, material):
        cube = UsdGeom.Cube.Define(self.stage, f"/Part/{name}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3d(*dims))
        self.surface(cube.GetPrim(), material)

    def save(self, name):
        self.stage.Export(str(ASSETS / f"{name}.usda"))


def wall_section(inner, outer, i):
    j = (i + 1) % len(inner)
    return [inner[i], outer[i], outer[j], inner[j]]


def create_cap(tool):
    a = Asset()
    a.rigid_body(tool.cap_mass)
    body = a.material("ShellPlastic", tool.pair_color)
    band = a.material("CompliantBand", tuple(0.63 * c for c in tool.pair_color), compliant=True)
    relief = outline(tool, expansion=0.65)
    exterior = outline(tool, expansion=CAP_WALL)
    interference = outline(tool, expansion=-tool.overlap)
    a.loft("ClosedEnd", [(0., exterior), (1.8, exterior)], body)
    for side in range(len(exterior)):
        shell = wall_section(relief, exterior, side)
        a.loft(f"Wall_{side:02d}", [(1.5, shell), (tool.mouth_z, shell)], body)
    for side in range(0, len(exterior), len(exterior) // 4):
        entry = wall_section(relief, exterior, side)
        land = wall_section(interference, exterior, side)
        a.loft(f"Band_{side:02d}", [
            (tool.mouth_z - 10, land), (tool.mouth_z - 4, land), (tool.mouth_z - 1, entry),
        ], band)
    a.save(tool.part_name(True))


def create_barrel(tool, low_friction=False):
    a = Asset()
    a.rigid_body(BARREL_MASS)
    plastic = a.material("BarrelPlastic", tool.barrel_color)
    tip = a.material("WritingEndPlastic", (0.14, 0.16, 0.19), friction=0.008 if low_friction else FRICTION)
    accent = a.material("PairColor", tool.pair_color)
    a.loft("WritingEnd", [
        (0., outline(tool, scale=0.27)), (3., outline(tool, scale=0.27)),
        (5., outline(tool, scale=0.66)), (9., outline(tool)), (tool.shoulder_z, outline(tool)),
    ], tip)
    grip = outline(tool, expansion=BARREL_SHOULDER)
    a.loft("Body", [(tool.shoulder_z, grip), (BARREL_LENGTH, grip)], plastic)
    stripe = outline(tool, expansion=BARREL_SHOULDER + 0.02)
    a.loft("ColorStripe", [(BARREL_LENGTH - 11, stripe), (BARREL_LENGTH - 6, stripe)],
           accent, collision=False)
    suffix = "_low_friction" if low_friction else ""
    a.save(tool.part_name(False) + suffix)


def create_rack():
    a = Asset()
    base = a.material("DeskTray", (0.86, 0.89, 0.92), friction=0.38)
    rest = a.material("PickUpRest", (0.26, 0.34, 0.42), friction=0.40)
    a.box("Base", (0.270, 0., 0.0025), (0.430, 0.260, 0.005), base)
    height = REST_Z - TABLE_Z
    for tool, cap in PARTS:
        x, y, _ = tool.reset_origin(cap)
        width = tool.grasp_width(cap) * 0.78
        pads = ((tool.cap_center, 0.016),) if cap else ((0.046, 0.008), (0.112, 0.008))
        for i, (station, length) in enumerate(pads):
            a.box(f"{tool.part_name(cap)}_{i}", (x + station, y, height / 2), (length, width, height), rest)
    a.save("pickup_tray")


def main():
    ASSETS.mkdir(exist_ok=True)
    for tool in TOOLS:
        create_barrel(tool)
        create_cap(tool)
    create_barrel(TOOLS[1], low_friction=True)
    create_rack()


if __name__ == "__main__":
    main()
