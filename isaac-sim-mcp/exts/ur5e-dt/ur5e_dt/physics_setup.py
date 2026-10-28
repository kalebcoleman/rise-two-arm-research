"""Static workstation collision proxies; stage distances are meters."""

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROBOT_START_POSITIONS_RAD = (0.0, -0.30, 0.0, 0.0, 0.0, 0.0)


def configure_robot_start(stage):
    """Drive toward a table-clear start using USD angular targets in degrees.

    Position targets stay dynamic: drives and contact response are unchanged.
    The flat zero pose drives the wrist into the table for positive lift commands.
    """
    import math
    names = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
    for name, radians in zip(names, ROBOT_START_POSITIONS_RAD):
        joint = stage.GetPrimAtPath(f'/World/UR5e/joints/{name}')
        # The separate gripper articulation and welds are authored at zero.
        # Move with drives; teleporting only the arm breaks weld consistency.
        joint.GetAttribute('state:angular:physics:position').Set(0.0)
        joint.GetAttribute('state:angular:physics:velocity').Set(0.0)
        UsdPhysics.DriveAPI(joint, 'angular').GetTargetPositionAttr().Set(math.degrees(radians))


def configure_workstation_collisions(stage):
    """Build one oriented box per CAD mesh and return the collider paths.

    Bounds retain each mesh's orientation and scale using USD row-vector matrices.
    Boxes approximate small holes and concavities; the render meshes stay unchanged.
    Repeated calls update the same proxy paths. Call with the timeline stopped, then
    restart physics before creating articulation or rigid-body tensor views.
    """
    root = stage.GetPrimAtPath('/World/Vention/Bench')
    if not root:
        raise RuntimeError('Load the Vention bench before configuring collisions')
    proxy_root = UsdGeom.Xform.Define(stage, '/World/Vention/Collisions')
    UsdGeom.Imageable(proxy_root).MakeInvisible()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    transforms = UsdGeom.XformCache()
    parent_inverse = transforms.GetLocalToWorldTransform(proxy_root.GetPrim()).GetInverse()
    material = UsdShade.Material.Define(stage, '/World/Vention/CollisionMaterial')
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(0.6)
    physics_material.CreateDynamicFrictionAttr(0.4)
    physics_material.CreateRestitutionAttr(0.0)
    paths = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        bounds = cache.ComputeUntransformedBound(prim).ComputeAlignedRange()
        size = bounds.GetSize()
        if bounds.IsEmpty() or min(size) <= 0:
            raise RuntimeError(f'Degenerate workstation mesh: {prim.GetPath()}')
        path = f'/World/Vention/Collisions/part_{len(paths):03d}'
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        matrix = (Gf.Matrix4d().SetScale(size)
                  * Gf.Matrix4d().SetTranslate(bounds.GetMidpoint())
                  * transforms.GetLocalToWorldTransform(prim) * parent_inverse)
        cube.MakeMatrixXform().Set(matrix)
        cube.GetPrim().CreateAttribute('rise:sourceMesh', Sdf.ValueTypeNames.String).Set(str(prim.GetPath()))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        collision = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        collision.CreateContactOffsetAttr(0.001)
        collision.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material, materialPurpose='physics')
        paths.append(path)
    return paths
