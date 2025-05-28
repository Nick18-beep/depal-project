# Copyright (c) 2018‑2024, NVIDIA CORPORATION. All rights reserved.








from __future__ import annotations
import asyncio
from typing import Optional, Tuple, List




from omni.isaac.kit import SimulationApp
SETUP = {"headless": False}
simulation_app = SimulationApp(SETUP)




import numpy as np
import omni
import omni.kit.commands
import omni.kit.usd
import omni.physx as _physx
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Usd
import traceback




from isaacsim.robot.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.stage import get_stage_units








def _quat_identity() -> Gf.Quatf:
    return Gf.Quatf(1.0, 0.0, 0.0, 0.0)








class SurfaceGripperDirectScript:
    def __init__(
        self,
        stage: Optional[str] = None,
        target_object_prim_path: Optional[str] = None,
        grasp_offset_from_top: float = 0.005,      
        initial_box_prim_path: str = "/Box",      
        box_mass: float = 0.05,
        absolute_grip_force: Optional[float] = None,
        grip_force_multiplier: float = 50.0,
        target_lift_height: float = 0.5,
        target_horizontal_position: np.ndarray = np.array([0.3, 0.3]),
        z_correction_factor: float = 10.0,
        max_z_correction_speed_factor: float = 1.5,
        cone_height: float = 0.1,
        cone_radius: float = 0.05,
        box_size: float = 0.1,
        lift_speed_vertical: float = 0.5,
        move_speed_horizontal: float = 0.1,
        steps_for_cube_to_settle: int = 30,
        steps_grace_period_after_settle: int = 20,
        steps_wait_before_grip_attempt: int = 100,
        steps_wait_after_grip_refresh: int = 120,
    ):
        self._timeline = omni.timeline.get_timeline_interface()
        self._usd_context = omni.usd.get_context()
        self._physx_interface = _physx.get_physx_interface()




        self.surface_gripper: Optional[Surface_Gripper] = None
        self.cone_prim_handle: Optional[SingleRigidPrim] = None
        self.target_srp_handle: Optional[SingleRigidPrim] = None
        self.cone_geom_usd: Optional[UsdGeom.Cone] = None
        self._physx_sub = None
        self._stage: Optional[omni.usd.Stage] = stage
        self._sim_step = 0
       
        self.target_object_prim_path = target_object_prim_path
        self.grasp_offset_from_top = float(grasp_offset_from_top)
        self.initial_box_prim_path = initial_box_prim_path
        self.object_to_grasp_path = target_object_prim_path if target_object_prim_path else initial_box_prim_path




        self.box_mass = float(box_mass)
        self.absolute_grip_force = float(absolute_grip_force) if absolute_grip_force is not None else None
        self.grip_force_multiplier = float(grip_force_multiplier)
        self.target_lift_height = float(target_lift_height)
        self.target_horizontal_position = np.array(target_horizontal_position, dtype=np.float64)
       
        self.z_correction_factor = float(z_correction_factor)
        self.max_z_correction_speed_factor = float(max_z_correction_speed_factor)
        self.cone_height = float(cone_height)
        self.cone_radius = float(cone_radius)
        self.box_size = float(box_size)
        self.lift_speed_vertical = float(lift_speed_vertical)
        self.move_speed_horizontal = float(move_speed_horizontal)




        self.steps_for_cube_to_settle = int(steps_for_cube_to_settle)
        self.steps_before_grace_period = self.steps_for_cube_to_settle + int(steps_grace_period_after_settle)
        self.steps_before_grip_attempt = self.steps_before_grace_period + int(steps_wait_before_grip_attempt)
        self.steps_after_grip_handle_refresh = int(steps_wait_after_grip_refresh)
        self.steps_before_initial_lift_phase = (
            self.steps_before_grip_attempt + self.steps_after_grip_handle_refresh
        )




        self.gripper_close_attempted_this_cycle = False
        self.cone_spawned_and_positioned = False
        self.cone_physics_dynamically_enabled = False # NUOVO FLAG




        self.color_closed = Gf.Vec3f(1.0, 0.2, 0.2)
        self.color_open = Gf.Vec3f(0.2, 1.0, 0.2)
       
        self.movement_phase = "idle"
        self.initial_object_position_for_move: Optional[np.ndarray] = None
        self.lift_start_z: Optional[float] = None
        self.target_descent_z: Optional[float] = None
       
        self._sgp_offset_p_local_to_cone = Gf.Vec3f(0.0, 0.0, -self.cone_radius + 0.002)
        self._sgp_offset_r_local_to_cone = Gf.Quatf(0.0, 0.0, 1.0, 0.0)




        print("SurfaceGripperDirectScript instance created with configuration:")
        if self.target_object_prim_path:
            print(f"  Target Object Prim Path: {self.target_object_prim_path}")
            print(f"  Grasp Offset From Top: {self.grasp_offset_from_top:.4f} m")
        else:
            print(f"  No specific target object, will use default box: {self.initial_box_prim_path}")
        print(f"  Box Mass (for default box or target if overridden): {self.box_mass:.3f} kg")
        if self.absolute_grip_force is not None:
            print(f"  Absolute Grip Force: {self.absolute_grip_force:.2f} N")
        else:
            min_force_calc = self.box_mass * 9.81 * self.grip_force_multiplier
            print(f"  Grip Force Multiplier: {self.grip_force_multiplier:.1f}x (Calculated Limit: {min_force_calc:.2f} N)")
        print(f"  Target Lift Height (absolute Z): {self.target_lift_height:.2f} m")
        print(f"  Target Horizontal Position: {self.target_horizontal_position}")








    def _get_target_object_grasp_pose(self, current_target_prim_path: str) -> Optional[Tuple[Gf.Vec3f, Gf.Quatf, Gf.Vec3f]]:
        if not self._stage:
            print(f"ERROR (_get_target_object_grasp_pose): Stage not available.")
            return None


        target_usd_prim = self._stage.GetPrimAtPath(current_target_prim_path)
        if not target_usd_prim.IsValid():
            print(f"ERROR (_get_target_object_grasp_pose): Target USD prim '{current_target_prim_path}' is not valid.")
            return None


        target_world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(target_usd_prim)
        target_world_position_d: Gf.Vec3d = target_world_transform.ExtractTranslation()
        target_world_rotation_quat_d: Gf.Quatd = target_world_transform.ExtractRotationQuat()
       
        target_world_position = Gf.Vec3f(target_world_position_d)
        target_world_rotation_quat = Gf.Quatf(target_world_rotation_quat_d)


        # Vettore "su" globale (normale del pavimento)
        world_up_vector = Gf.Vec3f(0.0, 0.0, 1.0)
        best_local_up_vector = Gf.Vec3f(0.0, 0.0, 1.0) # Default
        best_local_top_center_offset = Gf.Vec3f(0.0, 0.0, 0.0) # Default
        min_angle_with_world_up = 181.0  # Angolo in gradi, inizializza a un valore > 180


        prim_type = target_usd_prim.GetTypeName()
        time_code_default = Usd.TimeCode.Default()


        if target_usd_prim.IsA(UsdGeom.Cube):
            cube_geom = UsdGeom.Cube(target_usd_prim)
            size_attr = cube_geom.GetSizeAttr()
            size_val = size_attr.Get(time_code_default) if size_attr else self.box_size
            size = float(size_val) if size_val is not None else self.box_size
            half_size = size / 2.0


            # Normali locali del cubo e centri delle facce
            # (normale, offset_centro_faccia_locale)
            local_face_data = [
                (Gf.Vec3f(1.0, 0.0, 0.0), Gf.Vec3f(half_size, 0.0, 0.0)),  # +X
                (Gf.Vec3f(-1.0, 0.0, 0.0), Gf.Vec3f(-half_size, 0.0, 0.0)), # -X
                (Gf.Vec3f(0.0, 1.0, 0.0), Gf.Vec3f(0.0, half_size, 0.0)),  # +Y
                (Gf.Vec3f(0.0, -1.0, 0.0), Gf.Vec3f(0.0, -half_size, 0.0)), # -Y
                (Gf.Vec3f(0.0, 0.0, 1.0), Gf.Vec3f(0.0, 0.0, half_size)),  # +Z (default top)
                (Gf.Vec3f(0.0, 0.0, -1.0), Gf.Vec3f(0.0, 0.0, -half_size))  # -Z
            ]


            for local_normal, local_center_offset in local_face_data:
                # Trasforma la normale locale della faccia nello spazio mondo
                world_face_normal = target_world_rotation_quat.Transform(local_normal).GetNormalized()
               
                # Calcola l'angolo tra la normale della faccia (mondo) e il vettore "su" globale
                # Il prodotto scalare tra due vettori unitari è il coseno dell'angolo tra loro
                dot_product = Gf.Dot(world_face_normal, world_up_vector)
                # Assicurati che il dot_product sia nel range [-1, 1] per acos
                dot_product_clipped = max(-1.0, min(1.0, dot_product))
                angle_rad = np.arccos(dot_product_clipped)
                angle_deg = np.degrees(angle_rad)


                # Stiamo cercando la normale che punta più "in su", quindi l'angolo più piccolo
                # (se il prodotto scalare è vicino a 1, l'angolo è vicino a 0)
                if angle_deg < min_angle_with_world_up:
                    min_angle_with_world_up = angle_deg
                    best_local_up_vector = local_normal # Questa è la normale locale della faccia scelta
                    best_local_top_center_offset = local_center_offset # Questo è il centro locale della faccia scelta
           
            print(f"  Cube Grasp: Chosen local normal {best_local_up_vector}, center offset {best_local_top_center_offset}, angle with world_up: {min_angle_with_world_up:.2f} deg")
            local_top_center_offset = best_local_top_center_offset
            local_up_vector = best_local_up_vector


        # ... (gestione per Cylinder, Sphere, e fallback con Bounding Box rimane simile,
        # ma la logica di selezione della "faccia superiore" qui sarebbe più complessa o approssimata)
        elif target_usd_prim.IsA(UsdGeom.Cylinder):
            # Per il cilindro, potresti considerare le due basi e la superficie laterale.
            # Se l'asse è Z, le basi sono ovvie. Altrimenti, è più complesso.
            # Per semplicità, manteniamo la logica originale per ora.
            cyl_geom = UsdGeom.Cylinder(target_usd_prim)
            height_attr = cyl_geom.GetHeightAttr()
            height_val = height_attr.Get(time_code_default) if height_attr else self.cone_height
            height = float(height_val) if height_val is not None else self.cone_height
            axis_attr = cyl_geom.GetAxisAttr()
            axis_val = axis_attr.Get(time_code_default) if axis_attr else UsdGeom.Tokens.z
            axis = axis_val if axis_val is not None else UsdGeom.Tokens.z


            if axis == UsdGeom.Tokens.z:
                # Potremmo confrontare la normale della base superiore (0,0,1) e inferiore (0,0,-1)
                # e scegliere quella più allineata con world_up_vector.
                # Se la normale (0,0,1) trasformata è più allineata con (0,0,1) globale, usa la faccia superiore.
                # Altrimenti, usa la faccia inferiore (ma questo contraddice "presa dall'alto").
                # Per ora, si assume che se l'asse è Z, la "top" sia (0,0, height/2.0)
                local_top_center_offset = Gf.Vec3f(0.0, 0.0, height / 2.0)
                local_up_vector = Gf.Vec3f(0.0,0.0,1.0)
            elif axis == UsdGeom.Tokens.y:
                local_top_center_offset = Gf.Vec3f(0.0, height / 2.0, 0.0)
                local_up_vector = Gf.Vec3f(0.0,1.0,0.0)
            elif axis == UsdGeom.Tokens.x:
                local_top_center_offset = Gf.Vec3f(height / 2.0, 0.0, 0.0)
                local_up_vector = Gf.Vec3f(1.0,0.0,0.0)
            # Nota: per selezionare la "faccia più in alto" di un cilindro ruotato,
            # dovresti trasformare le normali delle sue basi e confrontarle con world_up_vector.


        elif target_usd_prim.IsA(UsdGeom.Sphere):
            # Per una sfera, qualsiasi punto sulla superficie è equivalente in termini di "faccia".
            # Si sceglie il punto più in alto lungo la Z globale.
            sphere_geom = UsdGeom.Sphere(target_usd_prim)
            radius_attr = sphere_geom.GetRadiusAttr()
            radius_val = radius_attr.Get(time_code_default) if radius_attr else self.cone_radius
            radius = float(radius_val) if radius_val is not None else self.cone_radius
           
            # Il punto sulla superficie della sfera più allineato con world_up_vector è
            # target_world_position + world_up_vector * radius
            # L'offset locale che, ruotato e sommato, dà questo punto è semplicemente (0,0,radius)
            # e la normale locale che, ruotata, dà world_up_vector è più complicato.
            # Semplifichiamo: la "cima" della sfera nel suo frame locale è (0,0,R) e la normale è (0,0,1).
            # La trasformazione si occuperà del resto.
            local_top_center_offset = Gf.Vec3f(0.0,0.0, radius)
            local_up_vector = Gf.Vec3f(0.0,0.0,1.0) # Normale uscente dal punto più "alto" locale.


        else: # Fallback a Bounding Box
            print(f"WARN (_get_target_object_grasp_pose): Target prim '{current_target_prim_path}' type '{prim_type}' not Cube. Using bounding box logic for 'top'.")
            boundable = UsdGeom.Boundable(target_usd_prim)
            try:
                bbox_cache = UsdGeom.BBoxCache(time_code_default, [UsdGeom.Tokens.default_] , useExtentsHint=True)
                prim_local_bound = bbox_cache.ComputeUntransformedBound(target_usd_prim)


                if prim_local_bound.GetBox().IsEmpty():
                    print(f"WARN: Could not get local bounds for generic prim {current_target_prim_path}. Assuming unit size at origin.")
                    local_top_center_offset = Gf.Vec3f(0.0,0.0,0.5) # Centro della "faccia superiore"
                    local_up_vector = Gf.Vec3f(0.0,0.0,1.0)       # Normale della "faccia superiore"
                else:
                    bbox_range_min_d = prim_local_bound.GetBox().GetMin()
                    bbox_range_max_d = prim_local_bound.GetBox().GetMax()
                    # Il centro della faccia superiore del bounding box locale
                    local_top_center_offset = Gf.Vec3f(
                        float((bbox_range_min_d[0] + bbox_range_max_d[0]) / 2.0),
                        float((bbox_range_min_d[1] + bbox_range_max_d[1]) / 2.0),
                        float(bbox_range_max_d[2]) # Z massimo del bbox locale
                    )
                    # La normale della faccia superiore del bounding box locale è sempre (0,0,1) nel frame locale del bbox.
                    local_up_vector = Gf.Vec3f(0.0,0.0,1.0)
            except Exception as e_bbox:
                print(f"ERROR computing bounding box for {current_target_prim_path}: {e_bbox}. Using fallback.")
                local_top_center_offset = Gf.Vec3f(0.0,0.0,0.5)
                local_up_vector = Gf.Vec3f(0.0,0.0,1.0)


        # Calcola il punto sulla superficie del target nel mondo (centro della faccia scelta)
        world_target_surface_point = target_world_rotation_quat.Transform(local_top_center_offset) + target_world_position
        # Calcola la normale della superficie scelta nel mondo
        world_surface_normal_of_target = target_world_rotation_quat.Transform(local_up_vector).GetNormalized()


        # Calcola l'orientamento del cono per allineare il suo asse Z con la normale della superficie scelta
        # L'asse Z locale del cono è (0,0,1)
        from_vec_cone_local_z = Gf.Vec3d(0.0, 0.0, 1.0)
        to_vec_world_surface_normal = Gf.Vec3d(world_surface_normal_of_target)
       
        rotation_to_align_cone_z_with_normal = Gf.Rotation(from_vec_cone_local_z, to_vec_world_surface_normal)
        cone_orientation_world = Gf.Quatf(rotation_to_align_cone_z_with_normal.GetQuat())


        # Il "phantom body" del gripper (dove avviene la presa)
        # self._sgp_offset_p_local_to_cone è (0,0, self.grasp_offset_from_top)
        # Se grasp_offset_from_top è 0, il phantom body è alla base del cono.
        # desired_phantom_body_origin_world deve essere sulla superficie del target se grasp_offset_from_top è 0.
        # Se grasp_offset_from_top > 0, il phantom body è "sopra" la superficie, lungo la normale.
       
        # Il punto desiderato per il phantom body del gripper nello spazio mondo
        desired_phantom_body_origin_world = world_target_surface_point + world_surface_normal_of_target * self.grasp_offset_from_top
       
        # Ora calcoliamo l'origine del cono (la sua base)
        # L'offset dal phantom body del gripper all'origine del cono, nel frame locale del cono, è -self._sgp_offset_p_local_to_cone
        # Cioè, se il phantom è a (0,0,Z_offset) rispetto alla base, la base è a (0,0,-Z_offset) rispetto al phantom
        offset_cone_origin_to_phantom_body_local = self._sgp_offset_p_local_to_cone
        offset_phantom_body_to_cone_origin_local = -offset_cone_origin_to_phantom_body_local


        # Trasforma questo offset locale (dal phantom alla base del cono) nello spazio mondo usando l'orientamento del cono
        offset_phantom_to_cone_origin_world = cone_orientation_world.Transform(offset_phantom_body_to_cone_origin_local)
       
        # L'origine del cono è la posizione del phantom body più l'offset (mondo) dal phantom alla base del cono
        cone_origin_world = desired_phantom_body_origin_world + offset_phantom_to_cone_origin_world
       
        return cone_origin_world, cone_orientation_world, world_surface_normal_of_target








    def _refresh_prim_handle(self, prim_path: str, current_handle: Optional[SingleRigidPrim]) -> Optional[SingleRigidPrim]:
        if current_handle and current_handle.is_valid():
            return current_handle
        current_stage = self._usd_context.get_stage()
        if not current_stage:
            if self._sim_step % 120 == 0: print(f"SIM STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): Stage not available.")
            return None
        usd_prim = current_stage.GetPrimAtPath(prim_path)
        if not usd_prim.IsValid():
            if self._sim_step > 10 :
                 if self._sim_step % 60 == 0: print(f"SIM STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): USD prim '{prim_path}' is NOT VALID.")
            return None
        try:
            new_handle = SingleRigidPrim(prim_path)
            if not new_handle.is_valid():
                if self._sim_step % 60 == 0: print(f"SIM STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): New SRP for '{prim_path}' immediately INVALID (USD prim was valid).")
                return None
            return new_handle
        except Exception as e_refresh:
            print(f"SIM STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): EXCEPTION: {e_refresh}")
            return None




    def _create_rigid_body_usd(
        self, body_geom_type, prim_path: str, *, mass: float, position: Gf.Vec3f,
        orientation: Gf.Quatf, color: Gf.Vec3f, height: Optional[float] = None,
        radius: Optional[float] = None, size: Optional[float] = None,
    ) -> UsdGeom.Gprim:
        if not self._stage:
            print(f"ERROR (_create_rigid_body_usd for {prim_path}): Stage not available.")
            raise RuntimeError("Stage not available for prim creation.")
           
        geom: UsdGeom.Gprim = body_geom_type.Define(self._stage, prim_path)
        prim = self._stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(geom)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
        xform.AddOrientOp().Set(Gf.Quatf(float(orientation.GetReal()), Gf.Vec3f(float(orientation.GetImaginary()[0]), float(orientation.GetImaginary()[1]), float(orientation.GetImaginary()[2]))))




        if height is not None and hasattr(geom, "CreateHeightAttr"): geom.CreateHeightAttr(float(height))
        if radius is not None and hasattr(geom, "CreateRadiusAttr"): geom.CreateRadiusAttr(float(radius))
        if size is not None and hasattr(geom, "CreateSizeAttr"): geom.CreateSizeAttr(float(size))
       
        disp_color_primvar = UsdGeom.PrimvarsAPI(geom).CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant)
        if disp_color_primvar:
            disp_color_primvar.Set([Gf.Vec3f(float(color[0]),float(color[1]),float(color[2]))])
        else:
            color_attr = geom.CreateDisplayColorAttr()
            if color_attr:
                color_attr.Set([Gf.Vec3f(float(color[0]),float(color[1]),float(color[2]))])
            else:
                print(f"WARN: Could not create DisplayColorAttr or Primvar for {prim_path}. Color might not be set.")




        UsdPhysics.CollisionAPI.Apply(prim)
        if mass > 0: # Apply RigidBodyAPI and MassAPI only if mass > 0
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        # If mass is 0, it's effectively static/kinematic by some interpretations,
        # but we'll explicitly control kinematic state later for clarity.
        # For non-positive mass, RigidBodyAPI might still be needed for joints,
        # so we apply it but rely on explicit kinematic flag or masslessness for behavior.
        # For simplicity with the current request, we ensure RigidBodyAPI is applied if mass > 0.
        # If mass is 0, it's treated as static unless kinematic is set.
        # For this script, cone mass is > 0, so RigidBodyAPI will be applied.
       
        return geom




    async def _setup_initial_scene_async(self):
        #await omni.usd.get_context().new_stage_async()
        #self._stage = self._usd_context.get_stage()
        if not self._stage:
            print("ERROR: Stage could not be retrieved after new_stage_async(). Cannot proceed.")
            return




        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
       
        scene_path = Sdf.Path("/physicsScene")
        scene = UsdPhysics.Scene.Define(self._stage, scene_path)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0,0.0,-1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
       
        omni.kit.commands.execute(
            "AddGroundPlaneCommand", stage=self._stage, planePath="/groundPlane", axis="Z",
            size=10.0, position=Gf.Vec3f(0.0,0.0,0.0), color=Gf.Vec3f(0.5,0.5,0.5)
        )
       
        light_path = Sdf.Path("/KeyLight")
        light = UsdLux.DistantLight.Define(self._stage, light_path)
        light.CreateIntensityAttr(1000)


        
        if not self.target_object_prim_path or self.target_object_prim_path == self.initial_box_prim_path :
            if not self._stage.GetPrimAtPath(self.initial_box_prim_path).IsValid():
                box_initial_center_z = self.box_size * 0.5
                self._create_rigid_body_usd(
                    UsdGeom.Cube, self.initial_box_prim_path, mass=self.box_mass,
                    position=Gf.Vec3f(0.0,0.0,box_initial_center_z),
                    orientation=_quat_identity(), color=Gf.Vec3f(0.2,0.2,1.0), size=self.box_size
                )
                print(f"Created default box at {self.initial_box_prim_path}")
            else:
                 print(f"Default box at {self.initial_box_prim_path} already exists or is the target. Not creating new.")
       
        if self.target_object_prim_path == "/World/MyTargetCube":
             if not self._stage.GetPrimAtPath("/World/MyTargetCube").IsValid():
                print("Creating /World/MyTargetCube for testing...")
                rot_x = Gf.Rotation(Gf.Vec3d(1.0,0.0,0.0), 30.0)
                rot_y = Gf.Rotation(Gf.Vec3d(0.0,1.0,0.0), 45.0)
                target_orientation_quatd = rot_y.GetQuat() * rot_x.GetQuat()
                target_orientation = Gf.Quatf(target_orientation_quatd)




                self._create_rigid_body_usd(
                    UsdGeom.Cube, "/World/MyTargetCube", mass=0.03,
                    position=Gf.Vec3f(0.3, -0.2, 0.025),
                    orientation=target_orientation,
                    color=Gf.Vec3f(1.0, 0.5, 0.0), size=0.05
                )




        set_camera_view(eye=(0.8,0.8,0.8), target=(0.0,0.0, self.box_size*0.5))
       
        self._physx_sub = self._physx_interface.subscribe_physics_step_events(self._on_simulation_step)
        self._timeline.play()
       
        print(f"Initial scene setup. Object to grasp: {self.object_to_grasp_path}. Simulation started.")
        print(f"Will wait {self.steps_for_cube_to_settle} steps before spawning cone.")








    def _spawn_and_position_cone_and_gripper(self):
        if not self._stage:
            print("WARN (_spawn_cone): Stage not ready. Skipping cone spawn.")
            return




        if not self.target_srp_handle or not self.target_srp_handle.is_valid():
            self.target_srp_handle = self._refresh_prim_handle(self.object_to_grasp_path, self.target_srp_handle)
            if not self.target_srp_handle or not self.target_srp_handle.is_valid():
                 print(f"WARN (_spawn_cone): Target object handle for '{self.object_to_grasp_path}' not ready. Skipping cone spawn.")
                 return




        cone_start_pos_gf: Gf.Vec3f
        cone_orientation_gf: Gf.Quatf
        time_code_default = Usd.TimeCode.Default()
       
        if self.target_object_prim_path:
            grasp_pose_data = self._get_target_object_grasp_pose(self.target_object_prim_path)
            if grasp_pose_data:
                cone_start_pos_gf, cone_orientation_gf, _ = grasp_pose_data
            else:
                print(f"ERROR: Could not calculate grasp pose for '{self.target_object_prim_path}'. Skipping cone spawn.")
                return
        else:
            print(f"No specific target. Using default box '{self.initial_box_prim_path}' for positioning.")
            target_pos_tuple, _ = self.target_srp_handle.get_world_pose()
            target_pos_np = np.array(target_pos_tuple, dtype=float)




            stage_units = get_stage_units()
            target_top_z_approx = target_pos_np[2] + (self.box_size / 2.0) * stage_units
            cone_origin_z = target_top_z_approx + self.grasp_offset_from_top - (self.cone_radius + 0.001)




            pos_x = float(target_pos_np[0]); pos_y = float(target_pos_np[1]); pos_z = float(cone_origin_z)
            cone_start_pos_gf = Gf.Vec3f(pos_x, pos_y, pos_z)
            cone_orientation_gf = _quat_identity()




        self._create_rigid_body_usd(
            UsdGeom.Cone, "/GripperCone", mass=0.01, position=cone_start_pos_gf,
            orientation=cone_orientation_gf, color=self.color_open,
            height=self.cone_height, radius=self.cone_radius
        )
       
        cone_prim_usd = self._stage.GetPrimAtPath("/GripperCone")
        if cone_prim_usd.IsValid():
            # MODIFICA: Rendi il cono cinematico inizialmente
            # _create_rigid_body_usd dovrebbe aver già applicato RigidBodyAPI dato che mass > 0.
            rigid_api = UsdPhysics.RigidBodyAPI(cone_prim_usd)
            if not rigid_api.GetPrim().IsValid():
                 print(f"WARN: RigidBodyAPI schema not found on {cone_prim_usd.GetPath()} after creation. Attempting to Apply it now.")
                 rigid_api = UsdPhysics.RigidBodyAPI.Apply(cone_prim_usd)


            if rigid_api.GetPrim().IsValid():
                kinematic_attr = rigid_api.CreateKinematicEnabledAttr()
                kinematic_attr.Set(True)
                print(f"Physics for {cone_prim_usd.GetPath()} initially set to KINEMATIC.")
                self.cone_physics_dynamically_enabled = False # Assicura che sia False all'inizio
            else:
                print(f"ERROR: Could not get/apply RigidBodyAPI for {cone_prim_usd.GetPath()} to set it kinematic.")
            # FINE MODIFICA


            if cone_prim_usd.IsA(UsdGeom.Cone): # Controlla prima di assegnare a self.cone_geom_usd
                self.cone_geom_usd = UsdGeom.Cone(cone_prim_usd)
            else:
                self.cone_geom_usd = None
                print(f"ERROR: {cone_prim_usd.GetPath()} prim is not a valid UsdGeom.Cone after creation.")
        else:
            self.cone_geom_usd = None
            print(f"ERROR: /GripperCone prim not valid after creation, cannot set kinematic or get geom.")


       
        sgp = Surface_Gripper_Properties()
        sgp.d6JointPath = "/GripperCone/SurfaceGripperJoint"
        sgp.parentPath = "/GripperCone" # Il cono, ora cinematico
       
        sgp.offset.p.x = float(self._sgp_offset_p_local_to_cone[0])
        sgp.offset.p.y = float(self._sgp_offset_p_local_to_cone[1])
        sgp.offset.p.z = float(self._sgp_offset_p_local_to_cone[2])
       
        sgp.offset.r.w = float(self._sgp_offset_r_local_to_cone.GetReal())
        sgp.offset.r.x = float(self._sgp_offset_r_local_to_cone.GetImaginary()[0])
        sgp.offset.r.y = float(self._sgp_offset_r_local_to_cone.GetImaginary()[1])
        sgp.offset.r.z = float(self._sgp_offset_r_local_to_cone.GetImaginary()[2])




        sgp.gripThreshold = 0.01
       
        mass_of_object_to_grip = self.box_mass
        if self.target_srp_handle and self.target_srp_handle.is_valid() and self._stage:
            try:
                mass_api = UsdPhysics.MassAPI.Get(self._stage, self.target_srp_handle.prim_path)
                if mass_api:
                    mass_attr = mass_api.GetMassAttr()
                    if mass_attr:
                         mass_val = mass_attr.Get(time_code_default)
                         if isinstance(mass_val, (float, int)) and mass_val > 0 :
                            mass_of_object_to_grip = float(mass_val)
                            print(f"Using mass from target object '{self.object_to_grasp_path}': {mass_of_object_to_grip:.3f} kg")
            except Exception as e_mass_read:
                print(f"WARN: Exception while trying to read mass from target '{self.object_to_grasp_path}': {e_mass_read}. Using default box_mass: {self.box_mass:.3f} kg")




        print(f"Target object mass for grip calculation: {mass_of_object_to_grip:.3f} kg")
       
        min_force_to_hold_box_val = 0.0
        if self.absolute_grip_force is not None:
            sgp.forceLimit = self.absolute_grip_force
            print(f"Using ABSOLUTE grip force: {sgp.forceLimit:.2f} N")
        else:
            min_force_to_hold_box_val = mass_of_object_to_grip * 9.81
            sgp.forceLimit = min_force_to_hold_box_val * self.grip_force_multiplier
            print(f"Using MULTIPLIER ({self.grip_force_multiplier}x) on min force ({min_force_to_hold_box_val:.2f} N)\n  Calculated grip force: {sgp.forceLimit:.2f} N")
       
        sgp.torqueLimit = 10.0
        sgp.stiffness = 1.0e4
        sgp.damping = 1.0e3
        sgp.retryClose = True
       
        self.surface_gripper = Surface_Gripper()
        try:
            self.surface_gripper.initialize(sgp)
            self.cone_spawned_and_positioned = True
            print("Cone and SurfaceGripper initialized.")
           
            cam_target_pos_final_gf = Gf.Vec3f(float(cone_start_pos_gf[0]), float(cone_start_pos_gf[1]), float(cone_start_pos_gf[2]))
            if self.target_srp_handle and self.target_srp_handle.is_valid():
                tp_tuple, _ = self.target_srp_handle.get_world_pose()
                tp_gf_vec = Gf.Vec3f(float(tp_tuple[0]), float(tp_tuple[1]), float(tp_tuple[2]))
                cam_target_pos_final_gf = (tp_gf_vec + cam_target_pos_final_gf) / 2.0
           
            eye_pos_tuple = (cam_target_pos_final_gf[0] + 0.5,
                             cam_target_pos_final_gf[1] + 0.5,
                             cam_target_pos_final_gf[2] + 0.5)
            cam_target_tuple = (cam_target_pos_final_gf[0],
                                cam_target_pos_final_gf[1],
                                cam_target_pos_final_gf[2])
            set_camera_view(eye=eye_pos_tuple, target=cam_target_tuple)
        except Exception as e_sg_init:
            print(f"ERROR: SurfaceGripper initialization failed: {e_sg_init}")
            traceback.print_exc()
            self.cone_spawned_and_positioned = False








    def _on_simulation_step(self, dt: float):
        self._sim_step += 1
        time_code_default = Usd.TimeCode.Default()




        self.target_srp_handle = self._refresh_prim_handle(self.object_to_grasp_path, self.target_srp_handle)
        if not (self.target_srp_handle and self.target_srp_handle.is_valid()):
             if self._sim_step % 60 == 0: print(f"SIM STEP {self._sim_step}: Target object '{self.object_to_grasp_path}' handle not valid. Waiting.")
             return




        if not self.cone_spawned_and_positioned and self._sim_step >= self.steps_for_cube_to_settle:
            self._spawn_and_position_cone_and_gripper()
            if not self.cone_spawned_and_positioned:
                if self._sim_step % 60 == 0: print(f"SIM STEP {self._sim_step}: Cone spawning/gripper init failed or deferred. Will retry.")
                return
       
        if self.cone_spawned_and_positioned: # Refresh cone handle if cone exists
            self.cone_prim_handle = self._refresh_prim_handle("/GripperCone", self.cone_prim_handle)




        if not self.cone_spawned_and_positioned: # Return if cone still not ready
            return
       
        if self._sim_step < self.steps_before_grace_period:
            if self._sim_step == self.steps_for_cube_to_settle + 1 and self.cone_spawned_and_positioned:
                 print(f"SIM STEP {self._sim_step}: Cone spawned (kinematic). Grace period until step {self.steps_before_grace_period}.")
            return
        elif self._sim_step == self.steps_before_grace_period:
            print(f"SIM STEP {self._sim_step}: Grace period finished. Gripper logic active.")




        if self.surface_gripper:
            try:
                self.surface_gripper.update()
            except Exception as e_update:
                print(f"SIM STEP {self._sim_step}: ERROR during surface_gripper.update(): {e_update}"); return




            if self.cone_geom_usd and self.cone_geom_usd.GetPrim().IsValid():
                try:
                    disp_color_primvar = UsdGeom.PrimvarsAPI(self.cone_geom_usd).GetPrimvar("displayColor")
                    if disp_color_primvar:
                        current_color_gf = self.color_closed if self.surface_gripper.is_closed() else self.color_open
                        disp_color_primvar.Set([current_color_gf], time_code_default)
                except Exception as e_color:
                     if self._sim_step % 60 == 0: print(f"SIM STEP {self._sim_step}: Error setting cone color: {e_color}")
       
        if self._sim_step == self.steps_before_grip_attempt and not self.gripper_close_attempted_this_cycle:
            print(f"SIM STEP {self._sim_step}: Attempting to close the gripper...")
            if self.surface_gripper:
                try:
                    self.surface_gripper.close()
                    self.gripper_close_attempted_this_cycle = True
                    print(f"    INFO: surface_gripper.close() called.")
                    # self.cone_prim_handle = None # Commentato: non invalidare qui, potrebbe servire subito
                except Exception as e_close:
                    print(f"    ERROR: surface_gripper.close() failed → {e_close}")
            else:
                print(f"    WARN: surface_gripper object is None at grip attempt.")


        # MODIFICA: Riabilita la fisica del cono (rendilo dinamico) quando la presa è chiusa
        if self.gripper_close_attempted_this_cycle and \
           not self.cone_physics_dynamically_enabled and \
           self.surface_gripper and self.surface_gripper.is_closed():
           
            print(f"SIM STEP {self._sim_step}: Gripper confirmed closed. Attempting to set cone physics to DYNAMIC.")
           
            cone_prim_usd = None
            cone_path_str = "/GripperCone" # Default path
            if self.cone_prim_handle and self.cone_prim_handle.is_valid():
                # Preferisci il path dall'handle SRP se disponibile, potrebbe essere più robusto a rinominazioni/namespace
                cone_path_str = self.cone_prim_handle.prim_path
                cone_prim_usd = self._stage.GetPrimAtPath(cone_path_str)
            elif self._stage:
                cone_prim_usd = self._stage.GetPrimAtPath(cone_path_str)


            if cone_prim_usd and cone_prim_usd.IsValid():
                rigid_api = UsdPhysics.RigidBodyAPI(cone_prim_usd)
                if not rigid_api.GetPrim().IsValid():
                    print(f"    WARN: RigidBodyAPI schema not found on {cone_prim_usd.GetPath()} when trying to set dynamic. Applying.")
                    rigid_api = UsdPhysics.RigidBodyAPI.Apply(cone_prim_usd)


                if rigid_api.GetPrim().IsValid():
                    kinematic_attr = rigid_api.CreateKinematicEnabledAttr()
                    current_kinematic_state = kinematic_attr.Get()
                    if current_kinematic_state is True:
                        kinematic_attr.Set(False)
                        print(f"    SUCCESS: Cone physics for {cone_prim_usd.GetPath()} set to DYNAMIC.")
                        self.cone_physics_dynamically_enabled = True
                    elif current_kinematic_state is False:
                        print(f"    INFO: Cone physics for {cone_prim_usd.GetPath()} was already DYNAMIC.")
                        self.cone_physics_dynamically_enabled = True
                    else:
                        print(f"    WARN: Kinematic attribute for {cone_prim_usd.GetPath()} in unexpected state ({current_kinematic_state}). Forcing DYNAMIC.")
                        kinematic_attr.Set(False)
                        self.cone_physics_dynamically_enabled = True
                else:
                    print(f"    ERROR: Could not get/apply RigidBodyAPI for {cone_prim_usd.GetPath()} to enable dynamic physics.")
            else:
                print(f"    ERROR: Cone prim at path '{cone_path_str}' not found or invalid when trying to enable dynamic physics.")
        # FINE MODIFICA




        if self._sim_step == self.steps_before_initial_lift_phase:
            sg_status = "N/A"
            if self.surface_gripper: sg_status = "Closed" if self.surface_gripper.is_closed() else "Open"
            cph_valid_str = "N/A"
            if self.cone_prim_handle: cph_valid_str = "Valid" if self.cone_prim_handle.is_valid() else "Invalid"
            cone_physics_str = "Dynamic" if self.cone_physics_dynamically_enabled else "Kinematic/NotYetDynamic"
            print(f"LIFT CHECK (Step {self._sim_step}): Phase='{self.movement_phase}', Gripper State='{sg_status}', Cone Handle='{cph_valid_str}', Cone Physics='{cone_physics_str}'")




        can_move = self.surface_gripper and self.surface_gripper.is_closed() and \
                   self.cone_prim_handle and self.cone_prim_handle.is_valid() and \
                   self.cone_physics_dynamically_enabled # MODIFICA: Aggiunto controllo




        if self.movement_phase == "idle" and \
           self._sim_step >= self.steps_before_initial_lift_phase and \
           can_move: # can_move ora verifica anche lo stato dinamico del cono
            try:
                current_cone_pos_tuple, _ = self.cone_prim_handle.get_world_pose()
                current_cone_pos_np = np.array(current_cone_pos_tuple, dtype=float)
                self.movement_phase = "lifting"
                self.lift_start_z = float(current_cone_pos_np[2])
                print(f"SIM STEP {self._sim_step} (Movement Start): Gripper closed, Cone DYNAMIC. Phase: LIFTING. Start Z: {self.lift_start_z:.4f}, Target Lift Z (absolute): {self.target_lift_height:.4f}")
            except RuntimeError as e_get_pose_idle:
                print(f"SIM STEP {self._sim_step} (Movement Idle->Lifting): RuntimeError getting cone pose: {e_get_pose_idle}. Invalidating handle.")
                self.cone_prim_handle = None
       
        if self.movement_phase in ["lifting", "moving_horizontal", "descending", "releasing"]:
            if not (self.cone_prim_handle and self.cone_prim_handle.is_valid()):
                if self.movement_phase != "releasing": # Non loggare errore se stiamo già rilasciando
                    print(f"SIM STEP {self._sim_step} (Movement - Phase {self.movement_phase}): cone_prim_handle invalid. Aborting move to 'done'.")
                self.movement_phase = "done" # Vai a 'done' se l'handle non è valido
                return
           
            # Assicurati ulteriormente che il cono sia dinamico prima di tentare di muoverlo con la fisica
            if not self.cone_physics_dynamically_enabled and self.movement_phase not in ["releasing", "done"]:
                print(f"SIM STEP {self._sim_step} (Movement - Phase {self.movement_phase}): Cone is NOT DYNAMIC. Cannot move with physics. Aborting to 'releasing'.")
                self.movement_phase = "releasing" # Salta a rilasciare se il cono non è dinamico




            current_cone_pos_np: Optional[np.ndarray] = None
            try:
                current_cone_pos_tuple, _ = self.cone_prim_handle.get_world_pose()
                current_cone_pos_np = np.array(current_cone_pos_tuple, dtype=float)
            except RuntimeError as e_get_pose_active:
                print(f"SIM STEP {self._sim_step} (Movement - Phase {self.movement_phase}): RuntimeError getting cone pose: {e_get_pose_active}. Invalidating handle. To 'done'.")
                self.cone_prim_handle = None; self.movement_phase = "done"
                return




            if current_cone_pos_np is not None: # Solo se la posa è stata ottenuta
                if self.surface_gripper and not self.surface_gripper.is_closed() and self.movement_phase not in ["releasing", "done"]:
                    print(f"SIM STEP {self._sim_step} (Movement - Phase {self.movement_phase}): Gripper opened unexpectedly! Aborting to 'releasing'.")
                    self.movement_phase = "releasing"
                    try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0,0.0], dtype=np.float32))
                    except RuntimeError as e: print(f" MOVEMENT ABORT STOP ERR: {e}"); self.cone_prim_handle = None; self.movement_phase = "done"




                if self.movement_phase == "lifting":
                    grip_stat = "Closed" if self.surface_gripper and self.surface_gripper.is_closed() else "Open/None"
                    if self._sim_step % 10 == 0 : print(f"    LIFTING (Step {self._sim_step}, Grip: {grip_stat}): Z: {current_cone_pos_np[2]:.4f}, Target Z: {self.target_lift_height:.4f}")
                    if current_cone_pos_np[2] < self.target_lift_height:
                        try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0,self.lift_speed_vertical],dtype=np.float32))
                        except RuntimeError as e: print(f" LIFTING ERR: {e}"); self.cone_prim_handle=None; self.movement_phase="done"
                    else:
                        print(f"SIM STEP {self._sim_step} (Movement): Reached lift height. Phase: MOVING_HORIZONTAL.")
                        self.movement_phase = "moving_horizontal"
                        try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0,0.0],dtype=np.float32))
                        except RuntimeError as e: print(f" LIFTING STOP ERR: {e}"); self.cone_prim_handle=None; self.movement_phase="done"
                   
                elif self.movement_phase == "moving_horizontal":
                    target_xy_np = self.target_horizontal_position
                    current_xy_np = current_cone_pos_np[:2]
                    diff_xy_np = target_xy_np - current_xy_np
                    dist_xy = float(np.linalg.norm(diff_xy_np))
                    grip_stat = "Closed" if self.surface_gripper and self.surface_gripper.is_closed() else "Open/None"




                    if self._sim_step % 10 == 0:
                         print(f"    MOVING_H (Step {self._sim_step}, Grip: {grip_stat}): TargetXY: {target_xy_np.round(3)}, CurrentXY: {current_xy_np.round(3)}, DistXY: {dist_xy:.3f}, TargetZ (Hold): {self.target_lift_height:.4f}, CurrentZ: {current_cone_pos_np[2]:.4f}")




                    if dist_xy > 0.02:
                        direction_xy_np = diff_xy_np / dist_xy
                        vel_xy_target_np = direction_xy_np * self.move_speed_horizontal
                       
                        z_error = self.target_lift_height - current_cone_pos_np[2]
                        vel_z_correction = self.z_correction_factor * z_error
                       
                        max_z_speed = self.lift_speed_vertical * self.max_z_correction_speed_factor
                        vel_z_correction = float(np.clip(vel_z_correction, -max_z_speed, max_z_speed))




                        if self._sim_step % 10 == 0 and abs(z_error) > 0.001:
                            print(f"      MOVING_H Z Correction: Error={z_error:.4f}, CorrectionVel={vel_z_correction:.4f}")
                       
                        try:
                            self.cone_prim_handle.set_linear_velocity(
                                np.array([vel_xy_target_np[0], vel_xy_target_np[1], vel_z_correction], dtype=np.float32)
                            )
                        except RuntimeError as e:
                            print(f" MOVING_H ERR (with Z correction): {e}")
                            self.cone_prim_handle = None; self.movement_phase = "done"
                    else:
                        print(f"SIM STEP {self._sim_step} (Movement): Reached horizontal target (DistXY: {dist_xy:.4f}). Phase: DESCENDING.")
                        self.movement_phase = "descending"
                        if self.lift_start_z is None:
                            print(f"    WARNING: lift_start_z is None before descending. Using current cone Z ({current_cone_pos_np[2]:.4f}) for target_descent_z.")
                            self.target_descent_z = float(current_cone_pos_np[2])
                        else:
                            self.target_descent_z = self.lift_start_z
                       
                        print(f"    MOVING_H -> DESCENDING: Current Z: {current_cone_pos_np[2]:.4f}, Target Descent Z: {self.target_descent_z:.4f}")
                        try:
                            self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0,0.0],dtype=np.float32))
                        except RuntimeError as e:
                            print(f" MOVING_H STOP ERR: {e}")
                            self.cone_prim_handle = None; self.movement_phase = "done"
               
                elif self.movement_phase == "descending":
                    grip_stat = "Closed" if self.surface_gripper and self.surface_gripper.is_closed() else "Open/None"
                    if self.target_descent_z is None:
                        print(f"    DESCENDING (Step {self._sim_step}, Grip: {grip_stat}): target_descent_z is None. Critical error. Skipping to RELEASING.")
                        self.movement_phase = "releasing"
                        try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0,0.0],dtype=np.float32))
                        except RuntimeError as e: print(f" DESCENDING STOP ERR (target None): {e}"); self.cone_prim_handle=None; self.movement_phase="done"
                    elif current_cone_pos_np[2] > self.target_descent_z + 0.005: # Buffer per evitare overshooting
                        if self._sim_step % 10 == 0: print(f"    DESCENDING (Step {self._sim_step}, Grip: {grip_stat}): Current Z: {current_cone_pos_np[2]:.4f}, Target Z: {self.target_descent_z:.4f}")
                        try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0, -self.lift_speed_vertical],dtype=np.float32))
                        except RuntimeError as e: print(f" DESCENDING ERR: {e}"); self.cone_prim_handle=None; self.movement_phase="done"
                    else:
                        print(f"SIM STEP {self._sim_step} (Movement): Reached descent height (Z: {current_cone_pos_np[2]:.4f}). Phase: RELEASING.")
                        self.movement_phase = "releasing"
                        try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0,0.0],dtype=np.float32))
                        except RuntimeError as e: print(f" DESCENDING STOP ERR: {e}"); self.cone_prim_handle=None; self.movement_phase="done"




                elif self.movement_phase == "releasing":
                    print(f"SIM STEP {self._sim_step} (Movement): Phase RELEASING. Opening gripper.")
                    if self.surface_gripper: self.surface_gripper.open()
                    # Potresti voler rendere di nuovo il cono cinematico qui se necessario per il ciclo successivo,
                    # ma per un singolo run, "done" è sufficiente.
                    self.movement_phase = "done"
       
        if self.movement_phase == "done":
            if self.cone_prim_handle and self.cone_prim_handle.is_valid():
                try:
                    # Assicurati che il cono sia fermo se è diventato dinamico
                    if self.cone_physics_dynamically_enabled:
                        _, current_lin_vel_tuple, _ = self.cone_prim_handle.get_velocities()
                        current_lin_vel_np = np.array(current_lin_vel_tuple, dtype=float)
                        if np.linalg.norm(current_lin_vel_np) > 0.01:
                            if self._sim_step % 30 == 0: print(f"    DONE (Step {self._sim_step}): Cone still moving (vel: {current_lin_vel_np.round(3)}). Stopping.")
                            self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0,0.0],dtype=np.float32))
                except RuntimeError:
                    self.cone_prim_handle = None # Handle non valido
                except Exception as e_done_stop:
                    if self._sim_step % 60 == 0: print(f"    DONE (Step {self._sim_step}): Exception while trying to stop cone: {e_done_stop}")
           
            if self._sim_step > self.steps_before_initial_lift_phase + 500 and self._sim_step % 120 == 0 :
                 cone_physics_final_state = "Dynamic" if self.cone_physics_dynamically_enabled else "Kinematic/Initial"
                 print(f"SIM STEP {self._sim_step}: Movement phase is 'done'. Cone physics state: {cone_physics_final_state}. Simulation continues.")








    def run(self):
        print("SurfaceGripperDirectScript.run() called.")
        if self._timeline.is_playing():
            self._timeline.stop()
       
        self._sim_step = 0
        self.gripper_close_attempted_this_cycle = False
        self.cone_spawned_and_positioned = False
        self.cone_physics_dynamically_enabled = False # MODIFICA: Resetta il flag
        self.movement_phase = "idle"
        self.initial_object_position_for_move = None
        self.lift_start_z = None
        self.target_descent_z = None
       
        self.cone_prim_handle = None
        self.target_srp_handle = None
        self.cone_geom_usd = None
        if self.surface_gripper: self.surface_gripper = None # Ricrea il gripper
       
        if self._physx_sub:
            self._physx_sub.unsubscribe()
            self._physx_sub = None




        asyncio.ensure_future(self._setup_initial_scene_async())




    def cleanup(self):
        print("Cleaning up SurfaceGripperDirectScript instance...")
        if self._timeline and self._timeline.is_playing(): self._timeline.stop()
       
        if self._physx_sub:
            self._physx_sub.unsubscribe(); self._physx_sub = None
       
        self.surface_gripper = None; self.cone_prim_handle = None; self.target_srp_handle = None
        self.cone_geom_usd = None; self._stage = None; self._sim_step = 0
        self.gripper_close_attempted_this_cycle = False; self.cone_spawned_and_positioned = False
        self.cone_physics_dynamically_enabled = False # MODIFICA: Resetta il flag
        self.movement_phase = "idle"; self.initial_object_position_for_move = None
        self.lift_start_z = None; self.target_descent_z = None
        print("SurfaceGripperDirectScript cleanup complete.")




# --- SCRIPT EXECUTION LOGIC ---
if "gripper_script_runner_instance" in globals() and gripper_script_runner_instance is not None:
    print("Cleaning up previous SurfaceGripperDirectScript instance...")
    try:
        # Chiamare il metodo cleanup dell'istanza esistente
        # Assicurarsi che cleanup sia definito per prevenire AttributeError se l'oggetto è parzialmente formato
        if hasattr(gripper_script_runner_instance, 'cleanup') and callable(gripper_script_runner_instance.cleanup):
            gripper_script_runner_instance.cleanup()
    except Exception as e_cleanup:
        print(f"Error during cleanup of previous instance: {e_cleanup}")
    # Rimuovi il riferimento all'istanza precedente
    try:
        del gripper_script_runner_instance
    except NameError:
        pass # Ignora se già eliminato o non definito
    gripper_script_runner_instance = None # Assicura che sia None

print("--- CONFIGURATION FOR NEW SIMULATION ---")


# Ottieni lo stage attivo
stage = omni.usd.get_context().get_stage()






config_target_prim_to_grasp = "/World/MyTargetCube"
config_grasp_offset = 0.000


config_initial_box_path = "/Box"
config_box_mass = 0.020
config_absolute_grip_force = 200.0 # Assicurati sia float
config_grip_force_multiplier = 500.0
config_lift_speed_vertical = 1 # Modificato per un movimento più lento/controllato




config_target_lift_height = 0.3
config_target_horizontal_position = np.array([0, 0]) # posizione finale cubo
config_z_correction_factor = 15.0
config_max_z_correction_speed_factor = 2.0
config_cone_height = 0.05
config_cone_radius = 0.03
config_box_size = 0.08
config_move_speed_horizontal = 0.1




config_steps_for_cube_to_settle = 60
config_steps_grace_period_after_settle = 30
config_steps_wait_before_grip_attempt = 120
config_steps_wait_after_grip_refresh = 40




print(f"  Config: Target Prim='{config_target_prim_to_grasp if config_target_prim_to_grasp else f'Default Box ({config_initial_box_path})'}'")
print(f"  Config: Grasp Offset={config_grasp_offset:.4f}m, Absolute Grip Force={config_absolute_grip_force if config_absolute_grip_force is not None else 'N/A (Using Multiplier)'}, Lift Speed={config_lift_speed_vertical}m/s")




print("\nCreating new SurfaceGripperDirectScript instance...")
gripper_script_runner_instance = SurfaceGripperDirectScript(
    stage=stage,
    target_object_prim_path=config_target_prim_to_grasp,
    grasp_offset_from_top=config_grasp_offset,
    initial_box_prim_path=config_initial_box_path,
    box_mass=config_box_mass,
    absolute_grip_force=config_absolute_grip_force,
    grip_force_multiplier=config_grip_force_multiplier,
    target_lift_height=config_target_lift_height,
    target_horizontal_position=config_target_horizontal_position,
    z_correction_factor=config_z_correction_factor,
    max_z_correction_speed_factor=config_max_z_correction_speed_factor,
    cone_height=config_cone_height,
    cone_radius=config_cone_radius,
    box_size=config_box_size,
    lift_speed_vertical=config_lift_speed_vertical,
    move_speed_horizontal=config_move_speed_horizontal,
    steps_for_cube_to_settle=config_steps_for_cube_to_settle,
    steps_grace_period_after_settle=config_steps_grace_period_after_settle,
    steps_wait_before_grip_attempt=config_steps_wait_before_grip_attempt,
    steps_wait_after_grip_refresh=config_steps_wait_after_grip_refresh
)




print("Running simulation script...")
gripper_script_runner_instance.run()










while True:
    simulation_app.update()
print("--- Script execution initiated ---")








