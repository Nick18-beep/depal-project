from __future__ import annotations
from typing import Optional, Tuple, List

import numpy as np
import omni
import omni.kit.usd
import omni.physx as _physx
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Usd
import traceback

from isaacsim.robot.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)
from isaacsim.core.prims import SingleRigidPrim

def _quat_identity() -> Gf.Quatf:
    return Gf.Quatf(1.0, 0.0, 0.0, 0.0)

class SurfaceGripperDirectScript:
    def __init__(
        self,
        target_object_prim_path: Optional[str] = None,
        grasp_offset_from_top: float = 0.000, 
        gripper_depth_offset: float = 0.005, 
        initial_box_prim_path: str = "/World/Box", 
        box_mass: float = 0.05, 
        absolute_grip_force: Optional[float] = None,
        grip_force_multiplier: float = 50.0,
        target_lift_height: float = 0.5,
        target_horizontal_position: np.ndarray = np.array([0.3, 0.3]),
        z_correction_factor: float = 10.0,
        max_z_correction_speed_factor: float = 1.5,
        cone_height: float = 0.1,
        cone_radius: float = 0.05,
        lift_speed_vertical: float = 0.5,
        move_speed_horizontal: float = 0.1,
        steps_for_cube_to_settle: int = 60,
        steps_grace_period_after_settle: int = 30,
        steps_wait_before_grip_attempt: int = 120,
        steps_wait_after_cone_usd_creation: int = 5,
        steps_wait_after_grip_refresh: int = 40,
        debug_pose_calculation: bool = True
    ):
        self._timeline = omni.timeline.get_timeline_interface()
        self._usd_context = omni.usd.get_context()
        self._physx_interface = _physx.get_physx_interface()
        self._stage: Optional[omni.usd.Stage] = None

        self.surface_gripper: Optional[Surface_Gripper] = None
        self.cone_prim_handle: Optional[SingleRigidPrim] = None
        self.target_srp_handle: Optional[SingleRigidPrim] = None
        self.cone_geom_usd: Optional[UsdGeom.Cone] = None
        self._physx_sub = None
        self._sim_step: int = 0
        self.debug_pose_calculation = debug_pose_calculation
        
        self.fallback_box_size = 0.1 # Usato solo se le dimensioni non possono essere determinate

        self.target_object_prim_path = target_object_prim_path
        self.grasp_offset_from_top = float(grasp_offset_from_top)
        self.initial_box_prim_path = initial_box_prim_path
        self.object_to_grasp_path = target_object_prim_path if target_object_prim_path else self.initial_box_prim_path

        self.box_mass = float(box_mass) 
        self.absolute_grip_force = float(absolute_grip_force) if absolute_grip_force is not None else None
        self.grip_force_multiplier = float(grip_force_multiplier)
        self.target_lift_height = float(target_lift_height)
        self.target_horizontal_position = np.array(target_horizontal_position, dtype=np.float64)

        self.z_correction_factor = float(z_correction_factor)
        self.max_z_correction_speed_factor = float(max_z_correction_speed_factor)
        self.cone_height = float(cone_height)
        self.cone_radius = float(cone_radius)
        self.lift_speed_vertical = float(lift_speed_vertical)
        self.move_speed_horizontal = float(move_speed_horizontal)

        self.steps_for_cube_to_settle = int(steps_for_cube_to_settle)
        self.steps_end_settle = self.steps_for_cube_to_settle
        self.steps_end_grace_period = self.steps_end_settle + int(steps_grace_period_after_settle)
        self.steps_for_cone_usd_creation_attempt = self.steps_end_grace_period + int(steps_wait_before_grip_attempt)
        self.steps_wait_after_cone_usd_creation = int(steps_wait_after_cone_usd_creation)
        self.steps_for_sg_init_and_grip_attempt = self.steps_for_cone_usd_creation_attempt + self.steps_wait_after_cone_usd_creation
        self.steps_before_initial_lift_phase = self.steps_for_sg_init_and_grip_attempt + int(steps_wait_after_grip_refresh)

        self.gripper_close_command_sent: bool = False
        self.cone_usd_prim_created: bool = False
        self.cone_sg_initialized: bool = False
        self.cone_physics_dynamically_enabled: bool = False

        self.color_closed = Gf.Vec3f(1.0, 0.2, 0.2)
        self.color_open = Gf.Vec3f(0.2, 1.0, 0.2)

        self.movement_phase: str = "idle"
        self.lift_start_z: Optional[float] = None
        self.target_descent_z: Optional[float] = None

        self._sgp_offset_p_local_to_cone = Gf.Vec3f(0.0, 0.0, float(gripper_depth_offset))
        rotation_as_quatd = Gf.Rotation(Gf.Vec3d(0,1,0), 90.0).GetQuat() 
        real_part_sgp = float(rotation_as_quatd.GetReal())
        imag_part_sgp_d = rotation_as_quatd.GetImaginary() 
        self._sgp_offset_r_local_to_cone = Gf.Quatf(
            real_part_sgp,
            Gf.Vec3f(float(imag_part_sgp_d[0]),
                     float(imag_part_sgp_d[1]),
                     float(imag_part_sgp_d[2]))
        )

        print("SurfaceGripperDirectScript instance created:")
        print(f"  Target: {self.object_to_grasp_path}")
        print(f"  Cone Base to Surface Offset (grasp_offset_from_top): {self.grasp_offset_from_top:.4f} m")
        print(f"  SGP offset local to cone (position, in cone frame Z up): {self._sgp_offset_p_local_to_cone}")
        sgp_rot_img_init = self._sgp_offset_r_local_to_cone.GetImaginary()
        sgp_euler_deg = Gf.Rotation(self._sgp_offset_r_local_to_cone).Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())[2]
        print(f"  SGP offset local to cone (rotation quat WXYZ): {self._sgp_offset_r_local_to_cone.GetReal():.4f}, {sgp_rot_img_init[0]:.4f}, {sgp_rot_img_init[1]:.4f}, {sgp_rot_img_init[2]:.4f} (Euler XYZ deg ~ {sgp_euler_deg})")
        print(f"  Timing: Settle: {self.steps_end_settle}, End Grace: {self.steps_end_grace_period}")
        print(f"  Timing: Cone USD Creation Attempt: {self.steps_for_cone_usd_creation_attempt}")
        print(f"  Timing: SG Init & Grip Attempt: {self.steps_for_sg_init_and_grip_attempt} (after {self.steps_wait_after_cone_usd_creation} steps for cone USD to settle)")
        print(f"  Timing: Lift Start: {self.steps_before_initial_lift_phase}")



    def _get_target_object_grasp_pose(self, current_target_prim_path: str) -> Optional[Tuple[Gf.Vec3f, Gf.Quatf, Gf.Vec3f]]:
        """
        Calcola la posa di grasp basata sul bounding box locale del target,
        applicando tutte le trasformazioni (rotazioni, scale, traslazioni USD)
        e selezionando la faccia con la normale maggiormente orientata verso +Z.
        Posiziona il cono in modo che la sua base tocchi la faccia.
        Restituisce: (origine_cono_world, orientazione_cono_world, normale_surface_world)
        """
        if self.debug_pose_calculation:
            print(f"\n--- Debug _get_target_object_grasp_pose per '{current_target_prim_path}' step {self._sim_step} ---")

        if not self._stage:
            print("ERROR (_get_target_object_grasp_pose): Stage non disponibile.")
            return None

        input_target_prim = self._stage.GetPrimAtPath(current_target_prim_path)
        if not input_target_prim or not input_target_prim.IsValid():
            print(f"ERROR (_get_target_object_grasp_pose): Prim '{current_target_prim_path}' non valido.")
            return None
            
        # Determina il prim effettivo su cui calcolare la geometria (potrebbe essere un figlio)
        geom_prim_for_calc = input_target_prim
        if input_target_prim.IsA(UsdGeom.Xform) and not input_target_prim.IsA(UsdGeom.Gprim):
            if self.debug_pose_calculation: print(f"  Input target '{input_target_prim.GetPath()}' is Xform. Searching for Gprim child...")
            found_gprim_child = None
            for child in input_target_prim.GetChildren():
                if child.IsA(UsdGeom.Boundable): # UsdGeom.Gprim eredita da Boundable
                    imgbl = UsdGeom.Imageable(child)
                    purpose = imgbl.GetPurposeAttr().Get()
                    if purpose in [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide]:
                        found_gprim_child = child
                        if self.debug_pose_calculation: print(f"    Found Boundable child: {child.GetPath()} with purpose '{purpose}'")
                        break 
            if found_gprim_child:
                geom_prim_for_calc = found_gprim_child
            else:
                if self.debug_pose_calculation: print(f"    No suitable Gprim child found under Xform '{input_target_prim.GetPath()}'. Using Xform itself for bbox calculation (might be inaccurate if it has no extent).")
        
        if self.debug_pose_calculation: print(f"  Using prim '{geom_prim_for_calc.GetPath()}' (type: {geom_prim_for_calc.GetTypeName()}) for geometry and BBox calculations.")

        # Bounding box di geom_prim_for_calc NEL SUO FRAME LOCALE
        # Questo bound NON include la trasformazione di geom_prim_for_calc stesso.
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=True, ignoreVisibility=False)
        geom_local_bbox_range = bbox_cache.ComputeUntransformedBound(geom_prim_for_calc).GetBox()

        if geom_local_bbox_range.IsEmpty():
            boundable_geom = UsdGeom.Boundable(geom_prim_for_calc)
            extent_attr = boundable_geom.GetExtentAttr()
            if extent_attr and extent_attr.HasValue():
                extents_array = extent_attr.Get()
                if extents_array and len(extents_array) == 2:
                    geom_local_bbox_range = Gf.Range3d(Gf.Vec3d(extents_array[0]), Gf.Vec3d(extents_array[1]))
                    if self.debug_pose_calculation: print(f"    BBox was empty, used extents for '{geom_prim_for_calc.GetPath()}': {geom_local_bbox_range}")
            
            if geom_local_bbox_range.IsEmpty():
                print(f"ERROR: BBox and extents for '{geom_prim_for_calc.GetPath()}' are unusable.")
                return None
        
        min_l_geom = geom_local_bbox_range.GetMin()  # Gf.Vec3d
        max_l_geom = geom_local_bbox_range.GetMax()  # Gf.Vec3d
        center_l_geom = (min_l_geom + max_l_geom) * 0.5 # Gf.Vec3d
        
        if self.debug_pose_calculation:
            dims_geom = max_l_geom - min_l_geom
            print(f"  BBox of '{geom_prim_for_calc.GetPath()}' (in its own local frame, untransformed): min={min_l_geom}, max={max_l_geom}, center={center_l_geom}, dimensions={dims_geom}")

        # Definizione facce (normali e offset dei centri) nel frame locale di geom_prim_for_calc
        faces_in_geom_local_frame = [
            (f"Face +X", Gf.Vec3f(1,0,0), Gf.Vec3f(float(max_l_geom[0]), float(center_l_geom[1]), float(center_l_geom[2]))),
            (f"Face -X", Gf.Vec3f(-1,0,0),Gf.Vec3f(float(min_l_geom[0]), float(center_l_geom[1]), float(center_l_geom[2]))),
            (f"Face +Y", Gf.Vec3f(0,1,0), Gf.Vec3f(float(center_l_geom[0]), float(max_l_geom[1]), float(center_l_geom[2]))),
            (f"Face -Y", Gf.Vec3f(0,-1,0),Gf.Vec3f(float(center_l_geom[0]), float(min_l_geom[1]), float(center_l_geom[2]))),
            (f"Face +Z", Gf.Vec3f(0,0,1), Gf.Vec3f(float(center_l_geom[0]), float(center_l_geom[1]), float(max_l_geom[2]))),
            (f"Face -Z", Gf.Vec3f(0,0,-1),Gf.Vec3f(float(center_l_geom[0]), float(center_l_geom[1]), float(min_l_geom[2])))
        ]

        # Ottieni la trasformazione MONDIALE di geom_prim_for_calc
        # Questa matrice include tutte le trasformazioni (scale, rot, trans) di geom_prim_for_calc e dei suoi parenti.
        W_geom_to_world = omni.usd.get_world_transform_matrix(geom_prim_for_calc)
        if self.debug_pose_calculation:
            print(f"  World transform matrix of geom_prim '{geom_prim_for_calc.GetPath()}':\n{W_geom_to_world}")


        world_up_vector = Gf.Vec3f(0,0,1)
        best_face_data = {
            "surface_point_world": None,
            "surface_normal_world": None,
            "dot_world_z": -2.0,          
            "name": "None"
        }

        if self.debug_pose_calculation: print("  Evaluating faces for grasp (all transformations applied):")
        for face_name, n_geom_local, off_geom_local in faces_in_geom_local_frame:
            # n_geom_local e off_geom_local sono nel frame locale (untransformed) di geom_prim_for_calc
            # Trasformali direttamente nel mondo usando W_geom_to_world
            
            # W_geom_to_world.TransformDir non applica la traslazione ed è corretto per le normali.
            # È importante che W_geom_to_world includa anche lo scale, se presente.
            n_world_d = W_geom_to_world.TransformDir(Gf.Vec3d(n_geom_local)).GetNormalized()
            n_world = Gf.Vec3f(n_world_d) # Già normalizzato
            
            # W_geom_to_world.Transform applica tutta la matrice, corretto per i punti/offset.
            off_world_d = W_geom_to_world.Transform(Gf.Vec3d(off_geom_local))
            off_world = Gf.Vec3f(off_world_d)
            
            dotz = Gf.Dot(n_world, world_up_vector)

            if self.debug_pose_calculation:
                print(f"    Face '{face_name}': n_geom_local={n_geom_local}, off_geom_local={off_geom_local}")
                print(f"      n_world={n_world}, off_world={off_world}, dotZ_world={dotz:.3f}")
            
            if dotz > best_face_data["dot_world_z"]:
                best_face_data["dot_world_z"] = dotz
                best_face_data["surface_point_world"] = off_world
                best_face_data["surface_normal_world"] = n_world
                best_face_data["name"] = face_name
        
        if best_face_data["surface_point_world"] is None:
            print(f"WARN: No valid face found for '{current_target_prim_path}'. Cannot calculate grasp pose.")
            return None

        surface_point_world = best_face_data["surface_point_world"]
        surface_normal_world = best_face_data["surface_normal_world"]

        if self.debug_pose_calculation:
            print(f"  Chosen Face: '{best_face_data['name']}' (dotZ_world={best_face_data['dot_world_z']:.3f})")
            print(f"  Calculated Surface Point (S_world): {surface_point_world}")
            print(f"  Calculated Surface Normal (N_world): {surface_normal_world}")


        # Orientamento del cono: l'asse Z locale del cono si allinea con surface_normal_world
        cone_axis_local_d = Gf.Vec3d(0,0,1) 
        target_normal_world_d = Gf.Vec3d(surface_normal_world)
        
        rotation_to_align_cone_d = Gf.Rotation(cone_axis_local_d, target_normal_world_d)
        # Gestione del caso in cui le normali sono opposte (angolo di 180 gradi)
        dot_product_cone_align = Gf.Dot(cone_axis_local_d, target_normal_world_d)
        if dot_product_cone_align < -0.99999: # Quasi -1
            # Scegli un asse di rotazione ortogonale a cone_axis_local_d
            ortho_axis = Gf.Cross(cone_axis_local_d, Gf.Vec3d.XAxis())
            if ortho_axis.GetLengthSq() < 1e-6: # Se cone_axis_local_d è parallelo a XAxis, prova YAxis
                ortho_axis = Gf.Cross(cone_axis_local_d, Gf.Vec3d.YAxis())
            
            if ortho_axis.GetLengthSq() > 1e-6: # Assicurati che l'asse sia valido
                rotation_to_align_cone_d = Gf.Rotation(ortho_axis.GetNormalized(), 180.0)
                if self.debug_pose_calculation: print(f"  Align Cone: Normals opposed (dot={dot_product_cone_align:.3f}). Using 180 deg rotation around {ortho_axis.GetNormalized()}.")
            else: # Caso molto improbabile se cone_axis_local_d è (0,0,1)
                 if self.debug_pose_calculation: print(f"  WARN: Could not determine 180 deg rotation axis for cone alignment, normals opposed but orthogonal axis is zero. Using identity rotation for cone.")
                 rotation_to_align_cone_d = Gf.Rotation() # Identità
        
        cone_orientation_world_d = rotation_to_align_cone_d.GetQuat().GetNormalized()
        cone_orientation_world_qf = Gf.Quatf(
            float(cone_orientation_world_d.GetReal()), 
            Gf.Vec3f(float(cone_orientation_world_d.GetImaginary()[0]),
                     float(cone_orientation_world_d.GetImaginary()[1]),
                     float(cone_orientation_world_d.GetImaginary()[2]))
        )
        
        # Origine del cono: la sua base (z=0 locale) è su surface_point_world + un offset lungo la normale
        cone_origin_world = surface_point_world + surface_normal_world * (self.grasp_offset_from_top) 
        
        if self.debug_pose_calculation:
            print(f"  Calculated Cone Origin (O_cone_world): {cone_origin_world}")
            print(f"  Calculated Cone Orientation (Q_cone_world): W={cone_orientation_world_qf.GetReal():.4f}, xyz={cone_orientation_world_qf.GetImaginary()}")
            print(f"--- End Debug _get_target_object_grasp_pose (path: {current_target_prim_path}) ---")

        return cone_origin_world, cone_orientation_world_qf, surface_normal_world

























   
    # ... (il resto del codice da _refresh_prim_handle in poi rimane invariato rispetto alla versione precedente) ...
    def _refresh_prim_handle(self, prim_path: str, current_handle: Optional[SingleRigidPrim]) -> Optional[SingleRigidPrim]:
        if current_handle and current_handle.is_valid():
            return current_handle
        current_stage = self._stage if self._stage else self._usd_context.get_stage()
        if not current_stage:
            if self._sim_step % 120 == 0: print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): Stage not available.")
            return None
       
        usd_prim = current_stage.GetPrimAtPath(prim_path)
        if not usd_prim.IsValid():
            if self._sim_step > 10 and self._sim_step % (120 if prim_path == "/GripperCone" else 60) == 1 :
                 print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): USD prim '{prim_path}' NOT VALID.")
            return None
        try:
            new_handle = SingleRigidPrim(prim_path)
            if not new_handle.is_valid():
                if self._sim_step % 60 == 0: print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): New SRP '{prim_path}' INVALID after creation.")
                return None
            return new_handle
        except RuntimeError as e_refresh:
            if self._sim_step % 60 == 0 and self.debug_pose_calculation: 
                 print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): RuntimeError creating SingleRigidPrim: {e_refresh}")
            return None
        except Exception as e_generic:
            print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): EXCEPTION creating SingleRigidPrim: {e_generic}")
            return None

    def _create_rigid_body_usd(
        self, body_geom_type: type[UsdGeom.Gprim], prim_path: str, *, mass: float, position: Gf.Vec3f,
        orientation: Gf.Quatf, color: Gf.Vec3f, height: Optional[float] = None,
        radius: Optional[float] = None, size: Optional[float] = None, 
    ) -> Optional[UsdGeom.Gprim]:
        if not self._stage:
            print(f"ERROR (_create_rigid_body_usd for {prim_path}): Stage not available.")
            return None
       
        if not prim_path.startswith("/"):
            prim_path = "/" + prim_path
            print(f"WARN (_create_rigid_body_usd): Relative prim path for {body_geom_type.__name__}, converted to: {prim_path}")

        geom_prim_path = Sdf.Path(prim_path)
        
        if self._stage.GetPrimAtPath(geom_prim_path).IsValid():
            try:
                omni.kit.commands.execute("DeletePrims", paths=[str(geom_prim_path)])
                if self.debug_pose_calculation: print(f"  (_create_rigid_body_usd) Deleted existing prim at {geom_prim_path} before recreation.")
            except Exception as e_del_recreate:
                print(f"  WARN (_create_rigid_body_usd): Could not delete existing prim {geom_prim_path}: {e_del_recreate}")

        geom: UsdGeom.Gprim = body_geom_type.Define(self._stage, geom_prim_path)
        if not geom.GetPrim().IsValid():
            print(f"ERROR (_create_rigid_body_usd for {prim_path}): Failed to define prim.")
            return None

        prim = geom.GetPrim()
        xform = UsdGeom.Xformable(geom)
        xform.ClearXformOpOrder() 
        
        translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat)
        translate_op.Set(position)

        orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
        orient_op.Set(orientation)
        
        scale_op = xform.AddScaleOp(UsdGeom.XformOp.PrecisionFloat)
        scale_op.Set(Gf.Vec3f(1.0,1.0,1.0))

        if isinstance(geom, UsdGeom.Cone):
            if height is not None: geom.CreateHeightAttr().Set(float(height))
            if radius is not None: geom.CreateRadiusAttr().Set(float(radius))
            axis_attr_usd = geom.GetAxisAttr() 
            if not axis_attr_usd.IsDefined() or not axis_attr_usd.HasValue():
                geom.CreateAxisAttr().Set(UsdGeom.Tokens.z) 
            elif axis_attr_usd.Get() != UsdGeom.Tokens.z:
                 axis_attr_usd.Set(UsdGeom.Tokens.z)
        elif isinstance(geom, UsdGeom.Cube):
            if size is not None: geom.CreateSizeAttr().Set(float(size))

        disp_color_primvar = UsdGeom.PrimvarsAPI(geom).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant
        )
        if disp_color_primvar:
            disp_color_primvar.Set([color])
        else: print(f"WARN: Could not create or get displayColor Primvar for {prim_path}.")

        UsdPhysics.CollisionAPI.Apply(prim)
        if mass > 0:
            rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            if rb_api:
                mass_api = UsdPhysics.MassAPI.Apply(prim)
                if mass_api:
                    mass_api.CreateMassAttr().Set(float(mass))
                else: print(f"WARN: Could not apply MassAPI to {prim_path}.")
            else: print(f"WARN: Could not apply RigidBodyAPI to {prim_path}.")
        return geom

    def _setup_initial_scene(self):
        self._stage = self._usd_context.get_stage()
        if not self._stage:
            print("ERROR (_setup_initial_scene): Stage could not be retrieved/created. Attempting to create new stage.")
            omni.usd.create_new_stage()
            self._stage = self._usd_context.get_stage()
            if not self._stage:
                print("FATAL (_setup_initial_scene): Still no stage after attempting creation. Cannot proceed.")
                return

        cone_prim_path_to_delete = Sdf.Path("/GripperCone")
        if self._stage.GetPrimAtPath(cone_prim_path_to_delete).IsValid():
            try:
                joint_path_to_delete = Sdf.Path(str(cone_prim_path_to_delete) + "/SurfaceGripperJoint")
                paths_to_delete = [str(cone_prim_path_to_delete)]
                if self._stage.GetPrimAtPath(joint_path_to_delete).IsValid():
                    paths_to_delete.append(str(joint_path_to_delete))
               
                omni.kit.commands.execute("DeletePrims", paths=paths_to_delete)
                print(f"Deleted existing prims {paths_to_delete} from previous run.")
            except Exception as e_del:
                print(f"Error deleting prims {paths_to_delete}: {e_del}")
       
        if self._physx_sub is not None:
            try:
                self._physx_sub.unsubscribe()
                print("Unsubscribed from previous physics step events.")
            except Exception as e_unsub:
                print(f"Error unsubscribing previous physics step: {e_unsub}")
            self._physx_sub = None

        self._physx_sub = self._physx_interface.subscribe_physics_step_events(self._on_simulation_step)
       
        if not self._timeline.is_playing():
            self._timeline.play()
           
        print(f"Initial scene setup. Object to grasp: {self.object_to_grasp_path}. Sim (re)started.")
        print(f"Will wait {self.steps_for_cube_to_settle} steps for objects to settle.")

    def _create_cone_usd_prim(self):
        if not self._stage:
            print("WARN (_create_cone_usd_prim): Stage not ready. Skipping cone USD prim creation.")
            self.cone_usd_prim_created = False
            return

        self.target_srp_handle = self._refresh_prim_handle(self.object_to_grasp_path, self.target_srp_handle)
        if not self.target_srp_handle or not self.target_srp_handle.is_valid():
             print(f"WARN (_create_cone_usd_prim): Target '{self.object_to_grasp_path}' handle not ready. Skipping.")
             self.cone_usd_prim_created = False
             return

        grasp_pose_data = self._get_target_object_grasp_pose(self.object_to_grasp_path)
        if not grasp_pose_data:
            print(f"ERROR (_create_cone_usd_prim): Could not calculate grasp pose for '{self.object_to_grasp_path}'. Skipping.")
            self.cone_usd_prim_created = False
            return
        cone_start_pos_gf, cone_orientation_gf, _ = grasp_pose_data 

        cone_rot_img = cone_orientation_gf.GetImaginary()
        print(f"Creating cone USD prim at Pos: {cone_start_pos_gf}, Rot (Quatf WXYZ): {cone_orientation_gf.GetReal():.4f}, {cone_rot_img[0]:.4f}, {cone_rot_img[1]:.4f}, {cone_rot_img[2]:.4f}")

        cone_geom = self._create_rigid_body_usd(
            UsdGeom.Cone, "/GripperCone", mass=0.01, position=cone_start_pos_gf, 
            orientation=cone_orientation_gf, color=self.color_open, 
            height=self.cone_height, radius=self.cone_radius
        )
        if not cone_geom:
            print("ERROR (_create_cone_usd_prim): Failed to create GripperCone prim.")
            self.cone_usd_prim_created = False
            return
        self.cone_geom_usd = UsdGeom.Cone(cone_geom.GetPrim())

        cone_prim_usd = self.cone_geom_usd.GetPrim()
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cone_prim_usd)
        if rigid_api.GetPrim().IsValid():
            kinematic_attr = rigid_api.CreateKinematicEnabledAttr()
            kinematic_attr.Set(True, Usd.TimeCode.Default())
            print(f"Physics for {cone_prim_usd.GetPath()} initially set to KINEMATIC.")
            self.cone_usd_prim_created = True
            self.cone_physics_dynamically_enabled = False
        else:
            print(f"ERROR (_create_cone_usd_prim): Could not apply RigidBodyAPI for {cone_prim_usd.GetPath()}.")
            self.cone_usd_prim_created = False

    def _initialize_surface_gripper_and_attempt_close(self):
        if not self.cone_usd_prim_created or not self.cone_geom_usd or not self.cone_geom_usd.GetPrim().IsValid():
            print("ERROR (_initialize_surface_gripper_and_attempt_close): Cone USD prim not created or invalid. Cannot initialize SG.")
            self.cone_sg_initialized = False
            return

        print(f"SIM_STEP {self._sim_step}: Attempting to initialize SurfaceGripper for /GripperCone.")
        sgp = Surface_Gripper_Properties()
        sgp.d6JointPath = "/GripperCone/SurfaceGripperJoint"
        sgp.parentPath = "/GripperCone" 
       
        sgp.offset.p.x, sgp.offset.p.y, sgp.offset.p.z = self._sgp_offset_p_local_to_cone[0], self._sgp_offset_p_local_to_cone[1], self._sgp_offset_p_local_to_cone[2]
        
        sg_rot_q_f = self._sgp_offset_r_local_to_cone
        sgp.offset.r.w = sg_rot_q_f.GetReal()
        sg_rot_img_gripper = sg_rot_q_f.GetImaginary() 
        sgp.offset.r.x, sgp.offset.r.y, sgp.offset.r.z = sg_rot_img_gripper[0], sg_rot_img_gripper[1], sg_rot_img_gripper[2]
       
        sgp.gripThreshold = 0.01 

        mass_of_object_to_grip = self.box_mass 
        if self.target_srp_handle and self.target_srp_handle.is_valid() and self._stage:
            try:
                target_prim_for_mass = self._stage.GetPrimAtPath(self.target_srp_handle.prim_path)
                if target_prim_for_mass.HasAPI(UsdPhysics.MassAPI):
                    mass_api = UsdPhysics.MassAPI(target_prim_for_mass)
                    mass_attr = mass_api.GetMassAttr()
                    if mass_attr and mass_attr.IsDefined() and mass_attr.HasValue():
                        mass_val = mass_attr.Get(Usd.TimeCode.Default())
                        if isinstance(mass_val, (float, int)) and mass_val > 0:
                            mass_of_object_to_grip = float(mass_val)
            except Exception as e_mass: print(f"WARN: Could not get mass for target '{self.target_srp_handle.prim_path}': {e_mass}")
        print(f"Target object mass for grip calc: {mass_of_object_to_grip:.3f} kg")

        if self.absolute_grip_force is not None:
            sgp.forceLimit = self.absolute_grip_force
            print(f"Using ABSOLUTE grip force: {sgp.forceLimit:.2f} N")
        else:
            min_force_to_hold = mass_of_object_to_grip * 9.81 
            sgp.forceLimit = min_force_to_hold * self.grip_force_multiplier
            print(f"Using MULTIPLIER ({self.grip_force_multiplier}x) on F_grav ({min_force_to_hold:.2f}N). Grip F_limit: {sgp.forceLimit:.2f} N")

        sgp.torqueLimit = 100.0 
        sgp.stiffness = 1.0e4   
        sgp.damping = 1.0e3     
        sgp.retryClose = True   

        self.surface_gripper = Surface_Gripper()
        try:
            self.surface_gripper.initialize(sgp)
            self.cone_sg_initialized = True
            print("SurfaceGripper initialized successfully.")

            if not self.gripper_close_command_sent:
                print(f"SIM_STEP {self._sim_step}: SG initialized. Attempting to close gripper...")
                self.surface_gripper.close()
                self.gripper_close_command_sent = True
                print("    INFO: surface_gripper.close() called.")

        except Exception as e_sg_init:
            print(f"ERROR: SurfaceGripper init failed: {e_sg_init}"); traceback.print_exc()
            self.cone_sg_initialized = False
            self.surface_gripper = None


    def _on_simulation_step(self, dt: float):
        self._sim_step += 1
        time_code_default = Usd.TimeCode.Default()

        if not self._stage :
            if self._sim_step % 120 == 0: print(f"SIM_STEP {self._sim_step}: Stage invalid. Stopping updates.")
            if self._physx_sub: self._physx_sub.unsubscribe(); self._physx_sub = None
            return

        if self._sim_step % 120 == 15: 
            if self.target_srp_handle and self.target_srp_handle.is_valid():
                try:
                    box_pos_tuple, box_rot_tuple = self.target_srp_handle.get_world_pose()
                    
                    if box_pos_tuple is not None and len(box_pos_tuple) == 3:
                        box_pos = Gf.Vec3f(float(box_pos_tuple[0]), float(box_pos_tuple[1]), float(box_pos_tuple[2]))
                    else:
                        box_pos = Gf.Vec3f(0.0,0.0,0.0)
                    
                    if box_rot_tuple is not None and len(box_rot_tuple) == 4: 
                        box_rot_q_f = Gf.Quatf(float(box_rot_tuple[0]), Gf.Vec3f(float(box_rot_tuple[1]), float(box_rot_tuple[2]), float(box_rot_tuple[3])))
                    else:
                        box_rot_q_f = _quat_identity()
                        
                    box_rot_img_print = box_rot_q_f.GetImaginary()
                    print(f"SIM_STEP {self._sim_step} DEBUG POSE: Box ('{self.object_to_grasp_path}') Pos={box_pos}, RotQuatWXYZ={box_rot_q_f.GetReal():.4f},{box_rot_img_print[0]:.4f},{box_rot_img_print[1]:.4f},{box_rot_img_print[2]:.4f}")
                except (RuntimeError, TypeError, IndexError, AttributeError) as e: 
                     print(f"SIM_STEP {self._sim_step} DEBUG (Box Pose Error): {type(e).__name__}: {e}")


            current_cone_prim_path_debug = "/GripperCone"
            if self.cone_prim_handle and self.cone_prim_handle.is_valid():
                try:
                    cone_pos_tuple, cone_rot_tuple = self.cone_prim_handle.get_world_pose() 
                    
                    if cone_pos_tuple is not None and len(cone_pos_tuple) == 3:
                        cone_pos = Gf.Vec3f(float(cone_pos_tuple[0]), float(cone_pos_tuple[1]), float(cone_pos_tuple[2]))
                    else:
                        cone_pos = Gf.Vec3f(0.0,0.0,0.0)

                    if cone_rot_tuple is not None and len(cone_rot_tuple) == 4:
                        cone_rot_q_f = Gf.Quatf(float(cone_rot_tuple[0]), Gf.Vec3f(float(cone_rot_tuple[1]), float(cone_rot_tuple[2]), float(cone_rot_tuple[3])))
                    else:
                        cone_rot_q_f = _quat_identity()
                        
                    cone_rot_img_print = cone_rot_q_f.GetImaginary()
                    print(f"SIM_STEP {self._sim_step} DEBUG POSE: Cone ('{current_cone_prim_path_debug}') Handle Pose: Pos={cone_pos}, RotQuatWXYZ={cone_rot_q_f.GetReal():.4f},{cone_rot_img_print[0]:.4f},{cone_rot_img_print[1]:.4f},{cone_rot_img_print[2]:.4f}")
                except (RuntimeError, TypeError, IndexError, AttributeError) as e:
                     print(f"SIM_STEP {self._sim_step} DEBUG (Cone Handle Pose Error): {type(e).__name__}: {e}")
            elif self.cone_geom_usd and self.cone_geom_usd.GetPrim().IsValid(): 
                try:
                    cone_prim_debug = self.cone_geom_usd.GetPrim()
                    cone_world_transform_debug: Gf.Matrix4d = omni.usd.get_world_transform_matrix(cone_prim_debug)
                    translation_dbg_d = cone_world_transform_debug.ExtractTranslation() 
                    cone_pos_debug = Gf.Vec3f(float(translation_dbg_d[0]), float(translation_dbg_d[1]), float(translation_dbg_d[2]))
                    cone_rot_q_d_debug = cone_world_transform_debug.ExtractRotationQuat().GetNormalized() 
                    real_part_dbg = float(cone_rot_q_d_debug.GetReal())
                    imag_part_dbg_d = cone_rot_q_d_debug.GetImaginary() 
                    imag_part_dbg_f = Gf.Vec3f(float(imag_part_dbg_d[0]), float(imag_part_dbg_d[1]), float(imag_part_dbg_d[2]))
                    cone_rot_q_f_debug = Gf.Quatf(real_part_dbg, imag_part_dbg_f)
                    cone_rot_img_print_debug = cone_rot_q_f_debug.GetImaginary()
                    print(f"SIM_STEP {self._sim_step} DEBUG POSE: Cone ('{current_cone_prim_path_debug}') USD Pose: Pos={cone_pos_debug}, RotQuatWXYZ={cone_rot_q_f_debug.GetReal():.4f},{cone_rot_img_print_debug[0]:.4f},{cone_rot_img_print_debug[1]:.4f},{cone_rot_img_print_debug[2]:.4f}")
                except Exception as e: 
                    print(f"SIM_STEP {self._sim_step} DEBUG (Cone USD Pose Error): {type(e).__name__}: {e}")

        self.target_srp_handle = self._refresh_prim_handle(self.object_to_grasp_path, self.target_srp_handle)
        if not (self.target_srp_handle and self.target_srp_handle.is_valid()):
            if self._sim_step % 60 == 0: print(f"SIM_STEP {self._sim_step}: Target '{self.object_to_grasp_path}' handle not valid. Waiting.")
            return

        if not self.cone_usd_prim_created and self._sim_step >= self.steps_for_cone_usd_creation_attempt:
            print(f"SIM_STEP {self._sim_step}: Target settled. Attempting to create cone USD prim at step {self.steps_for_cone_usd_creation_attempt}.")
            self._create_cone_usd_prim()
            if not self.cone_usd_prim_created:
                 if self._sim_step % 10 == 0: print(f"SIM_STEP {self._sim_step}: Cone USD prim creation failed. Will retry.")
                 return
            else: 
                self.cone_prim_handle = self._refresh_prim_handle("/GripperCone", self.cone_prim_handle)

        if self.cone_usd_prim_created and not self.cone_sg_initialized and self._sim_step >= self.steps_for_sg_init_and_grip_attempt:
            print(f"SIM_STEP {self._sim_step}: Cone USD prim created. Waiting period over. Attempting to initialize SG at step {self.steps_for_sg_init_and_grip_attempt}.")
            if not self.cone_prim_handle or not self.cone_prim_handle.is_valid():
                self.cone_prim_handle = self._refresh_prim_handle("/GripperCone", self.cone_prim_handle)
                if not (self.cone_prim_handle and self.cone_prim_handle.is_valid()):
                    if self._sim_step % 10 == 0: print(f"SIM_STEP {self._sim_step}: Cone handle for SG init not valid. Will retry.")
                    return 
            self._initialize_surface_gripper_and_attempt_close()
            if not self.cone_sg_initialized:
                if self._sim_step % 10 == 0: print(f"SIM_STEP {self._sim_step}: SurfaceGripper initialization failed. Will retry.")
                return

        if self.cone_usd_prim_created and (not self.cone_prim_handle or not self.cone_prim_handle.is_valid()):
            self.cone_prim_handle = self._refresh_prim_handle("/GripperCone", self.cone_prim_handle)

        if not self.cone_sg_initialized or not self.surface_gripper:
            return 

        try:
            self.surface_gripper.update()
        except Exception as e_update:
            print(f"SIM_STEP {self._sim_step}: ERROR during surface_gripper.update(): {e_update}"); return

        if self.cone_geom_usd and self.cone_geom_usd.GetPrim().IsValid():
            try:
                primvars_api = UsdGeom.PrimvarsAPI(self.cone_geom_usd.GetPrim())
                color_primvar = primvars_api.GetPrimvar("displayColor")
                if not color_primvar or not color_primvar.IsDefined():
                    color_primvar = primvars_api.CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant)
               
                if color_primvar:
                    new_color_gf = self.color_closed if self.surface_gripper.is_closed() else self.color_open 
                    current_color_val_array = color_primvar.Get(time_code_default) 
                    if current_color_val_array is None or len(current_color_val_array) == 0 or not Gf.IsClose(current_color_val_array[0], new_color_gf, 1e-5):
                        color_primvar.Set([new_color_gf], time_code_default)
            except Exception: 
                pass

        if self.gripper_close_command_sent and \
           not self.cone_physics_dynamically_enabled and \
           self.surface_gripper.is_closed() and \
           self._sim_step >= (self.steps_for_sg_init_and_grip_attempt + 5): 
           
            print(f"SIM_STEP {self._sim_step}: Gripper confirmed closed and stable. Attempting to set cone physics to DYNAMIC.")
            cone_prim_to_set_dynamic = None
            if self.cone_prim_handle and self.cone_prim_handle.is_valid() and self._stage:
                cone_prim_to_set_dynamic = self._stage.GetPrimAtPath(self.cone_prim_handle.prim_path)
            elif self.cone_geom_usd and self.cone_geom_usd.GetPrim().IsValid():
                cone_prim_to_set_dynamic = self.cone_geom_usd.GetPrim()

            if cone_prim_to_set_dynamic and cone_prim_to_set_dynamic.IsValid():
                rigid_api = UsdPhysics.RigidBodyAPI(cone_prim_to_set_dynamic) 
                if not rigid_api: rigid_api = UsdPhysics.RigidBodyAPI.Apply(cone_prim_to_set_dynamic)

                if rigid_api.GetPrim().IsValid(): 
                    kinematic_attr = rigid_api.GetKinematicEnabledAttr() 
                    if not kinematic_attr : kinematic_attr = rigid_api.CreateKinematicEnabledAttr() 

                    current_kinematic_state = kinematic_attr.Get(time_code_default)
                    if current_kinematic_state is not False: 
                        kinematic_attr.Set(False, time_code_default) 
                        print(f"    SUCCESS: Cone physics for {cone_prim_to_set_dynamic.GetPath()} set to DYNAMIC.")
                        self.cone_physics_dynamically_enabled = True
                    else: 
                        print(f"    INFO: Cone physics for {cone_prim_to_set_dynamic.GetPath()} was already DYNAMIC.")
                        self.cone_physics_dynamically_enabled = True
                else:
                    print(f"    ERROR: Could not apply/get RigidBodyAPI for {cone_prim_to_set_dynamic.GetPath()} to enable dynamic physics.")
            else:
                cone_path_debug_dyn = self.cone_prim_handle.prim_path if self.cone_prim_handle and self.cone_prim_handle.is_valid() else str(self.cone_geom_usd.GetPath() if self.cone_geom_usd else 'None')
                print(f"    ERROR: Cone prim for dynamic physics not found or invalid (path used: {cone_path_debug_dyn}).")
       
        can_start_moving = (
            self.surface_gripper.is_closed() and
            self.cone_prim_handle and self.cone_prim_handle.is_valid() and
            self.cone_physics_dynamically_enabled
        )

        if self.movement_phase == "idle" and \
           self._sim_step >= self.steps_before_initial_lift_phase and \
           can_start_moving:
            try:
                current_cone_pos_tuple, _ = self.cone_prim_handle.get_world_pose() 
                if current_cone_pos_tuple is None or len(current_cone_pos_tuple) != 3 : raise TypeError("Pose tuple is None or malformed from get_world_pose for movement start")
                self.movement_phase = "lifting"
                self.lift_start_z = float(current_cone_pos_tuple[2]) 
                print(f"SIM_STEP {self._sim_step} (Movement Start): Gripper closed, Cone DYNAMIC. Phase: LIFTING. Start Z: {self.lift_start_z:.4f}, Target Lift Z: {self.target_lift_height:.4f}")
            except (RuntimeError, TypeError, IndexError, AttributeError) as e_get_pose_idle: 
                print(f"SIM_STEP {self._sim_step} (Movement Idle->Lifting): Error getting cone pose: {e_get_pose_idle}. To 'done'.")
                self.movement_phase = "done"

        if self.movement_phase in ["lifting", "moving_horizontal", "descending", "releasing"]:
            if not (self.cone_prim_handle and self.cone_prim_handle.is_valid()):
                if self.movement_phase != "releasing": 
                    print(f"SIM_STEP {self._sim_step} (Movement - Phase {self.movement_phase}): cone_prim_handle invalid. Aborting move to 'done'.")
                self.movement_phase = "done"
                return
           
            if not self.cone_physics_dynamically_enabled and self.movement_phase not in ["releasing", "done"]:
                print(f"SIM_STEP {self._sim_step} (Movement - Phase {self.movement_phase}): Cone is NOT DYNAMIC. Cannot move. Aborting to 'releasing'.")
                self.movement_phase = "releasing" 
                try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32))
                except RuntimeError: pass 
                return

            current_cone_pos_np: Optional[np.ndarray] = None
            try:
                current_cone_pos_tuple, _ = self.cone_prim_handle.get_world_pose() 
                if current_cone_pos_tuple is None or len(current_cone_pos_tuple) != 3:
                    raise TypeError("get_world_pose returned None or malformed tuple during active movement")
                current_cone_pos_np = current_cone_pos_tuple 
            except (RuntimeError, TypeError, IndexError, AttributeError) as e_get_pose_active:
                print(f"SIM_STEP {self._sim_step} (Movement - Phase {self.movement_phase}): Error getting cone pose: {e_get_pose_active}. To 'done'.")
                self.movement_phase = "done"; return

            if not self.surface_gripper.is_closed() and self.movement_phase not in ["releasing", "done"]:
                print(f"SIM_STEP {self._sim_step} (Movement - Phase {self.movement_phase}): Gripper opened unexpectedly! Aborting to 'releasing'.")
                self.movement_phase = "releasing"
                try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32))
                except RuntimeError as e_stop: print(f"  Movement Abort (gripper open) STOP ERR: {e_stop}"); self.movement_phase = "done"
               
            if self.movement_phase == "lifting":
                current_z = float(current_cone_pos_np[2])
                if self._sim_step % 15 == 0 : print(f"    LIFTING (Step {self._sim_step}): Z: {current_z:.4f} -> Target Z: {self.target_lift_height:.4f}")
                if current_z < self.target_lift_height:
                    try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0,self.lift_speed_vertical],dtype=np.float32))
                    except RuntimeError as e_lift: print(f" LIFTING ERR: {e_lift}"); self.movement_phase="done"
                else: 
                    print(f"SIM_STEP {self._sim_step} (Movement): Reached lift height. Phase: MOVING_HORIZONTAL.")
                    self.movement_phase = "moving_horizontal"
                    try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32)) 
                    except RuntimeError as e_lift_stop: print(f" LIFTING STOP ERR: {e_lift_stop}"); self.movement_phase="done"
               
            elif self.movement_phase == "moving_horizontal":
                target_xy_np = self.target_horizontal_position 
                current_xy_np = current_cone_pos_np[:2].astype(np.float64) 
                current_z_np = float(current_cone_pos_np[2])
                diff_xy_np = target_xy_np - current_xy_np
                dist_xy = float(np.linalg.norm(diff_xy_np))

                if self._sim_step % 15 == 0:
                    print(f"    MOVING_H (Step {self._sim_step}): CurrXY: {current_xy_np.round(3)}, DistXY: {dist_xy:.3f}, TargetZ (Hold): {self.target_lift_height:.4f}, CurrentZ: {current_z_np:.4f}")
               
                if dist_xy > 0.02: 
                    direction_xy_np = diff_xy_np / dist_xy
                    vel_xy_target_np = direction_xy_np * self.move_speed_horizontal
                    
                    z_error = self.target_lift_height - current_z_np
                    vel_z_correction = self.z_correction_factor * z_error
                    max_z_speed = self.lift_speed_vertical * self.max_z_correction_speed_factor 
                    vel_z_correction = float(np.clip(vel_z_correction, -max_z_speed, max_z_speed))
                    try:
                        self.cone_prim_handle.set_linear_velocity(
                            np.array([vel_xy_target_np[0], vel_xy_target_np[1], vel_z_correction], dtype=np.float32)
                        )
                    except RuntimeError as e_moveh: print(f" MOVING_H ERR: {e_moveh}"); self.movement_phase = "done"
                else: 
                    print(f"SIM_STEP {self._sim_step} (Movement): Reached horizontal target. Phase: DESCENDING.")
                    self.movement_phase = "descending"
                    self.target_descent_z = self.lift_start_z +(2*self.cone_height)+0.001 if self.lift_start_z is not None else current_z_np 
                    print(f"    MOVING_H -> DESCENDING: Target Descent Z: {self.target_descent_z:.4f}")
                    try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32)) 
                    except RuntimeError as e_moveh_stop: print(f" MOVING_H STOP ERR: {e_moveh_stop}"); self.movement_phase = "done"
               
            elif self.movement_phase == "descending":
                if self.target_descent_z is None: 
                    print(f"    DESCENDING (Step {self._sim_step}): target_descent_z is None! Aborting to RELEASING.")
                    self.movement_phase = "releasing"; return 
                
                current_z = float(current_cone_pos_np[2])
                if current_z > self.target_descent_z + 0.005: 
                    if self._sim_step % 15 == 0: print(f"    DESCENDING (Step {self._sim_step}): CurrZ: {current_z:.4f} -> TargetZ: {self.target_descent_z:.4f}")
                    try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0, -self.lift_speed_vertical],dtype=np.float32))
                    except RuntimeError as e_desc: print(f" DESCENDING ERR: {e_desc}"); self.movement_phase="done"
                else: 
                    print(f"SIM_STEP {self._sim_step} (Movement): Reached descent height. Phase: RELEASING.")
                    self.movement_phase = "releasing"
                    try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32)) 
                    except RuntimeError as e_desc_stop: print(f" DESCENDING STOP ERR: {e_desc_stop}"); self.movement_phase="done"
               
            elif self.movement_phase == "releasing":
                print(f"SIM_STEP {self._sim_step} (Movement): Phase RELEASING. Opening gripper.")
                if self.surface_gripper: self.surface_gripper.open() 
                self.movement_phase = "done"


        if self.movement_phase == "done":
            if self.cone_prim_handle and self.cone_prim_handle.is_valid() and self.cone_physics_dynamically_enabled:
                try:
                    _, current_lin_vel_tuple, _ = self.cone_prim_handle.get_velocities() 
                    if current_lin_vel_tuple is not None and len(current_lin_vel_tuple) == 3 and np.linalg.norm(np.array([float(c) for c in current_lin_vel_tuple])) > 0.01:
                        if self._sim_step % 30 == 0: print(f"    DONE (Step {self._sim_step}): Cone still moving. Attempting to stop.")
                        self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32))
                except (RuntimeError, TypeError, IndexError, AttributeError): pass 
                except Exception: pass 
           
            if self._sim_step > self.steps_before_initial_lift_phase + 300: 
                if self._sim_step % 240 == 0 : 
                    sg_status = "N/A"
                    if self.surface_gripper: sg_status = "Closed" if self.surface_gripper.is_closed() else "Open"
                    print(f"SIM_STEP {self._sim_step}: Movement phase 'done'. Gripper: {sg_status}. Sim continues. (Consider stopping sim or resetting script if task is complete)")

    def run(self):
        print("SurfaceGripperDirectScript.run() called. Initializing states...")
        self._sim_step = 0
        self.gripper_close_command_sent = False
        self.cone_usd_prim_created = False
        self.cone_sg_initialized = False  
        self.cone_physics_dynamically_enabled = False
        self.movement_phase = "idle"
        self.lift_start_z = None; self.target_descent_z = None
       
        self.cone_prim_handle = None; self.target_srp_handle = None
        self.cone_geom_usd = None;
        if self.surface_gripper:
            self.surface_gripper = None 
       
        self._setup_initial_scene()
        print("Setup complete. Simulation loop will now take over via physics callback.")

    def cleanup(self):
        print("SurfaceGripperDirectScript.cleanup() called.")
        if self._physx_sub:
            try:
                self._physx_sub.unsubscribe()
                print("Unsubscribed from physics step events during cleanup.")
            except Exception as e_unsub_clean:
                print(f"Error unsubscribing physics step during cleanup: {e_unsub_clean}")
            self._physx_sub = None
       
        if self.surface_gripper:
            self.surface_gripper = None 
        self.cone_prim_handle = None
        self.target_srp_handle = None
        self.cone_geom_usd = None
        print("Cleanup finished.")


