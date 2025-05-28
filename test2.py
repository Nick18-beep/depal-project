# Copyright (c) 2018‑2024, NVIDIA CORPORATION. All rights reserved.

from __future__ import annotations
import asyncio
from typing import Optional, Tuple, List, Dict, Any
import functools # Added for functools.partial

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

# Global state dictionary, to be managed by the script execution logic
main_sim_state_dict: Optional[Dict[str, Any]] = None

# --- Utility Functions ---
def _quat_identity() -> Gf.Quatf:
    return Gf.Quatf(1.0, 0.0, 0.0, 0.0)

# --- State Initialization ---
def create_and_initialize_simulation_state(
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
) -> Dict[str, Any]:
    
    sim_state: Dict[str, Any] = {}

    sim_state["_timeline"] = omni.timeline.get_timeline_interface()
    sim_state["_usd_context"] = omni.usd.get_context()
    sim_state["_physx_interface"] = _physx.get_physx_interface()

    sim_state["surface_gripper"] = None
    sim_state["cone_prim_handle"] = None
    sim_state["target_srp_handle"] = None
    sim_state["cone_geom_usd"] = None
    sim_state["_physx_sub"] = None
    sim_state["_stage"] = None
    sim_state["_sim_step"] = 0
    
    sim_state["target_object_prim_path"] = target_object_prim_path
    sim_state["grasp_offset_from_top"] = float(grasp_offset_from_top)
    sim_state["initial_box_prim_path"] = initial_box_prim_path
    sim_state["object_to_grasp_path"] = target_object_prim_path if target_object_prim_path else initial_box_prim_path

    sim_state["box_mass"] = float(box_mass)
    sim_state["absolute_grip_force"] = float(absolute_grip_force) if absolute_grip_force is not None else None
    sim_state["grip_force_multiplier"] = float(grip_force_multiplier)
    sim_state["target_lift_height"] = float(target_lift_height)
    sim_state["target_horizontal_position"] = np.array(target_horizontal_position, dtype=np.float64)
   
    sim_state["z_correction_factor"] = float(z_correction_factor)
    sim_state["max_z_correction_speed_factor"] = float(max_z_correction_speed_factor)
    sim_state["cone_height"] = float(cone_height)
    sim_state["cone_radius"] = float(cone_radius)
    sim_state["box_size"] = float(box_size)
    sim_state["lift_speed_vertical"] = float(lift_speed_vertical)
    sim_state["move_speed_horizontal"] = float(move_speed_horizontal)

    sim_state["steps_for_cube_to_settle"] = int(steps_for_cube_to_settle)
    sim_state["steps_before_grace_period"] = sim_state["steps_for_cube_to_settle"] + int(steps_grace_period_after_settle)
    sim_state["steps_before_grip_attempt"] = sim_state["steps_before_grace_period"] + int(steps_wait_before_grip_attempt)
    sim_state["steps_after_grip_handle_refresh"] = int(steps_wait_after_grip_refresh)
    sim_state["steps_before_initial_lift_phase"] = (
        sim_state["steps_before_grip_attempt"] + sim_state["steps_after_grip_handle_refresh"]
    )

    sim_state["gripper_close_attempted_this_cycle"] = False
    sim_state["cone_spawned_and_positioned"] = False
    sim_state["cone_physics_dynamically_enabled"] = False

    sim_state["color_closed"] = Gf.Vec3f(1.0, 0.2, 0.2)
    sim_state["color_open"] = Gf.Vec3f(0.2, 1.0, 0.2)
   
    sim_state["movement_phase"] = "idle"
    sim_state["initial_object_position_for_move"] = None
    sim_state["lift_start_z"] = None
    sim_state["target_descent_z"] = None
   
    sim_state["_sgp_offset_p_local_to_cone"] = Gf.Vec3f(0.0, 0.0, -sim_state["cone_radius"] + 0.002) 
    sim_state["_sgp_offset_r_local_to_cone"] = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

    print("Simulation state created with configuration:")
    if sim_state["target_object_prim_path"]:
        print(f"  Target Object Prim Path: {sim_state['target_object_prim_path']}")
        print(f"  Grasp Offset From Top: {sim_state['grasp_offset_from_top']:.4f} m")
    else:
        print(f"  No specific target object, will use default box: {sim_state['initial_box_prim_path']}")
    print(f"  Box Mass (for default box or target if overridden): {sim_state['box_mass']:.3f} kg")
    if sim_state["absolute_grip_force"] is not None:
        print(f"  Absolute Grip Force: {sim_state['absolute_grip_force']:.2f} N")
    else:
        min_force_calc = sim_state["box_mass"] * 9.81 * sim_state["grip_force_multiplier"]
        print(f"  Grip Force Multiplier: {sim_state['grip_force_multiplier']:.1f}x (Calculated Limit: {min_force_calc:.2f} N)")
    print(f"  Target Lift Height (absolute Z): {sim_state['target_lift_height']:.2f} m")
    print(f"  Target Horizontal Position: {sim_state['target_horizontal_position']}")
    
    return sim_state

# --- Core Logic Functions ---

def _get_target_object_grasp_pose_func(sim_state: Dict[str, Any], current_target_prim_path: str) -> Optional[Tuple[Gf.Vec3f, Gf.Quatf, Gf.Vec3f]]:
    if not sim_state["_stage"]:
        print(f"ERROR (_get_target_object_grasp_pose_func): Stage not available.")
        return None

    target_usd_prim = sim_state["_stage"].GetPrimAtPath(current_target_prim_path)
    if not target_usd_prim.IsValid():
        print(f"ERROR (_get_target_object_grasp_pose_func): Target USD prim '{current_target_prim_path}' is not valid.")
        return None

    target_world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(target_usd_prim)
    target_world_position_d: Gf.Vec3d = target_world_transform.ExtractTranslation()
    target_world_rotation_quat_d: Gf.Quatd = target_world_transform.ExtractRotationQuat()
   
    target_world_position = Gf.Vec3f(target_world_position_d)
    target_world_rotation_quat = Gf.Quatf(target_world_rotation_quat_d)

    world_up_vector = Gf.Vec3f(0.0, 0.0, 1.0)
    best_local_up_vector = Gf.Vec3f(0.0, 0.0, 1.0)
    best_local_top_center_offset = Gf.Vec3f(0.0, 0.0, 0.0)
    min_angle_with_world_up = 181.0

    prim_type = target_usd_prim.GetTypeName()
    time_code_default = Usd.TimeCode.Default()

    if target_usd_prim.IsA(UsdGeom.Cube):
        cube_geom = UsdGeom.Cube(target_usd_prim)
        size_attr = cube_geom.GetSizeAttr()
        size_val = size_attr.Get(time_code_default) if size_attr else sim_state["box_size"]
        size = float(size_val) if size_val is not None else sim_state["box_size"]
        half_size = size / 2.0

        local_face_data = [
            (Gf.Vec3f(1.0, 0.0, 0.0), Gf.Vec3f(half_size, 0.0, 0.0)),
            (Gf.Vec3f(-1.0, 0.0, 0.0), Gf.Vec3f(-half_size, 0.0, 0.0)),
            (Gf.Vec3f(0.0, 1.0, 0.0), Gf.Vec3f(0.0, half_size, 0.0)),
            (Gf.Vec3f(0.0, -1.0, 0.0), Gf.Vec3f(0.0, -half_size, 0.0)),
            (Gf.Vec3f(0.0, 0.0, 1.0), Gf.Vec3f(0.0, 0.0, half_size)),
            (Gf.Vec3f(0.0, 0.0, -1.0), Gf.Vec3f(0.0, 0.0, -half_size))
        ]

        for local_normal, local_center_offset in local_face_data:
            world_face_normal = target_world_rotation_quat.Transform(local_normal).GetNormalized()
            dot_product = Gf.Dot(world_face_normal, world_up_vector)
            dot_product_clipped = max(-1.0, min(1.0, dot_product))
            angle_rad = np.arccos(dot_product_clipped)
            angle_deg = np.degrees(angle_rad)

            if angle_deg < min_angle_with_world_up:
                min_angle_with_world_up = angle_deg
                best_local_up_vector = local_normal
                best_local_top_center_offset = local_center_offset
       
        print(f"  Cube Grasp: Chosen local normal {best_local_up_vector}, center offset {best_local_top_center_offset}, angle with world_up: {min_angle_with_world_up:.2f} deg")
        local_top_center_offset = best_local_top_center_offset
        local_up_vector = best_local_up_vector

    elif target_usd_prim.IsA(UsdGeom.Cylinder):
        cyl_geom = UsdGeom.Cylinder(target_usd_prim)
        height_attr = cyl_geom.GetHeightAttr()
        height_val = height_attr.Get(time_code_default) if height_attr else sim_state["cone_height"] 
        height = float(height_val) if height_val is not None else sim_state["cone_height"]
        axis_attr = cyl_geom.GetAxisAttr()
        axis_val = axis_attr.Get(time_code_default) if axis_attr else UsdGeom.Tokens.z
        axis = axis_val if axis_val is not None else UsdGeom.Tokens.z

        if axis == UsdGeom.Tokens.z:
            local_top_center_offset = Gf.Vec3f(0.0, 0.0, height / 2.0)
            local_up_vector = Gf.Vec3f(0.0,0.0,1.0)
        elif axis == UsdGeom.Tokens.y:
            local_top_center_offset = Gf.Vec3f(0.0, height / 2.0, 0.0)
            local_up_vector = Gf.Vec3f(0.0,1.0,0.0)
        elif axis == UsdGeom.Tokens.x:
            local_top_center_offset = Gf.Vec3f(height / 2.0, 0.0, 0.0)
            local_up_vector = Gf.Vec3f(1.0,0.0,0.0)
        else: 
            local_top_center_offset = Gf.Vec3f(0.0, 0.0, height / 2.0)
            local_up_vector = Gf.Vec3f(0.0,0.0,1.0)


    elif target_usd_prim.IsA(UsdGeom.Sphere):
        sphere_geom = UsdGeom.Sphere(target_usd_prim)
        radius_attr = sphere_geom.GetRadiusAttr()
        radius_val = radius_attr.Get(time_code_default) if radius_attr else sim_state["cone_radius"] 
        radius = float(radius_val) if radius_val is not None else sim_state["cone_radius"]
        local_top_center_offset = Gf.Vec3f(0.0,0.0, radius)
        local_up_vector = Gf.Vec3f(0.0,0.0,1.0)

    else: 
        print(f"WARN (_get_target_object_grasp_pose_func): Target prim '{current_target_prim_path}' type '{prim_type}' not Cube/Cylinder/Sphere. Using bounding box logic.")
        boundable = UsdGeom.Boundable(target_usd_prim)
        try:
            bbox_cache = UsdGeom.BBoxCache(time_code_default, [UsdGeom.Tokens.default_] , useExtentsHint=True)
            prim_local_bound = bbox_cache.ComputeUntransformedBound(target_usd_prim)

            if prim_local_bound.GetBox().IsEmpty():
                print(f"WARN: Could not get local bounds for {current_target_prim_path}. Assuming unit size at origin.")
                local_top_center_offset = Gf.Vec3f(0.0,0.0,0.5)
                local_up_vector = Gf.Vec3f(0.0,0.0,1.0)
            else:
                bbox_range_min_d = prim_local_bound.GetBox().GetMin()
                bbox_range_max_d = prim_local_bound.GetBox().GetMax()
                local_top_center_offset = Gf.Vec3f(
                    float((bbox_range_min_d[0] + bbox_range_max_d[0]) / 2.0),
                    float((bbox_range_min_d[1] + bbox_range_max_d[1]) / 2.0),
                    float(bbox_range_max_d[2])
                )
                local_up_vector = Gf.Vec3f(0.0,0.0,1.0)
        except Exception as e_bbox:
            print(f"ERROR computing bounding box for {current_target_prim_path}: {e_bbox}. Using fallback.")
            local_top_center_offset = Gf.Vec3f(0.0,0.0,0.5)
            local_up_vector = Gf.Vec3f(0.0,0.0,1.0)

    world_target_surface_point = target_world_rotation_quat.Transform(local_top_center_offset) + target_world_position
    world_surface_normal_of_target = target_world_rotation_quat.Transform(local_up_vector).GetNormalized()

    from_vec_cone_local_z = Gf.Vec3d(0.0, 0.0, 1.0)
    to_vec_world_surface_normal = Gf.Vec3d(world_surface_normal_of_target)
    rotation_to_align_cone_z_with_normal = Gf.Rotation(from_vec_cone_local_z, to_vec_world_surface_normal)
    cone_orientation_world = Gf.Quatf(rotation_to_align_cone_z_with_normal.GetQuat())

    desired_phantom_body_origin_world = world_target_surface_point + world_surface_normal_of_target * sim_state["grasp_offset_from_top"]
    
    offset_cone_origin_to_phantom_body_local = sim_state["_sgp_offset_p_local_to_cone"]
    offset_phantom_body_to_cone_origin_local = -offset_cone_origin_to_phantom_body_local
    offset_phantom_to_cone_origin_world = cone_orientation_world.Transform(offset_phantom_body_to_cone_origin_local)
    cone_origin_world = desired_phantom_body_origin_world + offset_phantom_to_cone_origin_world
   
    return cone_origin_world, cone_orientation_world, world_surface_normal_of_target

def _refresh_prim_handle_func(sim_state: Dict[str, Any], prim_path: str, current_handle: Optional[SingleRigidPrim]) -> Optional[SingleRigidPrim]:
    if current_handle and current_handle.is_valid():
        return current_handle
    current_stage = sim_state["_usd_context"].get_stage()
    if not current_stage:
        if sim_state["_sim_step"] % 120 == 0: print(f"SIM STEP {sim_state['_sim_step']} (_refresh_prim_handle_func for {prim_path}): Stage not available.")
        return None
    usd_prim = current_stage.GetPrimAtPath(prim_path)
    if not usd_prim.IsValid():
        if sim_state["_sim_step"] > 10 :
             if sim_state["_sim_step"] % 60 == 0: print(f"SIM STEP {sim_state['_sim_step']} (_refresh_prim_handle_func for {prim_path}): USD prim '{prim_path}' is NOT VALID.")
        return None
    try:
        new_handle = SingleRigidPrim(prim_path)
        if not new_handle.is_valid():
            if sim_state["_sim_step"] % 60 == 0: print(f"SIM STEP {sim_state['_sim_step']} (_refresh_prim_handle_func for {prim_path}): New SRP for '{prim_path}' immediately INVALID.")
            return None
        return new_handle
    except Exception as e_refresh:
        print(f"SIM STEP {sim_state['_sim_step']} (_refresh_prim_handle_func for {prim_path}): EXCEPTION: {e_refresh}")
        return None

def _create_rigid_body_usd_func(
    sim_state: Dict[str, Any], body_geom_type, prim_path: str, *, mass: float, position: Gf.Vec3f,
    orientation: Gf.Quatf, color: Gf.Vec3f, height: Optional[float] = None,
    radius: Optional[float] = None, size: Optional[float] = None,
) -> UsdGeom.Gprim:
    if not sim_state["_stage"]:
        print(f"ERROR (_create_rigid_body_usd_func for {prim_path}): Stage not available.")
        raise RuntimeError("Stage not available for prim creation.")
       
    geom: UsdGeom.Gprim = body_geom_type.Define(sim_state["_stage"], prim_path)
    prim = sim_state["_stage"].GetPrimAtPath(prim_path)
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
        if color_attr: color_attr.Set([Gf.Vec3f(float(color[0]),float(color[1]),float(color[2]))])
        else: print(f"WARN: Could not create DisplayColorAttr or Primvar for {prim_path}.")

    UsdPhysics.CollisionAPI.Apply(prim)
    if mass > 0:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
    return geom

async def _setup_initial_scene_async_func(sim_state: Dict[str, Any]):
    await omni.usd.get_context().new_stage_async()
    sim_state["_stage"] = sim_state["_usd_context"].get_stage()
    if not sim_state["_stage"]:
        print("ERROR: Stage could not be retrieved. Cannot proceed.")
        return

    UsdGeom.SetStageUpAxis(sim_state["_stage"], UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(sim_state["_stage"], 1.0)
   
    scene = UsdPhysics.Scene.Define(sim_state["_stage"], Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0,0.0,-1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
   
    omni.kit.commands.execute(
        "AddGroundPlaneCommand", stage=sim_state["_stage"], planePath="/groundPlane", axis="Z",
        size=10.0, position=Gf.Vec3f(0.0,0.0,0.0), color=Gf.Vec3f(0.5,0.5,0.5)
    )
   
    light = UsdLux.DistantLight.Define(sim_state["_stage"], Sdf.Path("/KeyLight"))
    light.CreateIntensityAttr(1000)

    if sim_state["target_object_prim_path"] == "/World/MyTargetCube":
         if not sim_state["_stage"].GetPrimAtPath("/World/MyTargetCube").IsValid():
            print("Creating /World/MyTargetCube for testing...")
            rot_y_obj = Gf.Rotation(Gf.Vec3d(0.0,1.0,0.0), 90.0) 
            target_orientation_quatd = rot_y_obj.GetQuat()
            target_orientation = Gf.Quatf(target_orientation_quatd)

            _create_rigid_body_usd_func(
                sim_state, UsdGeom.Cube, "/World/MyTargetCube", mass=sim_state["box_mass"], 
                position=Gf.Vec3f(0.3, -0.2, sim_state["box_size"] / 2.0 + 0.01), 
                orientation=target_orientation,
                color=Gf.Vec3f(1.0, 0.5, 0.0), size=sim_state["box_size"] 
            )
            print(f"Created target cube /World/MyTargetCube with size {sim_state['box_size']} and mass {sim_state['box_mass']}")

    elif not sim_state["target_object_prim_path"] or sim_state["target_object_prim_path"] == sim_state["initial_box_prim_path"] :
        if not sim_state["_stage"].GetPrimAtPath(sim_state["initial_box_prim_path"]).IsValid():
            box_initial_center_z = sim_state["box_size"] * 0.5 + 0.01 
            _create_rigid_body_usd_func(
                sim_state, UsdGeom.Cube, sim_state["initial_box_prim_path"], mass=sim_state["box_mass"],
                position=Gf.Vec3f(0.0,0.0,box_initial_center_z),
                orientation=_quat_identity(), color=Gf.Vec3f(0.2,0.2,1.0), size=sim_state["box_size"]
            )
            print(f"Created default box at {sim_state['initial_box_prim_path']}")
        else:
             print(f"Default box at {sim_state['initial_box_prim_path']} already exists or is the target. Not creating new.")


    set_camera_view(eye=(0.8,0.8,0.8), target=(0.0,0.0, sim_state["box_size"]*0.5))
    
    physics_callback = functools.partial(_on_simulation_step_func, sim_state)
    sim_state["_physx_sub"] = sim_state["_physx_interface"].subscribe_physics_step_events(physics_callback)
    
    sim_state["_timeline"].play()
   
    print(f"Initial scene setup. Object to grasp: {sim_state['object_to_grasp_path']}. Sim started.")
    print(f"Will wait {sim_state['steps_for_cube_to_settle']} steps before spawning cone.")

def _spawn_and_position_cone_and_gripper_func(sim_state: Dict[str, Any]):
    if not sim_state["_stage"]:
        print("WARN (_spawn_cone_func): Stage not ready. Skipping cone spawn.")
        return

    if not sim_state["target_srp_handle"] or not sim_state["target_srp_handle"].is_valid():
        sim_state["target_srp_handle"] = _refresh_prim_handle_func(sim_state, sim_state["object_to_grasp_path"], sim_state["target_srp_handle"])
        if not sim_state["target_srp_handle"] or not sim_state["target_srp_handle"].is_valid():
             print(f"WARN (_spawn_cone_func): Target for '{sim_state['object_to_grasp_path']}' not ready. Skipping cone spawn.")
             sim_state["cone_spawned_and_positioned"] = False 
             return

    cone_start_pos_gf: Gf.Vec3f
    cone_orientation_gf: Gf.Quatf
    time_code_default = Usd.TimeCode.Default()
   
    if sim_state["target_object_prim_path"]:
        grasp_pose_data = _get_target_object_grasp_pose_func(sim_state, sim_state["target_object_prim_path"])
        if grasp_pose_data:
            cone_start_pos_gf, cone_orientation_gf, _ = grasp_pose_data
        else:
            print(f"ERROR: Could not calculate grasp pose for '{sim_state['target_object_prim_path']}'. Skipping cone spawn.")
            sim_state["cone_spawned_and_positioned"] = False 
            return
    else: 
        print(f"No specific target. Using default box '{sim_state['initial_box_prim_path']}' for positioning.")
        target_pos_tuple, _ = sim_state["target_srp_handle"].get_world_pose()
        target_pos_np = np.array(target_pos_tuple, dtype=float)
        stage_units = get_stage_units() 
        
        target_top_z_approx = target_pos_np[2] + (sim_state["box_size"] / 2.0) * stage_units
        phantom_body_desired_z = target_top_z_approx + sim_state["grasp_offset_from_top"]
        cone_origin_z = phantom_body_desired_z - sim_state["_sgp_offset_p_local_to_cone"][2]

        pos_x, pos_y, pos_z = float(target_pos_np[0]), float(target_pos_np[1]), float(cone_origin_z)
        cone_start_pos_gf = Gf.Vec3f(pos_x, pos_y, pos_z)
        cone_orientation_gf = _quat_identity() 

    _create_rigid_body_usd_func(
        sim_state, UsdGeom.Cone, "/GripperCone", mass=0.01, position=cone_start_pos_gf,
        orientation=cone_orientation_gf, color=sim_state["color_open"],
        height=sim_state["cone_height"], radius=sim_state["cone_radius"]
    )
   
    cone_prim_usd = sim_state["_stage"].GetPrimAtPath("/GripperCone")
    if cone_prim_usd.IsValid():
        rigid_api = UsdPhysics.RigidBodyAPI(cone_prim_usd)
        if not rigid_api.GetPrim().IsValid():
             rigid_api = UsdPhysics.RigidBodyAPI.Apply(cone_prim_usd)
        if rigid_api.GetPrim().IsValid():
            kinematic_attr = rigid_api.CreateKinematicEnabledAttr()
            kinematic_attr.Set(True) 
            print(f"Physics for {cone_prim_usd.GetPath()} initially set to KINEMATIC.")
            sim_state["cone_physics_dynamically_enabled"] = False 
        else:
            print(f"ERROR: Could not get/apply RigidBodyAPI for {cone_prim_usd.GetPath()} to set kinematic.")
        
        if cone_prim_usd.IsA(UsdGeom.Cone):
            sim_state["cone_geom_usd"] = UsdGeom.Cone(cone_prim_usd)
        else:
            sim_state["cone_geom_usd"] = None; print(f"ERROR: {cone_prim_usd.GetPath()} not UsdGeom.Cone.")
    else:
        sim_state["cone_geom_usd"] = None; print(f"ERROR: /GripperCone not valid after creation.")
        sim_state["cone_spawned_and_positioned"] = False 
        return
       
    sgp = Surface_Gripper_Properties()
    sgp.d6JointPath = "/GripperCone/SurfaceGripperJoint"
    sgp.parentPath = "/GripperCone" 
    sgp.offset.p.x = float(sim_state["_sgp_offset_p_local_to_cone"][0])
    sgp.offset.p.y = float(sim_state["_sgp_offset_p_local_to_cone"][1])
    sgp.offset.p.z = float(sim_state["_sgp_offset_p_local_to_cone"][2])
    sgp.offset.r.w = float(sim_state["_sgp_offset_r_local_to_cone"].GetReal())
    sgp.offset.r.x = float(sim_state["_sgp_offset_r_local_to_cone"].GetImaginary()[0])
    sgp.offset.r.y = float(sim_state["_sgp_offset_r_local_to_cone"].GetImaginary()[1])
    sgp.offset.r.z = float(sim_state["_sgp_offset_r_local_to_cone"].GetImaginary()[2])
    sgp.gripThreshold = 0.01 
   
    mass_of_object_to_grip = sim_state["box_mass"] 
    if sim_state["target_srp_handle"] and sim_state["target_srp_handle"].is_valid() and sim_state["_stage"]:
        try:
            target_prim_for_mass = sim_state["_stage"].GetPrimAtPath(sim_state["target_srp_handle"].prim_path)
            if target_prim_for_mass.IsValid():
                mass_api = UsdPhysics.MassAPI.Get(target_prim_for_mass) 
                if mass_api:
                    mass_attr = mass_api.GetMassAttr()
                    if mass_attr and mass_attr.IsDefined() and mass_attr.HasValue():
                         mass_val = mass_attr.Get(time_code_default)
                         if isinstance(mass_val, (float, int)) and mass_val > 0 :
                            mass_of_object_to_grip = float(mass_val)
                            print(f"Using mass from target USD '{sim_state['target_srp_handle'].prim_path}': {mass_of_object_to_grip:.3f} kg")
        except Exception as e_mass_read:
            print(f"WARN: Exception reading mass from target '{sim_state['object_to_grasp_path']}': {e_mass_read}. Using default: {sim_state['box_mass']:.3f} kg")
    
    print(f"Target object mass for grip calc: {mass_of_object_to_grip:.3f} kg")
   
    if sim_state["absolute_grip_force"] is not None:
        sgp.forceLimit = sim_state["absolute_grip_force"]
        print(f"Using ABSOLUTE grip force: {sgp.forceLimit:.2f} N")
    else:
        min_force_val = mass_of_object_to_grip * 9.81 
        sgp.forceLimit = min_force_val * sim_state["grip_force_multiplier"]
        print(f"Using MULTIPLIER ({sim_state['grip_force_multiplier']}x) on min force ({min_force_val:.2f} N)\n  Calculated grip force: {sgp.forceLimit:.2f} N")
    
    sgp.torqueLimit = 10.0; sgp.stiffness = 1.0e4; sgp.damping = 1.0e3; sgp.retryClose = True
   
    sim_state["surface_gripper"] = Surface_Gripper()
    try:
        sim_state["surface_gripper"].initialize(sgp)
        sim_state["cone_spawned_and_positioned"] = True
        print("Cone and SurfaceGripper initialized.")
        
        cam_target_focus_point_gf = Gf.Vec3f(cone_start_pos_gf) 
        if sim_state["target_srp_handle"] and sim_state["target_srp_handle"].is_valid():
            tp_tuple, _ = sim_state["target_srp_handle"].get_world_pose()
            target_object_center_gf = Gf.Vec3f(float(tp_tuple[0]), float(tp_tuple[1]), float(tp_tuple[2]))
            cam_target_focus_point_gf = (target_object_center_gf + cam_target_focus_point_gf) / 2.0 
        
        eye_pos_tuple = (cam_target_focus_point_gf[0] + 0.5, 
                         cam_target_focus_point_gf[1] + 0.5, 
                         cam_target_focus_point_gf[2] + 0.5)
        set_camera_view(eye=eye_pos_tuple, target=tuple(cam_target_focus_point_gf))

    except Exception as e_sg_init:
        print(f"ERROR: SurfaceGripper init or subsequent setup failed: {e_sg_init}"); traceback.print_exc()
        sim_state["cone_spawned_and_positioned"] = False 

def _on_simulation_step_func(sim_state: Dict[str, Any], dt: float): 
    sim_state["_sim_step"] += 1
    time_code_default = Usd.TimeCode.Default()

    sim_state["target_srp_handle"] = _refresh_prim_handle_func(sim_state, sim_state["object_to_grasp_path"], sim_state["target_srp_handle"])
    if not (sim_state["target_srp_handle"] and sim_state["target_srp_handle"].is_valid()):
         if sim_state["_sim_step"] % 60 == 0: print(f"SIM {sim_state['_sim_step']}: Target '{sim_state['object_to_grasp_path']}' handle invalid. Waiting.")
         return

    if not sim_state["cone_spawned_and_positioned"] and sim_state["_sim_step"] >= sim_state["steps_for_cube_to_settle"]:
        _spawn_and_position_cone_and_gripper_func(sim_state)
        if not sim_state["cone_spawned_and_positioned"]: 
            if sim_state["_sim_step"] % 60 == 0: print(f"SIM {sim_state['_sim_step']}: Cone/gripper init failed or deferred. Will retry.")
            return 
   
    if sim_state["cone_spawned_and_positioned"]: 
        sim_state["cone_prim_handle"] = _refresh_prim_handle_func(sim_state, "/GripperCone", sim_state["cone_prim_handle"])

    if not sim_state["cone_spawned_and_positioned"]: return 
   
    if sim_state["_sim_step"] < sim_state["steps_before_grace_period"]:
        if sim_state["_sim_step"] == sim_state["steps_for_cube_to_settle"] + 1 and sim_state["cone_spawned_and_positioned"]:
             print(f"SIM {sim_state['_sim_step']}: Cone spawned (kinematic). Grace period until {sim_state['steps_before_grace_period']}.")
        return
    elif sim_state["_sim_step"] == sim_state["steps_before_grace_period"]:
        print(f"SIM {sim_state['_sim_step']}: Grace period finished. Gripper logic active.")

    if sim_state["surface_gripper"]:
        try: sim_state["surface_gripper"].update()
        except Exception as e: print(f"SIM {sim_state['_sim_step']}: ERROR gripper.update(): {e}"); return

        if sim_state["cone_geom_usd"] and sim_state["cone_geom_usd"].GetPrim().IsValid():
            try:
                disp_color_primvar = UsdGeom.PrimvarsAPI(sim_state["cone_geom_usd"]).GetPrimvar("displayColor")
                if disp_color_primvar:
                    color = sim_state["color_closed"] if sim_state["surface_gripper"].is_closed() else sim_state["color_open"]
                    disp_color_primvar.Set([color], time_code_default)
            except Exception as e_color: # Added specific exception variable
                 if sim_state["_sim_step"] % 60 == 0: print(f"SIM {sim_state['_sim_step']}: Error setting cone color: {e_color}")
   
    if sim_state["_sim_step"] == sim_state["steps_before_grip_attempt"] and not sim_state["gripper_close_attempted_this_cycle"]:
        print(f"SIM {sim_state['_sim_step']}: Attempting to close gripper...")
        if sim_state["surface_gripper"]:
            try:
                sim_state["surface_gripper"].close()
                sim_state["gripper_close_attempted_this_cycle"] = True
                print(f"    INFO: surface_gripper.close() called.")
            except Exception as e_close: print(f"    ERROR: surface_gripper.close() failed → {e_close}")
        else: print(f"    WARN: surface_gripper is None at grip attempt.")

    if sim_state["gripper_close_attempted_this_cycle"] and \
       not sim_state["cone_physics_dynamically_enabled"] and \
       sim_state["surface_gripper"] and sim_state["surface_gripper"].is_closed():
       
        print(f"SIM {sim_state['_sim_step']}: Gripper confirmed closed. Attempting to set cone physics to DYNAMIC.")
        cone_path_str = "/GripperCone" 
        cone_prim_usd = sim_state["_stage"].GetPrimAtPath(cone_path_str) if sim_state["_stage"] else None

        if cone_prim_usd and cone_prim_usd.IsValid():
            rigid_api = UsdPhysics.RigidBodyAPI(cone_prim_usd)
            if not rigid_api.GetPrim().IsValid(): rigid_api = UsdPhysics.RigidBodyAPI.Apply(cone_prim_usd)

            if rigid_api.GetPrim().IsValid():
                kin_attr = rigid_api.CreateKinematicEnabledAttr() 
                current_kin_state = kin_attr.Get(time_code_default)
                if current_kin_state is True: 
                    kin_attr.Set(False) 
                    sim_state["cone_physics_dynamically_enabled"] = True
                    print(f"    SUCCESS: Cone physics for {cone_path_str} set to DYNAMIC.")
                elif current_kin_state is False:
                    sim_state["cone_physics_dynamically_enabled"] = True 
                    print(f"    INFO: Cone physics for {cone_path_str} was already DYNAMIC.")
                else: 
                    kin_attr.Set(False) 
                    sim_state["cone_physics_dynamically_enabled"] = True
                    print(f"    WARN: Cone kinematic attr for {cone_path_str} was {current_kin_state}. Forced DYNAMIC.")
            else: print(f"    ERROR: Could not get/apply RigidBodyAPI for {cone_path_str} for dynamic.")
        else: print(f"    ERROR: Cone prim '{cone_path_str}' not found/invalid for dynamic physics.")

    if sim_state["_sim_step"] == sim_state["steps_before_initial_lift_phase"]:
        sg_stat = "N/A"; cph_valid = "N/A"
        if sim_state["surface_gripper"]: sg_stat = "Closed" if sim_state["surface_gripper"].is_closed() else "Open"
        if sim_state["cone_prim_handle"]: cph_valid = "Valid" if sim_state["cone_prim_handle"].is_valid() else "Invalid"
        cone_phys = "Dynamic" if sim_state["cone_physics_dynamically_enabled"] else "Kinematic/NotYetDynamic"
        print(f"LIFT CHECK (Step {sim_state['_sim_step']}): Phase='{sim_state['movement_phase']}', Gripper='{sg_stat}', ConeHandle='{cph_valid}', ConePhys='{cone_phys}'")

    can_move = sim_state["surface_gripper"] and sim_state["surface_gripper"].is_closed() and \
               sim_state["cone_prim_handle"] and sim_state["cone_prim_handle"].is_valid() and \
               sim_state["cone_physics_dynamically_enabled"]

    if sim_state["movement_phase"] == "idle" and sim_state["_sim_step"] >= sim_state["steps_before_initial_lift_phase"] and can_move:
        try:
            pos, _ = sim_state["cone_prim_handle"].get_world_pose()
            sim_state["movement_phase"] = "lifting"; sim_state["lift_start_z"] = float(pos[2])
            print(f"SIM {sim_state['_sim_step']} (Move Start): Phase LIFTING. StartZ:{sim_state['lift_start_z']:.4f}, TargetZ:{sim_state['target_lift_height']:.4f}")
        except RuntimeError as e_get_pose: print(f"SIM {sim_state['_sim_step']} (Idle->Lifting): Cone pose error: {e_get_pose}"); sim_state["cone_prim_handle"] = None; sim_state["movement_phase"] = "done" 
   
    if sim_state["movement_phase"] in ["lifting", "moving_horizontal", "descending", "releasing"]:
        if not (sim_state["cone_prim_handle"] and sim_state["cone_prim_handle"].is_valid()):
            if sim_state["movement_phase"] != "releasing": print(f"SIM {sim_state['_sim_step']} (Move-{sim_state['movement_phase']}): cone_handle invalid. To 'done'.")
            sim_state["movement_phase"] = "done"; return
        if not sim_state["cone_physics_dynamically_enabled"] and sim_state["movement_phase"] not in ["releasing", "done"]:
            print(f"SIM {sim_state['_sim_step']} (Move-{sim_state['movement_phase']}): Cone NOT DYNAMIC. To 'releasing'.")
            sim_state["movement_phase"] = "releasing"

        current_pos_np: Optional[np.ndarray] = None
        try:
            pos_tuple, _ = sim_state["cone_prim_handle"].get_world_pose()
            current_pos_np = np.array(pos_tuple, dtype=float)
        except RuntimeError as e_get_pose_active:
            print(f"SIM {sim_state['_sim_step']} (Move-{sim_state['movement_phase']}): Cone pose error: {e_get_pose_active}. To 'done'.")
            sim_state["cone_prim_handle"] = None; sim_state["movement_phase"] = "done"; return

        if current_pos_np is not None:
            if sim_state["surface_gripper"] and not sim_state["surface_gripper"].is_closed() and sim_state["movement_phase"] not in ["releasing", "done"]:
                print(f"SIM {sim_state['_sim_step']} (Move-{sim_state['movement_phase']}): Gripper opened! To 'releasing'.")
                sim_state["movement_phase"] = "releasing"
                try: sim_state["cone_prim_handle"].set_linear_velocity(np.zeros(3, dtype=np.float32))
                except RuntimeError as e_stop: print(f" MOVE ABORT STOP ERR: {e_stop}"); sim_state["cone_prim_handle"] = None; sim_state["movement_phase"] = "done"

            if sim_state["movement_phase"] == "lifting":
                if sim_state["_sim_step"] % 10 == 0 : print(f"    LIFTING (Step {sim_state['_sim_step']}): Z: {current_pos_np[2]:.4f}, Target Z: {sim_state['target_lift_height']:.4f}")
                if current_pos_np[2] < sim_state["target_lift_height"]:
                    try: sim_state["cone_prim_handle"].set_linear_velocity(np.array([0.0,0.0,sim_state["lift_speed_vertical"]],dtype=np.float32))
                    except RuntimeError as e_lift: print(f" LIFTING ERR: {e_lift}"); sim_state["cone_prim_handle"]=None; sim_state["movement_phase"]="done"
                else:
                    print(f"SIM {sim_state['_sim_step']} (Movement): Reached lift height. Phase: MOVING_HORIZONTAL.")
                    sim_state["movement_phase"] = "moving_horizontal"
                    try: sim_state["cone_prim_handle"].set_linear_velocity(np.zeros(3,dtype=np.float32))
                    except RuntimeError as e_stop: print(f" LIFT STOP ERR: {e_stop}"); sim_state["cone_prim_handle"]=None; sim_state["movement_phase"]="done"
            
            elif sim_state["movement_phase"] == "moving_horizontal":
                target_xy_np = sim_state["target_horizontal_position"]
                current_xy_np = current_pos_np[:2]
                diff_xy_np = target_xy_np - current_xy_np
                dist_xy = float(np.linalg.norm(diff_xy_np))
                
                if sim_state["_sim_step"] % 10 == 0: print(f"    MOVING_H (Step {sim_state['_sim_step']}): TargetXY: {target_xy_np.round(3)}, CurrentXY: {current_xy_np.round(3)}, DistXY: {dist_xy:.3f}")

                if dist_xy > 0.02: 
                    direction_xy_np = diff_xy_np / dist_xy
                    vel_xy_target_np = direction_xy_np * sim_state["move_speed_horizontal"]
                    z_error = sim_state["target_lift_height"] - current_pos_np[2]
                    vel_z_correction = sim_state["z_correction_factor"] * z_error
                    max_z_speed = sim_state["lift_speed_vertical"] * sim_state["max_z_correction_speed_factor"]
                    vel_z_correction = float(np.clip(vel_z_correction, -max_z_speed, max_z_speed))
                    try: sim_state["cone_prim_handle"].set_linear_velocity(np.array([vel_xy_target_np[0], vel_xy_target_np[1], vel_z_correction], dtype=np.float32))
                    except RuntimeError as e_moveh: print(f" MOVING_H ERR: {e_moveh}"); sim_state["cone_prim_handle"]=None; sim_state["movement_phase"]="done"
                else:
                    print(f"SIM {sim_state['_sim_step']} (Movement): Reached H target. Phase: DESCENDING.")
                    sim_state["movement_phase"] = "descending"
                    if sim_state["lift_start_z"] is None: 
                        sim_state["target_descent_z"] = float(current_pos_np[2]) 
                    else:
                        sim_state["target_descent_z"] = sim_state["lift_start_z"]
                    print(f"    MOVING_H -> DESCENDING: Target Descent Z: {sim_state['target_descent_z']:.4f}")
                    try: sim_state["cone_prim_handle"].set_linear_velocity(np.zeros(3,dtype=np.float32))
                    except RuntimeError as e_stop: print(f" MOVING_H STOP ERR: {e_stop}"); sim_state["cone_prim_handle"]=None; sim_state["movement_phase"]="done"

            elif sim_state["movement_phase"] == "descending":
                if sim_state["target_descent_z"] is None: 
                    print(f"    DESCENDING (Step {sim_state['_sim_step']}): target_descent_z is None. Critical. To RELEASING.")
                    sim_state["movement_phase"] = "releasing"
                elif current_pos_np[2] > sim_state["target_descent_z"] + 0.005: 
                    if sim_state["_sim_step"] % 10 == 0: print(f"    DESCENDING (Step {sim_state['_sim_step']}): CurrentZ: {current_pos_np[2]:.4f}, TargetZ: {sim_state['target_descent_z']:.4f}")
                    try: sim_state["cone_prim_handle"].set_linear_velocity(np.array([0.0,0.0, -sim_state["lift_speed_vertical"]],dtype=np.float32))
                    except RuntimeError as e_desc: print(f" DESCENDING ERR: {e_desc}"); sim_state["cone_prim_handle"]=None; sim_state["movement_phase"]="done"
                else:
                    print(f"SIM {sim_state['_sim_step']} (Movement): Reached descent height. Phase: RELEASING.")
                    sim_state["movement_phase"] = "releasing"
                    try: sim_state["cone_prim_handle"].set_linear_velocity(np.zeros(3,dtype=np.float32))
                    except RuntimeError as e_stop: print(f" DESCENDING STOP ERR: {e_stop}"); sim_state["cone_prim_handle"]=None; sim_state["movement_phase"]="done"
            
            elif sim_state["movement_phase"] == "releasing":
                print(f"SIM {sim_state['_sim_step']} (Movement): Phase RELEASING. Opening gripper.")
                if sim_state["surface_gripper"]: sim_state["surface_gripper"].open()
                sim_state["movement_phase"] = "done"

    if sim_state["movement_phase"] == "done":
        if sim_state["cone_prim_handle"] and sim_state["cone_prim_handle"].is_valid() and sim_state["cone_physics_dynamically_enabled"]:
            try:
                 _, lin_vel_tuple, _ = sim_state["cone_prim_handle"].get_velocities()
                 if np.linalg.norm(np.array(lin_vel_tuple)) > 0.01: 
                    if sim_state["_sim_step"] % 30 == 0: print(f"    DONE (Step {sim_state['_sim_step']}): Cone still moving. Stopping.")
                    sim_state["cone_prim_handle"].set_linear_velocity(np.zeros(3,dtype=np.float32))
            except RuntimeError: sim_state["cone_prim_handle"] = None 
            except Exception as e_done_stop: 
                if sim_state["_sim_step"] % 60 == 0: print(f"    DONE (Step {sim_state['_sim_step']}): Exception stopping cone: {e_done_stop}")
        
        if sim_state["_sim_step"] > sim_state["steps_before_initial_lift_phase"] + 500 and sim_state["_sim_step"] % 120 == 0:
             cone_phys_final = "Dynamic" if sim_state["cone_physics_dynamically_enabled"] else "Kinematic/Initial"
             print(f"SIM {sim_state['_sim_step']}: Phase 'done'. Cone physics: {cone_phys_final}. Sim continues.")

# --- Simulation Control Functions ---
def run_simulation_func(sim_state: Dict[str, Any]):
    print("run_simulation_func() called.")
    if sim_state["_timeline"].is_playing():
        sim_state["_timeline"].stop()
   
    sim_state["_sim_step"] = 0
    sim_state["gripper_close_attempted_this_cycle"] = False
    sim_state["cone_spawned_and_positioned"] = False
    sim_state["cone_physics_dynamically_enabled"] = False
    sim_state["movement_phase"] = "idle"
    sim_state["initial_object_position_for_move"] = None
    sim_state["lift_start_z"] = None
    sim_state["target_descent_z"] = None
    sim_state["cone_prim_handle"] = None
    sim_state["target_srp_handle"] = None
    sim_state["cone_geom_usd"] = None
    if sim_state["surface_gripper"]: sim_state["surface_gripper"] = None 
   
    if sim_state["_physx_sub"]:
        sim_state["_physx_sub"].unsubscribe()
        sim_state["_physx_sub"] = None

    asyncio.ensure_future(_setup_initial_scene_async_func(sim_state))

def cleanup_simulation_func(sim_state: Dict[str, Any]):
    print("Cleaning up simulation state...")
    if sim_state.get("_timeline") and sim_state["_timeline"].is_playing():
        sim_state["_timeline"].stop()
   
    if sim_state.get("_physx_sub"):
        sim_state["_physx_sub"].unsubscribe()
        sim_state["_physx_sub"] = None
   
    sim_state["surface_gripper"] = None
    sim_state["cone_prim_handle"] = None
    sim_state["target_srp_handle"] = None
    sim_state["cone_geom_usd"] = None
    sim_state["_stage"] = None

    sim_state["_sim_step"] = 0
    sim_state["gripper_close_attempted_this_cycle"] = False
    sim_state["cone_spawned_and_positioned"] = False
    sim_state["cone_physics_dynamically_enabled"] = False
    sim_state["movement_phase"] = "idle"
    sim_state["initial_object_position_for_move"] = None
    sim_state["lift_start_z"] = None
    sim_state["target_descent_z"] = None
    print("Simulation state cleanup complete.")

# --- SCRIPT EXECUTION LOGIC ---
if main_sim_state_dict is not None:
    print("Cleaning up previous simulation state...")
    try:
        cleanup_simulation_func(main_sim_state_dict)
    except Exception as e_cleanup:
        print(f"Error during cleanup of previous state: {e_cleanup}")
    main_sim_state_dict = None 

print("--- CONFIGURATION FOR NEW SIMULATION ---")

config_target_prim_to_grasp = "/World/MyTargetCube" 
config_grasp_offset = 0.000 
config_initial_box_path = "/DefaultBox" 
config_box_mass = 0.030 
config_absolute_grip_force = None 
config_grip_force_multiplier = 100.0 
config_lift_speed_vertical = 1.5
config_target_lift_height = 0.3 
config_target_horizontal_position = np.array([0.1, 0.1]) 
config_z_correction_factor = 15.0
config_max_z_correction_speed_factor = 2.0
config_cone_height = 0.05 
config_cone_radius = 0.03 
config_box_size = 0.05 
config_move_speed_horizontal = 0.05 

config_steps_for_cube_to_settle = 60
config_steps_grace_period_after_settle = 30
config_steps_wait_before_grip_attempt = 120
config_steps_wait_after_grip_refresh = 40 

print(f"  Config: Target Prim='{config_target_prim_to_grasp if config_target_prim_to_grasp else f'Default Box ({config_initial_box_path})'}'")
print(f"  Config: Grasp Offset={config_grasp_offset:.4f}m, AbsGripForce={config_absolute_grip_force if config_absolute_grip_force is not None else 'N/A (Multiplier)'}, LiftSpeed={config_lift_speed_vertical}m/s")

print("\nInitializing new simulation state...")
main_sim_state_dict = create_and_initialize_simulation_state(
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
if main_sim_state_dict is not None: 
    run_simulation_func(main_sim_state_dict)
else:
    print("ERROR: Simulation state dictionary was not created.")

print("--- Script execution initiated ---")

while True: 
    simulation_app.update()


