# Copyright (c) 2018‑2024, NVIDIA CORPORATION. All rights reserved.
# ... (resto come prima) ...


from __future__ import annotations
import asyncio
from typing import Optional
import numpy as np
import omni
import omni.kit.commands
import omni.kit.usd
import omni.physx as _physx
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics


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
    def __init__(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self._usd_context = omni.usd.get_context()
        self._physx_interface = _physx.get_physx_interface()


        self.surface_gripper: Optional[Surface_Gripper] = None
        self.cone_prim_handle: Optional[SingleRigidPrim] = None
        self.box_srp_handle: Optional[SingleRigidPrim] = None
        self.cone_geom_usd: Optional[UsdGeom.Cone] = None
        self._physx_sub = None
        self._stage: Optional[omni.usd.Stage] = None


        self._sim_step = 0
        self.steps_for_cube_to_settle = 30
        self.steps_before_grace_period = self.steps_for_cube_to_settle + 20
        self.steps_before_grip_attempt = self.steps_before_grace_period + 100
        self.steps_after_grip_handle_refresh = 120
        self.steps_before_initial_lift_phase = (
            self.steps_before_grip_attempt + self.steps_after_grip_handle_refresh
        )


        self.gripper_close_attempted_this_cycle = False
        self.cone_spawned_and_positioned = False


        self.color_closed = Gf.Vec3f(1.0, 0.2, 0.2)
        self.color_open = Gf.Vec3f(0.2, 1.0, 0.2)
        self.cone_height = 0.1
        self.cone_radius = 0.05
        self.box_size = 0.1
       
        self.box_mass = 0.05
        self.grip_force_multiplier = 50.0
        self.absolute_grip_force: Optional[float] = None


        self.lift_speed_vertical = 0.5
        self.move_speed_horizontal = 0.1
        self.target_lift_height = 0.5
        self.target_horizontal_position = np.array([0.3, 0.3])


        self.movement_phase = "idle"
        self.initial_object_position_for_move: Optional[np.ndarray] = None
        self.lift_start_z: Optional[float] = None
        self.target_descent_z: Optional[float] = None

        # Parametri per la correzione attiva dell'altezza Z
        self.z_correction_factor = 10.0  # Guadagno proporzionale per la correzione Z
        self.max_z_correction_speed_factor = 1.5 # Moltiplicatore per lift_speed_vertical


    def _refresh_prim_handle(self, prim_path: str, current_handle: Optional[SingleRigidPrim]) -> Optional[SingleRigidPrim]:
        if current_handle and current_handle.is_valid():
            return current_handle
        current_stage = self._usd_context.get_stage()
        if not current_stage:
            if self._sim_step % 120 == 0: print(f"SIM STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): Stage not available.")
            return None
        usd_prim = current_stage.GetPrimAtPath(prim_path)
        if not usd_prim.IsValid():
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
        orientation: Gf.Quatf, color: Gf.Vec3f, height: float | None = None,
        radius: float | None = None, size: float | None = None,
    ) -> UsdGeom.Gprim:
        geom: UsdGeom.Gprim = body_geom_type.Define(self._stage, prim_path)
        prim = self._stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(geom)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(position)
        xform.AddOrientOp().Set(orientation)
        if height is not None and hasattr(geom, "CreateHeightAttr"): geom.CreateHeightAttr(height)
        if radius is not None and hasattr(geom, "CreateRadiusAttr"): geom.CreateRadiusAttr(radius)
        if size is not None and hasattr(geom, "CreateSizeAttr"): geom.CreateSizeAttr(size)
        color_attr = geom.CreateDisplayColorAttr()
        if color_attr: color_attr.Set([color])
        else: print(f"WARN: Could not create DisplayColorAttr for {prim_path}")
        UsdPhysics.CollisionAPI.Apply(prim)
        if mass > 0: UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        print(f"Created Rigid Body: {prim.GetPath().pathString} (mass: {mass:.3f}kg) at {position} with Gf.Quatf rot {orientation}")
        return geom


    async def _setup_initial_scene_async(self):
        await self._usd_context.new_stage_async()
        self._stage = self._usd_context.get_stage()
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0,0,-1))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        omni.kit.commands.execute(
            "AddGroundPlaneCommand", stage=self._stage, planePath="/groundPlane", axis="Z",
            size=10.0, position=Gf.Vec3f(0,0,0), color=Gf.Vec3f(0.5)
        )
        UsdLux.DistantLight.Define(self._stage, Sdf.Path("/KeyLight")).CreateIntensityAttr(1000)
        box_initial_center_z = self.box_size * 0.5
        self._create_rigid_body_usd(
            UsdGeom.Cube, "/Box", mass=self.box_mass, position=Gf.Vec3f(0,0,box_initial_center_z),
            orientation=_quat_identity(), color=Gf.Vec3f(0.2,0.2,1.0), size=self.box_size
        )
        set_camera_view(eye=[0.8,0.8,0.8], target=[0,0,self.box_size*0.5])
        self._physx_sub = self._physx_interface.subscribe_physics_step_events(self._on_simulation_step)
        self._timeline.play()
        print(f"Initial scene (box mass: {self.box_mass:.3f}kg) created. Simulation started.")
        print(f"Will wait {self.steps_for_cube_to_settle} steps before spawning cone.")


    def _spawn_and_position_cone_and_gripper(self):
        if not self._stage or not self.box_srp_handle or not self.box_srp_handle.is_valid():
            print(f"WARN (_spawn_cone): Stage or box handle (valid: {self.box_srp_handle.is_valid() if self.box_srp_handle else 'None'}) not ready. Skipping cone spawn.")
            return

        box_position, _ = self.box_srp_handle.get_world_pose()
        stage_units = get_stage_units()
        box_top_z_approx = box_position[2] + (self.box_size / 2.0) * stage_units
        cone_center_z_world = box_top_z_approx + self.cone_radius * stage_units + 0.005 * stage_units
        pos_x = float(box_position[0]); pos_y = float(box_position[1]); pos_z = float(cone_center_z_world)
        cone_start_pos_gf = Gf.Vec3f(pos_x, pos_y, pos_z)
        cone_orientation_gf = _quat_identity()
        cone_usd_gprim = self._create_rigid_body_usd(
            UsdGeom.Cone, "/GripperCone", mass=0.1, position=cone_start_pos_gf,
            orientation=cone_orientation_gf, color=self.color_open,
            height=self.cone_height, radius=self.cone_radius
        )
        if isinstance(cone_usd_gprim, UsdGeom.Cone): self.cone_geom_usd = cone_usd_gprim
        else: self.cone_geom_usd = None
       
        sgp = Surface_Gripper_Properties()
        sgp.d6JointPath = "/GripperCone/SurfaceGripperJoint"
        sgp.parentPath = "/GripperCone"
        sgp.offset.p.x = 0.0; sgp.offset.p.y = 0.0; sgp.offset.p.z = -self.cone_radius - 0.001
        sgp.offset.r.x = 0.0; sgp.offset.r.y = 1.0; sgp.offset.r.z = 0.0; sgp.offset.r.w = 0.0
        sgp.gripThreshold = 0.03
       
        print(f"--- GRIP STRENGTH SETTINGS (Cone Spawn) ---")
        print(f"Target box mass: {self.box_mass:.3f} kg")
        if self.absolute_grip_force is not None:
            sgp.forceLimit = self.absolute_grip_force
            print(f"Using ABSOLUTE grip force: {sgp.forceLimit:.2f} N")
        else:
            min_force_to_hold_box = self.box_mass * 9.81
            sgp.forceLimit = min_force_to_hold_box * self.grip_force_multiplier
            print(f"Using MULTIPLIER ({self.grip_force_multiplier}x) on min force ({min_force_to_hold_box:.2f} N)")
            print(f"Calculated grip force: {sgp.forceLimit:.2f} N")

        sgp.torqueLimit = 10.0
        print(f"sgp.torqueLimit set to: {sgp.torqueLimit:.2f} Nm")
        sgp.bendAngle = np.pi/2; sgp.stiffness = 1.0e4; sgp.damping = 1.0e3
        sgp.retryClose = True
       
        self.surface_gripper = Surface_Gripper()
        self.surface_gripper.initialize(sgp)
        self.cone_spawned_and_positioned = True
        print("Cone and SurfaceGripper initialized and positioned above the box.")
        set_camera_view(eye=[0.6,0.6,0.5], target=[pos_x, pos_y, box_top_z_approx + self.cone_radius*0.5])


    def _on_simulation_step(self, dt): # dt è disponibile qui
        self._sim_step += 1

        if not self.box_srp_handle or not self.box_srp_handle.is_valid():
             self.box_srp_handle = self._refresh_prim_handle("/Box", self.box_srp_handle)
        if not self.box_srp_handle:
             if self._sim_step % 60 == 0: print(f"SIM STEP {self._sim_step}: Box handle still not ready. Waiting.")
             return

        if not self.cone_spawned_and_positioned and self._sim_step >= self.steps_for_cube_to_settle:
            self._spawn_and_position_cone_and_gripper()
       
        if self.cone_spawned_and_positioned and (not self.cone_prim_handle or not self.cone_prim_handle.is_valid()):
            self.cone_prim_handle = self._refresh_prim_handle("/GripperCone", self.cone_prim_handle)

        if not self.cone_spawned_and_positioned: return
       
        if self._sim_step < self.steps_before_grace_period:
            if self._sim_step == self.steps_for_cube_to_settle + 1 and self.cone_spawned_and_positioned:
                 print(f"SIM STEP {self._sim_step}: Cone spawned. Grace period until step {self.steps_before_grace_period}.")
            return
        elif self._sim_step == self.steps_before_grace_period:
            print(f"SIM STEP {self._sim_step}: Grace period finished. Gripper logic active.")

        if self.surface_gripper:
            try: self.surface_gripper.update()
            except Exception as e_update: print(f"SIM STEP {self._sim_step}: ERROR during surface_gripper.update(): {e_update}"); return

        if self.cone_geom_usd and self.surface_gripper and self.cone_geom_usd.GetPrim().IsValid():
            try:
                color_attr = self.cone_geom_usd.GetDisplayColorAttr()
                if color_attr: color_attr.Set([self.color_closed if self.surface_gripper.is_closed() else self.color_open])
            except Exception as e_color:
                 if self._sim_step % 60 == 0: print(f"SIM STEP {self._sim_step}: Error setting cone color: {e_color}")

        if self._sim_step == self.steps_before_grip_attempt and not self.gripper_close_attempted_this_cycle:
            print(f"SIM STEP {self._sim_step}: Attempting to close the gripper...")
            if self.surface_gripper:
                try:
                    self.surface_gripper.close()
                    self.gripper_close_attempted_this_cycle = True
                    print(f"    INFO: surface_gripper.close() called. Invalidating cone_prim_handle (will refresh).")
                    self.cone_prim_handle = None
                except Exception as e_close: print(f"    ERROR: surface_gripper.close() failed → {e_close}")
            else: print(f"    WARN: surface_gripper object is None at grip attempt.")

        if self._sim_step == self.steps_before_initial_lift_phase:
            sg_status = "N/A"
            if self.surface_gripper: sg_status = "Closed" if self.surface_gripper.is_closed() else "Open"
            cph_valid = "N/A"
            if self.cone_prim_handle: cph_valid = "Valid" if self.cone_prim_handle.is_valid() else "Invalid"
            print(f"LIFT CHECK (Step {self._sim_step}): Phase='{self.movement_phase}', Gripper State='{sg_status}', Cone Handle='{cph_valid}'")

        if self.movement_phase == "idle" and \
           self._sim_step >= self.steps_before_initial_lift_phase and \
           self.surface_gripper and self.surface_gripper.is_closed() and \
           self.cone_prim_handle and self.cone_prim_handle.is_valid():
            try:
                current_cone_pos_tuple, _ = self.cone_prim_handle.get_world_pose()
                current_cone_pos = np.array(current_cone_pos_tuple)
                self.movement_phase = "lifting"
                self.lift_start_z = current_cone_pos[2]
                print(f"SIM STEP {self._sim_step} (Movement Start): Gripper confirmed closed. Phase: LIFTING. Start Z: {self.lift_start_z:.4f}, Target Lift Z: {self.target_lift_height:.4f}")
            except RuntimeError as e_get_pose_idle:
                print(f"SIM STEP {self._sim_step} (Movement Idle->Lifting): RuntimeError getting cone pose: {e_get_pose_idle}. Invalidating handle.")
                self.cone_prim_handle = None
       
        if self.movement_phase in ["lifting", "moving_horizontal", "descending", "releasing"]:
            if not (self.cone_prim_handle and self.cone_prim_handle.is_valid()):
                if self.movement_phase != "releasing":
                    print(f"SIM STEP {self._sim_step} (Movement - Phase {self.movement_phase}): cone_prim_handle invalid. To 'done'.")
                self.movement_phase = "done"
            else:
                current_cone_pos = None
                try:
                    current_cone_pos_tuple, _ = self.cone_prim_handle.get_world_pose()
                    current_cone_pos = np.array(current_cone_pos_tuple)
                except RuntimeError as e_get_pose_active:
                    print(f"SIM STEP {self._sim_step} (Movement - Phase {self.movement_phase}): RuntimeError getting cone pose: {e_get_pose_active}. Invalidating handle. To 'done'.")
                    self.cone_prim_handle = None; self.movement_phase = "done"

                if current_cone_pos is not None:
                    if self.movement_phase == "lifting":
                        grip_stat = "Closed" if self.surface_gripper and self.surface_gripper.is_closed() else "Open/None"
                        if self._sim_step % 10 == 0 : print(f"    LIFTING (Step {self._sim_step}, Grip: {grip_stat}): Z: {current_cone_pos[2]:.4f}, Target Z: {self.target_lift_height:.4f}")
                        if current_cone_pos[2] < self.target_lift_height:
                            try: self.cone_prim_handle.set_linear_velocity(np.array([0,0,self.lift_speed_vertical],dtype=np.float32))
                            except RuntimeError as e: print(f" LIFTING ERR: {e}"); self.cone_prim_handle=None; self.movement_phase="done"
                        else:
                            print(f"SIM STEP {self._sim_step} (Movement): Reached lift height. Phase: MOVING_HORIZONTAL.")
                            self.movement_phase = "moving_horizontal"
                            try: self.cone_prim_handle.set_linear_velocity(np.array([0,0,0],dtype=np.float32))
                            except RuntimeError as e: print(f" LIFTING STOP ERR: {e}"); self.cone_prim_handle=None; self.movement_phase="done"
                   
                    elif self.movement_phase == "moving_horizontal":
                        target_xy = self.target_horizontal_position
                        current_xy = current_cone_pos[:2]
                        diff_xy = target_xy - current_xy
                        dist_xy = np.linalg.norm(diff_xy)
                        grip_stat = "Closed" if self.surface_gripper and self.surface_gripper.is_closed() else "Open/None"

                        if self._sim_step % 10 == 0:
                             print(f"    MOVING_H (Step {self._sim_step}, Grip: {grip_stat}): TargetXY: {target_xy.round(3)}, CurrentXY: {current_xy.round(3)}, DistXY: {dist_xy:.3f}, TargetZ: {self.target_lift_height:.4f}, CurrentZ: {current_cone_pos[2]:.4f}")

                        if dist_xy > 0.02: # Soglia per la distanza XY
                            direction_xy = diff_xy / dist_xy
                            vel_xy_target = direction_xy * self.move_speed_horizontal
                            
                            # Correzione attiva dell'altezza Z
                            z_error = self.target_lift_height - current_cone_pos[2]
                            vel_z_correction = self.z_correction_factor * z_error
                            
                            # Limita la velocità di correzione Z
                            max_z_speed = self.lift_speed_vertical * self.max_z_correction_speed_factor
                            vel_z_correction = np.clip(vel_z_correction, -max_z_speed, max_z_speed)

                            if self._sim_step % 10 == 0 and abs(z_error) > 0.001: # Log solo se c'è errore significativo
                                print(f"      MOVING_H Z Correction: Error={z_error:.4f}, CorrectionVel={vel_z_correction:.4f}")
                            
                            try:
                                self.cone_prim_handle.set_linear_velocity(
                                    np.array([vel_xy_target[0], vel_xy_target[1], vel_z_correction], dtype=np.float32)
                                )
                            except RuntimeError as e:
                                print(f" MOVING_H ERR (with Z correction): {e}")
                                self.cone_prim_handle = None; self.movement_phase = "done"
                        else: # Raggiunta la destinazione XY
                            print(f"SIM STEP {self._sim_step} (Movement): Reached horizontal target (DistXY: {dist_xy:.4f}). Phase: DESCENDING.")
                            self.movement_phase = "descending"
                            if self.lift_start_z is None: # Safety check
                                print(f"    WARNING: lift_start_z is None before descending. Using current cone Z as target_descent_z.")
                                self.target_descent_z = current_cone_pos[2] 
                            else:
                                self.target_descent_z = self.lift_start_z
                            
                            print(f"    MOVING_H -> DESCENDING: Current Z: {current_cone_pos[2]:.4f}, Target Descent Z: {self.target_descent_z:.4f}")
                            try: 
                                self.cone_prim_handle.set_linear_velocity(np.array([0,0,0],dtype=np.float32)) # Ferma il movimento
                            except RuntimeError as e: 
                                print(f" MOVING_H STOP ERR: {e}")
                                self.cone_prim_handle = None; self.movement_phase = "done"

                    elif self.movement_phase == "descending":
                        grip_stat = "Closed" if self.surface_gripper and self.surface_gripper.is_closed() else "Open/None"
                        if self.target_descent_z is None:
                            print(f"    DESCENDING (Step {self._sim_step}, Grip: {grip_stat}): target_descent_z is None. Skipping to RELEASING.")
                            self.movement_phase = "releasing"
                            try: self.cone_prim_handle.set_linear_velocity(np.array([0,0,0],dtype=np.float32))
                            except RuntimeError as e: print(f" DESCENDING STOP ERR (target None): {e}"); self.cone_prim_handle=None; self.movement_phase="done"
                        elif current_cone_pos[2] > self.target_descent_z + 0.005: # Aggiunta piccola tolleranza
                            if self._sim_step % 10 == 0: print(f"    DESCENDING (Step {self._sim_step}, Grip: {grip_stat}): Current Z: {current_cone_pos[2]:.4f}, Target Z: {self.target_descent_z:.4f}")
                            try: self.cone_prim_handle.set_linear_velocity(np.array([0,0, -self.lift_speed_vertical],dtype=np.float32))
                            except RuntimeError as e: print(f" DESCENDING ERR: {e}"); self.cone_prim_handle=None; self.movement_phase="done"
                        else: 
                            print(f"SIM STEP {self._sim_step} (Movement): Reached descent height (Z: {current_cone_pos[2]:.4f}). Phase: RELEASING.")
                            self.movement_phase = "releasing"
                            try: self.cone_prim_handle.set_linear_velocity(np.array([0,0,0],dtype=np.float32))
                            except RuntimeError as e: print(f" DESCENDING STOP ERR: {e}"); self.cone_prim_handle=None; self.movement_phase="done"

                    elif self.movement_phase == "releasing":
                        print(f"SIM STEP {self._sim_step} (Movement): Phase RELEASING. Opening gripper.")
                        if self.surface_gripper: self.surface_gripper.open()
                        self.movement_phase = "done"
       
        if self.movement_phase == "done":
            if self.cone_prim_handle and self.cone_prim_handle.is_valid():
                try:
                    _, vel_lin, _ = self.cone_prim_handle.get_velocities()
                    if np.linalg.norm(vel_lin) > 0.001:
                        if self._sim_step % 30 == 0: print(f"    DONE (Step {self._sim_step}): Cone still moving. Stopping.")
                        self.cone_prim_handle.set_linear_velocity(np.array([0,0,0],dtype=np.float32))
                except RuntimeError: self.cone_prim_handle = None 
                except Exception: pass


    def run(self):
        if self._timeline.is_playing(): self._timeline.stop()
        self._sim_step = 0
        self.gripper_close_attempted_this_cycle = False; self.cone_spawned_and_positioned = False
        self.movement_phase = "idle"; self.initial_object_position_for_move = None
        self.lift_start_z = None; self.target_descent_z = None
        self.cone_prim_handle = None; self.box_srp_handle = None; self.cone_geom_usd = None
        self.surface_gripper = None
        asyncio.ensure_future(self._setup_initial_scene_async())


    def cleanup(self):
        print("Cleaning up SurfaceGripperDirectScript …")
        if self._timeline.is_playing(): self._timeline.stop()
        if self._physx_sub: self._physx_sub.unsubscribe(); self._physx_sub = None
        self.surface_gripper = None; self.cone_prim_handle = None; self.box_srp_handle = None
        self.cone_geom_usd = None; self._stage = None; self._sim_step = 0
        self.gripper_close_attempted_this_cycle = False; self.cone_spawned_and_positioned = False
        self.movement_phase = "idle"; self.initial_object_position_for_move = None
        self.lift_start_z = None; self.target_descent_z = None
        print("Cleanup complete.")


if "gripper_script_runner_instance" in globals() and gripper_script_runner_instance is not None:
    try: gripper_script_runner_instance.cleanup()
    except Exception as e: print(f"Error during cleanup of previous instance → {e}")
    del gripper_script_runner_instance


print("Spawning new SurfaceGripperDirectScript instance …")
gripper_script_runner_instance = SurfaceGripperDirectScript()


# --- CONFIGURAZIONE ---
print("--- CONFIGURAZIONE ATTIVA ---")
gripper_script_runner_instance.box_mass = 0.010  # Massa del cubo
gripper_script_runner_instance.absolute_grip_force = 20.0 # Forza di presa
# gripper_script_runner_instance.grip_force_multiplier = 50.0 # Non usato se absolute_grip_force è specificato

# Puoi modificare questi valori per testare
gripper_script_runner_instance.target_lift_height = 0.4 # Altezza a cui sollevare il cubo
gripper_script_runner_instance.target_horizontal_position = np.array([0.4, -0.3]) # Posizione XY target

# Parametri di correzione Z (puoi sintonizzarli se necessario)
# gripper_script_runner_instance.z_correction_factor = 10.0
# gripper_script_runner_instance.max_z_correction_speed_factor = 1.5


print(f"Configured Script - Box Mass: {gripper_script_runner_instance.box_mass:.3f} kg")
if gripper_script_runner_instance.absolute_grip_force is not None:
    print(f"Configured Script - Absolute Grip Force: {gripper_script_runner_instance.absolute_grip_force:.2f} N")
else:
    print(f"Configured Script - Grip Force Multiplier: {gripper_script_runner_instance.grip_force_multiplier:.1f}x (Min Force to Hold: {gripper_script_runner_instance.box_mass * 9.81:.2f} N)")
print(f"Configured Script - Target Lift Height: {gripper_script_runner_instance.target_lift_height:.2f} m")
print(f"Configured Script - Target Horizontal Position: {gripper_script_runner_instance.target_horizontal_position}")


gripper_script_runner_instance.run()


