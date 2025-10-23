"""Gripping and grasping workflows orchestrated during simulation."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np

from depal.grippers import pinza as pinza_module
from depal.grippers.surface_gripper import SurfaceGripperDirectScript
from depal.stage.io import StageIO
from depal.utils.image_pick import project_circle_topdown
from depal.utils.k_matrix import generate_k_matrix

from .environment import SimulationEnvironment
from .types import GripOptions, SpawnedEntities


@dataclass
class CameraData:
    prim_path: str
    position: np.ndarray
    rotation: object
    world_to_camera_matrix: np.ndarray
    rotation_matrix: np.ndarray
    translation_vector: np.ndarray
    world_quaternion: object


class GripWorkflow:
    """Coordinates the surface gripper and pinza pipelines."""

    def __init__(self, environment: SimulationEnvironment, script_dir: Path) -> None:
        self._env = environment
        self._script_dir = script_dir
        self._stage_io = StageIO()

    def run(
        self,
        *,
        stage,
        simulation_app,
        img_idx: int,
        paths_cfg: Dict[str, object],
        box_cfg: Dict[str, object],
        ycb_cfg: Dict[str, object],
        spawned_entities: SpawnedEntities,
        grip_options: GripOptions,
    ) -> None:
        if not grip_options.any_enabled():
            return

        # Persist the current stage so both workflows can restore it when needed.
        self._stage_io.save_stage("stage_freeze_temp")

        camera_data = self._collect_camera_data(stage, simulation_app)
        k_matrix = self._resolve_camera_matrix(img_idx, paths_cfg)
        output_pick_dir = Path(self._script_dir, paths_cfg["output_replicator_dir_base"], f"img{img_idx}", "left", "pick")
        output_pick_dir.mkdir(parents=True, exist_ok=True)
        final_image_path = output_pick_dir / "all_boxes_all_cones_combined_img.png"
        if final_image_path.exists():
            try:
                final_image_path.unlink()
            except OSError as exc:
                print(f"Errore durante l'eliminazione del file finale {final_image_path}: {exc}")

        projection_records: List[Dict[str, object]] = []

        if grip_options.enable_surface_gripper:
            projection_records = self._run_surface_gripper_workflow(
                simulation_app=simulation_app,
                stage=stage,
                img_idx=img_idx,
                box_cfg=box_cfg,
                ycb_cfg=ycb_cfg,
                spawned_entities=spawned_entities,
                camera_data=camera_data,
            )

        if grip_options.enable_pinza:
            self._run_pinza_workflow(
                simulation_app=simulation_app,
                img_idx=img_idx,
                box_cfg=box_cfg,
                ycb_cfg=ycb_cfg,
                spawned_entities=spawned_entities,
                camera_data=camera_data,
                k_matrix=k_matrix,
            )

        if grip_options.enable_surface_gripper and projection_records and k_matrix is not None:
            self._generate_projection_image(
                projections=projection_records,
                k_matrix=k_matrix,
                camera_data=camera_data,
                output_path=final_image_path,
            )

    # --------------------------------------------------------------------- #
    # Surface gripper
    # --------------------------------------------------------------------- #
    def _run_surface_gripper_workflow(
        self,
        *,
        simulation_app,
        stage,
        img_idx: int,
        box_cfg: Dict[str, object],
        ycb_cfg: Dict[str, object],
        spawned_entities: SpawnedEntities,
        camera_data: CameraData,
    ) -> List[Dict[str, object]]:
        box_paths = list(spawned_entities.boxes if box_cfg.get("enable") else [])
        object_paths = list(spawned_entities.objects if ycb_cfg.get("enable") else [])
        selected_paths = box_paths or object_paths
        if not selected_paths:
            print("Box insufficienti o box spawning disabilitato.")
            return []

        cfg = self._surface_gripper_config()
        projections: List[Dict[str, object]] = []

        for index, target_path in enumerate(selected_paths):
            print(f"\n  Processing data collection for target {index + 1}/{len(selected_paths)}: {target_path}")
            prim_deformable = self._is_prim_deformable(stage, target_path)
            if prim_deformable:
                print(f"   ATTENZIONE: L'oggetto '{target_path}' è deformabile.")

            physical_parts = self._find_physical_parts(stage, target_path)
            if not physical_parts:
                print(f"-> Nessuna parte fisica trovata per {target_path}. Salto.")
                continue
            print(f"-> Trovate {len(physical_parts)} parti fisiche: {physical_parts}")

            temp_sampler = SurfaceGripperDirectScript(
                target_object_prim_path=target_path,
                grasp_offset_from_top=cfg["grasp_offset"],
                initial_box_prim_path=cfg["initial_box_path"],
                box_mass=cfg["box_mass"],
                absolute_grip_force=cfg["absolute_grip_force"],
                grip_force_multiplier=cfg["grip_force_multiplier"],
                target_lift_height=cfg["target_lift_height"],
                target_horizontal_position=cfg["target_horizontal_position"],
                z_correction_factor=cfg["z_correction_factor"],
                max_z_correction_speed_factor=cfg["max_z_correction_speed_factor"],
                cone_height=cfg["cone_height"],
                cone_radius=cfg["cone_radius"],
                lift_speed_vertical=cfg["lift_speed_vertical"],
                move_speed_horizontal=cfg["move_speed_horizontal"],
                steps_for_cube_to_settle=cfg["steps_for_cube_to_settle"],
                steps_grace_period_after_settle=cfg["steps_grace_period_after_settle"],
                steps_wait_before_grip_attempt=cfg["steps_wait_before_grip_attempt"],
                steps_wait_after_grip_refresh=cfg["steps_wait_after_grip_refresh"],
                prim_deformable=prim_deformable,
                simulation_app=simulation_app,
            )

            candidate_positions = temp_sampler.sample_grasp_grid(target_path)
            del temp_sampler
            print(f"    Found {len(candidate_positions)} potential grasp positions.")

            for attempt_idx, position in enumerate(candidate_positions):
                print(f"\n      Processing grasp attempt {attempt_idx + 1}/{len(candidate_positions)} for {target_path}")
                runner = SurfaceGripperDirectScript(
                    target_object_prim_path=target_path,
                    grasp_offset_from_top=cfg["grasp_offset"],
                    initial_box_prim_path=cfg["initial_box_path"],
                    box_mass=cfg["box_mass"],
                    absolute_grip_force=cfg["absolute_grip_force"],
                    grip_force_multiplier=cfg["grip_force_multiplier"],
                    target_lift_height=cfg["target_lift_height"],
                    target_horizontal_position=cfg["target_horizontal_position"],
                    z_correction_factor=cfg["z_correction_factor"],
                    max_z_correction_speed_factor=cfg["max_z_correction_speed_factor"],
                    cone_height=cfg["cone_height"],
                    cone_radius=cfg["cone_radius"],
                    lift_speed_vertical=cfg["lift_speed_vertical"],
                    move_speed_horizontal=cfg["move_speed_horizontal"],
                    steps_for_cube_to_settle=cfg["steps_for_cube_to_settle"],
                    steps_grace_period_after_settle=cfg["steps_grace_period_after_settle"],
                    steps_wait_before_grip_attempt=cfg["steps_wait_before_grip_attempt"],
                    steps_wait_after_grip_refresh=cfg["steps_wait_after_grip_refresh"],
                    simulation_app=simulation_app,
                )
                runner.run(position)
                while not runner.is_task_complete():
                    simulation_app.update()

                initial_world = getattr(runner, "initial_cone_placement_position_world", None)
                initial_orientation = getattr(runner, "initial_cone_orientation", None)
                grip_status = getattr(runner, "grip_status_code", None)

                if initial_world is not None:
                    projection_info = {
                        "center_world_m": initial_world,
                        "orientation_wxyz": initial_orientation,
                        "circle_color": grip_status,
                        "z_value": initial_world[2],
                        "box_idx": index,
                        "attempt_idx": attempt_idx,
                        "cone_radius": cfg["cone_radius"],
                    }
                    projections.append(projection_info)
                else:
                    print("      WARNING: initial_cone_placement_position_world non definita, salto raccolta proiezioni.")

                self._stage_io.load_stage("stage_freeze_temp/saved_stage.usd")
                for _ in range(50):
                    simulation_app.update()

        return projections

    def _surface_gripper_config(self) -> Dict[str, object]:
        return {
            "initial_box_path": "/Box",
            "box_mass": 0.02,
            "absolute_grip_force": 0.7,
            "grip_force_multiplier": 1.0,
            "target_lift_height": 2.0,
            "target_horizontal_position": np.array([3.0, 3.0]),
            "z_correction_factor": 15.0,
            "max_z_correction_speed_factor": 2.0,
            "cone_height": 0.06,
            "cone_radius": 0.03,
            "lift_speed_vertical": 2.0,
            "move_speed_horizontal": 2.0,
            "steps_for_cube_to_settle": 60,
            "steps_grace_period_after_settle": 30,
            "steps_wait_before_grip_attempt": 120,
            "steps_wait_after_grip_refresh": 40,
            "grasp_offset": 0.06 / 2 + 0.001,
        }

    # --------------------------------------------------------------------- #
    # Pinza workflow
    # --------------------------------------------------------------------- #
    def _run_pinza_workflow(
        self,
        *,
        simulation_app,
        img_idx: int,
        box_cfg: Dict[str, object],
        ycb_cfg: Dict[str, object],
        spawned_entities: SpawnedEntities,
        camera_data: CameraData,
        k_matrix: Optional[np.ndarray],
    ) -> None:
        modules = self._env.modules

        self._stage_io.load_stage("stage_freeze_temp/saved_stage.usd")
        world = modules.World()
        for _ in range(10):
            simulation_app.update()

        target_paths = list(spawned_entities.boxes if box_cfg.get("enable") else [])
        if not target_paths and ycb_cfg.get("enable"):
            target_paths = list(spawned_entities.objects)

        if not target_paths:
            print("Nessun oggetto disponibile per la pinza.")
            return

        grasp_data: Dict[str, List[Dict[str, object]]] = {}
        robot, target_prim = pinza_module.setup_scene(target_paths[0], simulation_app)

        for index, target_path in enumerate(target_paths):
            print(f"\n  Processing pinza grasp for {target_path}")
            self._stage_io.load_stage("stage_freeze_temp/saved_stage.usd")
            robot, target_prim = pinza_module.setup_scene(target_path, simulation_app)

            poses = pinza_module.generate_grasp_poses(target_prim)
            if not poses:
                print(f"ATTENZIONE: Nessuna posa valida generata per {target_path}.")
                continue

            results_for_object: List[Dict[str, object]] = []
            for attempt_idx, (pos, quat) in enumerate(poses):
                print(f"\n--- Pinza attempt {attempt_idx + 1}/{len(poses)} for {target_path} ---")
                start_time = time.time()
                robot.set_world_pose(position=pos, orientation=quat)

                timeline = modules.omni_timeline.get_timeline_interface()
                timeline.play()
                for _ in range(5):
                    simulation_app.update()

                robot.initialize()
                simulation_app.update()

                robot.set_world_pose(position=pos, orientation=quat)
                robot.set_linear_velocity(np.zeros(3))
                robot.set_angular_velocity(np.zeros(3))
                simulation_app.update()

                obb_info = pinza_module._get_obb_info(target_prim)
                fsm = pinza_module.GraspingFSM(robot, target_prim, [(pos, quat)], obb_info, None)
                simulation_app.update()

                timeout_s = 10
                while simulation_app.is_running() and not fsm.is_finished():
                    simulation_app.update()
                    fsm.step()
                    if time.time() - start_time > timeout_s:
                        print(f"!!! Timeout di {timeout_s} secondi raggiunto. Risultato impostato su FAILURE. !!!")
                        fsm.state = pinza_module.State.FINISH
                        fsm.current_result = pinza_module.GraspResult.FAILURE
                        break

                result = fsm.get_result()
                success = "SUCCESS" in result.name.upper()

                position = getattr(fsm.grip_position, "tolist", lambda: [-1, -1, -1])()
                orientation = getattr(fsm.grip_orientation, "tolist", lambda: [-1, -1, -1, -1])()
                if not isinstance(position, list):
                    position = [-1, -1, -1]
                    success = False
                if not isinstance(orientation, list):
                    orientation = [-1, -1, -1, -1]
                    success = False

                attempt_data = {
                    "gripper_position": position,
                    "gripper_orientation_quat_wxyz": orientation,
                    "success": success,
                }
                results_for_object.append(attempt_data)

                self._stage_io.load_stage("stage_freeze_temp/saved_stage.usd")
                robot, target_prim = pinza_module.setup_scene(target_path, simulation_app)

            if results_for_object:
                grasp_data[target_path] = results_for_object

        if grasp_data:
            output_dir = Path(self._script_dir, "output", f"img{img_idx}", "left", "pinza")
            output_dir.mkdir(parents=True, exist_ok=True)
            file_path = output_dir / "grasp_results.json"
            with file_path.open("w", encoding="utf-8") as handle:
                json.dump(grasp_data, handle, indent=4)
            print(f"Dati di presa salvati in {file_path}")
        else:
            print("Nessun dato di presa raccolto per la pinza.")

    # --------------------------------------------------------------------- #
    # Camera helpers
    # --------------------------------------------------------------------- #
    def _collect_camera_data(self, stage, simulation_app) -> CameraData:
        modules = self._env.modules
        pxr = modules.pxr

        stereo_cam_left_path = "/Replicator/stereo_cam/stereo_cam_L_Xform/stereo_cam_L"
        cam_prim = stage.GetPrimAtPath(stereo_cam_left_path)

        simulation_app.update()
        xform_cache = pxr["UsdGeom"].XformCache(pxr["Usd"].TimeCode.Default())
        local_to_world = xform_cache.GetLocalToWorldTransform(cam_prim)
        world_to_camera = local_to_world.GetInverse()

        world_to_camera_np = np.array(world_to_camera)
        rotation_matrix = np.array(world_to_camera.ExtractRotationMatrix(), dtype=np.float64)
        translation = np.array(world_to_camera.ExtractTranslation(), dtype=np.float64)

        cache = pxr["UsdGeom"].XformCache(pxr["Usd"].TimeCode.Default())
        l2w: pxr["Gf"].Matrix4d = cache.GetLocalToWorldTransform(cam_prim)  # type: ignore[index]
        cam_pos = l2w.ExtractTranslation()
        cam_rot = l2w.ExtractRotation()

        return CameraData(
            prim_path=stereo_cam_left_path,
            position=np.array([cam_pos[0], cam_pos[1], cam_pos[2]]),
            rotation=cam_rot,
            world_to_camera_matrix=world_to_camera_np,
            rotation_matrix=rotation_matrix,
            translation_vector=translation,
            world_quaternion=world_to_camera.ExtractRotation().GetQuat(),
        )

    def _resolve_camera_matrix(self, img_idx: int, paths_cfg: Dict[str, object]) -> Optional[np.ndarray]:
        json_relative_path = Path(
            paths_cfg["output_replicator_dir_base"], f"img{img_idx}", "left", "camera_params", "camera_params_0000.json"
        )
        json_file_path = Path(self._script_dir) / json_relative_path
        if not json_file_path.exists():
            print(f"ATTENZIONE: File JSON globale non trovato: {json_file_path}. Impossibile generare K_matrix.")
            return None

        k_matrix = generate_k_matrix(str(json_file_path))
        print(f"K_matrix globale generata: {k_matrix}")
        return k_matrix

    def _generate_projection_image(
        self,
        *,
        projections: Sequence[Dict[str, object]],
        k_matrix: np.ndarray,
        camera_data: CameraData,
        output_path: Path,
    ) -> None:
        sorted_projections = sorted(projections, key=lambda item: item.get("z_value", 0.0))
        for proj in sorted_projections:
            project_circle_topdown(
                radius_m=proj.get("cone_radius", 0.03),
                center_world_m=proj.get("center_world_m"),
                orientation_wxyz=proj.get("orientation_wxyz"),
                camera_height_m=camera_data.position[2],
                K_matrix_3x3=k_matrix,
                image_width=1280,
                image_height=720,
                output_image_path=str(output_path),
                roll_deg=90.0,
                circle_color=proj.get("circle_color"),
            )
        print(f"\n--- Final combined image saved to: {output_path} ---")

    # --------------------------------------------------------------------- #
    # Scene helpers
    # --------------------------------------------------------------------- #
    def _check_hierarchy_for_attribute(self, stage, root_prim_path: str, attr_name: str) -> bool:
        pxr = self._env.modules.pxr
        root_prim = stage.GetPrimAtPath(root_prim_path)
        if not root_prim:
            return False

        for prim in pxr["Usd"].PrimRange(root_prim):
            if prim.HasAttribute(attr_name):
                return True
        return False

    def _is_prim_deformable(self, stage, prim_path: str) -> bool:
        pxr = self._env.modules.pxr
        root_prim = stage.GetPrimAtPath(prim_path)
        if not root_prim:
            return False

        for prim in pxr["Usd"].PrimRange(root_prim):
            if prim.HasAPI(pxr["PhysxSchema"].PhysxDeformableBodyAPI):
                return True
        return False

    def _find_physical_parts(self, stage, root_prim_path: str) -> List[str]:
        pxr = self._env.modules.pxr
        physical_parts: List[str] = []
        root_prim = stage.GetPrimAtPath(root_prim_path)
        if not root_prim:
            return physical_parts

        for prim in pxr["Usd"].PrimRange(root_prim):
            if prim.HasAPI(pxr["UsdPhysics"].CollisionAPI) and prim.IsA(pxr["UsdGeom"].Gprim):
                physical_parts.append(prim.GetPath().pathString)
        return physical_parts
