"""High-level simulation pipeline orchestration."""

from __future__ import annotations

import gc
import random
import shutil
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import numpy as np

from .environment import SimulationEnvironment
from .grip_controller import GripWorkflow
from .types import GripOptions, SpawnedEntities
from depal.replicator import data as replicator_data
from depal.scene import setup as scene_setup
from depal.stage.io import StageIO


def clean_output_directory(root_dir: Path, keep: Iterable[str] = ("rgb", "camera_params")) -> None:
    """Remove files in the provided directory while keeping specified subfolders."""
    if not root_dir.exists():
        return
    for entry in root_dir.iterdir():
        if entry.name in keep:
            continue
        if entry.is_dir():
            shutil.rmtree(entry, ignore_errors=True)
        else:
            try:
                entry.unlink()
            except FileNotFoundError:
                pass


class SimulationPipeline:
    """Encapsulates the main simulation workflow."""

    def __init__(self, environment: SimulationEnvironment, script_dir: Path) -> None:
        self._env = environment
        self._script_dir = script_dir
        self._grip_workflow = GripWorkflow(environment, script_dir)
        self._last_spawned = SpawnedEntities()
        self._stage_io = StageIO()

    @property
    def last_spawned(self) -> SpawnedEntities:
        return self._last_spawned

    def run_generation(self, config: Dict[str, object], options: Sequence[str]) -> None:
        simulation_app = self._env.initialize()
        modules = self._env.modules

        paths_cfg = config["paths"]
        sim_setup_cfg = config["simulation_setup"]
        asset_spawn_cfg = config["asset_spawner"]
        box_spawn_cfg = config["box_spawner"]
        ycb_spawn_cfg = config.get("object_creator_ycb", {})
        material_creator_cfg = config.get("material_creator", {})
        probability_activation_wall = config["invisible_wall_during_fall"]["probability_activation"]
        texture_dir = Path(self._script_dir, paths_cfg["texture_folder_relative"]).resolve()
        material_base_folder = paths_cfg["base_materials_usd"]
        scene_origin = np.array(asset_spawn_cfg.get("scene_origin_xyz", [0.0, 0.0, 0.0]), dtype=float)

        grip_options = GripOptions.from_option_list(options)

        num_images = sim_setup_cfg.get("num_images_to_generate", 1)

        for img_idx in range(1, num_images + 1):
            self._reset_stage(simulation_app)
            stage = scene_setup.setup_new_scene_for_image(
                simulation_app, modules.omni_usd, img_idx, num_images
            )
            simulation_app.update()

            pbr_components, pbr_direct_paths = scene_setup.create_scene_materials(
                stage,
                simulation_app,
                str(texture_dir),
                material_base_folder,
                material_creator_cfg,
            )
            simulation_app.update()

            scene_setup.setup_scene_floor(stage, paths_cfg["floor_prim_usd"], pbr_components)
            simulation_app.update()

            timeline = modules.omni_timeline.get_timeline_interface()
            timeline.play()

            scene_setup.setup_scene_lighting(
                stage, config["light_creator"], paths_cfg["light_base_usd"]
            )

            if asset_spawn_cfg.get("enable", True):
                scene_setup.spawn_main_asset(
                    stage,
                    asset_spawn_cfg,
                    paths_cfg["asset_spawner_parent_usd"],
                    scene_origin,
                    material_base_folder,
                    img_idx,
                )
                simulation_app.update()
            else:
                print("Istanziazione asset principale disabilitata.")

            if random.random() < probability_activation_wall:
                print("Muri invisibili attivati.")
                scene_setup.spawn_invisible_walls(stage)

            spawned_objects: List[str] = []
            if ycb_spawn_cfg.get("enable", False):
                spawned_objects = scene_setup.spawn_additional_ycb_objects(stage, ycb_spawn_cfg)
                simulation_app.update()

            spawned_boxes: List[str] = []
            if box_spawn_cfg.get("enable", True):
                spawned_boxes = scene_setup.spawn_boxes_on_scene(
                    stage,
                    box_spawn_cfg,
                    paths_cfg["box_parent_usd"],
                    scene_origin,
                    pbr_direct_paths,
                )
                simulation_app.update()

            for _ in range(50):
                simulation_app.update()

            for _ in range(sim_setup_cfg.get("simulation_updates_after_setup", 500)):
                simulation_app.update()

            scene_setup.setup_main_camera(
                stage,
                modules.prims_utils,
                modules.pxr["Gf"],
                modules.pxr["UsdGeom"],
                config["camera"],
                paths_cfg["camera_prim_usd"],
                scene_origin,
            )
            simulation_app.update()

            output_dir = Path(self._script_dir, paths_cfg["output_replicator_dir_base"], f"img{img_idx}")
            output_dir.mkdir(parents=True, exist_ok=True)

            replicator_data.run_replicator_data_generation(
                simulation_app,
                timeline,
                modules.rep,
                modules.carb,
                config["replicator"],
                paths_cfg["camera_prim_usd"],
                str(output_dir),
            )
            modules.rep.orchestrator.set_capture_on_play(False)

            for _ in range(50):
                simulation_app.update()

            carb_settings = modules.carb.settings.get_settings()
            carb_settings.set_string("/renderer/active", "rtx")
            carb_settings.set_string("/rtx/rendermode", "rtx")

            for _ in range(50):
                simulation_app.update()

            self._last_spawned = SpawnedEntities(boxes=spawned_boxes, objects=spawned_objects)
            self._grip_workflow.run(
                stage=stage,
                simulation_app=simulation_app,
                img_idx=img_idx,
                paths_cfg=paths_cfg,
                box_cfg=box_spawn_cfg,
                ycb_cfg=ycb_spawn_cfg,
                spawned_entities=self._last_spawned,
                grip_options=grip_options,
            )

            for _ in range(2):
                simulation_app.update()

            clean_output_directory(output_dir / "right")
            print(f"--- Generazione immagine {img_idx}/{num_images} completata ---")

    def regenerate_data(self, config: Dict[str, object], options: Sequence[str]) -> None:
        if not self._last_spawned.any():
            print("Nessuna entità istanziata in precedenza, impossibile rigenerare dati.")
            return

        simulation_app = self._env.initialize()
        modules = self._env.modules

        grip_options = GripOptions.from_option_list(options)
        img_idx = config["simulation_setup"]["num_images_to_generate"]
        output_dir = Path(self._script_dir, config["paths"]["output_replicator_dir_base"], f"img{img_idx}")
        output_dir.mkdir(parents=True, exist_ok=True)

        self._stage_io.load_stage("stage_freeze_temp/saved_stage.usd")

        world = modules.World()
        stage = world.stage

        timeline = modules.omni_timeline.get_timeline_interface()
        for _ in range(200):
            simulation_app.update()

        replicator_data.run_replicator_data_generation(
            simulation_app,
            timeline,
            modules.rep,
            modules.carb,
            config["replicator"],
            config["paths"]["camera_prim_usd"],
            str(output_dir),
        )

        for _ in range(50):
            simulation_app.update()

        modules.rep.orchestrator.set_capture_on_play(False)
        carb_settings = modules.carb.settings.get_settings()
        carb_settings.set_string("/renderer/active", "rtx")
        carb_settings.set_string("/rtx/rendermode", "rtx")

        clean_output_directory(output_dir / "right")

        self._grip_workflow.run(
            stage=stage,
            simulation_app=simulation_app,
            img_idx=img_idx,
            paths_cfg=config["paths"],
            box_cfg=config["box_spawner"],
            ycb_cfg=config.get("object_creator_ycb", {}),
            spawned_entities=self._last_spawned,
            grip_options=grip_options,
        )

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def _reset_stage(self, simulation_app) -> None:
        modules = self._env.modules
        world = modules.World()
        world.clear()
        modules.rep.orchestrator.stop()
        gc.collect()

        usd_context = modules.omni_usd.get_context()
        if usd_context.get_stage():
            usd_context.close_stage()
            print("Stage chiuso.")

        for _ in range(200):
            simulation_app.update()
