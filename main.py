import time
from omni.isaac.kit import SimulationApp
import traceback
import os
import random 
import numpy as np 
import yaml
import sys 

import shutil


def main_simulation():
    simulation_app = None
   
    def load_configuration(config_file_path):
        """Carica la configurazione YAML."""
        if not os.path.exists(config_file_path):
            raise FileNotFoundError(f"File di configurazione '{os.path.basename(config_file_path)}' non trovato.")
        with open(config_file_path, 'r') as f:
            config = yaml.safe_load(f)
        print(f"Configurazione caricata da '{os.path.basename(config_file_path)}'.")
        return config
   
    current_script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file_path = os.path.join(current_script_dir, "config.yaml")
    config = load_configuration(config_file_path)
   
    if current_script_dir not in sys.path:
        sys.path.append(current_script_dir)
        print(f"Aggiunta directory '{current_script_dir}' a sys.path per import moduli locali.")
   
    def initialize_simulation_app_internal(sim_setup_config):
        """Inizializza SimulationApp."""
        nonlocal simulation_app
        SETUP = {"headless": sim_setup_config.get('headless', True)}
        if 'renderer' in sim_setup_config and sim_setup_config['renderer']:
            SETUP['renderer'] = sim_setup_config['renderer']
        simulation_app = SimulationApp(SETUP)
        print("SimulationApp inizializzata.")
        return simulation_app
   
    simulation_app = initialize_simulation_app_internal(config['simulation_setup'])
   
    try:
        import omni.usd
        import omni.timeline
        from pxr import UsdGeom, Gf
        from omni.isaac.core.utils import prims as prims_utils
        import omni.replicator.core as rep
        import carb
   
        import src.material_creator as material_creator_module
        import src.scene_creator as scene_creator_module
        import src.box_creator as box_creator_module
        import src.light_creator as light_creator_module
        import src.object_creator as object_creator_module
        import src.container_pallet_creator as container_pallet_creator_module
        import src.grip as grip
        
        from depal_utils import scene_setup_utils 
        from depal_utils import replicator_utils
        from depal_utils import stage_utils
   
        print("Moduli Omni, PXR, SRC e Utils importati con successo post-inizializzazione app.")
    except ImportError as e:
        print(f"ERRORE CRITICO: Importazione moduli fallita dopo inizializzazione SimulationApp: {e}")
        traceback.print_exc()
        if simulation_app: simulation_app.close()
        return 
   
    try:
        paths_cfg = config['paths']
        sim_setup_cfg = config['simulation_setup']
        asset_spawn_cfg = config['asset_spawner']
        box_spawn_cfg = config['box_spawner'] 
        ycb_spawn_cfg = config.get('object_creator_ycb', {}) 
        material_creator_cfg = config.get('material_creator',{})
        probability_activation_wall = config["invisible_wall_during_fall"]["probability_activation"]
        stereo_cam_cfg =config["stereo_camera_setup"]
        
        

        scene_origin_np = np.array(asset_spawn_cfg.get('scene_origin_xyz', [0.0, 0.0, 0.0]))
        material_base_folder_str = paths_cfg['base_materials_usd'] 
        num_images_to_gen = sim_setup_cfg.get('num_images_to_generate', 1)
        texture_dir_abs = os.path.abspath(os.path.join(current_script_dir, paths_cfg['texture_folder_relative']))
        
        for img_idx in range(1, num_images_to_gen + 1):
            wall_activated=False

            print(f"\n--- Inizio generazione immagine {img_idx}/{num_images_to_gen} ---")
               
            stage = scene_setup_utils.setup_new_scene_for_image(
                simulation_app, omni.usd, img_idx, num_images_to_gen
            )
           
               
            pbr_components, pbr_direct_mat_paths = scene_setup_utils.create_scene_materials(
                stage, simulation_app, material_creator_module, texture_dir_abs, material_base_folder_str,material_creator_cfg
            )

            scene_setup_utils.setup_scene_floor(
                stage, scene_creator_module, paths_cfg['floor_prim_usd'], pbr_components
            )
               
            timeline = omni.timeline.get_timeline_interface()
            timeline.play() 
               
            scene_setup_utils.setup_scene_lighting(
                stage, light_creator_module, config['light_creator'], paths_cfg['light_base_usd']
            )
            
            if asset_spawn_cfg.get('enable', True): 
                scene_setup_utils.spawn_main_asset(
                    stage, container_pallet_creator_module, asset_spawn_cfg, 
                    paths_cfg['asset_spawner_parent_usd'], scene_origin_np, 
                    material_base_folder_str, img_idx 
                )
                simulation_app.update()
            else:
                print("Istanziazione asset principale (pallet/container) disabilitata.")

            if(random.random()<probability_activation_wall):
                wall_activated=True
                print("muri attivati")
                wall_paths =scene_setup_utils.spawn_invisible_walls(stage)



            if ycb_spawn_cfg.get('enable', False): 
                scene_setup_utils.spawn_additional_ycb_objects(
                    stage, object_creator_module, ycb_spawn_cfg
                )
                simulation_app.update()
            else:
                print("Istanziazione oggetti YCB disabilitata.")



            if box_spawn_cfg.get('enable', True): 
                spawned_box_prim_paths=scene_setup_utils.spawn_boxes_on_scene(
                    stage, box_creator_module, Gf, box_spawn_cfg, 
                    paths_cfg['box_parent_usd'], scene_origin_np, pbr_direct_mat_paths 
                )
                simulation_app.update()
            else:
                print("Istanziazione scatole (box_spawner) disabilitata.")
            
            print(spawned_box_prim_paths)
            
            for _ in range(50):
                simulation_app.update() 

            if wall_activated:
                scene_setup_utils.disable_walls(stage,wall_paths,remove=False)
   
            print("Setup scena completato. Attesa stabilizzazione fisica...")
            for _ in range(sim_setup_cfg.get('simulation_updates_after_setup', 500)):
                simulation_app.update() 
   
            scene_setup_utils.setup_main_camera(
                stage, prims_utils, Gf, UsdGeom, config['camera'], 
                paths_cfg['camera_prim_usd'], scene_origin_np
            )
            simulation_app.update() 


           



   
            image_output_directory = os.path.join(current_script_dir, paths_cfg['output_replicator_dir_base'], f"img{img_idx}")
            os.makedirs(image_output_directory, exist_ok=True)
   
            #replicator_utils.run_replicator_data_generation(simulation_app, timeline, rep, carb, config['replicator'], paths_cfg['camera_prim_usd'],image_output_directory)



            config_target_prim_to_grasp = "/World/MyTargetCube"
            


            config_initial_box_path = "/Box"
            config_box_mass = 0.020
            config_absolute_grip_force = 5000.0 # Assicurati sia float
            config_grip_force_multiplier = 500.0
            config_lift_speed_vertical = 2 # Modificato per un movimento più lento/controllato




            config_target_lift_height = 5
            config_target_horizontal_position = np.array([3, 3]) # posizione finale cubo
            config_z_correction_factor = 15.0
            config_max_z_correction_speed_factor = 2.0
            config_cone_height = 0.2
            config_cone_radius = 0.1
            config_move_speed_horizontal = 2
            config_grasp_offset = config_cone_height/2




            config_steps_for_cube_to_settle = 60
            config_steps_grace_period_after_settle = 30
            config_steps_wait_before_grip_attempt = 120
            config_steps_wait_after_grip_refresh = 40




            print(f"  Config: Target Prim='{config_target_prim_to_grasp if config_target_prim_to_grasp else f'Default Box ({config_initial_box_path})'}'")
            print(f"  Config: Grasp Offset={config_grasp_offset:.4f}m, Absolute Grip Force={config_absolute_grip_force if config_absolute_grip_force is not None else 'N/A (Using Multiplier)'}, Lift Speed={config_lift_speed_vertical}m/s")


            stage_utils.save_stage_with_pause_and_resume("stage_freeze_temp")

            print("\nCreating new SurfaceGripperDirectScript instance...")
            gripper_script_runner_instance = grip.SurfaceGripperDirectScript(
                
                target_object_prim_path="/World/SpawnedBasicBoxes/BasicBox_0",
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
                lift_speed_vertical=config_lift_speed_vertical,
                move_speed_horizontal=config_move_speed_horizontal,
                steps_for_cube_to_settle=config_steps_for_cube_to_settle,
                steps_grace_period_after_settle=config_steps_grace_period_after_settle,
                steps_wait_before_grip_attempt=config_steps_wait_before_grip_attempt,
                steps_wait_after_grip_refresh=config_steps_wait_after_grip_refresh
            )

           

            print("Running simulation script...")
            gripper_script_runner_instance.run()


            for _ in range(800):
                    simulation_app.update() 
            stage_utils.load_stage_in_new_stage("stage_freeze_temp/saved_stage.usd")


            while True:
                simulation_app.update()
           



         

            def clean_right_output(root_dir, keep=("rgb", "camera_params")):
                """
                Rimuove tutto ciò che c’è in <root_dir> tranne le
                sottocartelle elencate in *keep*.

                Parameters
                ----------
                root_dir : str
                    Percorso alla cartella della camera “right”, es. f"{output_dir_root}/right"
                keep : tuple[str]
                    Sottodirectory da conservare.
                """
                for entry in os.listdir(root_dir):
                    if entry in keep:
                        continue                         # la teniamo
                    path = os.path.join(root_dir, entry)
                    if os.path.isdir(path):
                        shutil.rmtree(path)              # cancella cartella e contenuto
                    else:
                        os.remove(path)                  # cancella file singolo
            for i in range(2):
                simulation_app.update() 
            right_dir = os.path.join(image_output_directory, "right")
            clean_right_output(right_dir)








            
            print(f"--- Generazione immagine {img_idx}/{num_images_to_gen} completata ---")
           
        print(f"\nGenerazione di tutte le {num_images_to_gen} immagini completata.")

        if sim_setup_cfg["headless"] == False:
            while True:
                simulation_app.update()
   
    except FileNotFoundError as fnf_e: 
        print(str(fnf_e))
        traceback.print_exc()
    except Exception as e:
        print(f"ERRORE CRITICO in main_simulation: {e}")
        traceback.print_exc()
    finally:
        if simulation_app:
            print("Chiusura SimulationApp...")
            simulation_app.close()
            print("SimulationApp chiusa.")
        else:
            print("SimulationApp non inizializzata o fallita prima dell'inizializzazione; nessuna chiusura esplicita necessaria.")
   
if __name__ == "__main__":
    main_simulation()