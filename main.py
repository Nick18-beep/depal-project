import time
from omni.isaac.kit import SimulationApp
import traceback
import os
import random 
import numpy as np 
import yaml
import sys 
import json
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
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Usd
   
        import src.material_creator as material_creator_module
        import src.scene_creator as scene_creator_module
        import src.box_creator as box_creator_module
        import src.light_creator as light_creator_module
        import src.object_creator as object_creator_module
        import src.container_pallet_creator as container_pallet_creator_module
        import src.grip as grip
        import src.pinza as pinza
        

        from depal_utils import scene_setup_utils 
        from depal_utils import replicator_utils
        from depal_utils import stage_utils
        from depal_utils import generate_K_matrix
        from depal_utils import generate_image_pick

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
                spawned_object_prim_paths=scene_setup_utils.spawn_additional_ycb_objects(
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
   
            replicator_utils.run_replicator_data_generation(simulation_app, timeline, rep, carb, config['replicator'], paths_cfg['camera_prim_usd'],image_output_directory)
            rep.orchestrator.set_capture_on_play(False)


                    
            

            

            carb_s = carb.settings.get_settings()
            carb_s.set_string("/renderer/active", "rtx")
            carb_s.set_string("/rtx/rendermode", "rtx")

            config_target_prim_to_grasp = "/World/MyTargetCube"
            


            config_initial_box_path = "/Box"
            config_box_mass = 0.02
            config_absolute_grip_force = 0.65 # Assicurati sia float 5000.0  0.6
            config_grip_force_multiplier = 1.0 #500.0
            config_lift_speed_vertical = 2 # Modificato per un movimento più lento/controllato




            config_target_lift_height = 2 #5
            config_target_horizontal_position = np.array([3, 3]) # posizione finale cubo 3 3
            config_z_correction_factor = 15.0
            config_max_z_correction_speed_factor = 2.0

            config_cone_height = 0.06
            config_cone_radius = 0.03
            config_move_speed_horizontal = 2
            config_grasp_offset = config_cone_height/2 + 0.005




            config_steps_for_cube_to_settle = 60
            config_steps_grace_period_after_settle = 30
            config_steps_wait_before_grip_attempt = 120
            config_steps_wait_after_grip_refresh = 40




            print(f"  Config: Target Prim='{config_target_prim_to_grasp if config_target_prim_to_grasp else f'Default Box ({config_initial_box_path})'}'")
            print(f"  Config: Grasp Offset={config_grasp_offset:.4f}m, Absolute Grip Force={config_absolute_grip_force if config_absolute_grip_force is not None else 'N/A (Using Multiplier)'}, Lift Speed={config_lift_speed_vertical}m/s")


            # --- path della camera -------------------------------------------------
            stereo_cam_left_path = "/Replicator/stereo_cam/stereo_cam_L_Xform/stereo_cam_L"
            cam_prim = stage.GetPrimAtPath(stereo_cam_left_path)


                    # --- matrice world→camera ---------------------------------------------
                    # 1) local→world (Gf.Matrix4d)
            xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            local_to_world_gf = xform_cache.GetLocalToWorldTransform(cam_prim)


                    # 2) world→camera = inverse(local→world)
            world_to_camera_gf = local_to_world_gf.GetInverse()        # ancora Gf.Matrix4d


                    # 3) converti in numpy e separa R e t
            world_to_camera_np = np.array(world_to_camera_gf)          # shape (4,4)
            R_world2cam = world_to_camera_np[:3, :3]                   # rotazione 3×3
            t_world2cam = world_to_camera_np[:3,  3]                   # traslazione 3×1


            print("R_world2cam =\n", R_world2cam)
            print("t_world2cam =\n", t_world2cam)
            cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            l2w: Gf.Matrix4d = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(stereo_cam_left_path))
            cam_pos = l2w.ExtractTranslation()          # Gf.Vec3d (x,y,z)

            for _ in range(20):
                simulation_app.update() 

            stage_utils.save_stage_with_pause_and_resume("stage_freeze_temp")
            
            
                        
            # --- INIZIALIZZAZIONE GLOBALE ---
            all_projections_across_all_boxes = [] # Lista per tutti i dati di proiezione

            # Definisci il percorso per l'UNICA immagine di output finale
            output_pick_dir_global = os.path.join(current_script_dir, paths_cfg['output_replicator_dir_base'], f"img{img_idx}", "left", "pick")
            os.makedirs(output_pick_dir_global, exist_ok=True)
            final_combined_image_path = os.path.join(output_pick_dir_global, f"all_boxes_all_cones_combined_img.png")

            print(f"--- Global Setup ---")
            print(f"Final combined image will be saved to: {final_combined_image_path}")

            # Elimina l'immagine finale se esiste già, per iniziare da zero ad ogni esecuzione
            if os.path.exists(final_combined_image_path):
                print(f"Deleting existing global image file: {final_combined_image_path}")
                try:
                    os.remove(final_combined_image_path)
                except OSError as e:
                    print(f"Error deleting file {final_combined_image_path}: {e}")
                    # Potresti voler gestire questo errore in modo più robusto, es. uscendo

            # Calcola K_matrix una volta
            script_dir_local = os.path.dirname(os.path.abspath(__file__))
            relative_path_to_json_global = os.path.join("output", f"img{img_idx}", "left", "camera_params", "camera_params_0000.json")
            json_file_path_global = os.path.abspath(os.path.join(script_dir_local, relative_path_to_json_global))

            K_matrix_global = None
            print(f"Calculating global K_matrix from: {json_file_path_global}")
            if not os.path.exists(json_file_path_global):
                print(f"ATTENZIONE: File JSON globale non trovato: {json_file_path_global}. Impossibile generare K_matrix.")
            else:
                K_matrix_global = generate_K_matrix.generate_k_matrix(json_file_path_global)
                print(f"K_matrix globale generata: {K_matrix_global}")

            if K_matrix_global is None:
                print("ERRORE CRITICO: K_matrix globale non disponibile. Impossibile procedere con la generazione delle immagini.")
                # Esci o gestisci l'errore come preferisci se K_matrix è essenziale
                # exit() # Esempio

            # --- FINE INIZIALIZZAZIONE GLOBALE ---


            USA_PINZA = True




            if USA_PINZA:


                # MODIFICA: Inizializza un DIZIONARIO per raccogliere i dati raggruppati per oggetto.
                grasp_data_by_object = {}


                # MODIFICA: Ottieni il quaternione di rotazione dalla matrice R_world2cam.
                # Questo quaternione rappresenta l'orientamento della camera rispetto al mondo
                # e ci servirà per trasformare l'orientamento del gripper.
                rot_matrix_gf = Gf.Matrix3d(
                    R_world2cam[0,0], R_world2cam[0,1], R_world2cam[0,2],
                    R_world2cam[1,0], R_world2cam[1,1], R_world2cam[1,2],
                    R_world2cam[2,0], R_world2cam[2,1], R_world2cam[2,2]
                )
                Q_cam_world = rot_matrix_gf.ExtractRotation().GetQuat()




                selected=None




                if box_spawn_cfg['enable']:
                    selected = spawned_box_prim_paths
                if ycb_spawn_cfg['enable']:
                    selected = spawned_object_prim_paths
                   




                for i, box_prim_path_original in enumerate(selected):
                               
                    print(f"\n  Processing data collection for box {i+1}")
                    box_path=None
                    if box_spawn_cfg['enable']:
                        box_path = f"/World/SpawnedBasicBoxes/BasicBox_{i}"
                    if ycb_spawn_cfg['enable']:
                        box_path = f"/World/GeneratedYCBObjects/SpawnedObject__{i}"




                   
                    print(f"    Targeting box prim path: {box_path}")




                    target_prim_path = box_path
                   
                    if i==0:
                        robot ,target_prim = pinza.setup_scene( target_prim_path ,simulation_app)
                        print("robot creato")
                       
                       
                        for _ in range(30):
                            simulation_app.update()
                        robot.initialize()
                        for _ in range(30):
                            simulation_app.update()
                   
                    poses = pinza.generate_grasp_poses(target_prim)
                    if not poses:
                        print(f"ATTENZIONE: Nessuna posa di presa valida generata per {target_prim_path}. Salto l'oggetto.")
                        continue
                   
                    all_results_for_this_object = []
                    print("le posizioni che testo sono: ",len(poses))
                    



                    for p_idx, p in enumerate(poses):
                       
                        print(f"\n--- Inizio tentativo di presa {p_idx + 1}/{len(poses)} per l'oggetto {target_prim_path} ---")


                        # 'p' contiene la posa in coordinate del MONDO
                        first_pos, first_quat = p
                        
                        # Il robot deve essere posizionato nel mondo, quindi usiamo le coordinate originali
                        robot.set_world_pose(position=first_pos, orientation=first_quat)
                   
                        print("✓ Avvio della simulazione e del motore fisico...")
                        timeline.play()
                        for _ in range(5): simulation_app.update()




                        print("✓ Inizializzazione della fisica del robot...")
                        robot.initialize()

                        
                        simulation_app.update()




                        print("✓ Sincronizzazione della posa fisica iniziale...")
                        robot.set_world_pose(position=first_pos, orientation=first_quat)
                        robot.set_linear_velocity(np.zeros(3))
                        robot.set_angular_velocity(np.zeros(3))
                        simulation_app.update()




                        obb_info = pinza._get_obb_info(target_prim)
                        fsm = pinza.GraspingFSM( robot, target_prim,[p], obb_info,stage_utils )
                        simulation_app.update()
                        
                        gripper_pos, gripper_orientation = robot.get_world_pose()
                        print(f"Posizione della pinza: {gripper_pos}")
                            
                        print(f"Orientamento della pinza: {gripper_orientation}")

                        
                           
                        while simulation_app.is_running() and not fsm.is_finished():
                            simulation_app.update()
                            fsm.step()
                            
                            




                     
                        result = fsm.get_result()
                        all_results_for_this_object.append(result)
                        print(f"--- Risultato per Posa: {result.name} ---")




                        # --- MODIFICA: Trasformazione coordinate da World a Camera SINISTRA ---
                        # 1. Trasforma la POSIZIONE
                        position_in_cam_coords = R_world2cam @ first_pos + t_world2cam


                        # 2. Trasforma l'ORIENTAMENTO (quaternione)
                        gripper_quat_world = Gf.Quatd(first_quat[0], Gf.Vec3d(first_quat[1], first_quat[2], first_quat[3]))
                        gripper_quat_cam = Q_cam_world * gripper_quat_world # Applica la rotazione della camera
                        
                        # 3. Riconverti in un formato salvabile (lista)
                        position_list = position_in_cam_coords.tolist()
                        orientation_quat_wxyz = [
                            gripper_quat_cam.GetReal(), 
                            *gripper_quat_cam.GetImaginary()
                        ]
                        # --- FINE MODIFICA ---


                        is_success = "SUCCESS" in result.name.upper()
                        
                        # Crea il dizionario usando le nuove coordinate trasformate
                        pose_result_data = {
                            "gripper_position": position_list,
                            "gripper_orientation_quat_wxyz": orientation_quat_wxyz,
                            "success": is_success
                        }


                        grasp_data_by_object.setdefault(target_prim_path, []).append(pose_result_data)




                        print("✓ Ricarico la scena per il prossimo tentativo...")
                        stage_utils.load_stage_in_new_stage("stage_freeze_temp/saved_stage.usd")
                        robot ,target_prim = pinza.setup_scene( target_prim_path ,simulation_app)
                   
                    print(f"\n--- Riepilogo risultati per l'oggetto {target_prim_path} ---")
                    for i_res, r in enumerate(all_results_for_this_object):
                        print(f"  Posa {i_res}: {r.name}")
               
                print("\n--- Tutti gli oggetti sono stati processati. ---")




                if grasp_data_by_object:
                    output_pinza_dir = os.path.join(
                        current_script_dir, 
                        "output", 
                        f"img{img_idx}", 
                        "left", 
                        "pinza"
                    )
                    os.makedirs(output_pinza_dir, exist_ok=True)
                    
                    json_file_path = os.path.join(output_pinza_dir, "grasp_results.json")
                    
                    print(f"\n--- Salvataggio dei risultati della pinza in: {json_file_path} ---")
                    
                    try:
                        with open(json_file_path, 'w') as f:
                            json.dump(grasp_data_by_object, f, indent=4)
                        print(f"✓ Dati di presa per {len(grasp_data_by_object)} oggetti (in coordinate camera) salvati con successo.")
                    except Exception as e:
                        print(f"ERRORE durante il salvataggio del file JSON: {e}")
                else:
                    print("\n--- Nessun dato di presa raccolto, nessun file JSON da salvare. ---")




                print("Simulazione completata.")






                   
                        
                
            


            
           



         

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