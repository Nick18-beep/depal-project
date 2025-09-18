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
from flask import Flask, jsonify, send_from_directory, request
import threading
import gc

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
        global simulation_app
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
        from pxr import Usd, PhysxSchema
        from omni.isaac.core.utils import prims as prims_utils
        import omni.replicator.core as rep
        import carb
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Usd
        from omni.isaac.core import World
        from omni.usd import get_context
   
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


def check_hierarchy_for_attribute(stage: Usd.Stage, root_prim_path: str, attr_name: str) -> bool:
    """
    Controlla se un attributo esiste su un prim o su uno qualsiasi dei suoi discendenti.

    Args:
        stage (Usd.Stage): Lo stage USD corrente.
        root_prim_path (str): Il percorso del prim radice da cui iniziare la ricerca.
        attr_name (str): Il nome dell'attributo da cercare.

    Returns:
        bool: True se l'attributo viene trovato, altrimenti False.
    """
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim:
        return False

    # Usd.PrimRange itera sul prim radice e su TUTTI i suoi discendenti, a qualsiasi livello.
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAttribute(attr_name):
            return True  # Trovato! Interrompi la ricerca e restituisci True.
            
    return False # Se il loop finisce, l'attributo non è stato trovato.

def is_prim_deformable(stage: Usd.Stage, prim_path: str) -> bool:
    """
    Controlla se un prim, o una qualsiasi delle sue mesh discendenti,
    ha l'API PhysxDeformableBody applicata.
    Restituisce True se è deformabile, altrimenti False.
    """
    root_prim = stage.GetPrimAtPath(prim_path)
    if not root_prim:
        return False

    # Itera sul prim radice e su tutti i suoi discendenti
    for prim in Usd.PrimRange(root_prim):
        # HasAPI() è il modo più pulito e diretto per controllare
        if prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
            # Trovata una parte deformabile, l'intero oggetto è considerato tale
            return True
            
    return False


def grip_pinza(USA_GRIP, USA_PINZA, stage, simulation_app, img_idx, box_spawn_cfg, ycb_spawn_cfg, config, paths_cfg,spawned_box_prim_paths, spawned_object_prim_paths):

            config_target_prim_to_grasp = "/World/MyTargetCube"
            
            config_initial_box_path = "/Box"
            config_box_mass = 0.02
            config_absolute_grip_force = 0.7 # Assicurati sia float 5000.0  0.6
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
            simulation_app.update()
            xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            local_to_world_gf = xform_cache.GetLocalToWorldTransform(cam_prim)

            
                    # 2) world→camera = inverse(local→world)
            world_to_camera_gf = local_to_world_gf.GetInverse()        # ancora Gf.Matrix4d

            print(world_to_camera_gf)
                    # 3) converti in numpy e separa R e t
            world_to_camera_np = np.array(world_to_camera_gf)          # shape (4,4)
            R_world2cam = world_to_camera_np[:3, :3]                   # rotazione 3×3
            t_world2cam = world_to_camera_np[:3,  3]                   # traslazione 3×1
            R_world2cam = np.array(world_to_camera_gf.ExtractRotationMatrix(), dtype=np.float64)
            t_world2cam = np.array(world_to_camera_gf.ExtractTranslation(), dtype=np.float64)

            # Estrai il quaternione di trasformazione per l'orientamento
            Q_world2cam = world_to_camera_gf.ExtractRotation().GetQuat()


            print("R_world2cam =\n", R_world2cam)
            print("t_world2cam =\n", t_world2cam)

            
            cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            l2w: Gf.Matrix4d = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(stereo_cam_left_path))
            cam_pos = l2w.ExtractTranslation()          # Gf.Vec3d (x,y,z)
            cam_rot = l2w.ExtractRotation()
            print("pos camera : ",cam_pos)
            print("rot camera : ",cam_rot)
            


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


           # USA_GRIP = False

            if USA_GRIP:
               
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

                    prim_deformable=False
                    print(f"    Targeting box prim path: {box_path}")

                    if is_prim_deformable(stage, box_path):
                        print(f"   ATTENZIONE: L'oggetto '{box_path}' è deformabile. ")
                        #prim_deformable=True
                        #continue # Salta il resto del loop per questo oggetto

                    # --- CODICE ALTERNATIVO (PIÙ COMPLESSO) ---

                    if check_hierarchy_for_attribute(stage, box_path, "custom:is_pickable"):
                        print(f"-> ⏭️  L'oggetto '{box_path}' o uno dei suoi figli ha il flag 'is_pickable'.")
                        prim_deformable=True

                   
                    # Non è strettamente necessario creare un'istanza qui se `sample_grasp_grid`
                    # potesse essere una funzione statica o non dipendente dallo stato dell'istanza
                    # che viene modificato da run(). Per coerenza con il codice precedente:
                    print("    Creating temporary SurfaceGripperDirectScript instance for sampling grasp grid...")
                    temp_sampler_instance = grip.SurfaceGripperDirectScript(
                        target_object_prim_path=box_path,
                        grasp_offset_from_top=config_grasp_offset, initial_box_prim_path=config_initial_box_path,
                        box_mass=config_box_mass, absolute_grip_force=config_absolute_grip_force,
                        grip_force_multiplier=config_grip_force_multiplier, target_lift_height=config_target_lift_height,
                        target_horizontal_position=config_target_horizontal_position, z_correction_factor=config_z_correction_factor,
                        max_z_correction_speed_factor=config_max_z_correction_speed_factor, cone_height=config_cone_height,
                        cone_radius=config_cone_radius, lift_speed_vertical=config_lift_speed_vertical,
                        move_speed_horizontal=config_move_speed_horizontal, steps_for_cube_to_settle=config_steps_for_cube_to_settle,
                        steps_grace_period_after_settle=config_steps_grace_period_after_settle,
                        steps_wait_before_grip_attempt=config_steps_wait_before_grip_attempt,
                        steps_wait_after_grip_refresh=config_steps_wait_after_grip_refresh,
                        prim_deformable=prim_deformable,
                        simulation_app=simulation_app
                    )
                    cone_positions = temp_sampler_instance.sample_grasp_grid(box_path)
                    del temp_sampler_instance

                    print(f"    Found {len(cone_positions)} potential grasp positions for box {i+1}.")

                    for pos_idx, position in enumerate(cone_positions):
                        print(f"\n      Processing grasp attempt {pos_idx+1}/{len(cone_positions)} for box {i+1} at position: {position}")

                        print("      Creating new SurfaceGripperDirectScript instance for this attempt...")
                        gripper_script_runner_instance = grip.SurfaceGripperDirectScript(
                            target_object_prim_path=box_path,
                            grasp_offset_from_top=config_grasp_offset, initial_box_prim_path=config_initial_box_path,
                            box_mass=config_box_mass, absolute_grip_force=config_absolute_grip_force,
                            grip_force_multiplier=config_grip_force_multiplier, target_lift_height=config_target_lift_height,
                            target_horizontal_position=config_target_horizontal_position, z_correction_factor=config_z_correction_factor,
                            max_z_correction_speed_factor=config_max_z_correction_speed_factor, cone_height=config_cone_height,
                            cone_radius=config_cone_radius, lift_speed_vertical=config_lift_speed_vertical,
                            move_speed_horizontal=config_move_speed_horizontal, steps_for_cube_to_settle=config_steps_for_cube_to_settle,
                            steps_grace_period_after_settle=config_steps_grace_period_after_settle,
                            steps_wait_before_grip_attempt=config_steps_wait_before_grip_attempt,
                            steps_wait_after_grip_refresh=config_steps_wait_after_grip_refresh,
                            simulation_app=simulation_app
                        )

                        print("      Running simulation script for this attempt...")
                        gripper_script_runner_instance.run(position)
                        while not gripper_script_runner_instance.is_task_complete():
                            simulation_app.update()
                        
                        print(f"      Simulation complete for attempt {pos_idx+1}.")
                        print(f"      Grip status code: {gripper_script_runner_instance.grip_status_code}")
                        # print(f"      Initial cone placement (world): {gripper_script_runner_instance.initial_cone_placement_position_world}")
                        # print(f"      Initial cone orientation: {gripper_script_runner_instance.initial_cone_orientation}")

                        if gripper_script_runner_instance.initial_cone_placement_position_world is not None:
                            projection_info = {
                                'center_world_m': gripper_script_runner_instance.initial_cone_placement_position_world,
                                'orientation_wxyz': gripper_script_runner_instance.initial_cone_orientation,
                                'circle_color': gripper_script_runner_instance.grip_status_code,
                                'z_value': gripper_script_runner_instance.initial_cone_placement_position_world[2],
                                'box_idx': i, # Opzionale: per debug o info future
                                'attempt_idx': pos_idx # Opzionale: per debug o info future
                            }
                            all_projections_across_all_boxes.append(projection_info)
                        else:
                            print(f"      WARNING: initial_cone_placement_position_world is None for attempt {pos_idx+1}, box {i+1}. Skipping projection data collection.")
                        
                        stage_utils.load_stage_in_new_stage("stage_freeze_temp/saved_stage.usd") # Se necessario dopo ogni run()
                        for _ in range (50):
                            simulation_app.update()


                # --- FINE RACCOLTA DATI PER TUTTI I BOX ---
                simulation_app.update()
                simulation_app.update()

                if not all_projections_across_all_boxes:
                    print("\n--- No projection data collected from any box. No image will be generated. ---")
                elif K_matrix_global is None:
                    print("\n--- K_matrix globale non disponibile. Impossibile generare l'immagine combinata. ---")
                else:
                    print(f"\n--- Preparing to draw {len(all_projections_across_all_boxes)} projections onto a single image ---")
                    
                    print("Sorting all collected projections by Z-value...")
                    all_projections_across_all_boxes.sort(key=lambda p: p['z_value'])

                    print(f"Generating combined image by repeatedly calling project_circle_topdown...")
                    print(f"Camera height (cam_pos[2]): {cam_pos[2]}") # Assicurati che cam_pos sia definito globalmente

                    for proj_idx, proj_data in enumerate(all_projections_across_all_boxes):
                        # print(f"\n  Drawing projection {proj_idx+1}/{len(all_projections_across_all_boxes)} (Box {proj_data['box_idx']+1}, Attempt {proj_data['attempt_idx']+1}) on {final_combined_image_path}")
                        # print(f"    Center: {proj_data['center_world_m']}, Z-value: {proj_data['z_value']:.4f}")
                        # print(f"    Orientation: {proj_data['orientation_wxyz']}")
                        # print(f"    Circle Color (Grip Status): {proj_data['circle_color']}")
                        
                        generate_image_pick.project_circle_topdown(
                            radius_m          = config_cone_radius, # Assicurati che config_cone_radius sia definito globalmente
                            center_world_m    = proj_data['center_world_m'],
                            orientation_wxyz  = proj_data['orientation_wxyz'],
                            camera_height_m   = cam_pos[2],
                                    K_matrix_3x3      = K_matrix_global,
                            image_width       = 1280, # O da configurazione
                            image_height      = 720,  # O da configurazione
                            output_image_path = final_combined_image_path, # SEMPRE LO STESSO PATH GLOBALE
                            roll_deg          = 90.0, # O da configurazione
                            circle_color      = proj_data['circle_color'],
                        )
                        # print(f"    Projection {proj_idx+1} drawn.")
                    
                    print(f"\n--- Final combined image saved to: {final_combined_image_path} ---")

                # print("\n--- All boxes processed, reloading stage ---") # Se necessario alla fine di tutto
                # stage_utils.load_stage_in_new_stage("stage_freeze_temp/saved_stage.usd")
                

            else:
                print("Box insufficienti o box spawning disabilitato.")

 
            

           # USA_PINZA = True
        



            if USA_PINZA:
                from omni.isaac.core import SimulationContext
                from omni.isaac.core import World

                box_path=None
                if box_spawn_cfg['enable']:
                            box_path = f"/World/SpawnedBasicBoxes/BasicBox_{0}"
                if ycb_spawn_cfg['enable']:
                            box_path = f"/World/GeneratedYCBObjects/SpawnedObject__{0}"

                stage_utils.load_stage_in_new_stage("stage_freeze_temp/saved_stage.usd")
                w=World()
                for i in range(10):
                    simulation_app.update()
                robot ,target_prim = pinza.setup_scene( box_path ,simulation_app)


                # MODIFICA: Inizializza un DIZIONARIO per raccogliere i dati raggruppati per oggetto.
                grasp_data_by_object = {}

                
                # MODIFICA: Ottieni il quaternione di rotazione dalla matrice R_world2cam.
                # Questo quaternione rappresenta l'orientamento della camera rispetto al mondo
                # e ci servirà per trasformare l'orientamento del gripper.
                




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
                    if is_prim_deformable(stage, box_path):
                        print(f"   ATTENZIONE: L'oggetto '{box_path}' è deformabile. Salto al prossimo.")
                        continue # Salta il resto del loop per questo oggetto




                    target_prim_path = box_path
                   
                    if i==-1:
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
                    

                    import time 
                   
                    time_out = 10 # secondi

                    for p_idx, p in enumerate(poses):
                        start_time = time.time()
                        print(f"\n--- Inizio tentativo di presa {p_idx + 1}/{len(poses)} per l'oggetto {target_prim_path} ---")


                        # 'p' contiene la posa in coordinate del MONDO
                        first_pos, first_quat = p
                        
                        # Il robot deve essere posizionato nel mondo, quindi usiamo le coordinate originali
                        robot.set_world_pose(position=first_pos, orientation=first_quat)
                        timeline = omni.timeline.get_timeline_interface()
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
                             # Controlla se il tempo è scaduto
                            if time.time() - start_time > time_out:
                                print(f"!!! Timeout di {time_out} secondi raggiunto. Imposto il risultato su FAILURE. !!!")
                                
                                # Imposta manualmente lo stato e il risultato come richiesto
                                fsm.state = pinza.State.FINISH
                                fsm.current_result = pinza.GraspResult.FAILURE
                                
                               
                            
                            




                     
                        result = fsm.get_result()
                        all_results_for_this_object.append(result)
                        print(f"--- Risultato per Posa: {result.name} ---")




                        #position_in_cam_coords = R_world2cam @ fsm.grip_position + t_world2cam
                       
                        # 2. Trasforma l'ORIENTAMENTO dal mondo alla camera
                        # Gestisci il tipo di dato per evitare errori (float32 vs double)
                        #w = float(fsm.grip_orientation[0])
                       # x = float(fsm.grip_orientation[1])
                        #y = float(fsm.grip_orientation[2])
                        #z = float(fsm.grip_orientation[3])
                        #gripper_quat_world = Gf.Quatd(w, Gf.Vec3d(x, y, z))

                        # Esegui la trasformazione dell'orientamento
                        #gripper_quat_cam = Q_world2cam * gripper_quat_world   

                        # 3. Prepara i dati finali per il salvataggio
                        #final_position_list = position_in_cam_coords.tolist()
                        #final_orientation_list = [
                        #    gripper_quat_cam.GetReal(),
                       #     *gripper_quat_cam.GetImaginary()
                        #]

                        # 4. Determina il successo della presa
                        is_success = "SUCCESS" in result.name.upper()

                        # Prova a convertire la posizione
                        try:
                            # Tenta di eseguire l'operazione che potrebbe fallire
                            position = fsm.grip_position.tolist()
                        except :
                            # Se .tolist() non esiste, viene eseguito questo blocco
                            position = [-1, -1, -1] # Assegna un valore di errore
                            is_success = False

                        # Fai lo stesso per l'orientamento
                        try:
                            orientation = fsm.grip_orientation.tolist()
                        except :
                            orientation = [-1, -1, -1, -1]
                            is_success = False # Se anche solo uno fallisce, il risultato non è un successo


                        # 5. Crea il dizionario usando i dati TRASFORMATI
                        pose_result_data = {
                            "gripper_position": position,
                            "gripper_orientation_quat_wxyz": orientation,
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


spawned_object_prim_paths=None
spawned_box_prim_paths=None



def main_simulation(config_path=None,options=None):
    global config  # 👈 Dichiara che vuoi modificare la 'config' globale

    # 1. Logica corretta: carica se il percorso NON è None
    if config_path is not None:
        print(f"Sovrascrivo la configurazione globale dal percorso: {config_path}")
        config = load_configuration(config_path) # Ora questo modifica la variabile globale
    else:
        print("Nessun config_path fornito, uso la configurazione globale esistente.")


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
            
            #provo a ripulire
            w=World()
            w.clear()
            rep.orchestrator.stop()
            # 2. Ottieni il contesto USD e chiudi lo stage
            gc.collect() # Forza il garbage collector di Python

            usd_context = get_context()
            if usd_context.get_stage():
                usd_context.close_stage()
                print("Stage chiuso.")
            

            for _ in range(200):
                simulation_app.update() 

            wall_activated=False

            USA_GRIP = 'grip' in options 
            USA_PINZA = 'pinza' in options

            print(f"\n--- Inizio generazione immagine {img_idx}/{num_images_to_gen} ---")
               
            stage = scene_setup_utils.setup_new_scene_for_image(
                simulation_app, omni.usd, img_idx, num_images_to_gen
            )
            simulation_app.update()
           
               
            pbr_components, pbr_direct_mat_paths = scene_setup_utils.create_scene_materials(
                stage, simulation_app, material_creator_module, texture_dir_abs, material_base_folder_str,material_creator_cfg
            )
            simulation_app.update()

            scene_setup_utils.setup_scene_floor(
                stage, scene_creator_module, paths_cfg['floor_prim_usd'], pbr_components
            )
            simulation_app.update()
               
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


            
            global spawned_object_prim_paths
            if ycb_spawn_cfg.get('enable', False): 
                spawned_object_prim_paths=scene_setup_utils.spawn_additional_ycb_objects(
                    stage, object_creator_module, ycb_spawn_cfg
                )
                simulation_app.update()
                
            else:
                print("Istanziazione oggetti YCB disabilitata.")


            global spawned_box_prim_paths
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

            #if wall_activated:
            #    scene_setup_utils.disable_walls(stage,wall_paths,remove=False)
   
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
            for i in range(50):
                simulation_app.update()

            carb_s = carb.settings.get_settings()
            carb_s.set_string("/renderer/active", "rtx")
            carb_s.set_string("/rtx/rendermode", "rtx")



            

            for i in range(50):
             simulation_app.update()

            grip_pinza(USA_GRIP=USA_GRIP, USA_PINZA=USA_PINZA, stage=stage, simulation_app=simulation_app, img_idx=img_idx, box_spawn_cfg=box_spawn_cfg, ycb_spawn_cfg=ycb_spawn_cfg, config=config, paths_cfg=paths_cfg,spawned_box_prim_paths=spawned_box_prim_paths, spawned_object_prim_paths=spawned_object_prim_paths)








                   
                        
                
            


            
           



         

            
            for i in range(2):
                simulation_app.update() 
            right_dir = os.path.join(image_output_directory, "right")
            clean_right_output(right_dir)






        

            
            print(f"--- Generazione immagine {img_idx}/{num_images_to_gen} completata ---")
           
        print(f"\nGenerazione di tutte le {num_images_to_gen} immagini completata.")

       

       # if sim_setup_cfg["headless"] == False:
            #while True:
             #   simulation_app.update()
   
    except FileNotFoundError as fnf_e: 
        print(str(fnf_e))
        traceback.print_exc()
    except Exception as e:
        print(f"ERRORE CRITICO in main_simulation: {e}")
        traceback.print_exc()
    finally:
        if simulation_app:
            print("Chiusura SimulationApp...")
            #simulation_app.close()
            print("SimulationApp chiusa.")
        else:
            print("SimulationApp non inizializzata o fallita prima dell'inizializzazione; nessuna chiusura esplicita necessaria.")
   


import queue

# Crea l'applicazione Flask
app = Flask(__name__)

# Crea la coda per la comunicazione
task_queue = queue.Queue()

# Flag per tracciare lo stato della simulazione
simulation_in_progress = False

# --- DA MODIFICARE: Percorso della cartella principale dei file di output ---
SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIRECTORY = os.path.join(SCRIPT_DIRECTORY, 'output')

# ==============================================================================
# ENDPOINT API
# ==============================================================================

@app.route("/")
def index():
    """Endpoint principale che mostra lo stato e gli endpoint disponibili."""
    return jsonify({
        "status": "server_running",
        "message": "Benvenuto nel server di simulazione Isaac Sim.",
        "endpoints": {
            "generate_scene": "POST /generate_scene",
            "regenerate_data": "POST /regenerate_data",
            "list_files": "GET /list_files",
            "get_document": "GET /get_document/<path:filename>"
        }
    })

# --- ENDPOINT MODIFICATO ---
@app.route("/generate_scene", methods=['POST'])
def start_simulation_endpoint():
    """
    Endpoint per avviare una NUOVA simulazione.
    Accetta sia JSON (senza config) sia multipart/form-data (con config.yaml).
    """
    global simulation_in_progress
    if simulation_in_progress:
        return jsonify({"status": "error", "message": "Simulazione già in corso."}), 409

    task = {'type': 'start_simulation'}
    
    try:
        # Controlla se è stata inviata una richiesta multipart (con file)
        if 'config_file' in request.files:
            print("Richiesta ricevuta a /generate_scene (con config.yaml).")
            # Legge le opzioni dal form
            options_str = request.form.get('options')
            task['options'] = json.loads(options_str) if options_str else []
            
            # Legge e salva il file di configurazione
            config_file = request.files['config_file']
            save_path = os.path.join(OUTPUT_DIRECTORY, 'active_config.yaml')
            os.makedirs(OUTPUT_DIRECTORY, exist_ok=True)
            config_file.save(save_path)
            
            task['config_path'] = save_path
            print(f"File di configurazione salvato in: {save_path}")

        # Altrimenti, si aspetta una richiesta JSON standard
        else:
            print("Richiesta ricevuta a /generate_scene (senza config.yaml).")
            data = request.get_json()
            task['options'] = data.get('options', [])
            task['config_path'] = None

        task_queue.put(task)
        return jsonify({"status": "success", "message": "Comando di avvio simulazione inviato."})

    except Exception as e:
        print(f"Errore durante l'elaborazione della richiesta /generate_scene: {e}")
        return jsonify({"status": "error", "message": f"Errore interno del server: {e}"}), 500


# --- ENDPOINT MODIFICATO ---
@app.route("/regenerate_data", methods=['POST'])
def regenerate_data_endpoint():
    """
    Endpoint per avviare la RIGENERAZIONE dei dati.
    Accetta solo richieste JSON con le opzioni.
    """
    global simulation_in_progress
    if simulation_in_progress:
        return jsonify({"status": "error", "message": "Simulazione già in corso."}), 409

    try:
        print("Richiesta ricevuta a /regenerate_data.")
        data = request.get_json()
        task = {
            'type': 'regenerate_data',
            'options': data.get('options', [])
        }
        task_queue.put(task)
        return jsonify({"status": "success", "message": "Comando di rigenerazione dati inviato."})
    except Exception as e:
        print(f"Errore durante l'elaborazione della richiesta /regenerate_data: {e}")
        return jsonify({"status": "error", "message": f"Errore interno del server: {e}"}), 500


@app.route("/list_files", methods=['GET'])
def list_files():
    """Scansiona la directory di output e restituisce un elenco JSON di file."""
    if not os.path.isdir(OUTPUT_DIRECTORY):
        return jsonify({"status": "error", "message": "Directory dei risultati non trovata."}), 404

    file_list = []
    for root, _, files in os.walk(OUTPUT_DIRECTORY):
        for file in files:
            relative_path = os.path.relpath(os.path.join(root, file), OUTPUT_DIRECTORY)
            file_list.append(relative_path.replace(os.path.sep, '/'))
    
    return jsonify({"status": "success", "files": file_list})


@app.route("/get_document/<path:filename>", methods=['GET'])
def get_document(filename):
    """Invia un singolo file richiesto dalla directory di output."""
    try:
        return send_from_directory(OUTPUT_DIRECTORY, filename, as_attachment=True)
    except FileNotFoundError:
        return jsonify({"error": "File non trovato"}), 404


# ==============================================================================
# BLOCCO MAIN MODIFICATO
# ==============================================================================
if __name__ == "__main__":
    server_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False), daemon=True)
    server_thread.start()
    print("\n--- SERVER FLASK AVVIATO IN BACKGROUND su http://127.0.0.1:5000 ---")
    print("--- AVVIO LOOP DI SIMULAZIONE PRINCIPALE. In attesa di comandi... ---")
    
    # --- ASSUMENDO CHE simulation_app SIA GIÀ DEFINITA QUI ---
    # simulation_app = SimulationApp(...) 
    
    try:
        while True:
            # Mantieni aggiornata l'app di simulazione
            # simulation_app.update() 

            try:
                task = task_queue.get_nowait()
                
                # --- BLOCCO DI GESTIONE TASK MODIFICATO ---
                simulation_in_progress = True  # Blocca nuove richieste

                task_type = task.get('type')
                options = task.get('options', []) # Opzioni selezionate (es. ['rgb', 'depth'])
                
                print(f"\n>>> Comando '{task_type}' ricevuto con opzioni: {options}")

                if task_type == "start_simulation":
                    config_path = task.get('config_path') # Può essere None
                    if config_path:
                        print(f"Utilizzando il file di configurazione: {config_path}")
                    
                    # Elimina tutto in OUTPUT_DIRECTORY tranne i file .yaml/.yml
                    for entry in os.listdir(OUTPUT_DIRECTORY):
                        path = os.path.join(OUTPUT_DIRECTORY, entry)
                        if os.path.isfile(path):
                            if not (entry.endswith('.yaml') or entry.endswith('.yml')):
                                os.remove(path)
                        elif os.path.isdir(path):
                            shutil.rmtree(path)


                    
                    # Esempio: main_simulation(options, config_path)
                    main_simulation(config_path,options) 
                    print(">>> Esecuzione di main_simulation() completata.")

                elif task_type == "regenerate_data":
                    
                    # Esempio: usare `options` per decidere quali annotatori attivare
                    stage_utils.load_stage_in_new_stage("stage_freeze_temp/saved_stage.usd")

                    
                    world = World()
                    stage = world.stage
                    
                    timeline = omni.timeline.get_timeline_interface()
                    #timeline.play()

                    for i in range(200):
                        simulation_app.update()
                    image_output_directory = os.path.join(current_script_dir, config['paths']['output_replicator_dir_base'], f"img{config['simulation_setup']['num_images_to_generate']}")
                    
                    


                    # Qui dovresti usare `options` per configurare il replicatore
                    # Ad esempio, attivando/disattivando annotatori prima di lanciare la generazione
                    replicator_utils.run_replicator_data_generation(simulation_app, timeline, rep, carb, config['replicator'], config['paths']['camera_prim_usd'],image_output_directory)
                    for i in range(50):
                        simulation_app.update()

                    rep.orchestrator.set_capture_on_play(False)
                    carb_s = carb.settings.get_settings()
                    carb_s.set_string("/renderer/active", "rtx")
                    carb_s.set_string("/rtx/rendermode", "rtx")
                    right_dir = os.path.join(image_output_directory, "right")
                    clean_right_output(right_dir)
                    
                    run_grip = "grip" in options
                    run_pinza = "pinza" in options
                    grip_pinza(USA_GRIP=run_grip, USA_PINZA=run_pinza, stage=stage, simulation_app=simulation_app, img_idx=config["simulation_setup"]["num_images_to_generate"], box_spawn_cfg=config['box_spawner'] , ycb_spawn_cfg=config.get('object_creator_ycb', {}) , config=config, paths_cfg=config['paths'], spawned_box_prim_paths=spawned_box_prim_paths, spawned_object_prim_paths=spawned_object_prim_paths)

                    print(">>> Esecuzione di rigenerazione dati completata.")

                simulation_in_progress = False  # Sblocca per nuove richieste
                print(">>> In attesa di nuovi comandi.")

            except queue.Empty:
                pass
            
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n--- Rilevato KeyboardInterrupt (CTRL+C). Chiusura in corso... ---")
    finally:
        # --- Il tuo codice di cleanup qui ---
        # if simulation_app:
        #     simulation_app.close()
        print("Programma terminato.")