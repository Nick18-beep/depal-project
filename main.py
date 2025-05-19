# main.py
import time
from omni.isaac.kit import SimulationApp
import traceback
import os
import random
import numpy as np
import yaml
# import asyncio # Non sembra essere utilizzato, potrebbe essere rimosso se non serve altrove.

#SETUP = {"headless": True,'renderer': 'PathTracing'}
SETUP = {"headless": True}


def  main_simulation():
    simulation_app = None
    try:
        simulation_app = SimulationApp(SETUP)
        print("SimulationApp inizializzata.")

        import omni.usd
        import omni.timeline
        
        from pxr import UsdGeom, Gf, Sdf, UsdPhysics
        from omni.isaac.core.utils import prims as prims_utils
        from omni.isaac.core.utils import rotations as rot_utils
        import omni.replicator.core as rep
        
        import carb
        

        import src.material_creator as material_creator
        import src.scene_creator as scene_creator
        import src.box_creator as box_creator
        import src.light_creator as light_creator
        import src.object_creator as object_creator
        import src.container_pallet_creator as container_pallet_creator

        # Esempio di utilizzo:
        yaml_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")  # Assumendo che il tuo file YAML si chiami "config.yaml"
        yaml_data = load_yaml_data(yaml_file)
        

        # Impostazioni generazione scatole
        BOX_PARENT_PATH = "/World/SpawnedBasicBoxes"
        CAMERA_PRIM_PATH = "/World/ReplicatorCamera"
        PALLET_POS = np.array([0.0, 0.0, 0.1])
        CAMERA_HEIGHT = 6
        CAMERA_POSITION = [0.0, 0.0, CAMERA_HEIGHT]
        NUM_BOXES_TO_SPAWN = 10
        BOX_MASS_KG = 1.2
        BASE_Z_OFFSET = 2
        Z_JITTER = 1
        BOX_ASSET_USD_PATHS = []
        BOX_SCALE_MIN = 0.15
        BOX_SCALE_MAX = 0.25
        DEFAULT_BOX_COLOR_RGB = Gf.Vec3f(0.8, 0.2, 0.2)
        BOX_SEMANTIC_LABEL = "box"
        NUM_IMAGES = 2

        try:
            from isaacsim.core.utils.stage import create_new_stage
        except ImportError:
            print("Avviso: 'create_new_stage' non trovato in 'isaacsim.core.utils.stage'. Uso 'omni.isaac.core.utils.stage'.")
            from omni.isaac.core.utils.stage import create_new_stage
        
        for img_number in range(1,NUM_IMAGES+1):

            if not create_new_stage():
                raise RuntimeError("ERRORE CRITICO: Impossibile creare una nuova scena.")
            simulation_app.update()
            print("Nuova scena creata.")

            stage = omni.usd.get_context().get_stage()
            if not stage:
                raise RuntimeError("Impossibile ottenere lo stage USD.")

            current_script_dir = os.path.dirname(os.path.abspath(__file__))
            texture_folder_path = os.path.abspath(os.path.join(current_script_dir, "texture"))
            print(f"Cartella texture: '{texture_folder_path}'")

            base_materials_usd_path = "/World/Looks"
            floor_prim_usd_path = "/World/PavimentoConMaterialeEsistente"

            print(f"Creazione materiali da '{texture_folder_path}'...")
            lista_componenti_pbr_creati = material_creator.crea_materiali_da_cartella_texture(
                stage=stage,
                simulation_app_instance=simulation_app,
                texture_dir_path=texture_folder_path,
                base_material_usd_path=base_materials_usd_path
            )
            
            pbr_components_da_applicare_al_pavimento = None
            if not lista_componenti_pbr_creati:
                print(f"AVVISO: Nessun materiale creato da '{texture_folder_path}'.")
            else:
                print(f"Creati {len(lista_componenti_pbr_creati)} materiali PBR.")
                pbr_components_da_applicare_al_pavimento = lista_componenti_pbr_creati[random.randint(0, len(lista_componenti_pbr_creati) - 1)]
                material_prim_applicato, _, _ = pbr_components_da_applicare_al_pavimento
                if material_prim_applicato and material_prim_applicato.IsValid():
                    print(f"Materiale '{material_prim_applicato.GetPath().pathString}' selezionato per il pavimento.")
                else:
                    print("AVVISO: Il materiale selezionato per il pavimento non è valido.")
                    pbr_components_da_applicare_al_pavimento = None
            
            print(f"Aggiunta pavimento a '{floor_prim_usd_path}'...")
            pavimento_object = scene_creator.aggiungi_pavimento_con_materiale_esistente(
                stage=stage,
                prim_path=floor_prim_usd_path,
                pbr_material_components_da_applicare=pbr_components_da_applicare_al_pavimento 
            )
            
            if pavimento_object:
                print(f"Pavimento '{pavimento_object.prim_path}' aggiunto.")
                if pbr_components_da_applicare_al_pavimento:
                    material_prim_applicato, _, _ = pbr_components_da_applicare_al_pavimento
                    if material_prim_applicato and material_prim_applicato.IsValid():
                        print(f"Materiale PBR '{material_prim_applicato.GetPath().pathString}' assegnato al pavimento.")
                    else:
                        print("Assegnazione materiale al pavimento saltata (materiale non valido).")
                else:
                    print("Nessun materiale PBR valido da assegnare al pavimento.")
            else:
                print("ERRORE: Aggiunta del pavimento fallita.")

            timeline = omni.timeline.get_timeline_interface()
            timeline.play()

            available_pbr_material_paths_for_boxes = []
            if lista_componenti_pbr_creati:
                for mat_prim, _, _ in lista_componenti_pbr_creati:
                    if mat_prim and mat_prim.IsValid():
                        available_pbr_material_paths_for_boxes.append(mat_prim.GetPath().pathString)
            if available_pbr_material_paths_for_boxes:
                print(f"{len(available_pbr_material_paths_for_boxes)} materiali PBR disponibili per le scatole.")


            # --- Definizione delle opzioni per le luci casuali ---
            light_configurations = [
        
        
        
        {
            "type": "DomeLight", 
            # 'aim_at_target' non si applica alle DomeLight, useranno euler_range
            "orientation_euler_range": {"x": (0.0,0.0), "y": (-180.0,180.0), "z": (0.0,0.0)}, 
            "intensity_range": (900.0, 1500.0), 
            "color_range": ((0.15, 0.15, 0.2), (0.5, 0.5, 0.7)), 
            "enable_color_temperature": random.choice([True, False]), 
            "temperature_k_range": (4000.0, 30000.0) 
        },
        {
            "type": "SphereLight", # Per SphereLight base, 'aim_at_target' ha meno impatto visivo,
                                # ma potrebbe essere utile se usi IES in futuro.
                                # Per ora, potremmo ometterlo o lasciarlo per coerenza.
            "position_range": {"x": (-3.0, 3.0), "y": (-3.0, 3.0), "z": (9.0, 15.0)},
            "orientation_euler_range": {"x": (0.0,360.0), "y": (0.0,360.0), "z": (0.0,360.0)}, 
            "intensity_range": (20000.0, 60000.0), 
            "color_range": ((0.5, 0.7, 0.8), (1.0, 1.0, 1.0)),
            "radius_range": (0.6, 2)
        },
        
        {
            "type": "DistantLight",
            # La 'position_range' per DistantLight serve solo per calcolare la direzione iniziale del puntamento.
            # La luce stessa non ha una posizione effettiva.
            "position_range": {"x": (-10.0, 10.0), "y": (-10.0, 10.0), "z": (5.0, 15.0)}, # Punto da cui "guarda"
            
            "orientation_euler_range": {"x": (-60.0,60.0), "y": (-60.0,60.0), "z": (-1.0,1.0)}, 
            "intensity_range": (900.0, 1500.0), 
            "color_range": ((0.9, 0.85, 0.75), (1.0, 1.0, 1.0)), 
            "angle_range": (0.5, 2.5) # L'angolo rimane casuale
        },
        
        
        
    ]

            # --- Chiamata alla funzione per creare le luci ---
            created_lights_summary = light_creator.spawn_variable_random_lights(
                stage=stage,
                base_path="/World/DefaultRandomLights", # Nuovo percorso per distinguerle
                num_lights_config=(1, 3), # Ad esempio, crea da 2 a 4 luci
                light_options=light_configurations,
                clear_existing=True
            )

            if created_lights_summary:
                print(f"MAIN: Riepilogo luci create ({len(created_lights_summary)}):")
                for light_info in created_lights_summary:
                    print(f"  - Path: {light_info['path']}, Tipo: {light_info['type']}")


            
            ASSET_PARENT_PATH = "/World"  # Percorso genitore comune
            BASE_ASSET_POSITION = PALLET_POS # Usa la stessa posizione base (es. np.array([0.0, 0.0, 0.0]))

            # Asset per i CONTAINER (precedentemente nella tua variabile PALLET_USD_ASSET_PATHS)
            CONTAINER_USD_ASSET_PATHS = [
                "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Storage/Containers/Container_A/Container_A04_71x56x38cm_PR_V_NVD_01.usd",
                "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Storage/Containers/Container_B/Container_B15_40x30x28cm_PR_V_NVD_01.usd",
                "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Storage/Containers/Container_H/Container_H03_60x40x23cm_PR_V_NVD_01.usd",
            ]

            # Asset per i PALLET (precedentemente nella tua variabile CONTAINER__USD_ASSET_PATHS)
            PALLET_USD_ASSET_PATHS = [
                "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Shipping/Pallets/Wood/Block_A/BlockPallet_A01_PR_NVD_01.usd",
                "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Shipping/Pallets/Metal/Aluminum_A/AluminumPallet_A01_PR_NVD_01.usd",
                "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Shipping/Pallets/Wood/Block_A/BlockPallet_A03_PR_NVD_01.usd",
                "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Shipping/Pallets/Wood/Block_B/BlockPallet_B03_PR_NVD_01.usd",
                "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Shipping/Wood_Crates/PalletCollar_A/WoodPalletCollar_A02_100x120x24cm_PR_NV_01.usd",
            ]

            # Scale: assumiamo che gli asset siano in cm, quindi 0.01 per convertirli in metri.
            # I container saranno leggermente più grandi nel loro fattore di scala.
            CM_TO_M_SCALE_FACTOR = 0.01
            PALLET_SCALE = np.array([1.0, 1.0, 1.0]) * CM_TO_M_SCALE_FACTOR
            CONTAINER_SCALE = np.array([2.0, 2.0, 2.0]) * CM_TO_M_SCALE_FACTOR # 20% più grandi nel fattore di scala

            # Masse (esempi, aggiusta secondo necessità)
            PALLET_MASS_KG = 20.0
            CONTAINER_MASS_KG = 8.0 # I container vuoti di plastica/metallo possono variare

            # Etichette semantiche
            PALLET_SEMANTIC_LABEL = "pallet"
            CONTAINER_SEMANTIC_LABEL = "container"

            # Probabilità di applicazione materiale (comune per entrambi, o definisci separatamente)
            # L'avevi a 0, quindi i materiali non verranno applicati a meno che non cambi questo valore.
            ASSET_MATERIAL_APPLICATION_PROBABILITY = 0.0

            # Assicurati che 'available_pbr_material_paths_for_assets' sia una LISTA di percorsi di materiali
            # Questa variabile dovrebbe essere popolata prima, quando crei i materiali da texture. Esempio:
            # available_pbr_material_paths_for_assets = []
            # if lista_componenti_pbr_creati:
            #     for mat_prim, _, _ in lista_componenti_pbr_creati:
            #         if mat_prim and mat_prim.IsValid():
            #             available_pbr_material_paths_for_assets.append(mat_prim.GetPath().pathString)

            # --- SCELTA CASUALE E CREAZIONE PALLET/CONTAINER ---
            print("Scelta casuale e creazione di pallet o container...")
            asset_type_choice = random.choice(["pallet", "container"])
            # Per testare:
            asset_type_choice = "pallet"
            #asset_type_choice = "container"

            current_usd_asset_paths = []
            current_scale = np.array([1.0, 1.0, 1.0])
            current_mass = 1.0
            current_semantic_label = "spawned_asset"
            prim_name_instance = "SpawnedAsset_01" # Nome di base
            
            if asset_type_choice == "pallet":
                print("Tipo scelto: PALLET")
                current_usd_asset_paths = PALLET_USD_ASSET_PATHS
                current_scale = PALLET_SCALE
                current_mass = PALLET_MASS_KG
                current_semantic_label = PALLET_SEMANTIC_LABEL
                prim_name_instance = "ImportedPallet_01"
            else: # "container"
                print("Tipo scelto: CONTAINER")
                current_usd_asset_paths = CONTAINER_USD_ASSET_PATHS
                current_scale = CONTAINER_SCALE
                current_mass = CONTAINER_MASS_KG
                current_semantic_label = CONTAINER_SEMANTIC_LABEL
                prim_name_instance = "ImportedContainer_01"

            spawned_asset_prim = None
            if not current_usd_asset_paths:
                print(f"ERRORE: La lista di asset per '{asset_type_choice}' è vuota. Impossibile spawnare.")
                # Imposta un'altezza di fallback se l'asset non può essere creato, così le scatole non fluttuano troppo
                superficie_superiore_z_asset = BASE_ASSET_POSITION[2] + 0.1 # Altezza di fallback
            else:
                random_asset_rotation_z = random.uniform(0.0, 360.0)
                
                # Assicurati che 'container_pallet_creator' sia il nome corretto del tuo modulo importato
                spawned_asset_prim = container_pallet_creator.spawn_single_pallet(
                    stage=stage,
                    usd_asset_paths=current_usd_asset_paths,
                    parent_path=ASSET_PARENT_PATH,
                    prim_name=prim_name_instance,
                    position=BASE_ASSET_POSITION,
                    orientation_euler_degrees=(0.0, 0.0, random_asset_rotation_z),
                    scale=current_scale,
                    mass=current_mass,
                    semantic_label=current_semantic_label,
                    available_material_paths="/World/Looks", # DEVE ESSERE UNA LISTA!
                    material_application_probability=ASSET_MATERIAL_APPLICATION_PROBABILITY
                )

            
            simulation_app.update() 
            


            '''
            #ITEM GENERATOR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            if stage:
            
                asset_list = [
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/002_master_chef_can.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/004_sugar_box.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/005_tomato_soup_can.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/007_tuna_fish_can.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/009_gelatin_box.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/010_potted_meat_can.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/011_banana.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/019_pitcher_base.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/021_bleach_cleanser.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/024_bowl.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/025_mug.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/037_scissors.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/040_large_marker.usd",
                    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Props/YCB/Axis_Aligned/051_large_clamp.usd",




                ]

                # ASSICURATI CHE QUESTO PERCORSO CONTENGA MATERIALI USD VALIDI (es. .mdl convertiti o UsdPreviewSurface)
                materials_folder_path = "/World/Looks"
                # Esempio: potresti creare alcuni materiali manualmente o con un altro script prima:
                # PreviewSurface(prim_path=f"{materials_folder_path}/Red", color=Gf.Vec3f(1,0,0))
                # PreviewSurface(prim_path=f"{materials_folder_path}/Blue", color=Gf.Vec3f(0,0,1))

                # Parametri per la generazione
                num_to_spawn = 10
                spawn_parent_path = "/World/GeneratedObjects"
                spawn_base_pos = np.array([0.0, 0.0])
                spawn_z_offset = 0.5
                spawn_z_jitter = 0.5
                spawn_xy_jitter = (-0.6, 0.6)
                spawn_scale_min = 0.8
                spawn_scale_max = 1.2
                spawn_mass = 0.2
                spawn_semantic = "generated_prop"
                spawn_material_prob = 0.6 # 60% di probabilità di applicare un materiale esistente

                # Chiama la funzione per generare gli oggetti
                generated_prims = object_creator.spawn_objects(
                    stage=stage,
                    num_objects=num_to_spawn,
                    usd_asset_paths=asset_list,
                    parent_path=spawn_parent_path,
                    base_position=spawn_base_pos,
                    base_z_offset=spawn_z_offset,
                    z_jitter=spawn_z_jitter,
                    xy_jitter_range=spawn_xy_jitter,
                    asset_scale_min=spawn_scale_min,
                    asset_scale_max=spawn_scale_max,
                    object_mass=spawn_mass,
                    semantic_label=spawn_semantic,
                    change_material_probability=spawn_material_prob, # Nome parametro aggiornato
                    existing_materials_path=materials_folder_path # Nome parametro aggiornato
                )

                print(f"\n--- Riepilogo ---")
                if generated_prims:
                    print(f"Creati {len(generated_prims)} prims:")
                    for prim in generated_prims:
                        print(f"- {prim.GetPath()}")
                else:
                    print("Nessun prim generato (controlla errori precedenti).")
            else:
                print("ERRORE: Impossibile ottenere lo stage USD corrente.")



            #OBJECT GENERATOR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            '''

            print(f"Avvio generazione di {NUM_BOXES_TO_SPAWN} scatole...")
            spawned_boxes = box_creator.spawn_basic_boxes( 
                stage=stage,
                num_boxes=NUM_BOXES_TO_SPAWN,
                parent_path=BOX_PARENT_PATH,
                base_position=PALLET_POS,
                base_z_offset=BASE_Z_OFFSET,
                z_jitter=Z_JITTER,
                xy_jitter_range=(-0.3, 0.3), 
                box_asset_paths=BOX_ASSET_USD_PATHS, 
                box_scale_min=BOX_SCALE_MIN,
                box_scale_max=BOX_SCALE_MAX,
                box_mass=BOX_MASS_KG,
                default_color=DEFAULT_BOX_COLOR_RGB,
                semantic_label=BOX_SEMANTIC_LABEL,
                available_pbr_material_paths=available_pbr_material_paths_for_boxes
            )
            if spawned_boxes:
                print(f"{len(spawned_boxes)} scatole create in '{BOX_PARENT_PATH}'.")
            else:
                print("Nessuna scatola creata.")
            


            simulation_app.update() 

            print("\nSetup completato. L'applicazione rimarrà in esecuzione.")
            print("Per avviare la fisica, premi PLAY nell'interfaccia o decommenta simulation_app.play().")
            # simulation_app.play() # Decommenta per avviare la fisica automaticamente

            for _ in range(500): simulation_app.update() 

            # Camera top‑down
            cam = prims_utils.create_prim(CAMERA_PRIM_PATH, prim_type="Camera", position=CAMERA_POSITION, attributes={"focalLength": 35.0})
            if cam:
                UsdGeom.XformCommonAPI(cam).SetRotate(Gf.Vec3f(-90.0, 0.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)



            
            OUTPUT_REPLICATOR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", f"img{img_number}")
            os.makedirs(OUTPUT_REPLICATOR_DIR, exist_ok=True)
            REPLICATOR_RESOLUTION = (1920, 1080)
            
            if timeline.is_playing():
                timeline.pause()
                print("Replicator: Timeline della simulazione messa in pausa.")
            
            # Un aggiornamento per assicurare che lo stato di pausa sia effettivo
            simulation_app.update() 
            print("Replicator: Configurazione e tentativo di generazione dati singola con run/stop.")

            
            with rep.new_layer():
                
                setting= carb.settings.get_settings()
                setting.set_string("/renderer/active", "rtx")
                setting.set_string("/rtx/rendermode", "PathTracing")
                #setting.set_int("/rtx/pathtracing/spp", 128)
                # Puoi anche configurare parametri specifici del path tracer se necessario, ad esempio:
                # carb.settings.get_settings().set("/rtx/pathtracing/spp", 64) # Imposta i campioni per pixel
                simulation_app.update()
                rp = rep.create.render_product(CAMERA_PRIM_PATH, REPLICATOR_RESOLUTION)
                
                # Annotatori per segmentazione semantica e normali
                rep.annotators.get("instance_segmentation").attach(rp)
                rep.annotators.get("normals").attach(rp)
                rep.annotators.get("bounding_box_2d_tight").attach(rp)
                rep.annotators.get("distance_to_camera").attach(rp)
                rep.annotators.get("pointcloud").attach(rp)
                rep.annotators.get("camera_params").attach(rp)
                rep.annotators.get("rgb").attach(rp)
                rep.annotators.get("distance_to_image_plane").attach(rp)
                

                writer = rep.WriterRegistry.get("BasicWriter")
                writer.initialize(output_dir=OUTPUT_REPLICATOR_DIR,rgb=True,instance_segmentation=True, normals=True,bounding_box_2d_tight=True, pointcloud=True, distance_to_camera=True, camera_params=True,distance_to_image_plane=True )
                writer.attach([rp])  # output di RGB, semantiche e normali
                rep.orchestrator.step()
            simulation_app.update()    


            import cv2
            # --- Inizio Visualizzazione Profondità (Modificata) ---
            print("\nVisualizzazione mappa di profondità...")
            depth_file_name = "distance_to_image_plane_0000.npy"
            depth_file_path = os.path.join(OUTPUT_REPLICATOR_DIR, depth_file_name)

            if os.path.exists(depth_file_path):
                print(f"Caricamento mappa di profondità da: {depth_file_path}")
                d = np.load(depth_file_path) # i valori sono in metri
                
                # Gestisci NaN e infiniti. Per la visualizzazione, decidiamo come trattare 'inf'.
                # Se vogliamo che i punti all'infinito (sfondo) siano i più lontani (bianchi),
                # li sostituiamo con un valore di profondità massimo ragionevole o il massimo finito.
                max_finite_depth = np.max(d[np.isfinite(d)]) if np.any(np.isfinite(d)) else 100.0 # Default a 100m se tutto è inf
                # Se non ci sono punti finiti e vuoi comunque visualizzare qualcosa, puoi impostare un valore.
                # Se la scena è vuota, d potrebbe essere tutto inf.
                
                d_processed = np.nan_to_num(d, nan=max_finite_depth, posinf=max_finite_depth, neginf=0.0)
                # Sostituiamo nan e posinf con max_finite_depth, neginf (improbabile) con 0.

                d_min_val = d_processed.min()
                d_max_val = d_processed.max()
                print(f"Valori profondità processati (metri): min={d_min_val:.4f}, max={d_max_val:.4f}")

                if d_max_val > d_min_val:
                    # Normalizza l'intervallo [d_min_val, d_max_val] a [0, 1]
                    d_normalized = (d_processed - d_min_val) / (d_max_val - d_min_val)
                elif d_max_val > 0 : # Se min e max sono uguali (scena piatta a una certa distanza)
                    d_normalized = np.full_like(d_processed, 0.5) # Visualizza come grigio medio
                else: # Tutti i valori sono 0 o meno
                    d_normalized = np.zeros_like(d_processed)
                
                # Scala a uint16 per la visualizzazione (0-65535)
                # Per la profondità, spesso si inverte (vicino=bianco, lontano=nero) per una migliore percezione
                # ma manteniamo vicino=scuro, lontano=chiaro per ora.
                d_vis_uint16 = (d_normalized * 65535).astype(np.uint16)
                
                output_visualization_path = os.path.join(current_script_dir, "depth_visualization.png")
                try:
                    cv2.imwrite(output_visualization_path, d_vis_uint16)
                    print(f"Visualizzazione della profondità (uint16) salvata in: {output_visualization_path}")
                    print(f"Per favore, apri questo file con un visualizzatore di immagini per vedere la mappa di profondità.")
                except Exception as e:
                    print(f"Errore durante il salvataggio dell'immagine di profondità: {e}")

                # La parte cv2.imshow() è stata rimossa per evitare l'errore GUI.
                # print("Visualizzazione con cv2.imshow() disabilitata a causa di potenziali problemi GUI in questo ambiente.")

            else:
                print(f"ERRORE: File della mappa di profondità non trovato: {depth_file_path}")
            # --- Fine Visualizzazione Profondità ---

            print("\nSetup e generazione dati (e visualizzazione profondità) completati.")
        
        

        #while simulation_app.is_running():
        #  simulation_app.update()

    except Exception as e:
        print(f"ERRORE CRITICO in main_simulation: {e}")
        traceback.print_exc()
    finally:
        if simulation_app:
            print("Chiusura SimulationApp...")
            simulation_app.close()
            print("SimulationApp chiusa.")
        else:
            print("SimulationApp non inizializzata, nessuna chiusura necessaria.")


def load_yaml_data(yaml_file_path):
    """
    Carica i dati da un file YAML.

    Args:
        yaml_file_path (str): Il percorso del file YAML.

    Returns:
        dict or None: Un dizionario contenente i dati caricati dal file YAML,
                     o None se il file non esiste o si verifica un errore durante la lettura.
    """
    try:
        with open(yaml_file_path, 'r') as f:
            data = yaml.safe_load(f)
        return data
    except FileNotFoundError:
        print(f"Errore: Il file YAML '{yaml_file_path}' non è stato trovato.")
        return None
    except yaml.YAMLError as e:
        print(f"Errore durante la lettura del file YAML '{yaml_file_path}': {e}")
        return None




if __name__ == "__main__": 
    main_simulation()