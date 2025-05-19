# src/simple_box_spawner.py

import random
import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics, Sdf, UsdShade # Aggiunto UsdShade

# Importazioni da Isaac Sim Core
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.semantics import add_update_semantics


def spawn_basic_boxes(
    stage,
    num_to_spawn_range: int,
    parent_path: str,
    base_position: np.ndarray,
    base_z_offset: float,
    z_jitter: float,
    xy_jitter_range: tuple = (-0.2, 0.2),
    box_asset_paths: list = None, 
    box_scale_min: float = 0.1,   
    box_scale_max: float = 0.2,   
    box_mass: float = 1.0,
    default_color: Gf.Vec3f = Gf.Vec3f(0.5, 0.5, 0.5),
    semantic_label: str = "box",
    available_pbr_material_paths: list = None # NUOVO: Lista di percorsi ai materiali PBR
):
    """
    Genera N scatole (da asset USD o cubi procedurali) in posizioni casuali
    applicando fisica e, se disponibili, materiali PBR casuali ai cubi procedurali.
    """
    num_boxes = random.randint(num_to_spawn_range[0],num_to_spawn_range[1])
    print(f"simple_box_spawner.py: Inizio generazione di {num_boxes} scatole.")

    if not get_prim_at_path(parent_path):
        UsdGeom.Xform.Define(stage, parent_path)
        print(f"simple_box_spawner.py: Creato Xform genitore a '{parent_path}'.")

    created_box_prims = []

    for i in range(num_boxes):
        box_prim_name = f"BasicBox_{i}"
        current_box_prim_path = f"{parent_path}/{box_prim_name}"

        pos_offset_x = random.uniform(xy_jitter_range[0], xy_jitter_range[1])
        pos_offset_y = random.uniform(xy_jitter_range[0], xy_jitter_range[1])
        pos_offset_z = base_z_offset + random.uniform(0, z_jitter)
        
        position_np = base_position + np.array([pos_offset_x, pos_offset_y, pos_offset_z])
        position_gf = Gf.Vec3d(float(position_np[0]), float(position_np[1]), float(position_np[2]))

        random_euler_rad = np.array([random.uniform(0, 2 * np.pi) for _ in range(3)])
        orientation_np_wxyz = euler_angles_to_quat(random_euler_rad, degrees=False) 
        
        prim = None
        actual_asset_path_chosen = None
        if (random.random()<0.75):
            is_procedural_cube =True
        else:
            is_procedural_cube=False
       

       
        if not is_procedural_cube: # Se si usano asset
            

            
            actual_asset_path_chosen = random.choice(box_asset_paths)
            # Definisci l'intervallo per la scala di ciascun asse
            # Puoi usare lo stesso intervallo per tutti gli assi, o intervalli diversi se necessario
            
            scale_factor = 0.005

            # Genera un fattore di scala casuale INDIPENDENTE per ciascun asse (X, Y, Z)
            scale_x = random.uniform(box_scale_min, box_scale_max) * scale_factor
            scale_y = random.uniform(box_scale_min, box_scale_max) * scale_factor
            scale_z = random.uniform(box_scale_min, box_scale_max) * scale_factor
            
            print(f"simple_box_spawner.py: Tentativo di usare asset specifico '{actual_asset_path_chosen}' per '{current_box_prim_path}'.")
            print(f"simple_box_spawner.py: Scala casuale applicata [X, Y, Z]: [{scale_x:.4f}, {scale_y:.4f}, {scale_z:.4f}].") # Stampa per controllo
            
            # Crea l'array numpy con le scale casuali indipendenti
            scale_np = np.array([scale_x, scale_y, scale_z])
           
            prim = create_prim(
                
                prim_path=current_box_prim_path,
                usd_path=actual_asset_path_chosen,
                position=position_gf,
                scale=scale_np,
                orientation=orientation_np_wxyz,
                
            )




        else: # Se si creano cubi procedurali
            scale_factor = 0.1

            dim_x = random.uniform(box_scale_min, box_scale_max) * scale_factor
            dim_y = random.uniform(box_scale_min, box_scale_max) * scale_factor
            dim_z = random.uniform(box_scale_min, box_scale_max) * scale_factor
            scale_np = np.array([dim_x , dim_y, dim_z ]) 

            prim = create_prim(
                prim_path=current_box_prim_path,
                prim_type="Cube",
                position=position_gf,
                orientation=orientation_np_wxyz, 
                scale=scale_np
            )
            
            # Applica materiale PBR o colore di fallback ai cubi procedurali
            if prim:
                material_applied_successfully = False
                if available_pbr_material_paths:
                    selected_material_path = random.choice(available_pbr_material_paths)
                    material_to_bind = UsdShade.Material.Get(stage, selected_material_path)
                    if material_to_bind and material_to_bind.GetPrim().IsValid():
                        try:
                            UsdShade.MaterialBindingAPI(prim).Bind(material_to_bind)
                            print(f"simple_box_spawner.py: Materiale PBR '{selected_material_path}' applicato a '{current_box_prim_path}'.")
                            material_applied_successfully = True
                        except Exception as e_bind:
                            print(f"simple_box_spawner.py: ERRORE durante il binding del materiale '{selected_material_path}' a '{current_box_prim_path}': {e_bind}")
                    else:
                        print(f"simple_box_spawner.py: AVVISO - Materiale PBR '{selected_material_path}' non trovato o non valido.")
                
                if not material_applied_successfully:
                    gprim_api = UsdGeom.Gprim(prim) 
                    color_primvar = gprim_api.CreateDisplayColorPrimvar() 
                    if color_primvar:
                        color_primvar.Set([default_color]) 
                        print(f"simple_box_spawner.py: Colore di fallback applicato a '{current_box_prim_path}'.")
                    else:
                        print(f"simple_box_spawner.py: AVVISO - Impossibile creare displayColor primvar per '{current_box_prim_path}'.")

        if not prim or not prim.IsValid():
            print(f"simple_box_spawner.py: ERRORE - Creazione del prim '{current_box_prim_path}' fallita (asset: {actual_asset_path_chosen}).")
            continue 

        print(f"simple_box_spawner.py: Prim '{current_box_prim_path}' creato.")

        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(box_mass)
        
        if semantic_label:
            add_update_semantics(prim, semantic_label)
            #print(f"simple_box_spawner.py: Etichetta semantica '{semantic_label}' applicata a '{current_box_prim_path}'.") # Già loggato da add_update_semantics

        print(f"simple_box_spawner.py: Fisica applicata a '{current_box_prim_path}'.")
        created_box_prims.append(prim)

    print(f"simple_box_spawner.py: Generazione di {len(created_box_prims)} scatole completata.")
    return created_box_prims
