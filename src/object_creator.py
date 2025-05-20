

# src/object_generator.py

import random
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Sdf, UsdShade
import os

# Importazioni da Isaac Sim Core
from isaacsim.core.utils.prims import create_prim, get_prim_at_path
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.semantics import add_update_semantics
from isaacsim.core.api.materials import PreviewSurface


def _get_existing_materials(stage: Usd.Stage, materials_path: str) -> list[UsdShade.Material]:
    # ... (codice invariato) ...
    found_materials = []
    materials_root_prim = stage.GetPrimAtPath(materials_path)
    if not materials_root_prim:
        print(f"object_generator.py: AVVISO - Percorso materiali '{materials_path}' non trovato.")
        return []
    for prim_spec in materials_root_prim.GetChildren():
        prim = stage.GetPrimAtPath(prim_spec.GetPath())
        if prim and prim.IsA(UsdShade.Material):
            material = UsdShade.Material(prim)
            if material:
                 found_materials.append(material)
    if not found_materials:
         print(f"object_generator.py: AVVISO - Nessun materiale UsdShade trovato sotto '{materials_path}'.")
    return found_materials

def _apply_random_existing_material_to_meshes(stage: Usd.Stage, geometry_container_prim: Usd.Prim, existing_materials: list[UsdShade.Material]):
    # ... (codice invariato) ...
    if not existing_materials: return False
    if not geometry_container_prim or not geometry_container_prim.IsValid():
        print(f"object_generator.py: AVVISO - Prim contenitore geometria non valido '{geometry_container_prim.GetPath()}'.")
        return False
    try:
        selected_material = random.choice(existing_materials)
        material_path = selected_material.GetPath()
        meshes_found_count = 0
        for descendant_prim_spec in Usd.PrimRange(geometry_container_prim):
            descendant_prim = stage.GetPrimAtPath(descendant_prim_spec.GetPath())
            if descendant_prim and descendant_prim.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI(descendant_prim).Bind(selected_material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
                print(f"object_generator.py: Materiale '{material_path}' applicato alla mesh '{descendant_prim.GetPath()}'.")
                meshes_found_count += 1
        if meshes_found_count > 0: return True
        else:
            print(f"object_generator.py: AVVISO - Nessuna mesh trovata sotto '{geometry_container_prim.GetPath()}' per applicare materiale.")
            return False
    except Exception as e:
        print(f"object_generator.py: ERRORE applicazione materiale alle mesh sotto '{geometry_container_prim.GetPath()}': {e}")
        return False

def spawn_objects(
    stage: Usd.Stage,
    num_to_spawn_range: list,
    usd_asset_paths: list[str],
    parent_path: str,
    base_position: np.ndarray,
    base_z_offset: float,
    z_jitter: float,
    xy_jitter_range: tuple[float, float] = (-0.2, 0.2),
    asset_folder_path: str =  r"C:\Users\cm03696\Desktop\depal project\pre_build_asset",
    folder_asset_pre_scale_factor: float = 0.005, # NUOVO: Fattore di pre-scalatura per asset da cartella
    asset_scale_min: float = 0.8, # Scala relativa applicata *dopo* la pre-scalatura (se applicabile)
    asset_scale_max: float = 1.2,
    object_mass: float = 1.0,
    semantic_label: str = "object",
    existing_materials_path: str = "/World/Looks",
    asset_material_override_probability : float = 0.5
):
    """
    Genera N oggetti USD. Asset da 'asset_folder_path' sono pre-scalati.
    Tutti gli asset sono poi centrati internamente, ulteriormente scalati (relativamente),
    ruotati e posizionati.
    """
    num_actual_objects_to_spawn = random.randint(num_to_spawn_range[0], num_to_spawn_range[1])
    print(f"object_generator.py: Inizio generazione di {num_actual_objects_to_spawn} oggetti.")

    # --- Raccogli tutte le possibili sorgenti di asset ---
    explicit_asset_paths = list(usd_asset_paths) if usd_asset_paths else []
    folder_asset_paths = []

    if asset_folder_path:
        if os.path.isdir(asset_folder_path):
            print(f"object_generator.py: Scansione cartella asset '{asset_folder_path}' per file .usd...")
            found_in_folder = 0
            for item in os.listdir(asset_folder_path):
                if item.lower().endswith(".usd"):
                    full_item_path = os.path.join(asset_folder_path, item)
                    if os.path.isfile(full_item_path):
                        if full_item_path not in explicit_asset_paths and full_item_path not in folder_asset_paths:
                            folder_asset_paths.append(full_item_path)
                        found_in_folder += 1
            if found_in_folder > 0:
                print(f"object_generator.py: Trovati e aggiunti {found_in_folder} asset USD da '{asset_folder_path}'.")
        else:
            print(f"object_generator.py: AVVISO - Percorso asset_folder_path '{asset_folder_path}' non valido.")
    
    all_available_asset_sources = explicit_asset_paths + folder_asset_paths
    if not all_available_asset_sources:
        print("object_generator.py: ERRORE - Nessuna sorgente di asset USD disponibile. Nessun oggetto generato.")
        return []
    # --- Fine raccolta asset ---

    parent_prim_check = stage.GetPrimAtPath(parent_path)
    if not parent_prim_check:
        UsdGeom.Xform.Define(stage, parent_path)
        print(f"object_generator.py: Creato Xform genitore a '{parent_path}'.")

    available_materials = _get_existing_materials(stage, existing_materials_path)
    if not available_materials:
         print(f"object_generator.py: AVVISO - Nessun materiale trovato. Non verranno applicati materiali casuali.")

    created_object_root_prims = []

    for i in range(num_actual_objects_to_spawn):
        object_instance_name_suffix = f"{random.randint(10000, 99999)}_{i}"
        object_root_prim_path = f"{parent_path}/SpawnedObject_{object_instance_name_suffix}"

        pos_offset_x = random.uniform(xy_jitter_range[0], xy_jitter_range[1])
        pos_offset_y = random.uniform(xy_jitter_range[0], xy_jitter_range[1])
        pos_offset_z = base_z_offset + random.uniform(0, z_jitter)
        
        if not isinstance(base_position, np.ndarray): base_position = np.array(base_position)
        base_position_3d = np.append(base_position, 0.0) if base_position.shape == (2,) else base_position
        if base_position_3d.shape != (3,):
             print(f"object_generator.py: ERRORE forma 'base_position'. Uso [0,0,0].")
             base_position_3d = np.array([0.0, 0.0, 0.0])
        
        spawn_position_np = base_position_3d + np.array([pos_offset_x, pos_offset_y, pos_offset_z])
        spawn_position_gf = Gf.Vec3d(*spawn_position_np.astype(float))

        random_euler_rad = np.array([random.uniform(0, 2 * np.pi) for _ in range(3)])
        spawn_orientation_np_wxyz = euler_angles_to_quat(random_euler_rad, degrees=False)

        actual_asset_path_chosen = random.choice(all_available_asset_sources)
        is_folder_asset = actual_asset_path_chosen in folder_asset_paths
        
        print(f"object_generator.py: DEBUG [{object_root_prim_path}] Scelto asset: '{actual_asset_path_chosen}' (Da cartella: {is_folder_asset})")

        # --- 1. Calcola l'offset dal pivot al centro dell'asset (considerando la pre-scalatura per asset da cartella) ---
        offset_from_pivot_to_center_local_potentially_prescaled = Gf.Vec3d(0,0,0)
        scale_for_measurement = Gf.Vec3f(1.0, 1.0, 1.0) # Scala per la misurazione del bound
        if is_folder_asset:
            scale_for_measurement = Gf.Vec3f(folder_asset_pre_scale_factor, folder_asset_pre_scale_factor, folder_asset_pre_scale_factor)
            print(f"object_generator.py: DEBUG [{object_root_prim_path}] Asset da cartella. Pre-scalatura per misurazione: {scale_for_measurement}")

        try:
            temp_stage = Usd.Stage.CreateInMemory()
            temp_prim_path_measure = "/AssetToMeasure"
            temp_prim_measure_xform = UsdGeom.Xform.Define(temp_stage, temp_prim_path_measure)
            
            # Applica la scala di misurazione all'Xform wrapper, poi referenzia l'asset
            xformable_measure = UsdGeom.Xformable(temp_prim_measure_xform.GetPrim())
            xformable_measure.ClearXformOpOrder()
            xformable_measure.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(scale_for_measurement)
            
            # Crea un prim figlio per la reference per evitare di sovrascrivere le xform ops dell'asset
            asset_ref_in_measure_prim = temp_stage.DefinePrim(f"{temp_prim_path_measure}/AssetRef", "Xform")
            asset_ref_in_measure_prim.GetReferences().AddReference(assetPath=actual_asset_path_chosen)
            
            # Calcola il bound sull'Xform wrapper che ha la scala e la reference
            imageable_temp = UsdGeom.Imageable(temp_prim_measure_xform.GetPrim())
            world_bbox_measured = imageable_temp.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)

            if not world_bbox_measured.GetRange().IsEmpty():
                bbox_range_measured = world_bbox_measured.GetRange()
                # Questo offset è ora relativo all'asset *potenzialmente pre-scalato*
                offset_from_pivot_to_center_local_potentially_prescaled = (bbox_range_measured.GetMin() + bbox_range_measured.GetMax()) / 2.0
                print(f"object_generator.py: DEBUG [{object_root_prim_path}] Offset (local, post-misura-scala): {offset_from_pivot_to_center_local_potentially_prescaled}")
            else:
                print(f"object_generator.py: AVVISO [{object_root_prim_path}] BBox (post-misura-scala) vuota. Offset {offset_from_pivot_to_center_local_potentially_prescaled}")
        except Exception as e_measure:
            print(f"object_generator.py: ERRORE [{object_root_prim_path}] Misurazione asset '{actual_asset_path_chosen}': {e_measure}")

        # --- 2. Calcola la scala finale dell'oggetto ---
        relative_scale_factor_x = random.uniform(asset_scale_min, asset_scale_max)
        relative_scale_factor_y = random.uniform(asset_scale_min, asset_scale_max)
        relative_scale_factor_z = random.uniform(asset_scale_min, asset_scale_max)

        if is_folder_asset:
            # Applica la scala relativa alla pre-scalatura
            final_scale_x = folder_asset_pre_scale_factor * relative_scale_factor_x
            final_scale_y = folder_asset_pre_scale_factor * relative_scale_factor_y
            final_scale_z = folder_asset_pre_scale_factor * relative_scale_factor_z
        else:
            # Usa direttamente la scala relativa per asset da lista esplicita
            final_scale_x = relative_scale_factor_x
            final_scale_y = relative_scale_factor_y
            final_scale_z = relative_scale_factor_z
            
        final_object_scale_gf = Gf.Vec3f(final_scale_x, final_scale_y, final_scale_z)
        print(f"object_generator.py: DEBUG [{object_root_prim_path}] Scala finale oggetto: {final_object_scale_gf}")

        # --- 3. Crea l'Xform radice dell'oggetto ---
        object_root_prim_usd = UsdGeom.Xform.Define(stage, object_root_prim_path).GetPrim()
        xformable_root = UsdGeom.Xformable(object_root_prim_usd)
        xformable_root.ClearXformOpOrder()
        xformable_root.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(spawn_position_gf)
        orient_quat_gf = Gf.Quatf(spawn_orientation_np_wxyz[0], spawn_orientation_np_wxyz[1], spawn_orientation_np_wxyz[2], spawn_orientation_np_wxyz[3])
        xformable_root.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(orient_quat_gf)
        xformable_root.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(final_object_scale_gf)
        
        # --- 4. Crea l'Xform di centratura ---
        # La traslazione di centratura usa l'offset calcolato sull'asset *potenzialmente pre-scalato*.
        # Questa traslazione di centratura avviene *dopo* che l'Xform radice ha applicato la sua scala finale.
        # Quindi, l'offset di centratura deve essere nello spazio "non scalato" rispetto alla scala finale dell'Xform radice.
        # L'offset che abbiamo è già nello spazio corretto (locale all'asset pre-scalato, che è quello che referenzieremo).
        centering_xform_name = "InternalCenteringXform"
        centering_xform_path = f"{object_root_prim_path}/{centering_xform_name}"
        centering_xform_usd = UsdGeom.Xform.Define(stage, centering_xform_path).GetPrim()
        xformable_centering = UsdGeom.Xformable(centering_xform_usd)
        xformable_centering.ClearXformOpOrder()
        # Applica la traslazione *opposta* all'offset. Questo offset è già "pre-scalato" se necessario.
        centering_translation = -offset_from_pivot_to_center_local_potentially_prescaled
        xformable_centering.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(centering_translation))
        print(f"object_generator.py: DEBUG [{object_root_prim_path}] Applicata traslazione di centratura interna: {centering_translation}")

        # --- 5. Referenzia l'asset USD ---
        # L'asset referenziato *non* deve avere la pre-scalatura applicata di nuovo qui,
        # perché l'offset di centratura è stato calcolato sulla versione pre-scalata,
        # e la scala finale dell'oggetto è sull'Xform radice.
        asset_content_prim_name = "AssetContent"
        asset_content_prim_path = f"{centering_xform_path}/{asset_content_prim_name}"
        asset_content_container_prim = stage.DefinePrim(asset_content_prim_path, "Xform")
        asset_content_container_prim.GetReferences().AddReference(assetPath=actual_asset_path_chosen)
        # Se l'asset è da cartella, la sua "dimensione naturale" sarà grande.
        # L'Xform radice (`object_root_prim_usd`) lo scala alla dimensione finale corretta.
        # L'Xform di centratura (`centering_xform_usd`) sposta questa geometria (già scalata dall'Xform radice)
        # in modo che il suo centro (calcolato sulla base della versione pre-scalata) sia all'origine dell'Xform di centratura.

        print(f"object_generator.py: Oggetto '{object_root_prim_path}' creato con geometria centrata internamente.")
        
        # ... (resto del codice per materiali, fisica, semantica invariato) ...
        if available_materials and random.random() < asset_material_override_probability:
            print(f"object_generator.py: Tentativo di applicare materiale a '{asset_content_container_prim.GetPath()}'...")
            _apply_random_existing_material_to_meshes(stage, asset_content_container_prim, available_materials)
        
        try:
            UsdPhysics.CollisionAPI.Apply(object_root_prim_usd)
            UsdPhysics.RigidBodyAPI.Apply(object_root_prim_usd)
            mass_api = UsdPhysics.MassAPI.Apply(object_root_prim_usd)
            mass_api.CreateMassAttr().Set(object_mass)
            print(f"object_generator.py: Fisica applicata a '{object_root_prim_path}'.")
        except Exception as e_phys:
             print(f"object_generator.py: ERRORE applicazione fisica a '{object_root_prim_path}': {e_phys}")

        if semantic_label:
            try:
                add_update_semantics(object_root_prim_usd, semantic_label)
            except Exception as e_sem:
                 print(f"object_generator.py: ERRORE applicazione semantica a '{object_root_prim_path}': {e_sem}")

        created_object_root_prims.append(object_root_prim_usd)


    print(f"object_generator.py: Generazione di {len(created_object_root_prims)}/{num_actual_objects_to_spawn} oggetti completata.")
    return created_object_root_prims