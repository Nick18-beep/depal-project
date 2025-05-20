

# src/simple_box_spawner.py

import random
import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics, Sdf, UsdShade, Usd
import os

# Importazioni da Isaac Sim Core
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path # get_prim_at_path non usato qui ma buona pratica
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.semantics import add_update_semantics


def spawn_basic_boxes(
    stage,
    num_to_spawn_range: tuple[int, int],
    parent_path: str,
    base_position: np.ndarray,
    base_z_offset: float,
    z_jitter: float,
    xy_jitter_range: tuple = (-0.2, 0.2),
    box_asset_paths: list = None,
    asset_folder_path: str = r"C:\Users\cm03696\Desktop\depal project\pre_build_asset",
    box_scale_min: float = 0.1,
    box_scale_max: float = 0.2,
    box_mass: float = 1.0,
    default_color: Gf.Vec3f = Gf.Vec3f(0.5, 0.5, 0.5),
    semantic_label: str = "box",
    available_pbr_material_paths: list = None,
    asset_material_override_probability: float = 1.0,
    procedural_vs_asset_probability: float = 0.5
):
    """
    Genera N scatole. Tutti gli asset USD vedono la loro geometria interna
    traslata in modo che il loro centro geometrico coincida con l'origine (0,0,0)
    del prim Xform che li contiene direttamente.
    Questo Xform contenitore viene poi scalato, ruotato e posizionato.
    """
    num_boxes = random.randint(num_to_spawn_range[0], num_to_spawn_range[1])
    print(f"simple_box_spawner.py: Inizio generazione di {num_boxes} scatole.")

    all_available_usd_asset_paths = []
    if box_asset_paths:
        all_available_usd_asset_paths.extend(box_asset_paths)

    if asset_folder_path and os.path.isdir(asset_folder_path):
        # ... (codice di scansione cartella invariato) ...
        print(f"simple_box_spawner.py: Scansione cartella asset '{asset_folder_path}' per file .usd...")
        found_in_folder = 0
        for item in os.listdir(asset_folder_path):
            if item.lower().endswith(".usd"):
                full_item_path = os.path.join(asset_folder_path, item)
                if os.path.isfile(full_item_path):
                    if full_item_path not in all_available_usd_asset_paths:
                        all_available_usd_asset_paths.append(full_item_path)
                    found_in_folder += 1
        if found_in_folder > 0:
            print(f"simple_box_spawner.py: Trovati e aggiunti {found_in_folder} asset USD da '{asset_folder_path}'.")
        else:
            print(f"simple_box_spawner.py: AVVISO - Nessun file .usd (.USD) trovato in '{asset_folder_path}'.")
    elif asset_folder_path:
        print(f"simple_box_spawner.py: AVVISO - Percorso asset_folder_path '{asset_folder_path}' non valido.")

    if not all_available_usd_asset_paths:
        print(f"simple_box_spawner.py: AVVISO - Nessun asset USD disponibile. Verranno creati solo cubi procedurali se scelti.")

    if not get_prim_at_path(parent_path): # get_prim_at_path è di Isaac Sim, non di PXR Usd
        # Usiamo stage.GetPrimAtPath per PXR puro se necessario, ma qui va bene
        # se la funzione get_prim_at_path è disponibile dall'ambiente Isaac Sim.
        # Se questo script è eseguito fuori da Isaac Sim, questa riga darebbe errore.
        # Assumendo che sia in Isaac Sim:
        parent_prim = stage.GetPrimAtPath(parent_path)
        if not parent_prim:
             UsdGeom.Xform.Define(stage, parent_path)
             print(f"simple_box_spawner.py: Creato Xform genitore a '{parent_path}'.")


    created_box_prims_roots = [] # Memorizzeremo gli Xform radice degli asset/cubi

    for i in range(num_boxes):
        box_prim_name = f"BasicBox_{i}"
        # Questo è il percorso del prim Xform radice per questo box/asset
        asset_root_prim_path = f"{parent_path}/{box_prim_name}"

        # La posizione di spawn desiderata per il *centro* dell'oggetto finale
        target_center_pos_offset_x = random.uniform(xy_jitter_range[0], xy_jitter_range[1])
        target_center_pos_offset_y = random.uniform(xy_jitter_range[0], xy_jitter_range[1])
        target_center_pos_offset_z = base_z_offset + random.uniform(0, z_jitter)
        spawn_position_np = base_position + np.array([target_center_pos_offset_x, target_center_pos_offset_y, target_center_pos_offset_z])
        spawn_position_gf = Gf.Vec3d(*spawn_position_np.astype(float))

        random_euler_rad = np.array([random.uniform(0, 2 * np.pi) for _ in range(3)])
        spawn_orientation_np_wxyz = euler_angles_to_quat(random_euler_rad, degrees=False)

        asset_root_prim = None # Il prim che verrà effettivamente aggiunto alla lista
        actual_asset_path_chosen = None
        is_procedural_cube = True

        if random.random() < procedural_vs_asset_probability:
            is_procedural_cube = True
        elif all_available_usd_asset_paths:
            is_procedural_cube = False
        else:
            print(f"simple_box_spawner.py: DEBUG [{box_prim_name}] Voleva asset, nessuno disponibile. Fallback a cubo.")
            is_procedural_cube = True

        prim_type_str = "cubo procedurale"

        if not is_procedural_cube:
            prim_type_str = "asset"
            actual_asset_path_chosen = random.choice(all_available_usd_asset_paths)
            print(f"simple_box_spawner.py: DEBUG [{box_prim_name}] Scelto asset: '{actual_asset_path_chosen}'")

            # --- 1. Calcola l'offset dal pivot al centro dell'asset (scala originale) ---
            offset_from_pivot_to_center_local_unscaled = Gf.Vec3d(0,0,0)
            try:
                temp_stage = Usd.Stage.CreateInMemory()
                temp_prim_path_measure = "/AssetToMeasure"
                temp_prim_measure = temp_stage.DefinePrim(temp_prim_path_measure, "Xform")
                temp_prim_measure.GetReferences().AddReference(assetPath=actual_asset_path_chosen)
                
                imageable_temp = UsdGeom.Imageable(temp_prim_measure)
                world_bbox_original = imageable_temp.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)

                if not world_bbox_original.GetRange().IsEmpty():
                    bbox_range_original = world_bbox_original.GetRange()
                    offset_from_pivot_to_center_local_unscaled = (bbox_range_original.GetMin() + bbox_range_original.GetMax()) / 2.0
                    print(f"simple_box_spawner.py: DEBUG [{box_prim_name}] Offset (pivot-to-center, local UNscaled): {offset_from_pivot_to_center_local_unscaled}")
                else:
                    print(f"simple_box_spawner.py: AVVISO [{box_prim_name}] BBox originale vuota. Offset {offset_from_pivot_to_center_local_unscaled}")
            except Exception as e_measure:
                print(f"simple_box_spawner.py: ERRORE [{box_prim_name}] Misurazione asset '{actual_asset_path_chosen}': {e_measure}")

            # --- 2. Calcola la scala finale dell'asset ---
            asset_scale_factor = 0.005
            scale_x = random.uniform(box_scale_min, box_scale_max) * asset_scale_factor
            scale_y = random.uniform(box_scale_min, box_scale_max) * asset_scale_factor
            scale_z = random.uniform(box_scale_min, box_scale_max) * asset_scale_factor
            final_asset_scale_np = np.array([scale_x, scale_y, scale_z])
            print(f"simple_box_spawner.py: DEBUG [{box_prim_name}] Scala finale asset: {final_asset_scale_np}")

            # --- 3. Crea l'Xform radice dell'asset con scala, rotazione e posizione finale ---
            # Poiché l'asset sarà centrato internamente, la posizione di questo Xform radice
            # è direttamente la posizione di spawn desiderata per il centro.
            asset_root_prim_usd = UsdGeom.Xform.Define(stage, asset_root_prim_path).GetPrim()
            xformable_root = UsdGeom.Xformable(asset_root_prim_usd)
            xformable_root.ClearXformOpOrder()
            xformable_root.AddTranslateOp().Set(spawn_position_gf)
            xformable_root.AddOrientOp().Set(Gf.Quatf(spawn_orientation_np_wxyz[0], spawn_orientation_np_wxyz[1], spawn_orientation_np_wxyz[2], spawn_orientation_np_wxyz[3]))
            xformable_root.AddScaleOp().Set(Gf.Vec3f(*final_asset_scale_np.astype(float)))
            
            asset_root_prim = asset_root_prim_usd # Per applicare fisica, semantica, etc.

            # --- 4. Crea l'Xform di centratura come figlio dell'Xform radice ---
            centering_xform_name = "CenteringXform"
            centering_xform_path = f"{asset_root_prim_path}/{centering_xform_name}"
            centering_xform_usd = UsdGeom.Xform.Define(stage, centering_xform_path).GetPrim()
            xformable_centering = UsdGeom.Xformable(centering_xform_usd)
            xformable_centering.ClearXformOpOrder()
            # Applica la traslazione *opposta* all'offset.
            # Questo offset è già nello spazio locale dell'asset, quindi non necessita di rotazione o scalatura qui.
            centering_translation = -offset_from_pivot_to_center_local_unscaled
            xformable_centering.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(centering_translation))
            print(f"simple_box_spawner.py: DEBUG [{box_prim_name}] Applicata traslazione di centratura interna: {centering_translation}")

            # --- 5. Referenzia l'asset USD come figlio dell'Xform di centratura ---
            # Il nome del prim che referenzia l'asset può essere qualsiasi cosa, es. "AssetGeometry"
            asset_geometry_prim_path = f"{centering_xform_path}/AssetGeometry"
            # Definiamo un semplice Prim, non necessariamente un Xform, per fare da contenitore alla reference
            # Usare Usd.Prim.Define è più generico se non si vuole un tipo specifico come Xform.
            # Tuttavia, creare un Xform e poi referenziare è anche comune.
            # Qui, UsdGeom.Xform.Define crea un Xform, il che è ok. Potrebbe anche essere un semplice Scope.
            # Per semplicità, usiamo DefinePrim che crea un typeless prim (over) se non specificato.
            # Oppure, più esplicitamente, creiamo un Xform per la reference.
            asset_ref_prim = stage.DefinePrim(asset_geometry_prim_path, "Xform") # Può anche essere un Scope
            asset_ref_prim.GetReferences().AddReference(assetPath=actual_asset_path_chosen)
            
        else: # Cubo procedurale
            cube_scale_factor = 0.1
            cube_dims_np = np.array([
                random.uniform(box_scale_min, box_scale_max) * cube_scale_factor,
                random.uniform(box_scale_min, box_scale_max) * cube_scale_factor,
                random.uniform(box_scale_min, box_scale_max) * cube_scale_factor
            ])
            # Per i cubi, il loro pivot è già al centro.
            # create_prim gestisce la creazione di un Cube con le trasformazioni date.
            asset_root_prim = create_prim( # Qui asset_root_prim sarà il cubo stesso
                prim_path=asset_root_prim_path, # Usiamo il path dell'asset root
                prim_type="Cube",
                position=spawn_position_gf, # Posizione del centro
                orientation=spawn_orientation_np_wxyz,
                scale=cube_dims_np
            )

        if not asset_root_prim or not asset_root_prim.IsValid():
            asset_info = f"(asset: {actual_asset_path_chosen})" if actual_asset_path_chosen else "(cubo procedurale)"
            print(f"simple_box_spawner.py: ERRORE [{box_prim_name}] Creazione prim root '{asset_root_prim_path}' fallita {asset_info}.")
            continue

        print(f"simple_box_spawner.py: Prim root '{asset_root_prim.GetPath()}' ({prim_type_str}) creato.")

        # --- Logica di applicazione Materiale PBR ---
        # Ora dobbiamo decidere a quale prim applicare il materiale.
        # Per i cubi procedurali: a asset_root_prim.
        # Per gli asset USD: alle mesh all'interno della geometria referenziata (sotto CenteringXform/AssetGeometry).
        material_target_prim_for_binding = asset_root_prim # Default per cubi procedurali

        if not is_procedural_cube:
            # Per gli asset, i materiali dovrebbero essere applicati alle mesh reali.
            # La ricerca delle mesh partirà dal prim che effettivamente referenzia la geometria.
            # Che è asset_ref_prim nel nostro nuovo schema.
            asset_geometry_prim = stage.GetPrimAtPath(f"{asset_root_prim_path}/CenteringXform/AssetGeometry")
            if asset_geometry_prim and asset_geometry_prim.IsValid():
                 material_target_prim_for_binding = asset_geometry_prim # Le mesh saranno figlie di questo
            else:
                print(f"simple_box_spawner.py: AVVISO [{box_prim_name}] Non trovato il prim della geometria dell'asset per il binding del materiale.")
                # Non fare binding se non troviamo il contenitore della geometria

        material_applied_to_this_prim_group = False
        if available_pbr_material_paths and material_target_prim_for_binding.IsValid(): # Aggiunto check validità
            apply_material_now = False
            if is_procedural_cube:
                apply_material_now = True
            elif not is_procedural_cube and asset_material_override_probability > 0 and random.random() < asset_material_override_probability:
                apply_material_now = True

            if apply_material_now:
                selected_material_path = random.choice(available_pbr_material_paths)
                material_to_bind = UsdShade.Material.Get(stage, selected_material_path)

                if material_to_bind and material_to_bind.GetPrim().IsValid():
                    if is_procedural_cube: # Cubo procedurale, lega direttamente al prim radice (che è il cubo)
                        try:
                            UsdShade.MaterialBindingAPI(asset_root_prim).Bind(material_to_bind)
                            print(f"simple_box_spawner.py: Materiale PBR '{selected_material_path}' applicato a {prim_type_str} '{asset_root_prim.GetPath()}'.")
                            material_applied_to_this_prim_group = True
                        except Exception as e_bind:
                            print(f"simple_box_spawner.py: ERRORE binding materiale a {prim_type_str} '{asset_root_prim.GetPath()}': {e_bind}")
                    else: # Asset USD, itera sulle mesh sotto 'material_target_prim_for_binding'
                        meshes_found_in_asset_count = 0
                        for descendant_prim in Usd.PrimRange(material_target_prim_for_binding): # Cerca sotto il prim che contiene la geometria
                            if descendant_prim.IsA(UsdGeom.Mesh):
                                try:
                                    UsdShade.MaterialBindingAPI(descendant_prim).Bind(material_to_bind)
                                    print(f"simple_box_spawner.py: Materiale PBR '{selected_material_path}' applicato alla mesh '{descendant_prim.GetPath()}'.")
                                    meshes_found_in_asset_count += 1
                                except Exception as e_bind_mesh:
                                    print(f"simple_box_spawner.py: ERRORE binding materiale a mesh '{descendant_prim.GetPath()}': {e_bind_mesh}")
                        
                        if meshes_found_in_asset_count > 0:
                            material_applied_to_this_prim_group = True
                        else:
                            print(f"simple_box_spawner.py: AVVISO - Nessuna mesh trovata sotto '{material_target_prim_for_binding.GetPath()}' per materiale.")
                else:
                    print(f"simple_box_spawner.py: AVVISO - Materiale PBR '{selected_material_path}' non trovato o non valido.")

        if is_procedural_cube and not material_applied_to_this_prim_group: # Per cubi procedurali
            gprim_api = UsdGeom.Gprim(asset_root_prim) # Il prim radice è il cubo
            if gprim_api:
                color_primvar = gprim_api.CreateDisplayColorPrimvar()
                if color_primvar:
                    color_primvar.Set([default_color])
                    print(f"simple_box_spawner.py: Colore fallback applicato a {prim_type_str} '{asset_root_prim.GetPath()}'.")
                # ... (else per errore creazione primvar)
            # ... (else per non Gprim)

        # Applica fisica e semantica all'Xform radice dell'asset/cubo
        UsdPhysics.CollisionAPI.Apply(asset_root_prim)
        UsdPhysics.RigidBodyAPI.Apply(asset_root_prim)
        mass_api = UsdPhysics.MassAPI.Apply(asset_root_prim)
        mass_api.CreateMassAttr().Set(box_mass)

        if semantic_label:
            add_update_semantics(asset_root_prim, semantic_label)

        created_box_prims_roots.append(asset_root_prim)

    print(f"simple_box_spawner.py: Generazione di {len(created_box_prims_roots)} scatole completata.")
    return created_box_prims_roots
