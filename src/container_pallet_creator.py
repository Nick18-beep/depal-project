# src/container_pallet_creator.py (o il nome del tuo modulo spawner)

import random
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Sdf, UsdShade # Assicurati che Usd sia importato
import warnings

# Importazioni da Isaac Sim Core
try:
    from isaacsim.core.utils.prims import create_prim, get_prim_at_path
    from isaacsim.core.utils.rotations import euler_angles_to_quat
    from isaacsim.core.utils.semantics import add_update_semantics
except ImportError:
    warnings.warn(
        "Importazioni da 'isaacsim.core' fallite. Tentativo con 'omni.isaac.core'. "
        "Assicurati che i percorsi di importazione siano corretti per la tua versione di Isaac Sim.",
        stacklevel=2)
    from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
    from omni.isaac.core.utils.rotations import euler_angles_to_quat
    from omni.isaac.core.utils.semantics import add_update_semantics


def get_all_mesh_prims_in_prim(stage: Usd.Stage, root_prim_path: str) -> list[Usd.Prim]:
    """
    Percorre la gerarchia del prim a partire da root_prim_path e restituisce
    un elenco di tutti i prim UsdGeom.Mesh concreti trovati.
    """
    mesh_prims = []
    start_prim = stage.GetPrimAtPath(root_prim_path)
    if not start_prim:
        warnings.warn(f"SpawnerModule: Prim radice non trovato a '{root_prim_path}'", stacklevel=2)
        return mesh_prims

    # CORREZIONE: Usa Usd.PrimRange per iterare sui discendenti
    # Usd.PrimRange.PreOrder(start_prim) itera su start_prim e poi sui suoi discendenti in pre-ordine.
    # Saltiamo il primo elemento se è start_prim stesso, poiché lo controlliamo separatamente.
    
    # Itera su tutti i prim nella gerarchia a partire da start_prim
    for prim_in_hierarchy in Usd.PrimRange.AllPrims(start_prim): # AllPrims itera su start_prim e tutti i suoi discendenti
        if prim_in_hierarchy == start_prim: # Controlliamo start_prim separatamente sotto
            continue

        if prim_in_hierarchy.IsA(UsdGeom.Mesh) and prim_in_hierarchy.GetPrimTypeInfo().GetTypeName() == "Mesh":
            mesh_prims.append(prim_in_hierarchy)
            
    # Controlla anche se il prim radice stesso è un Mesh
    if start_prim.IsA(UsdGeom.Mesh) and start_prim.GetPrimTypeInfo().GetTypeName() == "Mesh":
        if start_prim not in mesh_prims: # Evita duplicati se start_prim è l'unico prim
             mesh_prims.append(start_prim)
             
    if not mesh_prims:
        print(f"DEBUG (SpawnerModule): Nessun prim UsdGeom.Mesh concreto trovato sotto {root_prim_path}.")
    return mesh_prims

def _get_material_paths_from_folder(stage: Usd.Stage, materials_folder_str: str) -> list[str]:
    """
    Trova tutti i percorsi dei materiali UsdShade validi sotto la cartella specificata.
    """
    found_material_paths = []
    materials_root_prim = stage.GetPrimAtPath(materials_folder_str)
    if not materials_root_prim:
        print(f"SpawnerModule: AVVISO - Percorso cartella materiali '{materials_folder_str}' non trovato.")
        return []

    for prim in materials_root_prim.GetChildren(): 
        if prim.IsA(UsdShade.Material):
            material = UsdShade.Material(prim)
            if material: 
                found_material_paths.append(prim.GetPath().pathString)
    
    if not found_material_paths:
         print(f"SpawnerModule: AVVISO - Nessun materiale UsdShade trovato direttamente sotto '{materials_folder_str}'.")
    return found_material_paths


def spawn_single_pallet( 
    stage: Usd.Stage,
    usd_asset_paths: list[str], 
    parent_path: str,
    prim_name: str,
    position: np.ndarray,
    orientation_euler_degrees: tuple = (0.0, 0.0, 0.0),
    scale: np.ndarray = None, 
    mass: float = None, 
    
    semantic_label: str = "spawned_object",
    available_material_paths: str | None = "/World/Looks", 
    material_application_probability: float = 0.0
) -> Usd.Prim | None:
    """
    Crea un singolo asset (pallet, container, ecc.), applica fisica, semantica
    e opzionalmente un materiale casuale da una cartella specificata in available_material_paths.
    """
    if not usd_asset_paths:
        print("ERRORE (SpawnerModule): Nessun percorso USD fornito per l'asset.")
        return None

    chosen_usd_path = random.choice(usd_asset_paths)
    asset_prim_path_str = f"{parent_path}/{prim_name}"
    
    if parent_path and not get_prim_at_path(parent_path):
        UsdGeom.Xform.Define(stage, parent_path)
        print(f"INFO (SpawnerModule): Creato Xform genitore a '{parent_path}'.")

    current_scale_gf = Gf.Vec3f(*(scale if scale is not None else np.array([1.0, 1.0, 1.0])))
    position_gf = Gf.Vec3d(*position.astype(float))
    orientation_quat_wxyz = euler_angles_to_quat(np.array(orientation_euler_degrees), degrees=True)

    asset_prim = create_prim(
        prim_path=asset_prim_path_str,
        prim_type="Xform", 
        position=position_gf,
        orientation=orientation_quat_wxyz,
        scale=current_scale_gf,
        usd_path=chosen_usd_path 
    )

    if not asset_prim or not asset_prim.IsValid():
        print(f"ERRORE (SpawnerModule): Impossibile creare il prim '{asset_prim_path_str}' con USD '{chosen_usd_path}'.")
        return None
    
    print(f"INFO (SpawnerModule): Asset '{asset_prim.GetPath()}' creato con USD '{chosen_usd_path}'.")

    UsdPhysics.CollisionAPI.Apply(asset_prim)

    

    if semantic_label:
        try:
            add_update_semantics(asset_prim, semantic_label)
        except Exception as e_sem:
            print(f"ERRORE (SpawnerModule): durante l'applicazione dell'etichetta semantica a '{asset_prim.GetPath()}': {e_sem}")

    if available_material_paths and random.random() < material_application_probability:
        materials_folder_str = available_material_paths 
        list_of_actual_material_paths = _get_material_paths_from_folder(stage, materials_folder_str)

        if list_of_actual_material_paths:
            chosen_material_path_str = random.choice(list_of_actual_material_paths)
            material_to_apply_prim = stage.GetPrimAtPath(chosen_material_path_str)

            if material_to_apply_prim and material_to_apply_prim.IsA(UsdShade.Material):
                usd_shade_material_to_apply = UsdShade.Material(material_to_apply_prim)
                # Passa asset_prim (il prim radice dell'asset appena creato) a get_all_mesh_prims_in_prim
                mesh_prims_under_asset = get_all_mesh_prims_in_prim(stage, asset_prim.GetPath().pathString)


                if not mesh_prims_under_asset:
                    print(f"AVVISO (SpawnerModule): Nessun UsdGeom.Mesh trovato sotto '{asset_prim.GetPath()}'. Applico a Xform.")
                    try:
                        binding_api = UsdShade.MaterialBindingAPI.Apply(asset_prim)
                        binding_api.Bind(usd_shade_material_to_apply, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
                        print(f"INFO (SpawnerModule): Materiale '{chosen_material_path_str}' applicato (fallback) a Xform '{asset_prim.GetPath()}'.")
                    except Exception as e_bind_root:
                        print(f"ERRORE (SpawnerModule): Impossibile applicare materiale '{chosen_material_path_str}' al root Xform '{asset_prim.GetPath()}': {e_bind_root}")
                else:
                    applied_count = 0
                    for mesh_prim_in_list in mesh_prims_under_asset: # Rinomina variabile per chiarezza
                        try:
                            binding_api = UsdShade.MaterialBindingAPI.Apply(mesh_prim_in_list)
                            binding_api.Bind(usd_shade_material_to_apply, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
                            applied_count += 1
                        except Exception as e_bind_mesh:
                            print(f"ERRORE (SpawnerModule): Impossibile applicare materiale '{chosen_material_path_str}' a '{mesh_prim_in_list.GetPath()}': {e_bind_mesh}")
                    if applied_count > 0:
                         print(f"INFO (SpawnerModule): Materiale '{chosen_material_path_str}' applicato a {applied_count} mesh(es) in '{asset_prim.GetPath()}'.")
                    else:
                        print(f"AVVISO (SpawnerModule): Materiale non applicato a nessun mesh in '{asset_prim.GetPath()}'.")
            elif material_to_apply_prim: 
                 print(f"AVVISO (SpawnerModule): Il prim '{chosen_material_path_str}' (da '{materials_folder_str}') non è un UsdShade.Material.")
            else: 
                print(f"AVVISO (SpawnerModule): Materiale scelto '{chosen_material_path_str}' (da '{materials_folder_str}') non trovato.")
        elif available_material_paths: 
            print(f"AVVISO (SpawnerModule): Nessun materiale trovato nella cartella '{available_material_paths}'. Applicazione materiale saltata.")
            
    elif random.random() < material_application_probability and not available_material_paths:
        print("AVVISO (SpawnerModule): Tentativo di applicare materiale, ma 'available_material_paths' non fornito.")

    return asset_prim
