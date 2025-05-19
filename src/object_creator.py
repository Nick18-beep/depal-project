# src/object_generator.py

import random
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Sdf, UsdShade # Importa i moduli USD necessari

# Importazioni da Isaac Sim Core (AGGIORNATE ai nuovi percorsi)
from isaacsim.core.utils.prims import create_prim, get_prim_at_path, get_prim_children # Aggiornato
from isaacsim.core.utils.rotations import euler_angles_to_quat # Aggiornato
from isaacsim.core.utils.semantics import add_update_semantics # Aggiornato
# CORREZIONE: Usa il percorso API per i materiali (anche se non creiamo più PreviewSurface qui, lo manteniamo per coerenza se dovesse servire altrove)
from isaacsim.core.api.materials import PreviewSurface # Aggiornato al percorso API


def _get_existing_materials(stage: Usd.Stage, materials_path: str) -> list[UsdShade.Material]:
    """
    Trova tutti i materiali UsdShade validi sotto il percorso specificato.
    """
    found_materials = []
    materials_root_prim = stage.GetPrimAtPath(materials_path)
    if not materials_root_prim:
        print(f"object_generator.py: AVVISO - Percorso materiali '{materials_path}' non trovato.")
        return []

    # Itera sui figli diretti del percorso fornito
    # Potresti voler usare stage.Traverse() per una ricerca più profonda se i materiali sono nidificati
    for prim in materials_root_prim.GetChildren():
        if prim.IsA(UsdShade.Material):
            material = UsdShade.Material(prim)
            if material: # Verifica aggiuntiva
                 found_materials.append(material)
        # Opzionale: potresti voler controllare anche gli Xform che contengono materiali
        # elif prim.IsA(UsdGeom.Xform):
        #     for child_prim in prim.GetChildren():
        #         if child_prim.IsA(UsdShade.Material):
        #             material = UsdShade.Material(child_prim)
        #             if material:
        #                 found_materials.append(material)

    if not found_materials:
         print(f"object_generator.py: AVVISO - Nessun materiale UsdShade trovato sotto '{materials_path}'.")

    return found_materials


def _apply_random_existing_material(stage: Usd.Stage, prim: Usd.Prim, existing_materials: list[UsdShade.Material]):
    """
    Applica un materiale casuale dalla lista fornita al prim specificato.
    """
    if not existing_materials:
        print(f"object_generator.py: AVVISO - Nessun materiale esistente fornito per applicare a '{prim.GetPath()}'.")
        return False

    try:
        # Scegli un materiale a caso dalla lista
        selected_material = random.choice(existing_materials)
        material_path = selected_material.GetPath()

        # Associa il materiale selezionato al prim.
        # Lega al prim radice dell'asset importato.
        target_prim_for_binding = prim
        UsdShade.MaterialBindingAPI(target_prim_for_binding).Bind(selected_material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        print(f"object_generator.py: Materiale esistente '{material_path}' applicato a '{target_prim_for_binding.GetPath()}'.")
        return True

    except Exception as e:
        print(f"object_generator.py: ERRORE durante l'applicazione del materiale esistente a '{prim.GetPath()}': {e}")
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
    asset_scale_min: float = 0.8, # Scala minima relativa all'originale
    asset_scale_max: float = 1.2, # Scala massima relativa all'originale
    object_mass: float = 1.0,
    semantic_label: str = "object",
    change_material_probability: float = 0.3, # Probabilità di cambiare materiale (0.0 a 1.0)
    existing_materials_path: str = "/World/Looks" # Percorso dove TROVARE materiali esistenti da applicare
):
    """
    Genera N oggetti caricandoli casualmente da una lista di percorsi USD,
    applica fisica, semantica e opzionalmente applica un materiale casuale
    scelto tra quelli presenti sotto `existing_materials_path`.

    Args:
        stage: Lo stage USD corrente.
        num_objects: Il numero di oggetti da generare.
        usd_asset_paths: Lista di percorsi ai file USD degli asset da usare.
        parent_path: Il percorso USD sotto cui creare gli oggetti (es. /World/GeneratedObjects).
        base_position: Posizione centrale (x, y) attorno alla quale generare gli oggetti.
        base_z_offset: Offset Z di base da aggiungere alla posizione Z calcolata.
        z_jitter: Variazione casuale massima da aggiungere all'offset Z.
        xy_jitter_range: Intervallo (min, max) per la variazione casuale X e Y.
        asset_scale_min: Fattore di scala minimo (applicato casualmente per asse).
        asset_scale_max: Fattore di scala massimo (applicato casualmente per asse).
        object_mass: Massa da applicare agli oggetti per la fisica.
        semantic_label: Etichetta semantica da applicare agli oggetti.
        change_material_probability: Probabilità (0-1) che un materiale casuale esistente venga applicato all'oggetto.
        existing_materials_path: Percorso USD dove cercare i materiali esistenti da applicare casualmente.
    """
    num_objects=random.randint(num_to_spawn_range[0],num_to_spawn_range[1])
    print(f"object_generator.py: Inizio generazione di {num_objects} oggetti.")

    if not usd_asset_paths:
        print("object_generator.py: ERRORE - La lista 'usd_asset_paths' è vuota. Nessun oggetto verrà generato.")
        return []

    # Assicura che il prim genitore esista
    if not get_prim_at_path(parent_path):
        UsdGeom.Xform.Define(stage, parent_path)
        print(f"object_generator.py: Creato Xform genitore a '{parent_path}'.")

    # Trova i materiali esistenti UNA SOLA VOLTA all'inizio
    available_materials = _get_existing_materials(stage, existing_materials_path)
    if not available_materials:
         print(f"object_generator.py: AVVISO - Nessun materiale trovato in '{existing_materials_path}'. Non sarà possibile applicare materiali casuali.")
         # Potresti decidere di interrompere qui o continuare senza applicare materiali
         # return [] # Esempio: interrompi se non ci sono materiali


    created_object_prims = []

    for i in range(num_objects):
        # Rendi il nome più univoco per evitare potenziali conflitti se la funzione viene chiamata più volte
        object_prim_name = f"GeneratedObject_{random.randint(10000, 99999)}_{i}"
        current_object_prim_path = f"{parent_path}/{object_prim_name}"

        # Calcola posizione casuale
        pos_offset_x = random.uniform(xy_jitter_range[0], xy_jitter_range[1])
        pos_offset_y = random.uniform(xy_jitter_range[0], xy_jitter_range[1])
        pos_offset_z = base_z_offset + random.uniform(0, z_jitter)
        # Assicurati che base_position sia un array numpy e abbia dimensione 3
        if not isinstance(base_position, np.ndarray):
             base_position = np.array(base_position) # Converti se è una lista o tupla
        if base_position.shape == (2,): # Se sono date solo X, Y
            base_position_3d = np.append(base_position, 0.0) # Aggiungi Z=0
        elif base_position.shape == (3,):
             base_position_3d = base_position
        else:
             print(f"object_generator.py: ERRORE - 'base_position' ha forma {base_position.shape}, attesa (2,) o (3,). Uso [0,0,0].")
             base_position_3d = np.array([0.0, 0.0, 0.0])

        position_np = base_position_3d + np.array([pos_offset_x, pos_offset_y, pos_offset_z])
        position_gf = Gf.Vec3d(float(position_np[0]), float(position_np[1]), float(position_np[2]))

        # Calcola orientamento casuale
        random_euler_rad = np.array([random.uniform(0, 2 * np.pi) for _ in range(3)])
        orientation_np_wxyz = euler_angles_to_quat(random_euler_rad, degrees=False) # w,x,y,z

        # Scegli un asset USD casuale dalla lista
        actual_asset_path_chosen = random.choice(usd_asset_paths)

        # Calcola scala casuale indipendente per ogni asse come moltiplicatore
        scale_x = random.uniform(asset_scale_min, asset_scale_max)
        scale_y = random.uniform(asset_scale_min, asset_scale_max)
        scale_z = random.uniform(asset_scale_min, asset_scale_max)
        # La scala in create_prim si aspetta un Gf.Vec3f o simile, non numpy array direttamente
        scale_gf = Gf.Vec3f(scale_x, scale_y, scale_z)

        print(f"object_generator.py: Tentativo di creare '{current_object_prim_path}' usando asset '{actual_asset_path_chosen}'.")
        print(f"object_generator.py: Scala relativa casuale applicata [X, Y, Z]: [{scale_x:.4f}, {scale_y:.4f}, {scale_z:.4f}].")

        # Crea il prim referenziando l'asset USD
        prim = create_prim(
            prim_path=current_object_prim_path,
            usd_path=actual_asset_path_chosen,
            position=position_gf,
            scale=scale_gf, # Usa Gf.Vec3f per la scala
            orientation=orientation_np_wxyz, # Assicurati che sia w,x,y,z
        )

        if not prim or not prim.IsValid():
            print(f"object_generator.py: ERRORE - Creazione del prim '{current_object_prim_path}' fallita (asset: {actual_asset_path_chosen}).")
            continue

        print(f"object_generator.py: Prim '{current_object_prim_path}' creato con successo.")

        # --- Applica Materiale Casuale Esistente ---
        if available_materials and random.random() < change_material_probability:
            print(f"object_generator.py: Tentativo di applicare materiale esistente casuale a '{current_object_prim_path}'...")
            material_applied = _apply_random_existing_material(stage, prim, available_materials)
            if not material_applied:
                print(f"object_generator.py: Applicazione materiale esistente fallita per '{current_object_prim_path}'.")
        else:
             if not available_materials:
                 print(f"object_generator.py: Applicazione materiale saltata per '{current_object_prim_path}' (nessun materiale esistente trovato).")
             else:
                 print(f"object_generator.py: Applicazione materiale saltata per '{current_object_prim_path}' (probabilità: {change_material_probability}).")


        # --- Applica Fisica ---
        try:
            # Applica API al prim radice dell'asset importato
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.RigidBodyAPI.Apply(prim)
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(object_mass)
            print(f"object_generator.py: Fisica (Collision, RigidBody, Mass={object_mass}) applicata a '{current_object_prim_path}'.")
        except Exception as e_phys:
             print(f"object_generator.py: ERRORE durante l'applicazione della fisica a '{current_object_prim_path}': {e_phys}")


        # --- Applica Semantica ---
        if semantic_label:
            try:
                add_update_semantics(prim, semantic_label)
                # La funzione add_update_semantics logga già.
            except Exception as e_sem:
                 print(f"object_generator.py: ERRORE durante l'applicazione della semantica a '{current_object_prim_path}': {e_sem}")


        created_object_prims.append(prim)

    print(f"object_generator.py: Generazione di {len(created_object_prims)}/{num_objects} oggetti completata.")
    return created_object_prims

# --- Esempio di utilizzo ---
# Questo blocco verrebbe eseguito all'interno del tuo script Isaac Sim

# if __name__ == "__main__":
#     # Ottieni lo stage corrente (assumendo che sia già stato creato)
#     import omni.usd
#     stage = omni.usd.get_context().get_stage()
#
#     if stage:
#         # Definisci i percorsi degli asset USD che vuoi usare
#         asset_list = [
#             "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd",
#             "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Props/YCB/Axis_Aligned_Physics/004_sugar_box.usd",
#             "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
#             "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Props/YCB/Axis_Aligned_Physics/006_mustard_bottle.usd",
#             "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Props/YCB/Axis_Aligned_Physics/010_potted_meat_can.usd",
#         ]
#
#         # ASSICURATI CHE QUESTO PERCORSO CONTENGA MATERIALI USD VALIDI (es. .mdl convertiti o UsdPreviewSurface)
#         materials_folder_path = "/World/Looks"
#         # Esempio: potresti creare alcuni materiali manualmente o con un altro script prima:
#         # PreviewSurface(prim_path=f"{materials_folder_path}/Red", color=Gf.Vec3f(1,0,0))
#         # PreviewSurface(prim_path=f"{materials_folder_path}/Blue", color=Gf.Vec3f(0,0,1))
#
#         # Parametri per la generazione
#         num_to_spawn = 10
#         spawn_parent_path = "/World/GeneratedObjects"
#         spawn_base_pos = np.array([0.0, 0.0])
#         spawn_z_offset = 0.5
#         spawn_z_jitter = 0.5
#         spawn_xy_jitter = (-0.6, 0.6)
#         spawn_scale_min = 0.8
#         spawn_scale_max = 1.2
#         spawn_mass = 0.2
#         spawn_semantic = "generated_prop"
#         spawn_material_prob = 0.6 # 60% di probabilità di applicare un materiale esistente
#
#         # Chiama la funzione per generare gli oggetti
#         generated_prims = spawn_objects(
#             stage=stage,
#             num_objects=num_to_spawn,
#             usd_asset_paths=asset_list,
#             parent_path=spawn_parent_path,
#             base_position=spawn_base_pos,
#             base_z_offset=spawn_z_offset,
#             z_jitter=spawn_z_jitter,
#             xy_jitter_range=spawn_xy_jitter,
#             asset_scale_min=spawn_scale_min,
#             asset_scale_max=spawn_scale_max,
#             object_mass=spawn_mass,
#             semantic_label=spawn_semantic,
#             change_material_probability=spawn_material_prob, # Nome parametro aggiornato
#             existing_materials_path=materials_folder_path # Nome parametro aggiornato
#         )
#
#         print(f"\n--- Riepilogo ---")
#         if generated_prims:
#             print(f"Creati {len(generated_prims)} prims:")
#             for prim in generated_prims:
#                 print(f"- {prim.GetPath()}")
#         else:
#             print("Nessun prim generato (controlla errori precedenti).")
#     else:
#         print("ERRORE: Impossibile ottenere lo stage USD corrente.")
