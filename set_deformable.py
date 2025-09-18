from omni.isaac.kit import SimulationApp

# Inizializza l'applicazione in modalità headless
SETUP = {"headless": True}
simulation_app = SimulationApp(SETUP)

import omni.usd
import omni.kit.commands
from pxr import Gf, Usd, UsdGeom
import os

# Importa le utility e la classe World
from omni.physx.scripts import deformableUtils, physicsUtils
from omni.isaac.core import World

# --- CONFIGURAZIONE (invariata) ---
RELATIVE_INPUT_USD_PATH = "./pre_build_asset/result.usd" 
RELATIVE_OUTPUT_DIR = "./"
OUTPUT_FILENAME = "risultato_deformabile.usd"
TARGET_PRIM_PATH = "/World/node_"

YOUNGS_MODULUS = 80000.0
POISSONS_RATIO = 0.35
DAMPING_SCALE = 0.3

# --- FINE CONFIGURAZIONE ---

def make_prim_deformable():
    print(f"SCRIPT: Inizio del processo...")

    script_directory = os.path.dirname(os.path.abspath(__file__))
    input_usd_path_abs = os.path.join(script_directory, RELATIVE_INPUT_USD_PATH)
    output_usd_path_abs = os.path.join(script_directory, RELATIVE_OUTPUT_DIR, OUTPUT_FILENAME)

    if not os.path.isfile(input_usd_path_abs):
        print(f"ERRORE: File di input non trovato: {input_usd_path_abs}")
        simulation_app.close() # Chiudi se il file non esiste
        return

    print(f"1/4: Caricamento stage da: {input_usd_path_abs}")
    omni.usd.get_context().open_stage(input_usd_path_abs)
    
    # Esegui un paio di aggiornamenti per stabilizzare il caricamento
    for _ in range(4):
        simulation_app.update()

    # --- MODIFICA CHIAVE ---
    # Inizializza il World ORA, DOPO che lo stage è stato caricato.
    world = World()
    stage = world.scene.stage # Ottieni lo stage dal mondo appena creato
    # --- FINE MODIFICA ---

    if not stage:
        print(f"ERRORE: Impossibile ottenere lo stage dopo il caricamento.")
        simulation_app.close()
        return

    print(f"2/4: Ricerca del prim target a: {TARGET_PRIM_PATH}")
    root_prim_to_modify = stage.GetPrimAtPath(TARGET_PRIM_PATH)

    if not root_prim_to_modify.IsValid():
        print(f"ERRORE: Impossibile trovare un prim al percorso '{TARGET_PRIM_PATH}'.")
        simulation_app.close()
        return

    print(f"3/4: Applicazione delle proprietà deformabili...")
    
    material_path = "/World/Looks/DeformableMaterial_SolidRubber"
    deformableUtils.add_deformable_body_material(
        stage, material_path, youngs_modulus=YOUNGS_MODULUS,
        poissons_ratio=POISSONS_RATIO, damping_scale=DAMPING_SCALE
    )
    simulation_app.update()

    meshes_found = []
    if root_prim_to_modify.IsA(UsdGeom.Mesh):
        meshes_found.append(root_prim_to_modify)
    else:
        print(f"'{TARGET_PRIM_PATH}' non è una mesh. Cerco mesh discendenti...")
        for descendant_prim in Usd.PrimRange(root_prim_to_modify):
            if descendant_prim.IsA(UsdGeom.Mesh):
                meshes_found.append(descendant_prim)

    if not meshes_found:
        print(f"ERRORE: Nessuna mesh trovata sotto il percorso '{TARGET_PRIM_PATH}'.")
        simulation_app.close()
        return

    for mesh_prim in meshes_found:
        print(f"  -> Applico proprietà deformabili a: {mesh_prim.GetPath()}")
        deformableUtils.add_physx_deformable_body(
            stage, mesh_prim.GetPath(), simulation_hexahedral_resolution=10
        )
    
        simulation_app.update()
        physicsUtils.add_physics_material_to_prim(stage, mesh_prim, material_path)
        print(f"    -> Materiale '{material_path}' collegato a '{mesh_prim.GetPath()}'.")
    
    print("In attesa che i comandi di binding vengano processati...")
    for _ in range(10):
        simulation_app.update()

    print("Proprietà applicate con successo.")

    print(f"4/4: Salvataggio della nuova versione in: {output_usd_path_abs}")
    os.makedirs(os.path.dirname(output_usd_path_abs), exist_ok=True)
    omni.usd.get_context().save_as_stage(output_usd_path_abs)

    print(f"PROCESSO COMPLETATO! Il file modificato è stato salvato.")

# Esegui la funzione
make_prim_deformable()

# Chiudi l'applicazione in modo pulito
simulation_app.close()