import os, yaml, numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

# Tentativo di importare le API corrette per la versione di Isaac Sim (Kit 106 ~ 2022.2)
try:
    from isaacsim.core.world import World # Per Isaac Sim 2022.2 e precedenti
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.robot_setup.grasp_editor import import_grasps_from_file
except ImportError:
    # Per Isaac Sim 2023.1+ (se Kit 106 fosse mappato diversamente)
    from omni.isaac.core import World
    from omni.isaac.core.articulations import Articulation as SingleArticulation # Articulation è la classe base
    from omni.isaac.core.utils.types import ArticulationAction
    from omni.isaac.core.grasp_editing import import_grasp_configs_from_file as import_grasps_from_file # API cambiata


URL_GRIPPER = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/4.5/Isaac/Robots/Robotiq/2F-140/Robotiq_2F_140_config.usd"
)
URL_CAN = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/2023.1.1/Isaac/Props/YCB/Axis_Aligned/005_tomato_soup_can.usd"
)

# Nome del joint del dito che si vuole controllare.
# Questo DEVE corrispondere al nome del DOF nell'asset USD del gripper.
# Per Robotiq 2F-140, è spesso "finger_joint" o simile.
# Se questo script fallisce con "Joint ... non trovato", controlla i DOF disponibili stampati.
JOINT_TO_CONTROL = "finger_joint"

DROP_Z, LIFT_Z = 0.10, 0.15
DT = 1 / 60
T_PREOPEN, T_CLOSE, T_LIFT = 0.6, 0.9, 1.4 # Aumentato leggermente per dare più tempo
TMP_YAML = "top_grasp.yaml"

def find_articulation_root_prim_path(stage: Usd.Stage, parent_scope_path: str, expected_art_root_name_in_asset: str = None) -> str:
    """
    Cerca un prim con ArticulationRootAPI sotto parent_scope_path (dove l'asset è referenziato).
    expected_art_root_name_in_asset è il nome del prim radice dell'articolazione *all'interno* dell'asset.
    """
    parent_scope_prim = stage.GetPrimAtPath(parent_scope_path)
    if not parent_scope_prim.IsValid():
        raise RuntimeError(f"Scope genitore {parent_scope_path} non valido.")

    # Tentativo 1: Usare il nome atteso del prim radice dell'articolazione all'interno dell'asset
    if expected_art_root_name_in_asset:
        candidate_path = f"{parent_scope_path}/{expected_art_root_name_in_asset}"
        candidate_prim = stage.GetPrimAtPath(candidate_path)
        if candidate_prim.IsValid() and candidate_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"DEBUG: ArticulationRootAPI trovata esplicitamente a: {candidate_path}")
            return candidate_path
        else:
            print(f"WARN: ArticulationRootAPI NON trovata al path atteso: {candidate_path}. Tentativo con ricerca generica dei discendenti...")

    # Tentativo 2: Ricerca generica tra i discendenti diretti dello scope di referenziamento
    # Questo è utile se non conosciamo expected_art_root_name_in_asset o se l'asset ha una struttura diversa.
    for child_prim in parent_scope_prim.GetChildren(): # Solo figli diretti dello scope di referenza
        # L'ArticulationRootAPI dovrebbe essere su un prim che è la radice dell'asset referenziato
        if child_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"DEBUG: ArticulationRootAPI trovata su un figlio diretto dello scope: {child_prim.GetPath().pathString}")
            return child_prim.GetPath().pathString
        # A volte, l'ArticulationRoot è un discendente ulteriore
        for p_desc in Usd.PrimRange.AllDescendants(child_prim):
            if p_desc.HasAPI(UsdPhysics.ArticulationRootAPI):
                print(f"DEBUG: ArticulationRootAPI trovata tramite ricerca generica dei discendenti a: {p_desc.GetPath().pathString}")
                return p_desc.GetPath().pathString

    raise RuntimeError(f"Nessun ArticulationRootAPI trovato sotto {parent_scope_path}, né con nome atteso né con ricerca generica.")


def get_base_link_path(stage: Usd.Stage, art_root_prim_path: str, base_link_name_suffix: str = "_base_link") -> str:
    art_root_prim = stage.GetPrimAtPath(art_root_prim_path)
    if not art_root_prim.IsValid():
        raise RuntimeError(f"Prim ArticulationRoot {art_root_prim_path} non valido.")
    # Cerca il base_link come discendente dell'art_root_prim
    for p in Usd.PrimRange.AllDescendants(art_root_prim):
        if p.GetName().endswith(base_link_name_suffix): # es. "robotiq_base_link"
            print(f"DEBUG: base_link '{p.GetName()}' trovato a: {p.GetPath().pathString}")
            return p.GetPath().pathString
    raise RuntimeError(f"Nessun base_link con suffisso '{base_link_name_suffix}' trovato sotto {art_root_prim_path}")


def set_friction_on_prim_material(prim: Usd.Prim, static_friction=2.0, dynamic_friction=2.0):
    # Applica solo se il prim ha una geometria di collisione physics-enabled
    has_collision = False
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        has_collision = True
    else: # Controlla se è un UsdGeom prim che potrebbe avere collisioni implicite
        geom_prim = UsdGeom.Imageable(prim)
        if geom_prim:
            # Questa è una semplificazione; idealmente si controlla il purpose della geometria
            if UsdGeom.Mesh(geom_prim) or UsdGeom.Cube(geom_prim) or UsdGeom.Sphere(geom_prim) or UsdGeom.Capsule(geom_prim):
                 # Applica la CollisionAPI se non c'è ma c'è una geometria per essere sicuri
                UsdPhysics.CollisionAPI.Apply(prim)
                has_collision = True
    
    if has_collision:
        # print(f"DEBUG: Applicazione/aggiornamento materiale fisico per attrito su {prim.GetPath()}")
        mat_api = UsdPhysics.MaterialAPI.Apply(prim) # Apply crea se non esiste
        mat_api.CreateStaticFrictionAttr().Set(static_friction)
        mat_api.CreateDynamicFrictionAttr().Set(dynamic_friction)
    # else:
        # print(f"DEBUG: Attrito non applicato a {prim.GetPath()} - no CollisionAPI o geometria nota.")


def write_grasp_config_yaml(file_path: str, object_prim_path: str, gripper_base_link_path: str, joint_name: str):
    # Valori tipici per Robotiq 2F-140:
    # Joint range: circa 0.0 (chiuso) a 0.83-0.87 rad (completamente aperto)
    # 0.70 rad è quasi completamente aperto. 1.00 rad è probabilmente oltre il limite.
    # Usiamo valori più conservativi e comuni.
    grasp_joint_value = 0.05  # Quasi chiuso
    pre_grasp_joint_value = 0.80 # Ben aperto
    
    grasp_data = {
        "format": "isaac_grasp", "format_version": "1.0",
        "object_frame_link": object_prim_path,
        "gripper_frame_link": gripper_base_link_path,
        "grasps": {
            "top_can_grasp": { # Nome più descrittivo per la presa
                "confidence": 1.0,
                "position": [0.0, 0.0, 0.12], # Posizione dell'origine dell'oggetto nel frame del gripper_base_link
                "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}, # Orientamento identità
                "cspace_position": {joint_name: grasp_joint_value},
                "pregrasp_cspace_position": {joint_name: pre_grasp_joint_value},
            }
        },
    }
    with open(file_path, "w") as f:
        yaml.safe_dump(grasp_data, f)
    print(f"DEBUG: Configurazione grasp scritta in {file_path}")


def main():
    world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)
    world.scene.add_default_ground_plane()
    stage = world.stage
    print("DEBUG: Mondo e ground plane creati.")

    # --- 1) Lattina (Oggetto da afferrare) ---
    can_prim_path = "/World/can"
    can_prim = stage.DefinePrim(can_prim_path, "Xform")
    can_prim.GetReferences().AddReference(URL_CAN)
    UsdGeom.Xformable(can_prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.051)) # Leggermente sopra il suolo
    UsdPhysics.RigidBodyAPI.Apply(can_prim)
    UsdPhysics.CollisionAPI.Apply(can_prim) # Il warning su triangle mesh -> convex hull è ok
    UsdPhysics.MassAPI.Apply(can_prim).CreateMassAttr().Set(0.1) # kg
    print(f"DEBUG: Lattina definita a {can_prim_path}")

    # --- 2) Gripper ---
    # a) Definisci un Xform genitore per il gripper (il "pad" che muoveremo)
    gripper_pad_xform_path = "/World/gripper_pad"
    pad_xform = stage.DefinePrim(gripper_pad_xform_path, "Xform")

    # b) Definisci un Xform *sotto* il pad, a cui l'asset USD del gripper sarà referenziato.
    #    Questo aiuta a mantenere pulita la gerarchia.
    gripper_asset_scope_path = f"{gripper_pad_xform_path}/Robotiq_Gripper_Asset" # Nome descrittivo per lo scope
    gripper_asset_scope_prim = stage.DefinePrim(gripper_asset_scope_path, "Xform")
    gripper_asset_scope_prim.GetReferences().AddReference(URL_GRIPPER)
    print(f"DEBUG: Asset gripper {URL_GRIPPER} referenziato sotto {gripper_asset_scope_path}")

    print("DEBUG: Attesa caricamento asset gripper (simulazione di alcuni frame)...")
    # È cruciale dare tempo all'engine di caricare e processare l'asset USD.
    # Il numero di update necessari può variare.
    for i in range(120): # Numero di update; potrebbe necessitare di aggiustamenti
        simulation_app.update()
        # if i > 0 and i % 30 == 0:
        #     print(f"DEBUG: Update di caricamento asset #{i}")
    print("DEBUG: Caricamento asset (presumibilmente) completato.")

    # c) Trova il path effettivo del prim con ArticulationRootAPI *all'interno* dell'asset referenziato.
    #    IPOTESI: il prim radice dell'articolazione nell'asset USD si chiama "Robotiq_2F_140".
    #    VERIFICA QUESTO NOME se lo script fallisce qui o se l'articolazione non funziona.
    #    Apri l'USD in Isaac Sim e controlla il nome del prim con l'icona dell'articolazione.
    expected_root_name_in_asset_usd = "Robotiq_2F_140" # <-- IPOTESI CRITICA
    try:
        actual_gripper_art_root_path = find_articulation_root_prim_path(stage, gripper_asset_scope_path, expected_root_name_in_asset_usd)
    except RuntimeError as e:
        print(f"FATAL: {e}")
        print("SUGGERIMENTO: Controlla il nome 'expected_root_name_in_asset_usd' nello script.")
        print(f"SUGGERIMENTO: Apri l'asset {URL_GRIPPER} in Isaac Sim per trovare il nome corretto del prim con ArticulationRootAPI.")
        simulation_app.close()
        return

    print(f"DEBUG: Path ArticulationRoot effettivo del gripper: {actual_gripper_art_root_path}")

    # d) Trova il base_link del gripper sotto l'articulation root.
    #    Il suffisso "_base_link" è una convenzione comune (es. "robotiq_base_link").
    try:
        gripper_base_link_path = get_base_link_path(stage, actual_gripper_art_root_path, base_link_name_suffix="base_link")
    except RuntimeError as e:
        print(f"FATAL: {e}")
        print(f"SUGGERIMENTO: Controlla che un prim con suffisso '_base_link' esista sotto {actual_gripper_art_root_path}.")
        simulation_app.close()
        return
    print(f"DEBUG: Path base_link del gripper: {gripper_base_link_path}")

    # e) Posiziona il gripper_pad (che muove l'intero gripper)
    #    L'altezza di presa è (base lattina + offset y del grasp YAML) = 0.051 + 0.12 = 0.171m
    #    L'altezza iniziale del pad è (altezza di presa + DROP_Z) = 0.171 + 0.10 = 0.271m
    initial_pad_z = (0.051 + 0.12) + DROP_Z
    UsdGeom.Xformable(pad_xform).AddTranslateOp().Set(Gf.Vec3d(0, 0, initial_pad_z))
    UsdGeom.Xformable(pad_xform).AddOrientOp().Set(Gf.Quatf(0.0, Gf.Vec3f(1, 0, 0))) # Dita in giù (rotazione 180° su X locale)
    print(f"DEBUG: Gripper pad posizionato a z={initial_pad_z} con dita in giù.")

    # f) Applica attrito ai link di contatto del gripper (fingertips, pads, ecc.)
    #    I nomi dei link sono relativi all' `actual_gripper_art_root_path`.
    #    VERIFICA QUESTI NOMI se la presa slitta.
    gripper_contact_link_names = [
        "left_inner_finger", "right_inner_finger",
        "left_inner_finger_pad", "right_inner_finger_pad", # Spesso i pad sono i contatti principali
        "left_outer_finger", "right_outer_finger",
        "left_outer_knuckle", "right_outer_knuckle", # Knuckles potrebbero avere collisioni
        "left_inner_knuckle", "right_inner_knuckle"
    ]
    print("DEBUG: Applicazione attrito ai link di contatto del gripper...")
    art_root_prim_for_friction = stage.GetPrimAtPath(actual_gripper_art_root_path)
    if art_root_prim_for_friction.IsValid():
        for link_name_part in gripper_contact_link_names:
            # Cerca il link come discendente, non solo figlio diretto
            found_link = False
            for p_desc_fric in Usd.PrimRange.AllDescendants(art_root_prim_for_friction):
                if p_desc_fric.GetName().endswith(link_name_part): # Usa endswith per flessibilità
                    set_friction_on_prim_material(p_desc_fric)
                    found_link = True
                    # print(f"DEBUG: Attrito impostato per {p_desc_fric.GetPath()}")
                    break # Trovato, passa al prossimo nome
            # if not found_link:
            #     print(f"WARN: Link di contatto '{link_name_part}' non trovato sotto {actual_gripper_art_root_path} per l'attrito.")
    else:
        print(f"WARN: ArticulationRoot prim {actual_gripper_art_root_path} non valido per impostare l'attrito.")


    # --- 3) Configurazione Grasp ---
    write_grasp_config_yaml(TMP_YAML, can_prim_path, gripper_base_link_path, JOINT_TO_CONTROL)
    grasp_spec_collection = import_grasps_from_file(TMP_YAML) # API corretta per Kit 106
    grasp_definition = grasp_spec_collection.get_grasp_dict_by_name("top_can_grasp")
    
    # Valori dei joint dalla configurazione grasp (già corretti per Robotiq 2F-140)
    q_open_rad = grasp_definition["pregrasp_cspace_position"][JOINT_TO_CONTROL]
    q_close_rad = grasp_definition["cspace_position"][JOINT_TO_CONTROL]
    print(f"DEBUG: Valori joint: q_open={q_open_rad:.3f} rad, q_close={q_close_rad:.3f} rad")

    # --- 4) Inizializzazione Articolazione Gripper ---
    print(f"DEBUG: Aggiunta SingleArticulation per: {actual_gripper_art_root_path}")
    # Usa il path corretto dell'ArticulationRootAPI trovato!
    gripper_articulation = world.scene.add(
        SingleArticulation(
            prim_path=actual_gripper_art_root_path,
            name="robotiq_2f140_gripper" # Nome univoco per la simulazione
        )
    )
    
    # È buona pratica fare alcuni step di simulazione dopo aver aggiunto oggetti
    # e prima di chiamare reset() o interagire con la fisica.
    print("DEBUG: Esecuzione di alcuni step di simulazione post-aggiunta articolazione...")
    if world.is_playing(): world.pause() # Metti in pausa se era già in play
    for _ in range(10): # Numero di step
        world.step(render=False) # Non serve rendering qui

    print("DEBUG: Chiamata a world.reset()...")
    world.reset() # Qui avvengono le inizializzazioni della fisica critica
    print("DEBUG: world.reset() completato.")
    world.play() # Assicurati che la simulazione sia in esecuzione
    for _ in range(int(0.2/DT)): world.step(render=True) # Attendi un po' che si stabilizzi


    # --- 5) Controllo Articolazione ---
    gripper_controller = gripper_articulation.get_articulation_controller()
    finger_dof_index = gripper_articulation.get_dof_index(JOINT_TO_CONTROL)
    
    if finger_dof_index is None:
        all_dof_names = [gripper_articulation.get_dof_name(i) for i in range(gripper_articulation.num_dof)]
        print(f"FATAL: Joint '{JOINT_TO_CONTROL}' NON trovato nell'articolazione '{actual_gripper_art_root_path}'.")
        print(f"DOF disponibili: {all_dof_names}")
        print("SUGGERIMENTO: Verifica la variabile 'JOINT_TO_CONTROL' nello script.")
        simulation_app.close()
        return
    print(f"DEBUG: DOF '{JOINT_TO_CONTROL}' trovato all'indice {finger_dof_index}.")

    action_open = ArticulationAction(joint_positions=np.array([q_open_rad]), joint_indices=np.array([finger_dof_index]))
    action_close = ArticulationAction(joint_positions=np.array([q_close_rad]), joint_indices=np.array([finger_dof_index]))

    # --- 6) Esecuzione Sequenza Pick & Lift ---
    print("DEBUG: APERTURA dita del gripper...")
    gripper_controller.apply_action(action_open)
    for _ in range(int(T_PREOPEN / DT)): world.step(render=True)

    print("DEBUG: DISCESA del gripper...")
    descent_speed_mps = 0.03 # m/s
    num_steps_for_drop = max(1, int(DROP_Z / (descent_speed_mps * DT)))
    for i in range(num_steps_for_drop):
        current_pad_pos, current_pad_rot = pad_xform.GetWorldPose() # Muoviamo il pad
        new_pad_pos = current_pad_pos + Gf.Vec3d(0, 0, -DROP_Z / num_steps_for_drop)
        pad_xform.SetWorldPose(new_pad_pos, current_pad_rot)
        world.step(render=True)
    for _ in range(int(0.2/DT)): world.step(render=True) # Stabilizza

    print("DEBUG: CHIUSURA dita del gripper...")
    gripper_controller.apply_action(action_close)
    for _ in range(int(T_CLOSE / DT)): world.step(render=True)
    for _ in range(int(0.5/DT)): world.step(render=True) # Tempo per la chiusura e per il contatto

    print("DEBUG: Creazione PhysicsFixedJoint per la presa...")
    # Il giunto fisso collega il base_link del gripper alla lattina
    fixed_joint_prim_path = "/World/grasp_fixed_joint"
    physics_joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(fixed_joint_prim_path))
    physics_joint.GetBody0Rel().SetTargets([Sdf.Path(gripper_base_link_path)])
    physics_joint.GetBody1Rel().SetTargets([Sdf.Path(can_prim_path)])
    # Imposta le pose locali del giunto per mantenere la trasformazione relativa corrente
    # tra base_link e lattina al momento della presa.
    # Questo è spesso necessario per giunti creati dinamicamente.
    # Calcola la trasformazione da body1 (can) a body0 (base_link)
    tf_body0 = UsdGeom.Xformable(stage.GetPrimAtPath(gripper_base_link_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    tf_body1 = UsdGeom.Xformable(stage.GetPrimAtPath(can_prim_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    
    # Pose locali nel frame del giunto (che è il frame del body0 se localPose0 è identità)
    physics_joint.CreateLocalPose0Attr().Set(Gf.Vec3f(0,0,0)) # Gf.Quatf(1.0) per orientamento
    physics_joint.CreateLocalPose1Attr().Set( tf_body0.GetInverse() * tf_body1 * Gf.Vec3f(0,0,0) ) # Trasforma l'origine di body1 nel frame di body0
    # Per essere più precisi, si dovrebbe impostare la trasformazione completa (pos+rot)
    # physics_joint.GetPrim().GetAttribute("physics:localFrame0").Set(Gf.Transform(...)) # o UsdPhysics.JointSetLocalPose
    # UsdPhysics.JointSetLocalPose(physics_joint.GetPrim(), Usd.TimeCode.Default(), Usd.TimeCode.Default(), True, True)

    print(f"DEBUG: PhysicsFixedJoint creato tra {gripper_base_link_path} e {can_prim_path}")
    for _ in range(int(0.2/DT)): world.step(render=True) # Dai tempo al giunto di attivarsi


    print("DEBUG: SOLLEVAMENTO della lattina...")
    lift_speed_mps = 0.04 # m/s
    num_steps_for_lift = max(1, int(LIFT_Z / (lift_speed_mps * DT)))
    for i in range(num_steps_for_lift):
        current_pad_pos, current_pad_rot = pad_xform.GetWorldPose()
        new_pad_pos = current_pad_pos + Gf.Vec3d(0, 0, LIFT_Z / num_steps_for_lift)
        pad_xform.SetWorldPose(new_pad_pos, current_pad_rot)
        world.step(render=True)
    for _ in range(int(0.5/DT)): world.step(render=True) # Stabilizza in alto

    print("✅ Presa e sollevamento (si spera) completati!")
    print("Chiudi la finestra di Isaac Sim per terminare lo script.")
    while simulation_app.is_running():
        world.step(render=True) # Continua a fare lo step per mantenere la simulazione attiva

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"ERRORE CRITICO DURANTE L'ESECUZIONE DI main(): {e}")
        import traceback
        traceback.print_exc()
    finally:
        if os.path.exists(TMP_YAML):
            try:
                os.remove(TMP_YAML)
            except Exception as e_rem:
                print(f"Errore minore nella rimozione di {TMP_YAML}: {e_rem}")
        print("DEBUG: Chiusura dell'applicazione Isaac Sim...")
        simulation_app.close() # Assicura la chiusura corretta
        print("DEBUG: Applicazione Isaac Sim chiusa.")


