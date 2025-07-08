import numpy as np
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from enum import Enum, auto




# ------------------------------------------------------------------------------
# --- COSTANTI GLOBALI ---
# ------------------------------------------------------------------------------


NUM_CANDIDATE_POSES     = 10
GRIPPER_APPROACH_OFFSET = 1.5
MIN_SAFE_HEIGHT_Z       = 0.45
PAUSE_DURATION_SEC      = 0.25


APPROACH_SPEED          = 1
STOP_BEFORE_DIST        = 0.4


GRIPPER_OPEN_POS        = 0.0
GRIPPER_CLOSED_POS      = 0.95
GRIPPER_TOLERANCE       = 0.02
GRIP_TIMEOUT_SEC        = 2.0


LIFT_HEIGHT_Z           = 1.5
LIFT_SPEED              = 2


SCALE_FACTOR_GRIP       = 4.5


URL_GRIPPER = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/4.5/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd"
)
ROBOT_BASE_PATH         = "/World/RobotBase"
ARTICULATION_ROOT_PATH  = "/World/Robotiq_2F_85"
GRIPPER_BASE_LINK_PATH  = f"{ARTICULATION_ROOT_PATH}/Robotiq_2F_85/base_link"




def setup_scene( target_prim_path,simulation_app):
    world=World()
    stage = world.stage
    
    add_reference_to_stage(URL_GRIPPER, ARTICULATION_ROOT_PATH)
    for _ in range(20):
        simulation_app.update()
    gripper_prim = stage.GetPrimAtPath(ARTICULATION_ROOT_PATH)
    if not gripper_prim.IsValid():
        print(f"ERRORE FATALE: Impossibile trovare il prim del gripper a '{ARTICULATION_ROOT_PATH}'."); simulation_app.close(); return None, None
    xform_api = UsdGeom.Xformable(gripper_prim)
    xform_api.AddScaleOp().Set(Gf.Vec3f(SCALE_FACTOR_GRIP, SCALE_FACTOR_GRIP, SCALE_FACTOR_GRIP))
    print(f"✓ Gripper caricato e scalato di un fattore {SCALE_FACTOR_GRIP} a '{ARTICULATION_ROOT_PATH}'")

    apply_friction_to_gripper(stage, static_friction=200.0, dynamic_friction=200.0)

    robot = world.scene.add(Articulation(prim_path=ARTICULATION_ROOT_PATH, name="robotiq_gripper_system"))
    target_prim = stage.GetPrimAtPath(target_prim_path)
   
    return robot,target_prim




def _get_obb_info(obj: Usd.Prim) -> dict:
    """
    Calcola le informazioni dell'Oriented Bounding Box (OBB) di un prim
    nello spazio globale. Questa versione gestisce correttamente le scale non uniformi
    derivando gli assi e la scala direttamente dalla matrice di trasformazione.
    """
    time = Usd.TimeCode.Default()
    
    # 1. Calcola il bounding box locale (non trasformato)
    bbox_cache = UsdGeom.BBoxCache(time, [UsdGeom.Tokens.default_], useExtentsHint=True)
    untransformed_bbox = bbox_cache.ComputeUntransformedBound(obj)
    local_range = untransformed_bbox.GetRange()
    local_size = np.array(local_range.GetSize())
    
    # Gestione di oggetti con dimensione nulla
    if np.any(local_size < 1e-6):
        print(f"ATTENZIONE: Il prim {obj.GetPath()} ha una dimensione locale quasi nulla.")
        xformable = UsdGeom.Xformable(obj)
        world_transform_mtx = xformable.ComputeLocalToWorldTransform(time)
        center = np.array(world_transform_mtx.ExtractTranslation())
        # Restituisce una OBB di dimensione zero con orientamento standard
        return {
            "obb_center": center, 
            "obb_size": np.zeros(3), 
            "obb_axes": [np.array([1,0,0]), np.array([0,1,0]), np.array([0,0,1])]
        }

    # 2. Ottieni la matrice di trasformazione completa da locale a globale
    xformable = UsdGeom.Xformable(obj)
    world_transform_mtx = xformable.ComputeLocalToWorldTransform(time)
    
    # Il centro è semplicemente la componente di traslazione della matrice
    center = np.array(world_transform_mtx.ExtractTranslation())
    
    # Estrai la sottomatrice 3x3 che contiene rotazione e scala
    m3x3 = world_transform_mtx.ExtractRotationMatrix()
    
    # 3. Estrai scala e assi dalle colonne della matrice 3x3
    
    # Le colonne della matrice sono i vettori base trasformati
    col0 = m3x3.GetColumn(0)
    col1 = m3x3.GetColumn(1)
    col2 = m3x3.GetColumn(2)
    
    # La scala lungo ogni asse è la lunghezza (norma) del vettore colonna corrispondente
    scale_x = col0.GetLength()
    scale_y = col1.GetLength()
    scale_z = col2.GetLength()
    world_scale = np.array([scale_x, scale_y, scale_z])
    
    # Gli assi orientati nello spazio globale sono i vettori colonna normalizzati
    axes = [
        np.array(col0.GetNormalized()),
        np.array(col1.GetNormalized()),
        np.array(col2.GetNormalized())
    ]
    
    # La dimensione globale dell'OBB è la dimensione locale moltiplicata per la scala globale
    world_size = local_size * world_scale
    
    # (Opzionale) Stampa di debug per verifica
    # print(f"--- OBB Info per {obj.GetPath()} (Metodo Robusto) ---")
    # print(f" 	Centro Globale: {center}")
    # print(f" 	Scala Globale: {world_scale}")
    # print(f" 	Dimensione Globale: {world_size}")
    # print("-------------------------------------------------")
    
    return {"obb_center": center, "obb_size": world_size, "obb_axes": axes}





def _create_orientation_from_vectors(pos, center, long_axis):
    x = center - pos
    if np.linalg.norm(x) > 1e-6: x /= np.linalg.norm(x)
    y = np.cross(long_axis, x)
    if np.linalg.norm(y) < 1e-6:
        temp_up = np.array([0.0, 0.0, 1.0]) if not np.allclose(long_axis, [0,0,1]) else np.array([0.0, 1.0, 0.0])
        y = np.cross(temp_up, x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y); z /= np.linalg.norm(z)
    m = Gf.Matrix3d(1); m.SetRow(0, Gf.Vec3d(*x)); m.SetRow(1, Gf.Vec3d(*y)); m.SetRow(2, Gf.Vec3d(*z))
    q = m.ExtractRotation().GetQuat()
    return np.array([q.GetReal(), *q.GetImaginary()])


def generate_grasp_poses(obj, n=NUM_CANDIDATE_POSES, extra=GRIPPER_APPROACH_OFFSET):
    obb = _get_obb_info(obj); center, size, axes = obb["obb_center"], obb["obb_size"], obb["obb_axes"]
    long_axis = axes[int(np.argmax(size))]; dist = np.linalg.norm(size) * 0.5 + extra
    poses, tries = [], 0
    while len(poses) < n and tries < n * 1000:
        tries += 1
        phi, costheta = np.random.uniform(0, 2 * np.pi), np.random.uniform(-1, 1); theta = np.arccos(costheta)
        dir_vec = np.array([np.sin(theta) * np.cos(phi), np.sin(theta) * np.sin(phi), np.cos(theta)])
        pos = center + dir_vec * dist
        if pos[2] < MIN_SAFE_HEIGHT_Z: continue
        poses.append((pos.astype(float), _create_orientation_from_vectors(pos, center, long_axis).astype(float)))
    return poses






def ray_obb_intersection(ray_origin, ray_dir, obb_center, obb_size, obb_axes):
    t_min, t_max = -np.inf, np.inf; p = obb_center - ray_origin
    for i in range(3):
        axis = obb_axes[i]; e = np.dot(p, axis); f = np.dot(ray_dir, axis)
        if abs(f) > 1e-6:
            t1, t2 = (e + obb_size[i] / 2.0) / f, (e - obb_size[i] / 2.0) / f
            if t1 > t2: t1, t2 = t2, t1
            t_min, t_max = max(t_min, t1), min(t_max, t2)
            if t_min > t_max or t_max < 0: return None
        elif -e - obb_size[i] / 2.0 > 0 or -e + obb_size[i] / 2.0 < 0: return None
    return t_min if t_min > 0 else t_max


def apply_friction_to_gripper(stage, static_friction=2.0, dynamic_friction=1.5, restitution=0.0):
    """
    Trova il materiale fisico esistente del gripper, ne modifica le proprietà di attrito
    e si assicura che sia applicato ai collider delle dita.

    Args:
        stage (Usd.Stage): Lo stage USD corrente.
        static_friction (float): Nuovo coefficiente di attrito statico.
        dynamic_friction (float): Nuovo coefficiente di attrito dinamico.
        restitution (float): Nuovo coefficiente di restituzione.
    """
    # 1. Percorso del materiale fisico esistente fornito dall'utente
    material_path = "/World/Robotiq_2F_85/Robotiq_2F_85/PhysicsMaterial"
    
    # 2. Ottieni il prim del materiale e modificane le proprietà
    material_prim = stage.GetPrimAtPath(material_path)
    
    if not material_prim.IsValid():
        print(f"ERRORE: Impossibile trovare il materiale fisico esistente a '{material_path}'.")
        return

    # Applica l'API del materiale fisico per poterlo modificare
    physics_material = UsdPhysics.MaterialAPI.Apply(material_prim)
    physics_material.CreateStaticFrictionAttr().Set(static_friction)
    physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    physics_material.CreateRestitutionAttr().Set(restitution)
    print(f"✓ Materiale fisico esistente a '{material_path}' modificato: attrito statico={static_friction}, dinamico={dynamic_friction}")

    # 3. Identifica i prim dei collider delle dita (con i percorsi corretti)
    #    Questi percorsi sono stati corretti in base al tuo output di errore precedente.
    finger_collider_paths = [
        f"{ARTICULATION_ROOT_PATH}/left_inner_finger_pad/Collision",
        f"{ARTICULATION_ROOT_PATH}/right_inner_finger_pad/Collision"
    ]

    # 4. Applica il materiale ai collider delle dita
    for path in finger_collider_paths:
        collider_prim = stage.GetPrimAtPath(path)
        if collider_prim.IsValid():
            # Assicura che il prim abbia la CollisionAPI
            UsdPhysics.CollisionAPI.Apply(collider_prim)
            
            # Crea la relazione per associare il materiale fisico
            binding_api = UsdPhysics.MaterialBindingAPI.Apply(collider_prim)
            binding_api.CreatePhysicsMaterialBindingRel().SetTargets([Sdf.Path(material_path)])
            print(f"  -> Materiale '{material_path}' applicato con successo a '{path}'")
        else:
            print(f"ATTENZIONE: Impossibile trovare il prim del collider a '{path}'.")

class State(Enum):
    """Definisce gli stati della FSM per il processo di presa."""
    NEXT_POSE = auto()
    PAUSE = auto()
    APPROACH = auto()
    GRASP = auto()
    LIFT = auto()
    FINISH = auto()


class GraspResult(Enum):
    """Definisce i possibili risultati di un tentativo di presa."""
    PENDING = auto()  # Il tentativo è in corso
    SUCCESS = auto()  # La presa è riuscita
    FAILURE = auto()  # La presa è fallita per qualsiasi motivo


# ------------------------------------------------------------------------------
# --- CLASSE GRASPING FSM ---
# ------------------------------------------------------------------------------
class GraspingFSM:
    """
    Una FSM autonoma che esegue un ciclo di prese, partendo dal presupposto 
    di fallimento e cercando di raggiungere il successo, con reset di stabilità
    per prese più affidabili.
    """


    def __init__(self, robot: Articulation, target_prim: Usd.Prim, poses: list, obb_info: dict, stage_utils):
        # --- Componenti e Parametri ---
        self.world = World()
        self.robot = robot
        self.target_prim = target_prim
        self.poses = poses if poses else []
        self.obb_info = obb_info
        self.stage_utils = stage_utils
       
        # --- Stato Iniziale FSM ---
        self.state = State.NEXT_POSE if self.poses else State.FINISH
        self._pose_idx = -1


        # --- Tracciamento dei Risultati ---
        self.current_result = GraspResult.FAILURE # Default per l'esperimento
        self.results_history = []
        self._gripper_made_contact = False
        self.grip_position=None
        self.grip_orientation=None


        # --- Variabili di Supporto ---
        self._action_start_time = None
        self._lift_target_z = None
        self._pause_timer = 0
        self._next_state_after_pause = None
       
        # --- Configurazione Fisica ---
        physics_dt = self.world.get_physics_dt()
        self._pause_frames = int(PAUSE_DURATION_SEC / physics_dt) if physics_dt > 0 else 15


        # --- Inizializzazione Robot ---
        self.robot.disable_gravity()
        self._initialize_joints()
        
        
        # --- Dispatcher di Stato ---
        self._state_handlers = {
            s: getattr(self, f"_handle_{s.name.lower()}") for s in State if s != State.FINISH
        }


    def get_results_history(self) -> list:
        return self.results_history


    def get_result(self) -> GraspResult:
        return self.current_result


    def is_finished(self) -> bool:
        return self.state == State.FINISH


    def step(self):
        if not self.is_finished():
            handler = self._state_handlers.get(self.state)
            if handler:
                handler()


    # --------------------------------------------------------------------------
    # Metodi di Controllo e Logica Interna
    # --------------------------------------------------------------------------
    
    def _initialize_joints(self):
        try:
            dof_names = self.robot.dof_names
            self.leader_joint_idx = dof_names.index("finger_joint")
            self.follower_joint_idx = dof_names.index("right_outer_knuckle_joint")
        except (ValueError, IndexError) as e:
            print(f"ERRORE FSM in inizializzazione giunti: {e}")
            self._transition_to(State.FINISH, use_pause=False)


    def _transition_to(self, new_state: State, use_pause: bool = True):
        self._action_start_time = None
        if use_pause and new_state != State.FINISH:
            self.state = State.PAUSE; self._pause_timer = 0
            self._next_state_after_pause = new_state
        else:
            self.state = new_state


    # --------------------------------------------------------------------------
    # Metodi Gestori di Stato (_handle_*)
    # --------------------------------------------------------------------------


    def _handle_next_pose(self):
        """Archivia il risultato precedente e prepara il nuovo tentativo."""
        if self._pose_idx >= 0:
            self.results_history.append(self.current_result)
            print(f"--- Risultato Posa {self._pose_idx + 1}/{len(self.poses)}: {self.current_result.name} ---")


        self._pose_idx += 1
        
        if self._pose_idx >= len(self.poses):
            print("\n== Tutti i tentativi sono stati completati. ==")
            self._transition_to(State.FINISH, use_pause=False); return


        print(f"\n--- Inizio Tentativo di Presa {self._pose_idx + 1}/{len(self.poses)} ---")
        self.current_result = GraspResult.FAILURE
        self._gripper_made_contact = False
        
        initial_pos = np.zeros(self.robot.num_dof)
        initial_pos[self.leader_joint_idx] = GRIPPER_OPEN_POS
        self.robot.set_joint_positions(positions=initial_pos)
        pos, quat = self.poses[self._pose_idx]
        self.robot.set_world_pose(position=pos, orientation=quat)
        
        ### STABILITÀ ###
        # Azzera completamente qualsiasi moto residuo dal tentativo precedente
        self.robot.set_linear_velocity(np.zeros(3))
        self.robot.set_angular_velocity(np.zeros(3))
       
        self._transition_to(State.APPROACH, use_pause=False)


    def _handle_pause(self):
        self._pause_timer += 1
        if self._pause_timer >= self._pause_frames:
            self.state = self._next_state_after_pause; self._action_start_time = None
    
    def _handle_approach(self):
        gripper_pos, _ = self.robot.get_world_pose()
        ray_dir_vec = self.obb_info["obb_center"] - gripper_pos
        if np.linalg.norm(ray_dir_vec) < 1e-6: self._transition_to(State.GRASP, use_pause=False); return
        
        ray_dir = ray_dir_vec / np.linalg.norm(ray_dir_vec)
        dist_to_box = ray_obb_intersection(gripper_pos, ray_dir, **self.obb_info)

        #anche quando mi avvicino tengo la pinza sempre aperta
        initial_pos = np.zeros(self.robot.num_dof)
        initial_pos[self.leader_joint_idx] = GRIPPER_OPEN_POS
        self.robot.set_joint_positions(positions=initial_pos)
        
        if dist_to_box is not None and dist_to_box <= STOP_BEFORE_DIST:
            ### STABILITÀ ###
            # Momento critico 1: Fermati completamente prima di tentare la presa
            print("\n  -> Posizione di pre-presa raggiunta. Stabilizzazione...")
            self.robot.set_linear_velocity(np.zeros(3))
            self.robot.set_angular_velocity(np.zeros(3))
            self._transition_to(State.GRASP)
        else: 
            self.robot.set_linear_velocity(ray_dir * APPROACH_SPEED)


    def _handle_grasp(self):

        self.grip_position,self.grip_orientation =self.robot.get_world_pose()
        
        target_positions = np.zeros(self.robot.num_dof)
        target_positions[self.leader_joint_idx] = GRIPPER_CLOSED_POS
        target_positions[self.follower_joint_idx] = -GRIPPER_CLOSED_POS
        self._handle_joint_action(target_positions, "Chiusura", State.LIFT)
        self.robot.set_linear_velocity(np.zeros(3))
        self.robot.set_angular_velocity(np.zeros(3))


    def _handle_lift(self):
        if not self._gripper_made_contact:
            print("\n  -> Fallimento: Presa a vuoto (il gripper non ha toccato nulla).")
            self._transition_to(State.NEXT_POSE); return
            
        obj_xform = UsdGeom.Xformable(self.target_prim)
        obj_transform = obj_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        obj_pos = np.array(obj_transform.ExtractTranslation())
        gripper_pos, _ = self.robot.get_world_pose()
        
        object_diagonal = np.linalg.norm(self.obb_info["obb_size"])
        max_allowed_distance = object_diagonal / 2.0 + 0.5


        current_distance = np.linalg.norm(gripper_pos - obj_pos)


        if current_distance > max_allowed_distance:
            print(f"\n!!!--- Fallimento: Oggetto perso! Distanza: {current_distance:.2f}m > Soglia: {max_allowed_distance:.2f}m ---!!!")
            self._transition_to(State.NEXT_POSE); return
        
        if self._action_start_time is None:
            ### STABILITÀ ###
            # Momento critico 2: Azzera ogni rotazione residua dalla presa prima di sollevare
            print("  -> Presa riuscita. Stabilizzazione prima del sollevamento...")
            self.robot.set_angular_velocity(np.zeros(3))
            
            self._action_start_time = self.world.current_time
            self._lift_target_z = gripper_pos[2] + LIFT_HEIGHT_Z
           
        if self.robot.get_world_pose()[0][2] >= self._lift_target_z:
            print("\n  -> SUCCESSO: Sollevamento completato con oggetto in mano.")
            self.current_result = GraspResult.SUCCESS
            self._transition_to(State.NEXT_POSE)
        else:
            self.robot.set_linear_velocity(np.array([0, 0, LIFT_SPEED]))
            target_joint_positions = np.zeros(self.robot.num_dof)
            target_joint_positions[self.leader_joint_idx] = GRIPPER_CLOSED_POS
            target_joint_positions[self.follower_joint_idx] = -GRIPPER_CLOSED_POS
            self.robot.apply_action(ArticulationAction(joint_positions=target_joint_positions))


    def _handle_joint_action(self, target_positions: np.ndarray, action_name: str, next_state: State):
        if self._action_start_time is None:
            self.robot.apply_action(ArticulationAction(joint_positions=target_positions))
            self._action_start_time = self.world.current_time; return


        cur_pos = self.robot.get_joint_positions()
        if cur_pos is None: self._transition_to(State.NEXT_POSE); return


        is_fully_closed = np.allclose(cur_pos, target_positions, atol=GRIPPER_TOLERANCE)
        is_timed_out = (self.world.current_time - self._action_start_time) > GRIP_TIMEOUT_SEC
        
        if not is_fully_closed: self._gripper_made_contact = True


        if is_fully_closed or is_timed_out:
            if is_fully_closed: self._gripper_made_contact = False
            self.robot.apply_action(ArticulationAction(joint_positions=cur_pos))
            self._transition_to(next_state)




