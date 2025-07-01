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


NUM_CANDIDATE_POSES     = 3
GRIPPER_APPROACH_OFFSET = 1.5
MIN_SAFE_HEIGHT_Z       = 0.4
PAUSE_DURATION_SEC      = 0.25


APPROACH_SPEED          = 1
STOP_BEFORE_DIST        = 0.25


GRIPPER_OPEN_POS        = 0.0
GRIPPER_CLOSED_POS      = 0.80
GRIPPER_TOLERANCE       = 0.02
GRIP_TIMEOUT_SEC        = 2.0


LIFT_HEIGHT_Z           = 1
LIFT_SPEED              = 2


SCALE_FACTOR_GRIP       = 3.5


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
    nello spazio globale, tenendo conto di tutte le trasformazioni annidate.
    """
    time = Usd.TimeCode.Default()
    
    bbox_cache = UsdGeom.BBoxCache(time, [UsdGeom.Tokens.default_], useExtentsHint=True)
    untransformed_bbox = bbox_cache.ComputeUntransformedBound(obj)
    local_range = untransformed_bbox.GetRange()
    local_size = np.array(local_range.GetSize())
    
    if np.any(local_size < 1e-6):
        print(f"ATTENZIONE: Il prim {obj.GetPath()} ha una dimensione locale quasi nulla. Restituisco valori di default.")
        xformable = UsdGeom.Xformable(obj)
        world_transform_mtx = xformable.ComputeLocalToWorldTransform(time)
        center = np.array(world_transform_mtx.ExtractTranslation())
        return {"obb_center": center, "obb_size": np.zeros(3), "obb_axes": [np.array([1,0,0]), np.array([0,1,0]), np.array([0,0,1])]}


    xformable = UsdGeom.Xformable(obj)
    world_transform_mtx = xformable.ComputeLocalToWorldTransform(time)
    
    center = np.array(world_transform_mtx.ExtractTranslation())
    rotation_quat = world_transform_mtx.ExtractRotationQuat()
    
    m3x3 = world_transform_mtx.ExtractRotationMatrix()
    
    scale_x = Gf.Vec3d(m3x3[0][0], m3x3[1][0], m3x3[2][0]).GetLength()
    scale_y = Gf.Vec3d(m3x3[0][1], m3x3[1][1], m3x3[2][1]).GetLength()
    scale_z = Gf.Vec3d(m3x3[0][2], m3x3[1][2], m3x3[2][2]).GetLength()
    world_scale = np.array([scale_x, scale_y, scale_z])


    world_size = local_size * world_scale
    
    rotation_only_mtx = Gf.Matrix3d(rotation_quat)
    axes = [
        np.array(rotation_only_mtx * Gf.Vec3d.XAxis()),
        np.array(rotation_only_mtx * Gf.Vec3d.YAxis()),
        np.array(rotation_only_mtx * Gf.Vec3d.ZAxis())
    ]


    print(f"--- OBB Info per {obj.GetPath()} ---")
    print(f"  Centro Globale: {center}")
    print(f"  Scala Globale Calcolata: {world_scale}")
    print(f"  Dimensione Locale: {local_size}")
    print(f"  Dimensione Globale Calcolata: {world_size}")
    print("------------------------------------")
    
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
    while len(poses) < n and tries < n * 50:
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


class GraspingFSM:
    """
    Una Macchina a Stati Finiti (FSM) per gestire il processo di presa di un robot.
    Controlla il robot attraverso una sequenza di stati: avvicinamento, presa, sollevamento
    e passaggio alla posa successiva.
    """

    def __init__(self, robot: Articulation, poses: list, obb_info: dict, stage_utils):
        """
        Inizializza la FSM.

        Args:
            robot (Articulation): L'oggetto robot da controllare.
            poses (list): Una lista di pose (posizione, orientamento) che il robot deve raggiungere.
            obb_info (dict): Informazioni sull'Oriented Bounding Box (OBB) dell'oggetto da afferrare.
            load: Un parametro di carico (non utilizzato nel codice fornito, ma mantenuto nella firma).
        """
        # --- Componenti e Parametri Principali ---
        self.world = World()
        self.robot = robot
        self.poses = poses
        self.obb_info = obb_info

        self.stage_utils=stage_utils
        
        # --- Stato Iniziale della FSM ---
        self.state = State.APPROACH
        self._pose_idx = 0

        # --- Variabili di Supporto per Azioni e Pause ---
        self._action_start_time = None
        self._lift_target_z = None
        self._pause_timer = 0
        self._next_state_after_pause = None
        
        # --- Configurazione basata sulla Fisica ---
        physics_dt = self.world.get_physics_dt()
        self._pause_frames = int(PAUSE_DURATION_SEC / physics_dt) if physics_dt > 0 else 15

        # --- Inizializzazione del Robot ---
        self.robot.disable_gravity()
        self._initialize_joints()
        # --- Dispatcher per i Gestori di Stato ---
        # Associa ogni stato a un metodo gestore (_handle_STATO)
        # per evitare un lungo blocco if/elif/else nel metodo step().
        self._state_handlers = {
            s: getattr(self, f"_handle_{s.name.lower()}") for s in State if s != State.FINISH
        }

        #self.stage_utils.save_stage_with_pause_and_resume("stage_freeze_temp")


    def _initialize_joints(self):
        """
        Trova gli indici dei giunti del gripper e imposta la loro posizione iniziale (aperta).
        In caso di errore, la FSM passa direttamente allo stato FINISH.
        """
        try:
            dof_names = self.robot.dof_names
            self.leader_joint_idx = dof_names.index("finger_joint")
            self.follower_joint_idx = dof_names.index("right_outer_knuckle_joint")
            
            initial_pos = np.zeros(self.robot.num_dof)
            initial_pos[self.leader_joint_idx] = GRIPPER_OPEN_POS
            self.robot.set_joint_positions(positions=initial_pos)
        except (ValueError, IndexError) as e:
            print(f"ERRORE FSM in inizializzazione giunti: {e}")
            self.state = State.FINISH

    def step(self):
        """Esegue un singolo passo logico della FSM, invocando il gestore per lo stato corrente."""
        if not self.is_finished():
            handler = self._state_handlers.get(self.state)
            if handler:
                handler()

    def is_finished(self) -> bool:
        """Restituisce True se la FSM ha completato il suo ciclo."""
        return self.state == State.FINISH

    def _transition_to(self, new_state: State, use_pause: bool = True):
        """
        Gestisce la transizione a un nuovo stato, con una pausa intermedia opzionale.
        """
        self._action_start_time = None
        if use_pause:
            self.state = State.PAUSE
            self._pause_timer = 0
            self._next_state_after_pause = new_state
        else:
            self.state = new_state

    # --------------------------------------------------------------------------
    # Metodi Gestori di Stato (_handle_*)
    # --------------------------------------------------------------------------

    def _handle_next_pose(self):
        """
        Passa alla posa successiva nella lista. Se non ci sono più pose, termina.
        """
        self._pose_idx += 1
        if self._pose_idx >= len(self.poses):
            print("\nDemo terminata.")
            self.state = State.FINISH
            return

        
        # Resetta i giunti e teletrasporta il robot alla nuova posa
        print(f"\nPose {self._pose_idx + 1}/{len(self.poses)}: teletrasporto.")
        initial_pos = np.zeros(self.robot.num_dof)
        initial_pos[self.leader_joint_idx] = GRIPPER_OPEN_POS
        self.robot.set_joint_positions(positions=initial_pos)

        pos, quat = self.poses[self._pose_idx]
        self.robot.set_world_pose(position=pos, orientation=quat)
        self.robot.set_linear_velocity(np.zeros(3))
        self.robot.set_angular_velocity(np.zeros(3))
        
        self._transition_to(State.APPROACH)

    def _handle_pause(self):
        """
        Gestisce lo stato di pausa, attendendo per un numero predefinito di frame.
        """
       
        self._pause_timer += 1
        if self._pause_timer >= self._pause_frames:
            print(f"  -> Pausa terminata. Prossimo stato: {self._next_state_after_pause.name}")
            self.state = self._next_state_after_pause
            self._action_start_time = None # Resetta il timer per la nuova azione
           

    def _handle_approach(self):
        """
        Gestisce l'avvicinamento del gripper all'oggetto target.
        """
        print("  -> In avvicinamento...", end='\r')
        gripper_pos, _ = self.robot.get_world_pose()
        ray_dir_vec = self.obb_info["obb_center"] - gripper_pos
        dist_to_center = np.linalg.norm(ray_dir_vec)

        if dist_to_center < 1e-6: # Evita divisione per zero
            self.robot.set_linear_velocity(np.zeros(3))
            self._transition_to(State.GRASP)
            return

        ray_dir = ray_dir_vec / dist_to_center
        dist_to_box = ray_obb_intersection(gripper_pos, ray_dir, **self.obb_info)

        if dist_to_box is not None and dist_to_box <= STOP_BEFORE_DIST:
            print("\n  -> Posizione di pre-presa raggiunta.")
            self.robot.set_linear_velocity(np.zeros(3))
            self.robot.set_angular_velocity(np.zeros(3))
            self._transition_to(State.GRASP)
        else:
            self.robot.set_linear_velocity(ray_dir * APPROACH_SPEED)

    def _handle_grasp(self):
        """
        Gestisce la chiusura del gripper per afferrare l'oggetto.
        """
        target_positions = np.zeros(self.robot.num_dof)
        target_positions[self.leader_joint_idx] = GRIPPER_CLOSED_POS
        target_positions[self.follower_joint_idx] = -GRIPPER_CLOSED_POS
        self._handle_joint_action(target_positions, "Chiusura", State.LIFT)
        self.robot.set_linear_velocity(np.zeros(3))
        self.robot.set_angular_velocity(np.zeros(3))

    def _handle_lift(self):
        """
        Gestisce il sollevamento del robot (e dell'oggetto) a un'altezza predefinita.
        """
        current_pos, _ = self.robot.get_world_pose()
        
        if self._action_start_time is None:
            print("  -> Inizio sollevamento.")
            self._action_start_time = self.world.current_time
            self._lift_target_z = current_pos[2] + LIFT_HEIGHT_Z
            
        if current_pos[2] >= self._lift_target_z:
            print("\n  -> Sollevamento completato.")
            self.robot.set_linear_velocity(np.zeros(3))
            self._lift_target_z = None
            self._transition_to(State.NEXT_POSE)
        else:
            print(f"  -> In sollevamento... Z: {current_pos[2]:.2f}", end='\r')
            self.robot.set_linear_velocity(np.array([0, 0, LIFT_SPEED]))
        
        # --- INIZIO MODIFICHE ---
        
        # 1. Definisci la posizione di chiusura dei giunti
        target_joint_positions = np.zeros(self.robot.num_dof)
        target_joint_positions[self.leader_joint_idx] = GRIPPER_CLOSED_POS
        target_joint_positions[self.follower_joint_idx] = -GRIPPER_CLOSED_POS
        
        # 2. Definisci la velocità di sollevamento
        lift_velocity = np.array([0, 0, LIFT_SPEED])
        
        # 3. Applica ENTRAMBE le azioni (posizione giunti + velocità base) simultaneamente
        action = ArticulationAction(
            joint_positions=target_joint_positions,
           
        )
        self.robot.apply_action(action)

    # --------------------------------------------------------------------------
    # Metodi Helper
    # --------------------------------------------------------------------------
    
    def _handle_joint_action(self, target_positions: np.ndarray, action_name: str, next_state: State):
        """
        Helper generico per gestire un'azione sui giunti che richiede tempo per completarsi.
        Monitora il raggiungimento della posizione target o un timeout.

        Args:
            target_positions (np.ndarray): Le posizioni target dei giunti.
            action_name (str): Il nome dell'azione (es. "Chiusura") per i messaggi di log.
            next_state (State): Lo stato a cui passare al completamento.
        """
        if self._action_start_time is None:
            print(f"  -> Inizio {action_name.lower()}.")
            self.robot.apply_action(ArticulationAction(joint_positions=target_positions))
            self._action_start_time = self.world.current_time
            return

        cur_pos = self.robot.get_joint_positions()
        if cur_pos is None: return

        elapsed = self.world.current_time - self._action_start_time
        is_done = np.allclose(cur_pos, target_positions, atol=GRIPPER_TOLERANCE)
        is_timed_out = elapsed > GRIP_TIMEOUT_SEC

        if is_done or is_timed_out:
            status_icon = '⚠ TIMEOUT' if is_timed_out and not is_done else '✓'
            print(f"\n  -> {status_icon} {action_name} completata.")
            # Ferma il movimento mantenendo la posizione corrente
            self.robot.apply_action(ArticulationAction(joint_positions=cur_pos))
            self._transition_to(next_state)
        else:
            # Messaggio di stato in linea
            status_msg = (
                f"  -> In {action_name.lower()}... "
                f"L:{cur_pos[self.leader_joint_idx]:.2f} "
                f"R:{cur_pos[self.follower_joint_idx]:.2f} | T:{elapsed:.1f}s"
            )
            print(status_msg, end='\r')