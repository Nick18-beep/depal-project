# utils/scene_setup_utils.py
import os
import random
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd
import typing as T

# Funzioni helper per la configurazione e la creazione di elementi della scena.

def setup_new_scene_for_image(simulation_app, omni_usd, img_number, total_images):
    """Crea una nuova scena USD per l'immagine corrente."""
    try:
        from isaacsim.core.utils.stage import create_new_stage
    except ImportError:
        from omni.isaac.core.utils.stage import create_new_stage 
    if not create_new_stage():
        raise RuntimeError("Fallimento nella creazione di una nuova scena.")
    simulation_app.update()
    print(f"Nuova scena creata (Immagine {img_number}/{total_images}).")
    stage = omni_usd.get_context().get_stage()
    if not stage:
        raise RuntimeError("Fallimento nell'ottenere lo stage USD.")
    return stage

def create_scene_materials(stage, simulation_app, material_creator_module, texture_dir_path, base_material_usd_path_str,material_cfg):
    """Crea materiali PBR da texture e restituisce i componenti e i percorsi."""
    print(f"Creazione materiali da '{texture_dir_path}' in '{base_material_usd_path_str}'.")
    components = material_creator_module.crea_materiali_da_cartella_texture(
        stage=stage, simulation_app_instance=simulation_app,
        texture_dir_path=texture_dir_path, base_material_usd_path=base_material_usd_path_str,
        material_config=material_cfg
    )
    prim_paths = [c[0].GetPath().pathString for c in components if c[0] and c[0].IsValid()]
    if not components: print(f"AVVISO: Nessun materiale creato da '{texture_dir_path}'.")
    else: print(f"Creati {len(components)} materiali PBR. {len(prim_paths)} percorsi prim validi disponibili.")
    return components, prim_paths

def setup_scene_floor(stage, scene_creator_module, floor_prim_path, all_pbr_components):
    """Aggiunge un pavimento alla scena e applica un materiale PBR casuale se disponibile."""
    floor_material_comp = None
    if all_pbr_components:
        floor_material_comp = random.choice(all_pbr_components)
        if not (floor_material_comp and floor_material_comp[0] and floor_material_comp[0].IsValid()):
            floor_material_comp = None
    
    print(f"Aggiunta pavimento a '{floor_prim_path}'.")
    floor_obj = scene_creator_module.aggiungi_pavimento_con_materiale_esistente(
        stage=stage, prim_path=floor_prim_path, pbr_material_components_da_applicare=floor_material_comp
    )
    if floor_obj:
        print(f"Pavimento '{floor_obj.prim_path}' aggiunto.")
        if floor_material_comp: print(f"Materiale '{floor_material_comp[0].GetPath().pathString}' assegnato al pavimento.")
    else: print("ERRORE: Fallimento nell'aggiunta del pavimento.")
    return floor_obj

def setup_scene_lighting(stage, light_creator_module, light_cfg, light_base_path):
    """Istanzia luci casuali nella scena basate sulla configurazione."""
    print(f"Creazione luci casuali in '{light_base_path}'.")
    lights = light_creator_module.spawn_variable_random_lights(
        stage=stage, base_path=light_base_path, num_lights_config=tuple(light_cfg['num_lights_range']),
        light_options=light_cfg['configurations'], clear_existing=light_cfg['clear_existing']
    )
    if lights: print(f"Create {len(lights)} luci.")
    else: print("Nessuna luce creata.")
    return lights

def spawn_main_asset(stage, container_pallet_creator_module, asset_cfg, parent_path_str, base_pos_np, materials_folder_str, img_num):
    """Istanzia un pallet o un container basato sulla configurazione."""
    choice = asset_cfg.get('asset_type_choice_override') or random.choice(["pallet", "container"])
    details = asset_cfg[choice]
    prim_name = f"{details['prim_name_instance_base']}"
    scale = np.array(details['scale_multiplier_xyz']) * asset_cfg['cm_to_m_scale_factor']
    rot_z = random.uniform(*asset_cfg['random_asset_rotation_z_range'])
    
    print(f"Istanziazione {choice} '{prim_name}', usando la cartella materiali '{materials_folder_str}'.")
    spawned_prim = container_pallet_creator_module.spawn_single_pallet(
        stage=stage, usd_asset_paths=details['usd_asset_paths'], parent_path=parent_path_str,
        prim_name=prim_name, position=base_pos_np, orientation_euler_degrees=(0.0, 0.0, rot_z),
        scale=scale, mass=details['mass_kg'], semantic_label=details['semantic_label'],
        available_material_paths=materials_folder_str, 
        material_application_probability=asset_cfg['asset_material_application_probability']
    )
    if spawned_prim: print(f"Asset '{spawned_prim.GetPath()}' istanziato.")
    else: print(f"Fallimento istanziazione asset '{prim_name}'.")
    return spawned_prim

def spawn_additional_ycb_objects(stage, object_creator_module, ycb_cfg):
    """Istanzia oggetti YCB aggiuntivi se abilitato nella configurazione."""
    if not ycb_cfg.get('enable', False): return []
    if not ycb_cfg.get('asset_list'):
        print("AVVISO: Lista asset YCB vuota, salto istanziazione YCB.")
        return []
    
    mats_folder = ycb_cfg.get('materials_folder_path', "/World/Looks")
    print(f"Istanziazione oggetti YCB, usando la cartella materiali '{mats_folder}'.")
    prims = object_creator_module.spawn_objects(
        stage=stage, num_to_spawn_range=ycb_cfg['num_to_spawn_range'], usd_asset_paths=ycb_cfg['asset_list'],
        parent_path=ycb_cfg['spawn_parent_path'], base_position=np.array(ycb_cfg['base_pos_xy']),
        base_z_offset=ycb_cfg['spawn_z_offset'], z_jitter=ycb_cfg['spawn_z_jitter'],
        xy_jitter_range=tuple(ycb_cfg['spawn_xy_jitter_range']),
        asset_scale_min=ycb_cfg['spawn_scale_min'], asset_scale_max=ycb_cfg['spawn_scale_max'],
        object_mass=ycb_cfg['spawn_mass'], 
        semantic_label=ycb_cfg['semantic_label'], 
        change_material_probability=ycb_cfg['spawn_material_prob'], 
        existing_materials_path=mats_folder
    )
    if prims: print(f"Istanziati {len(prims)} oggetti YCB.")
    else: print("Nessun oggetto YCB istanziato.")
    return prims

def spawn_boxes_on_scene(stage, box_creator_module, Gf_module, box_cfg, parent_path_str, base_pos_np, direct_mat_paths_list):
    """Istanzia box di base basati sulla configurazione."""
    boxes_base_pos = np.array([base_pos_np[0], base_pos_np[1], base_pos_np[2] + box_cfg['base_z_offset']])
   
    if direct_mat_paths_list: print(f"Lo spawner di scatole ha ricevuto {len(direct_mat_paths_list)} percorsi materiali diretti.")

    spawned = box_creator_module.spawn_basic_boxes(
        stage=stage, num_to_spawn_range=box_cfg['num_to_spawn_range'], parent_path=parent_path_str,
        base_position=boxes_base_pos, base_z_offset=0, 
        z_jitter=box_cfg['z_jitter'],
        xy_jitter_range=tuple(box_cfg['xy_jitter_range']),
        box_asset_paths=box_cfg['asset_usd_paths'],
        box_scale_min=box_cfg['scale_min'], box_scale_max=box_cfg['scale_max'],
        box_mass=box_cfg['mass_kg'], default_color=Gf_module.Vec3f(box_cfg['default_color_rgb']),
        semantic_label=box_cfg['semantic_label'],
        available_pbr_material_paths=direct_mat_paths_list
    )
    if spawned: print(f"Istanziate {len(spawned)} scatole.")
    else: print("Nessuna scatola istanziata.")
    return spawned

def setup_main_camera(stage, prims_utils_module, Gf_module, UsdGeom_module, cam_cfg, cam_path_str, scene_origin_np):
    """Crea e configura la camera principale di Replicator con altezza Z variabile."""
    cam_height_min = cam_cfg.get('height_min', 5.0)
    cam_height_max = cam_cfg.get('height_max', 7.0)
    if cam_height_min > cam_height_max:
        cam_height_max = cam_height_min
    
    random_cam_height = random.uniform(cam_height_min, cam_height_max)
    
    pos = Gf_module.Vec3f(scene_origin_np[0], scene_origin_np[1], random_cam_height)
    rot = Gf_module.Vec3f(cam_cfg['rotation_xyz'])
    print(f"Setup camera a '{cam_path_str}' con altezza Z casuale: {random_cam_height:.2f}m.")
    cam = prims_utils_module.create_prim(
        cam_path_str, prim_type="Camera", position=pos,
        attributes={"focalLength": cam_cfg['focal_length']}
    )
    if cam: UsdGeom_module.XformCommonAPI(cam).SetRotate(rot, UsdGeom_module.XformCommonAPI.RotationOrderXYZ)
    else: print(f"ERRORE: Fallimento creazione camera a '{cam_path_str}'.")
    return cam




# ================================================================
#  scene_setup_utils.py
#  Pareti invisibili allineate al container / pallet principale
# ================================================================
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import typing as T
import math

# -----------------------------------------------------------------
# 0.  estrai scala da Matrix4d – compatibile USD 22-24
# -----------------------------------------------------------------
def _extract_scale(mat: Gf.Matrix4d) -> Gf.Vec3d:
    """Restituisce (sx, sy, sz) anche se Matrix4d non ha ExtractScale()."""
    if hasattr(mat, "ExtractScale"):                      # USD ≥ 23
        return mat.ExtractScale()

    # USD 22 – lunghezza dei vettori colonna
    c0 = Gf.Vec3d(mat[0][0], mat[0][1], mat[0][2])
    c1 = Gf.Vec3d(mat[1][0], mat[1][1], mat[1][2])
    c2 = Gf.Vec3d(mat[2][0], mat[2][1], mat[2][2])
    return Gf.Vec3d(c0.GetLength(), c1.GetLength(), c2.GetLength())


# -----------------------------------------------------------------
# helper: scala locale del SOLO prim container
# -----------------------------------------------------------------
def _local_scale(prim: Usd.Prim) -> Gf.Vec3d:
    """
    Ritorna la scala (sx, sy, sz) definita sugli xformOp del prim stesso,
    ignorando i trasform parent e i figli. Compatibile USD 22-24.
    """
    xf = UsdGeom.Xformable(prim)

    # USD ≥ 23: utilizza direttamente l’API comoda
    if hasattr(xf, "ComputeLocalToParentTransform"):
        mat_loc = xf.ComputeLocalToParentTransform(Usd.TimeCode.Default())
        return _extract_scale(mat_loc)

    # ---------- Fallback USD 22 -----------------------------------
    xcache    = UsdGeom.XformCache()
    mat_world = xcache.GetLocalToWorldTransform(prim)          # T · R · S del prim
    parent    = prim.GetParent()
    if parent and parent.IsValid():
        mat_parent = xcache.GetLocalToWorldTransform(parent)   # T · R · S del padre
        mat_loc    = mat_world * Gf.Matrix4d(mat_parent).GetInverse()  # locale
    else:
        mat_loc = mat_world                                    # prim è root

    return _extract_scale(mat_loc)

# -----------------------------------------------------------------
# 1.  trova il container / pallet
# -----------------------------------------------------------------
def _find_main_asset_prim(stage: Usd.Stage) -> str:
    for p in ("/World/ImportedPallet", "/World/ImportedContainer"):
        if stage.GetPrimAtPath(p).IsValid():
            return p

    for prim in stage.Traverse():
        if prim.HasAttribute("semanticLabel") and \
           prim.GetAttribute("semanticLabel").Get() in ("pallet", "container"):
            return prim.GetPath().pathString

    raise RuntimeError("Container / pallet non trovato nello stage.")


# -----------------------------------------------------------------
# 2.  bound locale (dimensioni, centro) – niente TRS del container
# -----------------------------------------------------------------
def _bbox_local(prim: Usd.Prim) -> T.Tuple[Gf.Vec3d, Gf.Vec3d]:
    tok_def = (UsdGeom.Tokens.default
               if hasattr(UsdGeom.Tokens, "default")
               else UsdGeom.Tokens.default_)
    cache   = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [tok_def])
    rng     = cache.ComputeUntransformedBound(prim).GetRange()
    mn, mx  = rng.GetMin(), rng.GetMax()
    return mx - mn, (mn + mx) * 0.5      # (size, center) locali


# -----------------------------------------------------------------
# 3.  utilities fisica + invisibilità
# -----------------------------------------------------------------
def _add_physics(prim):
    prim = prim.GetPrim() if not isinstance(prim, Usd.Prim) else prim
    #UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
    UsdPhysics.CollisionAPI.Apply(prim)


def _hide(prim):
    img = UsdGeom.Imageable(prim)
    img.CreateVisibilityAttr().Set("invisible")
    gp = UsdGeom.Gprim(prim)
    gp.CreateDisplayColorAttr().Set([(0.0, 0.0, 0.0)])
    gp.CreateDisplayOpacityAttr().Set([0.0])


def _quat_to_euler_xyz(q: Gf.Quatd) -> Gf.Vec3d:
    """Converte quaternion → Eulero XYZ (°)."""
    w, x, y, z = q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]
    # roll (X)
    sinr = 2*(w*x + y*z); cosr = 1 - 2*(x*x + y*y)
    roll = math.degrees(math.atan2(sinr, cosr))
    # pitch (Y)
    sinp = 2*(w*y - z*x)
    pitch = math.degrees(math.asin(max(-1, min(1, sinp))))
    # yaw (Z)
    siny = 2*(w*z + x*y); cosy = 1 - 2*(y*y + z*z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return Gf.Vec3d(roll, pitch, yaw)

def _local_orientation(prim: Usd.Prim) -> Gf.Vec3d:
    """
    Restituisce orientamento locale (Euler XYZ in °).
    Priorità: xformOp:orient (quatd) → rotateXYZ → (0,0,0).
    """
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            q = op.Get()                                   # Gf.Quatd
            return _quat_to_euler_xyz(q)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            r = op.Get()
            return Gf.Vec3d(*r)
    return Gf.Vec3d(0, 0, 0)
# -----------------------------------------------------------------
# 4.  crea pareti invisibili
# -----------------------------------------------------------------
def spawn_invisible_walls(stage: Usd.Stage,
                          suffix="__walls",
                          thickness=0.001,
                          extra_height=2.8) -> T.List[str]:
    """
    Crea 4 pareti invisibili attorno al container, scalate e ruotate come
    il container, con base a pavimento.
    """
    tgt_path = _find_main_asset_prim(stage)
    tgt      = stage.GetPrimAtPath(tgt_path)

    scl  = _local_scale(tgt)             # scala locale (sx, sy, sz)
    size, centro = _bbox_local(tgt)           # dimensioni interne
    print("dimensioni: ",size,"  ",centro)
    size = Gf.Vec3d(size[0]*scl[0], size[1]*scl[1], size[2]*scl[2])

    # spessore locale per thickness reale
    t_lx = thickness / max(scl[0], 1e-8)
    t_ly = thickness / max(scl[1], 1e-8)

    wall_h = size[2] + extra_height
    sx2, sy2 = size[0]/2, size[1]/2
    t2x, t2y = t_lx/2, t_ly/2
    tz2 = wall_h/2                       # centro muriccio

    scales = [Gf.Vec3d(t2x, sy2, tz2), Gf.Vec3d(t2x, sy2, tz2),
              Gf.Vec3d(sx2, t2y, tz2), Gf.Vec3d(sx2, t2y, tz2)]
    
    offs = [ Gf.Vec3d( sx2+t2x, 0, 0), Gf.Vec3d(-sx2-t2x, 0, 0),
             Gf.Vec3d(0,  sy2+t2y, 0), Gf.Vec3d(0, -sy2-t2y, 0)]

    grp = UsdGeom.Xform.Define(stage, f"{tgt_path}{suffix}")  # figlio → eredita T R S

    wall_paths = []
    for i, (off, sc) in enumerate(zip(offs, scales)):
        p    = f"{grp.GetPath()}/wall_{i}"
        cube = UsdGeom.Cube.Define(stage, p)

         # 2) rotazione: copia l'orient (quat) o, se non c'è, il rotateXYZ
        xf_tgt = UsdGeom.Xformable(tgt)
        quat   = None
        for op in xf_tgt.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                quat = op.Get()                 # Gf.Quatd
                break
            if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                e = op.Get();                   # tuple/Vec3
                quat = Gf.Quatd().SetFromEulerZYX(Gf.Vec3d(e[0], e[1], e[2]))
                break
        if quat is not None:
            qf = Gf.Quatf(float(quat.GetReal()),
            Gf.Vec3f(*quat.GetImaginary()))
            UsdGeom.Xformable(cube).AddOrientOp().Set(qf)

        # 1) traslazione: base sul pavimento
        UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(off[0], off[1], tz2))
        
       

        # 3) scala (ricorda: Cube = 2 unità)
        UsdGeom.Xformable(cube).AddScaleOp().Set(sc)

        _add_physics(cube)
        wall_paths.append(p)

    # debug
    print("[DEBUG] scala locale container :", scl)
    print("[DEBUG] orientazione (XYZ°)     :", _local_orientation(tgt))
    print("[DEBUG] size scalate (L P H)    :", size)

    return wall_paths

# -----------------------------------------------------------------
# 5.  Disattiva / rimuove pareti
# -----------------------------------------------------------------
def disable_walls(stage: Usd.Stage, wall_paths, remove=True):
    """Disabilita (o elimina) i collider specificati."""
    for p in wall_paths:
        prim = stage.GetPrimAtPath(p)
        if not prim:
            continue
        if remove:
            stage.RemovePrim(p)
        else:
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
            UsdGeom.Imageable(prim).CreateVisibilityAttr().Set("invisible")
