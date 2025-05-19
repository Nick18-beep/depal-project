# src/simple_random_lights.py

import omni.usd
from pxr import UsdGeom, UsdLux, Gf, Sdf, Vt 
import random
import math # Per atan2 e altre funzioni se necessarie per look-at più complessi

# --- Funzioni Helper per Attributi Luce --- (invariate dalla versione precedente)

def _set_prim_transform(prim_path: str, position: Gf.Vec3d = None, orientation_quat: Gf.Quatf = None):
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("Errore: Stage non disponibile in _set_prim_transform.")
        return

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder() 

    if position is not None:
        xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(position)

    if orientation_quat is not None and orientation_quat != Gf.Quatf(1.0): 
        xformable.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(orientation_quat)


def _get_light_schema(prim):
    if not prim or not prim.IsValid():
        return None
    
    prim_type_name = prim.GetTypeName()
    if prim_type_name == "SphereLight": return UsdLux.SphereLight(prim)
    if prim_type_name == "RectLight": return UsdLux.RectLight(prim)
    if prim_type_name == "DistantLight": return UsdLux.DistantLight(prim)
    if prim_type_name == "DomeLight": return UsdLux.DomeLight(prim)
    
    if prim.HasAPI(UsdLux.LightAPI):
        return UsdLux.LightAPI(prim) 
    
    print(f"Avviso: Tipo di luce non specificamente gestito o sconosciuto: {prim_type_name} a {prim.GetPath()}.")
    return None


def _set_light_attribute_float(light_prim_path: str, attr_name: str, value: float):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(light_prim_path)
    if not prim or not prim.IsValid():
        print(f"Prim luce non valido a {light_prim_path} per attributo {attr_name}.")
        return
    
    light_schema = _get_light_schema(prim)
    if not light_schema:
        print(f"Impossibile ottenere schema luce per {light_prim_path} per impostare {attr_name}.")
        return

    attr_to_set = None
    method_name_get = f"Get{attr_name[0].upper()}{attr_name[1:]}Attr" 

    if hasattr(light_schema, method_name_get):
        attr_to_set = getattr(light_schema, method_name_get)()
    else:
        print(f"Attributo o metodo '{method_name_get}' non trovato sullo schema {type(light_schema).__name__} per {light_prim_path}.")

    if attr_to_set:
        try:
            attr_to_set.Set(float(value))
        except Exception as e:
            print(f"Errore durante l'impostazione dell'attributo float '{attr_name}' per {light_prim_path}: {e}")
    else:
        print(f"Attributo float '{attr_name}' non ottenuto per {light_prim_path}.")


def _set_light_attribute_color(light_prim_path: str, value: Gf.Vec3f):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(light_prim_path)
    if not prim or not prim.IsValid():
        print(f"Prim luce non valido a {light_prim_path} per attributo colore.")
        return

    light_schema = _get_light_schema(prim)
    if not light_schema:
        print(f"Impossibile ottenere schema luce per {light_prim_path} per impostare il colore.")
        return
    
    attr = light_schema.GetColorAttr() 
    if attr:
        attr.Set(value)
    else:
        print(f"Attributo colore non trovato tramite schema {type(light_schema).__name__} per {light_prim_path}, il che è inaspettato.")


# --- Funzione Principale Unica ---

def spawn_variable_random_lights(
    stage,
    base_path: str = "/World/VariableLights",
    num_lights_config: tuple = (1, 3), 
    light_options: list = None, 
    clear_existing: bool = True
):
    if not stage:
        print("Errore: Stage non fornito a spawn_variable_random_lights.")
        return []
    
    if not light_options:
        # Configurazione di default se nessuna è fornita (invariata)
        print("Avviso: 'light_options' non fornito. Uso default per 1-2 luci Distant/Sphere.")
        light_options = [
            {
                "type": "DistantLight",
                "aim_at_target": {"target_pos": (0,0,0), "offset_range": {"x":(-0.5,0.5), "y":(-0.5,0.5), "z":(-0.2,0.2)}},
                "intensity_range": (50000.0, 70000.0), "color_range": ((0.9,0.9,0.8), (1.0,1.0,1.0)), "angle_range": (0.5, 1.0)
            },
            {
                "type": "SphereLight", "position_range": {"x": (-3.0,3.0), "y": (-3.0,3.0), "z": (2.0,4.0)},
                "intensity_range": (30000.0, 60000.0), "color_range": ((0.8,0.8,1.0), (1.0,1.0,1.0)), "radius_range": (0.5, 1.0)
            }
        ]
        num_lights_config = (1,2)

    if clear_existing and stage.GetPrimAtPath(base_path):
        stage.RemovePrim(base_path)
        print(f"Rimosso prim base luci esistente: {base_path}")
    
    if not stage.GetPrimAtPath(base_path):
        UsdGeom.Xform.Define(stage, base_path) 
        print(f"Creato prim base per luci: {base_path}")

    num_to_spawn = random.randint(num_lights_config[0], num_lights_config[1])
    created_light_prims_info = []

    print(f"Tentativo di spawnare {num_to_spawn} luci casuali sotto {base_path}...")

    # Asse di default che la luce usa per "puntare" (comunemente -Z)
    LIGHT_FORWARD_AXIS = Gf.Vec3d(0, 0, -1) 

    for i in range(num_to_spawn):
        config = random.choice(light_options) 
        light_type_str = config["type"]
        prim_name = f"{light_type_str}_{i}"
        light_prim_path = f"{base_path}/{prim_name}"

        defined_light_schema = None
        if hasattr(UsdLux, light_type_str):
            LightClass = getattr(UsdLux, light_type_str)
            defined_light_schema = LightClass.Define(stage, light_prim_path) 
            if not defined_light_schema or not defined_light_schema.GetPrim().IsValid():
                print(f"Fallimento nella creazione del prim luce: {light_prim_path}")
                continue
        else:
            print(f"Tipo di luce UsdLux non supportato: {light_type_str}")
            continue
        
        print(f"Creato: {light_prim_path}")

        # Posizione della luce (rimane casuale come prima)
        light_pos = Gf.Vec3d(0.0,0.0,1.0) 
        if "position_range" in config:
            pr = config["position_range"]
            light_pos = Gf.Vec3d(
                random.uniform(*pr.get("x", (0.0,0.0))),
                random.uniform(*pr.get("y", (0.0,0.0))),
                random.uniform(*pr.get("z", (1.0,1.0))) 
            )
        
        orientation_quat_to_set = Gf.Quatf(1.0) # Default: nessuna rotazione

        # NUOVA LOGICA PER IL PUNTAMENTO
        aim_config = config.get("aim_at_target")
        # Le DomeLight non "puntano" nel senso tradizionale, quindi usano sempre euler_range se specificato
        if aim_config and light_type_str != "DomeLight":
            target_base_pos_tuple = aim_config.get("target_pos", (0.0, 0.0, 0.0))
            target_base_pos = Gf.Vec3d(target_base_pos_tuple[0], target_base_pos_tuple[1], target_base_pos_tuple[2])
            
            offset_cfg = aim_config.get("offset_range", {})
            offset = Gf.Vec3d(
                random.uniform(*offset_cfg.get("x", (0.0, 0.0))),
                random.uniform(*offset_cfg.get("y", (0.0, 0.0))),
                random.uniform(*offset_cfg.get("z", (0.0, 0.0)))
            )
            final_target_pos = target_base_pos + offset

            # Per DistantLight, la sua posizione non conta per l'illuminazione,
            # ma usiamo la 'light_pos' generata come punto da cui calcolare la direzione.
            # La direzione è da 'light_pos' a 'final_target_pos'.
            direction_vector_double = (final_target_pos - light_pos)
            
            if direction_vector_double.GetLength() > 1e-6: # Evita normalizzazione di vettore nullo
                direction_vector_double.Normalize()
                
                # Calcola la rotazione per allineare LIGHT_FORWARD_AXIS con direction_vector_double
                # Gf.Rotation(from_vec, to_vec)
                rotation_double = Gf.Rotation(LIGHT_FORWARD_AXIS, direction_vector_double)
                quat_double = rotation_double.GetQuat()
                
                orientation_quat_to_set = Gf.Quatf(
                    quat_double.GetReal(), 
                    Gf.Vec3f(quat_double.GetImaginary()[0], quat_double.GetImaginary()[1], quat_double.GetImaginary()[2])
                )
            else: # La luce è troppo vicina o sopra il target, usa orientamento di default
                print(f"Avviso: Luce a {light_pos} coincide o è troppo vicina al target {final_target_pos}. Uso orientamento di default.")
                orientation_quat_to_set = Gf.Quatf(1.0)

        elif "orientation_euler_range" in config: # Vecchia logica di rotazione casuale
            orr = config["orientation_euler_range"]
            rot_x_deg = random.uniform(*orr.get("x", (0.0,0.0)))
            rot_y_deg = random.uniform(*orr.get("y", (0.0,0.0)))
            rot_z_deg = random.uniform(*orr.get("z", (0.0,0.0)))
            
            rotation_obj_x = Gf.Rotation(Gf.Vec3d.XAxis(), rot_x_deg)
            rotation_obj_y = Gf.Rotation(Gf.Vec3d.YAxis(), rot_y_deg)
            rotation_obj_z = Gf.Rotation(Gf.Vec3d.ZAxis(), rot_z_deg)
            
            composed_rotation_double = rotation_obj_z * rotation_obj_y * rotation_obj_x
            quat_double = composed_rotation_double.GetQuat()
            
            orientation_quat_to_set = Gf.Quatf(
                quat_double.GetReal(), 
                Gf.Vec3f(quat_double.GetImaginary()[0], quat_double.GetImaginary()[1], quat_double.GetImaginary()[2])
            )
        
        _set_prim_transform(light_prim_path, position=light_pos, orientation_quat=orientation_quat_to_set)

        # Impostazione attributi (intensità, colore, specifici) rimane invariata
        if "intensity_range" in config:
            intensity = random.uniform(*config["intensity_range"])
            _set_light_attribute_float(light_prim_path, "intensity", intensity)
        
        if "color_range" in config:
            cr_min, cr_max = config["color_range"]
            color_val = Gf.Vec3f(
                random.uniform(cr_min[0], cr_max[0]),
                random.uniform(cr_min[1], cr_max[1]),
                random.uniform(cr_min[2], cr_max[2])
            )
            _set_light_attribute_color(light_prim_path, color_val)
        
        if "exposure_range" in config:
            exposure = random.uniform(*config["exposure_range"])
            _set_light_attribute_float(light_prim_path, "exposure", exposure)

        if light_type_str == "RectLight":
            if "width_range" in config:
                _set_light_attribute_float(light_prim_path, "width", random.uniform(*config["width_range"]))
            if "height_range" in config:
                _set_light_attribute_float(light_prim_path, "height", random.uniform(*config["height_range"]))

        elif light_type_str == "DistantLight":
            if "angle_range" in config: # L'angolo rimane casuale
                _set_light_attribute_float(light_prim_path, "angle", random.uniform(*config["angle_range"]))

        elif light_type_str == "SphereLight":
            if "radius_range" in config:
                _set_light_attribute_float(light_prim_path, "radius", random.uniform(*config["radius_range"]))

        elif light_type_str == "DomeLight":
            if config.get("enable_color_temperature", False) and "temperature_k_range" in config:
                 dome_light_api_schema = UsdLux.DomeLight(defined_light_schema.GetPrim())
                 enable_temp_attr = dome_light_api_schema.GetEnableColorTemperatureAttr()
                 enable_temp_attr.Set(True)
                 temp_k = random.uniform(*config["temperature_k_range"])
                 temp_attr = dome_light_api_schema.GetColorTemperatureAttr()
                 temp_attr.Set(temp_k)

        created_light_prims_info.append({"path": light_prim_path, "type": light_type_str})

    print(f"Spawn completato. Create {len(created_light_prims_info)} luci.")
    return created_light_prims_info