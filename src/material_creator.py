
from pxr import Sdf, UsdShade, Usd, Gf
import os, random, traceback, typing as _t

try:
    from isaacsim.core.api.materials import OmniPBR
except ImportError:
    OmniPBR = None
    print("CRITICAL: OmniPBR non importato – verifica i Python-path di Isaac Sim.")

# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _set_input(shader: UsdShade.Shader, name: str, vtype, value) -> bool:
    """Imposta *solo* un input già presente nello shader.
    Ritorna True se l'input esiste ed è stato scritto, False altrimenti."""
    inp = shader.GetInput(name)
    if not inp :
        return False  # input non esposto dalla variante OmniPBR corrente
    try:
        inp.Set(value)
        return True
    except Exception as e:
        print(f" unable to set '{name}' → {value}: {e}")
        return False

# ---------------------------------------------------------------------------
# Parameter schema (cfg-key  →  shader-name  →  Sdf-type  →  sampler-fn)
# ---------------------------------------------------------------------------

_PARAM_DEFS: dict[str, tuple[str, Sdf.ValueTypeName, callable]] = {
    # --- PBR core -----------------------------------------------------------
    "metallic_range":     ("metallic_constant",             Sdf.ValueTypeNames.Float,  lambda r: random.uniform(*r)),
    "roughness_range":    ("reflection_roughness_constant", Sdf.ValueTypeNames.Float,  lambda r: random.uniform(*r)),
    "specular_level_range": ("specular_level",              Sdf.ValueTypeNames.Float,  lambda r: random.uniform(*r)),

    # --- Colors -------------------------------------------------------------
    "base_color_rgb_range": ("diffuse_color_constant",      Sdf.ValueTypeNames.Float3, lambda r: Gf.Vec3f(*(random.uniform(a, b) for a, b in zip(*r)))),
    "emissive_rgb_range":   ("emissive_color",              Sdf.ValueTypeNames.Float3, lambda r: Gf.Vec3f(*(random.uniform(a, b) for a, b in zip(*r)))),
    "emissive_strength_range": ("emissive_intensity",       Sdf.ValueTypeNames.Float,  lambda r: random.uniform(*r)),

    # --- Clear-coat ---------------------------------------------------------
    "clearcoat_intensity_range": ("clearcoat_weight",             Sdf.ValueTypeNames.Float,  lambda r: random.uniform(*r)),
    "clearcoat_roughness_range": ("clearcoat_reflection_roughness",Sdf.ValueTypeNames.Float, lambda r: random.uniform(*r)),
    "ior_range":                 ("clearcoat_ior",                Sdf.ValueTypeNames.Float,  lambda r: random.uniform(*r)),

    # --- Normal map ---------------------------------------------------------
    "normal_intensity_range": ("bump_factor",                  Sdf.ValueTypeNames.Float,  lambda r: random.uniform(*r)),
}

# ---------------------------------------------------------------------------
# Core builder
# ---------------------------------------------------------------------------

def _build_single_pbr(stage: Usd.Stage,
                      sim,
                      mtl_path: str,
                      tex_path: str,
                      cfg: dict):
    if OmniPBR is None:
        return None, [], None

    # Crea materiale OmniPBR --------------------------------------------------
    scale_rng = cfg.get("texture_scale_range", [1.0, 1.0])
    scale_val = random.uniform(*scale_rng)
    try:
        OmniPBR(prim_path=mtl_path,
                texture_path=tex_path,
                texture_scale=[scale_val, scale_val])
    except Exception as e:
        print(f"ERROR OmniPBR {mtl_path}: {e}")
        traceback.print_exc()
        return None, [], None

    # Spingi qualche frame per assicurare la creazione dei prim ----------------
    if sim:
        for _ in range(2):
            sim.update()

    # Recupera prim/material/shader -----------------------------------------
    mtl_prim   = stage.GetPrimAtPath(mtl_path)
    mtl_schema = UsdShade.Material(mtl_prim)
    shader_prim = next((c for c in mtl_prim.GetChildren() if c.IsA(UsdShade.Shader)), None)
    if not shader_prim:
        print(f"⚠︎ nessuno shader trovato sotto {mtl_path}")
        return mtl_prim, [], mtl_schema
    shader = UsdShade.Shader(shader_prim)

    # Applica i parametri -----------------------------------------------------
    applied = {}
    for cfg_key, (sname, stype, sampler) in _PARAM_DEFS.items():
        if cfg_key not in cfg:
            continue  # parametro opzionale non definito nel YAML
        
        val = sampler(cfg[cfg_key])
        if _set_input(shader, sname, stype, val):
            applied[sname] = val

    # Gestione probabilità di emissione ---------------------------------------
    if "emissive_probability" in cfg:
        prob = cfg["emissive_probability"]
        # Valida la probabilità (deve essere tra 0.0 e 1.0)
        if not isinstance(prob, (float, int)) or not (0.0 <= prob <= 1.0):
            print(f"   ⚠︎ emissive_probability ({prob}) non valida o fuori range [0,1]. Verrà ignorata.")
        # Se random.random() è MAGGIORE o UGUALE alla probabilità, l'emissione è SPENTA.
        # Esempio: se prob = 0.75 (75% di chance di essere emissivo),
        # random.random() >= 0.75 accade il 25% delle volte (emissione SPENTA).
        elif random.random() >= prob:
            # Disattiva emissione: imposta colore emissivo a nero e intensità a zero
            black_color = Gf.Vec3f(0.0, 0.0, 0.0)
            zero_intensity = 0.0
            
            if _set_input(shader, "emissive_color", Sdf.ValueTypeNames.Float3, black_color):
                applied["emissive_color"] = black_color # Aggiorna per il log
            
            if _set_input(shader, "emissive_intensity", Sdf.ValueTypeNames.Float, zero_intensity):
                applied["emissive_intensity"] = zero_intensity # Aggiorna per il log
            
            
            

            if applied:
                print("  · " + ", ".join(f"{k}={v}" for k, v in applied.items()))

            # Logga che l'emissione è stata disattivata a causa della probabilità
            print(f"   · emissione disattivata per probabilità (chance ON: {prob*100:.1f}%)")
        else: 
           pass

    _set_input(shader, "enable_emission", Sdf.ValueTypeNames.Bool, True)        
    _set_input(shader, "enable_clearcoat", Sdf.ValueTypeNames.Bool, True)
 
    if applied:
        # Formattazione per una stampa più pulita dei valori applicati
        print_applied_str_parts = []
        for k, v_obj in applied.items():
            if isinstance(v_obj, Gf.Vec3f):
                v_str = f"({v_obj[0]:.2f}, {v_obj[1]:.2f}, {v_obj[2]:.2f})"
            elif isinstance(v_obj, float):
                v_str = f"{v_obj:.2f}"
            else:
                v_str = str(v_obj)
            print_applied_str_parts.append(f"{k}={v_str}")
        print("   · " + ", ".join(print_applied_str_parts))


    return mtl_prim, [shader_prim], mtl_schema

# ---------------------------------------------------------------------------
# Public API – mantiene il prototipo richiesto dal progetto
# ---------------------------------------------------------------------------

def crea_materiali_da_cartella_texture(
        stage: Usd.Stage,
        simulation_app_instance,
        texture_dir_path: str,
        base_material_usd_path: str = "/World/Looks",
        material_config: _t.Optional[dict] = None):
    """Scansiona una cartella texture e crea un OmniPBR per ciascun file.

    Ritorna una lista di tuple `(primMaterial, [primShader], schemaMaterial)`.
    """

    sim = simulation_app_instance
    cfg = material_config or {}
    tdir = texture_dir_path
    baseu = base_material_usd_path

    if not os.path.isdir(tdir):
        print(f"Cartella texture inesistente: {tdir}")
        return []

    print(f"Creazione materiali da '{tdir}' in '{baseu}'…")
    results = []

    for fname in os.listdir(tdir):
        if not fname.lower().endswith((".png", ".jpg", ".jpeg", ".tga", ".dds", ".exr", ".hdr")):
            continue
        safe = "".join(c if c.isalnum() else "_" for c in os.path.splitext(fname)[0])
        mtl_path = f"{baseu}/Mtl_{safe}"
        tex_full = os.path.join(tdir, fname).replace("\\", "/")
        prim, shaders, schema = _build_single_pbr(stage, sim, mtl_path, tex_full, cfg)
        if prim and prim.IsValid():
            results.append((prim, shaders, schema))

    print(f"Creati {len(results)} materiali PBR.")
    return results

