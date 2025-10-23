"""Replicator data generation helpers."""

import os
import numpy as np
import tifffile
import matplotlib.pyplot as plt
import json
import random
from typing import Dict, Set, Tuple, List, Optional
import shutil
import typing as T

from depal.utils.logger import log_debug, log_error, log_info, log_section, log_warning

def _info(message: str) -> None:
    log_info(f"Replicator: {message}")

def _warn(message: str) -> None:
    log_warning(f"Replicator: {message}")

def _error(message: str) -> None:
    log_error(f"Replicator: {message}")

def _section(message: str) -> None:
    log_section(f"Replicator | {message}")

try:
    import cv2  # type: ignore
except ImportError:
    cv2 = None
    _warn("IMPORT ERROR: OpenCV (cv2) non trovato. Necessario per lo splitting della segmentazione.")

_PXR_CACHE = None


def _get_pxr_modules():
    global _PXR_CACHE
    if _PXR_CACHE is None:
        from pxr import UsdGeom, UsdPhysics, Gf, Usd  # pylint: disable=import-outside-toplevel

        _PXR_CACHE = (UsdGeom, UsdPhysics, Gf, Usd)
    return _PXR_CACHE


def _get_omni_usd():
    import omni.usd  # pylint: disable=import-outside-toplevel

    return omni.usd


def _info(message: str) -> None:
    log_info(f"Replicator: {message}")


def _warn(message: str) -> None:
    log_warning(f"Replicator: {message}")


def _error(message: str) -> None:
    log_error(f"Replicator: {message}")


def _section(message: str) -> None:
    log_section(f"Replicator | {message}")

SPLIT_CLASSES: Set[str] = {"box","ycb_object"}
IGNORE_CLASSES: Set[str] = {"BACKGROUND", "UNLABELLED"}
RGBA_Tuple = Tuple[int, int, int, int]


# --- Caratteristiche Fisse della Fotocamera (Definite Globalmente o come Costanti) ---
# Questi valori sono specifici per la tua configurazione della fotocamera.
# Modificali secondo le specifiche della TUA fotocamera.




def _calculate_focal_length_pixels(image_width_px, cam_focal_length_mm, cam_horizontal_aperture_mm):
    """Calcola la lunghezza focale in pixel basata sulla larghezza dell'immagine."""
    if cam_horizontal_aperture_mm == 0:
        _error(" CAM_HORIZONTAL_APERTURE_MM non puo essere zero.")
        return None
    return (cam_focal_length_mm / cam_horizontal_aperture_mm) * image_width_px


def _calculate_basic_depth_map(imgL_gray, imgR_gray, focal_length_px, baseline_m, max_depth_m=None):
    min_disp = 0
    num_disp = 16 * 5
    block_size = 7


    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=block_size,
        P1=8 * 1 * block_size**2,
        P2=32 * 1 * block_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    _info("Calcolo disparita (SGBM base)...")
    disparity = stereo.compute(imgL_gray, imgR_gray).astype(np.float32) / 16.0


    depth_map = np.zeros_like(disparity, dtype=np.float32)
    valid_disparity_mask = disparity > (min_disp / 16.0 + 0.01)
    
    baseline_mm = baseline_m * 1000.0
    depth_map[valid_disparity_mask] = (baseline_mm * focal_length_px) / disparity[valid_disparity_mask]


    if max_depth_m is not None:
        max_depth_mm = max_depth_m * 1000.0
        depth_map[depth_map > max_depth_mm] = max_depth_mm
        depth_map[depth_map <= 0] = max_depth_mm
        depth_map[~valid_disparity_mask] = max_depth_mm
    else:
        depth_map[~valid_disparity_mask] = 0
        depth_map[depth_map <= 0] = 0
    return depth_map, disparity

def _save_depth_map_as_image_only(depth_map_mm, output_path, max_depth_m=None):
    """
    Salva la mappa di profondita colorata come immagine pura (senza assi, colorbar, ecc.).
    """
    depth_display = depth_map_mm.copy()
    h, w = depth_display.shape


    # Normalizzazione per la colormap
    # Determina i limiti per la visualizzazione (vmin, vmax)
    valid_depths = depth_display[depth_display > 0] # Esclude 0 e valori negativi


    if max_depth_m is not None:
        max_depth_mm_val = max_depth_m * 1000.0
        # I valori <= 0 o > max_depth_mm sono gia stati clippati a max_depth_mm
        # nella funzione di calcolo se max_depth_m e specificato.
        # Per la normalizzazione, i valori 0 (che dovrebbero essere max_depth_mm in questo caso)
        # o i valori effettivamente a max_depth_mm saranno mappati al limite superiore della colormap.
        # I valori minimi "reali" saranno mappati al limite inferiore.
        # Se non ci sono profondita valide < max_depth_mm, l'immagine sara uniforme.
        vmin_norm = 0 # np.min(valid_depths) if valid_depths.size > 0 and np.min(valid_depths) < max_depth_mm_val else 0
        vmax_norm = max_depth_mm_val
        # Maschera per i valori che sono stati clippati o erano invalidi (ora a max_depth_mm)
        # o che sono zero se max_depth_m non e stato specificato.
        # Se max_depth_m e specificato, i valori 0 o non validi sono ora max_depth_mm.
        # Se max_depth_m NON e specificato, i valori 0 sono "profondita sconosciuta".
        
        # Normalizza tra 0 e 1.
        # I valori uguali a vmax_norm diventeranno 1.
        # I valori uguali a vmin_norm diventeranno 0.
        # Gestione divisione per zero se vmin_norm == vmax_norm (immagine piatta)
        if vmax_norm == vmin_norm:
            if vmax_norm == 0 : # Tutto e zero (o era invalido e max_depth_m non specificato)
                 normalized_depth = np.zeros_like(depth_display, dtype=np.float32)
            else: # Tutto e a max_depth (o era invalido e clippato a max_depth)
                 normalized_depth = np.ones_like(depth_display, dtype=np.float32)
        else:
            normalized_depth = (depth_display - vmin_norm) / (vmax_norm - vmin_norm)
        
        # Applica clipping dopo la normalizzazione per assicurarsi che sia tra 0 e 1
        normalized_depth = np.clip(normalized_depth, 0, 1)
        
        # Colora i valori che erano originariamente 0 (o invalidi, ora max_depth_mm)
        # con un colore specifico (es. nero) se vuoi distinguerli.
        # Per ora, saranno semplicemente il colore all'estremo della colormap.


    else: # max_depth_m non specificato, scala automatica
        if valid_depths.size > 0:
            vmin_norm = np.percentile(valid_depths, 1)
            vmax_norm = np.percentile(valid_depths, 99)
            if vmin_norm >= vmax_norm: # Caso degenere o dati molto piatti
                vmin_norm = np.min(valid_depths) * 0.9 if np.min(valid_depths) > 0 else 0
                vmax_norm = np.max(valid_depths) * 1.1 if np.max(valid_depths) > 0 else 1.0 # Evita vmax=0 se vmin=0
                if vmax_norm == 0 and vmin_norm == 0 : vmax_norm = 1.0 # Tutto zero


        else: # Nessuna profondita valida, immagine nera
            vmin_norm = 0
            vmax_norm = 1 # Evita divisione per zero


        if vmax_norm == vmin_norm: # Immagine piatta o tutta zero
            if vmax_norm == 0:
                normalized_depth = np.zeros_like(depth_display, dtype=np.float32)
            else: # Immagine piatta a un valore non zero (improbabile con percentile se c'e variazione)
                normalized_depth = np.ones_like(depth_display, dtype=np.float32) * 0.5 # grigio
        else:
            normalized_depth = (depth_display - vmin_norm) / (vmax_norm - vmin_norm)
        
        normalized_depth = np.clip(normalized_depth, 0, 1)
        # I pixel con profondita originale 0 (invalidi) saranno 0 nella mappa normalizzata (o il valore clippato).
        # Questi verranno colorati secondo l'estremo inferiore della colormap.


    # Applica la colormap
    # Usiamo 'viridis_r' come prima, ma potresti sceglierne altre da plt.cm.
    # La '_r' inverte la colormap, quindi i valori piu piccoli (vicini) sono piu chiari
    # e i valori piu grandi (lontani) sono piu scuri.
    colormap = plt.cm.get_cmap('viridis_r') 
    colored_image_rgba = colormap(normalized_depth) # Questo restituisce un array RGBA (0-1)


    # Converti da RGBA (0-1) a BGR uint8 (0-255) per OpenCV
    colored_image_bgr_uint8 = (colored_image_rgba[:, :, :3] * 255).astype(np.uint8)
    colored_image_bgr_uint8 = cv2.cvtColor(colored_image_bgr_uint8, cv2.COLOR_RGB2BGR)


    # Se max_depth_m non e stato specificato, colora i pixel con profondita 0 (invalidi) di nero
    if max_depth_m is None:
        mask_invalid = depth_display == 0
        colored_image_bgr_uint8[mask_invalid] = [0, 0, 0] # Nero per invalidi


    try:
        cv2.imwrite(output_path, colored_image_bgr_uint8)
        _info(f"Immagine mappa di profondita (solo immagine) salvata in: {output_path}")
    except Exception as e:
        _error(f" durante il salvataggio dell'immagine (solo immagine) della mappa di profondita: {e}")




# --- Funzione Minimale Richiesta (Aggiornata) ---
def process_stereo_images_minimal(
    left_image_path: str,
    right_image_path: str,
    output_folder: str,
    image_width_px: int, # Dimensione effettiva dell'immagine di input
    image_height_px: int, # Dimensione effettiva dell'immagine di input
    max_depth_meters: float,
    base_filename: str ,
    baseline_given:float,
    cameraFocalLength,
    cameraAperture
    
    ):
    """
    Funzione minimale per calcolare e salvare una mappa di profondita,
    utilizzando caratteristiche fisse della fotocamera e dimensioni dell'immagine di input.


    Args:
        left_image_path (str): Percorso dell'immagine sinistra.
        right_image_path (str): Percorso dell'immagine destra.
        output_folder (str): Cartella dove salvare i risultati.
        image_width_px (int): Larghezza delle immagini di input in pixel.
        image_height_px (int): Altezza delle immagini di input in pixel.
        max_depth_meters (float, optional): Massima profondita da considerare (in metri).
                                            Se None, la scala e automatica.
        base_filename (str, optional): Nome base per i file di output.
    Returns:
        bool: True se l'operazione ha avuto successo, False altrimenti.
    """


    _section("Processo stereo minimale")
    _info(f"Img Sinistra: {left_image_path}")
    _info(f"Img Destra: {right_image_path}")
    _info(f"Dimensioni Img Input: {image_width_px}x{image_height_px} px")
    _info(f"Output Folder: {output_folder}")
    _info(f"Max Profondita: {max_depth_meters} m" if max_depth_meters else "Max Profondita: Auto")


    # Usa le caratteristiche fisse della fotocamera definite sopra
    focal_length_px_calculated = _calculate_focal_length_pixels(
        image_width_px,
        cameraFocalLength,
        cameraAperture
    )
    if focal_length_px_calculated is None:
        _error("Errore nel calcolo della lunghezza focale in pixel. Interruzione.")
        return False
    
    baseline_to_use_meters = baseline_given # Usa la costante globale


    _info(f"Focale calcolata: {focal_length_px_calculated:.2f} px (da {cameraFocalLength}mm lens, {cameraAperture}mm sensor width)")
    _info(f"Baseline usata: {baseline_to_use_meters} m (costante)")


    if not os.path.exists(output_folder):
        try:
            os.makedirs(output_folder)
            _info(f"Cartella di output creata: {output_folder}")
        except OSError as e:
            _error(f": Impossibile creare la cartella di output {output_folder}. {e}")
            return False


    imgL_bgr = cv2.imread(left_image_path)
    imgR_bgr = cv2.imread(right_image_path)


    if imgL_bgr is None:
        _error(f": Impossibile caricare l'immagine sinistra da {left_image_path}")
        return False
    if imgR_bgr is None:
        _error(f": Impossibile caricare l'immagine destra da {right_image_path}")
        return False


    # Verifica e/o ridimensiona le immagini alle dimensioni specificate
    if imgL_bgr.shape[1] != image_width_px or imgL_bgr.shape[0] != image_height_px:
        _warn(f"Avviso: L'immagine sinistra {left_image_path} ha dimensioni {imgL_bgr.shape[1]}x{imgL_bgr.shape[0]},"
              f" ma sono state specificate {image_width_px}x{image_height_px}. Ridimensionamento...")
        imgL_bgr = cv2.resize(imgL_bgr, (image_width_px, image_height_px), interpolation=cv2.INTER_AREA)
    
    if imgR_bgr.shape[1] != image_width_px or imgR_bgr.shape[0] != image_height_px:
        _warn(f"Avviso: L'immagine destra {right_image_path} ha dimensioni {imgR_bgr.shape[1]}x{imgR_bgr.shape[0]},"
              f" ma sono state specificate {image_width_px}x{image_height_px}. Ridimensionamento...")
        imgR_bgr = cv2.resize(imgR_bgr, (image_width_px, image_height_px), interpolation=cv2.INTER_AREA)


    imgL_gray = cv2.cvtColor(imgL_bgr, cv2.COLOR_BGR2GRAY)
    imgR_gray = cv2.cvtColor(imgR_bgr, cv2.COLOR_BGR2GRAY)


    try:
        depth_map_mm, _ = _calculate_basic_depth_map(
            imgL_gray, imgR_gray,
            focal_length_px=focal_length_px_calculated,
            baseline_m=baseline_to_use_meters,
            max_depth_m=max_depth_meters
        )
    except Exception as e:
        _error(f" durante il calcolo della mappa di profondita: {e}")
        return False


    if depth_map_mm is None:
        _warn("Calcolo della mappa di profondita fallito")
        return False


    npy_filename = os.path.join(output_folder, f"{base_filename}.npy")
    try:
        np.save(npy_filename, depth_map_mm)
        _info(f"Mappa di profondita numerica (.npy) salvata in: {npy_filename}")
    except Exception as e:
        _error(f" durante il salvataggio del file .npy: {e}")



    depth_array = np.load(npy_filename)
    depth_tiff_basename = f"{base_filename}.tif"
    output_tiff_filepath = os.path.join(output_folder, f"{depth_tiff_basename}")
    tifffile.imwrite(output_tiff_filepath, depth_array, imagej=True)


    colored_map_filename = os.path.join(output_folder, f"{base_filename}_preview.png")
    try:
        title_info_plot = f"F={focal_length_px_calculated:.1f}px, B={baseline_to_use_meters*1000:.0f}mm"
        _save_depth_map_as_image_only(
            depth_map_mm,
            colored_map_filename,
            max_depth_m=max_depth_meters,
            
            )
    except Exception as e:
        _error(f" durante il salvataggio dell'immagine colorata della mappa di profondita: {e}")
        return False


    _section("Processo stereo minimale completato")
    return True


def _random_rgba(used: Set[RGBA_Tuple]) -> RGBA_Tuple:
    while True:
        rgb = tuple(random.randint(1, 255) for _ in range(3))
        rgba = (*rgb, 255)
        if rgba not in used: return rgba

def _load_rgba(path: str) -> Optional[np.ndarray]:
    if cv2 is None: return None
    if not os.path.exists(path): return None
    img_bgra = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img_bgra is None: return None
    if img_bgra.ndim == 2: return cv2.cvtColor(img_bgra, cv2.COLOR_GRAY2RGBA)
    if img_bgra.shape[2] == 3: return cv2.cvtColor(img_bgra, cv2.COLOR_BGR2RGBA)
    if img_bgra.shape[2] == 4: return cv2.cvtColor(img_bgra, cv2.COLOR_BGRA2RGBA)
    return None

def _save_rgba(path: str, img_rgba: np.ndarray) -> None:
    if cv2 is None: return
    output_dir = os.path.dirname(path)
    if not os.path.exists(output_dir) and output_dir != "": os.makedirs(output_dir, exist_ok=True)
    img_bgra = cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2BGRA)
    try: cv2.imwrite(path, img_bgra)
    except Exception as e: _error(f" salvataggio immagine '{path}': {e}")



def split_rgb_instance_segmentation(
    *, image_path: str, color_mapping_path: str, output_path_prefix: str,
    current_split_classes: Set[str], current_ignore_classes: Set[str]
) -> Tuple[Optional[str], Optional[str], bool]:
    if cv2 is None:
        _error("   Errore: cv2 non disponibile per split_rgb_instance_segmentation.")
        return None, None, False
    img = _load_rgba(image_path)
    if img is None: return None, None, False
    if not os.path.exists(color_mapping_path): return None, None, False
    with open(color_mapping_path, "r", encoding="utf-8") as f:
        try: map_in: Dict[str, Dict[str, str]] = json.load(f)
        except json.JSONDecodeError: return None, None, False
    if img.shape[2] != 4: return None, None, False
    unique_colors_in_image = [tuple(c) for c in np.unique(img.reshape(-1, 4), axis=0)]
    out_img = np.zeros_like(img)
    out_map: Dict[str, Dict[str, str]] = {}
    used_output_colors: Set[RGBA_Tuple] = set()
    for color_str_key, info in map_in.items():
        cls_label = info.get("class", "unknown")
        try:
            color_val = eval(color_str_key)
            if isinstance(color_val, list): color_val = tuple(color_val)
            if not (isinstance(color_val, tuple) and (len(color_val) == 3 or len(color_val) == 4)): continue
            if len(color_val) == 3: color_val = (*color_val, 255)
            if cls_label in current_ignore_classes or cls_label not in current_split_classes: used_output_colors.add(color_val)
        except Exception: pass
    for current_img_color_rgba in unique_colors_in_image:
        color_key_options = [str(list(current_img_color_rgba)), str(current_img_color_rgba), str(current_img_color_rgba).replace(" ", "")]
        if current_img_color_rgba[3] == 255:
            rgb_part = current_img_color_rgba[:3]
            color_key_options.extend([str(list(rgb_part)), str(rgb_part), str(rgb_part).replace(" ", "")])
        semantic_info = next((map_in[key_opt] for key_opt in color_key_options if key_opt in map_in), None)
        cls_label = semantic_info.get("class", "unknown") if semantic_info else "unknown"
        mask_pixels = np.all(img == current_img_color_rgba, axis=-1)
        if not semantic_info:
            out_img[mask_pixels] = current_img_color_rgba
            continue
        if cls_label in current_ignore_classes or cls_label not in current_split_classes:
            out_img[mask_pixels] = current_img_color_rgba
            out_map[str(list(current_img_color_rgba))] = {"class": cls_label}
            used_output_colors.add(current_img_color_rgba)
        else:
            binary_mask_for_cc = mask_pixels.astype(np.uint8)
            num_cc, labels_cc_img = cv2.connectedComponents(binary_mask_for_cc)
            instance_counter = 0
            for cc_id in range(1, num_cc):
                component_mask = (labels_cc_img == cc_id)
                if not np.any(component_mask): continue
                instance_counter += 1
                new_color = current_img_color_rgba if instance_counter == 1 and current_img_color_rgba not in used_output_colors else _random_rgba(used_output_colors)
                used_output_colors.add(new_color)
                out_img[component_mask] = new_color
                out_map[str(list(new_color))] = {"class": cls_label, "original_color_if_split": str(list(current_img_color_rgba))}
    final_output_png_path = f"{output_path_prefix}_split.png"
    final_output_json_path = f"{output_path_prefix}_split_mapping.json"
    _save_rgba(final_output_png_path, out_img)
    json_output_dir = os.path.dirname(final_output_json_path)
    if not os.path.exists(json_output_dir) and json_output_dir != "": os.makedirs(json_output_dir, exist_ok=True)
    with open(final_output_json_path, "w", encoding="utf-8") as f: json.dump(out_map, f, indent=2, sort_keys=True)
    _info(f"   Split mask: '{final_output_png_path}', Split mapping: '{final_output_json_path}'")
    return final_output_png_path, final_output_json_path, True

def run_replicator_data_generation(
    simulation_app, timeline_ref, rep_module, carb_module,
    rep_cfg: Dict, cam_path_str: str,
   
    output_dir_root: str
):
    _, _, Gf, _ = _get_pxr_modules()
    omni_usd = _get_omni_usd()
    if cv2 is None and rep_cfg.get("enable_segmentation_split", True):
        _warn("AVVISO: OpenCV (cv2) non disponibile, ma 'enable_segmentation_split' e True. Lo splitting non sara possibile.")

    if timeline_ref.is_playing():
        timeline_ref.pause()
    simulation_app.update()

    os.makedirs(output_dir_root, exist_ok=True)
    _info(f"Replicator: Inizio generazione dati in '{output_dir_root}'. Elaborazione telecamere sequenzialmente.")

    MAIN_CAMERA_RP_NAME = "main_cam_rp" # Suffisso RP per chiarezza interna
    LEFT_STEREO_RP_NAME = "left_stereo_rp"
    RIGHT_STEREO_RP_NAME = "right_stereo_rp"

  
    resolution_wh = tuple(rep_cfg['resolution_wh'])
    # Questa configurazione e per gli annotatori della MAIN CAMERA
    # e per le impostazioni globali del writer (es. formato RGB)
    writer_global_config = rep_cfg.get('writer_outputs', {}).copy()

    
    aniso_path = "/rtx/texture/maxAnisotropy" # Percorso comune
    carb_s = carb_module.settings.get_settings()
    carb_s.set_string("/renderer/active", rep_cfg.get("renderer_active", "rtx"))
    carb_s.set_string("/rtx/rendermode", rep_cfg.get("rtx_rendermode", "PathTracing"))
    carb_s.set_int(aniso_path,16)
    carb_s.set_int("/rtx/pathtracing/spp", rep_cfg["rtx_pathtracing_spp"])
    #simulation_app.update()

    '''
    # --- 1. Elaborazione Telecamera Principale ---
    if cam_path_str:
        _info(f"\n--- Elaborazione camera principale: {cam_path_str} -> '{output_dir_root}' ---")
        with  rep_module.new_layer():

            
            rp_main = rep_module.create.render_product(cam_path_str, resolution_wh, name=MAIN_CAMERA_RP_NAME)
            
            annotators_for_main_cam = rep_cfg.get('annotators_to_attach', [])
            _info(f"  Collegamento annotatori (da rep_cfg) a {MAIN_CAMERA_RP_NAME}:")
            for annotator_config in annotators_for_main_cam:
                annotator_name = annotator_config.get("name") if isinstance(annotator_config, dict) else annotator_config
                if not annotator_name: continue
                try:
                    annotator_instance = rep_module.annotators.get(annotator_name)
                    annotator_instance.attach(rp_main)
                    _info(f"    Annotatore '{annotator_name}' collegato.")
                except Exception as e:
                    _info(f"    ATTENZIONE: Annotatore '{annotator_name}' non collegato a {MAIN_CAMERA_RP_NAME}: {e}")
            
            writer_main = rep_module.WriterRegistry.get("BasicWriter")
            # Usa writer_global_config che puo specificare formati per annotatori (es. rgb:png)
            writer_main.initialize(output_dir=output_dir_root, **writer_global_config)
            writer_main.attach([rp_main])
            rep_module.orchestrator.step()'''

            
        
    # ------------------------------------------------------------------
    # 2) Query worldposition (height) of the main camera.
    # ------------------------------------------------------------------
    stage = omni_usd.get_context().get_stage()
    main_cam_prim = stage.GetPrimAtPath(cam_path_str)
    if not main_cam_prim.IsValid():
        raise ValueError(f"Camera prim not found at path: {cam_path_str}")

    # Extract world translation (Pxr USD util)  returns Gf.Vec3d.
    world_m44 = omni_usd.utils.get_world_transform_matrix(main_cam_prim)  # type: ignore
    main_cam_world_pos = Gf.Vec3d(world_m44.ExtractTranslation())


     #provo a forzare un reset
    carb_s.set_string("/rtx/rendermode", "RayTracedLighting") 
    carb_s.set_string("/rtx/rendermode", "PathTracing") 
    
    carb_s.set_int("/rtx/pathtracing/spp", rep_cfg["rtx_pathtracing_spp"])
    
    

    with rep_module.new_layer():


        stereo_position = (
            rep_cfg.get("stereo_origin_x", 0.0),
            rep_cfg.get("stereo_origin_y", 0.0),
            float(main_cam_world_pos[2]),
        )
        _info("posizione camera stereo", stereo_position)

   
        stereo = rep_module.create.stereo_camera(
                stereo_baseline=rep_cfg['stereo_baseline'],          # 6 cm
                name="stereo_cam",
                position=stereo_position,            # dove vuoi tu
                look_at=(0,0,0),
                focus_distance=0.0,
                focal_length = 35.0,
                f_stop=0,
                
            )
        
        



        #rps = rep_module.create.render_product(stereo, resolution_wh)

        rp_left = rep_module.create.render_product(
        "/Replicator/stereo_cam/stereo_cam_L_Xform/stereo_cam_L", # Passa il path del prim della camera sinistra
        resolution_wh,
        name="left" # Questo nome sara usato per il RenderProduct prim e la cartella
        )
        rp_right = rep_module.create.render_product(
            "/Replicator/stereo_cam/stereo_cam_R_Xform/stereo_cam_R", # Passa il path del prim della camera destra
            resolution_wh,
            name="right" # Questo nome sara usato per il RenderProduct prim e la cartella
        )




        carb_module.settings.get_settings().set_bool("/rtx/pathtracing/dof", False)

        writer = rep_module.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=output_dir_root, **writer_global_config)
        
        writer.attach([rp_left,rp_right])

        rep_module.orchestrator.set_next_rt_subframes(64)  
           
        rep_module.orchestrator.step()
        

        
       
        for _ in range(10):
            simulation_app.update()
        
        
        



    # --- Post-processing (Per output della Telecamera Principale, che si trovano in output_dir_root) ---
    # Questa parte rimane invariata, opera sui file in output_dir_root
    _info(f"\n--- Inizio Post-Processing per output della camera principale in '{output_dir_root}' ---")
    
    depth_npy_basename = os.path.join("left", "distance_to_image_plane", "distance_to_image_plane_0000.npy")
    input_npy_filepath = os.path.join(output_dir_root, depth_npy_basename)
    conversione = False
    
    if os.path.exists(input_npy_filepath):
        _info(f"  Trovato file di profondita: '{input_npy_filepath}'")
        try:
            depth_array = np.load(input_npy_filepath)
            
            depth_tiff_basename = "distance_to_image_plane_0000.tif"
            output_tiff_filepath = os.path.join(output_dir_root, "left", "distance_to_image_plane", depth_tiff_basename)
            tifffile.imwrite(output_tiff_filepath, depth_array, imagej=True)
            _info(f"   Dati di profondita TIFF: '{output_tiff_filepath}'")
            
            depth_preview_png_basename = "distance_to_image_plane_0000_preview.png"
            output_png_preview_filepath = os.path.join(output_dir_root, "left", "distance_to_image_plane", depth_preview_png_basename)
            min_val, max_val = np.min(depth_array), np.max(depth_array)
            depth_norm = np.zeros_like(depth_array,dtype=np.float32) if max_val == min_val else (depth_array - min_val) / (max_val - min_val)
            depth_img_preview = (depth_norm * 255).astype(np.uint8)
            plt.imsave(output_png_preview_filepath, depth_img_preview, cmap='plasma')
            _info(f"   Preview profondita PNG: '{output_png_preview_filepath}'")
        except Exception as e: 
            _error(f"   Errore durante la conversione/salvataggio dei dati di profondita: {e}")
    else:
        _warn(f"   File .npy di profondita '{input_npy_filepath}' non trovato in '{output_dir_root}'. Verranno generati i file tramite pipeline stereo.")

    if rep_cfg.get("enable_segmentation_split", True):
        if cv2 is None:
            _warn("CV2 non disponibile. Splitting segmentazione ignorato")
        else:
            _info(f"\n--- Tentativo di splitting segmentazione (per output della camera principale in '{output_dir_root}') ---")
            segmentation_json_basename = "instance_segmentation_semantics_mapping_0000.json"
            segmentation_image_basename_for_split = rep_cfg.get("segmentation_image_filename_for_split", "instance_segmentation_0000.png")
            
            segmentation_json_path = os.path.join(output_dir_root, "left", "instance_segmentation", segmentation_json_basename)
            segmentation_img_path = os.path.join(output_dir_root, "left", "instance_segmentation", segmentation_image_basename_for_split)
            
            log_debug(f"  Cercando immagine segmentazione per splitting: '{segmentation_img_path}'")
            log_debug(f"  Cercando JSON mapping per splitting: '{segmentation_json_path}'")

            if os.path.exists(segmentation_img_path) and os.path.exists(segmentation_json_path):
                base_for_split_output = os.path.splitext(segmentation_image_basename_for_split)[0]
                output_prefix_for_split = os.path.join(output_dir_root, "left", "instance_segmentation", base_for_split_output)
                
                current_split_classes = set(rep_cfg.get("split_tool_config", {}).get("split_classes", list(SPLIT_CLASSES)))
                current_ignore_classes = set(rep_cfg.get("split_tool_config", {}).get("ignore_classes", list(IGNORE_CLASSES)))
                
                log_debug(f"  Avvio splitting per: Img='{segmentation_img_path}', Json='{segmentation_json_path}', OutPrefix='{output_prefix_for_split}'")
                split_rgb_instance_segmentation(
                    image_path=segmentation_img_path, color_mapping_path=segmentation_json_path,
                    output_path_prefix=output_prefix_for_split,
                    current_split_classes=current_split_classes, current_ignore_classes=current_ignore_classes
                )
            else:
                _warn(f"   Immagine segmentazione o JSON mapping non trovati in '{output_dir_root}'. Skipping splitting.")
                if not os.path.exists(segmentation_img_path): log_debug(f"    File immagine NON TROVATO: '{segmentation_img_path}'")
                if not os.path.exists(segmentation_json_path): log_debug(f"    File JSON NON TROVATO: '{segmentation_json_path}'")
    else:
        _info("\n--- Splitting segmentazione disabilitato in rep_cfg ---")
    
    _info(f"\nReplicator: Generazione dati e post-processing in '{output_dir_root}' (e sottocartelle stereo) completati.")


    rgb_left = os.path.join(output_dir_root, "left", "rgb", "rgb_0000.png")
    rgb_right = os.path.join(output_dir_root, "right", "rgb", "rgb_0000.png")
    json_file_path = os.path.join(output_dir_root, "left", "camera_params", "camera_params_0000.json")

    if not os.path.exists(json_file_path):
        _warn(f"   Parametri camera non trovati a '{json_file_path}'. La fase stereo viene ignorata.")
        return

    try:
        with open(json_file_path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except json.JSONDecodeError as exc:
        _warn(f"   Parametri camera non leggibili ({exc}). La fase stereo viene ignorata.")
        return

    cameraAperture = float(data["cameraAperture"][0])
    cameraFocalLength = float(data["cameraFocalLength"])
    log_debug(f"Camera aperture: {cameraAperture}")
    log_debug(f"Camera focal length: {cameraFocalLength}")

    success = process_stereo_images_minimal(
        left_image_path=rgb_left,
        right_image_path=rgb_right,
        output_folder=os.path.join(output_dir_root, "left", "distance_to_image_plane"),
        image_width_px=resolution_wh[0], # Passa le dimensioni effettive
        image_height_px=resolution_wh[1], # Passa le dimensioni effettive
        max_depth_meters=float(main_cam_world_pos[2]),
        base_filename="distance_to_image_plane_0000",
        baseline_given=rep_cfg['stereo_baseline'],
        cameraAperture=cameraAperture,
        cameraFocalLength=cameraFocalLength
        
    )




# Esempio rep_cfg:
# rep_config = {
#     "resolution_wh": (1920, 1080), 
#     "renderer_active": "PathTracing", "rtx_rendermode": "PathTracing", "rtx_pathtracing_spp": 16,
#     "annotators_to_attach": [
#         {"name": "rgb"}, # O l'annotatore che produce il file PNG specificato in segmentation_image_filename_fixed
#         {"name": "instance_segmentation"}, # O l'annotatore che produce il mapping instance_segmentation_semantics_mapping_0000.json
#         {"name": "distance_to_image_plane"}
#     ],
#     "writer_outputs": { /* Configura BasicWriter per outputtare file con nomi fissi direttamente in output_dir_root */ },
#     "enable_segmentation_split": True, 
#     "split_tool_config": { "split_classes": ["TuaClasse1"], "ignore_classes": ["BACKGROUND"] }
# }
