# utils/replicator_utils.py
import os
import numpy as np
import tifffile
import matplotlib.pyplot as plt
import json
import random
from typing import Dict, Set, Tuple, List, Optional

try:
    import cv2
except ImportError:
    print("IMPORT ERROR: OpenCV (cv2) non trovato. Necessario per lo splitting della segmentazione.")
    cv2 = None

SPLIT_CLASSES: Set[str] = {"box","ycb_object"}
IGNORE_CLASSES: Set[str] = {"BACKGROUND", "UNLABELLED"}
RGBA_Tuple = Tuple[int, int, int, int]

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
    except Exception as e: print(f"Errore salvataggio immagine '{path}': {e}")

def split_rgb_instance_segmentation(
    *, image_path: str, color_mapping_path: str, output_path_prefix: str,
    current_split_classes: Set[str], current_ignore_classes: Set[str]
) -> Tuple[Optional[str], Optional[str], bool]:
    if cv2 is None:
        print("  ❌ Errore: cv2 non disponibile per split_rgb_instance_segmentation.")
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
    print(f"  ✅ Split mask: '{final_output_png_path}', Split mapping: '{final_output_json_path}'")
    return final_output_png_path, final_output_json_path, True

def run_replicator_data_generation(
    simulation_app, timeline_ref, rep_module, carb_module, 
    rep_cfg: Dict, cam_path_str: str, output_dir_root: str
):
    if cv2 is None and rep_cfg.get("enable_segmentation_split", True):
        print("ERRORE: OpenCV (cv2) non disponibile, ma 'enable_segmentation_split' è True.")
        return

    if timeline_ref.is_playing(): timeline_ref.pause()
    simulation_app.update()
    os.makedirs(output_dir_root, exist_ok=True)
    print(f"Replicator: Generazione dati in '{output_dir_root}'.")

    with rep_module.new_layer():
        carb_s = carb_module.settings.get_settings()
        carb_s.set_string("/renderer/active", rep_cfg.get("renderer_active", "rtx"))
        carb_s.set_string("/rtx/rendermode", rep_cfg.get("rtx_rendermode", "PathTracing"))
        if "rtx_pathtracing_spp" in rep_cfg: carb_s.set_int("/rtx/pathtracing/spp", rep_cfg["rtx_pathtracing_spp"])
        simulation_app.update()
        rp = rep_module.create.render_product(cam_path_str, tuple(rep_cfg['resolution_wh']))
        for annotator_config in rep_cfg.get('annotators_to_attach', []):
            annotator_name = annotator_config.get("name") if isinstance(annotator_config, dict) else annotator_config
            if not annotator_name: continue
            try: 
                rep_module.annotators.get(annotator_name).attach(rp)
                print(f"  Annotatore '{annotator_name}' attaccato.")
            except Exception as e: print(f"ATTENZIONE: Annotatore '{annotator_name}' non attaccato: {e}")
        writer = rep_module.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=output_dir_root, **rep_cfg.get('writer_outputs', {}))
        writer.attach([rp])
        print("Avvio rep_module.orchestrator.step()...")
        rep_module.orchestrator.step()
        simulation_app.update() 
        print("rep_module.orchestrator.step() completato.")

    depth_filename_fixed = "distance_to_image_plane_0000.npy"
    input_npy_filepath = os.path.join(output_dir_root, depth_filename_fixed)
    if os.path.exists(input_npy_filepath):
        depth_array = np.load(input_npy_filepath)
        output_tiff_filepath = os.path.join(output_dir_root, "distance_to_image_plane_0000.tif")
        try:
            tifffile.imwrite(output_tiff_filepath, depth_array, imagej=True)
            print(f"  ✅ Dati di profondità TIFF: '{output_tiff_filepath}'")
        except Exception as e: print(f"  ❌ Errore salvataggio TIFF profondità: {e}")
        output_png_preview_filepath = os.path.join(output_dir_root, "distance_to_imageplane.png")
        try:
            min_val, max_val = np.min(depth_array), np.max(depth_array)
            depth_norm = np.zeros_like(depth_array,dtype=np.float32) if max_val == min_val else (depth_array - min_val) / (max_val - min_val)
            depth_img_preview = (depth_norm * 255).astype(np.uint8)
            plt.imsave(output_png_preview_filepath, depth_img_preview, cmap='plasma')
            print(f"  ✅ Preview profondità PNG: '{output_png_preview_filepath}'")
        except Exception as e: print(f"  ❌ Errore salvataggio PNG preview profondità: {e}")
    else:
        print(f"  ⚠️ File .npy di profondità '{depth_filename_fixed}' non trovato in '{output_dir_root}'.")

    if rep_cfg.get("enable_segmentation_split", True):
        if cv2 is None: print("  ⚠️ CV2 non disponibile. Skipping splitting segmentazione.")
        else:
            print("\n--- Tentativo di splitting segmentazione ---")
            segmentation_json_filename_fixed = "instance_segmentation_semantics_mapping_0000.json"
            segmentation_json_path = os.path.join(output_dir_root, segmentation_json_filename_fixed)

            # ---> MODIFICA QUI il nome fisso del file PNG di segmentazione se diverso da "instance_segmentation_0000.png" <---
            segmentation_image_filename_fixed = "instance_segmentation_0000.png" # Esempio, CAMBIA SE NECESSARIO
            # Altre possibilità: "rgb_0000.png", "nome_annotatore_custom_0000.png"
            
            segmentation_img_path = os.path.join(output_dir_root, segmentation_image_filename_fixed)
            
            print(f"  Cercando immagine segmentazione: '{segmentation_img_path}' (nome fisso: '{segmentation_image_filename_fixed}')")
            print(f"  Cercando JSON mapping: '{segmentation_json_path}' (nome fisso)")

            if os.path.exists(segmentation_img_path) and os.path.exists(segmentation_json_path):
                base_for_split_output = os.path.splitext(segmentation_image_filename_fixed)[0] 
                output_prefix_for_split = os.path.join(output_dir_root, base_for_split_output) 
                current_split_classes = set(rep_cfg.get("split_tool_config", {}).get("split_classes", list(SPLIT_CLASSES)))
                current_ignore_classes = set(rep_cfg.get("split_tool_config", {}).get("ignore_classes", list(IGNORE_CLASSES)))
                print(f"  Avvio splitting per: Img='{segmentation_img_path}', Json='{segmentation_json_path}'")
                split_rgb_instance_segmentation(
                    image_path=segmentation_img_path, color_mapping_path=segmentation_json_path,
                    output_path_prefix=output_prefix_for_split,
                    current_split_classes=current_split_classes, current_ignore_classes=current_ignore_classes
                )
            else:
                print("  ⚠️ Immagine segmentazione o JSON mapping non trovati. Skipping splitting.")
                if not os.path.exists(segmentation_img_path): print(f"    File immagine NON TROVATO: '{segmentation_img_path}' (Nome atteso: '{segmentation_image_filename_fixed}')")
                if not os.path.exists(segmentation_json_path): print(f"    File JSON NON TROVATO: '{segmentation_json_path}' (Nome atteso: '{segmentation_json_filename_fixed}')")
    else:
        print("\n--- Splitting segmentazione disabilitato in rep_cfg ---")
    print(f"\nReplicator: Generazione dati e post-processing in '{output_dir_root}' completati.")

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