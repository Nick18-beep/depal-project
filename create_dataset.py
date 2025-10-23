import os
import json
import glob
import numpy as np
from PIL import Image
from skimage import measure
from datetime import datetime
import shutil

def create_final_coco_dataset(output_dir, start_dir):
    """
    Genera un dataset COCO completo e strutturato. Copia le immagini RGB reali
    e usa le maschere di segmentazione per creare le annotazioni.

    Args:
        output_dir (str): La cartella principale dove salvare il dataset (es. 'mio_dataset_coco').
        start_dir (str): La directory di partenza che contiene le cartelle img1, img2, etc. (es. 'output').
    """
    # 1. Definisci e crea la struttura delle cartelle di output
    images_output_dir = os.path.join(output_dir, "images")
    annotations_output_path = os.path.join(output_dir, "annotations.json")
    
    os.makedirs(images_output_dir, exist_ok=True)
    print(f"Struttura del dataset creata o verificata in: '{output_dir}/'")

    now = datetime.now()
    coco_output = {
        "info": {"description": "Dataset COCO con Immagini Reali", "version": "1.0", "year": now.year, "date_created": now.strftime("%Y/%m/%d")},
        "licenses": [], "images": [], "annotations": [], "categories": []
    }
    coco_output["categories"].append({"id": 1, "name": "box", "supercategory": "object"})

    image_id_counter = 1
    annotation_id_counter = 1

    # Cerca tutte le directory 'img*' nella cartella di partenza
    image_dirs = sorted(glob.glob(os.path.join(start_dir, 'img*')))
    if not image_dirs:
        print(f"\nATTENZIONE: Nessuna directory trovata nel percorso: '{os.path.join(start_dir, 'img*')}'")
        print("-> Assicurati di eseguire lo script dalla cartella che contiene la directory 'output'.")
        return

    print(f"Trovate {len(image_dirs)} directories. Inizio elaborazione...")

    for img_dir in image_dirs:
        print(f"Elaboro la directory: {img_dir}")

        # 2. Definisci i percorsi per TUTTI i file necessari
        # Percorso dell'IMMAGINE REALE (RGB)
        real_image_path = os.path.join(img_dir, "left", "rgb", "rgb_0000.png")
        # Percorso della MASCHERA di segmentazione
        mask_image_path = os.path.join(img_dir, "left", "instance_segmentation", "instance_segmentation_0000_split.png")
        # Percorso del file di MAPPATURA json
        json_map_path = os.path.join(img_dir, "left", "instance_segmentation", "instance_segmentation_0000_split_mapping.json")

        # Controlla che tutti i file esistano
        if not all(os.path.exists(p) for p in [real_image_path, mask_image_path, json_map_path]):
            print(f"  - Attenzione: File mancanti (RGB, maschera o JSON) in {img_dir}. Salto.")
            continue
        
        # 3. Copia l'IMMAGINE REALE e ottieni le sue dimensioni
        dir_name = os.path.basename(img_dir)
        new_image_name = f"{dir_name}.png"
        destination_image_path = os.path.join(images_output_dir, new_image_name)

        try:
            # Copia l'immagine RGB
            shutil.copy2(real_image_path, destination_image_path)
            # Apri l'immagine RGB per ottenere le dimensioni
            with Image.open(real_image_path) as img:
                width, height = img.size
            # Apri l'immagine MASCHERA per elaborare le annotazioni
            with Image.open(mask_image_path) as mask_img:
                 mask_image_rgba = np.array(mask_img.convert("RGBA"))

        except Exception as e:
            print(f"  - Errore durante la gestione dei file in {img_dir}: {e}. Salto.")
            continue
            
        # 4. Aggiungi le info dell'immagine al JSON (usando il percorso nuovo e le dimensioni dell'img reale)
        image_info = {"id": image_id_counter, "file_name": f"{new_image_name}", "width": width, "height": height}
        coco_output["images"].append(image_info)

        with open(json_map_path, 'r') as f:
            mapping_data = json.load(f)

        # 5. Elabora la MASCHERA per creare le annotazioni
        for color_str, class_info in mapping_data.items():
            if class_info.get("class") == "box":
                color_rgba = np.array(json.loads(color_str), dtype=np.uint8)
                binary_mask = np.all(mask_image_rgba == color_rgba, axis=-1).astype(np.uint8)
                contours = measure.find_contours(binary_mask, 0.5)

                for contour in contours:
                    contour = np.round(contour).astype(int)
                    x_min, y_min = np.min(contour[:, 1]), np.min(contour[:, 0])
                    x_max, y_max = np.max(contour[:, 1]), np.max(contour[:, 0])
                    bbox = [int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min)]
                    
                    x, y = contour[:, 1], contour[:, 0]
                    area = 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
                    
                    if area < 10: continue
                        
                    segmentation = contour[:, ::-1].flatten().tolist()
                    annotation = {
                        "id": annotation_id_counter, "image_id": image_id_counter, "category_id": 1,
                        "segmentation": [segmentation], "area": float(area), "bbox": bbox, "iscrowd": 0
                    }
                    coco_output["annotations"].append(annotation)
                    annotation_id_counter += 1

        image_id_counter += 1

    # 6. Salva il file JSON finale
    with open(annotations_output_path, 'w') as f:
        json.dump(coco_output, f, indent=4)

    print(f"\n--- Operazione Completata con Successo ---")
    print(f"Dataset salvato in: '{output_dir}'")
    print(f"-> Annotazioni: {annotations_output_path}")
    print(f"-> Immagini reali copiate in: {images_output_dir}")
    print(f"Totale immagini: {len(coco_output['images'])} | Totale annotazioni: {len(coco_output['annotations'])}")

# --- ESECUZIONE DELLO SCRIPT ---
if __name__ == '__main__':
    # Cartella di partenza con i dati grezzi (deve contenere 'img1', 'img2', ...)
    start_directory = 'output' 
    
    # Cartella di destinazione dove verrà creato il dataset pulito
    output_dataset_dir = 'mio_dataset_coco' 

    create_final_coco_dataset(output_dataset_dir, start_directory)