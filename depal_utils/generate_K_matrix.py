import json
import numpy as np
import os

def generate_k_matrix(camera_params_path: str) -> np.ndarray | None:
    """
    Genera la matrice intrinseca K utilizzando i parametri della camera da un file JSON.
    Il metodo si basa sulla 'cameraProjection' matrix per maggiore precisione.
    """
    try:
        with open(camera_params_path, 'r') as f:
            data = json.load(f)

        img_width_px = float(data["renderProductResolution"][0])
        img_height_px = float(data["renderProductResolution"][1])
        
        # P[0][0] e P[1][1] dalla cameraProjection (assunta row-major)
        # P[0][0] = cameraProjection[0], P[1][1] = cameraProjection[5]
        proj_matrix_P00 = data["cameraProjection"][0]
        proj_matrix_P11 = data["cameraProjection"][5]

        # Calcolo di fx e fy
        # Se P[0][0] = 2 * fx / larghezza_immagine => fx = P[0][0] * larghezza_immagine / 2
        # Se P[1][1] = 2 * fy / altezza_immagine => fy = P[1][1] * altezza_immagine / 2
        fx = proj_matrix_P00 * img_width_px / 2.0
        fy = proj_matrix_P11 * img_height_px / 2.0

        # Calcolo di cx e cy
        # Se cameraProjection[2] (P[0][2]) e cameraProjection[6] (P[1][2]) sono 0,
        # il punto principale (cx, cy) è al centro dell'immagine.
        # Questo è coerente con cameraApertureOffset: [0.0, 0.0].
        cx = img_width_px / 2.0
        cy = img_height_px / 2.0
        
        s = 0.0  # Skew, assunto pari a 0

        K = np.array([
            [fx,  s, cx],
            [0,  fy, cy],
            [0,   0,  1.0]
        ], dtype=np.float64)
        
        return K

    except FileNotFoundError:
        print(f"Errore: File non trovato a '{camera_params_path}'")
        return None
    except (KeyError, IndexError, TypeError) as e:
        print(f"Errore: Dati mancanti o malformattati nel file JSON. Dettagli: {e}")
        return None
    except Exception as e:
        print(f"Si è verificato un errore imprevisto: {e}")
        return None

if __name__ == "__main__":
  
    script_dir = os.path.dirname(os.path.abspath(__file__))
    relative_path_to_json = os.path.join("..", "output", "img1", "camera_params_0000.json")
    json_file_path = os.path.abspath(os.path.join(script_dir, relative_path_to_json))
    
    print(f"Percorso calcolato per il file JSON: {json_file_path}") # Stampa di debug
    
    K_matrix = generate_k_matrix(json_file_path)

    if K_matrix is not None:
        print("Matrice K generata:")
        print(K_matrix)