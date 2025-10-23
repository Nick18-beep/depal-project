"""Image projection helpers for grasp visualization."""

from __future__ import annotations
from typing import Tuple, List
import os, math
import numpy as np
from PIL import Image, ImageDraw


def _quat_to_matrix(q: Tuple[float, float, float, float]) -> np.ndarray:
    """Convert quaternion (w, x, y, z) to 3×3 rotation matrix."""
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-8:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1 - (yy + zz),   xy - wz,         xz + wy],
        [xy + wz,         1 - (xx + zz),   yz - wx],
        [xz - wy,         yz + wx,         1 - (xx + yy)],
    ], float)


def _project_topdown(P: Tuple[float, float, float], h: float, K: np.ndarray):
    """Project 3-D point (X, Y, Z) into image plane (u, v)."""
    X, Y, Z = P
    if Z >= h - 1e-6:
        return None, None, False  # punto sopra/allo stesso livello della camera: invisibile
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    Zc = h - Z
    u = cx + fx * (X / Zc)
    v = cy - fy * (Y / Zc)  # Y+ in avanti ⇒ v decresce verso l’alto
    return u, v, True


def _rotate_sensor(u: float, v: float, cx: float, cy: float, roll_deg: float):
    if abs(roll_deg) < 1e-6:
        return int(round(u)), int(round(v))
    r = math.radians(roll_deg)
    cr, sr = math.cos(r), math.sin(r)
    du, dv = u - cx, v - cy
    u_r = cr * du - sr * dv + cx
    v_r = sr * du + cr * dv + cy
    return int(round(u_r)), int(round(v_r))


def project_circle_topdown(
    *,
    radius_m: float,
    center_world_m: Tuple[float, float, float],
    orientation_wxyz: Tuple[float, float, float, float],
    camera_height_m: float,
    K_matrix_3x3: np.ndarray,
    image_width: int,
    image_height: int,
    output_image_path: str,
    roll_deg: float = 90.0,
    circle_color: Tuple[int, int, int] | int = (255, 255, 255),
    num_segments: int = 64,
    background_rgba: Tuple[int, int, int, int] = (0, 0, 0, 255),
) -> None:
    """Project a *filled* 3-D circle to a top-down view and save the result.


    Se l'immagine di output esiste già, la apre e disegna sopra il cerchio.
    Altrimenti, ne crea una nuova con sfondo specificato.
    """


    # Map integer code → RGB
    _code2rgb = {
    1: (255, 64, 64),   # rosso
    2: (255, 255, 0),   # giallo
    3: (0, 255, 0),     # verde
   -1: (128, 0, 128)    # viola 
}


    # Determina i colori
    if isinstance(circle_color, int):
        fill_rgb = _code2rgb.get(circle_color, (255, 255, 255))
    else:
        # accetta (R,G,B) o (R,G,B,A)
        fill_rgb = tuple(circle_color[:3])


    # ── prepara immagine di output ─────────────────────────────────
    # Se esiste già, la apre; altrimenti la crea da zero
    if os.path.isfile(output_image_path):
        # Apre l'immagine esistente
        img = Image.open(output_image_path).convert("RGBA")
        # Se le dimensioni non corrispondono, ridimensiona per sicurezza
        if img.size != (image_width, image_height):
            img = img.resize((image_width, image_height), resample=Image.BILINEAR)
    else:
        # Crea nuova immagine RGBA con il colore di sfondo
        img = Image.new("RGBA", (image_width, image_height), background_rgba)


    # Matrice di rotazione dal quaternione
    R = _quat_to_matrix(orientation_wxyz)
    e1 = R[:, 0]  # asse locale X
    e2 = R[:, 1]  # asse locale Y


    cx_cam, cy_cam = K_matrix_3x3[0, 2], K_matrix_3x3[1, 2]


    # Calcola i vertici del poligono circolare
    pts_px: List[Tuple[int, int]] = []
    for k in range(num_segments):
        theta = 2 * math.pi * k / num_segments
        offset = radius_m * (math.cos(theta) * e1 + math.sin(theta) * e2)
        P = tuple(np.array(center_world_m) + offset)


        u, v, vis = _project_topdown(P, camera_height_m, K_matrix_3x3)
        if not vis:
            continue
        u_r, v_r = _rotate_sensor(u, v, cx_cam, cy_cam, roll_deg)
        if 0 <= u_r < image_width and 0 <= v_r < image_height:
            pts_px.append((u_r, v_r))


    if len(pts_px) < 3:
        print("[WARN] Circle non visibile o fuori dall'immagine.")
    else:
        draw = ImageDraw.Draw(img)
        # Disegna poligono riempito
        draw.polygon(pts_px, fill=fill_rgb)


    # Se voglio salvare in JPEG, converto prima in RGB
    if output_image_path.lower().endswith((".jpg", ".jpeg")):
        img = img.convert("RGB")


    # Assicuro che la cartella esista e salvo
    os.makedirs(os.path.dirname(os.path.abspath(output_image_path)), exist_ok=True)
    img.save(output_image_path)
    print("[INFO] Cerchio proiettato e salvato in:", output_image_path)





