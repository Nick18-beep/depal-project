import omni.physx
import omni.usd
from pxr import Gf, Usd, UsdGeom
from typing import List, Tuple, Optional

def get_sampled_grasp_poses(
    self,
    target_prim_path: str,
    grid_resolution: int = 5,
    upward_normal_threshold: float = 0.7,
) -> List[Tuple[Gf.Vec3f, Gf.Quatf, Gf.Vec3f]]:
    """
    Campiona la superficie superiore di un oggetto per trovare molteplici pose di presa.

    Utilizza una griglia di raycast proiettata dall'alto e filtra i punti di impatto
    in base all'orientamento della loro normale.

    Args:
        target_prim_path (str): Il path del prim da campionare.
        grid_resolution (int): La densità della griglia di campionamento (es. 5 -> 5x5=25 raggi).
        upward_normal_threshold (float): La soglia del prodotto scalare per considerare
                                         una normale come "rivolta verso l'alto". 
                                         1.0 = perfettamente verticale, 0.0 = orizzontale.

    Returns:
        List[Tuple[Gf.Vec3f, Gf.Quatf, Gf.Vec3f]]: Una lista di pose di presa valide.
        Ogni tupla contiene (posizione_presa, orientamento_presa, normale_superficie).
    """
    valid_grasp_poses = []
    
    if not self._stage:
        print("ERROR (get_sampled_grasp_poses): Stage non disponibile.")
        return valid_grasp_poses

    target_prim = self._stage.GetPrimAtPath(target_prim_path)
    if not target_prim or not target_prim.IsValid():
        print(f"ERROR (get_sampled_grasp_poses): Prim '{target_prim_path}' non valido.")
        return valid_grasp_poses

    # 1. Calcola il Bounding Box Mondiale per definire l'area di campionamento
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    world_bbox = bbox_cache.ComputeWorldBound(target_prim).GetBox()

    if world_bbox.IsEmpty():
        print(f"ERROR: BBox mondiale per '{target_prim_path}' non calcolabile.")
        return valid_grasp_poses

    min_bound = world_bbox.GetMin()
    max_bound = world_bbox.GetMax()
    bbox_size = world_bbox.GetSize()
    
    # Prepara l'interfaccia per il raycast
    physx_query_interface = omni.physx.get_physx_scene_query_interface()
    world_up_vector = Gf.Vec3f(0, 0, 1)

    # 2. Itera sulla griglia di campionamento
    for i in range(grid_resolution):
        for j in range(grid_resolution):
            # Calcola il punto (x, y) sulla griglia
            # Usiamo un piccolo margine per evitare i bordi esatti
            t_x = (i + 0.5) / grid_resolution
            t_y = (j + 0.5) / grid_resolution
            
            sample_x = min_bound[0] + t_x * bbox_size[0]
            sample_y = min_bound[1] + t_y * bbox_size[1]
            
            # 3. Definisci e esegui il raycast per questo punto
            ray_start = Gf.Vec3d(sample_x, sample_y, max_bound[2] + 0.1 * bbox_size[2])
            ray_dir = Gf.Vec3d(0, 0, -1.0)
            ray_max_dist = bbox_size[2] * 1.2

            hit = physx_query_interface.raycast_closest(ray_start, ray_dir, ray_max_dist)

            if not hit or hit["hit"] is None:
                continue # Nessun impatto, passa al prossimo punto della griglia

            # Assicurati di aver colpito il prim corretto o un suo figlio
            hit_prim_path = str(hit["rigid_body"])
            if not hit_prim_path.startswith(target_prim_path):
                continue
                
            # 4. Estrai i dati e filtra in base alla normale
            surface_normal_world = Gf.Vec3f(hit["normal"]).GetNormalized()
            
            dot_product = Gf.Dot(surface_normal_world, world_up_vector)
            
            if dot_product >= upward_normal_threshold:
                # La normale è valida, procedi a calcolare la posa di presa completa
                surface_point_world = Gf.Vec3f(hit["position"])
                
                # 5. Calcola la posa di presa (logica riutilizzata)
                cone_axis_local = Gf.Vec3d(0, 0, 1)
                target_normal_world_d = Gf.Vec3d(surface_normal_world)
                
                rotation_to_align_cone_d = Gf.Rotation(cone_axis_local, target_normal_world_d)
                if Gf.Dot(cone_axis_local, target_normal_world_d) < -0.999:
                    ortho_axis = Gf.Cross(cone_axis_local, Gf.Vec3d.XAxis())
                    if ortho_axis.GetLengthSq() < 1e-6:
                        ortho_axis = Gf.Cross(cone_axis_local, Gf.Vec3d.YAxis())
                    rotation_to_align_cone_d = Gf.Rotation(ortho_axis.GetNormalized(), 180.0)

                cone_orientation = Gf.Quatf(rotation_to_align_cone_d.GetQuat())
                cone_origin = surface_point_world + surface_normal_world * self.grasp_offset_from_top

                # Aggiungi la posa valida alla lista
                valid_grasp_poses.append((cone_origin, cone_orientation, surface_normal_world))

    if self.debug_pose_calculation:
        print(f"  Trovate {len(valid_grasp_poses)} pose di presa valide per '{target_prim_path}' "
              f"con risoluzione {grid_resolution}x{grid_resolution}.")

    return valid_grasp_poses