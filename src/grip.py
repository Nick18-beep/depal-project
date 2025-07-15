from __future__ import annotations

from typing import Optional, Tuple, List, Dict, Set
from pxr import Usd, UsdGeom, Gf, Vt
import omni.usd

import numpy as np
import omni
import omni.kit.usd
import omni.physx as _physx
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Usd
from pxr import Gf, UsdGeom, UsdPhysics, Sdf, UsdShade, Usd,UsdPhysics, PhysxSchema, Gf
import traceback
import math
from isaacsim.robot.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)
from isaacsim.core.prims import SingleRigidPrim

from omni.physx.scripts import physicsUtils as phys_utils

            # ──────────────────────────────────────────────────────────────
from omni.physx import get_physx_simulation_interface
from omni.physx.scripts import physicsUtils as phys_utils

def _quat_identity() -> Gf.Quatf:
    return Gf.Quatf(1.0, 0.0, 0.0, 0.0)

class SurfaceGripperDirectScript:
    def __init__(
        self,
        target_object_prim_path: Optional[str] = None,
        grasp_offset_from_top: float = 0.000, 
        gripper_depth_offset: float = 0.005, #0.005
        initial_box_prim_path: str = "/World/Box", 
        box_mass: float = 0.05, 
        absolute_grip_force: Optional[float] = None,
        grip_force_multiplier: float = 50.0,
        target_lift_height: float = 0.5,
        target_horizontal_position: np.ndarray = np.array([0.3, 0.3]),
        z_correction_factor: float = 10.0,
        max_z_correction_speed_factor: float = 1.5,
        cone_height: float = 0.1,
        cone_radius: float = 0.05,
        lift_speed_vertical: float = 0.5,
        move_speed_horizontal: float = 0.1,
        steps_for_cube_to_settle: int = 60,
        steps_grace_period_after_settle: int = 30,
        steps_wait_before_grip_attempt: int = 120,
        steps_wait_after_cone_usd_creation: int = 5,#5
        steps_wait_after_grip_refresh: int = 40,
        debug_pose_calculation: bool = False,
        simulation_app = None
    ):
        self._timeline = omni.timeline.get_timeline_interface()
        self._usd_context = omni.usd.get_context()
        self._physx_interface = _physx.get_physx_interface()
        self._stage: Optional[omni.usd.Stage] =  self._usd_context.get_stage()

        self.surface_gripper: Optional[Surface_Gripper] = None
        self.cone_prim_handle: Optional[SingleRigidPrim] = None
        self.target_srp_handle: Optional[SingleRigidPrim] = None
        self.cone_geom_usd: Optional[UsdGeom.Cone] = None
        self._physx_sub = None
        self._sim_step: int = 0
        self.debug_pose_calculation = debug_pose_calculation
        
        self.fallback_box_size = 0.1 # Usato solo se le dimensioni non possono essere determinate

        self.target_object_prim_path = target_object_prim_path
        self.grasp_offset_from_top = float(grasp_offset_from_top)
        self.initial_box_prim_path = initial_box_prim_path
        self.object_to_grasp_path = target_object_prim_path if target_object_prim_path else self.initial_box_prim_path

        self.box_mass = float(box_mass) 
        self.absolute_grip_force = float(absolute_grip_force) if absolute_grip_force is not None else None
        self.grip_force_multiplier = float(grip_force_multiplier)
        self.target_lift_height = float(target_lift_height)
        self.target_horizontal_position = np.array(target_horizontal_position, dtype=np.float64)

        self.z_correction_factor = float(z_correction_factor)
        self.max_z_correction_speed_factor = float(max_z_correction_speed_factor)
        self.cone_height = float(cone_height)
        self.cone_radius = float(cone_radius)
        self.lift_speed_vertical = float(lift_speed_vertical)
        self.move_speed_horizontal = float(move_speed_horizontal)

        self.steps_for_cube_to_settle = int(steps_for_cube_to_settle)
        self.steps_end_settle = self.steps_for_cube_to_settle
        self.steps_end_grace_period = self.steps_end_settle + int(steps_grace_period_after_settle)
        self.steps_for_cone_usd_creation_attempt = self.steps_end_grace_period + int(steps_wait_before_grip_attempt)
        self.steps_wait_after_cone_usd_creation = int(steps_wait_after_cone_usd_creation)
        self.steps_for_sg_init_and_grip_attempt = self.steps_for_cone_usd_creation_attempt + self.steps_wait_after_cone_usd_creation
        self.steps_before_initial_lift_phase = self.steps_for_sg_init_and_grip_attempt + int(steps_wait_after_grip_refresh)

        self.gripper_close_command_sent: bool = False
        self.cone_usd_prim_created: bool = False
        self.cone_sg_initialized: bool = False
        self.cone_physics_dynamically_enabled: bool = False
        self.task_complete= False

        self.color_closed = Gf.Vec3f(1.0, 0.2, 0.2)
        self.color_open = Gf.Vec3f(0.2, 1.0, 0.2)

        self.movement_phase: str = "idle"
        self.lift_start_z: Optional[float] = None
        self.target_descent_z: Optional[float] = None

        self._sgp_offset_p_local_to_cone = Gf.Vec3f(0.0, 0.0, float(gripper_depth_offset))
        rotation_as_quatd = Gf.Rotation(Gf.Vec3d(0,1,0), 90.0).GetQuat() 
        real_part_sgp = float(rotation_as_quatd.GetReal())
        imag_part_sgp_d = rotation_as_quatd.GetImaginary() 
        self._sgp_offset_r_local_to_cone = Gf.Quatf(
            real_part_sgp,
            Gf.Vec3f(float(imag_part_sgp_d[0]),
                     float(imag_part_sgp_d[1]),
                     float(imag_part_sgp_d[2]))
        )

        self.grip_status_code=0
        self._contact_sub = None
        self.simulation_app=simulation_app
        self.grasp_pone_cone=None
        

        

        

        print("SurfaceGripperDirectScript instance created:")
        print(f"  Target: {self.object_to_grasp_path}")
        print(f"  Cone Base to Surface Offset (grasp_offset_from_top): {self.grasp_offset_from_top:.4f} m")
        print(f"  SGP offset local to cone (position, in cone frame Z up): {self._sgp_offset_p_local_to_cone}")
        sgp_rot_img_init = self._sgp_offset_r_local_to_cone.GetImaginary()
        sgp_euler_deg = Gf.Rotation(self._sgp_offset_r_local_to_cone).Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())[2]
        print(f"  SGP offset local to cone (rotation quat WXYZ): {self._sgp_offset_r_local_to_cone.GetReal():.4f}, {sgp_rot_img_init[0]:.4f}, {sgp_rot_img_init[1]:.4f}, {sgp_rot_img_init[2]:.4f} (Euler XYZ deg ~ {sgp_euler_deg})")
        print(f"  Timing: Settle: {self.steps_end_settle}, End Grace: {self.steps_end_grace_period}")
        print(f"  Timing: Cone USD Creation Attempt: {self.steps_for_cone_usd_creation_attempt}")
        print(f"  Timing: SG Init & Grip Attempt: {self.steps_for_sg_init_and_grip_attempt} (after {self.steps_wait_after_cone_usd_creation} steps for cone USD to settle)")
        print(f"  Timing: Lift Start: {self.steps_before_initial_lift_phase}")




    def _get_target_object_grasp_pose(
            self, current_target_prim_path: str
    ) -> Optional[Tuple[Gf.Vec3f, Gf.Quatf, Gf.Vec3f]]:
        """
        Calcola la posa di grasp per il prim indicato. Supporta:
            • UsdGeom.Mesh  (qualsiasi mesh triangolare/quad)
            • UsdGeom.Cube  (primitives “Cube” di USD)

        Restituisce (cone_origin, cone_orientation, surface_normal_world)
        nelle stesse unità/spazi usati dal resto della classe.

        Se nessuna delle due geometrie è trovata, fa fallback
        al vecchio algoritmo basato sul bounding-box.
        """
        if not self._stage:
            print("ERROR: Stage non disponibile.")
            return None

        prim = self._stage.GetPrimAtPath(current_target_prim_path)
        if not prim or not prim.IsValid():
            print(f"ERROR: Prim '{current_target_prim_path}' non valido.")
            return None

        # ────────────────────────────────
        # 1. Cerca una Mesh o un Cube nel sotto-albero
        # ────────────────────────────────
        mesh_prim = cube_prim = None
        queue = [prim]
        while queue and not (mesh_prim or cube_prim):
            p = queue.pop(0)
            if p.IsA(UsdGeom.Mesh):
                mesh_prim = p
                break
            if p.IsA(UsdGeom.Cube):
                cube_prim = p
                # non usciamo subito: diamo priorità a una Mesh eventuale
            queue.extend(p.GetChildren())

        # ────────────────────────────────
        # 2a. Caso Mesh  → algoritmo completo sulle facce
        # ────────────────────────────────
        if mesh_prim:
            mesh = UsdGeom.Mesh(mesh_prim)
            world_Xf = omni.usd.get_world_transform_matrix(mesh_prim)

            pts_local = mesh.GetPointsAttr().Get()
            if not pts_local:
                print("ERROR: Mesh senza punti validi.")
                return None
            pts_world = [Gf.Vec3f(world_Xf.Transform(Gf.Vec3d(p))) for p in pts_local]

            face_counts = mesh.GetFaceVertexCountsAttr().Get()
            face_indices = mesh.GetFaceVertexIndicesAttr().Get()
            if not face_counts or not face_indices:
                print("ERROR: Mesh senza indici faccia.")
                return None

            up_world = Gf.Vec3f(0, 0, 1)
            best = {"dot": -2.0, "centroid": None, "normal": None}

            idx = 0
            for n_v in face_counts:
                v_idx = face_indices[idx: idx + n_v]
                idx += n_v
                if len(v_idx) < 3:
                    continue
                v0, v1, v2 = (pts_world[v_idx[0]],
                            pts_world[v_idx[1]],
                            pts_world[v_idx[2]])
                n = Gf.Cross(v1 - v0, v2 - v0)
                if n.GetLengthSq() < 1e-12:
                    continue
                n.Normalize()
                dot = Gf.Dot(n, up_world)
                if dot <= 0:
                    continue
                centroid = Gf.Vec3f(sum((pts_world[i] for i in v_idx),
                                        Gf.Vec3f(0)).GetArray() / len(v_idx))
                if dot > best["dot"]:
                    best = {"dot": dot, "centroid": centroid, "normal": n}

            if best["centroid"] is None:
                print("WARN: Nessuna faccia con normale +Z in Mesh; uso bounding-box.")
                return super()._get_target_object_grasp_pose(current_target_prim_path)  # type: ignore

            surface_p_w = best["centroid"]
            surface_n_w = best["normal"]

        # ────────────────────────────────
        # 2b. Caso Cube  → faccia superiore implicita
        # ────────────────────────────────
        elif cube_prim:
            cube = UsdGeom.Cube(cube_prim)
            size = cube.GetSizeAttr().Get() or 1.0  # default: 1 m
            world_Xf = omni.usd.get_world_transform_matrix(cube_prim)

            # Centro della faccia +Z nel local-space del cubo: (0,0,+size/2)
            local_top_center = Gf.Vec3d(0, 0, size * 0.5)
            surface_p_w = Gf.Vec3f(world_Xf.Transform(local_top_center))

            # Normale = asse +Z trasformato dal world_Xf
            surface_n_w = Gf.Vec3f(world_Xf.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()

        # ────────────────────────────────
        # 2c. Nessuna geometria → fallback
        # ────────────────────────────────
        else:
            return super()._get_target_object_grasp_pose(current_target_prim_path)  # type: ignore

        # ────────────────────────────────
        # 3. Orientazione e origine del cono
        # ────────────────────────────────
        rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(surface_n_w))
        q = rot.GetQuat().GetNormalized()
        cone_orientation = Gf.Quatf(float(q.GetReal()),
                                    Gf.Vec3f(float(q.GetImaginary()[0]),
                                            float(q.GetImaginary()[1]),
                                            float(q.GetImaginary()[2])))

        cone_origin = surface_p_w + surface_n_w * self.grasp_offset_from_top
        return cone_origin, cone_orientation, surface_n_w











    


    



    


    # ──────────────────────────────────────────────────────────────────────────────
    #  helpers geometrici
    # ──────────────────────────────────────────────────────────────────────────────
    def _point_in_triangle(self,p: Gf.Vec3f, a: Gf.Vec3f, b: Gf.Vec3f, c: Gf.Vec3f) -> bool:
        """test barycentrico 2D/3D (funziona anche in 3D)"""
        v0, v1, v2 = c - a, b - a, p - a
        dot00 = Gf.Dot(v0, v0); dot01 = Gf.Dot(v0, v1); dot02 = Gf.Dot(v0, v2)
        dot11 = Gf.Dot(v1, v1); dot12 = Gf.Dot(v1, v2)
        denom = dot00 * dot11 - dot01 * dot01
        if abs(denom) < 1e-12:
            return False
        invd = 1.0 / denom
        u = (dot11 * dot02 - dot01 * dot12) * invd
        v = (dot00 * dot12 - dot01 * dot02) * invd
        return (u >= 0.0) and (v >= 0.0) and (u + v <= 1.0)


    def _ray_intersect_tri(self,o: Gf.Vec3f, dir: Gf.Vec3f,
                        a: Gf.Vec3f, b: Gf.Vec3f, c: Gf.Vec3f, n: Gf.Vec3f):
        """intersezione raggio–triangolo; ritorna (t, p) o None"""
        dn = Gf.Dot(dir, n)
        if abs(dn) < 1e-8:        # quasi parallelo
            return None
        t = Gf.Dot(a - o, n) / dn
        if t < 0:
            return None
        p = o + dir * t
        if self._point_in_triangle(p, a, b, c):
            return t, p
        return None


    # ──────────────────────────────────────────────────────────────────────────────
    #  FUNZIONE PRINCIPALE
    # ──────────────────────────────────────────────────────────────────────────────
    def sample_grasp_grid(
        self,
        prim_path: str,
        grid_step_m: float = 0.1,#0.1
        max_tilt_deg: float = 65.0,
        stage_time: Usd.TimeCode = Usd.TimeCode.Default(),
    ) -> List[Tuple[Gf.Vec3f, Gf.Quatf, Gf.Vec3f]]:
        """
        Griglia XY → proiezione verticale → un solo punto per nodo.
        Evita sovracampionamenti sugli spigoli e copre tutta la superficie.
        """
        _UP = Gf.Vec3f(0, 0, 1)
        stage = self._stage
        if not stage:
            print("ERROR: Stage non disponibile.")
            return []

        root = stage.GetPrimAtPath(prim_path)
        if not root or not root.IsValid():
            print(f"ERROR: Prim '{prim_path}' non valido.")
            return []

        up_dot_min = math.cos(math.radians(max_tilt_deg))

        # ------------------------------------------------------------------
        # 1. estrai tutti i triangoli (Mesh + Cube) + normali
        # ------------------------------------------------------------------
        tris = []   # (a,b,c,n)
        vmin = Gf.Vec3f(float("inf"))
        vmax = Gf.Vec3f(float("-inf"))

        stack = [root]
        while stack:
            prim = stack.pop()
            xf_world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(stage_time)

            # --------- Mesh -----------------------------------------------------
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                pts_l = mesh.GetPointsAttr().Get(stage_time) or []
                if not pts_l:
                    stack.extend(prim.GetChildren()); continue
                pts_w = [Gf.Vec3f(xf_world.Transform(Gf.Vec3d(p))) for p in pts_l]

                f_cnt = mesh.GetFaceVertexCountsAttr().Get() or []
                f_idx = mesh.GetFaceVertexIndicesAttr().Get() or []
                k = 0
                for nv in f_cnt:
                    ids = f_idx[k:k + nv]; k += nv
                    if len(ids) < 3:   continue
                    v0, v1, v2 = (pts_w[ids[0]], pts_w[ids[1]], pts_w[ids[2]])
                    n = Gf.Cross(v1 - v0, v2 - v0)
                    if n.GetLength() < 1e-8:   continue
                    n.Normalize()
                    if Gf.Dot(n, _UP) < up_dot_min:   continue
                    for i in range(1, nv - 1):
                        a, b, c = v0, pts_w[ids[i]], pts_w[ids[i + 1]]
                        tris.append((a, b, c, n))
                        for v in (a, b):
                            vmin = Gf.Vec3f(min(vmin[0], v[0]),
                                            min(vmin[1], v[1]),
                                            min(vmin[2], v[2]))
                            vmax = Gf.Vec3f(max(vmax[0], v[0]),
                                            max(vmax[1], v[1]),
                                            max(vmax[2], v[2]))

            # --------- Cube -----------------------------------------------------
            elif prim.IsA(UsdGeom.Cube):
                cube = UsdGeom.Cube(prim)
                s    = cube.GetSizeAttr().Get(stage_time) or 1.0
                h    = s * 0.5
                vL   = [Gf.Vec3d(-h,-h,-h), Gf.Vec3d(h,-h,-h), Gf.Vec3d(h,h,-h), Gf.Vec3d(-h,h,-h),
                        Gf.Vec3d(-h,-h,h),  Gf.Vec3d(h,-h,h),  Gf.Vec3d(h,h,h),  Gf.Vec3d(-h,h,h)]
                vW   = [Gf.Vec3f(xf_world.Transform(p)) for p in vL]
                it   = xf_world.GetInverse().GetTranspose()
                face_vidx = [[4,5,6,7],[0,1,2,3],[1,5,6,2],[0,4,7,3],[3,2,6,7],[0,1,5,4]]
                local_ns  = [Gf.Vec3d(0,0,1),Gf.Vec3d(0,0,-1),Gf.Vec3d(1,0,0),
                            Gf.Vec3d(-1,0,0),Gf.Vec3d(0,1,0),Gf.Vec3d(0,-1,0)]
                for vids, nloc in zip(face_vidx, local_ns):
                    n = Gf.Vec3f(it.TransformDir(nloc)).GetNormalized()
                    if Gf.Dot(n, _UP) < up_dot_min:  continue
                    quads = [(vids[0],vids[1],vids[2]),(vids[0],vids[2],vids[3])]
                    for i0,i1,i2 in quads:
                        a,b,c = vW[i0], vW[i1], vW[i2]
                        tris.append((a,b,c,n))
                        for v in (a,b):
                            vmin = Gf.Vec3f(min(vmin[0], v[0]), min(vmin[1], v[1]), min(vmin[2], v[2]))
                            vmax = Gf.Vec3f(max(vmax[0], v[0]), max(vmax[1], v[1]), max(vmax[2], v[2]))

            stack.extend(prim.GetChildren())

        if not tris:
            print(f"WARN: nessuna faccia entro {max_tilt_deg}°.")
            return []

        # ------------------------------------------------------------------
        # 2. griglia su XY e intersezione verticale
        # ------------------------------------------------------------------
        step = grid_step_m
        min_x, max_x = vmin[0] - step*0.5, vmax[0] + step*0.5
        min_y, max_y = vmin[1] - step*0.5, vmax[1] + step*0.5
        cols = int(math.ceil((max_x - min_x) / step))
        rows = int(math.ceil((max_y - min_y) / step))

        dir_down = Gf.Vec3f(0, 0, -1)
        z_high   = vmax[2] + step   # punto di partenza raggi

        poses: List[Tuple[Gf.Vec3f, Gf.Quatf, Gf.Vec3f]] = []

        for ix in range(cols + 1):
            x = min_x + ix * step
            for iy in range(rows + 1):
                y = min_y + iy * step
                o = Gf.Vec3f(x, y, z_high)

                best_t, best_hit, best_n = None, None, None
                for a, b, c, n in tris:
                    hit = self._ray_intersect_tri(o, dir_down, a, b, c, n)
                    if hit is None:  continue
                    t, p = hit
                    if (best_t is None) or (t < best_t):
                        best_t, best_hit, best_n = t, p, n

                if best_hit is None:
                    continue  # il nodo di griglia cade fuori dall'oggetto

                # orientazione: asse Z → normale
                rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(best_n))
                qd  = rot.GetQuat().GetNormalized()
                q   = Gf.Quatf(float(qd.GetReal()),
                            Gf.Vec3f(float(qd.GetImaginary()[0]),
                                        float(qd.GetImaginary()[1]),
                                        float(qd.GetImaginary()[2])))

                origin = best_hit + best_n * self.grasp_offset_from_top
                poses.append((origin, q, best_n))

        return poses


























   
    # ... (il resto del codice da _refresh_prim_handle in poi rimane invariato rispetto alla versione precedente) ...
    def _refresh_prim_handle(self, prim_path: str, current_handle: Optional[SingleRigidPrim]) -> Optional[SingleRigidPrim]:
        if current_handle and current_handle.is_valid():
            return current_handle
        current_stage = self._stage if self._stage else self._usd_context.get_stage()
        if not current_stage:
            if self._sim_step % 120 == 0: print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): Stage not available.")
            return None
       
        usd_prim = current_stage.GetPrimAtPath(prim_path)
        if not usd_prim.IsValid():
            if self._sim_step > 10 and self._sim_step % (120 if prim_path == "/GripperCone" else 60) == 1 :
                 print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): USD prim '{prim_path}' NOT VALID.")
            return None
        try:
            new_handle = SingleRigidPrim(prim_path)
            if not new_handle.is_valid():
                if self._sim_step % 60 == 0: print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): New SRP '{prim_path}' INVALID after creation.")
                return None
            return new_handle
        except RuntimeError as e_refresh:
            if self._sim_step % 60 == 0 and self.debug_pose_calculation: 
                 print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): RuntimeError creating SingleRigidPrim: {e_refresh}")
            return None
        except Exception as e_generic:
            print(f"SIM_STEP {self._sim_step} (_refresh_prim_handle for {prim_path}): EXCEPTION creating SingleRigidPrim: {e_generic}")
            return None

    def _create_rigid_body_usd(
        self, body_geom_type: type[UsdGeom.Gprim], prim_path: str, *, mass: float, position: Gf.Vec3f,
        orientation: Gf.Quatf, color: Gf.Vec3f, height: Optional[float] = None,
        radius: Optional[float] = None, size: Optional[float] = None, 
    ) -> Optional[UsdGeom.Gprim]:
        if not self._stage:
            print(f"ERROR (_create_rigid_body_usd for {prim_path}): Stage not available.")
            return None
       
        if not prim_path.startswith("/"):
            prim_path = "/" + prim_path
            print(f"WARN (_create_rigid_body_usd): Relative prim path for {body_geom_type.__name__}, converted to: {prim_path}")

        geom_prim_path = Sdf.Path(prim_path)
        
        if self._stage.GetPrimAtPath(geom_prim_path).IsValid():
            try:
                omni.kit.commands.execute("DeletePrims", paths=[str(geom_prim_path)])
                if self.debug_pose_calculation: print(f"  (_create_rigid_body_usd) Deleted existing prim at {geom_prim_path} before recreation.")
            except Exception as e_del_recreate:
                print(f"  WARN (_create_rigid_body_usd): Could not delete existing prim {geom_prim_path}: {e_del_recreate}")

        geom: UsdGeom.Gprim = body_geom_type.Define(self._stage, geom_prim_path)
        if not geom.GetPrim().IsValid():
            print(f"ERROR (_create_rigid_body_usd for {prim_path}): Failed to define prim.")
            return None

        prim = geom.GetPrim()
        xform = UsdGeom.Xformable(geom)
        xform.ClearXformOpOrder() 
        
        translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat)
        translate_op.Set(position)

        orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
        orient_op.Set(orientation)
        
        scale_op = xform.AddScaleOp(UsdGeom.XformOp.PrecisionFloat)
        scale_op.Set(Gf.Vec3f(1.0,1.0,1.0))

        if isinstance(geom, UsdGeom.Cone):
            if height is not None: geom.CreateHeightAttr().Set(float(height))
            if radius is not None: geom.CreateRadiusAttr().Set(float(radius))
            axis_attr_usd = geom.GetAxisAttr() 
            if not axis_attr_usd.IsDefined() or not axis_attr_usd.HasValue():
                geom.CreateAxisAttr().Set(UsdGeom.Tokens.z) 
            elif axis_attr_usd.Get() != UsdGeom.Tokens.z:
                 axis_attr_usd.Set(UsdGeom.Tokens.z)
        elif isinstance(geom, UsdGeom.Cube):
            if size is not None: geom.CreateSizeAttr().Set(float(size))

        disp_color_primvar = UsdGeom.PrimvarsAPI(geom).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant
        )
        if disp_color_primvar:
            disp_color_primvar.Set([color])
        else: print(f"WARN: Could not create or get displayColor Primvar for {prim_path}.")

        UsdPhysics.CollisionAPI.Apply(prim)
        if mass > 0:
            rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            if rb_api:
                mass_api = UsdPhysics.MassAPI.Apply(prim)
                if mass_api:
                    mass_api.CreateMassAttr().Set(float(mass))
                else: print(f"WARN: Could not apply MassAPI to {prim_path}.")
            else: print(f"WARN: Could not apply RigidBodyAPI to {prim_path}.")
        return geom

    def _setup_initial_scene(self):
        self._stage = self._usd_context.get_stage()
        if not self._stage:
            print("ERROR (_setup_initial_scene): Stage could not be retrieved/created. Attempting to create new stage.")
            omni.usd.create_new_stage()
            self._stage = self._usd_context.get_stage()
            if not self._stage:
                print("FATAL (_setup_initial_scene): Still no stage after attempting creation. Cannot proceed.")
                return

        cone_prim_path_to_delete = Sdf.Path("/GripperCone")
        if self._stage.GetPrimAtPath(cone_prim_path_to_delete).IsValid():
            try:
                joint_path_to_delete = Sdf.Path(str(cone_prim_path_to_delete) + "/SurfaceGripperJoint")
                paths_to_delete = [str(cone_prim_path_to_delete)]
                if self._stage.GetPrimAtPath(joint_path_to_delete).IsValid():
                    paths_to_delete.append(str(joint_path_to_delete))
               
                omni.kit.commands.execute("DeletePrims", paths=paths_to_delete)
                print(f"Deleted existing prims {paths_to_delete} from previous run.")
            except Exception as e_del:
                print(f"Error deleting prims {paths_to_delete}: {e_del}")
       
        if self._physx_sub is not None:
            try:
                self._physx_sub.unsubscribe()
                print("Unsubscribed from previous physics step events.")
            except Exception as e_unsub:
                print(f"Error unsubscribing previous physics step: {e_unsub}")
            self._physx_sub = None

        self._physx_sub = self._physx_interface.subscribe_physics_step_events(self._on_simulation_step)
       
        if not self._timeline.is_playing():
            self._timeline.play()
           
        print(f"Initial scene setup. Object to grasp: {self.object_to_grasp_path}. Sim (re)started.")
        print(f"Will wait {self.steps_for_cube_to_settle} steps for objects to settle.")

    def _create_cone_usd_prim(self):
        if not self._stage:
            print("WARN (_create_cone_usd_prim): Stage not ready. Skipping cone USD prim creation.")
            self.cone_usd_prim_created = False
            return

        self.target_srp_handle = self._refresh_prim_handle(self.object_to_grasp_path, self.target_srp_handle)
        if not self.target_srp_handle or not self.target_srp_handle.is_valid():
             print(f"WARN (_create_cone_usd_prim): Target '{self.object_to_grasp_path}' handle not ready. Skipping.")
             self.cone_usd_prim_created = False
             return

        grasp_pose_data = self.grasp_pone_cone#self._get_target_object_grasp_pose(self.object_to_grasp_path)
        if not grasp_pose_data:
            print(f"ERROR (_create_cone_usd_prim): Could not calculate grasp pose for '{self.object_to_grasp_path}'. Skipping.")
            self.cone_usd_prim_created = False
            return
        cone_start_pos_gf, cone_orientation_gf, _ = grasp_pose_data 

        self.initial_cone_placement_position_world = np.array([
            cone_start_pos_gf[0],
            cone_start_pos_gf[1],
            cone_start_pos_gf[2]
        ], dtype=np.float32)

        cone_rot_img = cone_orientation_gf.GetImaginary()
        print(f"Creating cone USD prim at Pos: {cone_start_pos_gf}, Rot (Quatf WXYZ): {cone_orientation_gf.GetReal():.4f}, {cone_rot_img[0]:.4f}, {cone_rot_img[1]:.4f}, {cone_rot_img[2]:.4f}")

        self.initial_cone_orientation = (cone_orientation_gf.GetReal(),cone_rot_img[0],cone_rot_img[1],cone_rot_img[2])
        
        cone_geom = self._create_rigid_body_usd(
            UsdGeom.Cone, "/GripperCone", mass=0.01, position=cone_start_pos_gf, 
            orientation=cone_orientation_gf, color=self.color_open, 
            height=self.cone_height, radius=self.cone_radius
        )
        if not cone_geom:
            print("ERROR (_create_cone_usd_prim): Failed to create GripperCone prim.")
            self.cone_usd_prim_created = False
            return
        self.cone_geom_usd = UsdGeom.Cone(cone_geom.GetPrim())

        cone_prim_usd = self.cone_geom_usd.GetPrim()
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cone_prim_usd)

        
        #creo collione check
        collision_api=UsdPhysics.CollisionAPI.Apply(cone_prim_usd)
        

        api = PhysxSchema.PhysxContactReportAPI.Apply(cone_prim_usd)
        api.CreateThresholdAttr().Set(0.0)  

            


        if rigid_api.GetPrim().IsValid():
            kinematic_attr = rigid_api.CreateKinematicEnabledAttr()
            kinematic_attr.Set(True, Usd.TimeCode.Default())
            print(f"Physics for {cone_prim_usd.GetPath()} initially set to KINEMATIC.")
            self.cone_usd_prim_created = True
            self.cone_physics_dynamically_enabled = False
            self.step_of_cone_creation=self._sim_step
        else:
            print(f"ERROR (_create_cone_usd_prim): Could not apply RigidBodyAPI for {cone_prim_usd.GetPath()}.")
            self.cone_usd_prim_created = False

    def _initialize_surface_gripper_and_attempt_close(self):
        if not self.cone_usd_prim_created or not self.cone_geom_usd or not self.cone_geom_usd.GetPrim().IsValid():
            print("ERROR (_initialize_surface_gripper_and_attempt_close): Cone USD prim not created or invalid. Cannot initialize SG.")
            self.cone_sg_initialized = False
            return

        print(f"SIM_STEP {self._sim_step}: Attempting to initialize SurfaceGripper for /GripperCone.")
        sgp = Surface_Gripper_Properties()
        sgp.d6JointPath = "/GripperCone/SurfaceGripperJoint"
        sgp.parentPath = "/GripperCone" 
       
        sgp.offset.p.x, sgp.offset.p.y, sgp.offset.p.z = self._sgp_offset_p_local_to_cone[0], self._sgp_offset_p_local_to_cone[1], self._sgp_offset_p_local_to_cone[2]
        
        sg_rot_q_f = self._sgp_offset_r_local_to_cone
        sgp.offset.r.w = sg_rot_q_f.GetReal()
        sg_rot_img_gripper = sg_rot_q_f.GetImaginary() 
        sgp.offset.r.x, sgp.offset.r.y, sgp.offset.r.z = sg_rot_img_gripper[0], sg_rot_img_gripper[1], sg_rot_img_gripper[2]
       
        sgp.gripThreshold = 0.05 #0.01 

        mass_of_object_to_grip = self.box_mass 
        if self.target_srp_handle and self.target_srp_handle.is_valid() and self._stage:
            try:
                target_prim_for_mass = self._stage.GetPrimAtPath(self.target_srp_handle.prim_path)
                if target_prim_for_mass.HasAPI(UsdPhysics.MassAPI):
                    mass_api = UsdPhysics.MassAPI(target_prim_for_mass)
                    mass_attr = mass_api.GetMassAttr()
                    if mass_attr and mass_attr.IsDefined() and mass_attr.HasValue():
                        mass_val = mass_attr.Get(Usd.TimeCode.Default())
                        if isinstance(mass_val, (float, int)) and mass_val > 0:
                            mass_of_object_to_grip = float(mass_val)
            except Exception as e_mass: print(f"WARN: Could not get mass for target '{self.target_srp_handle.prim_path}': {e_mass}")
        print(f"Target object mass for grip calc: {mass_of_object_to_grip:.3f} kg")

        if self.absolute_grip_force is not None:
            sgp.forceLimit = self.absolute_grip_force
            print(f"Using ABSOLUTE grip force: {sgp.forceLimit:.2f} N")
        else:
            min_force_to_hold = mass_of_object_to_grip * 9.81 
            sgp.forceLimit = min_force_to_hold * self.grip_force_multiplier
            print(f"Using MULTIPLIER ({self.grip_force_multiplier}x) on F_grav ({min_force_to_hold:.2f}N). Grip F_limit: {sgp.forceLimit:.2f} N")

        sgp.torqueLimit = 15.0 #100.0
        sgp.stiffness = 300   #1.0e4
        sgp.damping = 50     #1.0e3
        sgp.retryClose = True   

        self.surface_gripper = Surface_Gripper()
        try:
            self.surface_gripper.initialize(sgp)
            self.cone_sg_initialized = True
            print("SurfaceGripper initialized successfully.")

            if not self.gripper_close_command_sent:
                print(f"SIM_STEP {self._sim_step}: SG initialized. Attempting to close gripper...")
                self.surface_gripper.close()
                self.gripper_close_command_sent = True
                print("    INFO: surface_gripper.close() called.")

        except Exception as e_sg_init:
            print(f"ERROR: SurfaceGripper init failed: {e_sg_init}"); traceback.print_exc()
            self.cone_sg_initialized = False
            self.surface_gripper = None


    def _on_simulation_step(self, dt: float):
        self._sim_step += 1
        time_code_default = Usd.TimeCode.Default()


      


        
     

        if ( self.task_complete ):
            return 

        if not self._stage :
            if self._sim_step % 120 == 0: print(f"SIM_STEP {self._sim_step}: Stage invalid. Stopping updates.")
            if self._physx_sub: self._physx_sub.unsubscribe(); self._physx_sub = None
            return
        
        # ---- TIMEOUT CHECK FOR INITIAL GRIP CLOSURE (Code 1) ----
        if not self.task_complete and \
           self.gripper_close_command_sent and \
           self.cone_sg_initialized and \
           self.surface_gripper and not self.surface_gripper.is_closed() and \
           self._sim_step > (self.steps_for_sg_init_and_grip_attempt + 500): #tempo che attenfo prima di ritornare
            print(f"SIM_STEP {self._sim_step}: TIMEOUT - Gripper did not close after {500} additional steps (total steps: {self._sim_step}, init_attempt_step: {self.steps_for_sg_init_and_grip_attempt}). Status: 1.")        
            if(self.grip_status_code ==0):
                self.grip_status_code = 1
                
            #self.movement_phase = "done" # Stop any potential future movement teso un attimo
            self.task_complete = True
            if self.surface_gripper: # Attempt to open it to reset state
                try: self.surface_gripper.open()
                except Exception: pass
            if self.cone_prim_handle and self.cone_prim_handle.is_valid():
                try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32))
                except RuntimeError: pass # Already stopping or error setting velocity
            # Let the function flow to the "done" phase logic at the end

        if not self.task_complete and \
           self.cone_sg_initialized and \
           self.gripper_close_command_sent and \
           self.surface_gripper and self.movement_phase in ["idle","lifting"] : #tempo che attenfo prima di ritornare
          


            psi   = get_physx_simulation_interface()
            


            def _on_contact_report(contact_headers, _contact_data):
                """
                Callback PhysX eseguito a fine step.
                Registra solo i contatti gripper-cone ↔ box_<n>.
                Se nello stesso frame ce ne sono più di due, imposta grip_status_code a –1.
                """
                
                collisions_this_step = 0


                for h in contact_headers:
                    # Decodifica gli ID interi → percorsi USD leggibili
                    path0 = phys_utils.PhysicsSchemaTools.intToSdfPath(h.actor0)
                    path1 = phys_utils.PhysicsSchemaTools.intToSdfPath(h.actor1)


                    # Rendi l’ordine indifferente: gripper è sempre p0, box sempre p1
                    if path0 == "/GripperCone" and str(path1).startswith("/World/SpawnedBasicBoxes/BasicBox_"):
                        collisions_this_step += 1
                        #print(f"[GRIPPER→BOX] cone  <-->  {path1}")
                    elif path1 == "/GripperCone" and str(path0).startswith("/World/SpawnedBasicBoxes/BasicBox_"):
                        collisions_this_step += 1
                        #print(f"[GRIPPER→BOX]  cone <-->  {path0}")

                    # Rendi l’ordine indifferente: gripper è sempre p0, box sempre p1
                    if path0 == "/GripperCone" and str(path1).startswith("/World/GeneratedYCBObjects/SpawnedObject"):
                        collisions_this_step += 1
                        #print(f"[GRIPPER→BOX] cone  <-->  {path1}")
                    elif path1 == "/GripperCone" and str(path0).startswith("/World/GeneratedYCBObjects/SpawnedObject"):
                        collisions_this_step += 1
                        #print(f"[GRIPPER→BOX]  cone <-->  {path0}")

                        


                # Aggiorna il codice di stato
                if collisions_this_step > 1:
                    
                    self.grip_status_code = -1
                    self.movement_phase = "done" # Transita alla fase di rilascio/termine testo un attimo
                    self.task_complete = True
                    
                    print("HO 2 COLLISSIONI STACCO ")
                    
                    
                





         
            # iscrizione (la facciamo una sola volta)
            if self._contact_sub == None:
                self._contact_sub = psi.subscribe_contact_report_events(_on_contact_report)

                print("Contact-report callback registrato.")
           


        if self._sim_step % 120 == 15: 
            if self.target_srp_handle and self.target_srp_handle.is_valid():
                try:
                    box_pos_tuple, box_rot_tuple = self.target_srp_handle.get_world_pose()
                    
                    if box_pos_tuple is not None and len(box_pos_tuple) == 3:
                        box_pos = Gf.Vec3f(float(box_pos_tuple[0]), float(box_pos_tuple[1]), float(box_pos_tuple[2]))
                    else:
                        box_pos = Gf.Vec3f(0.0,0.0,0.0)
                    
                    if box_rot_tuple is not None and len(box_rot_tuple) == 4: 
                        box_rot_q_f = Gf.Quatf(float(box_rot_tuple[0]), Gf.Vec3f(float(box_rot_tuple[1]), float(box_rot_tuple[2]), float(box_rot_tuple[3])))
                    else:
                        box_rot_q_f = _quat_identity()
                        
                    box_rot_img_print = box_rot_q_f.GetImaginary()
                    print(f"SIM_STEP {self._sim_step} DEBUG POSE: Box ('{self.object_to_grasp_path}') Pos={box_pos}, RotQuatWXYZ={box_rot_q_f.GetReal():.4f},{box_rot_img_print[0]:.4f},{box_rot_img_print[1]:.4f},{box_rot_img_print[2]:.4f}")
                except (RuntimeError, TypeError, IndexError, AttributeError) as e: 
                     print(f"SIM_STEP {self._sim_step} DEBUG (Box Pose Error): {type(e).__name__}: {e}")


            current_cone_prim_path_debug = "/GripperCone"
            if self.cone_prim_handle and self.cone_prim_handle.is_valid():
                try:
                    cone_pos_tuple, cone_rot_tuple = self.cone_prim_handle.get_world_pose() 
                    
                    if cone_pos_tuple is not None and len(cone_pos_tuple) == 3:
                        cone_pos = Gf.Vec3f(float(cone_pos_tuple[0]), float(cone_pos_tuple[1]), float(cone_pos_tuple[2]))
                    else:
                        cone_pos = Gf.Vec3f(0.0,0.0,0.0)

                    if cone_rot_tuple is not None and len(cone_rot_tuple) == 4:
                        cone_rot_q_f = Gf.Quatf(float(cone_rot_tuple[0]), Gf.Vec3f(float(cone_rot_tuple[1]), float(cone_rot_tuple[2]), float(cone_rot_tuple[3])))
                    else:
                        cone_rot_q_f = _quat_identity()
                        
                    cone_rot_img_print = cone_rot_q_f.GetImaginary()
                    print(f"SIM_STEP {self._sim_step} DEBUG POSE: Cone ('{current_cone_prim_path_debug}') Handle Pose: Pos={cone_pos}, RotQuatWXYZ={cone_rot_q_f.GetReal():.4f},{cone_rot_img_print[0]:.4f},{cone_rot_img_print[1]:.4f},{cone_rot_img_print[2]:.4f}")
                except (RuntimeError, TypeError, IndexError, AttributeError) as e:
                     print(f"SIM_STEP {self._sim_step} DEBUG (Cone Handle Pose Error): {type(e).__name__}: {e}")
            elif self.cone_geom_usd and self.cone_geom_usd.GetPrim().IsValid(): 
                try:
                    cone_prim_debug = self.cone_geom_usd.GetPrim()
                    cone_world_transform_debug: Gf.Matrix4d = omni.usd.get_world_transform_matrix(cone_prim_debug)
                    translation_dbg_d = cone_world_transform_debug.ExtractTranslation() 
                    cone_pos_debug = Gf.Vec3f(float(translation_dbg_d[0]), float(translation_dbg_d[1]), float(translation_dbg_d[2]))
                    cone_rot_q_d_debug = cone_world_transform_debug.ExtractRotationQuat().GetNormalized() 
                    real_part_dbg = float(cone_rot_q_d_debug.GetReal())
                    imag_part_dbg_d = cone_rot_q_d_debug.GetImaginary() 
                    imag_part_dbg_f = Gf.Vec3f(float(imag_part_dbg_d[0]), float(imag_part_dbg_d[1]), float(imag_part_dbg_d[2]))
                    cone_rot_q_f_debug = Gf.Quatf(real_part_dbg, imag_part_dbg_f)
                    cone_rot_img_print_debug = cone_rot_q_f_debug.GetImaginary()
                    print(f"SIM_STEP {self._sim_step} DEBUG POSE: Cone ('{current_cone_prim_path_debug}') USD Pose: Pos={cone_pos_debug}, RotQuatWXYZ={cone_rot_q_f_debug.GetReal():.4f},{cone_rot_img_print_debug[0]:.4f},{cone_rot_img_print_debug[1]:.4f},{cone_rot_img_print_debug[2]:.4f}")
                except Exception as e: 
                    print(f"SIM_STEP {self._sim_step} DEBUG (Cone USD Pose Error): {type(e).__name__}: {e}")

        self.target_srp_handle = self._refresh_prim_handle(self.object_to_grasp_path, self.target_srp_handle)
        if not (self.target_srp_handle and self.target_srp_handle.is_valid()):
            if self._sim_step % 60 == 0: print(f"SIM_STEP {self._sim_step}: Target '{self.object_to_grasp_path}' handle not valid. Waiting.")
            return

        if not self.cone_usd_prim_created and self._sim_step >= self.steps_for_cone_usd_creation_attempt:
            print(f"SIM_STEP {self._sim_step}: Target settled. Attempting to create cone USD prim at step {self.steps_for_cone_usd_creation_attempt}.")
            self._create_cone_usd_prim()
            if not self.cone_usd_prim_created:
                 if self._sim_step % 10 == 0: print(f"SIM_STEP {self._sim_step}: Cone USD prim creation failed. Will retry.")
                 return
            else: 
                self.cone_prim_handle = self._refresh_prim_handle("/GripperCone", self.cone_prim_handle)



       







        if self.cone_usd_prim_created and not self.cone_sg_initialized and self._sim_step >= self.steps_for_sg_init_and_grip_attempt:
            print(f"SIM_STEP {self._sim_step}: Cone USD prim created. Waiting period over. Attempting to initialize SG at step {self.steps_for_sg_init_and_grip_attempt}.")
            if not self.cone_prim_handle or not self.cone_prim_handle.is_valid():
                self.cone_prim_handle = self._refresh_prim_handle("/GripperCone", self.cone_prim_handle)
                if not (self.cone_prim_handle and self.cone_prim_handle.is_valid()):
                    if self._sim_step % 10 == 0: print(f"SIM_STEP {self._sim_step}: Cone handle for SG init not valid. Will retry.")
                    return 
            self._initialize_surface_gripper_and_attempt_close()
            if not self.cone_sg_initialized:
                if self._sim_step % 10 == 0: print(f"SIM_STEP {self._sim_step}: SurfaceGripper initialization failed. Will retry.")
                return

        if self.cone_usd_prim_created and (not self.cone_prim_handle or not self.cone_prim_handle.is_valid()):
            self.cone_prim_handle = self._refresh_prim_handle("/GripperCone", self.cone_prim_handle)

        if not self.cone_sg_initialized or not self.surface_gripper:
            return 

        try:
            self.surface_gripper.update()
        except Exception as e_update:
            print(f"SIM_STEP {self._sim_step}: ERROR during surface_gripper.update(): {e_update}"); return

        if self.cone_geom_usd and self.cone_geom_usd.GetPrim().IsValid():
            try:
                primvars_api = UsdGeom.PrimvarsAPI(self.cone_geom_usd.GetPrim())
                color_primvar = primvars_api.GetPrimvar("displayColor")
                if not color_primvar or not color_primvar.IsDefined():
                    color_primvar = primvars_api.CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant)
               
                if color_primvar:
                    new_color_gf = self.color_closed if self.surface_gripper.is_closed() else self.color_open 
                    current_color_val_array = color_primvar.Get(time_code_default) 
                    if current_color_val_array is None or len(current_color_val_array) == 0 or not Gf.IsClose(current_color_val_array[0], new_color_gf, 1e-5):
                        color_primvar.Set([new_color_gf], time_code_default)
            except Exception: 
                pass

        if self.gripper_close_command_sent and \
           not self.cone_physics_dynamically_enabled and \
           self.surface_gripper.is_closed() and \
           self._sim_step >= (self.steps_for_sg_init_and_grip_attempt + 5): 
           
            print(f"SIM_STEP {self._sim_step}: Gripper confirmed closed and stable. Attempting to set cone physics to DYNAMIC.")
            cone_prim_to_set_dynamic = None
            if self.cone_prim_handle and self.cone_prim_handle.is_valid() and self._stage:
                cone_prim_to_set_dynamic = self._stage.GetPrimAtPath(self.cone_prim_handle.prim_path)
            elif self.cone_geom_usd and self.cone_geom_usd.GetPrim().IsValid():
                cone_prim_to_set_dynamic = self.cone_geom_usd.GetPrim()

            if cone_prim_to_set_dynamic and cone_prim_to_set_dynamic.IsValid():
                rigid_api = UsdPhysics.RigidBodyAPI(cone_prim_to_set_dynamic) 
                if not rigid_api: rigid_api = UsdPhysics.RigidBodyAPI.Apply(cone_prim_to_set_dynamic)

                if rigid_api.GetPrim().IsValid(): 


                    kinematic_attr = rigid_api.GetKinematicEnabledAttr() 
                    if not kinematic_attr : kinematic_attr = rigid_api.CreateKinematicEnabledAttr() 

                    current_kinematic_state = kinematic_attr.Get(time_code_default)
                    if current_kinematic_state is not False: 
                        kinematic_attr.Set(False, time_code_default) 
                        print(f"    SUCCESS: Cone physics for {cone_prim_to_set_dynamic.GetPath()} set to DYNAMIC.")
                        self.cone_physics_dynamically_enabled = True
                    else: 
                        print(f"    INFO: Cone physics for {cone_prim_to_set_dynamic.GetPath()} was already DYNAMIC.")
                        self.cone_physics_dynamically_enabled = True
                else:
                    print(f"    ERROR: Could not apply/get RigidBodyAPI for {cone_prim_to_set_dynamic.GetPath()} to enable dynamic physics.")
            else:
                cone_path_debug_dyn = self.cone_prim_handle.prim_path if self.cone_prim_handle and self.cone_prim_handle.is_valid() else str(self.cone_geom_usd.GetPath() if self.cone_geom_usd else 'None')
                print(f"    ERROR: Cone prim for dynamic physics not found or invalid (path used: {cone_path_debug_dyn}).")
       
        can_start_moving = (
            self.surface_gripper.is_closed() and
            self.cone_prim_handle and self.cone_prim_handle.is_valid() and
            self.cone_physics_dynamically_enabled
        )

        if self.movement_phase == "idle" and \
           self._sim_step >= self.steps_before_initial_lift_phase and \
           can_start_moving:
            try:
                current_cone_pos_tuple, _ = self.cone_prim_handle.get_world_pose() 
                if current_cone_pos_tuple is None or len(current_cone_pos_tuple) != 3 : raise TypeError("Pose tuple is None or malformed from get_world_pose for movement start")
                self.movement_phase = "lifting"
                self.lift_start_z = float(current_cone_pos_tuple[2]) 
                print(f"SIM_STEP {self._sim_step} (Movement Start): Gripper closed, Cone DYNAMIC. Phase: LIFTING. Start Z: {self.lift_start_z:.4f}, Target Lift Z: {self.target_lift_height:.4f}")
            except (RuntimeError, TypeError, IndexError, AttributeError) as e_get_pose_idle: 
                print(f"SIM_STEP {self._sim_step} (Movement Idle->Lifting): Error getting cone pose: {e_get_pose_idle}. To 'done'.")
                self.movement_phase = "done"

        if self.movement_phase in ["lifting", "moving_horizontal", "descending", "releasing"]:
            if not (self.cone_prim_handle and self.cone_prim_handle.is_valid()):
                if self.movement_phase != "releasing": 
                    print(f"SIM_STEP {self._sim_step} (Movement - Phase {self.movement_phase}): cone_prim_handle invalid. Aborting move to 'done'.")
                self.movement_phase = "done"
                return
           
            if not self.cone_physics_dynamically_enabled and self.movement_phase not in ["releasing", "done"]:
                print(f"SIM_STEP {self._sim_step} (Movement - Phase {self.movement_phase}): Cone is NOT DYNAMIC. Cannot move. Aborting to 'releasing'.")
                self.movement_phase = "releasing" 
                try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32))
                except RuntimeError: pass 
                return

            current_cone_pos_np: Optional[np.ndarray] = None

            if not self.task_complete and self.surface_gripper and not self.surface_gripper.is_closed() and self.movement_phase not in ["releasing", "done"] and self.cone_physics_dynamically_enabled: # Implica che la presa era stata stabilita
            
                print(f"SIM_STEP {self._sim_step} (Movement - Phase {self.movement_phase}): GRIP LOST! Status: 2.")
                if(self.grip_status_code ==0):
                    self.grip_status_code = 2
                    
                #self.movement_phase = "releasing" # Transita alla fase di rilascio/termine testo un attimo
                self.task_complete = True
                if self.cone_prim_handle and self.cone_prim_handle.is_valid():
                    try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32))
                    except RuntimeError: pass


            try:
                current_cone_pos_tuple, _ = self.cone_prim_handle.get_world_pose() 
                if current_cone_pos_tuple is None or len(current_cone_pos_tuple) != 3:
                    raise TypeError("get_world_pose returned None or malformed tuple during active movement")
                current_cone_pos_np = current_cone_pos_tuple 
            except (RuntimeError, TypeError, IndexError, AttributeError) as e_get_pose_active:
                print(f"SIM_STEP {self._sim_step} (Movement - Phase {self.movement_phase}): Error getting cone pose: {e_get_pose_active}. To 'done'.")
                self.movement_phase = "done"; return

            if not self.surface_gripper.is_closed() and self.movement_phase not in ["releasing", "done"]:
                print(f"SIM_STEP {self._sim_step} (Movement - Phase {self.movement_phase}): Gripper opened unexpectedly! Aborting to 'releasing'.")
                self.movement_phase = "releasing"
                try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32))
                except RuntimeError as e_stop: print(f"  Movement Abort (gripper open) STOP ERR: {e_stop}"); self.movement_phase = "done"
               
            if self.movement_phase == "lifting":
                current_z = float(current_cone_pos_np[2])
                if self._sim_step % 15 == 0 : print(f"    LIFTING (Step {self._sim_step}): Z: {current_z:.4f} -> Target Z: {self.target_lift_height:.4f}")
                if current_z < self.target_lift_height:
                    try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0,self.lift_speed_vertical],dtype=np.float32))
                    except RuntimeError as e_lift: print(f" LIFTING ERR: {e_lift}"); self.movement_phase="done"
                else: 
                    print(f"SIM_STEP {self._sim_step} (Movement): Reached lift height. Phase: MOVING_HORIZONTAL.")
                    self.movement_phase = "moving_horizontal"
                    try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32)) 
                    except RuntimeError as e_lift_stop: print(f" LIFTING STOP ERR: {e_lift_stop}"); self.movement_phase="done"
               
            elif self.movement_phase == "moving_horizontal":
                target_xy_np = self.target_horizontal_position 
                current_xy_np = current_cone_pos_np[:2].astype(np.float64) 
                current_z_np = float(current_cone_pos_np[2])
                diff_xy_np = target_xy_np - current_xy_np
                dist_xy = float(np.linalg.norm(diff_xy_np))

                if self._sim_step % 15 == 0:
                    print(f"    MOVING_H (Step {self._sim_step}): CurrXY: {current_xy_np.round(3)}, DistXY: {dist_xy:.3f}, TargetZ (Hold): {self.target_lift_height:.4f}, CurrentZ: {current_z_np:.4f}")
               
                if dist_xy > 0.02: 
                    direction_xy_np = diff_xy_np / dist_xy
                    vel_xy_target_np = direction_xy_np * self.move_speed_horizontal
                    
                    z_error = self.target_lift_height - current_z_np
                    vel_z_correction = self.z_correction_factor * z_error
                    max_z_speed = self.lift_speed_vertical * self.max_z_correction_speed_factor 
                    vel_z_correction = float(np.clip(vel_z_correction, -max_z_speed, max_z_speed))
                    try:
                        self.cone_prim_handle.set_linear_velocity(
                            np.array([vel_xy_target_np[0], vel_xy_target_np[1], vel_z_correction], dtype=np.float32)
                        )
                    except RuntimeError as e_moveh: print(f" MOVING_H ERR: {e_moveh}"); self.movement_phase = "done"
                else: 
                    print(f"SIM_STEP {self._sim_step} (Movement): Reached horizontal target. Phase: DESCENDING.")
                    self.movement_phase = "descending"
                    self.target_descent_z = self.lift_start_z +(2*self.cone_height)+0.001 if self.lift_start_z is not None else current_z_np 
                    print(f"    MOVING_H -> DESCENDING: Target Descent Z: {self.target_descent_z:.4f}")
                    try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32)) 
                    except RuntimeError as e_moveh_stop: print(f" MOVING_H STOP ERR: {e_moveh_stop}"); self.movement_phase = "done"
               
            elif self.movement_phase == "descending":
                if self.target_descent_z is None: 
                    print(f"    DESCENDING (Step {self._sim_step}): target_descent_z is None! Aborting to RELEASING.")
                    self.movement_phase = "releasing"; return 
                
                current_z = float(current_cone_pos_np[2])
                if current_z > self.target_descent_z + 0.005: 
                    if self._sim_step % 15 == 0: print(f"    DESCENDING (Step {self._sim_step}): CurrZ: {current_z:.4f} -> TargetZ: {self.target_descent_z:.4f}")
                    try: self.cone_prim_handle.set_linear_velocity(np.array([0.0,0.0, -self.lift_speed_vertical],dtype=np.float32))
                    except RuntimeError as e_desc: print(f" DESCENDING ERR: {e_desc}"); self.movement_phase="done"
                else: 
                    print(f"SIM_STEP {self._sim_step} (Movement): Reached descent height. Phase: RELEASING.")
                    self.movement_phase = "releasing"
                    try: self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32)) 
                    except RuntimeError as e_desc_stop: print(f" DESCENDING STOP ERR: {e_desc_stop}"); self.movement_phase="done"
               
            elif self.movement_phase == "releasing":
                print(f"SIM_STEP {self._sim_step} (Movement): Phase RELEASING. Opening gripper.")
                if self.surface_gripper: self.surface_gripper.open() 
                self.movement_phase = "done"


        if self.movement_phase == "done":
            
            if self.cone_prim_handle and self.cone_prim_handle.is_valid() and self.cone_physics_dynamically_enabled:
                try:
                    _, current_lin_vel_tuple, _ = self.cone_prim_handle.get_velocities() 
                    if current_lin_vel_tuple is not None and len(current_lin_vel_tuple) == 3 and np.linalg.norm(np.array([float(c) for c in current_lin_vel_tuple])) > 0.01:
                        if self._sim_step % 30 == 0: print(f"    DONE (Step {self._sim_step}): Cone still moving. Attempting to stop.")
                        self.cone_prim_handle.set_linear_velocity(np.zeros(3, dtype=np.float32))
                except (RuntimeError, TypeError, IndexError, AttributeError): pass 
                except Exception: pass 
            

            if self._sim_step > self.steps_before_initial_lift_phase + 300: 
                if(self.grip_status_code ==0):
                    self.grip_status_code = 3
                    
                self.task_complete = True
                if self._sim_step % 240 == 0 : 
                    sg_status = "N/A"
                    if self.surface_gripper: sg_status = "Closed" if self.surface_gripper.is_closed() else "Open"
                    print(f"SIM_STEP {self._sim_step}: Movement phase 'done'. Gripper: {sg_status}. Sim continues. (Consider stopping sim or resetting script if task is complete)")

    def is_task_complete(self):
        return self.task_complete

    def run(self,position):
        print("SurfaceGripperDirectScript.run() called. Initializing states...")
        self.grasp_pone_cone=position
        self._sim_step = 0
        self.gripper_close_command_sent = False
        self.cone_usd_prim_created = False
        self.cone_sg_initialized = False  
        self.cone_physics_dynamically_enabled = False
        self.movement_phase = "idle"
        self.lift_start_z = None; self.target_descent_z = None
       
        self.cone_prim_handle = None; self.target_srp_handle = None
        self.cone_geom_usd = None;
        if self.surface_gripper:
            self.surface_gripper = None 
       
        self._setup_initial_scene()
        print("Setup complete. Simulation loop will now take over via physics callback.")

    def cleanup(self):
        print("SurfaceGripperDirectScript.cleanup() called.")
        if self._physx_sub:
            try:
                self._physx_sub.unsubscribe()
                print("Unsubscribed from physics step events during cleanup.")
            except Exception as e_unsub_clean:
                print(f"Error unsubscribing physics step during cleanup: {e_unsub_clean}")
            self._physx_sub = None
       
        if self.surface_gripper:
            self.surface_gripper = None 
        self.cone_prim_handle = None
        self.target_srp_handle = None
        self.cone_geom_usd = None
        print("Cleanup finished.")


