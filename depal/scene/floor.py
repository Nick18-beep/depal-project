"""Helpers to create physics-enabled floors within the scene."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass
class FloorMaterialComponent:
    """Container describing an existing PBR material triple."""

    material_prim: object
    shader_prims: Sequence[object]
    material_schema: object


class FloorBuilder:
    """Encapsulates the operations required to spawn a ground plane."""

    def __init__(self, stage) -> None:
        try:
            from isaacsim.core.api.objects import GroundPlane
            from isaacsim.core.api.materials import VisualMaterial
            from isaacsim.core.utils.semantics import add_update_semantics
        except ImportError as exc:  # pragma: no cover - Isaac Sim specific import
            print(f"ERRORE CRITICO: impossibile importare risorse Isaac Sim: {exc}")
            self._ground_plane_cls = None
            self._visual_material_cls = None
            self._add_semantics = None
        else:
            self._ground_plane_cls = GroundPlane
            self._visual_material_cls = VisualMaterial
            self._add_semantics = add_update_semantics

        self._stage = stage

    def create(
        self,
        prim_path: str = "/World/PhysicsFloor",
        material_component: Optional[FloorMaterialComponent] = None,
    ):
        """Create a physics-enabled ground plane, optionally binding an existing material."""
        if not self._ground_plane_cls or not self._visual_material_cls:
            print("FloorBuilder: classi Isaac Sim mancanti. Creazione pavimento annullata.")
            return None

        visual_material = None
        if material_component:
            visual_material = self._create_visual_material(prim_path, material_component)

        try:
            ground_plane = self._ground_plane_cls(
                prim_path=prim_path,
                name=f"{prim_path.split('/')[-1]}_isaac_obj",
                z_position=0.0,
                visual_material=visual_material,
            )
        except Exception as exc:  # pragma: no cover - Isaac Sim specific import
            print(f"FloorBuilder: errore durante la creazione del GroundPlane: {exc}")
            return None

        if self._add_semantics:
            self._add_semantics(prim=ground_plane.prim, semantic_label="floor", type_label="class")

        if visual_material:
            print(
                f"FloorBuilder: il pavimento '{ground_plane.prim_path}' utilizza il materiale "
                f"'{material_component.material_prim.GetPath()}'."
            )
        else:
            print(
                f"FloorBuilder: nessun materiale personalizzato fornito per '{ground_plane.prim_path}'. "
                "Uso dell'aspetto di default."
            )
        return ground_plane

    def _create_visual_material(self, prim_path: str, component: FloorMaterialComponent):
        new_prim_path = f"{prim_path}/Looks/PBRMaterialReference"
        try:
            vm = self._visual_material_cls(
                prim_path=new_prim_path,
                name=f"{component.material_prim.GetName()}_VMObjectForFloor",
                prim=component.material_prim,
                shaders_list=list(component.shader_prims),
                material=component.material_schema,
            )
        except TypeError as exc:  # pragma: no cover - Isaac Sim specific import
            print(f"FloorBuilder: errore di tipo durante la creazione del VisualMaterial: {exc}")
            return None
        except Exception as exc:  # pragma: no cover
            print(f"FloorBuilder: errore generico durante la creazione del VisualMaterial: {exc}")
            return None
        return vm
