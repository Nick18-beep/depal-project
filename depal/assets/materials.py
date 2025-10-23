"""Utilities to build and manage OmniPBR materials for the Depal project."""

from __future__ import annotations

import random
import traceback
from depal.utils.logger import log_debug, log_error, log_info, log_warning
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, TYPE_CHECKING

from depal.utils.logger import log_error, log_info, log_warning

if TYPE_CHECKING:  # pragma: no cover
    from pxr import Gf, Sdf, UsdShade, Usd


SUPPORTED_TEXTURE_EXTENSIONS: Sequence[str] = (
    ".png",
    ".jpg",
    ".jpeg",
    ".tga",
    ".dds",
    ".exr",
    ".hdr",
)


def _safe_material_name(texture_file: Path) -> str:
    return f"Mtl_{''.join(c if c.isalnum() else '_' for c in texture_file.stem)}"


def _format_vec3(value) -> str:
    return f"({value[0]:.2f}, {value[1]:.2f}, {value[2]:.2f})"


def _get_omni_pbr():
    from isaacsim.core.api.materials import OmniPBR  # pylint: disable=import-outside-toplevel

    return OmniPBR


def _get_pxr_material_modules():
    from pxr import Gf, Sdf, UsdShade, Usd  # pylint: disable=import-outside-toplevel

    return Gf, Sdf, UsdShade, Usd


@dataclass
class MaterialComponent:
    """Represents the relevant USD prims created for an OmniPBR material."""

    material_prim: "Usd.Prim"
    shader_prims: List["Usd.Prim"]
    material_schema: "UsdShade.Material"

    def as_legacy_tuple(self):
        """Return the legacy tuple format used by the previous codebase."""
        return self.material_prim, self.shader_prims, self.material_schema


class MaterialFactory:
    """Builds OmniPBR materials from texture folders."""

    def __init__(
        self,
        stage,
        simulation_app,
        *,
        base_material_path: str = "/World/Looks",
        config: Optional[Dict] = None,
    ) -> None:
        try:
            self._omni_pbr_cls = _get_omni_pbr()
        except ImportError as exc:  # pragma: no cover - depends on Isaac Sim installation
            raise RuntimeError(
                "OmniPBR non disponibile. Assicurati che Isaac Sim sia correttamente configurato."
            ) from exc

        self._Gf, self._Sdf, self._UsdShade, _ = _get_pxr_material_modules()
        self._stage = stage
        self._simulation_app = simulation_app
        self._base_material_path = base_material_path
        self._config = config or {}

        self._parameter_definitions: Dict[str, tuple[str, object, callable]] = {
            "metallic_range": ("normal_roughness_bias", self._Sdf.ValueTypeNames.Float, lambda r: random.uniform(*r)),
            "roughness_range": ("clearcoat_roughness", self._Sdf.ValueTypeNames.Float, lambda r: random.uniform(*r)),
            "specular_level_range": ("specular_reflectance", self._Sdf.ValueTypeNames.Float, lambda r: random.uniform(*r)),
            "base_color_rgb_range": (
                "diffuse_color_constant",
                self._Sdf.ValueTypeNames.Float3,
                lambda r: self._Gf.Vec3f(*(random.uniform(a, b) for a, b in zip(*r))),
            ),
            "emissive_rgb_range": (
                "emission_color_constant",
                self._Sdf.ValueTypeNames.Float3,
                lambda r: self._Gf.Vec3f(*(random.uniform(a, b) for a, b in zip(*r))),
            ),
            "emissive_strength_range": ("emission_intensity_constant", self._Sdf.ValueTypeNames.Float, lambda r: random.uniform(*r)),
            "clearcoat_intensity_range": ("clearcoat", self._Sdf.ValueTypeNames.Float, lambda r: random.uniform(*r)),
            "clearcoat_roughness_range": ("clearcoat_roughness", self._Sdf.ValueTypeNames.Float, lambda r: random.uniform(*r)),
            "ior_range": ("clearcoat_ior", self._Sdf.ValueTypeNames.Float, lambda r: random.uniform(*r)),
            "normal_intensity_range": ("normalmap_scale", self._Sdf.ValueTypeNames.Float, lambda r: random.uniform(*r)),
        }

    def create_from_directory(self, texture_directory: Path) -> List[MaterialComponent]:
        """Scan a directory and generate an OmniPBR material for each supported file."""
        if not texture_directory.is_dir():
            log_warning(f"MaterialFactory: cartella texture inesistente ({texture_directory})")
            return []

        created_components: List[MaterialComponent] = []
        for texture_file in sorted(texture_directory.iterdir()):
            if texture_file.suffix.lower() not in SUPPORTED_TEXTURE_EXTENSIONS:
                continue
            component = self._build_single_material(texture_file)
            if component:
                created_components.append(component)

        log_info(f"MaterialFactory: creati {len(created_components)} materiali PBR")
        return created_components

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _build_single_material(self, texture_file: Path) -> Optional[MaterialComponent]:
        material_path = f"{self._base_material_path}/{_safe_material_name(texture_file)}"

        try:
            texture_scale = self._sample_config_range("texture_scale_range", default=(1.0, 1.0))
            self._omni_pbr_cls(
                prim_path=material_path,
                texture_path=str(texture_file.as_posix()),
                texture_scale=[texture_scale, texture_scale],
            )
        except Exception as exc:  # pragma: no cover - OmniPBR specific
            log_error(f"MaterialFactory: errore OmniPBR per {material_path} ({exc})")
            traceback.print_exc()
            return None

        self._pump_simulation_frames(2)

        material_prim = self._stage.GetPrimAtPath(material_path)
        material_schema = self._UsdShade.Material(material_prim)
        shader_prim = next((child for child in material_prim.GetChildren() if child.IsA(self._UsdShade.Shader)), None)
        if not shader_prim:
            log_warning(f"MaterialFactory: nessuno shader trovato per {material_path}")
            return MaterialComponent(material_prim, [], material_schema)

        shader = self._UsdShade.Shader(shader_prim)
        applied_values = self._apply_configured_parameters(shader)
        self._apply_emissive_probability(shader, applied_values)

        # Enable key toggles to ensure OmniPBR behaves as expected.
        self._set_shader_input(shader, "enable_emission", self._Sdf.ValueTypeNames.Bool, True)
        self._set_shader_input(shader, "enable_clearcoat", self._Sdf.ValueTypeNames.Bool, True)

        if applied_values:
            formatted = []
            for key, value in applied_values.items():
                if isinstance(value, self._Gf.Vec3f):
                    formatted.append(f"{key}={_format_vec3(value)}")
                elif isinstance(value, float):
                    formatted.append(f"{key}={value:.2f}")
                else:
                    formatted.append(f"{key}={value}")
            log_debug(f"MaterialFactory: parametri applicati -> {'; '.join(formatted)}")

        return MaterialComponent(material_prim, [shader_prim], material_schema)

    def _sample_config_range(self, key: str, default: Sequence[float]) -> float:
        range_values = self._config.get(key, default)
        try:
            return random.uniform(*range_values)
        except Exception:
            return float(default[0])

    def _apply_configured_parameters(self, shader: UsdShade.Shader) -> Dict[str, object]:
        applied: Dict[str, object] = {}
        for cfg_key, (shader_input, value_type, sampler) in self._parameter_definitions.items():
            if cfg_key not in self._config:
                continue

            value = sampler(self._config[cfg_key])
            if self._set_shader_input(shader, shader_input, value_type, value):
                applied[shader_input] = value
        return applied

    def _apply_emissive_probability(self, shader, applied: Dict[str, object]) -> None:
        if "emissive_probability" not in self._config:
            return

        probability = self._config["emissive_probability"]
        if not isinstance(probability, (float, int)) or not (0.0 <= probability <= 1.0):
            log_warning(f"MaterialFactory: emissive_probability ({probability}) fuori range [0,1], ignorata")
            return

        if random.random() < probability:
            return  # Emission stays enabled

        black = self._Gf.Vec3f(0.0, 0.0, 0.0)
        disabled = self._set_shader_input(shader, "emissive_color", self._Sdf.ValueTypeNames.Float3, black)
        disabled |= self._set_shader_input(shader, "emissive_intensity", self._Sdf.ValueTypeNames.Float, 0.0)
        if disabled:
            applied["emissive_color"] = black
            applied["emissive_intensity"] = 0.0
            log_info(f"MaterialFactory: emissione disattivata (probabilita ON {probability * 100:.1f}%)")

    def _set_shader_input(self, shader, name: str, value_type, value) -> bool:
        input_attr = shader.GetInput(name)
        if not input_attr:
            return False
        try:
            input_attr.Set(value)
        except Exception as exc:  # pragma: no cover - USD layer specific
            log_warning(f"MaterialFactory: impossibile impostare l'input {name} a {value} ({exc})")
            return False
        return True

    def _pump_simulation_frames(self, updates: int) -> None:
        if not self._simulation_app:
            return
        for _ in range(updates):
            self._simulation_app.update()


def create_materials_from_directory(
    stage,
    simulation_app,
    texture_dir_path: str,
    *,
    base_material_usd_path: str = "/World/Looks",
    material_config: Optional[Dict] = None,
) -> List[MaterialComponent]:
    """Convenience helper mirroring the legacy public API."""
    factory = MaterialFactory(
        stage,
        simulation_app,
        base_material_path=base_material_usd_path,
        config=material_config,
    )
    return factory.create_from_directory(Path(texture_dir_path))

def crea_materiali_da_cartella_texture(
    stage,
    simulation_app_instance,
    texture_dir_path: str,
    base_material_usd_path: str = "/World/Looks",
    material_config: Optional[Dict] = None,
):
    """Backward-compatible helper matching the legacy naming convention."""
    components = create_materials_from_directory(
        stage,
        simulation_app_instance,
        texture_dir_path,
        base_material_usd_path=base_material_usd_path,
        material_config=material_config,
    )
    return [component.as_legacy_tuple() for component in components]
