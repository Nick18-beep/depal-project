"""Simulation environment bootstrap utilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from omni.isaac.kit import SimulationApp


@dataclass
class SimulationModules:
    """Container that exposes core Isaac/Replicator modules."""

    omni_usd: Any
    omni_timeline: Any
    rep: Any
    carb: Any
    prims_utils: Any
    World: Any
    pxr: Dict[str, Any]


class SimulationEnvironment:
    """Initialises the SimulationApp and lazily imports Isaac modules."""

    def __init__(self, sim_setup_cfg: Dict[str, Any]) -> None:
        self._sim_setup_cfg = sim_setup_cfg
        self._app: Optional[SimulationApp] = None
        self._modules: Optional[SimulationModules] = None

    @property
    def app(self) -> SimulationApp:
        if not self._app:
            raise RuntimeError("SimulationApp non inizializzata.")
        return self._app

    @property
    def modules(self) -> SimulationModules:
        if not self._modules:
            raise RuntimeError("I moduli della simulazione non sono stati caricati.")
        return self._modules

    def initialize(self) -> SimulationApp:
        """Create the SimulationApp if needed and load dependent modules."""
        if self._app:
            return self._app

        setup = {"headless": self._sim_setup_cfg.get("headless", True)}
        renderer = self._sim_setup_cfg.get("renderer")
        if renderer:
            setup["renderer"] = renderer

        self._app = SimulationApp(setup)
        print("SimulationApp inizializzata.")
        self._modules = self._import_runtime_modules()
        return self._app

    def close(self) -> None:
        if self._app:
            self._app.close()
            self._app = None
            self._modules = None

    def _import_runtime_modules(self) -> SimulationModules:
        """Import runtime modules that require an active SimulationApp."""
        import omni.usd
        import omni.timeline
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
        from pxr import PhysxSchema  # type: ignore
        import omni.replicator.core as rep
        import carb
        from omni.isaac.core import World
        from omni.isaac.core.utils import prims as prims_utils

        pxr_modules: Dict[str, Any] = {
            "Gf": Gf,
            "Sdf": Sdf,
            "Usd": Usd,
            "UsdGeom": UsdGeom,
            "UsdLux": UsdLux,
            "UsdPhysics": UsdPhysics,
            "PhysxSchema": PhysxSchema,
        }

        return SimulationModules(
            omni_usd=omni.usd,
            omni_timeline=omni.timeline,
            rep=rep,
            carb=carb,
            prims_utils=prims_utils,
            World=World,
            pxr=pxr_modules,
        )
