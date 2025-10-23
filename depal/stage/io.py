"""Utilities for saving and loading USD stages with timeline management."""

from __future__ import annotations

import os
import sys
from functools import cached_property
from pathlib import Path
from typing import Optional

from depal.utils.logger import log_debug, log_error, log_info, log_warning


class StageIO:
    """Encapsulates save/load operations for the active USD stage.

    The heavy Isaac imports are deferred until an instance is created so callers
    can construct StageIO only after SimulationApp has been initialised.
    """

    def __init__(self) -> None:
        import omni.timeline  # noqa: F401
        import omni.usd  # noqa: F401
        import omni.kit.app  # noqa: F401

        self._timeline_module = omni.timeline
        self._usd_module = omni.usd
        self._kit_app_module = omni.kit.app

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    def save_stage(self, output_folder: str, file_name: str = "saved_stage.usd") -> Optional[Path]:
        """Save the currently open stage to the given folder."""
        log_info(f"StageIO: salvataggio in '{output_folder}/{file_name}'")
        was_playing = self._pause_if_needed()

        try:
            output_path = self._prepare_output_path(output_folder, file_name)
        except Exception as exc:
            log_error(f"StageIO: creazione cartella fallita ({exc})")
            self._resume_if_needed(was_playing)
            return None

        stage = self._usd_context.get_stage()
        if not stage:
            log_warning("StageIO: nessuno stage attivo da salvare")
            self._resume_if_needed(was_playing)
            return None

        root_layer = stage.GetRootLayer()
        try:
            if not root_layer.Export(str(output_path)):
                raise RuntimeError(f"root_layer.Export() non riuscito per '{output_path}'")
            log_info(f"StageIO: stage salvato -> {output_path}")
        except Exception as exc:
            log_error(f"StageIO: esportazione fallita ({exc})")
            output_path = None
        finally:
            self._resume_if_needed(was_playing)

        return output_path

    def load_stage(self, file_path: str) -> bool:
        """Load the provided USD file into a fresh stage context."""
        absolute_path = Path(self.resolve_main_script_dir(), file_path).resolve()
        log_info(f"StageIO: caricamento stage da '{absolute_path}'")

        if not absolute_path.exists():
            msg = f"File USD non trovato presso {absolute_path}"
            log_error(f"StageIO: {msg}")
            raise FileNotFoundError(msg)

        was_playing = self._pause_if_needed()

        log_debug("StageIO: creazione nuovo stage in corso")
        if not self._usd_context.new_stage():
            log_error("StageIO: impossibile creare un nuovo stage vuoto")
            self._resume_if_needed(was_playing)
            raise RuntimeError("Impossibile creare un nuovo stage vuoto.")

        self._pump_app_updates(1)

        log_info(f"StageIO: apertura file USD '{absolute_path}'")
        try:
            if not self._usd_context.open_stage(str(absolute_path)):
                usd_error_log = self._usd_context.get_error_log()
                error_msg = f"open_stage() ha restituito False per '{absolute_path}'."
                if usd_error_log:
                    error_msg += f"\nLog errori USD:\n{usd_error_log}"
                raise RuntimeError(error_msg)
            self._pump_app_updates(1)
            log_info(f"StageIO: stage caricato ({absolute_path})")
        finally:
            self._resume_if_needed(was_playing)

        return True

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def _prepare_output_path(self, output_folder: str, file_name: str) -> Path:
        base_dir = Path(self.resolve_main_script_dir())
        folder_path = (base_dir / output_folder).resolve()
        folder_path.mkdir(parents=True, exist_ok=True)
        output_path = folder_path / file_name
        log_debug(f"StageIO: cartella output -> {folder_path}")
        log_debug(f"StageIO: percorso salvataggio -> {output_path}")
        return output_path

    def _pump_app_updates(self, iterations: int) -> None:
        for _ in range(iterations):
            self._kit_app.update()

    def _pause_if_needed(self) -> bool:
        if self._timeline.is_playing():
            log_debug("StageIO: timeline in esecuzione, richiesta pausa")
            self._timeline.stop()
            self._pump_app_updates(1)
            return True
        return False

    def _resume_if_needed(self, was_playing: bool) -> None:
        if was_playing:
            log_debug("StageIO: timeline ripresa")
            self._timeline.play()
            self._pump_app_updates(1)

    @staticmethod
    def resolve_main_script_dir() -> str:
        main_mod = sys.modules.get("__main__")
        main_file = getattr(main_mod, "__file__", None)
        if main_file:
            return os.path.dirname(os.path.abspath(main_file))
        log_warning("StageIO: __main__.__file__ non disponibile, uso os.getcwd()")
        return os.getcwd()

    # ------------------------------------------------------------------ #
    # Cached accessors
    # ------------------------------------------------------------------ #
    @cached_property
    def _timeline(self):
        return self._timeline_module.get_timeline_interface()

    @cached_property
    def _usd_context(self):
        return self._usd_module.get_context()

    @cached_property
    def _kit_app(self):
        return self._kit_app_module.get_app()
