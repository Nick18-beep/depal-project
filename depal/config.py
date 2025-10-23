"""Configuration helpers for the Depal Isaac Sim application."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


def load_config(config_path: Path) -> Dict[str, Any]:
    """Load the YAML configuration file."""
    if not config_path.exists():
        raise FileNotFoundError(
            f"File di configurazione '{config_path.name}' non trovato."
        )

    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    print(f"Configurazione caricata da '{config_path.name}'.")
    return config


class ConfigStore:
    """In-memory configuration storage with optional overrides."""

    def __init__(self, default_path: Path) -> None:
        self._default_path = default_path
        self._active_path = default_path
        self._config = load_config(default_path)

    @property
    def active_path(self) -> Path:
        return self._active_path

    def current(self) -> Dict[str, Any]:
        """Return a copy of the currently loaded configuration."""
        return copy.deepcopy(self._config)

    def override(self, new_path: Optional[Path]) -> None:
        """Override the active configuration file."""
        path_to_use = new_path or self._default_path
        if path_to_use != self._active_path:
            print(f"Sovrascrivo la configurazione attiva dal percorso: {path_to_use}")
        self._active_path = path_to_use
        self._config = load_config(path_to_use)

    def reload(self) -> None:
        """Reload the current configuration from disk."""
        self._config = load_config(self._active_path)
