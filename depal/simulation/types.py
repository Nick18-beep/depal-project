"""Shared dataclasses used across the simulation modules."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence


@dataclass
class SpawnedEntities:
    """Tracks the last spawned USD prim paths for boxes and YCB objects."""

    boxes: Sequence[str] = field(default_factory=list)
    objects: Sequence[str] = field(default_factory=list)

    def any(self) -> bool:
        return bool(self.boxes or self.objects)


@dataclass(frozen=True)
class GripOptions:
    """Surface gripper and pinza toggles."""

    enable_surface_gripper: bool
    enable_pinza: bool

    @classmethod
    def from_option_list(cls, options: Sequence[str]) -> "GripOptions":
        normalized = {opt.lower() for opt in options}
        return cls(
            enable_surface_gripper="grip" in normalized,
            enable_pinza="pinza" in normalized,
        )

    def any_enabled(self) -> bool:
        return self.enable_surface_gripper or self.enable_pinza
