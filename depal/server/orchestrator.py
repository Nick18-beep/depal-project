"""Task orchestration for simulation requests coming from the API."""

from __future__ import annotations

import queue
import shutil
import threading
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Iterable, Optional, Sequence

from depal.config import ConfigStore
from depal.simulation import SimulationPipeline


class TaskType(Enum):
    START = auto()
    REGENERATE = auto()


@dataclass
class SimulationTask:
    task_type: TaskType
    options: Sequence[str]
    config_path: Optional[Path] = None


class SimulationOrchestrator:
    """Queue-based orchestrator that executes simulation tasks sequentially."""

    def __init__(
        self,
        config_store: ConfigStore,
        pipeline: SimulationPipeline,
        output_dir: Path,
    ) -> None:
        self._config_store = config_store
        self._pipeline = pipeline
        self._output_dir = output_dir
        self._task_queue: "queue.Queue[SimulationTask]" = queue.Queue()
        self._running = threading.Event()

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    def submit(self, task: SimulationTask) -> None:
        self._task_queue.put(task)

    def submit_start(self, options: Sequence[str], config_path: Optional[Path]) -> None:
        self.submit(SimulationTask(TaskType.START, tuple(options), config_path))

    def submit_regenerate(self, options: Sequence[str]) -> None:
        self.submit(SimulationTask(TaskType.REGENERATE, tuple(options)))

    def run_forever(self) -> None:
        while True:
            task = self._task_queue.get()
            self._running.set()
            try:
                if task.task_type is TaskType.START:
                    self._handle_start(task)
                elif task.task_type is TaskType.REGENERATE:
                    self._handle_regenerate(task)
            except Exception as exc:  # pylint: disable=broad-exception-caught
                print(f"ERRORE durante l'esecuzione del task {task.task_type.name}: {exc}")
            finally:
                self._running.clear()
                self._task_queue.task_done()

    # ------------------------------------------------------------------ #
    # Task handlers
    # ------------------------------------------------------------------ #
    def _handle_start(self, task: SimulationTask) -> None:
        if task.config_path:
            self._config_store.override(task.config_path)
        else:
            self._config_store.reload()

        self._prepare_output_directory()
        config = self._config_store.current()
        self._pipeline.run_generation(config, task.options)

    def _handle_regenerate(self, task: SimulationTask) -> None:
        config = self._config_store.current()
        self._pipeline.regenerate_data(config, task.options)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def _prepare_output_directory(self) -> None:
        self._output_dir.mkdir(parents=True, exist_ok=True)
        for entry in self._output_dir.iterdir():
            if entry.suffix.lower() in (".yaml", ".yml"):
                continue
            if entry.is_dir():
                shutil.rmtree(entry, ignore_errors=True)
            else:
                try:
                    entry.unlink()
                except FileNotFoundError:
                    pass
