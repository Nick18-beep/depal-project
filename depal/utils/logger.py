"""Lightweight logging utilities used across the Depal project."""

from __future__ import annotations

import os
import sys
from datetime import datetime

LOG_LEVELS = {"DEBUG": 10, "INFO": 20, "WARNING": 30, "ERROR": 40}

_configured_level = os.environ.get("DEPAL_LOG_LEVEL", "INFO").upper()
_current_level = LOG_LEVELS.get(_configured_level, LOG_LEVELS["INFO"])


def _should_log(level_name: str) -> bool:
    return LOG_LEVELS.get(level_name, 0) >= _current_level


def _emit(prefix: str, message: str) -> None:
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] {prefix} {message}", file=sys.stdout)


def log_debug(message: str) -> None:
    if _should_log("DEBUG"):
        _emit("DEBUG", message)


def log_info(message: str) -> None:
    if _should_log("INFO"):
        _emit("INFO ", message)


def log_warning(message: str) -> None:
    if _should_log("WARNING"):
        _emit("WARN ", message)


def log_error(message: str) -> None:
    if _should_log("ERROR"):
        _emit("ERROR", message)


def log_section(title: str) -> None:
    """Highlight the start of a logical section in the console output."""
    if not _should_log("INFO"):
        return
    separator = "-" * max(10, len(title) + 4)
    _emit("INFO ", f"{separator}\n  {title}\n{separator}")
