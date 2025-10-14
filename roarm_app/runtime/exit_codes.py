"""Common exit codes for the RoArm Isaac Sim runtime."""

from __future__ import annotations

from enum import Enum


class ExitCode(str, Enum):
    """Enumeration of launcher termination reasons."""

    COMPLETED = "completed"
    USD_STAGE_OPEN_FAILED = "usd_stage_open_failed"
    MAX_STEPS_REACHED = "max_steps_reached"
    IS_RUNNING_FALSE = "is_running_false"
    UNRESOLVED = "unresolved"

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


__all__ = ["ExitCode"]
