"""Utilities for managing Isaac Sim headless overlay deployment."""

from __future__ import annotations

import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Optional, Sequence

OverlayLogger = Callable[[str, Dict[str, object]], None]


@dataclass(frozen=True)
class OverlayResult:
    """Structured outcome for headless overlay preparation."""

    experience: Optional[str]
    note: str
    status: str
    reason: Optional[str] = None
    source: Optional[Path] = None
    target: Optional[Path] = None


_OVERLAY_SOURCE = Path(__file__).resolve().parents[2] / "scripts" / "config" / "roarm_headless.overlay.kit"
_ISAAC_ROOT_ENV_VARS: Sequence[str] = ("ISAAC_SIM_ROOT", "ISAAC_SIM_PATH")


def locate_isaac_root() -> Optional[Path]:
    """Best-effort discovery of the Isaac Sim root directory."""

    for key in _ISAAC_ROOT_ENV_VARS:
        raw = os.environ.get(key)
        if raw:
            candidate = Path(raw).expanduser().resolve()
            if candidate.exists():
                return candidate

    default_install = Path.home() / "isaacsim"
    if default_install.exists():
        return default_install.resolve()
    return None


def overlay_source() -> Path:
    """Return the expected location of the headless overlay template."""

    return _OVERLAY_SOURCE


def deploy_overlay(target_root: Path, source: Path = overlay_source()) -> Optional[Path]:
    """Copy the overlay file into the Isaac Sim installation.

    Returns the path to the deployed overlay if successful, else ``None``.
    """

    if not source.exists():
        return None

    target_path = target_root / "apps" / source.name
    try:
        target_path.parent.mkdir(parents=True, exist_ok=True)
        if not target_path.exists() or source.read_bytes() != target_path.read_bytes():
            shutil.copy2(source, target_path)
        return target_path
    except Exception:
        return None


def prepare_headless_overlay(
    skip_env: bool,
    skip_cli: bool,
    logger: Optional[OverlayLogger] = None,
) -> OverlayResult:
    """Determine whether the overlay should be applied and, if so, stage it."""

    def emit(event: str, **payload: object) -> None:
        if logger is not None:
            logger(event, dict(payload))

    source = overlay_source()

    if skip_env:
        emit("overlay_skipped", reason="env", source=str(source))
        return OverlayResult(None, "overlay=skipped(env)", "skipped", reason="env", source=source)
    if skip_cli:
        emit("overlay_skipped", reason="cli", source=str(source))
        return OverlayResult(None, "overlay=skipped(cli)", "skipped", reason="cli", source=source)

    if not source.exists():
        emit("overlay_missing_source", source=str(source))
        return OverlayResult(None, "overlay=missing_source", "missing_source", source=source)

    isaac_root = locate_isaac_root()
    if isaac_root is None:
        emit("overlay_missing_root", source=str(source))
        return OverlayResult(None, "overlay=missing_root", "missing_root", source=source)

    expected_target = isaac_root / "apps" / source.name
    target = deploy_overlay(isaac_root, source)
    if target is None:
        emit("overlay_failed_copy", source=str(source), target=str(expected_target))
        return OverlayResult(None, "overlay=failed_copy", "failed_copy", source=source, target=expected_target)

    emit("overlay_applied", source=str(source), target=str(target))
    return OverlayResult(str(target), "overlay=applied", "applied", source=source, target=target)
