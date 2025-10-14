"""Environment variable helpers for Isaac Sim launch configuration."""

from __future__ import annotations

import os
from typing import List


def configure_quiet_shutdown(disable: bool) -> str:
    """Enable or disable quiet shutdown behavior.

    Returns a note describing the applied mode for downstream logging.
    """

    if disable:
        os.environ["CARB_APP_QUIET_SHUTDOWN"] = "0"
        return "quiet_shutdown=disabled"

    os.environ.setdefault("CARB_APP_QUIET_SHUTDOWN", "1")
    return "quiet_shutdown=default"


def apply_headless_defaults(headless: bool, glx_vendor: str | None = "nvidia") -> List[str]:
    """Apply standard KIT/EGL environment defaults when running headless.

    Returns notes describing the effective configuration so they can be surfaced to the
    runtime summary at exit.
    """

    notes: List[str] = []
    if not headless:
        return notes

    os.environ.setdefault("KIT_USE_EGL", "1")
    os.environ.setdefault("ENABLE_HEADLESS", "1")
    notes.append("egl=headless-defaults")

    if glx_vendor:
        os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", glx_vendor)
        notes.append(f"glx_vendor={glx_vendor}")
    else:
        # Allow explicit unsetting to test EGL-only paths.
        if "__GLX_VENDOR_LIBRARY_NAME" in os.environ:
            del os.environ["__GLX_VENDOR_LIBRARY_NAME"]
        notes.append("glx_vendor=unset")

    preserve_cuda = os.environ.get("ROARM_PRESERVE_CUDA_VISIBLE_DEVICES", "").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    if "CUDA_VISIBLE_DEVICES" in os.environ and not preserve_cuda:
        del os.environ["CUDA_VISIBLE_DEVICES"]
        notes.append("cuda_visible_devices=unset")
    elif preserve_cuda:
        notes.append("cuda_visible_devices=preserved")
    else:
        notes.append("cuda_visible_devices=inherit")

    notes.append("cpu_governor_hint=performance")

    return notes
