"""Central location for default launcher configuration values."""

from __future__ import annotations

from pathlib import Path

DEFAULT_USD_PATH = Path("assets/roarm_m3/usd/roarm_m3.generated.usd")
DEFAULT_PRIM_PATH = "/World/roarm_m3"
DEFAULT_MODE = "train"
DEFAULT_WARMUP_STEPS = 0
DEFAULT_MAX_STEPS = 0  # 0 == infinite loop
DEFAULT_STAGE_POLL_LIMIT = 600
DEFAULT_TRACKED_NOTES = (
    "quiet_shutdown",
    "egl",
    "blocklist",
    "overlay",
)
