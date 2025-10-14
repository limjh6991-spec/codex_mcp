#!/usr/bin/env python3
"""Compatibility launcher that delegates to ``roarm_app.cli``."""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


from roarm_app.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
