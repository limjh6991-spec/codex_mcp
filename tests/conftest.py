"""Test configuration: ensure project root and src are on sys.path for imports.
If editable install is not performed, this still allows importing modules.
"""
from __future__ import annotations
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if SRC.exists() and str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
