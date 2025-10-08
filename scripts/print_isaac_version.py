#!/usr/bin/env python
"""Print detected Isaac Sim version (kit app) if available, else fallback JSON.
Usage:
  python scripts/print_isaac_version.py
"""
from __future__ import annotations
import json
import sys
import os
from pathlib import Path

def main():
    data = {
        "isaac_available": False,
        "version": None,
        "detail": None,
    }
    try:  # pragma: no cover (CI 환경 비가용 예상)
        import omni.kit.app  # type: ignore
        app = omni.kit.app.get_app()
        ver = getattr(app, "get_version", lambda: None)()
        data["isaac_available"] = True
        data["version"] = ver
    except Exception as e:  # noqa
        # Try environment-driven autodetect hints
        root_env = os.getenv("ISAAC_SIM_ROOT")
        candidate_roots: list[Path] = []
        if root_env:
            candidate_roots.append(Path(root_env))
        # User provided canonical path (project customization) fallback
        default_hint = Path("/home/roarm_m3/isaac_sim")
        if default_hint.exists():
            candidate_roots.append(default_hint)
        # Common launcher install base
        home_pkg = Path.home() / ".local" / "share" / "ov" / "pkg"
        if home_pkg.exists():
            for p in home_pkg.glob("isaac-sim-*"):
                candidate_roots.append(p)
        # Deduplicate
        seen = set()
        uniq_roots = []
        for c in candidate_roots:
            if c not in seen:
                uniq_roots.append(c)
                seen.add(c)
        data["detail"] = f"{type(e).__name__}: {e}; candidates={[str(c) for c in uniq_roots]}"
        # Provide guidance if setup script discovered
        setup_scripts = []
        for r in uniq_roots:
            for name in ("setup_python_env.sh", "python.sh"):
                sp = r / name
                if sp.exists():
                    setup_scripts.append(str(sp))
        if setup_scripts:
            data["suggest_setup"] = setup_scripts
    print(json.dumps(data, ensure_ascii=False))
    if not data["isaac_available"]:
        sys.exit(0)

if __name__ == "__main__":
    main()
