#!/usr/bin/env python3
"""Remove Isaac Sim crash dump artifacts produced during headless runs."""

from __future__ import annotations

import argparse
import site
from pathlib import Path
from typing import Iterable


def _default_dump_dirs() -> Iterable[Path]:
    for base in site.getsitepackages():
        data_root = Path(base) / "omni" / "data" / "Kit"
        if not data_root.exists():
            continue
        for child in data_root.glob("roarm_headless.overlay/*"):
            if child.is_dir():
                yield child


def cleanup(paths: Iterable[Path], *, dry_run: bool) -> int:
    removed = 0
    for directory in paths:
        for artifact in directory.glob("*.*"):
            if artifact.suffix.lower() not in {".dmp", ".zip", ".txt"}:
                continue
            if dry_run:
                print(f"[cleanup_headless_dumps] would remove {artifact}")
            else:
                try:
                    artifact.unlink()
                    removed += 1
                    print(f"[cleanup_headless_dumps] removed {artifact}")
                except FileNotFoundError:
                    continue
    return removed


def main() -> int:
    parser = argparse.ArgumentParser(description="Purge headless crash dumps from the Isaac Sim data cache.")
    parser.add_argument(
        "paths",
        nargs="*",
        type=Path,
        default=list(_default_dump_dirs()),
        help="Specific directories to scan for dump files (defaults to discovered overlay cache paths).",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print actions without removing files.")
    args = parser.parse_args()

    existing = [path for path in args.paths if path.exists()]
    if not existing:
        print("[cleanup_headless_dumps] no dump directories found")
        return 0

    removed = cleanup(existing, dry_run=args.dry_run)
    if removed:
        print(f"[cleanup_headless_dumps] removed {removed} artifact(s)")
    else:
        print("[cleanup_headless_dumps] no matching artifacts found")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
