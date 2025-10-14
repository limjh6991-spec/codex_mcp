#!/usr/bin/env python3
"""Validate headless Isaac Sim logs for crash signatures and GUI regressions."""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Iterable

_CRASH_PATTERNS: tuple[tuple[str, str], ...] = (
    ("minidump", r"crash\] Wrote dump file"),
    ("allocator", r"unsorted double linked list corrupted"),
    ("manipulator", r"NameError: ManipulatorFactory"),
)

_GUI_FLOOD_PATTERN = re.compile(r"Action 'omni\.kit\.window\.file")


def _read_lines(path: Path) -> Iterable[str]:
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            yield line.rstrip("\n")


def check_log(path: Path, *, max_hotkey: int) -> int:
    """Return 0 if the log passes validation else 1."""

    failures: list[str] = []
    hotkey_hits = 0

    for line in _read_lines(path):
        if _GUI_FLOOD_PATTERN.search(line):
            hotkey_hits += 1
        for label, pattern in _CRASH_PATTERNS:
            if pattern in line:
                failures.append(f"matched {label}: {line}")

    if hotkey_hits > max_hotkey:
        failures.append(f"hotkey warning flood: {hotkey_hits} hits > {max_hotkey}")

    if failures:
        for entry in failures:
            print(f"[check_headless_log] FAIL: {entry}")
        return 1

    print(f"[check_headless_log] PASS: no crash signatures detected ({hotkey_hits} hotkey warnings)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate headless run logs for crash patterns.")
    parser.add_argument("log", type=Path, help="Path to the Isaac Sim headless log file to inspect.")
    parser.add_argument(
        "--max-hotkey",
        type=int,
        default=5,
        help="Maximum allowed occurrences of GUI hotkey warnings before treating the run as failed.",
    )
    args = parser.parse_args()

    if not args.log.exists():
        parser.error(f"log file not found: {args.log}")

    return check_log(args.log, max_hotkey=args.max_hotkey)


if __name__ == "__main__":
    raise SystemExit(main())
