#!/usr/bin/env python3
"""Utility to toggle Isaac Sim extension defaults.

This helper writes per-extension TOML descriptors under the user
configuration directory so that selected extensions stay disabled (or
enabled) across launches even when the GUI lacks a save button.

Example usage
-------------
Disable two extensions::

    python scripts/manage_isaacsim_extensions.py disable \
        omni.isaac.asset_browser omni.isaac.franka

Re-enable an extension (writes ``enabled = true``)::

    python scripts/manage_isaacsim_extensions.py enable omni.isaac.franka

By default the script targets the Isaac Sim "Full" experience for
version 5.0, matching the layout produced by the official launcher on
Linux. Use the ``--kit`` or ``--version`` flags if your install differs.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from textwrap import dedent

DEFAULT_KIT = "Isaac-Sim Full"
DEFAULT_VERSION = "5.0"
USER_EXT_SUBDIR = ("exts", "user")


def resolve_user_ext_dir(base: Path, kit: str, version: str) -> Path:
    """Return the directory that stores per-extension TOML overrides."""
    kit_dir = base / kit / version
    ext_dir = kit_dir.joinpath(*USER_EXT_SUBDIR)
    ext_dir.mkdir(parents=True, exist_ok=True)
    return ext_dir


def write_extension_state(ext_dir: Path, extension: str, enabled: bool) -> Path:
    """Create or update the TOML file for *extension* with the desired state."""
    safe_name = extension.strip()
    if not safe_name:
        raise ValueError("Extension name must be non-empty")

    target = ext_dir / f"{safe_name}.toml"
    content = dedent(
        f"""
        [package]
        enabled = {'true' if enabled else 'false'}
        """
    ).lstrip()
    target.write_text(content)
    return target


def delete_extension_state(ext_dir: Path, extension: str) -> Path:
    """Remove the custom TOML override for *extension*, if it exists."""
    target = ext_dir / f"{extension.strip()}.toml"
    if target.exists():
        target.unlink()
    return target


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manage persistent Isaac Sim extension enable/disable state",
    )
    parser.add_argument(
        "action",
        choices=("disable", "enable", "remove"),
        help="Operation to apply. 'remove' deletes the override file entirely.",
    )
    parser.add_argument(
        "extensions",
        nargs="+",
        help="Extension package names, e.g. omni.isaac.asset_browser",
    )
    parser.add_argument(
        "--base",
        type=Path,
        default=Path.home() / ".local" / "share" / "ov" / "data" / "Kit",
        help="Root of the Kit user data directory (default: %(default)s)",
    )
    parser.add_argument(
        "--kit",
        default=DEFAULT_KIT,
        help="Kit experience name (default: %(default)s)",
    )
    parser.add_argument(
        "--version",
        default=DEFAULT_VERSION,
        help="Kit version directory (default: %(default)s)",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    ext_dir = resolve_user_ext_dir(args.base, args.kit, args.version)
    enabled = args.action == "enable"

    for ext in args.extensions:
        if args.action == "remove":
            target = delete_extension_state(ext_dir, ext)
            print(f"Removed override (if present): {target}")
        else:
            target = write_extension_state(ext_dir, ext, enabled)
            state = "enabled" if enabled else "disabled"
            print(f"Set {ext!r} to {state}: {target}")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        raise SystemExit(130)
