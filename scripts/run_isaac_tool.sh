#!/usr/bin/env bash
# Wrapper to execute a Python script inside Isaac Sim bundled Python.
# Usage: ./scripts/run_isaac_tool.sh path/to/script.py [args...]
set -euo pipefail
ISAAC_ROOT=${ISAAC_SIM_ROOT:-/home/roarm_m3/isaac_sim}
if [ ! -x "$ISAAC_ROOT/python.sh" ]; then
  echo "[run_isaac_tool] python.sh not found at $ISAAC_ROOT" >&2
  exit 1
fi
exec "$ISAAC_ROOT/python.sh" "$@"
