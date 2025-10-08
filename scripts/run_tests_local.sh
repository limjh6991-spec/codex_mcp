#!/usr/bin/env bash
# Run tests with system / active venv Python, avoiding IsaacSim site-packages pollution.
set -euo pipefail
PY_BIN=${PY_BIN:-python3}
export PYTHONPATH=$(pwd):${PYTHONPATH:-}
exec $PY_BIN -m pytest -q "$@"
