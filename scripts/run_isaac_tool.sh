#!/usr/bin/env bash
# Wrapper to execute a Python script with the pip-installed Isaac Sim environment.
# Usage: ./scripts/run_isaac_tool.sh path/to/script.py [args...]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${ISAACSIM_VENV:-$HOME/isaacsim-venv}"

if [[ ! -d "$VENV_DIR" ]]; then
  echo "[run_isaac_tool] Isaac Sim venv not found at $VENV_DIR" >&2
  exit 1
fi

if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
  echo "[run_isaac_tool] missing activate script under $VENV_DIR/bin" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

ACTIVATE_ISAACSIM_QUIET=1 source "$SCRIPT_DIR/activate_isaacsim_env.sh" "$VENV_DIR"

PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
if [[ -z "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$PROJECT_ROOT"
else
  case ":$PYTHONPATH:" in
    *":$PROJECT_ROOT:"*) ;;
    *) export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH" ;;
  esac
fi

exec python "$@"
