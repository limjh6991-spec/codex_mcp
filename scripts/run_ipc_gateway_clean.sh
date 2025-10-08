#!/usr/bin/env bash
# Clean launcher for ipc_policy_gateway.py removing IsaacSim path pollution.
# Usage: ./scripts/run_ipc_gateway_clean.sh [--host 127.0.0.1] [--port 45123]
set -euo pipefail
HOST=127.0.0.1
PORT=45123
while [[ $# -gt 0 ]]; do
  case "$1" in
    --host) HOST="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    *) echo "Unknown arg: $1"; exit 1;;
  esac
done
# Activate venv if exists (keeps PATH)
if [[ -f .venv/bin/activate ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

# Strip Isaac specific pollution only
unset ISAAC_SIM_ROOT
PYTHONPATH_CLEAN="$(pwd)"
export PYTHONPATH="$PYTHONPATH_CLEAN"

echo "[ipc_clean] Launching gateway on $HOST:$PORT (python: $(command -v python))"
exec python scripts/ipc_policy_gateway.py --host "$HOST" --port "$PORT"
