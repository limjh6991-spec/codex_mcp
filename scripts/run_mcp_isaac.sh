#!/usr/bin/env bash
set -euo pipefail

: "${ISAAC_SIM_PATH:?ISAAC_SIM_PATH not set}" 
export PYTHONUNBUFFERED=1
python mcp_servers/isaac_controller_server.py "$@"
