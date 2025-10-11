#!/usr/bin/env bash
# shellcheck shell=bash
set -euo pipefail

# Launch Isaac Sim GUI from the repository root.
# Usage:
#   bash scripts/launch_isaacsim_gui.sh [-- kit args...]
# Environment variables:
#   ISAACSIM_VENV: override default venv path (defaults to $HOME/isaacsim-venv)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ISAACSIM_VENV:-$HOME/isaacsim-venv}"
ISAAC_BIN="$VENV_DIR/bin/isaacsim"

if [[ ! -x "$ISAAC_BIN" ]]; then
  echo "[launch-isaacsim] 실행 파일을 찾을 수 없습니다: $ISAAC_BIN" >&2
  echo "  ISAACSIM_VENV 환경 변수를 이용해 가상환경 경로를 지정해 주세요." >&2
  exit 127
fi

# Isaac Sim 환경 활성화 (필수 모듈 경로 설정)
# shellcheck source=/dev/null
source "$REPO_ROOT/scripts/activate_isaacsim_env.sh" "$VENV_DIR"

# CUDA, RTX 등 GUI 모드를 위한 기본 설정을 명시적으로 보장
export ISAACSIM_PXR_WORKAROUND_DISABLE=1
export OMNI_KIT_ACCEPT_EULA=${OMNI_KIT_ACCEPT_EULA:-YES}

echo "[launch-isaacsim] Launching GUI via $ISAAC_BIN" >&2
exec "$ISAAC_BIN" "$@"
