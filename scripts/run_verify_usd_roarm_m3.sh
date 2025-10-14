#!/usr/bin/env bash
# Isaac Sim 전용 가상환경을 활성화하고 RoArm M3 USD 검증 스크립트를 실행합니다.
# Usage:
#   ./scripts/run_verify_usd_roarm_m3.sh <stage.usd> [추가 옵션]
# 예시:
#   ./scripts/run_verify_usd_roarm_m3.sh assets/roarm_m3/roarm_m3_stage.usd --hierarchy --max-depth 4

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ $# -lt 1 ]]; then
  echo "사용법: $0 <stage.usd> [추가 옵션]" >&2
  exit 1
fi

USD_ARG="$1"
shift

if [[ -f "$USD_ARG" ]]; then
  USD_PATH="$(realpath "$USD_ARG")"
elif [[ -f "$PROJECT_ROOT/$USD_ARG" ]]; then
  USD_PATH="$(realpath "$PROJECT_ROOT/$USD_ARG")"
else
  echo "[run_verify_usd_roarm_m3] USD 파일을 찾을 수 없습니다: $USD_ARG" >&2
  exit 1
fi

exec "$SCRIPT_DIR/run_isaac_tool.sh" "$SCRIPT_DIR/verify_usd_roarm_m3.py" "$USD_PATH" "$@"
