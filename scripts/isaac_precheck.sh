#!/usr/bin/env bash
set -euo pipefail

# Isaac Sim environment precheck
# - Detect install root
# - Report version and bundled Python
# - Attempt minimal module import check via helper scripts

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
echo "[precheck] repo_dir=$REPO_DIR"

# 1) Detect environment
VENV_DIR="${ISAACSIM_VENV:-$HOME/isaacsim-venv}"
FOUND_ROOT=""
VENV_PY=""
PX_IMPORT="unknown"
IMPORT_OK="unknown"

if [[ -d "$VENV_DIR" && -f "$VENV_DIR/bin/activate" ]]; then
  echo "[precheck] isaacsim_venv=$VENV_DIR"
  if source "$VENV_DIR/bin/activate"; then
    if ACTIVATE_ISAACSIM_QUIET=1 source "$REPO_DIR/scripts/activate_isaacsim_env.sh" "$VENV_DIR"; then
      FOUND_ROOT="${ISAACSIM_ROOT:-}"
      if [[ -n "$FOUND_ROOT" ]]; then
        VENV_PY="$(python - <<'PY'
import sys
print(sys.version.replace('\n',' '))
PY
)"
        PX_IMPORT="$(python - <<'PY'
try:
    from pxr import Usd  # noqa: F401
    print('true')
except Exception as exc:  # noqa: BLE001
    print(f"false:{exc}")
PY
)"
        if python "$REPO_DIR/scripts/check_isaac_import.py" >/dev/null 2>&1; then
          IMPORT_OK="true"
        else
          IMPORT_OK="false"
        fi
      fi
    else
      echo "[precheck] activate_isaacsim_env.sh failed"
    fi
  fi
fi

# Legacy fallback (ZIP install)
if [[ -z "$FOUND_ROOT" ]]; then
  CANDIDATES=()
  if [[ -n "${ISAAC_SIM_ROOT:-}" ]]; then CANDIDATES+=("$ISAAC_SIM_ROOT"); fi
  CANDIDATES+=("$HOME/.local/share/ov/pkg/isaac-sim" "$HOME/.local/share/ov/pkg/isaac_sim" \
              "$HOME/.local/share/ov/pkg/isaac-sim-*" "$HOME/.local/share/ov/pkg/isaac_sim-*" \
              "$HOME/isaac-sim" "$HOME/isaac_sim" \
              "/opt/nvidia/isaac-sim" "/opt/isaac-sim")

  for p in "${CANDIDATES[@]}"; do
    for x in $(ls -d $p 2>/dev/null || true); do
      if [[ -d "$x" ]]; then FOUND_ROOT="$x"; break 2; fi
    done
  done

  if [[ -n "$FOUND_ROOT" ]]; then
    echo "[precheck] legacy_root=$FOUND_ROOT"
    if [[ -x "$FOUND_ROOT/python.sh" ]]; then
      VENV_PY="$($FOUND_ROOT/python.sh - <<'PY'
import sys; print(sys.version)
PY
)"
      if "$FOUND_ROOT/python.sh" "$REPO_DIR/scripts/check_isaac_import.py" >/dev/null 2>&1; then
        IMPORT_OK="true"
      else
        IMPORT_OK="false"
      fi
    fi
  fi
fi

if [[ -z "$FOUND_ROOT" ]]; then
  echo "[precheck] Isaac Sim environment not detected. Set ISAACSIM_VENV or install Isaac Sim."
  echo "{\"ok\":false,\"reason\":\"root_not_found\"}"
  exit 0
fi

echo "[precheck] isaac_root=$FOUND_ROOT"

# 2) Version file lookup
VER_FILE=""
for candidate in "$FOUND_ROOT/version.txt" "$FOUND_ROOT/VERSION"; do
  if [[ -f "$candidate" ]]; then
    VER_FILE="$candidate"
    break
  fi
done

if [[ -n "$VER_FILE" ]]; then
  VER_STR="$(cat "$VER_FILE" | tr -d '\n\r')"
  echo "[precheck] version_file=$VER_STR"
else
  echo "[precheck] version_file=missing"
fi

# 5) System Python
SYS_PY_VER="$(python - <<'PY'
import sys; print(sys.version)
PY
)"
echo "[precheck] system_python: $SYS_PY_VER"

# 6) JSON summary
python - <<PY
import json, os
print(json.dumps({
  'ok': True,
  'isaac_root': os.environ.get('ISAAC_SIM_ROOT','') or '$FOUND_ROOT',
  'version_file': bool('$VER_FILE'),
  'venv_python': "$VENV_PY".strip(),
  'system_python': "$SYS_PY_VER".strip(),
  'pxr_import': "$PX_IMPORT".strip(),
  'import_ok': "$IMPORT_OK",
}, ensure_ascii=False))
PY

exit 0
#!/usr/bin/env bash
set -euo pipefail

YELLOW="\033[33m"; GREEN="\033[32m"; RED="\033[31m"; NC="\033[0m"

function info(){ echo -e "${YELLOW}[INFO]${NC} $*"; }
function ok(){ echo -e "${GREEN}[OK]${NC} $*"; }
function fail(){ echo -e "${RED}[FAIL]${NC} $*"; }

: "${ISAAC_SIM_PATH:?ISAAC_SIM_PATH not set (export ISAAC_SIM_PATH=/path/to/isaac-sim)}"

info "1) 경로 확인: $ISAAC_SIM_PATH"
[ -d "$ISAAC_SIM_PATH" ] && ok "디렉토리 존재" || { fail "경로 없음"; exit 1; }

if [ -f "$ISAAC_SIM_PATH/version.txt" ]; then
  VERSION=$(cat "$ISAAC_SIM_PATH/version.txt" | head -n1)
  ok "버전 파일: $VERSION"
else
  info "version.txt 없음 → GUI Help > About에서 수동 확인 필요"
fi

info "2) Python API 환경 설정 여부 테스트"
if command -v python >/dev/null 2>&1; then
  python scripts/check_isaac_import.py || { fail "Python 모듈 임포트 실패"; exit 1; }
else
  fail "python 명령을 찾을 수 없음"
  exit 1
fi

info "3) 기본 환경 변수 체크"
for V in ISAAC_SIM_PATH; do
  [ -n "${!V:-}" ] || { fail "환경변수 $V 비어 있음"; exit 1; }
  ok "$V=${!V}"
done

info "4) Headless 실행 권장 플래그 예시"
echo "  ${GREEN}--/kit/python/bin/./python.sh path/to/script.py --enable omni.kit.renderer.core.hydra.engine="ngp"${NC}" | sed 's/--/ /'

ok "Precheck 완료 (GUI 세부 메뉴는 수동 확인 필요)"
