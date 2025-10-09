#!/usr/bin/env bash
set -euo pipefail

# Isaac Sim environment precheck
# - Detect install root
# - Report version and bundled Python
# - Attempt minimal module import check via helper scripts

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
echo "[precheck] repo_dir=$REPO_DIR"

# 1) Detect root
CANDIDATES=()
if [[ -n "${ISAAC_SIM_ROOT:-}" ]]; then CANDIDATES+=("$ISAAC_SIM_ROOT"); fi
CANDIDATES+=("$HOME/.local/share/ov/pkg/isaac-sim" "$HOME/.local/share/ov/pkg/isaac_sim" \
            "$HOME/.local/share/ov/pkg/isaac-sim-*" "$HOME/.local/share/ov/pkg/isaac_sim-*" \
            "$HOME/isaac-sim" "$HOME/isaac_sim" \
            "/opt/nvidia/isaac-sim" "/opt/isaac-sim")

FOUND_ROOT=""
for p in "${CANDIDATES[@]}"; do
  for x in $(ls -d $p 2>/dev/null || true); do
    if [[ -d "$x" ]]; then FOUND_ROOT="$x"; break 2; fi
  done
done

if [[ -z "$FOUND_ROOT" ]]; then
  echo "[precheck] Isaac Sim root not found. Set ISAAC_SIM_ROOT or install Isaac Sim."
  echo "{\"ok\":false,\"reason\":\"root_not_found\"}"
  exit 0
fi

echo "[precheck] isaac_root=$FOUND_ROOT"

# 2) Version
VER_FILE="$FOUND_ROOT/version.txt"
if [[ -f "$VER_FILE" ]]; then
  VER_STR="$(cat "$VER_FILE" | tr -d '\n' | tr -d '\r')"
  echo "[precheck] version_file=$VER_STR"
else
  echo "[precheck] version_file=missing"
fi

# 3) Bundled Python check
PY_VER=""
if [[ -x "$FOUND_ROOT/python.sh" ]]; then
  PY_VER="$($FOUND_ROOT/python.sh - <<'PY'
import sys; print(sys.version)
PY
)"
  echo "[precheck] bundled_python via python.sh: $PY_VER"
elif [[ -f "$FOUND_ROOT/setup_python_env.sh" ]]; then
  # shellcheck disable=SC1090
  source "$FOUND_ROOT/setup_python_env.sh" >/dev/null 2>&1 || true
  if command -v python >/dev/null 2>&1; then
    PY_VER="$(python - <<'PY'
import sys; print(sys.version)
PY
)"
    echo "[precheck] bundled_python via setup_python_env.sh: $PY_VER"
  else
    echo "[precheck] python not available after setup_python_env.sh"
  fi
else
  echo "[precheck] Neither python.sh nor setup_python_env.sh found"
fi

# 4) Minimal import checks (best-effort)
IMPORT_OK="unknown"
if [[ -x "$FOUND_ROOT/python.sh" ]]; then
  if "$FOUND_ROOT/python.sh" "$REPO_DIR/scripts/check_isaac_import.py"; then
    IMPORT_OK="true"
  else
    IMPORT_OK="false"
  fi
elif [[ -f "$FOUND_ROOT/setup_python_env.sh" ]]; then
  # shellcheck disable=SC1090
  source "$FOUND_ROOT/setup_python_env.sh" >/dev/null 2>&1 || true
  if python "$REPO_DIR/scripts/check_isaac_import.py"; then
    IMPORT_OK="true"
  else
    IMPORT_OK="false"
  fi
else
  IMPORT_OK="unavailable_env"
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
  'version_file': os.path.exists('$VER_FILE'),
  'bundled_python': "$PY_VER".strip(),
  'system_python': "$SYS_PY_VER".strip(),
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
