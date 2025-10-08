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
