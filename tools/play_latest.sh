#!/usr/bin/env bash
set -euo pipefail

LATEST_ZIP=$(ls -1t policies/ppo_roarm_*/latest.zip 2>/dev/null | head -n1)
if [[ -z "${LATEST_ZIP}" ]]; then
  echo "[play_latest] 최신 정책을 찾지 못했습니다" >&2
  exit 1
fi
VECPATH="$(dirname "${LATEST_ZIP}")/vecnorm"

python scripts/play_roarm_policy.py \
  --policy "${LATEST_ZIP}" \
  --vecnorm "${VECPATH}" \
  --usd assets/roarm_m3/usd/roarm_m3_patched.usd \
  --prim /World/roarm \
  --episodes 3 \
  --slow-ms 10

read -r -p "메모(없으면 Enter): " NOTE
if [[ -n "${NOTE}" ]]; then
  NOTES_DIR="$(dirname "${LATEST_ZIP}")"
  NOTES_FILE="${NOTES_DIR}/playback_notes.txt"
  printf '%s  %s\n' "$(date +'%F %T')" "${NOTE}" >> "${NOTES_FILE}"
  echo "[play_latest] 노트를 ${NOTES_FILE} 에 기록했습니다"
fi
