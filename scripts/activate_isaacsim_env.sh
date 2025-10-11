#!/usr/bin/env bash
# shellcheck shell=bash
set -euo pipefail

# Isaac Sim pip environment activator
# Usage: source scripts/activate_isaacsim_env.sh [venv_path]
# Default venv_path is $HOME/isaacsim-venv

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "[activate-isaacsim] Please source this script instead of executing it." >&2
  echo "  source $0" >&2
  exit 1
fi

VENV_DIR="${1:-$HOME/isaacsim-venv}"
if [[ ! -d "$VENV_DIR" ]]; then
  echo "[activate-isaacsim] virtualenv not found at $VENV_DIR" >&2
  return 1
fi

if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
  echo "[activate-isaacsim] missing activate script under $VENV_DIR/bin" >&2
  return 1
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

export OMNI_KIT_ACCEPT_EULA=${OMNI_KIT_ACCEPT_EULA:-YES}

mapfile -t __isaac_paths < <(OMNI_KIT_ACCEPT_EULA=YES python - <<'PY'
import glob
import os
import sys

try:
  import isaacsim
except ImportError as exc:
  sys.exit(f"import isaacsim failed: {exc}")

root = os.path.dirname(os.path.abspath(os.path.realpath(isaacsim.__file__)))
print(root)

candidates = sorted(
  glob.glob(os.path.join(root, "extscache", "omni.usd.libs-*")),
  reverse=True,
)
print(candidates[0] if candidates else "")
PY
)

if (( ${#__isaac_paths[@]} < 2 )); then
  echo "[activate-isaacsim] unable to resolve isaacsim package metadata" >&2
  return 1
fi

__isaac_root="${__isaac_paths[0]}"
__usd_libs_dir="${__isaac_paths[1]}"

unset __isaac_paths

if [[ -z "$__isaac_root" ]]; then
  echo "[activate-isaacsim] unable to resolve isaacsim site-packages path" >&2
  return 1
fi

if [[ -z "$__usd_libs_dir" || ! -d "$__usd_libs_dir" ]]; then
  echo "[activate-isaacsim] USD libs extension not found under $__isaac_root/extscache" >&2
  return 1
fi

export ISAACSIM_ROOT="$__isaac_root"
export ISAACSIM_USD_LIBS="$__usd_libs_dir"

PYTHONPATH_ENTRY="$ISAACSIM_USD_LIBS"
if [[ -z "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$PYTHONPATH_ENTRY"
else
  case ":$PYTHONPATH:" in
    *":$PYTHONPATH_ENTRY:"*) ;;
    *) export PYTHONPATH="$PYTHONPATH_ENTRY:$PYTHONPATH" ;;
  esac
fi

LD_ENTRY="$ISAACSIM_USD_LIBS/bin"
if [[ -d "$LD_ENTRY" ]]; then
  declare -a __ld_parts=()
  __ld_parts+=("$LD_ENTRY")
  __ld_parts+=("/usr/lib/x86_64-linux-gnu")
  if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
    IFS=':' read -r -a __existing_ld <<<"$LD_LIBRARY_PATH"
    for item in "${__existing_ld[@]}"; do
      [[ -n "$item" ]] && __ld_parts+=("$item")
    done
  fi
  LD_LIBRARY_PATH="$(printf '%s:' "${__ld_parts[@]}")"
  LD_LIBRARY_PATH="${LD_LIBRARY_PATH%:}"
  export LD_LIBRARY_PATH
  unset __ld_parts __existing_ld
else
  echo "[activate-isaacsim] warning: $LD_ENTRY missing; USD shared libs may not resolve" >&2
fi

if [[ "${ACTIVATE_ISAACSIM_QUIET:-0}" != "1" ]]; then
  python - <<'PY'
from pxr import Usd
import isaacsim
import os

print("[activate-isaacsim] pxr imported from:", os.path.dirname(Usd.__file__))
print("[activate-isaacsim] isaacsim root:", os.path.dirname(isaacsim.__file__))
PY
fi

unset __isaac_root __usd_libs_dir PYTHONPATH_ENTRY LD_ENTRY
