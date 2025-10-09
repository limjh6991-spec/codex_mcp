#!/usr/bin/env bash
# Collect environment diagnostics for RoArm M3 Sim2Real stack.
# Output is saved to logs/YYYY-MM-DD/env_diag_*.txt
set -euo pipefail

TS=$(date +%F_%H-%M-%S)
OUTDIR="logs/$(date +%F)"
mkdir -p "$OUTDIR"
OUT="$OUTDIR/env_diag_${TS}.txt"

log(){ echo -e "$1" | tee -a "$OUT"; }
run(){ echo "> $*" | tee -a "$OUT"; bash -lc "$*" 2>&1 | tee -a "$OUT"; echo | tee -a "$OUT"; }

log "# RoArm Environment Diagnostic ($TS)\n"

log "## System"
run 'uname -a'
run 'lsb_release -a || cat /etc/os-release'
run 'echo SHELL=$SHELL'

log "## Python (system)"
run 'which python || true'
run 'python --version || true'
run 'python -c "import sys; print(sys.version);" || true'

log "## Virtualenv / Conda"
run 'echo VIRTUAL_ENV=${VIRTUAL_ENV:-}'
run 'conda env list || echo "conda not available"'

log "## GPU / Drivers"
run 'nvidia-smi || echo "nvidia-smi not available"'

log "## ROS2"
run 'ros2 doctor || echo "ros2 not available"'

log "## Git"
run 'git status -sb'
run 'git log -1 --oneline'

log "## VS Code settings (MCP)"
run 'sed -n "1,200p" .vscode/settings.json || true'

log "## Isaac Precheck"
run 'scripts/isaac_precheck.sh || true'

log "## Isaac Import (bundled)"
run 'if [ -x "$ISAAC_SIM_ROOT/python.sh" ]; then "$ISAAC_SIM_ROOT/python.sh" scripts/check_isaac_import.py; else echo "python.sh not found"; fi'

log "## MCP Ports (if any)"
run 'ss -ltnp | head -n 50 || true'

log "# Done: $OUT"
