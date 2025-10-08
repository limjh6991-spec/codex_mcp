#!/usr/bin/env bash
set -euo pipefail

TOTAL=${1:-2000}
SEED=${2:-0}
LOGDIR=logs/train_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOGDIR"
python training/train_ppo.py --total_timesteps "$TOTAL" --seed "$SEED" 2>&1 | tee "$LOGDIR/output.log"
echo "Logs: $LOGDIR/output.log"
