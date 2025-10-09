#!/usr/bin/env python
"""Check latest (or provided) performance baseline JSON against thresholds.

Usage:
  python scripts/check_perf_threshold.py --p95-threshold-ms 60
  python scripts/check_perf_threshold.py --baseline-json logs/perf_baseline_run_1759905545.json --p95-threshold-ms 50

Exit codes:
  0 = OK (all scenarios within thresholds)
  1 = Missing baseline (and not ignored)
  2 = Threshold violation
"""
from __future__ import annotations
import argparse, os, json, glob, sys
from typing import Dict, Any

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'logs')

def _find_latest_baseline() -> str | None:
    files = sorted(glob.glob(os.path.join(LOG_DIR, 'perf_baseline_run_*.json')))
    return files[-1] if files else None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--baseline-json', help='특정 baseline JSON 경로 (없으면 최신 파일 자동 검색)', default=None)
    ap.add_argument('--p95-threshold-ms', type=float, default=60.0, help='허용 최대 p95(ms)')
    ap.add_argument('--fail-on-missing', action='store_true', help='baseline 없음시 실패 처리(exit 1)')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()

    path = args.baseline_json or _find_latest_baseline()
    if not path or not os.path.exists(path):
        msg = {'status':'missing_baseline','searched': path or 'none'}
        print(json.dumps(msg, ensure_ascii=False))
        if args.fail_on_missing:
            sys.exit(1)
        else:
            sys.exit(0)
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    scenarios = data.get('scenarios') or []
    violations = []
    for sc in scenarios:
        p95 = sc.get('p95_ms')
        if isinstance(p95, (int,float)) and p95 > args.p95_threshold_ms:
            violations.append({'q_dim': sc.get('q_dim'), 'p95_ms': p95})
    result: Dict[str, Any] = {
        'baseline_path': path,
        'thresholds': {'p95_ms': args.p95_threshold_ms},
        'scenario_count': len(scenarios),
        'violations': violations,
        'status': 'ok' if not violations else 'violation'
    }
    print(json.dumps(result, ensure_ascii=False))
    if violations:
        sys.exit(2)
    sys.exit(0)

if __name__ == '__main__':
    main()
