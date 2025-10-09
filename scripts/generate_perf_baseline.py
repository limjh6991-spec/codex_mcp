#!/usr/bin/env python
"""Generate performance baseline measurements for the IPC policy gateway.

Scenarios:
  - Run gateway (in-process import) and send N synthetic observations.
  - Collect per-message latency from action responses (policy_latency_ms in payload) and compute stats.

Usage (basic):
  python scripts/generate_perf_baseline.py --q-dims 12,48 --iters 300

Outputs:
  - JSON summary file (logs/perf_baseline_run_<ts>.json)
  - Optional markdown table snippet for docs/PERF_BASELINE.md

Assumptions:
  - No external policy loaded => dummy_zero action (fast path)
  - For now we reuse direct socket interaction rather than spawning multiple processes per scenario.
"""
from __future__ import annotations
import argparse, json, os, socket, time, statistics, subprocess, tempfile, sys, random
from typing import List, Dict, Any, Tuple

ROOT = os.path.dirname(os.path.dirname(__file__))
LOG_DIR = os.path.join(ROOT, 'logs')
os.makedirs(LOG_DIR, exist_ok=True)

GATEWAY_SCRIPT = os.path.join(ROOT, 'scripts', 'ipc_policy_gateway.py')

def _pick_free_port(start: int = 45000, end: int = 47000, max_attempts: int = 50) -> int:
    for _ in range(max_attempts):
        p = random.randint(start, end)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(("127.0.0.1", p))
                return p
            except OSError:
                continue
    raise RuntimeError("no_free_port")

def _spawn_gateway(port: int, deadline_ms: float | None = None) -> subprocess.Popen:
    cmd = [sys.executable, GATEWAY_SCRIPT, '--port', str(port)]
    if deadline_ms is not None:
        cmd += ['--deadline-ms', str(deadline_ms)]
    # Keep metrics flush short for quick capture
    cmd += ['--metrics-interval', '1.0']
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

def _load_dummy_policy(port: int, action_dim: int) -> None:
    """Create a temporary dummy linear policy spec and load it so gateway returns latency."""
    spec = {"type": "linear_dummy", "action_dim": int(action_dim)}
    tmp_path = os.path.join(LOG_DIR, f"dummy_policy_{action_dim}.json")
    with open(tmp_path, 'w', encoding='utf-8') as f:
        json.dump(spec, f)
    # Send load_policy message
    with socket.create_connection(("127.0.0.1", port), timeout=2.0) as s:
        msg = json.dumps({"type": "load_policy", "path": tmp_path}).encode('utf-8') + b"\n"
        s.sendall(msg)
        # consume response
        try:
            s.recv(4096)
        except Exception:
            pass

def _wait_gateway_ready(port: int, timeout: float = 5.0) -> None:
    start = time.time()
    while time.time() - start < timeout:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5) as s:
                s.sendall(b'{"type":"ping"}\n')
                line = s.recv(4096)
                if line:
                    return
        except Exception:
            time.sleep(0.05)
    raise RuntimeError('gateway_not_ready')

def _run_scenario(port: int, q_dim: int, iters: int) -> Dict[str, Any]:
    latencies = []
    import json as _json
    for i in range(iters):
        q = list(range(q_dim))
        dq = [0.0] * q_dim
        msg = _json.dumps({"type":"obs","data":{"q":q,"dq":dq}}).encode('utf-8') + b"\n"
        with socket.create_connection(("127.0.0.1", port), timeout=2.0) as s:
            s.sendall(msg)
            line = s.recv(8192)
        try:
            resp = _json.loads(line.decode('utf-8'))
            data = resp.get('data') or {}
            lat = data.get('policy_latency_ms')
            if isinstance(lat, (int, float)):
                latencies.append(float(lat))
        except Exception:
            continue
    if not latencies:
        raise RuntimeError('no_latency_samples')
    latencies.sort()
    def pct(p):
        if not latencies:
            return None
        if len(latencies) == 1:
            return latencies[0]
        rank = p * (len(latencies)-1)
        lo = int(rank)
        hi = int(rank+1)
        if hi >= len(latencies):
            return latencies[-1]
        w = rank - lo
        return latencies[lo]*(1-w) + latencies[hi]*w
    mean = sum(latencies)/len(latencies)
    var = statistics.pvariance(latencies) if len(latencies) > 1 else 0.0
    std = var ** 0.5
    return {
        'q_dim': q_dim,
        'iters': iters,
        'count': len(latencies),
        'mean_ms': round(mean, 3),
        'std_ms': round(std, 3),
        'min_ms': round(latencies[0], 3),
        'max_ms': round(latencies[-1], 3),
        'p50_ms': round(pct(0.5), 3),
        'p90_ms': round(pct(0.9), 3),
        'p95_ms': round(pct(0.95), 3),
        'p99_ms': round(pct(0.99), 3),
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--q-dims', default='12,48', help='콤마 구분 q dimension 목록')
    ap.add_argument('--iters', type=int, default=300, help='각 시나리오 반복 횟수')
    ap.add_argument('--deadline-ms', type=float, default=None, help='옵션: deadline 적용하여 miss 여부 관찰')
    ap.add_argument('--output-json', default=None, help='요약 JSON 출력 경로')
    ap.add_argument('--markdown', action='store_true', help='Markdown 테이블 스니펫 출력')
    args = ap.parse_args()

    q_dims = [int(x) for x in args.q_dims.split(',') if x.strip()]
    ts = int(time.time())
    out_json = args.output_json or os.path.join(LOG_DIR, f'perf_baseline_run_{ts}.json')

    port = _pick_free_port()
    proc = _spawn_gateway(port, args.deadline_ms)
    try:
        _wait_gateway_ready(port)
        # Ensure a policy is loaded; choose action_dim = max q_dim
        if q_dims:
            _load_dummy_policy(port, max(q_dims))
        results = []
        for qd in q_dims:
            if qd <= 0:
                continue
            r = _run_scenario(port, qd, args.iters)
            r['policy_kind'] = 'dummy_zero'
            r['timestamp'] = ts
            results.append(r)
        summary = {'timestamp': ts, 'scenarios': results}
        with open(out_json, 'w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print(json.dumps(summary, ensure_ascii=False))
        if args.markdown:
            # Build markdown table
            hdr = '| policy_kind | q_dim | iters | mean_ms | p95_ms | p99_ms | std_ms | max_ms | timestamp |'\
                  '\n|-------------|------|-------|--------|--------|--------|--------|--------|-----------|'
            lines = [hdr]
            for r in results:
                lines.append(f"| {r['policy_kind']} | {r['q_dim']} | {r['iters']} | {r['mean_ms']} | {r['p95_ms']} | {r['p99_ms']} | {r['std_ms']} | {r['max_ms']} | {r['timestamp']} |")
            print('\n'.join(lines))
    finally:
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            proc.wait(timeout=2)
        except Exception:
            proc.kill()

if __name__ == '__main__':
    main()
