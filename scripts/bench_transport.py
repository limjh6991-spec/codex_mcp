#!/usr/bin/env python
"""
Transport Benchmark Tool

Measures round-trip latency distribution for the existing TCP JSON line IPC gateway
and (optionally) placeholder alternative transports (future: ZeroMQ, shared memory).

Current implementation:
- TCP mode: open persistent socket, send N observation messages, measure response latency.
- JSON payload size scaling (vary joint dimension) to observe serialization impact.
- Warm-up iterations excluded from stats.
- Outputs quantiles, basic stats, and optional raw CSV with per-iteration timings.

Future extension hooks:
- ZMQ REQ/REP or DEALER/ROUTER.
- Shared memory ring buffer (writer/reader indices) + side-channel sync.
- gRPC unary or bidirectional stream.

Usage examples:
  python scripts/bench_transport.py --host 127.0.0.1 --port 45123 \
      --iterations 2000 --warmup 100 --joints 18 --csv bench_tcp.csv

  # Compare different joint dimensions
  for j in 6 12 18 24 36; do \
    python scripts/bench_transport.py --joints $j --iterations 1000 --warmup 200 --quiet; \
  done

Metrics reported:
- count, mean, std, min, max
- p50, p90, p95, p99
- recent window (last W, default 200) p50/p95 (quick drift check)

CSV columns: iter, latency_ms
"""
from __future__ import annotations
import argparse, json, math, socket, statistics, time, csv, sys
from typing import List, Dict


def quantiles(samples: List[float], qs=(0.5,0.9,0.95,0.99)) -> Dict[float,float]:
    if not samples:
        return {q: float('nan') for q in qs}
    s = sorted(samples)
    n = len(s)
    out = {}
    for q in qs:
        if n == 1:
            out[q] = s[0]
            continue
        rank = q * (n - 1)
        lo = int(math.floor(rank))
        hi = int(math.ceil(rank))
        if lo == hi:
            out[q] = s[lo]
        else:
            w = rank - lo
            out[q] = s[lo]*(1-w) + s[hi]*w
    return out

def run_tcp(host: str, port: int, joints: int, iterations: int, warmup: int, timeout: float) -> List[float]:
    latencies = []
    obs_template = {
        "type": "obs",
        "data": {
            "q": [0.0]*joints,
            "dq": [0.0]*joints,
            "rand_hash": "benchhash"
        }
    }
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.settimeout(timeout)
        # warmup
        for _ in range(warmup):
            s.sendall(json.dumps(obs_template).encode()+b"\n")
            try:
                s.recv(4096)
            except Exception:
                pass
        # measure
        for _ in range(iterations):
            start = time.perf_counter()
            s.sendall(json.dumps(obs_template).encode()+b"\n")
            try:
                s.recv(4096)
            except Exception:
                # timeout / error => record as NaN placeholder
                latencies.append(float('nan'))
                continue
            latencies.append((time.perf_counter()-start)*1000.0)
    return latencies

def summarize(samples: List[float]):
    # filter NaN for stats but keep original for error rate
    clean = [x for x in samples if not math.isnan(x)]
    err = len(samples) - len(clean)
    count = len(clean)
    if not clean:
        return {
            'count': 0,
            'errors': err,
            'mean': float('nan'), 'std': float('nan'), 'min': float('nan'), 'max': float('nan'),
            'p50': float('nan'), 'p90': float('nan'), 'p95': float('nan'), 'p99': float('nan'),
            'recent_p50': float('nan'), 'recent_p95': float('nan'),
            'error_rate': 1.0 if len(samples) else 0.0,
        }
    mean = statistics.mean(clean)
    std = statistics.pstdev(clean) if count > 1 else 0.0
    q = quantiles(clean)
    recent = clean[-200:] if len(clean) > 200 else clean
    rq = quantiles(recent, qs=(0.5,0.95))
    return {
        'count': count,
        'errors': err,
        'error_rate': round(err/len(samples), 6) if samples else 0.0,
        'mean': round(mean,3), 'std': round(std,3),
        'min': round(min(clean),3), 'max': round(max(clean),3),
        'p50': round(q[0.5],3), 'p90': round(q[0.9],3), 'p95': round(q[0.95],3), 'p99': round(q[0.99],3),
        'recent_p50': round(rq[0.5],3), 'recent_p95': round(rq[0.95],3),
    }

def main():
    ap = argparse.ArgumentParser(description="Transport latency benchmark (TCP baseline)")
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=45123)
    ap.add_argument('--mode', choices=['tcp'], default='tcp', help='Transport mode (tcp only for now)')
    ap.add_argument('--joints', type=int, default=18)
    ap.add_argument('--iterations', type=int, default=1000)
    ap.add_argument('--warmup', type=int, default=100)
    ap.add_argument('--timeout', type=float, default=2.0, help='Socket timeout seconds')
    ap.add_argument('--csv', type=str, help='Optional CSV output path')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    if args.mode == 'tcp':
        lats = run_tcp(args.host, args.port, args.joints, args.iterations, args.warmup, args.timeout)
    else:
        print('Unsupported mode')
        sys.exit(2)

    stats = summarize(lats)
    if not args.quiet:
        print("=== Transport Benchmark (mode=tcp) ===")
        print(f"joints={args.joints} iterations={args.iterations} warmup={args.warmup}")
        for k in ['count','errors','error_rate','mean','std','min','max','p50','p90','p95','p99','recent_p50','recent_p95']:
            print(f"{k}: {stats[k]}")
    if args.csv:
        try:
            with open(args.csv, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['iter','latency_ms'])
                for i, v in enumerate(lats):
                    w.writerow([i, v])
        except Exception as e:
            print(f"Failed to write CSV: {e}", file=sys.stderr)

if __name__ == '__main__':
    main()
