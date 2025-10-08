#!/usr/bin/env python3
import argparse, time, socket, json, statistics

def send_recv(sock, msg):
    line = (json.dumps(msg) + '\n').encode()
    t0 = time.perf_counter()
    sock.sendall(line)
    data = b''
    while not data.endswith(b'\n'):
        chunk = sock.recv(4096)
        if not chunk:
            break
        data += chunk
    t1 = time.perf_counter()
    return json.loads(data.decode().strip()), (t1 - t0) * 1000.0

def main():
    ap = argparse.ArgumentParser(description='Measure round-trip latency to IPC gateway')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=45123)
    ap.add_argument('--iters', type=int, default=100)
    ap.add_argument('--q-dim', type=int, default=8)
    ap.add_argument('--dq-dim', type=int, default=0)
    ap.add_argument('--warmup', type=int, default=5)
    ap.add_argument('--policy', help='Optional policy load path before measuring')
    ap.add_argument('--csv', help='Optional path to write raw per-iteration latency ms as CSV (columns: idx,lat_ms)', default=None)
    args = ap.parse_args()

    q = list(range(args.q_dim))
    dq = [0.0]*args.dq_dim

    with socket.create_connection((args.host, args.port), timeout=5) as s:
        if args.policy:
            resp, _ = send_recv(s, {"type":"load_policy","path": args.policy})
            if resp.get('type') != 'load_policy_ack':
                print('Policy load failed:', resp)
        # warmup
        for _ in range(args.warmup):
            send_recv(s, {"type":"obs","data":{"q": q, "dq": dq}})
        samples = []
        for _ in range(args.iters):
            _, ms = send_recv(s, {"type":"obs","data":{"q": q, "dq": dq}})
            samples.append(ms)
    if not samples:
        print('No samples collected')
        return
    samples.sort()
    n = len(samples)
    def pct(p):
        if n == 1: return samples[0]
        rank = p*(n-1)
        lo = int(rank)
        hi = min(n-1, lo+1)
        w = rank - lo
        return samples[lo] if lo==hi else samples[lo]*(1-w)+samples[hi]*w
    summary = {
        'count': n,
        'mean_ms': statistics.mean(samples),
        'min_ms': samples[0],
        'max_ms': samples[-1],
        'p50_ms': pct(0.5),
        'p90_ms': pct(0.9),
        'p95_ms': pct(0.95),
        'p99_ms': pct(0.99),
    }
    print(json.dumps(summary, indent=2))
    if args.csv:
        import csv
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['idx','latency_ms'])
            for i, v in enumerate(samples):
                w.writerow([i, f"{v:.6f}"])

if __name__ == '__main__':
    main()
