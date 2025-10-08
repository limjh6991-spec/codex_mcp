#!/usr/bin/env python
"""
Analyze hash mismatch forensic log produced by ipc_policy_gateway.

Input file default: logs/hash_mismatch_events.jsonl
Each line: {"ts": <epoch>, "corr_id": str, "expected": str, "actual": str}

Features:
- Summary counts (total mismatches, distinct expected, distinct actual)
- Top K (expected->actual) transition frequency
- Time clustering (gaps > --cluster-gap seconds start new cluster)
- Optional CSV export of enriched rows (cluster_id, delta_prev_sec)
- Simple heuristic anomaly score: cluster size * median inter-arrival

Usage:
  python scripts/analyze_hash_drift.py --file logs/hash_mismatch_events.jsonl --top 10 --cluster-gap 5 --csv drift.csv
"""
from __future__ import annotations
import argparse, json, os, statistics, math, csv
from collections import Counter, defaultdict
from typing import List, Dict, Any


def load_events(path: str) -> List[Dict[str, Any]]:
    events = []
    if not os.path.exists(path):
        return events
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                ev = json.loads(line)
                if all(k in ev for k in ('ts','expected','actual')):
                    events.append(ev)
            except Exception:
                pass
    events.sort(key=lambda e: e['ts'])
    return events

def cluster_events(events: List[Dict[str, Any]], gap: float):
    clusters = []
    current = []
    last_ts = None
    for ev in events:
        if last_ts is None or ev['ts'] - last_ts <= gap:
            current.append(ev)
        else:
            clusters.append(current)
            current = [ev]
        last_ts = ev['ts']
    if current:
        clusters.append(current)
    return clusters

def analyze(events: List[Dict[str, Any]], cluster_gap: float, top: int):
    if not events:
        return {'empty': True}
    trans_counter = Counter()
    exp_counter = Counter()
    act_counter = Counter()
    for ev in events:
        trans_counter[(ev['expected'], ev['actual'])] += 1
        exp_counter[ev['expected']] += 1
        act_counter[ev['actual']] += 1
    clusters = cluster_events(events, cluster_gap)
    cluster_summaries = []
    for idx, cl in enumerate(clusters):
        ts_list = [e['ts'] for e in cl]
        deltas = [t2 - t1 for t1, t2 in zip(ts_list, ts_list[1:])]
        median_gap = statistics.median(deltas) if deltas else 0.0
        size = len(cl)
        anomaly_score = size * (median_gap if median_gap else 0.0)
        cluster_summaries.append({
            'cluster_id': idx,
            'size': size,
            'start_ts': ts_list[0],
            'end_ts': ts_list[-1],
            'duration': ts_list[-1] - ts_list[0],
            'median_gap': median_gap,
            'anomaly_score': anomaly_score,
        })
    cluster_summaries.sort(key=lambda x: x['anomaly_score'], reverse=True)

    def top_n(counter: Counter, n: int):
        return [{'item': k, 'count': v} for k, v in counter.most_common(n)]

    return {
        'empty': False,
        'total_mismatches': len(events),
        'distinct_expected': len(exp_counter),
        'distinct_actual': len(act_counter),
        'top_transitions': [{'expected': a, 'actual': b, 'count': c} for (a,b), c in trans_counter.most_common(top)],
        'top_expected': top_n(exp_counter, top),
        'top_actual': top_n(act_counter, top),
        'clusters': cluster_summaries,
    }

def write_csv(path: str, events: List[Dict[str, Any]], clusters, cluster_gap: float):
    # map event to cluster id by iterating sequentially
    if not events:
        return
    cluster_iter = iter(clusters)
    current_cluster = next(cluster_iter, None)
    cluster_index = 0
    rows = []
    prev_ts = None
    ci = 0
    for ev in events:
        # Determine cluster id by gap
        if prev_ts is not None and ev['ts'] - prev_ts > cluster_gap:
            cluster_index += 1
        prev_ts = ev['ts']
        rows.append({
            'ts': ev['ts'],
            'expected': ev.get('expected'),
            'actual': ev.get('actual'),
            'corr_id': ev.get('corr_id'),
            'cluster_id': cluster_index,
        })
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['ts','cluster_id','expected','actual','corr_id'])
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description='Analyze hash mismatch forensic log (drift)')
    ap.add_argument('--file', default='logs/hash_mismatch_events.jsonl')
    ap.add_argument('--cluster-gap', type=float, default=5.0, help='Seconds gap to start a new cluster')
    ap.add_argument('--top', type=int, default=10, help='Top N transitions/values to list')
    ap.add_argument('--json', type=str, help='Write full analysis result JSON to path')
    ap.add_argument('--csv', type=str, help='Write flattened events with cluster ids to CSV')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    events = load_events(args.file)
    result = analyze(events, args.cluster_gap, args.top)

    if result.get('empty'):
        print('No mismatch events found.')
        return

    if not args.quiet:
        print('=== Hash Drift Analysis Summary ===')
        print(f"total_mismatches: {result['total_mismatches']}")
        print(f"distinct_expected: {result['distinct_expected']} distinct_actual: {result['distinct_actual']}")
        print('\nTop transitions:')
        for t in result['top_transitions']:
            print(f"  {t['expected']} -> {t['actual']} : {t['count']}")
        print('\nTop expected:')
        for t in result['top_expected']:
            print(f"  {t['item']} : {t['count']}")
        print('\nTop actual:')
        for t in result['top_actual']:
            print(f"  {t['item']} : {t['count']}")
        print('\nTop clusters by anomaly_score:')
        for c in result['clusters'][:10]:
            print(f"  cluster {c['cluster_id']} size={c['size']} duration={round(c['duration'],3)}s median_gap={round(c['median_gap'],3)} anomaly_score={round(c['anomaly_score'],3)}")

    if args.json:
        try:
            with open(args.json, 'w', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"Failed to write JSON: {e}")
    if args.csv:
        try:
            write_csv(args.csv, events, result['clusters'], args.cluster_gap)
        except Exception as e:
            print(f"Failed to write CSV: {e}")

if __name__ == '__main__':
    main()
