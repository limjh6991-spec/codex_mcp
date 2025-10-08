from __future__ import annotations
"""Domain Randomization Coverage Analysis Utilities.

Reads a directory (or JSONL file) of sampled domain randomization dictionaries and
computes coverage statistics vs the configured ranges.

Contract:
    analyze_samples(samples, config) -> CoverageReport(dict)

CoverageReport keys:
    total_samples: int
    flat_stats: { key: { 'min': float, 'max': float, 'mean': float, 'std': float, 'count': int } }
    out_of_range: { key: [values...] }  # values observed outside config range (if range defined)
    missing_keys: [keys not observed]
    unused_config_keys: [keys defined in config but never sampled]
    per_key_hist: { key: { 'bins': [...], 'counts': [...] } } (lightweight fixed bin hist, numeric only)

Design notes:
  * Config can be nested; we flatten with section.key path join using '.'
  * Only numeric scalar values considered for stats; non-numeric ignored (but tracked for presence)
  * Histogram: 10 bins uniform between config range if provided else observed min/max
"""
from dataclasses import dataclass
from typing import Any, Dict, List, Iterable, Tuple
import math

Number = float | int

@dataclass
class CoverageReport:
    total_samples: int
    flat_stats: Dict[str, Dict[str, float]]
    out_of_range: Dict[str, List[Number]]
    missing_keys: List[str]
    unused_config_keys: List[str]
    per_key_hist: Dict[str, Dict[str, List[Number]]]


def _flatten(d: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    out = {}
    for k, v in d.items():
        path = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(_flatten(v, path))
        else:
            out[path] = v
    return out


def _is_range(v: Any) -> bool:
    return (
        isinstance(v, (list, tuple))
        and len(v) == 2
        and all(isinstance(x, (int, float)) for x in v)
    )


def analyze_samples(samples: Iterable[Dict[str, Any]], config: Dict[str, Any]) -> CoverageReport:
    flat_cfg = _flatten(config)
    # Extract numeric ranges from config
    cfg_ranges: Dict[str, Tuple[float, float]] = {
        k: (float(v[0]), float(v[1])) for k, v in flat_cfg.items() if _is_range(v)
    }

    # Initialize accumulators
    stats: Dict[str, List[float]] = {}
    observed_keys: set[str] = set()
    out_of_range: Dict[str, List[Number]] = {k: [] for k in cfg_ranges}

    total = 0
    for s in samples:
        total += 1
        flat_s = _flatten(s)
        for k, v in flat_s.items():
            observed_keys.add(k)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                stats.setdefault(k, []).append(float(v))
                # range check if config range exists
                if k in cfg_ranges:
                    lo, hi = cfg_ranges[k]
                    if not (lo <= float(v) <= hi):
                        out_of_range[k].append(v)

    # Compute numeric stats
    flat_stats: Dict[str, Dict[str, float]] = {}
    for k, arr in stats.items():
        if not arr:
            continue
        n = len(arr)
        mn = min(arr)
        mx = max(arr)
        mean = sum(arr) / n
        var = sum((x - mean) ** 2 for x in arr) / n if n > 0 else 0.0
        std = math.sqrt(var)
        flat_stats[k] = {"count": float(n), "min": mn, "max": mx, "mean": mean, "std": std}

    # Build histograms (only for numeric stats keys)
    per_key_hist: Dict[str, Dict[str, List[Number]]] = {}
    for k, info in flat_stats.items():
        n = int(info["count"])
        if n < 2:
            continue
        # Determine range
        if k in cfg_ranges:
            lo, hi = cfg_ranges[k]
        else:
            lo, hi = info["min"], info["max"]
            if lo == hi:
                continue
        bins = 10
        step = (hi - lo) / bins
        edges = [lo + i * step for i in range(bins + 1)]
        counts = [0] * bins
        for x in stats[k]:
            if x < lo or x > hi:
                continue  # already flagged in out_of_range if within cfg range
            # assign bin (inclusive of lo, exclusive of hi except last)
            b = min(bins - 1, int((x - lo) / step))
            counts[b] += 1
        per_key_hist[k] = {"bins": edges, "counts": counts}

    missing_keys = [k for k in cfg_ranges.keys() if k not in observed_keys]
    # config keys that are not ranges but also never observed
    unused_config_keys = [k for k in flat_cfg.keys() if k not in observed_keys]

    # prune empty out_of_range entries
    out_of_range = {k: v for k, v in out_of_range.items() if v}

    return CoverageReport(
        total_samples=total,
        flat_stats=flat_stats,
        out_of_range=out_of_range,
        missing_keys=missing_keys,
        unused_config_keys=unused_config_keys,
        per_key_hist=per_key_hist,
    )


def report_to_markdown(rep: CoverageReport) -> str:
    lines = [f"# Domain Randomization Coverage", f"Total samples: {rep.total_samples}"]
    lines.append("\n## Key Statistics")
    header = "| key | count | min | max | mean | std | out_of_range |"
    lines.append(header)
    lines.append("|---|---:|---:|---:|---:|---:|---|")
    for k, s in sorted(rep.flat_stats.items()):
        oor = len(rep.out_of_range.get(k, []))
        lines.append(
            f"| {k} | {int(s['count'])} | {s['min']:.6g} | {s['max']:.6g} | {s['mean']:.6g} | {s['std']:.3g} | {oor} |"
        )
    if rep.missing_keys:
        lines.append("\n### Missing Keys (never observed but have range)")
        for k in rep.missing_keys:
            lines.append(f"- {k}")
    if rep.unused_config_keys:
        lines.append("\n### Unused Config Keys (never observed)")
        for k in rep.unused_config_keys:
            lines.append(f"- {k}")
    if rep.out_of_range:
        lines.append("\n### Out of Range Values")
        for k, vals in rep.out_of_range.items():
            lines.append(f"- {k}: {vals[:5]}{'...' if len(vals)>5 else ''}")
    return "\n".join(lines)
