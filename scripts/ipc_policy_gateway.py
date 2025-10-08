#!/usr/bin/env python
"""Simple TCP JSON line gateway (stub) for obs→policy→action loop.

Protocol (line-delimited JSON):
Client sends object with either:
  {"type":"obs","data":{...observation payload...}}
  {"type":"ping"}
Server replies:
  {"type":"ack","ts": <wall_time>} for ping
  {"type":"action","data":{...action stub...}} for obs

This is a placeholder; real integration will load a policy and compute action.
"""
from __future__ import annotations
import socket, json, time, threading, os, traceback, math, statistics, atexit
from typing import Dict, Any, Optional, Callable, Tuple
import uuid

LOG_DIR = os.path.join(os.path.dirname(__file__), '..', 'logs')
os.makedirs(LOG_DIR, exist_ok=True)
LOG_PATH = os.path.join(LOG_DIR, 'ipc_gateway_events.jsonl')
_max_log_bytes: Optional[int] = None
_compress_rotated: bool = False
_alert_latency_p95_ms: Optional[float] = None  # legacy single threshold (treated as WARN if tiered not provided)
_alert_deadline_miss_rate: Optional[float] = None  # legacy single threshold (treated as WARN if tiered not provided)
_alert_latency_p95_ms_warn: Optional[float] = None
_alert_latency_p95_ms_crit: Optional[float] = None
_alert_deadline_miss_rate_warn: Optional[float] = None
_alert_deadline_miss_rate_crit: Optional[float] = None
_last_latency_severity = 0
_last_deadline_miss_severity = 0
_last_alert_ts: float = 0.0
_alert_cooldown_sec: float = 30.0
_prom_textfile: Optional[str] = None
_hash_mismatch_buffer = []  # store last N entries
_hash_mismatch_buffer_max = 100
HASH_MISMATCH_LOG = os.path.join(LOG_DIR, 'hash_mismatch_events.jsonl')

_log_file_lock = threading.Lock()

def _rotate_log_if_needed():
    if _max_log_bytes is None:
        return
    try:
        if os.path.exists(LOG_PATH) and os.path.getsize(LOG_PATH) > _max_log_bytes:
            ts = time.strftime('%Y%m%d-%H%M%S')
            rotated = LOG_PATH + '.' + ts
            os.rename(LOG_PATH, rotated)
            if _compress_rotated:
                import gzip, shutil
                gz_path = rotated + '.gz'
                with open(rotated, 'rb') as fin, gzip.open(gz_path, 'wb') as fout:
                    shutil.copyfileobj(fin, fout)
                try:
                    os.remove(rotated)
                except Exception:
                    pass
    except Exception:
        pass

def _write_event(ev: Dict[str, Any]):
    try:
        ev.setdefault('ts_wall', time.time())
        line = json.dumps(ev, ensure_ascii=False)
        with _log_file_lock:
            _rotate_log_if_needed()
            with open(LOG_PATH, 'a', encoding='utf-8') as f:
                f.write(line + '\n')
    except Exception:
        pass

SCHEMA_PATH = os.path.join(os.path.dirname(__file__), '..', 'configs', 'schemas', 'obs_action_schema.json')

_validate: Optional[Callable[[dict], None]] = None
_schema_loaded: Optional[dict] = None

# Loaded policy container
_policy: Dict[str, Any] = {}

# Latency metrics (simple aggregation)
_lat_lock = threading.Lock()
_lat_samples: list = []  # ring buffer semantics (truncate)
_lat_max_samples = 5000
_lat_count = 0
_lat_sum = 0.0
_lat_sum_sq = 0.0
_lat_min = float('inf')
_lat_max = 0.0
METRICS_PATH = os.path.join(LOG_DIR, 'ipc_metrics.json')
_metrics_interval_sec = 5.0
_metrics_thread_started = False
_deadline_miss_count = 0
_rand_hash_mismatch_count = 0

def _record_latency(ms: float):
    global _lat_count, _lat_sum, _lat_sum_sq, _lat_min, _lat_max
    with _lat_lock:
        _lat_count += 1
        _lat_sum += ms
        _lat_sum_sq += ms * ms
        if ms < _lat_min:
            _lat_min = ms
        if ms > _lat_max:
            _lat_max = ms
        _lat_samples.append(ms)
        if len(_lat_samples) > _lat_max_samples:
            # drop oldest half to avoid continual shifting cost
            del _lat_samples[:len(_lat_samples)//2]

def _compute_quantiles(samples, qs):
    if not samples:
        return {q: None for q in qs}
    data = sorted(samples)
    res = {}
    n = len(data)
    for q in qs:
        if n == 1:
            res[q] = data[0]
            continue
        rank = q * (n - 1)
        lo = int(math.floor(rank))
        hi = int(math.ceil(rank))
        if lo == hi:
            res[q] = data[lo]
        else:
            w = rank - lo
            res[q] = data[lo] * (1-w) + data[hi] * w
    return res

def _flush_metrics():
    with _lat_lock:
        count = _lat_count
        if count == 0:
            snapshot = {}
        else:
            mean = _lat_sum / count
            var = max(0.0, (_lat_sum_sq / count) - mean*mean)
            std = math.sqrt(var)
            qs = _compute_quantiles(_lat_samples, [0.5, 0.9, 0.95, 0.99])
            recent_samples = _lat_samples[-200:] if len(_lat_samples) > 200 else list(_lat_samples)
            rqs = _compute_quantiles(recent_samples, [0.5, 0.9, 0.95, 0.99]) if recent_samples else {0.5:None,0.9:None,0.95:None,0.99:None}
            snapshot = {
                'policy_latency_ms': {
                    'count': count,
                    'mean': round(mean, 3),
                    'std': round(std, 3),
                    'min': round(_lat_min if _lat_min != float('inf') else 0.0, 3),
                    'max': round(_lat_max, 3),
                    'p50': None if qs[0.5] is None else round(qs[0.5], 3),
                    'p90': None if qs[0.9] is None else round(qs[0.9], 3),
                    'p95': None if qs[0.95] is None else round(qs[0.95], 3),
                    'p99': None if qs[0.99] is None else round(qs[0.99], 3),
                },
                'recent_latency_ms': {
                    'window': len(recent_samples),
                    'p50': None if rqs[0.5] is None else round(rqs[0.5], 3),
                    'p90': None if rqs[0.9] is None else round(rqs[0.9], 3),
                    'p95': None if rqs[0.95] is None else round(rqs[0.95], 3),
                    'p99': None if rqs[0.99] is None else round(rqs[0.99], 3),
                },
                'counters': {
                    'deadline_miss': _deadline_miss_count,
                    'rand_hash_mismatch': _rand_hash_mismatch_count,
                    'deadline_miss_rate': round((_deadline_miss_count / count) if count else 0.0, 6),
                },
                'updated_ts': time.time(),
            }
    try:
        tmp = METRICS_PATH + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(snapshot, f, ensure_ascii=False, indent=2)
        os.replace(tmp, METRICS_PATH)
    except Exception:
        pass
    # Prometheus export
    if snapshot and _prom_textfile:
        try:
            lines = []
            pl = snapshot.get('policy_latency_ms', {})
            for k in ('count','mean','std','min','max','p50','p90','p95','p99'):
                v = pl.get(k)
                if v is not None:
                    lines.append(f"gateway_policy_latency_ms_{k} {v}")
            rec = snapshot.get('recent_latency_ms', {})
            for k in ('window','p50','p90','p95','p99'):
                v = rec.get(k)
                if v is not None:
                    lines.append(f"gateway_recent_latency_ms_{k} {v}")
            cnt = snapshot.get('counters', {})
            for k,v in cnt.items():
                if v is not None:
                    lines.append(f"gateway_{k} {v}")
            tmp = _prom_textfile + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
            os.replace(tmp, _prom_textfile)
        except Exception:
            pass
    # SLA alert evaluation with severity (outside lock)
    if snapshot:
        try:
            now = time.time()
            global _last_alert_ts, _last_latency_severity, _last_deadline_miss_severity
            p95 = snapshot.get('policy_latency_ms', {}).get('p95')
            miss_rate = snapshot.get('counters', {}).get('deadline_miss_rate')
            # Determine effective thresholds (tiered overrides legacy single threshold)
            lat_warn = _alert_latency_p95_ms_warn if _alert_latency_p95_ms_warn is not None else _alert_latency_p95_ms
            lat_crit = _alert_latency_p95_ms_crit
            miss_warn = _alert_deadline_miss_rate_warn if _alert_deadline_miss_rate_warn is not None else _alert_deadline_miss_rate
            miss_crit = _alert_deadline_miss_rate_crit

            def _severity(val, warn, crit):
                if val is None:
                    return 0
                if crit is not None and val > crit:
                    return 2
                if warn is not None and val > warn:
                    return 1
                return 0

            sev_latency = _severity(p95, lat_warn, lat_crit)
            sev_miss = _severity(miss_rate, miss_warn, miss_crit)
            # augment snapshot for downstream consumers (only in-process now)
            snapshot.setdefault('severity', {})['latency_p95'] = sev_latency
            snapshot['severity']['deadline_miss_rate'] = sev_miss

            highest = max(sev_latency, sev_miss)
            if highest > 0 and (now - _last_alert_ts >= _alert_cooldown_sec) and (sev_latency != _last_latency_severity or sev_miss != _last_deadline_miss_severity):
                reasons = []
                if sev_latency == 1:
                    reasons.append(f"p95_latency {p95}ms > WARN {lat_warn}ms")
                elif sev_latency == 2:
                    reasons.append(f"p95_latency {p95}ms > CRIT {lat_crit}ms")
                if sev_miss == 1:
                    reasons.append(f"deadline_miss_rate {miss_rate} > WARN {miss_warn}")
                elif sev_miss == 2:
                    reasons.append(f"deadline_miss_rate {miss_rate} > CRIT {miss_crit}")
                _write_event({
                    'phase': 'alert',
                    'type': 'sla_alert',
                    'severity': highest,
                    'sev_latency_p95': sev_latency,
                    'sev_deadline_miss_rate': sev_miss,
                    'reasons': reasons,
                    'p95_latency_ms': p95,
                    'deadline_miss_rate': miss_rate,
                    'ts_wall': now,
                })
                _last_alert_ts = now
            _last_latency_severity = sev_latency
            _last_deadline_miss_severity = sev_miss
        except Exception:
            pass
    return snapshot

def _metrics_loop():
    while True:
        time.sleep(_metrics_interval_sec)
        _flush_metrics()

def _ensure_metrics_thread():
    global _metrics_thread_started
    if _metrics_thread_started:
        return
    t = threading.Thread(target=_metrics_loop, daemon=True)
    t.start()
    _metrics_thread_started = True
    atexit.register(_flush_metrics)

def _log(msg: str):
    print(f"[ipc_gateway] {msg}")

def _try_import_sb3():
    try:
        import stable_baselines3 as sb3  # type: ignore
        return sb3
    except Exception:
        return None

def _try_import_torch():
    try:
        import torch  # type: ignore
        return torch
    except Exception:
        return None

def _load_policy_file(path: str) -> Tuple[bool, str]:
    """Attempt to load policy by file extension.
    Supports:
      - .zip (SB3 PPO) if stable-baselines3 installed
      - .pt/.pth (Torch) if torch installed (assumes Module saved via torch.save(model.state_dict()) requires architecture guess)
      - .json (dummy spec {"type":"linear_dummy","action_dim":N})
    For .pt/.pth without architecture we create a simple single Linear model expecting input dim inferred from first obs (lazy)."""
    global _policy
    ext = os.path.splitext(path)[1].lower()
    if not os.path.exists(path):
        return False, "file_not_found"
    try:
        if ext == '.zip':
            sb3 = _try_import_sb3()
            if not sb3:
                return False, "sb3_missing"
            from stable_baselines3 import PPO  # type: ignore
            model = PPO.load(path, device='cpu')
            _policy = {"kind": "sb3_ppo", "obj": model, "action_dim": int(model.action_space.shape[0])}
            return True, "ok"
        if ext in ('.pt', '.pth'):
            torch = _try_import_torch()
            if not torch:
                return False, "torch_missing"
            state = torch.load(path, map_location='cpu')
            # We'll wrap state dict into a lazy linear model built on first inference
            class LazyLinearPolicy(torch.nn.Module):
                def __init__(self):
                    super().__init__()
                    self.net = None
                    self.out_features = None
                def forward(self, x):
                    if self.net is None:
                        in_features = x.shape[1]
                        # Heuristic: if state has weight/bias use them else random init
                        if isinstance(state, dict) and any(k.endswith('weight') for k in state.keys()):
                            # pick first weight
                            w_key = [k for k in state.keys() if k.endswith('weight')][0]
                            out_f, in_f = state[w_key].shape
                            if in_f == in_features:
                                self.net = torch.nn.Linear(in_f, out_f)
                                self.net.load_state_dict({k.split('.',1)[-1]:v for k,v in state.items() if k.split('.',1)[-1] in ('weight','bias')}, strict=False)
                                self.out_features = out_f
                            else:
                                self.net = torch.nn.Linear(in_features, max(1, min(16, in_features)))
                                self.out_features = self.net.out_features
                        else:
                            self.net = torch.nn.Linear(in_features, max(1, min(16, in_features)))
                            self.out_features = self.net.out_features
                    return self.net(x)
            model = LazyLinearPolicy()
            _policy = {"kind": "torch_lazy", "obj": model, "action_dim": None}
            return True, "ok"
        if ext == '.json':
            with open(path, 'r', encoding='utf-8') as f:
                spec = json.load(f)
            if spec.get('type') == 'linear_dummy':
                adim = int(spec.get('action_dim', 0))
                if adim <= 0:
                    return False, 'bad_action_dim'
                _policy = {"kind": "dummy_zero", "action_dim": adim}
                return True, "ok"
            return False, 'unknown_json_type'
        return False, 'unknown_format'
    except Exception as e:
        return False, f"exception:{type(e).__name__}:{str(e)[:120]}"

def _infer_action(obs_q: list, obs_dq: list) -> Tuple[list, Dict[str, float], Dict[str, Any]]:
    """Run action inference using currently loaded policy. Returns (delta, profiling_ms, extra_meta)."""
    if not _policy:
        raise RuntimeError('policy_unavailable')
    start_pre = time.perf_counter()
    q = obs_q if isinstance(obs_q, list) else []
    dq = obs_dq if isinstance(obs_dq, list) else []
    obs_vec = q + dq
    preprocess_end = time.perf_counter()
    infer_start = preprocess_end
    delta = []
    meta = {"policy_kind": _policy.get('kind')}
    try:
        if _policy['kind'] == 'sb3_ppo':
            model = _policy['obj']
            import numpy as np  # local import
            import math as _m
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')
                act, _ = model.predict(obs_vec, deterministic=True)
            delta = act.tolist()
            meta['action_dim'] = len(delta)
        elif _policy['kind'] == 'torch_lazy':
            torch = _try_import_torch()
            if not torch:
                raise RuntimeError('torch_missing_runtime')
            import numpy as np
            x = torch.tensor([obs_vec], dtype=torch.float32)
            model = _policy['obj']
            out = model(x)
            if _policy.get('action_dim') is None and hasattr(model, 'out_features') and model.out_features:
                _policy['action_dim'] = model.out_features
            delta = out.detach().cpu().numpy().flatten().tolist()
            meta['action_dim'] = len(delta)
        elif _policy['kind'] == 'dummy_zero':
            ad = int(_policy['action_dim'])
            delta = [0.0] * ad
            meta['action_dim'] = ad
        else:
            raise RuntimeError('unsupported_policy_kind')
    except Exception as e:
        raise RuntimeError(f'inference_failed:{type(e).__name__}:{str(e)[:100]}')
    infer_end = time.perf_counter()
    profiling = {
        'preprocess_ms': (preprocess_end - start_pre) * 1000.0,
        'infer_ms': (infer_end - infer_start) * 1000.0,
    }
    return delta, profiling, meta

def _load_schema_validator():
    global _validate, _schema_loaded
    if _validate is not None:
        return
    try:
        with open(SCHEMA_PATH, 'r', encoding='utf-8') as f:
            _schema_loaded = json.load(f)
    except FileNotFoundError:
        print(f"[ipc_gateway] schema not found at {SCHEMA_PATH}; validation disabled")
        return
    # Try fastjsonschema first (speed), fall back to jsonschema
    try:
        import fastjsonschema  # type: ignore
        _validate = fastjsonschema.compile(_schema_loaded)
        print("[ipc_gateway] fastjsonschema validator active")
    except Exception:
        try:
            import jsonschema  # type: ignore
            def _v(instance: dict):
                jsonschema.validate(instance, _schema_loaded)  # may raise
            _validate = _v
            print("[ipc_gateway] jsonschema validator active")
        except Exception as e:
            print(f"[ipc_gateway] validation libs unavailable ({e}); proceeding without schema checks")
            _validate = None

HOST = "127.0.0.1"
PORT = 45123

# Simple echo policy: returns zero delta vector sized like q in observation

def _validate_obs_payload(data: dict) -> Optional[str]:
    if not _validate:
        return None
    # Wrap observation+action into outer structure to match schema expectation if needed
    # Accept either full schema object already or raw observation minimal subset
    try:
        if 'observation' in data or 'action' in data:
            _validate(data)
        else:
            # Build minimal structure for validation attempt
            candidate = {
                "schema_version": _schema_loaded.get("schema_version", 1) if _schema_loaded else 1,
                "observation": {"obs_version": 1, "q": data.get("q", []), "dq": data.get("dq", [])},
                "action": {"action_version": 1, "delta": []},
            }
            _validate(candidate)
    except Exception as e:
        return str(e)
    return None


def handle_client(conn: socket.socket, addr, deadline_ms: Optional[float] = None):
    global _deadline_miss_count, _rand_hash_mismatch_count
    conn.settimeout(60)
    with conn:
        buf = b""
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line.decode("utf-8"))
                except Exception:
                    conn.sendall(b'{"type":"error","error":"bad_json"}\n')
                    continue
                mtype = msg.get("type")
                corr_id = msg.get('corr_id') or uuid.uuid4().hex
                base_event = {"corr_id": corr_id, "remote": f"{addr[0]}:{addr[1]}", "msg_type": mtype}
                _write_event({**base_event, "phase": "recv", "raw": msg})
                start_t = time.perf_counter()
                if mtype == "ping":
                    resp = {"type": "ack", "ts": time.time(), "corr_id": corr_id}
                elif mtype == "load_policy":
                    path = msg.get('path') or msg.get('policy') or msg.get('data', {}).get('path')
                    if not path:
                        resp = {"type": "error", "error": "load_failed", "detail": "no_path", "corr_id": corr_id}
                    else:
                        ok, detail = _load_policy_file(path)
                        if ok:
                            resp = {"type": "load_policy_ack", "path": path, "policy_kind": _policy.get('kind'), "corr_id": corr_id}
                        else:
                            resp = {"type": "error", "error": "load_failed", "detail": detail, "corr_id": corr_id}
                elif mtype == "obs":
                    data = msg.get("data", {})
                    verr = _validate_obs_payload(data)
                    if verr:
                        resp = {"type": "error", "error": "schema_violation", "detail": verr[:240], "corr_id": corr_id}
                        conn.sendall(json.dumps(resp, ensure_ascii=False).encode("utf-8") + b"\n")
                        _write_event({**base_event, "phase": "send", "resp": resp})
                        continue
                    q = data.get("q") or data.get("observation", {}).get("q")
                    size = len(q) if isinstance(q, list) else 0
                    dq = data.get("dq") or data.get("observation", {}).get("dq", [])
                    # Randomization hash echo (reproducibility)
                    rand_hash = data.get("rand_hash") or data.get("observation", {}).get("rand_hash")
                    # Basic mismatch detection placeholder: if client sends expected_rand_hash and differs from rand_hash
                    expected_hash = data.get("expected_rand_hash") or data.get("observation", {}).get("expected_rand_hash")
                    if not _policy:
                        resp = {"type": "error", "error": "policy_unavailable", "corr_id": corr_id}
                    else:
                        try:
                            # Pre-inference watchdog soft check: if already over deadline, skip inference
                            if deadline_ms is not None:
                                elapsed_pre = (time.perf_counter() - start_t) * 1000.0
                                if elapsed_pre > deadline_ms:
                                    safe_delta = [0.0] * size
                                    total_ms = round(elapsed_pre, 3)
                                    _record_latency(total_ms)
                                    action_payload = {
                                        "action_version": 2,
                                        "delta": safe_delta,
                                        "policy_latency_ms": total_ms,
                                        "profiling": {},
                                        "policy_kind": _policy.get('kind'),
                                        "deadline_miss": True,
                                    }
                                    if rand_hash is not None:
                                        action_payload["rand_hash"] = rand_hash
                                    if expected_hash and rand_hash and expected_hash != rand_hash:
                                        action_payload["rand_hash_mismatch"] = True
                                        action_payload["expected_rand_hash"] = expected_hash
                                        _rand_hash_mismatch_count += 1
                                    resp = {"type": "action", "data": action_payload, "corr_id": corr_id}
                                    conn.sendall(json.dumps(resp, ensure_ascii=False).encode("utf-8") + b"\n")
                                    _write_event({**base_event, "phase": "send", "resp": resp})
                                    continue
                            delta, profiling, meta = _infer_action(q, dq)
                            # Adjust length to q dimension if mismatch
                            if size and len(delta) != size:
                                meta['act_dim_mismatch'] = {"policy": len(delta), "obs_q": size}
                                if len(delta) > size:
                                    delta = delta[:size]
                                else:
                                    # pad zeros
                                    delta = delta + [0.0] * (size - len(delta))
                            total_ms = (time.perf_counter() - start_t) * 1000.0
                            total_ms = round(total_ms, 3)
                            deadline_miss = False
                            if deadline_ms is not None and total_ms > deadline_ms:
                                # Override action with safe zero delta and flag miss
                                delta = [0.0] * size
                                deadline_miss = True
                                _deadline_miss_count += 1
                            _record_latency(total_ms)
                            action_payload = {
                                "action_version": 2,
                                "delta": delta,
                                "policy_latency_ms": total_ms,
                                "profiling": {k: round(v, 3) for k, v in profiling.items()},
                                "deadline_miss": deadline_miss,
                                **meta,
                            }
                            if rand_hash is not None:
                                action_payload["rand_hash"] = rand_hash
                            if expected_hash and rand_hash and expected_hash != rand_hash:
                                action_payload["rand_hash_mismatch"] = True
                                action_payload["expected_rand_hash"] = expected_hash
                                _rand_hash_mismatch_count += 1
                                # ring buffer record
                                try:
                                    _hash_mismatch_buffer.append({'ts': time.time(), 'corr_id': corr_id, 'expected': expected_hash, 'actual': rand_hash})
                                    if len(_hash_mismatch_buffer) > _hash_mismatch_buffer_max:
                                        del _hash_mismatch_buffer[:len(_hash_mismatch_buffer)//2]
                                    with open(HASH_MISMATCH_LOG, 'a', encoding='utf-8') as hf:
                                        hf.write(json.dumps({'ts': time.time(), 'corr_id': corr_id, 'expected': expected_hash, 'actual': rand_hash})+'\n')
                                except Exception:
                                    pass
                                try:
                                    _hash_mismatch_buffer.append({'ts': time.time(), 'corr_id': corr_id, 'expected': expected_hash, 'actual': rand_hash})
                                    if len(_hash_mismatch_buffer) > _hash_mismatch_buffer_max:
                                        del _hash_mismatch_buffer[:len(_hash_mismatch_buffer)//2]
                                    with open(HASH_MISMATCH_LOG, 'a', encoding='utf-8') as hf:
                                        hf.write(json.dumps({'ts': time.time(), 'corr_id': corr_id, 'expected': expected_hash, 'actual': rand_hash})+'\n')
                                except Exception:
                                    pass
                            resp = {"type": "action", "data": action_payload, "corr_id": corr_id}
                        except RuntimeError as e:
                            resp = {"type": "error", "error": str(e).split(':',1)[0], "detail": str(e)[:200], "corr_id": corr_id}
                else:
                    resp = {"type": "error", "error": "unknown_type", "corr_id": corr_id}
                try:
                    conn.sendall(json.dumps(resp, ensure_ascii=False).encode("utf-8") + b"\n")
                    _write_event({**base_event, "phase": "send", "resp": resp})
                except Exception:
                    return

def main():
    import argparse
    parser = argparse.ArgumentParser(description="RoArmM3 IPC Policy Gateway")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--policy", help="Optional initial policy path (.zip / .pt / .pth / .json)", default=None)
    parser.add_argument("--deadline-ms", type=float, default=None, help="Optional watchdog deadline (ms) for observation handling; if exceeded returns safe zero action with deadline_miss=true")
    parser.add_argument("--metrics-interval", type=float, default=5.0, help="Metrics flush interval seconds (default 5.0)")
    parser.add_argument("--max-log-mb", type=float, default=None, help="Rotate event log when size exceeds this many MB (simple rename with timestamp suffix)")
    parser.add_argument("--compress-rotated", action='store_true', help="Gzip compress rotated event log files")
    parser.add_argument("--alert-latency-p95-ms", type=float, default=None, help="Emit alert event if overall p95 latency exceeds this ms (cooldown 30s)")
    parser.add_argument("--alert-deadline-miss-rate", type=float, default=None, help="Emit alert event if deadline_miss_rate exceeds this (0.0-1.0), cooldown 30s")
    parser.add_argument("--alert-latency-p95-ms-warn", type=float, default=None, help="WARN threshold for p95 latency (ms); overrides legacy single threshold for WARN if set")
    parser.add_argument("--alert-latency-p95-ms-crit", type=float, default=None, help="CRIT threshold for p95 latency (ms)")
    parser.add_argument("--alert-deadline-miss-rate-warn", type=float, default=None, help="WARN threshold for deadline_miss_rate (0-1); overrides legacy single threshold for WARN if set")
    parser.add_argument("--alert-deadline-miss-rate-crit", type=float, default=None, help="CRIT threshold for deadline_miss_rate (0-1)")
    parser.add_argument("--prometheus-textfile", type=str, default=None, help="Write Prometheus textfile metrics to this path on each flush")
    args = parser.parse_args()
    host = args.host
    port = args.port
    deadline_ms = args.deadline_ms
    global _metrics_interval_sec
    _metrics_interval_sec = max(0.5, float(args.metrics_interval))
    global _max_log_bytes
    _max_log_bytes = int(args.max_log_mb * 1024 * 1024) if args.max_log_mb else None
    global _compress_rotated
    _compress_rotated = bool(args.compress_rotated)
    global _alert_latency_p95_ms, _alert_deadline_miss_rate
    _alert_latency_p95_ms = args.alert_latency_p95_ms
    _alert_deadline_miss_rate = args.alert_deadline_miss_rate
    global _alert_latency_p95_ms_warn, _alert_latency_p95_ms_crit, _alert_deadline_miss_rate_warn, _alert_deadline_miss_rate_crit
    _alert_latency_p95_ms_warn = args.alert_latency_p95_ms_warn
    _alert_latency_p95_ms_crit = args.alert_latency_p95_ms_crit
    _alert_deadline_miss_rate_warn = args.alert_deadline_miss_rate_warn
    _alert_deadline_miss_rate_crit = args.alert_deadline_miss_rate_crit
    global _prom_textfile
    _prom_textfile = args.prometheus_textfile
    _load_schema_validator()
    if args.policy:
        ok, detail = _load_policy_file(args.policy)
        if ok:
            _log(f"policy loaded kind={_policy.get('kind')} path={args.policy}")
        else:
            _log(f"policy load failed path={args.policy} detail={detail}")
    _ensure_metrics_thread()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(5)
        print(f"[ipc_gateway] listening on {host}:{port}")
        while True:
            conn, addr = s.accept()
            threading.Thread(target=handle_client, args=(conn, addr, deadline_ms), daemon=True).start()

if __name__ == "__main__":
    main()
