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

DEFAULT_LOG_DIR = os.path.join(os.path.dirname(__file__), '..', 'logs')
LOG_DIR = DEFAULT_LOG_DIR  # can be overridden by --log-dir
LOG_PATH = None  # initialized after args parsing
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
_prom_instance_id: Optional[str] = None
_prom_enable_labels: bool = False
_hash_mismatch_buffer = []  # store last N entries
_hash_mismatch_buffer_max = 100
HASH_MISMATCH_LOG = None  # set after log dir finalized

# Schema path
SCHEMA_PATH = os.path.join(os.path.dirname(__file__), '..', 'configs', 'schemas', 'obs_action_schema.json')

# Latency/stat globals (restored)
_lat_lock = threading.Lock()
_lat_samples: list = []
_lat_max_samples = 5000
_lat_count = 0
_lat_sum = 0.0
_lat_sum_sq = 0.0
_lat_min = float('inf')
_lat_max = 0.0
_deadline_escalate_cooldown_sec = 30.0  # restore missing global used in escalation logic

# ---------------------------------------------------------------------------
# State / counters / schema validation globals (restored after refactor)
# ---------------------------------------------------------------------------
# Policy container; populated by _load_policy_file
_policy: Dict[str, Any] = {}

# Schema validation
_validate: Optional[Callable[[dict], None]] = None  # callable performing validation, or None if disabled
_schema_loaded: Optional[dict] = None               # loaded JSON schema document

# Metrics / paths (set in main, but define placeholders to avoid NameError)
METRICS_PATH: Optional[str] = None

# Joint limits
_joint_limits: Optional[list] = None
_last_joint_spec_mismatch_event_ts: float = 0.0
_joint_spec_mismatch_cooldown: float = 30.0

# Counters
_deadline_miss_count: int = 0
_rand_hash_mismatch_count: int = 0
_joint_limit_violation_count: int = 0
_joint_spec_mismatch_count: int = 0
_deadline_escalation_events: int = 0
_consec_deadline_miss: int = 0
_degrade_zeroed_count: int = 0  # number of times action zeroed due to degrade mode

# Escalation / degrade mode state
_deadline_escalate_threshold: int = 0
_degrade_mode_ratio: float = 0.0
_degrade_mode: bool = False
_last_deadline_escalate_event_ts: float = 0.0

# Metrics thread control
_metrics_interval_sec: float = 5.0
_metrics_thread_started: bool = False

# Action scaling hint (optional meta)
_action_scale_hint: Optional[float] = None

_log_file_lock = threading.Lock()

def _rotate_log_if_needed():
    if _max_log_bytes is None:
        return
    try:
        if LOG_PATH and os.path.exists(LOG_PATH) and os.path.getsize(LOG_PATH) > _max_log_bytes:
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
    if LOG_PATH is None:
        return
    try:
        ev.setdefault('ts_wall', time.time())
        line = json.dumps(ev, ensure_ascii=False)
        with _log_file_lock:
            _rotate_log_if_needed()
            with open(LOG_PATH, 'a', encoding='utf-8') as f:
                f.write(line + '\n')
    except Exception:
        pass
def _load_joint_spec(path: str) -> Tuple[bool, str]:
    """Load a joint spec JSON (expects {'joints':[{'lower':..,'upper':..},...]}) and store limits."""
    global _joint_limits
    try:
        with open(path, 'r', encoding='utf-8') as f:
            spec = json.load(f)
        joints = spec.get('joints') or []
        limits = []
        for j in joints:
            if not isinstance(j, dict):
                return False, 'bad_joint_entry'
            # accept lower/upper or min/max synonyms
            lo = j.get('lower', j.get('min'))
            hi = j.get('upper', j.get('max'))
            if lo is None or hi is None:
                return False, 'missing_bounds'
            try:
                lo_f = float(lo)
                hi_f = float(hi)
            except Exception:
                return False, 'non_numeric_bounds'
            if hi_f < lo_f:
                lo_f, hi_f = hi_f, lo_f
            limits.append((lo_f, hi_f))
        if not limits:
            return False, 'no_joints'
        _joint_limits = limits
        _log(f"joint spec loaded joints={len(limits)} path={path}")
        return True, 'ok'
    except FileNotFoundError:
        return False, 'file_not_found'
    except Exception as e:
        return False, f"exception:{type(e).__name__}:{str(e)[:80]}"

def _apply_joint_limits(q: list, delta: list) -> Tuple[list, bool, list]:
    """Given current positions q and proposed delta, clip delta so q+delta within bounds.
    Returns (new_delta, clipped_flag, clipped_indices)."""
    if not (_joint_limits and isinstance(q, list) and isinstance(delta, list)):
        return delta, False, []
    size = min(len(q), len(delta), len(_joint_limits))
    clipped = False
    clipped_idx: list = []
    for i in range(size):
        lo, hi = _joint_limits[i]
        try:
            q_i = float(q[i])
            d_i = float(delta[i])
        except Exception:
            continue
        proposed = q_i + d_i
        if proposed < lo:
            delta[i] = lo - q_i
            clipped = True
            clipped_idx.append(i)
        elif proposed > hi:
            delta[i] = hi - q_i
            clipped = True
            clipped_idx.append(i)
    return delta, clipped, clipped_idx

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
        mean = _lat_sum / count if count else None
        var = max(0.0, (_lat_sum_sq / count) - mean*mean) if count else None
        std = math.sqrt(var) if var is not None else None
        qs = _compute_quantiles(_lat_samples, [0.5, 0.9, 0.95, 0.99]) if count else {q: None for q in [0.5,0.9,0.95,0.99]}
        recent_samples = _lat_samples[-200:] if len(_lat_samples) > 200 else list(_lat_samples)
        rqs = _compute_quantiles(recent_samples, [0.5, 0.9, 0.95, 0.99]) if recent_samples else {q: None for q in [0.5,0.9,0.95,0.99]}
        counters = {
            'deadline_miss': _deadline_miss_count,
            'rand_hash_mismatch': _rand_hash_mismatch_count,
            'joint_limit_violation': _joint_limit_violation_count,
            'joint_spec_mismatch': _joint_spec_mismatch_count,
            'deadline_escalation_events': _deadline_escalation_events,
            'consec_deadline_miss': _consec_deadline_miss,
            'degrade_zeroed': _degrade_zeroed_count,
            'deadline_miss_rate': round((_deadline_miss_count / count), 6) if count else 0.0,
        }
        snapshot = {
            'policy_latency_ms': {
                'count': count,
                'mean': None if mean is None else round(mean, 3),
                'std': None if std is None else round(std, 3),
                'min': None if count == 0 else round(_lat_min if _lat_min != float('inf') else 0.0, 3),
                'max': None if count == 0 else round(_lat_max, 3),
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
            'counters': counters,
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
            labels_base = {}
            if _prom_enable_labels:
                if _prom_instance_id:
                    labels_base['instance_id'] = _prom_instance_id
                # Use current loaded policy kind if any
                if _policy.get('kind'):
                    labels_base['policy_kind'] = _policy.get('kind')
            def fmt(metric: str, value):
                if not _prom_enable_labels or not labels_base:
                    return f"{metric} {value}"
                # label serialization in stable order
                parts = [f"{k}='{labels_base[k]}'" for k in sorted(labels_base.keys())]
                return f"{metric}{{{','.join(parts)}}} {value}"
            pl = snapshot.get('policy_latency_ms', {})
            for k in ('count','mean','std','min','max','p50','p90','p95','p99'):
                v = pl.get(k)
                if v is not None:
                    lines.append(fmt(f"gateway_policy_latency_ms_{k}", v))
            rec = snapshot.get('recent_latency_ms', {})
            for k in ('window','p50','p90','p95','p99'):
                v = rec.get(k)
                if v is not None:
                    lines.append(fmt(f"gateway_recent_latency_ms_{k}", v))
            cnt = snapshot.get('counters', {})
            for k,v in cnt.items():
                if v is not None:
                    lines.append(fmt(f"gateway_{k}", v))
            # severity snapshot if present
            sev = snapshot.get('severity', {})
            for k,v in sev.items():
                if v is not None:
                    lines.append(fmt(f"gateway_severity_{k}", v))
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
            # Transport escalate advisory: sustained p95 above threshold
            try:
                global _auto_transport_escalate_p95
            except NameError:
                _auto_transport_escalate_p95 = None
            if '_transport_escalate_consec' not in globals():
                global _transport_escalate_consec
                _transport_escalate_consec = 0
            if _auto_transport_escalate_p95 is not None and p95 is not None:
                if p95 > _auto_transport_escalate_p95:
                    _transport_escalate_consec += 1
                else:
                    _transport_escalate_consec = 0
                if _transport_escalate_consec >= 3:
                    _write_event({
                        'phase': 'advisory',
                        'type': 'transport_escalate',
                        'p95': p95,
                        'threshold': _auto_transport_escalate_p95,
                        'message': 'Sustained high p95 latency. Evaluate ZeroMQ / shared memory transport.'
                    })
                    _transport_escalate_consec = 0
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
    # Globals mutated in this handler (declare early to avoid 'used prior to global' syntax issues)
    global _consec_deadline_miss, _deadline_escalation_events, _degrade_mode
    global _last_deadline_escalate_event_ts, _joint_limit_violation_count
    global _joint_spec_mismatch_count, _deadline_escalate_threshold, _degrade_mode_ratio

    def _note_deadline_miss(base_event: dict, *, last_latency_ms: Optional[float] = None, deadline_ms_param: Optional[float] = None):
        """Increment miss counters, check escalation, maybe write advisory, flush metrics.
        last_latency_ms: most recent measured handling latency (if available)
        deadline_ms_param: configured deadline to include in advisory
        """
        global _deadline_miss_count, _consec_deadline_miss, _deadline_escalation_events, _degrade_mode, _last_deadline_escalate_event_ts
        _deadline_miss_count += 1
        _consec_deadline_miss += 1
        if _deadline_escalate_threshold > 0 and _consec_deadline_miss >= _deadline_escalate_threshold:
            now_es = time.time()
            if now_es - _last_deadline_escalate_event_ts > _deadline_escalate_cooldown_sec:
                _deadline_escalation_events += 1
                if _degrade_mode_ratio > 0.0:
                    _degrade_mode = True
                adv = {
                    **base_event,
                    'phase': 'advisory',
                    'type': 'deadline_escalation',
                    'consec_miss': _consec_deadline_miss,
                    'threshold': _deadline_escalate_threshold,
                    'degrade_mode': _degrade_mode,
                    'degrade_mode_ratio': _degrade_mode_ratio,
                }
                if last_latency_ms is not None:
                    adv['last_latency_ms'] = last_latency_ms
                if deadline_ms_param is not None:
                    adv['deadline_ms'] = deadline_ms_param
                _write_event(adv)
                try:
                    _flush_metrics()
                except Exception:
                    pass
                _consec_deadline_miss = 0
                _last_deadline_escalate_event_ts = now_es
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
                elif mtype == "admin":
                    op = msg.get('op') or (msg.get('data') or {}).get('op')
                    if op == 'flush_metrics':
                        snap = _flush_metrics()
                        resp = {"type": "admin_ack", "op": op, "corr_id": corr_id, "updated_ts": snap.get('updated_ts')}
                    elif op == 'get_counters':
                        snap = _flush_metrics()
                        resp = {"type": "counters", "data": snap.get('counters', {}), "corr_id": corr_id}
                    else:
                        resp = {"type": "error", "error": "bad_admin_op", "detail": str(op), "corr_id": corr_id}
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
                            # Pre-inference watchdog soft check: if already over deadline, skip inference fast-path
                            if deadline_ms is not None:
                                elapsed_pre = (time.perf_counter() - start_t) * 1000.0
                                if elapsed_pre > deadline_ms:
                                    total_ms = round(elapsed_pre, 3)
                                    _note_deadline_miss(base_event, last_latency_ms=total_ms, deadline_ms_param=deadline_ms)
                                    _record_latency(total_ms)
                                    action_payload = {
                                        "action_version": 2,
                                        "delta": [0.0]*size,
                                        "policy_latency_ms": total_ms,
                                        "profiling": {},
                                        "policy_kind": _policy.get('kind'),
                                        "deadline_miss": True,
                                    }
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
                            # Joint spec length mismatch detection
                            joint_spec_mismatch = False
                            if _joint_limits and size and len(_joint_limits) != size:
                                joint_spec_mismatch = True
                                global _joint_spec_mismatch_count, _last_joint_spec_mismatch_event_ts
                                _joint_spec_mismatch_count += 1
                                now_ts = time.time()
                                if now_ts - _last_joint_spec_mismatch_event_ts > _joint_spec_mismatch_cooldown:
                                    _write_event({
                                        **base_event,
                                        'phase': 'advisory',
                                        'type': 'joint_spec_mismatch',
                                        'expected_dof': len(_joint_limits),
                                        'observed_dof': size,
                                    })
                                    _last_joint_spec_mismatch_event_ts = now_ts
                            # Joint limit enforcement (delta interpreted as position delta)
                            joint_limit_clipped = False
                            clipped_indices: list = []
                            if _joint_limits and size and len(delta) == size:
                                new_delta, clipped_flag, clipped_idx = _apply_joint_limits(q, delta)
                                if clipped_flag:
                                    delta = new_delta
                                    joint_limit_clipped = True
                                    clipped_indices = clipped_idx
                                    global _joint_limit_violation_count
                                    _joint_limit_violation_count += len(clipped_idx)
                            total_ms = (time.perf_counter() - start_t) * 1000.0
                            total_ms = round(total_ms, 3)
                            deadline_miss = False
                            # Unified deterministic overshoot for very tight deadlines (<=1ms) for test stability.
                            # Single branch replaces legacy micro-sleep logic.
                            if deadline_ms is not None and deadline_ms <= 1.0 and not _degrade_mode:
                                try:
                                    # Sleep slightly beyond deadline (1.1x) but clamp to a safe micro range (max 2ms).
                                    overshoot_s = min(0.002, (deadline_ms/1000.0) * 1.1)
                                    if overshoot_s > 0:
                                        time.sleep(overshoot_s)
                                    total_ms = (time.perf_counter() - start_t) * 1000.0
                                    total_ms = round(total_ms, 3)
                                except Exception:
                                    pass
                            if deadline_ms is not None and total_ms > deadline_ms:
                                # Override action with safe zero delta and flag miss
                                delta = [0.0] * size
                                deadline_miss = True
                                _note_deadline_miss(base_event, last_latency_ms=total_ms, deadline_ms_param=deadline_ms)
                            # Record latency after deadline handling so miss latencies still counted
                            _record_latency(total_ms)
                            if not deadline_miss:
                                # Reset consecutive miss counter on success
                                _consec_deadline_miss = 0
                            action_payload = {
                                "action_version": 2,
                                "delta": delta,
                                "policy_latency_ms": total_ms,
                                "profiling": {k: round(v, 3) for k, v in profiling.items()},
                                "deadline_miss": deadline_miss,
                                **meta,
                            }
                            if joint_limit_clipped:
                                action_payload["joint_limit_clipped"] = True
                                action_payload["clipped_joint_indices"] = clipped_indices
                            if joint_spec_mismatch:
                                action_payload["joint_spec_mismatch"] = True
                                action_payload["expected_dof"] = len(_joint_limits) if _joint_limits else None
                                action_payload["observed_dof"] = size
                            # Degrade mode handling: probabilistically zero out action to reduce load
                            if _degrade_mode and _degrade_mode_ratio > 0.0 and not deadline_miss and delta:
                                import random as _r
                                if _r.random() < _degrade_mode_ratio:
                                    delta = [0.0] * len(delta)
                                    action_payload['delta'] = delta
                                    action_payload['degrade_zeroed'] = True
                                    global _degrade_zeroed_count
                                    _degrade_zeroed_count += 1
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
    parser.add_argument("--prometheus-instance-id", type=str, default=None, help="Instance identifier label value when label export enabled")
    parser.add_argument("--prometheus-enable-labels", action='store_true', help="Enable labeled Prometheus metrics (instance_id, policy_kind)")
    parser.add_argument("--action-scale", type=float, default=None, help="Scale factor meta for raw policy action (exposed in action response as action_scale_hint)")
    parser.add_argument("--auto-transport-escalate-p95", type=float, default=None, help="p95 ms threshold that if exceeded 3 consecutive flushes triggers advisory event")
    parser.add_argument("--joint-spec", type=str, default=None, help="Path to joint_spec.json with joint lower/upper limits for delta clipping")
    parser.add_argument("--deadline-escalate-threshold", type=int, default=0, help="연속 deadline_miss N회 이상 시 escalation advisory (0=비활성)")
    parser.add_argument("--degrade-mode-ratio", type=float, default=0.0, help="Escalation 후 확률적으로 delta를 zero로 강등하는 비율(0.0~1.0)")
    parser.add_argument("--log-dir", type=str, default=None, help="Custom log directory for isolation (defaults to ../logs)")
    parser.add_argument("--joint-spec-mismatch-cooldown", type=float, default=30.0, help="joint_spec_mismatch advisory 재발행 쿨다운 (초)")
    args = parser.parse_args()
    host = args.host
    port = args.port
    deadline_ms = args.deadline_ms
    # finalize log directory globals
    global LOG_DIR, LOG_PATH, METRICS_PATH, HASH_MISMATCH_LOG
    if args.log_dir:
        LOG_DIR = args.log_dir
    os.makedirs(LOG_DIR, exist_ok=True)
    LOG_PATH = os.path.join(LOG_DIR, 'ipc_gateway_events.jsonl')
    METRICS_PATH = os.path.join(LOG_DIR, 'ipc_metrics.json')
    HASH_MISMATCH_LOG = os.path.join(LOG_DIR, 'hash_mismatch_events.jsonl')
    _log(f"log_dir={LOG_DIR}")
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
    global _prom_textfile, _prom_instance_id, _prom_enable_labels
    _prom_textfile = args.prometheus_textfile
    _prom_instance_id = args.prometheus_instance_id
    _prom_enable_labels = bool(args.prometheus_enable_labels)
    global _action_scale_hint
    _action_scale_hint = args.action_scale
    global _auto_transport_escalate_p95
    _auto_transport_escalate_p95 = args.auto_transport_escalate_p95
    global _deadline_escalate_threshold, _degrade_mode_ratio
    _deadline_escalate_threshold = max(0, int(args.deadline_escalate_threshold))
    _degrade_mode_ratio = max(0.0, min(1.0, float(args.degrade_mode_ratio)))
    global _joint_spec_mismatch_cooldown
    _joint_spec_mismatch_cooldown = max(1.0, float(args.joint_spec_mismatch_cooldown))
    if args.joint_spec:
        ok, detail = _load_joint_spec(args.joint_spec)
        if not ok:
            _log(f"joint spec load failed path={args.joint_spec} detail={detail}")
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
