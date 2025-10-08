# Observability Guide

This document describes the runtime observability features provided by the IPC policy gateway and supporting scripts.

## 1. Event Logging (Correlation IDs)
- File: `logs/ipc_gateway_events.jsonl`
- Each line: JSON object with phases:
  - `phase: recv` captures raw inbound message
  - `phase: send` captures response payload
- `corr_id`:
  - Client may provide; otherwise server generates (UUID hex)
  - Preserved across request/response for correlation
- Suggested usage:
  - For debugging a failed action, grep by `corr_id`
  - For timing, compare wall timestamps of paired recv/send events if finer than internal latency needed

Example line (abbreviated):
```json
{"corr_id":"95b7...","remote":"127.0.0.1:53244","msg_type":"obs","phase":"send","resp":{"type":"action","data":{...}}}
```

## 2. Latency Metrics Aggregation
- File: `logs/ipc_metrics.json` (periodically rewritten ~5s interval)
- Structure:
```json
{
  "policy_latency_ms": {
    "count": 42,
    "mean": 5.123,
    "std": 1.02,
    "min": 3.0,
    "max": 9.5,
    "p50": 5.0,
    "p90": 6.7,
    "p95": 7.8,
    "p99": 9.2
  },
  "updated_ts": 1733660000.123
}
```
- Quantiles computed from in-memory sample window (recent up to ~5000 samples; older are truncated)
- Revisit condition: Architecture decision triggers transport reconsideration if sustained `p95 > 60ms` (see `ARCH_DECISION_DUAL_ENV.md` section 11)
 
- Counters (added):
  - `counters.deadline_miss`: 누적 deadline 초과 횟수 (action safe fallback 발생)
  - `counters.rand_hash_mismatch`: 예상 hash 대비 불일치 발생 누적 (재현성 drift 감지) — forensic 세부 항목은 `hash_mismatch_events.jsonl` 및 `scripts/analyze_hash_drift.py` 참고
  - `counters.deadline_miss_rate`: deadline_miss / total action count (비율, 0.0~1.0)
  - `counters.joint_limit_violation`: (stub) 조인트 한계 초과 감지 카운터(Isaac articulation 통합 시 활성화 예정)

### 2.2 Recent Window Metrics
`recent_latency_ms` 섹션: 마지막 최대 200개 샘플에 대한 p50/p90/p95/p99 추이(단기 편차 관찰용).

### 2.1 Metrics 관련 CLI
- `--metrics-interval <sec>`: flush 주기 (기본 5.0, 최소 0.5)
- `--deadline-ms <ms>`: watchdog 시간 제한
- `--max-log-mb <MB>`: 이벤트 로그 파일 용량 초과 시 timestamp suffix로 단순 rename 회전
- `--compress-rotated`: 회전된 이벤트 로그 gzip 압축 (`.gz`)

### 2.3 Latency 측정 유틸
`scripts/measure_round_trip.py`:
```
python scripts/measure_round_trip.py --iters 200 --q-dim 12 --csv out.csv
```
출력: 요약 JSON 표준 출력 + (선택) CSV (`idx,latency_ms`).

## 3. Watchdog Deadline
- CLI flag: `--deadline-ms <float>`
- Behavior:
  - Measures total handling time from message receipt to action packaging
  - If pre-inference elapsed already exceeds deadline, inference is skipped (fast fail safe action)
  - If total time post-inference exceeds deadline, result action is overridden with zero delta
  - Response adds `deadline_miss: true` in `data`
- Safe Action Strategy: zero `delta` matching observed joint dimension
- Observability/Debugging:
  - `deadline_miss` flag surfaces in action payload and event log `resp`
  - Latency is still recorded for statistical tracking

## 4. Randomization Hash Echo
- Purpose: reproducibility and detection of unintended simulation parameter drift
- Input (observation message): may include `rand_hash` and optionally `expected_rand_hash`
- Output (action message): echoes `rand_hash`; sets `rand_hash_mismatch: true` if `expected_rand_hash` provided and differs
- Intended Workflow:
  1. Simulator includes `rand_hash` in every observation (hash of domain randomization state)
  2. Policy side can assert continuity by sending `expected_rand_hash` (last known / planned)
  3. Mismatch triggers logging; future extension could escalate to error or pause

## 5. Policy Inference Profiling
- Fields under `data.profiling` for successful actions:
  - `preprocess_ms`: vector assembly, lightweight transforms
  - `infer_ms`: model forward / predict call
- Total end-to-end time: `policy_latency_ms` (may exceed sum due to scheduling or post-processing)

## 6. Error Path Observability
- Common error responses:
  - `{"type":"error","error":"bad_json"}`: malformed line
  - `{"type":"error","error":"schema_violation"}`: observation failed schema
  - `{"type":"error","error":"policy_unavailable"}`: inference requested before policy load
  - `{"type":"error","error":"load_failed"}`: policy load path or format issue
- All errors get logged with correlation ID for traceability

## 7. Log Rotation / Size Considerations
- Current implementation appends indefinitely; recommended next steps (not yet implemented):
  - Size-based rotation (>100MB)
  - Compression of rotated segments
  - Structured ingestion (e.g., sending JSONL to vector store for analysis)

## 8. Integration Testing Coverage
- `tests/test_ipc_gateway_integration.py` currently validates:
  - Ping round-trip
  - Policy load (dummy JSON)
  - Observation action response with `rand_hash` echo under watchdog pressure
- Future additions (planned):
  - Explicit deadline miss assertion path with induced inference delay stub
  - Quantile metric sanity check (after multiple obs)

## 9. Extensibility Roadmap
| Area | Next Step | Rationale |
|------|-----------|-----------|
| Metrics | Add deadline_miss count rate | SLA breach monitoring |
| Hash Echo | Persist hash transitions timeline | Drift forensics |
| Logging | Add log rotation/compression | Prevent disk bloat |
| Transport | Evaluate ZeroMQ / shared memory | If p95 latency breach triggers |
| Safety | Add consecutive deadline miss escalation | Failsafe behavior |

## 10. Quick Reference
| Feature | Location | Key Fields |
|---------|----------|------------|
| Events | `logs/ipc_gateway_events.jsonl` | corr_id, phase, msg_type |
| Metrics | `logs/ipc_metrics.json` | p50/p90/p95/p99, count |
| Watchdog | action.data | deadline_miss |
| Rand Hash | action.data | rand_hash, rand_hash_mismatch |
| Profiling | action.data.profiling | preprocess_ms, infer_ms |

---
Change log:
- v1 (initial): correlation IDs, latency stats, watchdog, rand_hash echo.

## 11. SLA Alerting
- Flags:
  - `--alert-latency-p95-ms <ms>`: 전체 p95 latency 임계 초과 시 `sla_alert` 이벤트 (쿨다운 30s)
  - `--alert-deadline-miss-rate <ratio>`: deadline_miss_rate 초과 시 동일 이벤트
- Event 예시:
```json
{"phase":"alert","type":"sla_alert","reasons":["p95_latency 65.2ms > threshold 60ms"],"p95_latency_ms":65.2,"deadline_miss_rate":0.012}
```

## 12. Prometheus Export
- `--prometheus-textfile <path>` 사용 시 flush마다 Prometheus textfile 포맷 작성
- Metric 네이밍: `gateway_<category>_<field>` (예: `gateway_policy_latency_ms_p95`)

## 13. Hash Mismatch Forensics
- 파일: `logs/hash_mismatch_events.jsonl` (각 mismatch append)
- 메모리 ring buffer 최근 100개 유지 (과거 절반씩 축소)
- 활용: randomization drift 원인 추적, 재현성 breakdown 시퀀스 재구성

## 14. Alert 운영 가이드 (초기)
| 항목 | 초기 임계 | 연속 조건 | 후속 액션 |
|------|----------|-----------|-----------|
| p95 latency | 60ms | 3+ flush | 직렬화/네트워크 프로파일, ZeroMQ/SHM 검토 (자동 advisory 가능: `--auto-transport-escalate-p95`) |
| deadline_miss_rate | 0.02 | 2+ flush | 관측 벡터 축소, 정책 경량화 |
| hash mismatch | >5 / min | 즉시 | seed 재확인, 랜덤화 구성 diff |
| joint_limit_violation | TBD | 1+ (중요) | 관절 한계/스케일 조정, action scaling 재평가 |

---
- v2: recent window metrics, deadline_miss_rate, log rotation, gzip, Prometheus export, SLA alerts, hash mismatch forensic log.
