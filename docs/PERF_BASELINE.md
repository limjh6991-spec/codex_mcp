# Performance Baseline (Initial Draft)

Purpose: Track latency & throughput metrics before introducing alternative transports (ZeroMQ, shared memory) or articulation-heavy loops.

## Current Scope
- Policy inference gateway only (no real Isaac sim step yet).
- Placeholder 6-DOF RoArm M3 model not loaded in simulation.

## Measurement Tools
| Tool | Purpose | Notes |
|------|---------|-------|
| `scripts/measure_round_trip.py` | Sample round-trip latency (ping/obs) | Provides quantiles, optional CSV |
| `scripts/bench_transport.py` | Bulk latency benchmark (current TCP) | Future modes: `zmq`, `shm` |
| `scripts/ipc_policy_gateway.py` | Produces full + recent quantiles | Exports Prometheus textfile if enabled |

## Initial Targets
| Metric | Target | Rationale |
|--------|--------|-----------|
| p95 policy_latency_ms | < 60ms | Escalate transport if sustained exceedance |
| Recent window p95 | < 50ms | Early detection of regression |
| deadline_miss_rate | < 0.02 | Maintain control stability |

## Automated Baseline (Latest synthetic run)
아래 표는 `scripts/generate_perf_baseline.py` 실행 결과( dummy_zero 정책, 관절만 증가 )를 기록합니다.

| policy_kind | q_dim | iters | mean_ms | p95_ms | p99_ms | std_ms | max_ms | timestamp |
|-------------|------|-------|--------|--------|--------|--------|--------|-----------|
| dummy_zero | 12 | 120 | 2.072 | 2.456 | 2.622 | 0.274 | 3.264 | 1759905545 |
| dummy_zero | 48 | 120 | 1.970 | 2.101 | 2.245 | 0.086 | 2.562 | 1759905545 |

### How to Refresh
```
python scripts/generate_perf_baseline.py --q-dims 12,48 --iters 300 --markdown > /tmp/perf.json
```
JSON 파일은 `logs/perf_baseline_run_<ts>.json` 에 저장되며, Markdown 테이블을 위 섹션으로 복사 반영합니다.

## Data Collection Procedure
1. Start gateway: `python scripts/ipc_policy_gateway.py --metrics-interval 5 --prometheus-textfile logs/gateway_metrics.prom`
2. Run round-trip sampler: `python scripts/measure_round_trip.py --samples 500`
3. (Optional) Run benchmark: `python scripts/bench_transport.py --samples 5000 --csv bench_tcp.csv`
4. Record quantiles in table; commit changes referencing git hash.

## Future Extensions
- Add articulation step timing (sim_step_ms) and serialization/infer/apply breakdown.
- Include CPU/GPU utilization snapshot.
- Automate regression detection via simple thresholds in CI.

## Revision Log
- 2025-10-08: Initial scaffold created (no empirical values yet).
- 2025-10-08: Added first synthetic baseline (commit: <hash_pending>)
