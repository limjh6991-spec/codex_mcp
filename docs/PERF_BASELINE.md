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

## Placeholder Baseline (to be filled after real run)
| Scenario | Samples | p50 | p90 | p95 | p99 | Notes |
|----------|---------|-----|-----|-----|-----|-------|
| gateway_idle_dummy_policy | TBD | - | - | - | - | Dummy linear policy, no Isaac step |
| gateway_sb3_loaded | TBD | - | - | - | - | PPO zip loaded |
| future_isacc_loop_10hz | TBD | - | - | - | - | With articulation update |
| future_isacc_loop_30hz | TBD | - | - | - | - | With articulation update |

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
