# Changelog

All notable changes to this project will be documented in this file.
The format roughly follows Keep a Changelog (dates in YYYY-MM-DD) and semantic grouping when possible.

## [Unreleased]
### Added
- Joint limit enforcement in gateway (delta clipping via `--joint-spec`); action flags: `joint_limit_clipped`, `clipped_joint_indices`; metrics counter `joint_limit_violation`
- Joint spec mismatch detection (flags: `joint_spec_mismatch`, `expected_dof`, `observed_dof`; counter: `joint_spec_mismatch`; advisory event with cooldown)
- Consecutive deadline miss escalation (CLI: `--deadline-escalate-threshold`, `--degrade-mode-ratio`; counters: `deadline_escalation_events`, `consec_deadline_miss`; advisory event `deadline_escalation`)
### Changed
- (placeholder)
### Removed
- (placeholder)

## [2025-10-08] - Observability & SLA Expansion
### Added
- SLA alert thresholds CLI flags (latency p95, deadline_miss_rate) with cooldown
- Prometheus textfile export (`gateway_metrics.prom`)
- Hash mismatch ring buffer + forensic JSONL log
- Recent sliding window latency quantiles in metrics (`recent_latency_ms`)
- CSV export option in `measure_round_trip.py`
- Event log size-based rotation + optional gzip compression
- deadline_miss_rate metric & counter exposure
- Severity-tiered alerts (WARN/CRIT) for latency / deadline miss
- Prometheus labels (instance_id, policy_kind)
- Transport benchmarking script (`bench_transport.py`)
- Drift analysis tool (`analyze_hash_drift.py`)
- Hash mismatch aggregate counter Prometheus 노출
- RoArm M3 asset scaffold (URDF placeholder, joint_spec.json, asset README)
- Observation/Action schema 확장: `joint_names`, `action_scale_hint`
- Gateway transport escalate advisory (`--auto-transport-escalate-p95`)
- Performance baseline 문서 (`docs/PERF_BASELINE.md`)
- Isaac side control loop stub (`scripts/isaac_roarm_loop_stub.py`)

### Changed
- README updated to reflect mature gateway (no longer a stub)
- OBSERVABILITY doc expanded with alerts, Prometheus, forensics
- Architecture decision doc enhancement status + transport escalate 완료 표시

## [2025-10-07] - Core IPC & Safety
### Added
- Watchdog deadline with safe zero-action fallback (`--deadline-ms`)
- Randomization hash echo mechanism + mismatch flag
- Correlation ID event logging (JSONL) and latency metrics flush
- Policy inference profiling timings
- Policy loaders: SB3 PPO zip, Torch .pt/.pth, dummy linear JSON spec

### Changed
- Architecture decision doc updated with completed transition steps

## [2025-10-06] - Initial Dual Environment & IPC Skeleton
### Added
- Dual environment architecture separation (Isaac 3.11 vs RL 3.12)
- TCP JSON line-delimited IPC gateway skeleton
- Observation/Action JSON schema + validation tests
- Basic README, domain randomization loader, dummy PPO training script
- Initial integration tests (ping, policy load, schema validation)

---
Future entries will be appended above as features are merged.
