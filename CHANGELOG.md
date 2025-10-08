# Changelog

All notable changes to this project will be documented in this file.
The format roughly follows Keep a Changelog (dates in YYYY-MM-DD) and semantic grouping when possible.

## [Unreleased]
### Added
- Severity-tiered alerts (WARN/CRIT) for latency / deadline miss (planned)
- Prometheus labels (instance_id, policy_kind) (planned)
- Hash mismatch aggregate counter (planned)
- Transport benchmarking script (planned)
- Drift analysis tool for hash mismatch forensic log (planned)

### Changed
- (planned) Architecture doc to reflect advanced alerting & benchmarking

### Removed
- (none)

## [2025-10-08] - Observability & SLA Expansion
### Added
- SLA alert thresholds CLI flags (latency p95, deadline_miss_rate) with cooldown
- Prometheus textfile export (`gateway_metrics.prom`)
- Hash mismatch ring buffer + forensic JSONL log
- Recent sliding window latency quantiles in metrics (`recent_latency_ms`)
- CSV export option in `measure_round_trip.py`
- Event log size-based rotation + optional gzip compression
- deadline_miss_rate metric & counter exposure

### Changed
- README updated to reflect mature gateway (no longer a stub)
- OBSERVABILITY doc expanded with alerts, Prometheus, forensics

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
