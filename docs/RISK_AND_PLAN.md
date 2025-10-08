# Risk & Resource Assessment (2025-10-08)

## 1. Technical Risk Matrix
| Area | Current State | Risk | Impact | Mitigation / Next Step |
|------|---------------|------|--------|------------------------|
| Isaac Articulation Attach | Retry + prim existence checks | Medium (API version drift) | High (env unusable) | Add version probe + unit test with monkeypatch; expose error_code metrics |
| Joint State Fidelity | Cached fallback if Isaac missing | Low | Medium (training realism) | Add validation test when Isaac available (future CI profile) |
| Physics Randomization Application | Solver iteration + damping scaling partial | Medium | High (Sim2Real gap) | Implement friction/material traversal; add success ratio metric in rollout meta |
| Reward Composer Introspection | Not yet in meta | Low | Medium (debug difficulty) | Implement introspection & log weights per run |
| Multi-Env Rollout Logging | Single env only | Medium | Medium (scaling training) | Extend callback buffering by env index |
| Policy Inference Validation | Basic shape path only | Medium | Medium (runtime errors in MCP) | Add input schema guard + NaN screening + error codes |
| Structured MCP JSON Logging | Not implemented | Medium | Medium (observability) | Create jsonl appender with PII filter list |
| Dependency Segmentation | Single extras block | Low | Medium (deployment footprint) | Split into extras: core, rl, isaac, dev |
| CI Coverage | Missing | High | High | Add GH Actions: lint + type + tests (Isaac subset skipped) |
| Safety Fail-safes | Not implemented | Medium | High (real robot risk) | Implement command rate limiter & emergency stop flag before real bridge |
| DomainRand Coverage Analytics | Not implemented | Low | Medium | Histogram + outlier detection script |
| ROS2 Bridge Design | Not started | Medium | High (integration timeline) | Author interface spec early to de-risk message mismatch |

## 2. Resource Snapshot
- Code Modules Stable: joint API, reward composer, rollout logging core.
- In-Progress: physics parameter real application (friction/env), meta enrichment integrated.
- Test Suite: 7 passing, 2 skipped (Isaac-dependent). No regression after latest changes.
- Logging: Basic JSON snippets inside JointAPI; missing centralized structured sink.

## 3. Immediate Priorities (Recommended Order)
1. Complete Physics Application (link friction + env params) → close Sim2Real fidelity gap early.
2. Multi-env Rollout Support → prepare for scaling PPO training.
3. Reward Term Introspection → faster reward tuning iterations.
4. Policy Inference Input Validation → harden MCP interface.
5. Structured MCP JSON Logging → observability foundation.
6. CI Pipeline + Dependency Extras → reliability & reproducibility.
7. Episode Analytics Script & DomainRand Coverage → quantitative monitoring.
8. Safety / Fail-safe Hooks → prerequisite for real robot bridge.
9. ROS2 Bridge Spec Draft → unblock downstream integration tasks.
10. Advanced Docs (Sim2Real guide, tuning FAQ) → knowledge transfer.

## 4. Detailed Next-Step Breakdown
### A. Physics Completion
- Implement link material friction scaling (static/dynamic attributes) with fallback counters.
- Add env-level parameters (e.g., ground friction) if accessible; else mark gracefully skipped.
- Extend physics_report: friction_applied_count, damping_scale_applied.

### B. Multi-env Rollout
- Detect VecEnv via `env.num_envs`.
- Buffer per-env episode transitions, flush on done.
- Merge physics_report once per reset per env.

### C. Reward Introspection
- Expose internal term contributions from `RewardComposer.compute`.
- Capture per-episode aggregated sums and means into meta.json.

### D. Policy Inference Validation
- Add shape check, NaN/inf guard, timing measurement, structured error codes (e.g., E_OBS_DIM, E_NAN, E_EMPTY).

### E. Structured JSON Logging
- Create `logs/mcp_calls/*.jsonl` sink.
- Fields: ts, tool, request_id, latency_ms, status, error_code(optional), redacted_inputs.
- Redaction list: fields containing tokens like password, secret, key.

### F. CI & Dependencies
- GH Actions workflow: python setup, install dev extras, run `pytest -q`, `ruff`, `mypy`.
- Split extras in pyproject: `[project.optional-dependencies] core, rl, isaac, dev`.

## 5. Open Questions / Assumptions
- Isaac-specific APIs for friction may vary by version (assume PhysX material attributes available as `physxMaterial:*Friction`).
- Real articulation handle availability at randomization time (assumed attached by reset).

## 6. KPIs to Track Post-Implementation
- physics_success_ratio = success_keys / (success_keys + failed_keys).
- reward_terms_drift (stddev across episodes) for stability.
- limit_violation_rate per 1k steps.
- inference_error_rate (MCP policy_infer).

## 7. Risk Mitigation Checklist
- [ ] Add friction application feature flag for quick disable.
- [ ] Add watchdog on apply_delta saturation rate (too high => warning).
- [ ] Persist randomization + physics_report snapshot once per training seed.

---
Generated automatically (2025-10-08). Update this document as milestones complete.
