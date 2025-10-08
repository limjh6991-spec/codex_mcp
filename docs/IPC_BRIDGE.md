# IPC Bridge (Dual-Environment Boundary)

This document defines the lightweight TCP JSON line protocol between the *Isaac Runtime* (simulation / real robot control side) and the *RL Policy Environment* (training / inference side) under the accepted dual-environment architecture.

## Goals
- Strong isolation between Python runtimes (Isaac Python 3.11 vs RL stack 3.12+)
- Deterministic, schema-based interchange of observations and actions
- Extensible metadata for latency, safety flags, and domain randomization context
- Human-inspectable and easy to replay (line-delimited JSON)

## Transport
- TCP (default host `127.0.0.1`, port `45123`)
- Each message is a single UTF-8 JSON object terminated by `\n`
- Server (default implementation): `scripts/ipc_policy_gateway.py`
- Multiple concurrent clients allowed (thread-per-connection model in stub)

## Message Types
| Direction | Type    | Description |
|-----------|---------|-------------|
| C→S       | `ping`  | Health check / latency baseline |
| S→C       | `ack`   | Response to `ping` with timestamp |
| C→S       | `obs`   | Observation payload requesting an action |
| S→C       | `action`| Action decision (stub returns zeros) |
| any       | `error` | Error surfaced; contains `error` field |

## Observation Message (`obs`)
Schema alignment reference: `configs/schemas/obs_action_schema.json` (draft 2020-12). Minimum stub shape:
```json
{
  "type": "obs",
  "data": {
    "schema_version": 1,
    "timestamp": 1730000000.123,
    "q": [0.0, 1.2, -0.3],
    "q_dot": [0.0, 0.0, 0.0],
    "goal": [0.25, 0.10, 0.30],
    "randomization": {"physics_seed": 12345},
    "safety": {"emergency_stop": false}
  }
}
```

### Required Fields (current stub)
- `q`: Joint positions (list[float])

All other fields optional for stub but recommended for parity with schema. Production implementation MUST validate against the JSON Schema.

## Action Message (`action`)
Stub gateway returns:
```json
{
  "type": "action",
  "data": {
    "action_version": 1,
    "delta": [0.0, 0.0, 0.0],
    "policy_latency_ms": 0.0
  }
}
```

### Planned Extensions
- `logits` or `distribution_parameters`
- `value_estimate`
- `safety_overrides` (boolean or object)
- `watchdog_ms` (deadline remaining)

## Error Message
```json
{"type":"error","error":"bad_json"}
```
Common error codes (planned):
- `bad_json`
- `schema_violation`
- `policy_unavailable`
- `timeout`

## Versioning Strategy
- Wire schema major version increments stored in `obs_action_schema.json` with `version` field.
- Gateway announces supported version during handshake (future enhancement: `hello` message).
- Backward-compatible additions allowed by making new fields optional.

## Latency & Timing (Planned Metrics)
- `policy_latency_ms`: Time from observation receipt to action decision.
- Future: `end_to_end_ms` echoed back by control side to include actuation delay.

## Security Considerations
- Default binds to loopback; for remote use add TLS tunnel or VPN.
- Input length guard (future): reject lines > 256KB to mitigate memory pressure.
- Strict schema validation recommended before policy inference to prevent malformed inputs.

## Deployment Patterns
1. Start gateway in RL environment (loads policy weights, e.g. PPO) — future.
2. Isaac side opens persistent socket, streams `obs` per control tick, receives `action`.
3. Safety layer on Isaac side can override or scale actions before application.

## Testing
A basic smoke test can be performed manually:
```
# Terminal 1
python scripts/ipc_policy_gateway.py

# Terminal 2
python - <<'PY'
import socket, json, time
s=socket.create_connection(("127.0.0.1",45123))
for m in [
  {"type":"ping"},
  {"type":"obs","data":{"q":[0,1,2]}},
]:
  s.sendall(json.dumps(m).encode()+b"\n")
  print(s.recv(4096).decode().strip())
PY
```

## Roadmap
- [ ] JSON Schema validation (fastjsonschema or jsonschema lib)
- [ ] Policy load & hot-reload (FS watcher)
- [ ] Aggregated metrics endpoint (Prometheus text format or JSON)
- [ ] Graceful shutdown handshake (`close` type)
- [ ] Structured error codes enumeration file

## References
- Dual Environment Decision: `docs/ARCH_DECISION_DUAL_ENV.md`
- ROS2 Bridge Design: `docs/ROS2_BRIDGE.md`
- Sim2Real Guide: `docs/SIM2REAL_GUIDE.md`
