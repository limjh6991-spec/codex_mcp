# Project Status
_Last updated: 2025-10-11_

## Environment Snapshot
| Item | Value | Notes |
|------|-------|-------|
| Isaac Sim Version | 5.0 (pip, headless-capable) | Installed inside dedicated venv |
| Python Venv | `~/isaacsim-venv` | Activate via `scripts/activate_isaacsim_env.sh` |
| GPU | RTX 5090 + AMD iGPU | AMD is auto-disabled; watch CUDA driver warnings |
| Extension Overrides | TOML under `~/.local/share/ov/data/Kit/Isaac-Sim Full/5.0/exts/user/` | Requires launcher flags to guarantee disable |

## Workboard
### Open Todos
1. Fix `scripts/verify_usd_roarm_m3.py` CLI parsing and reinstate hierarchy dump.
2. Update Isaac Sim launch scripts with explicit `--disable/--enableOnly` extension flags and validate persistence.
3. Integrate `src/envs/isaac_roarm_env.py` with `training/train_ppo.py` and sanity-check observation/action scaling.
4. Draft real-world parameter bounds for `configs/domain_randomization.yaml` (friction, latency, mass, gripper contact).

### In Progress
- None (end-of-day).

### Blocked / Watchlist
- Real hardware calibration pending — gather friction/mass measurements once rig is ready.
- Extension persistence: reliance on KIT dependency tree means some packages may re-enable until launcher flags are tuned.

### Recently Completed
- Added `scripts/manage_isaacsim_extensions.py` and applied to priority extensions.
- Verified USD includes `/World/roarm_m3/link5/hand_tcp` and documented Stage navigation guidance.
- Outlined Sim2Real RL roadmap (USD pre-training → calibration → feedback loop).
