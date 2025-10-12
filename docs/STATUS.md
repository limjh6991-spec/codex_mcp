# Project Status
_Last updated: 2025-10-12_

## Environment Snapshot
| Item | Value | Notes |
|------|-------|-------|
| Isaac Sim Version | 5.0 (pip, headless-capable) | Installed inside dedicated venv |
| Python Venv | `~/isaacsim-venv` | Activate via `scripts/activate_isaacsim_env.sh` |
| GPU | RTX 5090 + AMD iGPU | AMD auto-disabled. PyTorch는 임시로 CPU 강제 실행 중 |
| Extension Overrides | TOML under `~/.local/share/ov/data/Kit/Isaac-Sim Full/5.0/exts/user/` | Requires launcher flags to guarantee disable |

## Workboard
### Open Todos
1. Fix `scripts/verify_usd_roarm_m3.py` CLI parsing and reinstate hierarchy dump.
2. Update Isaac Sim launch scripts with explicit `--disable/--enableOnly` extension flags and validate persistence.
3. Integrate `src/envs/isaac_roarm_env.py` with `training/train_ppo.py` and sanity-check observation/action scaling.
4. Draft real-world parameter bounds for `configs/domain_randomization.yaml` (friction, latency, mass, gripper contact).

### In Progress
- PyTorch CUDA(sm_120) 호환 빌드 조사 및 Isaac Sim 런타임과의 버전 정합 계획 수립.

### Blocked / Watchlist
- Real hardware calibration pending — gather friction/mass measurements once rig is ready.
- Extension persistence: reliance on KIT dependency tree means some packages may re-enable until launcher flags are tuned.

### Recently Completed
- `training/train_ppo.py`에 SB3 디바이스 스위치(`--device {auto,cpu,cuda}`) 추가.
- Isaac Sim 환경에서 PPO 스모크 테스트를 CPU 경로로 통과시키고 정책 저장(`policies/ppo_roarm.zip`) 확인.
- README 및 작업 요약 문서를 최신 현황으로 갱신.
