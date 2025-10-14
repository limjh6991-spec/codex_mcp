# Project Status
_Last updated: 2025-10-14 (20:55 UTC)_

## Environment Snapshot
| Item | Value | Notes |
|------|-------|-------|
| Isaac Sim Version | 5.0 (pip, headless-capable) | Installed inside dedicated venv |
| Python Venv | `~/isaacsim-venv` | Activate via `scripts/activate_isaacsim_env.sh` |
| GPU | RTX 5090 + AMD iGPU | CUDA 12.8 기반 PyTorch 나이틀리(2.10.0.dev+cu128)에서 GPU 학습 정상 동작 |
| Extension Overrides | TOML under `~/.local/share/ov/data/Kit/Isaac-Sim Full/5.0/exts/user/` | Requires launcher flags to guarantee disable |

## Workboard
### Open Todos
1. Diagnose Isaac GUI 세션이 원격 환경에서 빈 화면으로 보이는 문제(Omniverse UI 출력 경로 확인 및 캡처 지원 대안 마련).
2. 500-step GUI 재생에서 누적 보상이 -2600대에 머무는 원인을 파악하고 목표 추적/보상 항목을 점검.
3. Wire `scripts/verify_usd_roarm_m3.py` smoke run into VS Code task / CI (headless) for regression coverage.
4. Draft real-world parameter bounds for `configs/domain_randomization.yaml` (friction, latency, mass, gripper contact).

### In Progress
- PyTorch nightly 안정성 모니터링 및 정식 sm_120 대응 빌드 전환 타이밍 검토.
- Joint alias 매핑 검증 및 GUI 재생 로그 기반 분석.

### Blocked / Watchlist
- Real hardware calibration pending — gather friction/mass measurements once rig is ready.
- Extension persistence: reliance on KIT dependency tree means some packages may re-enable until launcher flags are tuned.

### Recently Completed
- `training/train_ppo.py`에 SB3 디바이스 스위치(`--device {auto,cpu,cuda}`) 및 정책 입출력 플래그 추가.
- CUDA 12.8 대응 PyTorch 나이틀리를 설치하고 Isaac Sim PPO GPU 스모크/소크/재개 테스트를 통과.
- README 및 작업 요약 문서를 최신 현황으로 갱신.
- `/World/roarm_m3` fixed-base 전환 완료: ArticulationRootAPI 적용, `physics:fixedBase = True` 속성 명시, `/World/roarm_m3/root_joint` 제거 및 검증 스크립트 통과.
- `scripts/verify_usd_roarm_m3.py`에 조인트 부착 검사와 계층 덤프 CLI를 복구하고 headless 실행 경로를 재검증.
- `scripts/open_roarm_m3_gui.py` 추가: Isaac Sim GUI에서 `assets/roarm_m3/usd/roarm_m3.generated.usd` 바로 로딩.
- Waveshare 공식 위키 요약(`resource/roarm_M3/wiki_summary.md`) 작성 및 외부 자료 링크 정리.
- Isaac RL 환경에 조인트 별칭 매핑과 목표 재매핑을 도입하고 도메인 랜덤라이제이션 범위를 재조정.
- `training/train_ppo.py --device cpu` 재학습 및 `scripts/play_roarm_policy.py` GUI 재생 500스텝 실행(시각 출력 이슈 확인, 로그 정리 완료).
