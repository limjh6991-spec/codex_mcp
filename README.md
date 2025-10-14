# Codex + MCP 통합 로봇팔 (RoArm M3) Isaac Sim RL 환경

## 한눈에 보기
- VS Code + GitHub Copilot(MCP) 조합으로 Isaac Sim ↔ 강화학습 ↔ Sim2Real 검증을 하나의 워크플로로 묶습니다.
- Isaac 전용 파이썬(3.11)과 RL/분석 파이썬(3.12)을 분리한 이중 런타임이 기본 전제입니다.
- 일일 로그(`docs/daily/`)와 상태 보드(`docs/STATUS.md`)를 통해 작업 흐름과 잔여 이슈를 추적합니다.
- Isaac 확장 토글을 자동화하는 `scripts/manage_isaacsim_extensions.py`와 각종 환경 점검 스크립트를 제공합니다.

### 2025-10-14 업데이트
- `configs/domain_randomization.yaml`의 관절 목표 범위를 중간값 중심으로 재조정하여 비현실적인 목표가 샘플되는 문제를 완화했습니다.
- `src/envs/isaac_roarm_env.py`에 USD ↔ 조인트 설정 간 별칭 맵을 구축하고 랜덤 목표 재매핑을 통합해 학습·재생 루프가 동일한 순서를 따르도록 정리했습니다.
- 새 설정으로 `training/train_ppo.py --device cpu` 재학습을 수행하여 `policies/ppo_roarm_retrained.zip`을 생성했습니다.
- `scripts/play_roarm_policy.py` GUI 재생을 실행해 500스텝 수집까지는 동작을 확인했으나, 원격 세션에서는 화면 출력이 비어 있는 현상이 반복되어 추가 조사 항목으로 남겨두었습니다.

### 2025-10-13 업데이트
- `roarm_app/runtime/run_loop.py`에 USD 스테이지 종료 전 이벤트 스트림 해제, `detach_stage` 호출, PhysX 전용 정리 유틸 순차 실행, asyncio 태스크 취소 등 안전한 종료 루틴을 추가했습니다. 헤드리스 세션에서 복수의 경고가 이어지던 경우에도 Kit/SimulationApp이 중복 종료되지 않도록 `_KIT_SHUTDOWN_DONE` 가드를 둡니다.
- `roarm_app/cli.py`가 생성하는 headless 블록리스트에 오디오 계열 확장을 포함시키고, `CARB_AUDIO_DISABLED=1`/`CARB_ENABLE_AUDIO=0` 환경변수 설정, `--/exts/<name>/enabled=false` 플래그 삽입으로 Kit 플러그인 단계에서 즉시 비활성화하도록 보강했습니다.
- Isaac RL 환경(`src/envs/isaac_roarm_env.py`)이 직접 `collect_extension_blocklist`와 `expand_prefix_blocklist`를 호출하여 SimulationApp 인스턴스화 직후 다시 한 번 확장 매니저와 동기화합니다. CLI 이외 경로로 학습 스크립트를 구동하더라도 GUI/Replicator/오디오 확장이 재활성화되지 않습니다.
- 최신 GPU PPO 검증(`training/train_ppo.py --env-kind isaac --device cuda --total_timesteps 512`)은 성공적으로 정책을 저장했지만, Replicator 카테고리 제거 실패/`USD stage detach not called`/오디오 컨텍스트 누수 경고가 여전히 발생합니다. 해당 이슈는 다음 릴리스에서 추가 정비가 필요합니다.

#### 2025-10-13 GPU PPO 로그 요약
- 실행 명령: `~/isaacsim-venv/bin/python training/train_ppo.py --env-kind isaac --device cuda --total_timesteps 512 --seed 42`
- 정책 산출물: `policies/ppo_roarm_gpu_validation.zip`
- 잔여 경고: Replicator 카테고리 제거 실패, USD Stage detach 미호출, PhysX 히스토리 잔존, `carb.audio.context` 누수, 플러그인 재귀 언로드. 세부 로그는 `logs/2025-10-13-gpu-ppo-summary.md` 참고.

### 2025-10-12 업데이트
- `training/train_ppo.py`에 `--device {auto,cpu,cuda}` 옵션을 추가하여 Stable-Baselines3가 사용할 연산 디바이스를 제어할 수 있게 했습니다. Isaac Sim은 GPU 메모리를 초기화하므로 PyTorch `cu128` 빌드 설치 후 `--device cuda`로 전환하면 즉시 GPU 학습을 복원할 수 있고, 구버전 PyTorch가 남아 있을 경우엔 `--device cpu`로 안전하게 동작합니다.
- Isaac Sim 전용 Python 환경(3.11)에서 PPO 스모크 테스트(`--env-kind isaac`, `--total_timesteps 2`)를 CPU 경로로 통과시켜 정책(`policies/ppo_roarm.zip`) 저장을 확인했습니다.
- 장기적으로는 PyTorch와 Isaac Sim이 동일한 CUDA 드라이버/SM 조합을 사용하도록 재빌드 또는 NVIDIA 제공 binary를 확보해야 합니다. 관련 후속작업은 `docs/STATUS.md` 및 `docs/오늘_작업_요약.md`에 정리했습니다.

> 최신 진행 상황은 항상 `docs/STATUS.md`에서 확인할 수 있습니다.

## 저장소 구조 (2025-10-11 기준)
```
codex_mcp/
  ├─ configs/                     # 도메인 랜덤라이제이션, 스키마, 조인트 맵
  ├─ docs/
  │    ├─ STATUS.md               # 현재 진행 상황 & TODO 보드
  │    ├─ daily/                  # YYYY-MM-DD 일일 로그
  │    ├─ ROADMAP.md 등 참고 문서
  ├─ mcp_servers/                 # Isaac 제어용 MCP 서버 스켈레톤
  ├─ scripts/                     # 환경 점검, Isaac 전용 도구, 학습 헬퍼
  │    ├─ activate_isaacsim_env.sh
  │    ├─ isaac_precheck.sh
  │    ├─ manage_isaacsim_extensions.py
  │    ├─ run_isaac_tool.sh / run_tests_local.sh
  ├─ sim/                         # USD 스테이지 & 검사 유틸리티
  ├─ src/envs/                    # 강화학습 환경 (Dummy + Isaac 예정)
  ├─ training/                    # SB3 기반 학습 스크립트
  ├─ tests/                       # pytest 기반 검증
  ├─ policies/                    # 학습된 정책 산출물 보관
  └─ 배경설명                     # 원문 요구사항 레퍼런스
```

## 필수 준비물
### RL/분석 환경 (Python 3.12)
```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```
- Stable-Baselines3 및 PyTorch CUDA 빌드는 GPU 환경에 맞춰 별도 설치합니다.
- RL 측 스크립트는 `.venv` 활성화를 전제로 합니다.

### Isaac 전용 환경 (Python 3.11)
```bash
python3.11 -m venv ~/isaacsim-venv
source ~/isaacsim-venv/bin/activate
pip install -U pip
pip install isaacsim[all,extscache]==5.0.0 \
  --extra-index-url https://pypi.nvidia.com
```
- Isaac 관련 작업 시 `scripts/activate_isaacsim_env.sh`를 호출하여 `PYTHONPATH`, `LD_LIBRARY_PATH`, `OMNI_KIT_ACCEPT_EULA` 등을 일괄 설정합니다.
- GUI/헤드리스 모두 동일한 환경을 공유하도록 `~/isaacsim-venv` 경로를 기본값으로 사용합니다.

## Isaac 환경 활성화 & 확장 관리
1. Isaac 전용 venv 활성화 후 `source scripts/activate_isaacsim_env.sh` (필요 시 사용자 정의 경로 인자 전달).
2. `scripts/isaac_precheck.sh` 또는 `python scripts/check_isaac_import.py`로 `pxr` 임포트, Python 버전, USD 경로를 점검합니다.
3. 특정 확장을 영구적으로 끄거나 켜야 할 경우:
   ```bash
   # 예: 두 확장을 비활성화
   python scripts/manage_isaacsim_extensions.py disable \
     omni.isaac.asset_browser omni.isaac.franka

   # override 제거
   python scripts/manage_isaacsim_extensions.py remove omni.isaac.franka
   ```
   생성되는 TOML은 `~/.local/share/ov/data/Kit/Isaac-Sim Full/5.0/exts/user/` 아래에 저장되며, 런처 플래그(`--disable`, `--enableOnly`)와 함께 사용할 때 가장 확실하게 적용됩니다. 실험 결과와 권장 조합은 `docs/STATUS.md`의 "Open Todos" 섹션을 참고하세요.

4. Headless 학습 런처: `python scripts/open_roarm_m3_gui.py --mode train`을 사용하면 `scripts/config/roarm_headless.overlay.kit`을 Isaac Sim 앱 디렉터리에 복사해 GUI/Replicator 계열 확장을 선제적으로 차단합니다. 기본적으로 `KIT_USE_EGL=1`, `ENABLE_HEADLESS=1`, `CARB_APP_QUIET_SHUTDOWN=1` 등을 자동 설정하며, `ROARM_ENABLE_REPLICATOR=1`로 재활성화할 수 있습니다. 종료 시에는 타임라인 정지→USD 스테이지 닫기→Kit/SimulationApp 순으로 정리하여 headless 세그폴트를 방지합니다.

## 강화학습 파이프라인
- 기본 Dummy 환경은 `src/envs/base_env.py`와 `training/train_ppo.py`로 구성되어 있으며, SB3 PPO를 통해 즉시 학습을 돌려볼 수 있습니다.
- 실행 예시:
  ```bash
  python training/train_ppo.py --total_timesteps 2000 --seed 0
  ```
  결과 모델은 `policies/ppo_dummy_roarm.zip`에 저장됩니다.
- 추가 플래그
  - `--record-rollouts`: 학습 대신 샘플 수집 전용 루프
  - `--record-callback-rollouts`: 학습 중 SB3 Callback으로 에피소드 로그 저장
  - `--reward-config`: 외부 YAML 기반 보상 구성값 병합
  - `--device {auto,cpu,cuda}`: SB3가 사용할 연산 디바이스 지정. Isaac Sim를 GPU에서 구동하면서도 PyTorch가 GPU를 지원하지 못할 때는 `--device cpu`로 임시 강제할 수 있습니다. (장기적으로는 PyTorch와 Isaac Sim의 CUDA 드라이버/SM 버전을 일치시키는 것이 목표)
- 생성된 `rollouts/<timestamp>/meta.json`에는 도메인 랜덤라이제이션 샘플, 물리 파라미터 리포트 등 메타데이터가 포함됩니다.

## 도메인 랜덤라이제이션 & Sim2Real 계획
- `configs/domain_randomization.yaml`은 `DomainRandomizer`가 로드하여 환경에 주입합니다.
  ```python
  from src.envs.base_env import DummyRoArmM3Env
  env = DummyRoArmM3Env(domain_randomizer="configs/domain_randomization.yaml")
  obs, info = env.reset()
  print(env.randomization)
  ```
- 실기 로봇과의 파라미터 매칭 계획은 `docs/SIM2REAL_GUIDE.md`와 `docs/ROADMAP.md`에 정리되어 있습니다.
- TODO: 실제 Isaac 환경(`src/envs/isaac_roarm_env.py`)을 PPO 루프에 연결하고, 마찰/질량/지연 값을 현장 측정값으로 업데이트해야 합니다. (상세는 `docs/STATUS.md`의 Open Todos 참고)

## 문서화 & 작업 흐름
- `docs/daily/YYYY-MM-DD.md`: 작업 로그 + 이슈 + 다음날 계획. 최신 기록은 `docs/daily/2025-10-13.md`.
- `docs/STATUS.md`: 환경 스냅샷, Open Todos, Blocked 항목 등 상시 업데이트되는 보드.
- 요구사항/배경 참조는 레포 루트의 `배경설명`, 설계 결정은 `docs/ARCH_DECISION_*.md` 시리즈에서 확인합니다.
- 하루를 시작할 때는 `매일_작업_시작_체크리스트.md`로 환경 점검 → README/STATUS 반영 순서를 유지합니다.

## 테스트 & 진단
- RL 측 테스트는 Isaac 경로를 격리한 `scripts/run_tests_local.sh` 또는 환경을 완전히 재설정한 `env -i` 방식으로 실행합니다.
- `scripts/isaac_precheck.sh`: Isaac 전용 venv, `pxr` 모듈, CUDA 드라이버 경고 등을 요약.
- `scripts/run_mcp_isaac.sh`: MCP 서버 실행 전 환경 변수를 확인하고 STDIO 기반 서버를 기동합니다.
- VS Code 명령 팔레트 → `Tasks: Run Task`에서 "Env Diagnostic", "Train PPO (quick)", "View RoArm (USD)" 등의 작업을 자동화했습니다.

## 로깅 & 관측성
- TCP IPC 게이트웨이(`scripts/ipc_policy_gateway.py`)는 정책 로딩, 관측/행동 직렬화, SLA 모니터링을 담당합니다.
- 주요 로그 경로
  - `logs/ipc_gateway_events.jsonl`: 이벤트 + 관측성
  - `logs/ipc_metrics.json`: 통계 및 SLA 상태
  - `logs/gateway_metrics.prom`: Prometheus textfile export
  - `logs/hash_mismatch_events.jsonl`: 랜덤라이제이션 해시 드리프트 포렌식
- 벤치마크·분석 스크립트: `measure_round_trip.py`, `bench_transport.py`, `analyze_hash_drift.py` 등.

## 다음 단계 (요약)
- Replicator 카테고리 제거 실패, `USD stage detach not called`, 오디오 컨텍스트 누수, recursive unload 경고를 재현 가능한 테스트와 함께 해결합니다.
- Isaac 런처 플래그(`--disable`, `--enableOnly`)와 TOML override 조합을 재검증하고, 현재 블록리스트 로직이 모든 경로(스크립트/CLI/MCP)에서 동일하게 적용되는지 확인합니다.
- 실제 Isaac 환경을 PPO 학습 루프에 연결한 뒤, 내일은 Isaac Sim GUI에서 학습된 정책을 로드해 간단한 재생(trajectory playback)을 검증합니다.
- 실기 파라미터(마찰, 질량, 지연) 측정치를 확보해 `configs/domain_randomization.yaml`에 반영하고, RL 관측/행동 스케일 정규화를 마무리합니다.

## 라이선스
MIT
