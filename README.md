# Codex + MCP 통합 로봇팔 (RoArm M3) Isaac Sim RL 환경

## 목표
- VS Code 내 Copilot(Chat) + MCP 서버들을 통해 Isaac Sim ↔ RL 학습 ↔ 정책 테스트를 단일 워크플로우로 오케스트레이션.
- Sim2Real 전이를 고려한 도메인 랜덤라이제이션 및 공통 Env 추상화.

## 구성 개요
```
codex_mcp/
  ├─ pyproject.toml                # Python 패키지/의존성 정의
  ├─ 배경설명                      # 요구사항 원문
  ├─ configs/
  │    └─ domain_randomization.yaml
  ├─ src/
  │    └─ envs/
  │          └─ base_env.py        # Base + Dummy 환경
  ├─ training/
  │    └─ train_ppo.py             # PPO 학습 예시 (Dummy)
  ├─ policies/                     # 학습된 정책 저장
  ├─ mcp_servers/
  │    └─ isaac_controller_server.py  # Isaac 제어 MCP 스켈레톤
  ├─ sim/                          # Isaac 관련 스테이지 / USD 등 (추가 예정)
  ├─ real/                         # 실물 로봇 드라이버/ROS2 브릿지 (추가 예정)
  ├─ scripts/                      # 런처/툴 스크립트
  │    ├─ run_isaac_tool.sh        # Isaac 번들 Python 래퍼 (이중 환경 경계)
  │    ├─ ipc_policy_gateway.py    # 관측→액션 IPC TCP 게이트웨이 (정책 로딩 + SLA 모니터)
  │    ├─ run_tests_local.sh       # Isaac PYTHONPATH 오염 없이 테스트 실행
  └─ tests/                        # 기본 테스트
```

## Dual Environment (이중 Python 런타임) 요약
Isaac Sim 번들 Python(3.11)과 RL/분석 venv(예: 3.12)를 분리하여 버전 충돌(SRE mismatch 등)을 방지합니다.

핵심 포인트:
- RL 측: `.venv` / 시스템 Python → 학습, 분석, 정책 로딩
- Isaac 측: `run_isaac_tool.sh` → Isaac 번들 모듈 접근 전용
- 경계(IPC): `ipc_policy_gateway.py` (TCP line-delimited JSON)
- 데이터 계약: `configs/schemas/obs_action_schema.json` (schema_version 유지)

자세한 의사결정 배경: `docs/ARCH_DECISION_DUAL_ENV.md`
프로토콜 상세: `docs/IPC_BRIDGE.md`

## 설치
Python 3.12 환경 권장:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

Stable-Baselines3 설치 후 GPU 사용을 위해서는 PyTorch CUDA 빌드가 필요합니다. (PyPI torch 패키지는 GPU 지원 자동 매칭.)

Isaac Sim 5.0.0은 별도 ZIP 설치 대신 pip 메타패키지로 통합 배포됩니다. 권장 절차:

```bash
python3.11 -m venv ~/isaacsim-venv
source ~/isaacsim-venv/bin/activate
pip install -U pip
pip install isaacsim[all,extscache]==5.0.0 --extra-index-url https://pypi.nvidia.com
```

이후 Isaac Sim이 필요한 터미널에서는 가상환경을 활성화한 뒤 아래 스크립트를 호출하면 USD/pxr 경로와 라이브러리가 자동 등록됩니다.

```bash
source scripts/activate_isaacsim_env.sh            # 기본 위치 ~/isaacsim-venv 사용
# 또는 별도 경로 사용 시
source scripts/activate_isaacsim_env.sh /path/to/custom-venv
```

스크립트는 `OMNI_KIT_ACCEPT_EULA=YES`, `PYTHONPATH`, `LD_LIBRARY_PATH` 등을 적절히 설정하고 `from pxr import Usd` 스모크 테스트 결과를 출력합니다. Isaac 관련 모든 Python 스크립트는 이 환경이 활성화되어 있다는 가정하에 동작합니다.

### 환경 빠른 점검 (필수)
아래 명령으로 Isaac 전용 가상환경과 pxr 모듈 상태를 자동 점검할 수 있습니다.

```bash
# Isaac 전용 venv + USD 경로 감지, pxr 임포트 결과 요약
scripts/isaac_precheck.sh

# 현재 셸이 activate_isaacsim_env.sh 를 통과했는지 확인 (pxr/isaacsim 모듈 검사)
python scripts/check_isaac_import.py
```

출력 키 포인트:
- `venv_python`: Isaac 전용 가상환경의 파이썬 버전 (예: 3.11.x)
- `system_python`: 현재 셸의 파이썬 버전 (예: 3.12.x)
- `pxr_import`: `true`가 아니면 `source scripts/activate_isaacsim_env.sh` 실행 여부 확인

## VS Code 환경 구성 (자동화)
프로젝트를 열면 `.vscode/` 하위 설정이 적용됩니다.

- `settings.json`: 인터프리터, PYTHONUNBUFFERED, MCP 서버(isaac) 선언
- `tasks.json`: 의존성 설치 / 테스트 / 학습 / Isaac 임포트 체크 / 환경 진단 / MCP 서버 실행 / RoArm USD 뷰어
- `launch.json`: PPO 학습 디버그, Isaac MCP 서버 디버그
- `extensions.json`: 권장 확장 (Python, Copilot Chat, YAML 등)

작업 예:
```bash
code .  # VS Code로 폴더 오픈 후
# 명령 팔레트 (Ctrl+Shift+P) → Tasks: Run Task → Train PPO (quick)
# 명령 팔레트 → Tasks: Run Task → Env Diagnostic  (환경/버전/Isaac precheck 결과를 logs/YYYY-MM-DD/env_diag_*.txt 로 저장)
# 명령 팔레트 → Tasks: Run Task → View RoArm (USD)  (Isaac 전용 venv 활성화 + scripts/activate_isaacsim_env.sh 실행 후 사용)
```

## MCP 서버 사용
1. Copilot Chat 열기 → "isaac 서버 상태 알려줘" 등 자연어 프롬프트
2. 필요 시 MCP Inspector 확장(설치 가정) → `mcp_servers/isaac_controller_server.py` 직접 연결하여 tool schema 검증
3. `mcp.json` 또는 `settings.json`에 정의된 서버가 STDIO로 구동 (PYTHONUNBUFFERED=1)

현재 포함된 tool (스켈레톤):
- `start_sim(headless: bool)`
- `stop_sim()`
- `list_robots()`
- `capture_observation()`
- `apply_action(joints_delta: List[float])`

향후 추가 예정:
- `load_robot(usd_path)`
- `get_joint_state()`
- `policy_infer(policy_path)`
- `rollout_capture(episodes)`

## 도메인 랜덤라이제이션 활용
`configs/domain_randomization.yaml` 정의를 `DomainRandomizer`가 로드 후 샘플링하여 환경에 `env.randomization` 속성으로 주입.

사용 예:
```python
from src.envs.base_env import DummyRoArmM3Env
env = DummyRoArmM3Env(domain_randomizer="configs/domain_randomization.yaml")
obs, info = env.reset()
print(env.randomization)  # 샘플된 파라미터 확인
```

## 스크립트 요약
| 스크립트 | 목적 |
|----------|------|
| `scripts/check_isaac_import.py` | Isaac 핵심 모듈 임포트 가능 여부 검사 |
| `scripts/run_mcp_isaac.sh` | 환경 변수 확인 후 MCP 서버 실행 |
| `scripts/run_training.sh` | 학습 실행 + 로그 디렉토리 생성 |
| `scripts/run_isaac_tool.sh` | Isaac 전용 venv 활성화 후 Python 스크립트 실행 |
| `scripts/ipc_policy_gateway.py` | 관측→액션 TCP IPC 게이트웨이 stub |
| `scripts/run_tests_local.sh` | 깨끗한 PYTHONPATH로 pytest 실행 |

## .env 예시
`.env.example`를 복사하여 `.env` 생성:
```bash
cp .env.example .env
```
필요 키를 채운 뒤 VS Code 재시작 또는 터미널에서 `export $(cat .env | xargs)`.

## Isaac Sim 연동 가이드 (기초)
1. Isaac Sim 버전 확인 (요구: 5.0.0 대응) → `pip show isaacsim` 또는 `python -c "import isaacsim, pkgutil; print(isaacsim.__version__)"`
2. 환경 준비 흐름:
```bash
source ~/isaacsim-venv/bin/activate
source scripts/activate_isaacsim_env.sh
python scripts/check_isaac_import.py
```
3. MCP 서버에서 Isaac 제어하려면: 로컬에서 Isaac GUI 띄운 뒤 WebSocket/RPC 브릿지 (추가 구현 예정)

## 학습 예시 실행 (Dummy 환경)
```bash
python training/train_ppo.py --total_timesteps 2000 --seed 0
```
결과 모델: `policies/ppo_dummy_roarm.zip`

### Rollout 기록 + Reward 설정 커스터마이즈
```bash
python training/train_ppo.py \
  --total_timesteps 1000 \
  --record-rollouts \
  --reward-config configs/reward_config.yaml \
  --tracking-weight 1.2 --smoothness-weight 0.02 --goal-bonus 0.7
```
생성 구조 예시:
```
rollouts/
  2025-10-08T12-34-56/
    meta.json         # env / reward_terms / git hash
    ep_00001.npz      # obs/actions/rewards/dones + 선택 info 키
```
`configs/reward_config.yaml` 값 + CLI override를 병합하여 RewardComposer 구성.

## 다음 확장 포인트
- [ ] Isaac Sim 실제 RoArm M3 USD/URDF 로드 + 관절 맵핑
- [x] 도메인 랜덤라이제이션 기본 로더 (`DomainRandomizer`) 구현
- [ ] Isaac 물리/머터리얼 적용 로직 추가 (API 연동)

## 추가 문서
- 로드맵: `docs/ROADMAP.md`
- Isaac 사전 점검: `docs/ISAAC_PRECHECK.md`
- ROS2 브릿지 설계 초안: `docs/ROS2_BRIDGE.md`
- Sim2Real 심화 가이드: `docs/SIM2REAL_GUIDE.md`
- 테스트 환경 격리: `docs/TEST_ENV_ISOLATION.md`
- Dual Env 결정: `docs/ARCH_DECISION_DUAL_ENV.md`
- IPC 프로토콜: `docs/IPC_BRIDGE.md`

## IPC 게이트웨이 사용
정책 추론 + 관측/행동 직렬화 + SLA / 관측성 기능을 제공하는 TCP line-delimited JSON 게이트웨이:

```bash
python scripts/ipc_policy_gateway.py
```

다른 터미널에서 핑 + 관측 전송:
```bash
python - <<'PY'
import socket, json
s = socket.create_connection(("127.0.0.1", 45123))
for m in [{"type":"ping"}, {"type":"obs", "data":{"q":[0,1,2]}}]:
  s.sendall(json.dumps(m).encode()+b"\n")
  print(s.recv(4096).decode().strip())
PY
```

핵심 특징:
- SB3 PPO zip / Torch (.pt/.pth) / dummy linear 정책 로딩 (지연 로딩)
- JSON Schema (fastjsonschema fallback jsonschema) 검증
- Correlation ID 기반 이벤트 JSONL 로깅 + 크기 기반 rotation(+gzip)
- Latency 전수 통계 + 최근 슬라이딩 윈도 p50/p90/p95/p99
- Watchdog deadline 초과 시 zero-action 안전 fallback + deadline_miss 플래그
- Randomization hash echo & mismatch 포렌식 로그(`hash_mismatch_events.jsonl`)
- SLA 알림(p95 latency, deadline_miss_rate) 이벤트 + cooldown
- Prometheus textfile export (`gateway_metrics.prom`) for node exporter
- CSV 라운드트립 측정 스크립트 (`measure_round_trip.py --csv`)
 - 전송 레이턴시 벤치마킹 (`bench_transport.py --mode tcp --samples 2000 --csv bench.csv`)
 - Hash mismatch forensic drift 분석 (`analyze_hash_drift.py --file logs/hash_mismatch_events.jsonl --csv drift.csv`)
 - 자동 전송 에스컬레이션 어드바이저 (`--auto-transport-escalate-p95`) 3회 연속 초과 시 advisory 이벤트
 - Action scaling 메타 (`--action-scale` → action 응답에 `action_scale_hint`)
 - 확장 schema: `joint_names`, `action_scale_hint` (옵셔널)

## 스키마 파일
`configs/schemas/obs_action_schema.json` 은 관측/액션 교환의 최소 계약을 정의합니다. 상위 `schema_version` 필드는 wire 호환성 관리에 사용됩니다.

테스트로 기본 구조 확인:
```bash
env -i PATH=$PATH HOME=$HOME TERM=$TERM python3 -m pytest tests/test_schema_file.py -q
```

## 테스트 환경 분리 (요약)
Isaac 환경이 `PYTHONPATH`를 전역 오염 시키면 `SRE module mismatch` 발생 가능 → RL/테스트 실행 시 아래 중 하나 사용:
```bash
./scripts/run_tests_local.sh
# 혹은
env -i PATH=$PATH HOME=$HOME TERM=$TERM python3 -m pytest -q
```
상세 가이드는 추후 별도 문서/섹션 확장 예정.

## Isaac Stage 로더 프로토타입
RoArm M3 USD 로딩 테스트:
```bash
python sim/load_roarm.py --usd /absolute/path/to/roarm.usd --headless 0
```
실패 시: `bash scripts/isaac_precheck.sh` 재확인 후 USD 경로/권한 점검.

### 조인트 자동 추출 (inspect_stage)
USD에서 joint 이름 heuristic 추출 & 매핑 파일 업데이트:
```bash
python sim/inspect_stage.py --usd /absolute/path/to/roarm.usd
python sim/inspect_stage.py --usd /absolute/path/to/roarm.usd --update-joints
```
`--update-joints` 실행 시 `configs/roarm_joints.yaml` 조인트 목록 갱신 (기존 limit 유지 or 기본값 -3.14~3.14).

### Isaac Integration Increment 메모 (I1/I2)
진행 중인 Isaac 연동 단계 기록:
| Increment | 내용 | 상태 |
|-----------|------|------|
| I1 | JointAPI get_state/apply_delta 기본 스켈레톤 + fallback | 완료 |
| I2 | World 기반 ArticulationView attach 시도, list_joints 동적 탐색 | 완료 |
| I3 | (예정) 실제 joint positions/velocities fetch → obs 반영 | 예정 |
| I4 | (예정) apply_delta limit-aware clamp + drive targets 적용 | 예정 |
| I5 | (예정) physics_randomizer 실제 PhysX 파라미터 매핑 | 예정 |

추후 I3~I5 완료 후: RL loop에 실제 Isaac step(s) 삽입 및 RolloutLogger 메타에 articulation/joint hash 기록 예정.

## MCP Joint Tool (스켈레톤)
현재 서버(`isaac_controller_server.py`)에 포함된 Joint 관련 placeholder tool:
- `get_joint_state()` : 최근 저장된 joint 상태 반환 (실제 Isaac 연동 전까지 mock)
- `set_joint_targets(targets: List[float])` : 내부 상태 업데이트 용 mock
- `apply_action(joints_delta: List[float])` : delta 적용 mock

향후 실제 Isaac 연동 시:
1. JointAPI에서 articulation handle로 실제 position/velocity fetch
2. set_joint_targets → drive target 설정 + step 후 실제 측정 반영
3. apply_action → delta 누적 후 clamp → target 반영
- [ ] 실물 로봇 ROS2 Bridge (action/observation topic)
- [ ] MCP: 정책 평가 tool (policy_infer), 데이터 수집 tool (rollout_capture)
### 정책 추론 (policy_infer)
`mcp_servers/isaac_controller_server.py`에 `policy_infer` tool이 추가되었습니다.

입력 파라미터:
| 파라미터 | 타입 | 설명 |
|----------|------|------|
| observation | Optional[List[float]] | 관측 벡터 (생략 시 마지막 joint positions 기반) |
| policy_path | Optional[str] | SB3 저장 정책 zip 경로 (미지정 + 미로드 상태면 오류) |
| deterministic | bool | 결정적 행동 여부 |

출력 예시:
```json
{
  "action": [0.01, -0.02, 0.0, 0.0, 0.0, 0.01],
  "policy_path": "policies/ppo_dummy_roarm.zip",
  "deterministic": true,
  "obs_dim": 18
}
```

정책이 아직 로드되지 않았다면 `policy_path`를 반드시 제공해야 하며, 다른 경로 전달 시 자동 재로딩합니다.

### 콜백 기반 Rollout 저장
`train_ppo.py` 실행 시:
| 플래그 | 설명 |
|--------|------|
| `--record-rollouts` | 수동 루프 기반 RolloutLogger (학습 대신 샘플 수집 중심) |
| `--record-callback-rollouts` | 학습(process_bar 포함) 진행 중 SB3 Callback으로 에피소드 자동 저장 |

두 플래그는 동시에 사용할 수 없으며, `--record-callback-rollouts` 경로는 정책 학습과 로그 수집을 동시에 수행합니다.

meta.json 확장 필드:
| 필드 | 설명 |
|------|------|
| `randomization` | 마지막 도메인 랜덤라이제이션 샘플 |
| `physics_report` | physics_randomizer 적용 상세 (solver iterations 등) |
- [ ] 실험 관리 (Hydra/Weights & Biases 선택 검토)

## 보안/비밀 관리
`.env` 파일 사용: API_KEY, MODEL_ENDPOINT 등. VS Code Settings Sync 제외 권장.

## 로깅 & 관측성 요약
- 이벤트: `logs/ipc_gateway_events.jsonl` (회전 + gzip 선택) / alert 이벤트 포함
- 메트릭: `logs/ipc_metrics.json` (full + recent quantiles, counters, SLA 상태)
- Prometheus: `logs/gateway_metrics.prom` (textfile collector)
- 해시 불일치 포렌식: `logs/hash_mismatch_events.jsonl`
- 라운드트립 벤치: `scripts/measure_round_trip.py`
세부 내용은 `docs/OBSERVABILITY.md` 참고.

## 테스트
간단 smoke test 추가 예정:
```bash
pytest -q
```

## 라이선스
MIT
