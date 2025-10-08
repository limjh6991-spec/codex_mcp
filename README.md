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
  └─ tests/                        # 기본 테스트
```

## 설치
Python 3.12 환경 권장:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

Stable-Baselines3 설치 후 GPU 사용을 위해서는 PyTorch CUDA 빌드가 필요합니다. (PyPI torch 패키지는 GPU 지원 자동 매칭.)

Isaac Sim Python API 연동은 NVIDIA Isaac Sim 설치 디렉토리의 `python.sh` 또는 `setup_python_env.sh` 스크립트를 이용해 PYTHONPATH를 주입해야 합니다.

## VS Code 환경 구성 (자동화)
프로젝트를 열면 `.vscode/` 하위 설정이 적용됩니다.

- `settings.json`: 인터프리터, PYTHONUNBUFFERED, MCP 서버(isaac) 선언
- `tasks.json`: 의존성 설치 / 테스트 / 학습 / Isaac 임포트 체크 / MCP 서버 실행
- `launch.json`: PPO 학습 디버그, Isaac MCP 서버 디버그
- `extensions.json`: 권장 확장 (Python, Copilot Chat, YAML 등)

작업 예:
```bash
code .  # VS Code로 폴더 오픈 후
# 명령 팔레트 (Ctrl+Shift+P) → Tasks: Run Task → Train PPO (quick)
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

## .env 예시
`.env.example`를 복사하여 `.env` 생성:
```bash
cp .env.example .env
```
필요 키를 채운 뒤 VS Code 재시작 또는 터미널에서 `export $(cat .env | xargs)`.

## Isaac Sim 연동 가이드 (기초)
1. Isaac Sim 버전 확인 (요구: 5.0.0 대응) → GUI 실행 후 Help > About 또는 설치 디렉토리 Release Notes.
2. Python API 경로 설정 (예시):
```bash
source /path/to/isaac-sim/setup_python_env.sh
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
- [ ] 실험 관리 (Hydra/Weights & Biases 선택 검토)

## 보안/비밀 관리
`.env` 파일 사용: API_KEY, MODEL_ENDPOINT 등. VS Code Settings Sync 제외 권장.

## 로깅 & 관측성
- 학습: SB3 logger
- MCP: 추후 request/response JSON 로깅 + latency 측정
- Rollout: `--record-rollouts` 사용 시 episode 단위 NPZ + meta.json 저장
- Randomization: 마지막 샘플 캐시(`DomainRandomizer.last_sample()`) → 추후 meta 확장 예정

## 테스트
간단 smoke test 추가 예정:
```bash
pytest -q
```

## 라이선스
MIT
