# 프로젝트 로드맵 (Sim2Real RoArm M3)

본 문서는 우선순위/리스크/가치 기준으로 정렬된 향후 작업 계획입니다.

## 1. Isaac Sim 실제 연동 기반 확보
목표: RoArm M3 모델을 Isaac Stage에 로드하고 관절 상태/명령 최소 기능 확보.
작업:
- RoArm M3 URDF/USD 자산 준비 (필요시 URDF → USD 변환)
- `sim/load_roarm.py` 프로토타입 (Stage 열기 → 모델 삽입)
- `sim/inspect_stage.py` 로 joint 이름/개수 자동 추출 → 조인트 매핑 갱신
- `configs/roarm_joints.yaml` 조인트 이름/인덱스 매핑
- `IsaacControlServer`에 joint I/O 메서드 확장 (추후 구현)
리스크 & 완화:
- API 버전 차이 → 작은 독립 스크립트에서 import 및 articulation 테스트 후 본 코드 반영.

## 2. 실제 Isaac 환경 클래스 구현
목표: RL 학습 루프에서 Dummy 환경을 실제 시뮬레이터로 교체.
작업:
- `src/envs/isaac_roarm_env.py` (조건부 import, headless 옵션)
- reset: Stage 로딩 / 초기 포즈 / randomization 진입 훅
- step: action → articulation command → 관측/보상 계산
테스트:
- Isaac 사용 가능 시에만 실행되는 skip 테스트 (`pytest -k isaac`)

## 3. Domain Randomization 물리 적용
목표: Sim2Real 전이 촉진.
작업:
- 현재 `DomainRandomizer` 결과를 Isaac Material/PhysX/Link 속성에 반영 레이어
- YAML 확장: enable 플래그, 스케줄 (curriculum)
- 실패 시 경고 로그 (치명적 중단 회피)

## 4. 학습 파이프라인 고도화
목표: 재현성/유연성 향상.
작업:
- 통합 config (OmegaConf/Hydra) → train/env/randomization/curriculum
- 알고리즘 선택 (PPO/SAC) → 단일 진입점 `training/train.py`
- 체크포인트 주기적 저장 + 최근 N개 유지
- Metrics: TensorBoard 혹은 wandb 옵션

## 5. MCP 툴 확장 (실험 자동화)
목표: Copilot Chat에서 실험/평가/데이터 수집 자동화.
툴 후보:
- `rollout_episode(policy_path, steps, record)`
- `policy_infer(policy_path, observation)`
- `list_checkpoints()`
- `capture_dataset(episodes, save_path)`

## 6. Rollout 데이터 & 리플레이 구조
목표: 재현/분석/회귀 테스트.
작업:
- `rollouts/<ts>/episode_###.npz` (obs, act, rew, info)
- 메타 JSON (seed, policy hash, randomization 샘플)
- Git LFS 고려 (대형 파일)

## 7. 실로봇 ROS2 브릿지
목표: Real 환경 학습/평가.
작업:
- `real/ros2_bridge.py` rclpy Node 스켈레톤
- `RealRoArmM3Env` (BaseSim2RealEnv 상속)
- 안전: torque/vel clamp, emergency stop hook

## 8. Calibration & Latency
목표: 전이 정확도 향상.
작업:
- joint zero offset / latency 측정 스크립트
- `configs/calibration.yaml` 저장 및 적용 레이어

## 9. CI / 품질 게이트
목표: 안정성.
작업:
- GitHub Actions (pytest + lint)
- Isaac 관련 테스트 조건부 skip
- 간단 smoke 학습 (수백 step) 실행 안정성 확인

## 10. 문서/운영 가이드 강화
목표: 팀 온보딩 및 유지관리.
작업:
- End-to-End 흐름 다이어그램
- Trouble shooting: NaN reward, 관절 발산, sim crash
- Randomization 튜닝 가이드

---

우선순위 요약:
1) 시뮬레이터 연결 → 2) 실제 Env → 3) Randomization 적용 → 4) 학습 파이프라인 → 5) MCP 자동화 → 6) 데이터 구조 → 7) ROS2 → 8) Calibration → 9) CI → 10) 문서.
