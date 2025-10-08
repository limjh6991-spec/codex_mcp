# Step1 확장 체크리스트 (시뮬레이터 연결 심화)

## A. Stage / 모델
- [ ] USD 경로 검증 (절대경로)
- [ ] Articulation prim 경로 식별 (/World/Root/...) 기록
- [ ] Joint 개수 / 이름 → configs/roarm_joints.yaml 반영
- [ ] `sim/inspect_stage.py` 사용으로 joint 자동 추출 검증

## B. 관절 I/O
- [ ] get_joint_state() 구현 (position, velocity)
- [ ] set_joint_targets() 구현 (drive target 반영)
- [ ] Action scaling (delta vs absolute) 결정

## C. 초기화 & Reset
- [ ] 초기 자세(q0) 설정 YAML (configs/roarm_init.yaml)
- [ ] Reset 시 randomization 적용 순서 (Stage load → articulation → DR)

## D. Randomization 적용
- [ ] 마찰계수(material) 적용
- [ ] 질량/관성 스케일 테스트
- [ ] 조명/텍스처 랜덤화 (선택)

## E. 성능 / 안정성
- [ ] Headless FPS 측정 (기준 기록)
- [ ] Physics substep / solver iteration 튜닝
- [ ] 메모리 누수 여부 (장시간 step)

## F. MCP Tool 확장
- [ ] tool:get_joint_state
- [ ] tool:set_joint_targets
- [ ] tool:rollout_episode (Dummy → Isaac)

## G. 로깅
- [ ] per-step latency 측정 (env.step wrapper)
- [ ] Episode summary (reward mean, length)

## H. 안전
- [ ] Joint limit clamp
- [ ] Velocity/acceleration soft limit
- [ ] Emergency stop flag (MCP tool)
