# Sim2Real 심화 가이드

본 문서는 RoArm M3 프로젝트에서 시뮬레이션(Isaac Sim) 정책을 실 로봇으로 이전(Transfer)하기 위한 전략, 튜닝 방법, 관측성 도구 활용, 공통 문제 해결 패턴을 정리한다.

---
## 1. 전이(Transfer) 기본 철학
| 축 | 전략 | 원칙 |
|----|------|------|
| 관측(Observation) | 상태 최소화 + 정규화 | 센서 드리프트/노이즈에 민감한 파생량 최소화 |
| 동역학(Dynamics) | Domain Randomization | 물리 파라미터 범위 커버(마찰, 관성, joint damping 등) |
| 보상(Reward) | 분리형 Term + 가중치 탐색 | 명시적 term 기록 → 후처리로 영향 추적 |
| 안전(Safety) | rate limit + E-stop + 모니터링 | 제어 경계 조건(속도/토크) 선행 적용 |
| 재현성(Reproducibility) | seed + git hash + randomization sample 로그 | meta.json + JSONL 통합 저장 |

---
## 2. Reward 튜닝 전략
### 2.1 Term 구성 재검토 체크리스트
- tracking_term: 목표 궤적/자세 오차 기반 (L2 또는 L1) → scale 1.0 baseline
- smoothness_term: Δaction 또는 Δq 기반 (부호 음수) → 과도한 smoothing은 tracking 저하
- action_penalty_term: 에너지/모터 보호 (scale 작게 시작: 1e-3~1e-2)
- goal_bonus_term: 임계거리 이하 즉시 보상 스파이크 → shaping vs sparse trade-off 균형

### 2.2 탐색 절차 (권장)
1. tracking_term 단독 학습 → 수렴 속도/최종 오차 기록
2. smoothness_term 추가 (가중치 sweep: {0.01, 0.05, 0.1}) → reward sum 대비 변동률
3. action_penalty_term 도입 (0.001~0.01) → 과제 불안정(oscillation) 완화 여부 관찰
4. goal_bonus_term 도입 → 성공률 상승 vs 과도한 탐욕 행동 여부 평가

### 2.3 후처리 분석 활용
`rollout_analysis.py` 출력에서:
- per-term sum: 각 term 비중 (%) = term_sum / total_reward_sum
- success_rate 증가 구간에서 어떤 term 조정이 있었는지 커밋 로그/실험 메타 비교

---
## 3. Domain Randomization 커버리지
### 3.1 샘플 로깅
`DomainRandomizer(log_path=...)` 설정 시 JSONL로 모든 샘플이 기록:
```
logs/domain_rand/samples.jsonl
```

### 3.2 커버리지 리포트 생성
```python
import json
from src.utils.domainrand_coverage import analyze_samples, report_to_markdown
from src.utils.domain_randomizer import DomainRandomizer

cfg = DomainRandomizer.from_yaml('configs/domain_randomization.yaml').config
samples = [json.loads(l) for l in open('logs/domain_rand/samples.jsonl','r',encoding='utf-8')]
rep = analyze_samples(samples, cfg)
print(report_to_markdown(rep))
```

### 3.3 해석 가이드
| 지표 | 의미 | 대응 |
|------|------|------|
| missing_keys | 설정 범위를 가진 항목이 한 번도 샘플되지 않음 | YAML 경로 오타/조건부 분기 점검 |
| out_of_range | 샘플 값이 선언 범위 벗어남 | 샘플링 로직 또는 범위 정의 오류 |
| std ≈ 0 | 거의 변화 없음 | 랜덤 범위 과도하게 좁음 / 고정값 의도 재확인 |

---
## 4. 관측성(Observability) 도구 모음
| 도구 | 파일/명령 | 목적 |
|------|-----------|------|
| Episode 분석 | `scripts/analyze_rollouts.py` | success_rate, reward 통계 요약 |
| Randomization Coverage | `domainrand_coverage.py` | 범위, 표준편차, 이상치 확인 |
| MCP Call Logs | `logs/mcp_calls/*.jsonl` | tool latency / 오류 비율 추적 |
| Physics Report | `meta.json` 내 `physics_report` | 적용/실패/스킵 파라미터 확인 |
| Reward Term Sums | rollout NPZ (`sum_<term>`) | 보상 구성 비중/드리프트 분석 |

---
## 5. 안전 운영(Safety Ops)
### 5.1 제어 명령 파이프라인 (실행 순서)
```
policy_action → sanitize(NaN→0) → per-joint clip(±rate_limit)
  → global norm scale(optional) → joint limit clamp → publish/apply
```

### 5.2 E-stop & Watchdog
- E-stop 발생 시 모든 후속 명령 0
- Watchdog 타임아웃 → 강제 zero command + Event 로그
- 복구 절차: 원인 점검 → E-stop 해제 API/service 호출 → 점진적 작은 action 재개

### 5.3 Limit Violation 대응
- 반복 위반 시: (a) joint limit 재설정 검증 (b) 제어 gains/scale 조정 (c) 정책 exploration 감소

---
## 6. 재현성 (Reproducibility)
| 요소 | 저장 위치 | 설명 |
|------|-----------|------|
| random seed | 학습 스크립트 arg/meta.json | seed 고정 & 기록 |
| domain sample | meta.json + samples.jsonl | 특정 episode 조건 재현 |
| git commit hash | meta.json | 코드 상태 스냅샷 |
| policy checksum | (추후) 모델 로딩 시 로그 | 정책 변조 탐지 |

재현 episode 실행: 동일 정책 + randomization sample 강제 적용 + 초기 상태 seed 고정.

---
## 7. 트러블슈팅 (Troubleshooting)
| 증상 | 가능 원인 | 해결 절차 |
|------|----------|-----------|
| 관절 값 NaN | Isaac API fetch 실패 / 정책 폭주 | 정책 action 로그 확인 → 관측 정규화 다시 검증 |
| 정책 추론 OBS_DIM mismatch | 관측 벡터 차원 변경 | policy_infer expected_dim 로그 확인 → 관측 어댑터 갱신 후 재학습 |
| success_rate 정체 | reward scaling 불균형 | per-term sum 비율 분석 → tracking 강화 or penalty 감소 |
| randomization 영향 미미 | 범위 과도하게 좁음 | coverage rep std / min/max 확인 → YAML 범위 확대 |
| latency 스파이크 | MCP tool 과부하 / I/O 병목 | mcp_calls 로그 latency_ms 상위 outlier 조사 |
| limit_violation 반복 | 잘못된 joint limits / scale 과대 | limits 파일 검증 + action scale 조정 |
| episode 조기 종료 과다 | within_tol 조건 과도 | goal_bonus / tolerance 완화 or reward weight 조정 |
| out_of_range 값 다수 | 샘플링 로직/스케일 오류 | YAML & domain_randomizer flatten key 검토 |

---
## 8. 실 로봇 전환 체크리스트
- [ ] ROS2 Bridge R1~R3 완료 (joint_states, joint_targets round trip)
- [ ] E-stop 하드웨어 라인 + 소프트웨어 토픽 모두 테스트
- [ ] Clamp 경로 시뮬레이션(로그) vs 실측 동일성 확인
- [ ] 랜덤라이제이션 중 실제 적용 불가능 항목 제외(문서화)
- [ ] Episode 분석 기준 baseline (success_rate ≥ X%)
- [ ] 정책 inference latency p95 목표 (예: < 15ms)

---
## 9. 향후 개선 로드맵 (문서 관점)
| 카테고리 | 아이템 |
|----------|--------|
| Metrics | reward drift 그래프 자동 생성 스크립트 |
| Safety | 명령 가속도/jerk 제한 추가 문서화 |
| Coverage | 다변량 상관(예: friction vs tracking_error) 분석 가이드 |
| 실험 관리 | Hydra/Weights&Biases 통합 절차 |
| Debug | rosbag → rollout 변환 툴 사용 예 |

---
## 10. 빠른 참고 TL;DR
1. 항상 per-term reward sum을 먼저 본다 (tracking 비중 <60%면 가중치 재조정 고려)
2. coverage 리포트에서 std≈0 키는 비활성화 또는 범위 확대
3. latency 스파이크는 정책 추론 / I/O 양쪽 로그 교차검증
4. E-stop 테스트는 "예정 없는 상황"에서 주기적으로 수행 (훈련 loop 중간)
5. 재현 실험: seed + sample + git hash + policy zip 네 요소 모두 고정

---
문서 개선 제안은 PR로 환영한다.
