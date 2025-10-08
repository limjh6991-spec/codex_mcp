# Architecture Decision: Dual Environment (Isaac 3.11 / RL 3.12)

## 1. Context
SRE module mismatch 및 Isaac 번들 Python(3.11)과 시스템/프로젝트 Python(3.12) 충돌로 인해 단일 통합 환경 접근이 반복적으로 실패. RL 학습/분석 측은 최신 3.12 기능 및 패키지를 활용 중이며 Isaac 측은 번들 환경 의존도가 높아 업그레이드 변경이 위험.

## 2. Decision Summary
"혼합"을 제거하고 "분리" 전략을 표준으로 채택한다.
- Isaac 관련 실행: `/home/roarm_m3/isaac_sim/python.sh` (번들 3.11)
- RL / 정책 학습 / 분석: 시스템(or venv) Python 3.12
- 경계: 명시적 직렬화(IPC) 또는 MCP tool wrapper
- 통합/공유메모리 방식은 성능 요구(고주파수 제어, <20ms 왕복) 발생 시 재평가

## 3. Options Considered (Decision Matrix)
| 옵션 | 설명 | 장점 | 단점 | 채택 여부 |
|------|------|------|------|-----------|
| A | Dual Environment 분리 | 충돌 최소, 최신 기능 유지 | IPC 추가 계층 | ✅ 채택 |
| B | 전체 3.11 다운그레이드 | 단일 환경 | 최신 3.12 기능 손실 | ❌ |
| C | Isaac 번들 Python 승격 | 완전 일관성 | 패키지 제약, 업그레이드 어려움 | ❌ |
| D | 컨테이너 캡슐화 | 재현성 | 초기 세팅 복잡 | △ (후보) |
| E | gRPC/IPC 완전 서비스화 | 확장성, 격리 | 초기 오버헤드 | △ (중장기) |
| F | pyenv 다중 전환 | 전환 간단 | 사용 실수 여지 | △ (보조) |

## 4. Risk & Mitigation
| 리스크 | 영향 | 완화 |
|--------|------|------|
| IPC latency 증가 | 제어 주기 저하 | 지연 모니터링, ZeroMQ/SHM 단계적 전환 |
| 관측/행동 동기화 실패 | 학습 품질 저하 | schema version + timestamp 이중 기록 |
| Randomization 샘플 불일치 | 재현성 붕괴 | hash echo RPC + meta.json 기록 |
| Adapter silent failure | 빈 상태 학습 | 연속 empty-joint 경고/테스트 |
| 환경 전환 실수 | 재발 오류 | wrapper script + README Quick Start |
| 로깅 혼선 | 분석 혼동 | source-of-truth 규칙 문서 |
| Deadline miss 빈발 | 실시간 제어 불안정 | watchdog latency 모니터링 + 모델 최적화/전송 개선 |
| Randomization hash mismatch 누적 | 재현성/분석 신뢰 하락 | hash echo + mismatch 알림, drift 원인 추적 |

## 5. Chosen Architecture (Execution Layers)
```
[ RL/Policy (Py3.12) ]  <--JSON/IPC-->  [ Isaac Sim Runtime (Py3.11) ]
 |  SB3, analysis                        | SimulationApp, physics, articulation
 |  Rollout logger / metrics             | Joint state, domain randomization apply
```

## 6. Interface Contract (Initial)
| 항목 | 필드 | 비고 |
|------|------|------|
| Observation msg | {"obs_version":1, "sim_time": float, "q": [...], "dq": [...]} | 확장 가능 |
| Action msg | {"action_version":1, "ts_policy": float, "delta": [...] } | rate limit Isaac 측 재검증 |
| Telemetry | {"latency_ms": float, "hash_rand": str} | 누적 EMA 관리 |

## 7. Transition Plan
1. Wrapper 스크립트 `run_isaac_tool.sh` 추가 (완료)
2. Observation/Action schema JSON (`configs/schemas/obs_action_schema.json`) (완료)
3. TCP JSON IPC gateway (`scripts/ipc_policy_gateway.py`) 초기 버전 (완료)
4. README 링크 업데이트 + Quick Start (완료)
5. Schema validation test (`tests/test_schema_file.py`) (완료)
6. Correlation ID + JSONL 이벤트 로깅 (완료)
7. Latency metrics 수집(p50/p90/p95/p99) + 주기적 flush (완료)
8. Watchdog deadline (`--deadline-ms`) + 안전 zero action fallback (완료)
9. Randomization hash echo + mismatch flag (완료)
10. 통합 테스트 (`tests/test_ipc_gateway_integration.py`) ping + 정책 로드 + rand_hash echo (완료)
11. Observability 문서 (`docs/OBSERVABILITY.md`) (완료)
12. deadline_miss rate 메트릭 (완료)
13. 이벤트 로그 size 기반 rotation (--max-log-mb) (완료)
14. rotated 로그 gzip 압축 (--compress-rotated) 옵션 (완료)
15. 최근 구간(슬라이딩 윈도) p95 추가 지표 (완료)
16. (조건부) p95 > 60ms 지속 시 ZeroMQ/SHM 연구 착수 (대기)

> 관측 관련 상세 필드는 `docs/OBSERVABILITY.md` 문서 참조.

## 8. Decision Drivers
- 유지보수성: Isaac 업스트림 변경 영향 최소화
- 재현성: randomization + seed + git hash 조합 보전
- 확장성: 실로봇, ROS2 Bridge, 원격 inference 가능성 대비

## 9. Trade-offs
- 초기 IPC 오버헤드 존재 (serialization + context switch)
- 디버깅 경계 증가 (이중 로그 correlation 필요) → correlation id 도입으로 완화

## 10. Acceptance Criteria
- Isaac 관련 스크립트 실행 시 SRE mismatch 0회
- RL 학습 루프에서 omni import 시도 없음 (guard test 통과)
- obs/action schema 파일 존재 & JSON parse OK
- gateway stub 통해 round-trip echo 테스트 가능
- Correlation ID 기반 recv/send 이벤트 로그(JSONL) 생성
- 정책 로딩 후 obs→action 응답에 latency 및 profiling 필드 포함
- watchdog deadline 적용 시 초과 샘플에서 deadline_miss=true 플래그 확인
- rand_hash 포함 obs 전송 시 action에서 동일 hash echo 및 mismatch flag 동작
- metrics 파일(`logs/ipc_metrics.json`)에 p95 latency 값 집계

## 11. Revisit Conditions
| 조건 | 조치 |
|------|------|
| p95 round-trip > 60ms 지속 | Shared memory / 단일 프로세스 병합 검토 |
| 빈 joint state 경고 반복 | Isaac 직접 통합 재평가 |
| 고주파수 제어(>200Hz) 요구 | On-process 정책 추론 또는 C++/TensorRT 이식 |

## 12. Status
Accepted (2025-10-08).

| 13. Enhancement Status (Post-Adoption)|
| 항목 | 설명 | 상태 | 비고 |
|------|------|------|------|
| Severity-tiered alerts | p95 / miss-rate WARN/CRIT 단계 | 완료 | 이벤트+Prometheus severity 지표 |
| Prometheus labels | instance_id, policy_kind 라벨 | 완료 | 라벨 비활성화 옵션 지원 |
| Hash mismatch aggregate | rand_hash_mismatch 누적 counter | 완료 | forensic JSONL + drift 분석 스크립트 |
| Transport benchmark script | TCP 지연 측정 스크립트 | 완료 | `scripts/bench_transport.py` |
| Drift analysis tooling | mismatch forensic 로그 패턴 요약 | 완료 | `scripts/analyze_hash_drift.py` |
| Auto transport escalate | 임계 초과 sustained 시 메시지 출력/권고 | 완료 | `--auto-transport-escalate-p95` advisory 이벤트 구현 |

---
문서 개선 또는 상태 변경 시 업데이트 후 git commit 메시지: `docs: update dual env decision`.
