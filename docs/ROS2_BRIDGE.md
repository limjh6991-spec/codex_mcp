# RoArm M3 ROS2 Bridge 설계 초안

> Tested-With: Isaac Sim (placeholder) 2024.1.x — 실제 버전을 `scripts/print_isaac_version.py` 실행 결과로 갱신하세요. 환경 변수 `ISAAC_SIM_ROOT` 를 설치 경로(`/home/roarm_m3/isaac_sim` 등)로 export 한 뒤 setup 스크립트를 source 하면 자동 감지 정확도가 높아집니다.
>
> Version Capture Procedure:
> 1. Isaac 환경 활성화: `source /path/to/isaac-sim/setup_python_env.sh`
> 2. 실행: `python scripts/print_isaac_version.py` → 출력된 버전 문자열을 본 문서 상단에 반영
> 3. 변경 시 커밋 메시지 예: `docs: update tested Isaac version to 2024.1.3`

본 문서는 Isaac Sim / 학습 정책 / 실물 로봇 간 제어 및 관측 데이터 흐름을 ROS2 기반으로 연결하기 위한 브릿지 아키텍처 초안이다.

## 1. 목표
- 학습된 정책 (SB3 PPO 등) 의 출력을 실 로봇 구동 명령으로 안전/신뢰성 있게 전달
- 실 로봇 상태(joint state, sensor) → 시뮬레이터/정책 관측으로 재사용
- 안전(E-stop, rate limit) 및 모니터링(telemetry, heartbeat) 내장
- Sim ↔ Real 스위칭을 최소한의 코드 변경으로 수행 (env factory / adapter)

## 2. 데이터 흐름 개요
```
[Policy / Inference Node]
    │   (action_cmd)
    ▼
[ros2_bridge_node] --(filtered_action)--> [/roarm/joint_targets]
    ▲                                         │
    │ (joint_states)                           ▼
[/roarm/joint_states] <--- [hardware_driver_node]
    │
    ├─ /roarm/telemetry ------------------------------>
    ├─ /roarm/events (E-STOP, limit_violation, etc.)
    └─ /roarm/heartbeat
```

## 3. 주요 Topic / Service / Action 정의
| 이름 | 타입 | 방향 | 목적 | QoS (제안) |
|------|------|------|------|------------|
| `/roarm/joint_targets` | `roarm_msgs/msg/JointCommand` | Bridge → HW | 목표 관절 위치/속도/모드 | Reliable, Depth=1 |
| `/roarm/joint_states`  | `sensor_msgs/msg/JointState` | HW → Bridge | 현재 관절 상태 (pos/vel/effort) | Reliable, Depth=10 |
| `/roarm/telemetry`     | `roarm_msgs/msg/Telemetry` | Bridge → Monitor | 주기적 상태(snapshot, randomization meta, reward KPIs) | BestEffort, Depth=10 |
| `/roarm/events`        | `roarm_msgs/msg/Event` | Bridge → Monitor | 이슈/경고/에러/비상정지 이벤트 | Reliable, Latching(TransientLocal) |
| `/roarm/heartbeat`     | `std_msgs/msg/Bool` | Bridge ↔ HW | 생존 신호 (양방향 가능) | BestEffort, Depth=1 |
| `/roarm/emergency_stop`| `std_msgs/msg/Bool` | Monitor → Bridge | E-stop 상태 토글 | Reliable, Depth=1 |
| `/roarm/bridge_status` | `roarm_msgs/msg/BridgeStatus` | Bridge → Monitor | 브릿지 내부 상태 (latency, queue length) | Reliable, Depth=5 |
| `/roarm/policy_action_debug` | `roarm_msgs/msg/ActionDebug` | Bridge → Monitor | 정책 원본 action + clamp 후 action | BestEffort |
| `/roarm/randomization_sample` | `roarm_msgs/msg/DomainRandomSample` | Bridge → Monitor | 현재(또는 최근) 도메인 랜덤라이제이션 샘플 | Reliable, Depth=5 |

추가 Service/Action:
| 이름 | 타입 | 용도 |
|------|------|------|
| `/roarm/reset` | Service (std_srvs/Trigger) | 정책/환경/드라이버 상태 초기화 |
| `/roarm/set_mode` | Service (roarm_msgs/srv/SetMode) | 제어 모드(position/velocity/effort) 전환 |
| `/roarm/reload_policy` | Service (roarm_msgs/srv/ReloadPolicy) | 정책 zip hot reload |

## 4. 메시지 정의 (rosidl 추상)
```text
# roarm_msgs/msg/JointCommand.msg
string[] names
float64[] position  # optional length=N
float64[] velocity  # optional length=N
float64[] effort    # optional length=N
string mode         # "position" | "velocity" | "effort" | "mixed"

# roarm_msgs/msg/Telemetry.msg
builtin_interfaces/Time stamp
string run_id
uint32 step
float64[] last_action
float64[] applied_action
float64 episode_reward
float64 tracking_error
float64 smoothness
float64 goal_distance
bool within_tol
float64 inference_latency_ms

# roarm_msgs/msg/Event.msg
builtin_interfaces/Time stamp
string level   # INFO|WARN|ERROR|CRITICAL
string code    # e.g. LIMIT_VIOLATION, ESTOP_ENGAGED
string message
string json_payload  # opaque JSON string(off-nominal details)

# roarm_msgs/msg/BridgeStatus.msg
builtin_interfaces/Time stamp
bool estop
float64 avg_inference_latency_ms
float64 avg_hw_rtt_ms
uint32 dropped_action_frames
uint32 queue_depth

# roarm_msgs/msg/ActionDebug.msg
builtin_interfaces/Time stamp
float64[] raw_policy_action
float64[] clipped_action
bool rate_limited

# roarm_msgs/msg/DomainRandomSample.msg
builtin_interfaces/Time stamp
string json_sample  # JSON 인코딩된 샘플(dict)
```

## 5. 상태 머신 (Bridge)
| 상태 | 진입 조건 | 주요 동작 | 종료 조건 |
|------|-----------|-----------|-----------|
| INIT | 노드 시작 | 파라미터 로드, publisher/subscriber 생성 | 하드웨어 ping 성공 → CONNECTING |
| CONNECTING | HW heartbeat 수신 대기 | 제한 시간 내 여러 ping 전송 | heartbeat OK → READY; timeout → ERROR |
| READY | 정상 동작 | 정책 action 수신 → 필터/클램프 → publish | estop → ESTOP; HW 오류 → ERROR |
| ESTOP | emergency_stop True | 모든 action 0 publish, 이벤트 로깅 | estop 해제 → READY |
| ERROR | 예외 발생 | 재시도 또는 안전 종료 | 재시도 성공 → CONNECTING; 사용자 종료 |

## 6. 안전 계층
- Clamp 순서: raw_policy_action → NaN/Inf sanitize → per-joint rate limit → global norm scale(optional) → joint_limits clamp
- E-Stop 우선순위: estop=True 시 위 과정 bypass 후 zero vector 송신
- Watchdog: 최근 action publish 시각 초과(예: 200 ms) → ZERO 명령 주기 전송 + 이벤트 발생
- Limit Violation 피드백: HW에서 실제 pos가 target 대비 오차 > tolerance 반복 → Event(LIMIT_VIOLATION)

## 7. 지연(latency) 측정
| 구간 | 방법 |
|------|------|
| policy_infer_latency | MCP server/내부 timestamp 차이 |
| action_dispatch_latency | action publish 시각 - infer 완료 시각 |
| hw_round_trip | joint_targets publish → 첫 joint_states timestamp 차이 |

Telemetry 내 rolling average 유지 (exponential moving average).

## 8. 파라미터 (ROS2 Parameters)
| 이름 | 타입 | 기본 | 설명 |
|------|------|------|------|
| rate_limit_per_joint | double | 0.2 | 정책 action per-step 절대값 제한 |
| action_publish_rate | double | 50.0 | Hz 단위 publish 주기 (policy inference가 더 느리면 skip) |
| estop_latch | bool | true | E-stop 해제 시 명시적 서비스 호출 요구 여부 |
| joint_tolerance | double | 0.01 | 위치 오차 허용치 |
| watchdog_timeout_ms | int | 200 | action 미도착 감시 타임아웃 |
| max_action_norm | double | 1.0 | (옵션) 전체 벡터 norm scaling |

## 9. Inference 연동 패턴
1) 정책 노드 (Python) 가 `/roarm/joint_states` 구독 → 관측 벡터 구성 → 내부 infer (또는 MCP policy_infer tool 호출) → action publish.
2) 또는 브릿지가 policy zip 읽어 내부 inference thread 실행 (단일 노드형). 첫 단계에서는 분리형 권장 (관심사 분리 + 장애 영향 최소화).

## 10. Isaac Sim ↔ Real 동형성 전략
| 요소 | Isaac Sim | Real Bridge | 수렴 전략 |
|------|-----------|-------------|-----------|
| 관측 | joint q/dq + goal | joint_states 기반 동일 | 동일 전처리 함수 재사용 (shared module) |
| 랜덤화 | DomainRandomizer 샘플 | 실환경 적용 X (메타 기록만) | diff 로깅 → 추후 sim policy selection 근거 |
| 안전 | clamp + estop mock | 실제 E-stop + 하드웨어 rate limit | 동일 API 인터페이스 유지 |
| 로깅 | JSONL + rollout npz | rosbag2 + JSON 이벤트 | 공통 변환 스크립트 제공 |

## 11. 통합 단계 로드맵
| 단계 | 목표 | 산출물 |
|------|------|--------|
| R1 | 메시지 패키지 스켈레톤 | roarm_msgs/ + CMakeLists/ package.xml |
| R2 | bridge node 초안 (Sim mock hw) | ros2_bridge_node.py (simulated joint_states publisher) |
| R3 | policy action ingest + clamp + estop | rate limit / estop test, unit tests |
| R4 | telemetry/events/hb 완성 | 모니터 대시보드 준비(external) |
| R5 | 실제 HW driver 연동 | hardware_driver_node + latency metrics |
| R6 | Launch files & docs | launch/roarm_bringup.launch.py |

## 12. 테스트 전략
- 단위: clamp 함수, estop state machine, watchdog 타이머, message conversion
- 통합: ros2 launch (bridge + dummy hw + dummy policy) → action round trip latency 측정
- 회귀: GitHub Actions에서 colcon build + pytest (ament) (옵션, HW 제외)

## 13. 보안/신뢰성
- E-stop 토픽은 latched + 별도 서비스(`/roarm/reset_estop`) 제공 고려
- Policy reload 시 checksum(log) 출력 → 재현성 추적
- rosbag2 split size/time 제한 설정(디스크 고갈 방지)

## 14. 추가 고려 사항
- QoS mismatch (sim vs real) → RMW 구현 차이 테스트 필요
- Timestamp 동기화 (NTP) → latency 정확도 향상
- Multi-robot 확장 시 네임스페이스(`/roarmX/`) parameter화

## 15. 예시 코드 조각 (Clamp 순서)
```python
def process_action(raw, prev, rate_limit, joint_lower, joint_upper, max_norm=None):
    import numpy as np
    a = np.array(raw, dtype=float)
    # NaN/Inf sanitize
    if not np.isfinite(a).all():
        a = np.nan_to_num(a, nan=0.0, posinf=0.0, neginf=0.0)
    # Per-joint rate limit
    a = np.clip(a, -rate_limit, rate_limit)
    # Global norm scaling
    if max_norm is not None:
        n = np.linalg.norm(a)
        if n > max_norm and n > 0:
            a = a * (max_norm / n)
    # Joint hard limits
    a = np.minimum(np.maximum(a, joint_lower), joint_upper)
    return a.tolist()
```

## 16. 차후 문서화 TODO
- 메시지 패키지 상세(CMakeLists, package.xml)
- ros2 launch 구성도
- 정책 hot reload 절차
- bag → rollout 변환 스크립트

---
초안 완료. 피드백 수집 후 R1 단계(메시지 패키지) 구현으로 진행 권장.
