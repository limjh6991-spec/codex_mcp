# Isaac Sim 사전 점검 체크리스트

`배경설명` 문서 3. 작업전 필수 체크 필요 항목을 자동/반자동으로 검증하기 위한 가이드.

## 1. 설치 패키지 / 버전 확인
- 설치 파일: `isaac-sim-standalone-5.0.0-linux-x86_64.zip`
- 실제 설치 디렉토리 존재 여부: $ISAAC_SIM_PATH
- 버전 확인: `$ISAAC_SIM_PATH/version.txt` 또는 GUI Help > About
 - 환경 변수 권장: `export ISAAC_SIM_ROOT=/home/roarm_m3/isaac_sim` (또는 실제 설치 경로)
 - 자동 버전 출력: `python scripts/print_isaac_version.py` (omni import 실패 시 후보 경로/스크립트 안내)

## 2. SDK / Python API 환경
- `$ISAAC_SIM_PATH/setup_python_env.sh` 실행 후 `python scripts/check_isaac_import.py`
- 실패 시: PYTHONPATH, LD_LIBRARY_PATH 누락 여부 점검

## 3. 필수 모듈 임포트
| 모듈 | 목적 |
|------|------|
| `omni.isaac.kit` | Isaac 런처 초기화 |
| `omni.isaac.core` | 핵심 시뮬레이션 API |
| `omni.isaac.core.simulation_context` | SimulationContext 접근 |

## 4. GUI / Headless 모드 구분
- GUI: 시각적 디버깅(articulation pose, collision 확인)
- Headless: 학습 대량 실행 (fps 향상/자원 절약)

## 5. 흔한 문제 & 해결
| 현상 | 원인 | 해결 |
|------|------|------|
| 모듈 임포트 실패 | PATH/LD_LIBRARY_PATH 미설정 | `source setup_python_env.sh` 재실행 |
| 로봇 모델 누락 | USD 위치 오타 | 절대경로 사용 & Stage Console 확인 |
| 성능 저하 | FPS 제한/VSYNC | Headless 또는 PhysX Substep 최적화 |
| 관절 명령 반영 안됨 | Degree/Radian 혼동 | Isaac API 단위(rad) 확인 |

## 6. Precheck 스크립트
`bash scripts/isaac_precheck.sh` 실행 → 환경 변수 / 디렉토리 / 버전 / 모듈 임포트 순차 점검.

추가 확인 순서 (수동):
```bash
export ISAAC_SIM_ROOT=/home/roarm_m3/isaac_sim
source $ISAAC_SIM_ROOT/setup_python_env.sh
python scripts/print_isaac_version.py  # JSON 출력 확인
python scripts/check_isaac_import.py   # 핵심 모듈 임포트 결과
```

## 7. 수동 점검 (AI 인지가 어려운 GUI 항목)
- 메뉴 위치 차이: Help > About (버전), Window > Extensions (플러그인 충돌 여부)
- Stage: 로봇 USD 로드 후 Hierarchy 패널에서 link/joint 구조 시각 확인
- Articulation Inspector: 각 joint limit / drive 속성 값 검사

## 8. 다음 단계
Precheck 통과 후 `src/envs/isaac_roarm_env.py` 구현 및 MCP 서버 확장 진행.
