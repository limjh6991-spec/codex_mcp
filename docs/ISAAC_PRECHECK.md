# Isaac Sim 사전 점검 체크리스트

`배경설명` 문서 3. 작업전 필수 체크 필요 항목을 자동/반자동으로 검증하기 위한 가이드.

## 1. 설치 패키지 / 버전 확인
- 권장 배포: pip 메타패키지 `isaacsim[all,extscache]==5.0.0`
- 기본 경로: `~/isaacsim-venv` (환경 변수 `ISAACSIM_VENV`로 변경 가능)
- 설치 예시:
	```bash
	python3.11 -m venv ~/isaacsim-venv
	source ~/isaacsim-venv/bin/activate
	pip install -U pip
	pip install isaacsim[all,extscache]==5.0.0 --extra-index-url https://pypi.nvidia.com
	```
- 버전 확인:
	```bash
	source ~/isaacsim-venv/bin/activate
	source scripts/activate_isaacsim_env.sh
	python - <<'PY'
	import isaacsim
	print(getattr(isaacsim, "__version__", "unknown"))
	PY
	```
- 레거시 ZIP 설치를 유지 중이라면 `ISAAC_SIM_ROOT`와 `python.sh`를 직접 지정해야 합니다.

## 2. SDK / Python API 환경
- 세션마다 아래 두 줄로 환경을 로드합니다.
	```bash
	source ~/isaacsim-venv/bin/activate
	source scripts/activate_isaacsim_env.sh  # PX, USD, EULA 설정까지 자동
	```
- 조용한 모드가 필요하면 `export ACTIVATE_ISAACSIM_QUIET=1` 후 실행하세요.
- 래퍼 스크립트: `scripts/run_isaac_tool.sh <python_file> [args...]`

## 3. 필수 모듈 임포트
| 모듈 | 목적 |
|------|------|
| `isaacsim.simulation_app` | 최신 SimulationApp 엔트리포인트 |
| `omni.isaac.kit` | 레거시 Kit 초기화 경로 |
| `omni.isaac.core` | 핵심 시뮬레이션 API |

## 4. GUI / Headless 모드 구분
- GUI: 시각적 디버깅(articulation pose, collision 확인)
- Headless: 학습 대량 실행 (FPS 향상/자원 절약)
- 예시: `scripts/run_isaac_tool.sh scripts/view_roarm.py --usd ... --headless`

## 5. 흔한 문제 & 해결
| 현상 | 원인 | 해결 |
|------|------|------|
| `ModuleNotFoundError: pxr` | activate 스크립트 미실행 또는 LD_LIBRARY_PATH 누락 | `source scripts/activate_isaacsim_env.sh` 재실행 |
| `ImportError: libusd_tf.so` | 시스템 libpython3.11 미설치 | `sudo apt install libpython3.11`
| 로봇 모델 누락 | USD 경로 오타 | 절대경로 사용 & Stage Console 확인 |
| 성능 저하 | FPS 제한/VSYNC | Headless 모드 또는 PhysX Substep 조정 |
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
