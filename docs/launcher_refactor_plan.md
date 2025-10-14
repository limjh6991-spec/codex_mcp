# RoArm Launcher 리팩터링 계획

## 개요
RoArm Isaac Sim 런처는 점진적인 구조 개편을 통해 유지보수성과 테스트 용이성을 향상시키려 합니다. 아래 계획은 3개의 PR로 나누어 수행하며, 각 단계에서 명확한 검증 포인트를 제공하도록 구성했습니다.

## PR-1: 엔트리 가벼워지기
- `scripts/open_roarm_m3_gui.py` → `roarm_app.cli` 위임 (호출 시나리오 유지).
- 런처 핵심 로직 `roarm_app.runtime.run_loop` 로 이관.
- 환경 변수 설정은 `roarm_app.launch.envvars` 모듈에서 담당.
- 결과: 엔트리 스크립트는 import + main 호출만 담당하고, 새 모듈들은 기존 실행 경로를 유지합니다.

### 검증 포인트
- `python -m compileall roarm_app/cli.py roarm_app/runtime/run_loop.py roarm_app/launch/overlay.py`
- 기존 CLI 인자(usd/prim/mode 등)로 실행 시도 → 기존과 동일한 로그/동작 확인.

## PR-2: overlay·extensions 격리 + 스위치 정리
- `roarm_app.launch.overlay` · `roarm_app.launch.extensions` (신규)로 관심사를 분리.
- `cli.py` 에 `--no-quiet-shutdown`, `--no-blocklist`, `--warmup-steps`, `--max-steps` 등 스위치 정식 반영 및 도움말 정리.
- `runtime.exit_codes` (enum/상수) 도입으로 종료 요약 문자열을 일관성 있게 관리.
- overlay/extension 적용 여부를 구조화된 로그 이벤트로 남기고, 필요 시 호출 측에서 로깅 hook 연결 가능.

### 검증 포인트
- headless / GUI 모드 전환 시 overlay·blocklist 이벤트 로그 확인.
- 종료 경로별(quiet/force/step 제한) `EXIT_REASON` 출력이 enum 기반 문자열과 일치하는지 검사.

## PR-3: verify 공통화
- `roarm_app.verify.usd_checks`, `roarm_app.verify.report` 구현.
- `scripts/verify_usd_roarm_m3.py` 및 런처에서 공통 함수 호출.
- CI 파이프라인에서 `--fast`/`--full` 스위치로 USD-only vs Isaac API 경로 선택.

### 검증 포인트
- 단위 테스트: `tests/test_usd_checks.py` 에 샘플 USD 입력으로 스케일/리미트/드라이브 검증.
- 통합 테스트: CLI `--verify` 시나리오(향후 추가 예정)에서 fast/full 분기 확인.

## 테스트 전략 요약
- `tests/test_envvars.py`: quiet/EGL/GLX 분기 동작 검증.
- `tests/test_overlay.py`: overlay 준비 실패/성공 시 note 및 로그 이벤트 확인.
- `tests/test_usd_checks.py`: USD 정합성 체크 로직 단위테스트.

## 향후 과제
- `runtime.exit_codes` 기반으로 런타임 서머리 템플릿 정리.
- overlay/logger hook을 활용해 구조화된 JSON 로그(예: Loki/Elastic)에 연결.
- CLI → Python API(함수 호출) 경로 제공으로 VS Code Tasks 등에서 재사용성 확보.
