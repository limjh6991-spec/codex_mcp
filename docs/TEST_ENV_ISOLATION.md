# 테스트 환경 격리 (Test Environment Isolation)

Isaac Sim 번들 Python과 RL/일반 테스트 Python이 뒤섞이면 `SRE module mismatch` 등 표준 라이브러리 충돌이 발생할 수 있습니다. 이 문서는 해당 문제의 원인, 재현, 해결 패턴을 정리합니다.

## 1. 증상
- `AssertionError: SRE module mismatch` (re/_compiler.py)
- 표준 라이브러리 모듈이 Isaac 경로(`/isaac_sim/kit/python/lib/python3.11/...`)에서 먼저 로드됨
- pytest 실행 즉시 실패

## 2. 근본 원인
Isaac Sim 환경 스크립트(`setup_python_env.sh`, `python.sh`)가 광범위한 `PYTHONPATH`를 export 하여 다른 버전(예: 시스템 3.12)과 혼합 로딩.

## 3. 재현 방법
```bash
# Isaac 환경 로드 후
source /path/to/isaac-sim/setup_python_env.sh
pytest -q
# → SRE mismatch 발생 가능
```

## 4. 해결 전략 요약
| 전략 | 장점 | 단점 |
|------|------|------|
| 완전 클린 서브쉘(env -i) | 충돌 100% 차단 | PATH 등 재구성 필요 |
| 선택적 unset (권장) | 간단, 유지보수 용이 | 누락 시 재발 가능 |
| 컨테이너 분리 | 강력한 격리 | 관리/빌드 오버헤드 |

## 5. 권장 워크플로우
1. RL/분석/테스트 전용 venv 생성:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -U pip
   pip install -e .
   ```
2. 테스트 시 Isaac 환경 로딩 금지 (동일 터미널에서 `setup_python_env.sh` 실행하지 않기)
3. 필요 시 Isaac 쪽 작업은 별도 터미널 (프롬프트에 venv 표식 구분 권장)
4. 자동화 스크립트 사용: `./scripts/run_tests_local.sh`

## 6. 클린 실행 예시
```bash
# 단일 파일 테스트(스키마)
env -i PATH=$PATH HOME=$HOME TERM=$TERM python3 -m pytest tests/test_schema_file.py -q

# 전체 테스트 (필요 시)
env -i PATH=$PATH HOME=$HOME TERM=$TERM python3 -m pytest -q
```

## 7. 게이트웨이 실행 (정상 분리)
```bash
./scripts/run_ipc_gateway_clean.sh --port 45220
# 다른 터미널에서
python - <<'PY'
import socket, json
s=socket.create_connection(("127.0.0.1",45220))
s.sendall(b'{"type":"ping"}\n')
print(s.recv(4096).decode())
PY
```

## 8. CI/자동화 제안
- GitHub Actions / GitLab CI에서 Isaac 관련 경로 미설정 런너 사용
- 테스트 job: `pip install fastjsonschema jsonschema` 후 pytest
- (선택) `make test` 타겟으로 내부에서 클린 환경 호출

## 9. 빠른 체크리스트
| 체크 | OK 기준 |
|------|---------|
| `echo $PYTHONPATH` | Isaac 경로 없음 |
| `pytest -q` | SRE mismatch 없음 |
| `which python` | `.venv/bin/python` 가리킴 |
| 게이트웨이 실행후 ping | ack 응답 |

## 10. 문제 재발 시 점검
1. 현재 쉘에서 `type -a python` → Isaac 경로 우선 등장 여부
2. `env | grep -i isaac` → 변수가 남아 있으면 새 쉘/로그아웃
3. `.bashrc` / `.zshrc` 에 Isaac 환경 자동 로드 설정 제거
4. 여전히 혼합 → `pip list | grep isaac` (불필요 패키지 제거 여부)

## 11. 향후 개선(Roadmap)
- Makefile: `make test` → 내부에서 clean wrapper 적용
- pre-commit 훅: Isaac 경로 검출 시 경고
- `scripts/diagnose_env.py` 추가 (PYTHONPATH, interpreter, core libs 버전 diff 출력)

## 12. 참고
- Dual Environment 결정: `docs/ARCH_DECISION_DUAL_ENV.md`
- IPC 프로토콜: `docs/IPC_BRIDGE.md`
- Sim2Real 가이드: `docs/SIM2REAL_GUIDE.md`

---
문서 개선 제안이나 자동 점검 스크립트 추가가 필요하면 issue로 남기세요.
