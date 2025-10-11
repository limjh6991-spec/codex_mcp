# Context Handoff — 2025-10-11

목표: 이 문서는 이후 에이전트/모델(예: OpenAI GPT-5-Codex)이 현 작업 맥락을 신속히 복원하고 바로 이어받아 실행하도록 핵심 정보를 요약합니다.

## 환경 개요
- OS: Ubuntu 24.04.3 LTS (Noble), Kernel 6.14.0-33-generic
- Python: 3.11 (Isaac 번들 venv: `~/isaacsim-venv`)
- Isaac Sim: 5.0.0 pip 배포 (로그: Isaac Sim Full Version: 5.0.0-rc.45)
- GPU: NVIDIA GeForce RTX 5090 (CUDA 경고는 있으나 런타임 동작)

## 워크스페이스 구조 요약
- 로봇 자산: `assets/roarm_m3`
  - 메시: `assets/roarm_m3/meshes/roarm_m3/*.stl`
  - URDF/Xacro: `assets/roarm_m3/urdf/roarm_m3.xacro`, `roarm_m3.clean.urdf` (정상 XML), 문제있는 원본 `roarm_m3.urdf`
  - 변환된 USD: `assets/roarm_m3/usd/roarm_m3.usd`, `roarm_m3.clean.usd`
- 스크립트:
  - `scripts/verify_usd_roarm_m3.py` — SimulationApp 부트스트랩 후 USD 요약(아티큘/조인트/링크)
  - `scripts/inspect_usd.py` — pxr 또는 SimulationApp 통해 USD 검사 (defaultPrim/children/Articulation API)
  - `scripts/run_roarm_m3_headless.py` — Headless 로더, World 초기화, JointAPI 부착, 스텝 및 상태 로깅
  - `scripts/view_roarm.py` — GUI 뷰어 + 아티큘레이션 스캔/조인트 나열
- 유틸:
  - `src/utils/isaac_joint_api.py` — 조인트 접근/상태 조회/명령용 도우미 (Isaac 가용 시 동작)

## 최근 이슈 및 해결
- URDF Import 실패: `roarm_m3.urdf`가 깨진 XML 구조 → 파서 실패 → 링크 0 → null prim 에러.
- 조치: `roarm_m3.clean.urdf` 생성(정상화), GUI를 통해 USD 변환 성공(`assets/roarm_m3/usd/roarm_m3.usd`).
- 검증: `scripts/inspect_usd.py` 및 `scripts/verify_usd_roarm_m3.py`로 USD 구조 확인 가능.

## 실행 가이드
사전 준비: Isaac 번들 Python 사용 권장

```bash
source ~/isaacsim-venv/bin/activate
```

1) USD 구조 검사
```bash
python scripts/inspect_usd.py --usd assets/roarm_m3/usd/roarm_m3.usd --prim /World/roarm_m3
```

2) Headless 로봇 로드/스텝
```bash
python scripts/run_roarm_m3_headless.py --usd assets/roarm_m3/usd/roarm_m3.usd --prim /World/roarm_m3 --steps 120 --headless
```
- 주의: `scripts/run_roarm_m3_headless.py`는 `src` 패키지 import가 필요합니다.
  - 레포 루트에서 실행하거나, `PYTHONPATH`에 레포 루트를 추가하세요:
    ```bash
    PYTHONPATH=$(pwd) python scripts/run_roarm_m3_headless.py --usd assets/roarm_m3/usd/roarm_m3.usd --prim /World/roarm_m3 --steps 120 --headless
    ```

3) GUI 뷰어
```bash
python scripts/view_roarm.py --usd assets/roarm_m3/usd/roarm_m3.usd --scan
```

## 메모/주의사항
- 로그에 `Warp CUDA error ... cuDeviceGetUuid` 경고가 보일 수 있으나 현재 구성에서는 진행 가능.
- ROS2 Bridge는 비활성화(환경변수 미설정), 필요 시 `docs/ROS2_BRIDGE.md` 참고.
- Isaac API 네임스페이스가 5.0에서 다소 변경됨. 본 레포 스크립트는 4.x/5.x 양쪽을 가급적 호환하려고 방어적으로 작성.

## 다음 단계 제안
- run_roarm_m3_headless.py로 조인트명/도프 수 확인 후, 간단한 position target 명령 테스트 추가
- joint_spec.json과 실제 USD 조인트 매핑 확인 및 불일치 시 보정
- RL 환경(`src/envs/isaac_roarm_env.py`)에 실제 아티큘 연동 이어붙이기
