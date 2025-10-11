# Isaac Sim 변환/검증 가이드

이 문서는 `RoArm M3` 모델을 Isaac Sim으로 변환하고 검증할 때 참고할 절차와 운영 팁을 정리한 것입니다.

## 1. 실행 스크립트 & 기본 플로우

1. 레포지토리 루트에서 Isaac Sim 가상환경을 활성화합니다.
   ```bash
   source scripts/activate_isaacsim_env.sh
   ```
2. GUI가 필요한 경우 새 터미널에서 다음 스크립트를 실행합니다.
   ```bash
   bash scripts/launch_isaacsim_gui.sh [추가 kit 인자]
   ```
   - 내부에서 자동으로 가상환경을 다시 로드하고 `$HOME/isaacsim-venv/bin/isaacsim` 실행 파일을 호출합니다.
   - KIT 인자를 그대로 전달하므로, 필요 시 `--/renderer/multiGpu/enabled=False` 등 커스텀 옵션을 추가할 수 있습니다.
3. XACRO → USD 파이프라인은 다음 명령으로 수행합니다.
   ```bash
   python scripts/xacro_to_usd.py --clean-stray-world \
     --usd-out assets/roarm_m3/usd/roarm_m3.generated.usd
   ```
   - `--xacro` 미지정 시 `assets/roarm_m3/urdf/roarm_m3.xacro`를 사용합니다.
   - XACRO 의존성이 없으면 `assets/roarm_m3/urdf/roarm_m3.clean.urdf`를 복사해 사용합니다.

## 2. 산출물 경로 및 파일 관리 전략

- **Generated USD/URDF 분리**  
  - 원본 수동 편집 파일: `assets/roarm_m3/urdf/roarm_m3.clean.urdf`  
  - 변환 결과물: `assets/roarm_m3/usd/roarm_m3.generated.usd`
  - 반복 변환 시 덮어쓰기가 발생하므로, 특정 버전을 보존하려면 `assets/roarm_m3/usd/snapshots/` 등의 별도 폴더를 두고 날짜/커밋 태그를 붙여 관리합니다.

- **Git 추적 범위 정의**  
  - 운영 중 생성되는 `.usd`/`.urdf` 파일이 많다면 `.gitignore`에 다음 패턴을 추가해 산출물만 제외하는 것을 권장합니다.
    ```gitignore
    # Isaac Sim generated artifacts
    assets/roarm_m3/usd/*.usd
    assets/roarm_m3/urdf/roarm_m3.generated.*
    ```
  - 장기 보존이 필요한 버전은 `git add -f assets/roarm_m3/usd/roarm_m3.generated.usd`처럼 강제로 추적하거나, `git lfs`를 사용해 대용량 파일을 관리합니다.

- **검증 리포트 유지**  
  - `scripts/xacro_to_usd.py` 실행 시 `/tmp/xacro_to_usd.trace`가 갱신됩니다. 중요한 세션은 별도 디렉터리에 백업하고 커밋 메시지에 링크를 남기면, 동일 설정 재현이 쉬워집니다.

## 3. USD 구조 검증 팁

Isaac Sim 또는 `usdview`에서 아래 항목을 우선 확인합니다.

- 기본 Prim이 `DefaultPrim = /roarm_m3`로 설정되어 있는지.
- `world` 서브트리가 필요 이상으로 남아 있지 않은지 (`--clean-stray-world` 옵션으로 삭제 가능).
- `PhysicsArticulationRoot`가 의도한 링크에 적용되었는지 및 조인트 개수.
- 시뮬레이션 시작 전 `Physics -> Validate` 패널에서 관성/질량 경고가 없는지.

## 4. 경고 로그 대응 체크리스트

| 경고 메시지 | 원인 | 대응 |
|-------------|------|------|
| `link <name> has no body properties` | URDF에 질량/관성/충돌 정보가 누락되어 고정 조인트 병합 시 링크가 제거됨 | 해당 링크에 `<inertial>` 및 `<collision>` 블록을 추가 (예: 이번 커밋에서 `hand_tcp`에 구체형 충돌체 추가) |
| `Creating Asset in an in-memory stage` | Importer가 레이어 파일 없이 메모리 스테이지를 사용 | Scene을 저장하기 전 `File -> Save` 또는 `stage.GetRootLayer().Export()` 수행 |
| `Skipping unsupported non-NVIDIA GPU` | 다중 GPU 환경에서 NVIDIA 외 장치가 감지됨 | `--/renderer/multiGpu/enabled=False` 옵션으로 비NVIDIA 장치 무시, 필요 시 iGPU 사용 해제 |
| `deprecated extension ...` | Isaac Sim 5.0에서 구 버전 확장 로드 | `exts/` 설정에서 해당 확장을 비활성화하거나 최신 모듈로 대체 (예: `omni.isaac.core` → `isaacsim.core.api`) |
| `Warp CUDA error: ... get driver entry point` | CUDA 드라이버와 Warp 모듈 버전 차이 | NVIDIA 드라이버가 Isaac Sim 요구 버전을 충족하는지 확인 (`nvidia-smi`), 문제 지속 시 Warp를 CPU 모드로 제한 |

> ⚠️ 로그를 수집해 자동 점검하려면 `scripts/verify_usd_roarm_m3.py` 같은 단위 검증 스크립트에서 `carb.settings`를 활용해 특정 경고를 허용/차단할 수 있습니다.

## 5. 물리 속성 보강 히스토리

- 2025-10-11: `hand_tcp` 링크에 질량(0.5g), 관성, 구형 충돌체를 추가하여 병합 경고를 해소했습니다. 필요 시 실제 툴 좌표의 질량/크기를 계측하여 값을 보정하세요.

## 6. 향후 자동화 제안

1. `scripts/verify_usd_roarm_m3.py`에 USD 구조/관성 검사를 추가하고 CI에서 실행.
2. `scripts/launch_isaacsim_gui.sh`를 사용해 GUI 버튼 하나로 실행하도록 `.desktop` 또는 VS Code task 작성.
3. Stage 저장물(exported USD)을 S3/Nucleus 등 외부 스토리지에 백업하고, 메타데이터(JSON)로 생성 시점과 커밋 해시를 기록.

필요 시 이 문서를 업데이트하여 새로운 경고/설정 변경 사항을 공유하세요.
