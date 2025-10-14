# 듀얼 GPU(AMD iGPU + NVIDIA dGPU) Isaac Sim 헤드리스 운영 가이드

이 문서는 2025-10-13 안정화 작업에서 정리한 듀얼 GPU 시스템(AMD 내장 + NVIDIA RTX)에서 Isaac Sim을 헤드리스로 운용할 때의 핵심 설정과 주의사항을 모아 둔 참고 자료다.

## 핵심 메시지
- 문제의 원인은 Isaac Sim이 초기화 과정에서 AMD Mesa 경로로 진입했다가 RTX 렌더러와 섞이며 충돌한 데 있었다.
- Vulkan/GLX/EGL 로더에게 **항상 NVIDIA 벤더만 보이도록 강제**하자 headless 종료 크래시가 사라졌다.
- `scripts/activate_isaacsim_env.sh`에 동일한 환경 변수를 기본값으로 넣어 재부팅 이후에도 안전하게 유지한다.

## 마무리 체크리스트
- `VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json` 유지.
- `__GLX_VENDOR_LIBRARY_NAME=nvidia`, `CUDA_VISIBLE_DEVICES=0` 고정.
- (선택) `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json` 고정.
- UI/메뉴 확장 비활성화 TOML(예: `omni.isaac.menu`, `omni.kit.property.isaac`) 유지.
- 런처에서는 `--no-window`, `hideUi=True`, `disable_viewport_updates=True` 등 headless 옵션 유지.

## 지속 적용 팁
- 위 환경변수를 `~/isaacsim/run_py.sh` 또는 전용 `.env` 파일(예: `env/headless_isaac.env`)로 묶어두면 실수를 줄일 수 있다.
- 오버레이(`scripts/config/roarm_headless.overlay.kit`)는 **화이트리스트 방식**으로 유지해, 필요 확장만 켠다.

## 왜 충돌이 발생했는가?
- GLVND/Vulkan loader는 시스템 전체 드라이버(AMD Mesa, NVIDIA)를 모두 나열한 뒤 벤더를 고른다.
- Isaac Sim headless 앱이 EGL/GLX/Vulkan 초기화 시 AMD 경로를 먼저 잡으면 CUDA/RTX 경로와 섞이면서 세그폴트가 발생한다.
- AMD ↔ NVIDIA 간 zero-copy가 불가하여 혼합 파이프라인은 CPU 메모리 왕복이 늘어나고 성능도 저하된다.
- Isaac RTX/Replicator/GUI 확장 일부는 그래픽 스택을 다시 건드리므로, 종료 단계에서 NVIDIA 고정 설정이 풀릴 위험이 있다.

## 언제 멀티 GPU가 이득인가?
- **동일 벤더(NVIDIA+NVIDIA)**: RTX 다중 GPU, 대량 카메라/센서 렌더링, Replicator 합성 등에서 확실한 스케일링 가능.
- **이기종(AMD+NVIDIA)**: "역할 분리"(AMD=디스플레이, NVIDIA=오프스크린 연산)일 때는 안정적. 두 GPU를 하나의 파이프라인에서 동시에 쓰면 충돌·오버헤드가 커질 가능성이 높다.

## 운영 모드별 권장 구성
### 1. NVIDIA 전용 Headless (현재 모드)
- NVIDIA ICD, GLX, EGL만 노출.
- `MESA_LOADER_DRIVER_OVERRIDE=nvidia`, `__VK_LAYER_NV_optimus=NVIDIA_only`와 함께 사용.
- Headless 옵션 및 확장 비활성화 유지.

### 2. 하이브리드 데스크탑
- X/Wayland는 AMD, Isaac 실행 래퍼에서는 NVIDIA 고정 환경 변수를 적용.
- 필요 시 `CUDA_VISIBLE_DEVICES`로 NVIDIA만 노출.

### 3. 다중 NVIDIA 확장
- `CUDA_VISIBLE_DEVICES=0,1`, `/renderer/multiGpu/enabled=True` 등을 조합해 테스트.
- 워크로드가 GPU 바운드(카메라 수, 해상도, 샘플 수 등)일 때만 스케일링 이득이 크다.

## 빠른 자기진단 명령
```bash
vulkaninfo | grep -E "deviceName"        # NVIDIA만 노출되는지 확인
glxinfo -B                                  # OpenGL vendor/renderer가 NVIDIA인지 확인
eglinfo -B                                 # EGL vendor/renderer가 NVIDIA인지 확인
```

## 설정 유지 체크리스트
- Isaac 전용 실행 스크립트에 환경 변수 고정.
- 확장 화이트리스트 유지(불필요 UI/메뉴/Replicator 비활성화).
- 드라이버/커널 업데이트 후 위 3가지 명령으로 즉시 재검증.

## 추가 고려 사항
- IOMMU/전원 관리 설정은 멀티 NVIDIA 구성에서 P2P 성능에 영향을 줄 수 있다.
- 다중 GPU 확장 계획이 생기면, NVLink 유무·루트 컴플렉스 구조·워크로드 특성을 함께 검토해야 한다.

---
**결론:** NVIDIA 한 장을 고정해 사용하는 현재 구성이 가장 안정적이다. AMD+NVIDIA를 동시에 하나의 파이프라인으로 묶는 것은 충돌과 오버헤드 위험이 크므로 권장하지 않는다. 멀티 NVIDIA로 확장할 계획이 생기면 워크로드 요구 사항을 기준으로 별도 튜닝을 진행한다.
