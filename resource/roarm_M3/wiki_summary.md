# Waveshare RoArm-M3 위키 요약 (2025-10-12 기준)

공식 위키: <https://www.waveshare.com/wiki/RoArm-M3>

## 1. 제품 개요 & 하드웨어 특징
- 5+1 DOF(기본 회전 + 5개 관절) 설계, 유효 작업 반경 약 1m.
- 0.2kg 페이로드 @ 0.5m, 듀얼 드라이브 숄더 구동으로 토크 향상.
- ESP32 기반 메인 컨트롤러, Wi-Fi(AP/STA), ESP-NOW, I2C/UART/TTL 버스 서보 인터페이스 제공.
- 12-bit 자기 엔코더, 재현 정밀도 0.088°.
- RoArm-M3 시리즈는 LeRobot, ROS2 등과 호환되며 웹 UI 제공.

## 2. 전원 및 안전 주의사항
- 권장 전원: 12V 5A 어댑터 또는 3S LiPo (동작 범위 7~12.6V).
- 출고 시 완전 조립, 분해 비권장; 고토크 서보 취급 시 안전거리 확보.
- 기본 데모 속도는 안전을 위해 느리게 설정 → 필요 시 튜토리얼 참조하여 조정.

## 3. 인터페이스 & 보드 구성
- 주요 보드: General Driver for Robots (ESP32-WROOM-32 + 다수 확장 포트).
- 포트 요약: Wi-Fi 안테나(IPEX), LiDAR, I2C 확장, USB(Type-C) 2계열, XH2.54 전원 입력, ST3215 버스 서보 포트, PH2.0 모터 포트, INA219 센서, IMU(QMI8658), 전자 나침반(AK09918), TF 카드, 듀얼 40핀 헤더(RPi/Jetson 호환), CP2102 USB-UART 2계열 등.
- 보드 스위치/버튼: 전원, 리셋(EN), 다운로드(BOOT), 자동 다운로드 회로 내장.

## 4. 기본 사용 흐름
1. 전원 연결 및 스위치 ON → 모든 관절이 중립 위치로 이동.
2. OLED 표시: AP SSID(기본 RoArm-M3), STA 모드 상태, ESP-NOW MAC.
3. 모바일/PC에서 SSID `RoArm-M3` 접속(패스워드 12345678) → 브라우저 `192.168.4.1` 접속.
4. 웹 UI 섹션:
   - **AngleCtrl**: 각 관절 라디안 단위 제어(B/L, S/U, E, W, R, G).
   - **Torque**: 토크 잠금 ON/OFF.
   - **DEFA**: 외력 보상 기능.
   - **LED**: LED 제어.
   - **Horizontal/Vertical Drag**: 평면 드로잉 도구 (별도 문서 참조).
   - **CoordCtrl**: TCP 좌표/자세 제어 (XYZTRG).
   - **Feedback Information**: JSON 명령 송수신 콘솔.

## 5. JSON 명령 & 통신 옵션
- JSON 명령 구조: `{ "T": <cmd_id>, "cmd": <value>, "spd": <value>, "acc": <value> }` 등.
- 장점: 가독성, 확장성, 다중 언어 호환.
- 통신 경로:
  - **Web UI**: 기본 입력 박스에서 JSON 송신.
  - **HTTP API**: `http_simple_ctrl.py` 예제 (요청 URL: `http://<ip>/js?json=...`).
  - **Serial**: `/dev/ttyUSB*` 등으로 UART 연결, 115200bps. `serial_simple_ctrl.py` 예제 제공.
  - **ESP-NOW**: 예제/구성 JSON 파일은 펌웨어 자료 참고.
- JSON ID 정의: `json_cmd.h` 참고 (펌웨어 아카이브 `RoArm-M3_example-250108.zip` 포함).

## 6. 고급 기능 & 튜토리얼 맵
### 6.1 RoArm-M3-S Tutorial 시리즈
- Web 사용법, 2차 개발 도구, JSON 명령 의미, Wi-Fi 설정, 로봇팔 제어, FLASH FS, 스텝 레코딩/재생.
- ESP-NOW 제어, Python UART/HTTP 제어 튜토리얼 포함.

### 6.2 ROS2 Tutorial Catalog
1. ROS2 설치 가이드
2. ROS2 워크스페이스 구조
3. 실물 로봇팔 제어 드라이버 노드
4. MoveIt2 드래그 앤 드롭 인터랙션
5. 키보드 제어
6. 명령어 기반 제어
7. MoveIt MTC 데모

## 7. 리소스 모음 (다운로드 링크)
- **펌웨어/예제**: `RoArm-M3_example-250108.zip`
- **Python 데모**: `RoArm-M3_Python.zip`
- **3D CAD (STEP)**: `RoArm-M3_STEP.zip`
- **2D 도면**: `RoArm-M3_2Dsize.zip`, 카메라 마운트 브래킷 도면
- **드라이버/툴**:
  - 수평/수직 드로잉 툴: `Vertical and horizontal plane control tools.zip`
  - 초기화 다운로드 툴: `RoArm-M3_FACTORY-250113.zip`
  - CP2102 USB-UART 드라이버: `CP210x_USB_TO_UART.zip`
  - 로봇 드라이버 보드 회로도: `General_Driver_for_Robots.pdf`

## 8. FAQ 핵심 Q&A
1. **M3-S vs M3-Pro 차이**: Pro는 대부분 금속 쉘 ST3235 서보 → 백래시 감소.
2. **시리얼 제어 무응답**: 전원/스위치 확인 → USB 포트(번호 9) 확인 → 포트 번호 점검.
3. **COM 포트 미표시**: USB 연결 위치 확인, CP2102 드라이버 설치 필요.
4. **펌웨어 업로드 실패(Connecting...)**: EN/BOOT 수동 시퀀스로 강제 다운로드.
5. **재플래시 후 Wi-Fi 오류**: 시리얼로 `{ "T":604 }` 명령 전송해 NVS 초기화 후 재부팅.

## 9. 개발 참고 노트
- Isaac Sim/ROS2 연동 시 JSON 명령과 ROS2 튜토리얼 문서 참고해 제어 명령 매핑.
- 듀얼 드라이브 숄더 서보(주/종 ID 12/13) → URDF 내 mimic/전송 구성 시 주의.
- STEP/2D 자료로 정확한 링크 치수 확보 → USD/URDF 갱신에 활용.
- ESP32 펌웨어 빌드는 Arduino 기반; `RoArm-M3_config.h` 로봇 기구값, 토크 파라미터 확인.
- Wi-Fi 모드(AP/STA) 상태에 따라 HTTP 예제의 대상 IP가 달라짐: OLED 출력 참고.

---
최종 갱신: 2025-10-12 (작성자: GitHub Copilot)
