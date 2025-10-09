VS Code에서 실제로 쓰는 방법 (대화 예)
사용자(당신): “새 로봇팔 콘셉트 이미지 만들고, Blender 장면 세팅해줘.”
**Copilot(Chat)**는 내부적으로:
    1. PromptGeneratorMcp.generate_3d_robot_prompt(theme='bio-inspired', constraints={links:7, payloadKg:2})
    2. StableDiffusionMcp.create_robot_image(prompt=<1 결과>)
    3. BlenderMcp.import_image_and_prepare_scene(imagePath=...)
    4. (옵션) GitHubMcp.create_branch(repo='roarm-design', base='main', newBranch='concept-v1')로 산출물 버전관리

MCP Inspector로 점검
    1. VS Code에서 MCP Inspector 열기 → STDIO 선택
    2. 서버 스크립트 경로 지정(예: servers/prompt_generator/server.py) → Connect
    3. 좌측 툴 목록/우측 요청-응답 확인 → 인자/출력 형식 검증

운영 팁 & 베스트 프랙티스
    • 로그: 각 서버에서 logs/<server>.log로 파일 로그 남기기(+ 회전/단일 파일 크기 제한).
    • 타임아웃: 이미지 생성/Blender 호출은 요청별 타임아웃과 재시도 정책 추가.
    • 리소스 분리: 무거운 툴(Blender/Isaac Sim)은 별 프로세스/머신으로 분리해도 MCP로 동일하게 제어 가능.
    • 권한 최소화: 서버별 필요한 키만 env로 주입. .env.example 제공으로 팀 온보딩 간편화.
    • 테스트: 각 툴에 대해 단위 테스트(입력→예상 출력) 스냅샷 유지.

흔한 오류 빠른 해결
    • Invalid API key · Please run /login
→ 실제로는 MCP 서버 측에서 process.env.X(Node) 또는 os.environ['X'](Python)가 비었을 가능성↑.
        1. .env 채움 → VS Code 재시작 또는 터미널에서 export $(cat .env | xargs)
        2. mcp.json의 "env": {"KEY":"${env:KEY}"} 확인
        3. 서버 코드에서 키 존재 여부 사전 검증 후 친절한 에러 메시지 반환
    • 표준입출력 버퍼링으로 응답 없음
→ Python -u/PYTHONUNBUFFERED=1 + print(..., flush=True) 필수.
→ Node는 라인 단위 \n로 반드시 끝내기.
    • 경로/권한 문제(Blender/Isaac Sim)
→ OS별 절대경로 사용, 공백있는 경로는 인용부호 처리.
→ Linux는 실행 권한(chmod +x) 및 디스플레이/Wayland 변수 확인.
