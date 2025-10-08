# Codex MCP Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        VS Code IDE                           │
│  ┌──────────────────────────────────────────────────────┐   │
│  │         GitHub Copilot Chat Extension                │   │
│  │  - 자연어 이해                                        │   │
│  │  - MCP 클라이언트                                     │   │
│  └──────────────────┬───────────────────────────────────┘   │
│                     │                                         │
│                     │ MCP Protocol (stdio)                    │
│                     │                                         │
│  ┌──────────────────▼───────────────────────────────────┐   │
│  │          Codex MCP Server (Node.js)                  │   │
│  │                                                       │   │
│  │  ┌─────────────────────────────────────────────┐    │   │
│  │  │         Tool Handlers                       │    │   │
│  │  │  • read_file      - 파일 읽기              │    │   │
│  │  │  • write_file     - 파일 쓰기              │    │   │
│  │  │  • list_directory - 디렉토리 목록          │    │   │
│  │  │  • analyze_code   - 코드 분석              │    │   │
│  │  │  • search_files   - 파일 검색              │    │   │
│  │  └─────────────────────────────────────────────┘    │   │
│  │                                                       │   │
│  └───────────────────┬───────────────────────────────────┘   │
│                      │                                        │
└──────────────────────┼────────────────────────────────────────┘
                       │
                       ▼
              ┌────────────────────┐
              │  File System       │
              │  - 프로젝트 파일    │
              │  - 소스 코드        │
              │  - 설정 파일        │
              └────────────────────┘
```

## 워크플로우

1. **사용자 요청**
   - 사용자가 Copilot Chat에 자연어로 요청
   - 예: "README.md 파일을 읽어줘"

2. **요청 처리**
   - Copilot Chat이 요청을 분석
   - 적절한 MCP 도구를 선택

3. **MCP 통신**
   - MCP 프로토콜로 서버에 요청 전송
   - JSON-RPC 형식의 메시지

4. **도구 실행**
   - MCP 서버가 요청된 도구 실행
   - 파일 시스템 작업 수행

5. **응답 반환**
   - 실행 결과를 MCP 프로토콜로 반환
   - Copilot Chat이 사용자에게 표시

## 프로젝트 구조

```
codex_mcp/
├── src/
│   └── index.ts              # MCP 서버 구현
│       ├── CodexMCPServer    # 메인 서버 클래스
│       ├── setupToolHandlers # 도구 등록
│       └── handle* methods   # 각 도구의 핸들러
│
├── .vscode/
│   ├── mcp-config.json       # MCP 서버 설정 ⚙️
│   ├── extensions.json       # 권장 확장
│   ├── launch.json           # 디버그 설정
│   └── tasks.json            # 빌드 작업
│
├── dist/                     # 컴파일된 JavaScript (gitignore)
│   ├── index.js
│   └── index.js.map
│
├── package.json              # 프로젝트 의존성
├── tsconfig.json             # TypeScript 설정
├── README.md                 # 메인 문서
├── VSCODE_SETUP.md          # VS Code 설정 가이드
└── EXAMPLES.md              # 사용 예제
```

## 핵심 기술

- **MCP (Model Context Protocol)**: AI와 도구 간의 표준 통신 프로토콜
- **TypeScript**: 타입 안전한 서버 구현
- **Node.js**: 런타임 환경
- **stdio Transport**: 표준 입출력을 통한 통신
- **VS Code Extensions API**: VS Code와의 통합

## 확장 가능성

이 구조는 다음과 같이 확장할 수 있습니다:

1. **AI 통합**
   - OpenAI Codex API 연동
   - 고급 코드 분석 및 생성

2. **추가 도구**
   - 코드 리팩토링
   - 테스트 생성
   - 문서 자동 생성

3. **외부 서비스**
   - Git 작업
   - 데이터베이스 쿼리
   - API 호출

4. **성능 최적화**
   - 캐싱
   - 비동기 처리
   - 병렬 실행
