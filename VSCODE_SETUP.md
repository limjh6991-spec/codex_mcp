# VS Code 설정 가이드

## 1. 사전 준비

### 필수 설치 항목
- [VS Code](https://code.visualstudio.com/) (최신 버전)
- [Node.js](https://nodejs.org/) (v18 이상)
- [GitHub Copilot](https://marketplace.visualstudio.com/items?itemName=GitHub.copilot) 확장
- [GitHub Copilot Chat](https://marketplace.visualstudio.com/items?itemName=GitHub.copilot-chat) 확장

## 2. 프로젝트 설정

### 단계별 설치

1. **프로젝트 클론 및 설치**
   ```bash
   git clone https://github.com/limjh6991-spec/codex_mcp.git
   cd codex_mcp
   npm install
   npm run build
   ```

2. **VS Code에서 프로젝트 열기**
   ```bash
   code .
   ```

3. **권장 확장 프로그램 설치**
   - VS Code가 프로젝트를 열면 `.vscode/extensions.json`에 정의된 확장 프로그램 설치를 권장합니다
   - "Install" 버튼을 클릭하여 GitHub Copilot 확장을 설치하세요

## 3. MCP 서버 구성

### MCP 설정 파일 위치
- `.vscode/mcp-config.json` - MCP 서버 설정

### 설정 내용
```json
{
  "mcpServers": {
    "codex": {
      "command": "node",
      "args": ["./dist/index.js"],
      "env": {}
    }
  }
}
```

이 설정은 GitHub Copilot Chat이 자동으로 MCP 서버를 시작하도록 합니다.

## 4. 사용 방법

### 기본 사용

1. **Copilot Chat 열기**
   - 단축키: `Ctrl+Shift+I` (Windows/Linux) 또는 `Cmd+Shift+I` (Mac)
   - 또는 사이드바에서 Chat 아이콘 클릭

2. **MCP 도구 사용**
   - Chat에서 자연어로 명령을 입력하면, Copilot이 자동으로 MCP 서버의 도구를 사용합니다
   
   예시:
   ```
   "이 프로젝트의 모든 TypeScript 파일을 찾아줘"
   "README.md 파일의 내용을 보여줘"
   "src/index.ts 파일을 분석해줘"
   ```

### 개발 모드

개발 중에는 TypeScript 파일을 수정하면서 자동으로 재컴파일할 수 있습니다:

1. **터미널에서 watch 모드 시작**
   ```bash
   npm run watch
   ```

2. **코드 수정**
   - `src/index.ts` 파일을 수정
   - 저장하면 자동으로 `dist/index.js`가 업데이트됨

3. **MCP 서버 재시작**
   - VS Code를 다시 로드하거나
   - Copilot Chat을 다시 열어서 새 세션 시작

## 5. 디버깅

### VS Code 디버거 사용

1. **디버그 뷰 열기**
   - 단축키: `Ctrl+Shift+D` (Windows/Linux) 또는 `Cmd+Shift+D` (Mac)

2. **디버그 설정 선택**
   - "Launch MCP Server" 선택

3. **디버그 시작**
   - F5 키를 누르거나 "Start Debugging" 버튼 클릭

4. **브레이크포인트 설정**
   - `src/index.ts` 파일에서 원하는 줄에 클릭하여 브레이크포인트 추가

### 로그 확인

MCP 서버는 stderr로 로그를 출력합니다:
```bash
npm start 2>&1 | tee mcp-server.log
```

## 6. 문제 해결

### MCP 서버가 시작되지 않는 경우

1. **빌드 확인**
   ```bash
   npm run build
   ls -la dist/
   ```
   `dist/index.js` 파일이 존재하는지 확인

2. **Node.js 버전 확인**
   ```bash
   node --version
   ```
   v18 이상이어야 함

3. **수동 실행 테스트**
   ```bash
   npm start
   ```
   에러 메시지 확인

### Copilot이 MCP 도구를 사용하지 않는 경우

1. **GitHub Copilot Chat 버전 확인**
   - VS Code 확장 메뉴에서 GitHub Copilot Chat 확장이 최신 버전인지 확인
   - MCP 지원 버전이어야 함 (2024년 이후 버전)

2. **VS Code 재시작**
   - VS Code를 완전히 종료하고 다시 시작

3. **설정 파일 확인**
   - `.vscode/mcp-config.json` 파일의 경로가 올바른지 확인
   - 상대 경로가 프로젝트 루트를 기준으로 하는지 확인

### 권한 오류

파일 시스템 작업 시 권한 오류가 발생하면:

```bash
# Linux/Mac
chmod +x dist/index.js

# Windows에서는 관리자 권한으로 VS Code 실행
```

## 7. 고급 설정

### 환경 변수 추가

`.vscode/mcp-config.json`에서 환경 변수를 설정할 수 있습니다:

```json
{
  "mcpServers": {
    "codex": {
      "command": "node",
      "args": ["./dist/index.js"],
      "env": {
        "DEBUG": "true",
        "LOG_LEVEL": "verbose"
      }
    }
  }
}
```

### 여러 MCP 서버 사용

다른 MCP 서버와 함께 사용:

```json
{
  "mcpServers": {
    "codex": {
      "command": "node",
      "args": ["./dist/index.js"]
    },
    "another-server": {
      "command": "path/to/another/server",
      "args": []
    }
  }
}
```

## 8. 추가 리소스

- [MCP 프로토콜 문서](https://modelcontextprotocol.io/)
- [GitHub Copilot 문서](https://docs.github.com/en/copilot)
- [VS Code API 문서](https://code.visualstudio.com/api)
