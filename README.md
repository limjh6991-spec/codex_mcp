# Codex MCP Server

VS Code와 통합된 MCP (Model Context Protocol) 서버로 코드 분석 및 조작 도구를 제공합니다.

## 개요

이 프로젝트는 VS Code 환경에서 AI 어시스턴트(Copilot 등)가 코드베이스와 상호작용할 수 있도록 하는 MCP 서버를 구현합니다. MCP는 AI 모델이 다양한 도구와 데이터 소스에 접근할 수 있게 하는 표준 프로토콜입니다.

## 기능

- **파일 읽기**: 워크스페이스의 파일 내용 읽기
- **파일 쓰기**: 워크스페이스에 파일 생성/수정
- **디렉토리 목록**: 디렉토리 구조 탐색
- **코드 분석**: 기본 코드 구조 분석 및 통계
- **파일 검색**: 패턴 기반 파일 검색

## 설치

### 필수 조건

- Node.js 18.x 이상
- VS Code
- GitHub Copilot 확장 프로그램 (권장)

### 설치 단계

1. 저장소 클론:
```bash
git clone https://github.com/limjh6991-spec/codex_mcp.git
cd codex_mcp
```

2. 의존성 설치:
```bash
npm install
```

3. 프로젝트 빌드:
```bash
npm run build
```

## 사용법

### 서버 실행

```bash
npm start
```

### VS Code와 통합

1. VS Code에서 프로젝트 열기
2. GitHub Copilot Chat 확장이 설치되어 있는지 확인
3. MCP 서버가 자동으로 시작됩니다 (`.vscode/mcp-config.json` 참조)

### 개발 모드

TypeScript 파일을 수정하면서 자동으로 컴파일하려면:

```bash
npm run watch
```

## MCP 도구

이 서버는 다음 도구를 제공합니다:

### read_file
파일 내용을 읽습니다.
```json
{
  "path": "path/to/file.txt"
}
```

### write_file
파일에 내용을 씁니다.
```json
{
  "path": "path/to/file.txt",
  "content": "파일 내용"
}
```

### list_directory
디렉토리의 파일과 폴더를 나열합니다.
```json
{
  "path": "path/to/directory"
}
```

### analyze_code
코드를 분석하고 통계와 인사이트를 제공합니다.
```json
{
  "code": "function hello() { return 'world'; }",
  "language": "javascript"
}
```

### search_files
패턴과 일치하는 파일을 검색합니다.
```json
{
  "directory": "src",
  "pattern": "*.ts"
}
```

## 프로젝트 구조

```
codex_mcp/
├── src/
│   └── index.ts          # MCP 서버 메인 구현
├── .vscode/
│   ├── extensions.json   # 권장 VS Code 확장
│   ├── mcp-config.json   # MCP 서버 설정
│   ├── launch.json       # 디버깅 설정
│   └── tasks.json        # 빌드 작업 설정
├── package.json          # 프로젝트 의존성
├── tsconfig.json         # TypeScript 설정
└── README.md             # 이 파일
```

## VS Code 설정

`.vscode/mcp-config.json` 파일은 Copilot Chat이 MCP 서버를 자동으로 시작하도록 설정합니다:

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

## 확장 기능

이 서버는 기본 기능을 제공하며, 다음과 같이 확장할 수 있습니다:

- **OpenAI Codex 통합**: 고급 코드 분석을 위해 OpenAI API 추가
- **추가 도구**: 코드 리팩토링, 테스트 생성 등의 도구 추가
- **데이터베이스 연동**: 프로젝트 메타데이터 저장
- **Git 통합**: 버전 관리 작업 지원

## 문제 해결

### 서버가 시작되지 않는 경우
1. `npm run build`를 실행하여 프로젝트가 빌드되었는지 확인
2. `dist/` 디렉토리에 컴파일된 파일이 있는지 확인
3. Node.js 버전이 18.x 이상인지 확인

### VS Code에서 MCP 서버를 인식하지 못하는 경우
1. GitHub Copilot Chat 확장이 최신 버전인지 확인
2. VS Code를 다시 시작
3. `.vscode/mcp-config.json` 파일의 경로가 올바른지 확인

## 기여

이슈 보고 및 풀 리퀘스트를 환영합니다!

## 라이선스

MIT License