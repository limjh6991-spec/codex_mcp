# 🚀 빠른 시작 가이드

## 30초 설치

```bash
# 클론 및 설치
git clone https://github.com/limjh6991-spec/codex_mcp.git
cd codex_mcp
npm install && npm run build

# VS Code에서 열기
code .
```

## 5분 사용법

### 1단계: GitHub Copilot 설치
VS Code에서 확장 프로그램 설치:
- GitHub Copilot
- GitHub Copilot Chat

### 2단계: Copilot Chat 열기
- 단축키: `Ctrl+Shift+I` (Windows/Linux)
- 또는 `Cmd+Shift+I` (Mac)

### 3단계: 사용 시작! 🎉

```
💬 "이 프로젝트의 모든 파일을 보여줘"
💬 "README.md를 읽어줘"
💬 "src/index.ts 파일을 분석해줘"
💬 "test.txt 파일을 만들고 'Hello, World!'를 써줘"
```

## 작동 확인

터미널에서 직접 실행:
```bash
npm start
# "Codex MCP Server running on stdio" 메시지 확인
```

## 문제 해결

### 서버가 안 뜬다면?
```bash
npm run build  # 다시 빌드
ls dist/       # index.js 파일 확인
```

### Copilot이 인식 안 된다면?
1. VS Code 재시작
2. Copilot 확장 업데이트 확인
3. `.vscode/mcp-config.json` 파일 확인

## 상세 문서

- 📘 [전체 README](README.md)
- ⚙️ [VS Code 설정](VSCODE_SETUP.md)
- 📝 [사용 예제](EXAMPLES.md)
- 🏗️ [아키텍처](ARCHITECTURE.md)
- 🤝 [기여하기](CONTRIBUTING.md)

## 주요 기능

| 도구 | 설명 |
|------|------|
| 📖 read_file | 파일 읽기 |
| ✍️ write_file | 파일 쓰기 |
| 📂 list_directory | 디렉토리 목록 |
| 🔍 analyze_code | 코드 분석 |
| 🔎 search_files | 파일 검색 |

---

**즐거운 코딩 되세요!** 🎨✨
