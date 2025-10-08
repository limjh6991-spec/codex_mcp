# Contributing to Codex MCP

이 프로젝트에 기여해 주셔서 감사합니다! 🎉

## 기여 방법

### 1. 이슈 제기

버그를 발견하거나 새로운 기능을 제안하고 싶다면:

1. [Issues](https://github.com/limjh6991-spec/codex_mcp/issues) 페이지에서 기존 이슈 확인
2. 중복되지 않는다면 새 이슈 생성
3. 명확한 제목과 설명 작성

### 2. Pull Request

코드 기여를 하고 싶다면:

1. **Fork & Clone**
   ```bash
   git clone https://github.com/YOUR_USERNAME/codex_mcp.git
   cd codex_mcp
   ```

2. **브랜치 생성**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **개발 환경 설정**
   ```bash
   npm install
   npm run build
   ```

4. **변경사항 작성**
   - 코드를 수정하거나 새 기능 추가
   - 주석과 문서 업데이트
   - TypeScript의 타입 안전성 유지

5. **테스트**
   ```bash
   npm run build
   npm start
   ```

6. **커밋**
   ```bash
   git add .
   git commit -m "Add: 새 기능 설명"
   ```

   커밋 메시지 규칙:
   - `Add:` - 새 기능 추가
   - `Fix:` - 버그 수정
   - `Update:` - 기존 기능 개선
   - `Docs:` - 문서 변경
   - `Refactor:` - 코드 리팩토링

7. **Push & PR**
   ```bash
   git push origin feature/your-feature-name
   ```
   그런 다음 GitHub에서 Pull Request 생성

### 3. 코드 스타일

- TypeScript strict 모드 사용
- ESLint 규칙 준수 (설정될 예정)
- 명확한 변수/함수 이름 사용
- 복잡한 로직에는 주석 추가

### 4. 새 도구 추가

MCP 도구를 추가하려면:

1. **도구 정의** (`setupToolHandlers` 메서드)
   ```typescript
   {
     name: "your_tool_name",
     description: "도구 설명",
     inputSchema: {
       type: "object",
       properties: {
         param1: {
           type: "string",
           description: "매개변수 설명",
         },
       },
       required: ["param1"],
     },
   }
   ```

2. **핸들러 구현**
   ```typescript
   private async handleYourTool(args: any) {
     const param1 = args.param1 as string;
     // 도구 로직 구현
     return {
       content: [
         {
           type: "text",
           text: "결과",
         },
       ],
     };
   }
   ```

3. **switch 문에 케이스 추가**
   ```typescript
   case "your_tool_name":
     return await this.handleYourTool(args);
   ```

4. **문서 업데이트**
   - README.md에 도구 설명 추가
   - EXAMPLES.md에 사용 예제 추가

## 개발 팁

### Watch 모드 사용
```bash
npm run watch
```
파일 변경 시 자동으로 재컴파일됩니다.

### 디버깅
VS Code에서 F5를 눌러 디버그 모드로 실행할 수 있습니다.

### 로그 확인
```bash
npm start 2>&1 | tee logs.txt
```

## 질문이 있나요?

- [Issues](https://github.com/limjh6991-spec/codex_mcp/issues)에서 질문하세요
- 또는 Pull Request에 코멘트를 남겨주세요

## Code of Conduct

모든 참여자는 서로를 존중하고 건설적인 피드백을 제공해야 합니다.

감사합니다! 🙏
