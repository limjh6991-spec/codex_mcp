# MCP Server Usage Examples

## Example 1: Reading a File

```javascript
// MCP 도구 호출 예시
{
  "tool": "read_file",
  "arguments": {
    "path": "./README.md"
  }
}

// 예상 응답
{
  "content": [
    {
      "type": "text",
      "text": "# Codex MCP Server\n\nVS Code와 통합된..."
    }
  ]
}
```

## Example 2: Analyzing Code

```javascript
// JavaScript 코드 분석
{
  "tool": "analyze_code",
  "arguments": {
    "code": `
      class Calculator {
        add(a, b) {
          return a + b;
        }
        
        // 곱셈 함수
        multiply(a, b) {
          return a * b;
        }
      }
    `,
    "language": "javascript"
  }
}

// 예상 응답
{
  "content": [
    {
      "type": "text",
      "text": {
        "language": "javascript",
        "statistics": {
          "lines": 10,
          "characters": 234
        },
        "features": {
          "hasClasses": true,
          "hasFunctions": true,
          "hasComments": true
        },
        "suggestion": "This is a basic analysis..."
      }
    }
  ]
}
```

## Example 3: Searching Files

```javascript
// TypeScript 파일 검색
{
  "tool": "search_files",
  "arguments": {
    "directory": "./src",
    "pattern": "*.ts"
  }
}

// 예상 응답
{
  "content": [
    {
      "type": "text",
      "text": [
        "/home/user/project/src/index.ts",
        "/home/user/project/src/utils.ts",
        "/home/user/project/src/types.ts"
      ]
    }
  ]
}
```

## Example 4: Writing a File

```javascript
// 새 파일 생성
{
  "tool": "write_file",
  "arguments": {
    "path": "./output/result.txt",
    "content": "Hello, World!\nThis is generated content."
  }
}

// 예상 응답
{
  "content": [
    {
      "type": "text",
      "text": "Successfully wrote to ./output/result.txt"
    }
  ]
}
```

## Example 5: Listing Directory Contents

```javascript
// 디렉토리 내용 조회
{
  "tool": "list_directory",
  "arguments": {
    "path": "./src"
  }
}

// 예상 응답
{
  "content": [
    {
      "type": "text",
      "text": [
        {
          "name": "index.ts",
          "type": "file"
        },
        {
          "name": "utils",
          "type": "directory"
        },
        {
          "name": "types.ts",
          "type": "file"
        }
      ]
    }
  ]
}
```

## VS Code Copilot Chat에서 사용하기

GitHub Copilot Chat과 함께 사용할 때는 자연어로 요청하면 됩니다:

1. "현재 프로젝트의 모든 TypeScript 파일을 찾아줘"
2. "README.md 파일의 내용을 보여줘"
3. "이 코드를 분석해줘: [코드 붙여넣기]"
4. "새 파일을 생성해서 이 내용을 저장해줘"

MCP 서버가 이러한 요청을 적절한 도구 호출로 변환하여 처리합니다.
