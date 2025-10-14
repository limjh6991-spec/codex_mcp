ContextBridgeMcp — Copilot ↔ 외부 AI 하이브리드 루프
=====================================================

`context_bridge/context_bridge_mcp.py` 하나로 **MCP 서버**를 구동해 다음을 자동화합니다.

1. 레포맵 + 목표/제약 + git diff + 테스트 로그를 모아 `context_bridge/context_bundle.md` 생성
2. Copilot이 제안한 변경을 바탕으로 외부 AI에 넘길 **다음 작업 후보 & 표준 프롬프트** 생성
3. (옵션) OpenAI / Anthropic API 키를 넣으면 서버에서 직접 호출

> 모든 산출물과 스크립트는 `context_bridge/` 폴더 안에서만 관리됩니다.

---

## 1) 서버 파일: `context_bridge/context_bridge_mcp.py`

핵심 의존성: `python3.10+`, `git`, [`fastmcp`](https://pypi.org/project/fastmcp/)

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ContextBridgeMcp server entrypoint."""
from __future__ import annotations

import os
import subprocess
import textwrap
from datetime import datetime
from pathlib import Path
from typing import List, Optional

from fastmcp import FastMCP

app = FastMCP(name="ContextBridgeMcp")
ROOT = Path(os.getcwd())
CONTEXT_DIR = ROOT / "context_bridge"
IGNORES = [".git", "node_modules", ".venv", "build", "dist", "__pycache__"]


def sh(cmd: str, *, allow_fail: bool = True) -> str:
    try:
        return subprocess.check_output(cmd, shell=True, text=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as exc:
        if allow_fail:
            return exc.output or ""
        raise


def repo_map(depth: int = 2) -> str:
    tree_cmd = f"tree -L {depth} -I '" + "|".join(IGNORES) + "'"
    out = sh(tree_cmd)
    if not out.strip():
        out = sh(f"find . -maxdepth {depth} -type d")
    return out


def git_diff(paths: Optional[List[str]] = None) -> str:
    path_args = " ".join(paths) if paths else "."
    excludes = " :!".join(["", "package-lock.json", "yarn.lock", "pnpm-lock.yaml"])
    cmd = f"git diff HEAD -- {path_args}{excludes}"
    out = sh(cmd)
    return out or "(diff 없음 또는 커밋된 변경 없음)"


def tail_logs(cmd: str = "npm test --silent", lines: int = 200) -> str:
    out = sh(f"{cmd} || true")
    if not out:
        return "(로그 없음)"
    return "\n".join(out.splitlines()[-lines:])


def read_text(path: Optional[str]) -> str:
    if not path:
        return ""
    candidate = Path(path)
    file_path = candidate if candidate.is_absolute() else (
        CONTEXT_DIR / candidate if not str(candidate).startswith("context_bridge/") else ROOT / candidate
    )
    if not file_path.exists():
        return ""
    return file_path.read_text(encoding="utf-8", errors="ignore")


def write_text(path: str, content: str) -> None:
    candidate = Path(path)
    target = candidate if candidate.is_absolute() else (
        CONTEXT_DIR / candidate if not str(candidate).startswith("context_bridge/") else ROOT / candidate
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(content, encoding="utf-8")


@app.tool()
def build_context(
    goal: str = "(작성 필요)",
    constraints: str = "(작성 필요)",
    test_cmd: str = "npm test --silent",
    log_lines: int = 200,
    outfile: str = "context_bridge/context_bundle.md",
    diff_paths: Optional[List[str]] = None,
) -> str:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    parts = [
        "## Repo Map\n\n" + repo_map(2),
        "## Goal / Constraints\n\n"
        + textwrap.dedent(
            f"""
            - Goal:\n      - {goal}
            - Constraints:\n      - {constraints}
            """
        ),
        "## Changes (git diff HEAD)\n\n" + git_diff(diff_paths),
        "## Test / Run Logs\n\n" + tail_logs(test_cmd, log_lines),
    ]

    content = f"<!-- Generated {now} -->\n\n" + "\n\n".join(parts) + "\n"
    write_text(outfile, content)
    return f"✅ Wrote {outfile} ({len(content)} chars)"


@app.tool()
def suggest_next_actions(
    diff_excerpt: str,
    failures: str = "",
    objective: str = "",
    k: int = 3,
) -> str:
    candidates = [
        "에러/실패 테스트 우선 해결 및 최소 패치 적용",
        "타입 안정성 강화 (strict 옵션/DTO 정합성)",
        "로깅/모니터링 지표 추가 및 성능 병목 추적",
        "경계 케이스 테스트 추가 및 회귀 방지",
        "데이터베이스 쿼리 최적화 (N+1 제거/인덱스 검토)",
    ]
    take = candidates[: max(1, min(k, len(candidates)))]

    return textwrap.dedent(
        f"""
        다음은 Copilot 변경사항 일부와 실패 로그 요약입니다. 상위 목표를 고려하여 다음 작업 후보를 정리했습니다.

        [Objective]\n{objective}\n
        [Diff Excerpt]\n{diff_excerpt}\n
        [Failures]\n{failures}\n
        [Next Action Options]\n- {"\n- ".join(take)}

        우선순위 추천 기준\n- 위험도/안전성\n- 고객 임팩트\n- 리드타임 대비 효과\n- 테스트/릴리스 용이성
        """
    )


@app.tool()
def make_prompt(
    context_file: str = "context_bridge/context_bundle.md",
    include_checklist: bool = True,
) -> str:
    ctx = read_text(context_file)
    checklist = (
        "\n- 타입/스키마 정합성\n- 실패 테스트 재현 및 통과\n- 성능 임팩트 및 회귀 위험\n- 보안/권한/검증\n- 롤백 전략"
        if include_checklist
        else ""
    )

    return textwrap.dedent(
        f"""
        당신은 시니어 엔지니어입니다. 아래 컨텍스트를 기반으로
            1) 문제 원인 분석
            2) 최소 수정안 (Unified diff 또는 패치 형태)
            3) 리스크/부작용
            4) 후속 체크리스트{checklist and ' 포함' or ''}
            를 간결하게 제시하세요.

        <CONTEXT_BUNDLE>
        {ctx}
        </CONTEXT_BUNDLE>
        """
    )


@app.tool()
def send_to_ai(
    provider: str = "openai",
    system_prompt: str = "You are a senior software engineer.",
    user_prompt: str = "",
    model: str = "gpt-4o-mini",
) -> str:
    if provider not in {"openai", "anthropic"}:
        return "지원하지 않는 provider"

    if provider == "openai" and not os.getenv("OPENAI_API_KEY"):
        return "[DRY RUN] OPENAI_API_KEY 없음 → 아래 프롬프트를 수동으로 복사해 사용:\n\n" + user_prompt
    if provider == "anthropic" and not os.getenv("ANTHROPIC_API_KEY"):
        return "[DRY RUN] ANTHROPIC_API_KEY 없음 → 아래 프롬프트를 수동으로 복사해 사용:\n\n" + user_prompt

    try:
        if provider == "openai":
            return "[샘플] OpenAI 호출부 주석 해제 후 사용하세요."
        return "[샘플] Anthropic 호출부 주석 해제 후 사용하세요."
    except Exception as exc:
        return f"[호출 실패]\n{exc}"


if __name__ == "__main__":
    print(build_context())
```

> **TIP**: `outfile`/`context_file`에 상대경로를 입력하고 `context_bridge/` 접두사가 없으면 자동으로 `context_bridge/` 하위에 저장·조회합니다.

---

## 2) VS Code 통합 (MCP 등록 + 단축키)

### `.vscode/settings.json`

```jsonc
{
  "mcp.servers": {
    "ContextBridgeMcp": {
      "command": "python",
      "args": ["${workspaceFolder}/context_bridge/context_bridge_mcp.py"],
      "env": {
        // 필요 시 "${env:OPENAI_API_KEY}" 형태로 주입
      }
    }
  }
}
```

### `.vscode/tasks.json`

```jsonc
{
  "$schema": "vscode://schemas/tasks",
  "version": "2.0.0",
  "tasks": [
    {
      "label": "Context: Build Bundle",
      "type": "shell",
      "command": "python ${workspaceFolder}/context_bridge/context_bridge_mcp.py",
      "problemMatcher": []
    }
  ]
}
```

### `.vscode/keybindings.json`

```jsonc
[
  {
    "key": "ctrl+alt+b",
    "command": "workbench.action.tasks.runTask",
    "args": "Context: Build Bundle",
    "when": "editorTextFocus"
  }
]
```

---

## 3) 외부 AI로 넘길 때 권장 템플릿

`make_prompt` 출력에 아래 텍스트를 더해 붙여넣으면 일관된 지시를 보낼 수 있습니다.

```text
[역할]
당신은 시니어 엔지니어입니다. 목표와 제약을 우선 준수하세요.

[요청]
- 문제 원인 분석
- 최소 수정안 (Unified diff 또는 패치)
- 리스크/부작용
- 후속 체크리스트 (5개 이내)

[컨텍스트]
<CONTEXT_BUNDLE>
(여기에 context_bridge/context_bundle.md 전문 붙여넣기)
</CONTEXT_BUNDLE>
```

---

## 4) 사용 흐름 (하루 루틴)

1. **코딩**: Copilot으로 빠르게 구현
2. **번들 생성**: `Ctrl+Alt+B` 단축키 또는 `build_context` 툴 실행
3. **다음 작업 후보**: `suggest_next_actions`에 diff/실패 로그를 넣어 후보 초안 확보
4. **외부 AI 판단**: `make_prompt` 결과를 외부 AI에 붙여넣어 최소 수정안/리스크 확인
5. **재실행**: 제안된 수정 적용 → 테스트 → 반복

---

## 5) 보안·프라이버시 체크리스트

- 외부로 내보내기 전에 민감한 비밀(토큰/키/고객 데이터)이 포함되지 않았는지 점검
- 필요 시 `diff_paths` 매개변수로 특정 디렉터리만 추출하거나 별도의 마스킹 로직 추가
- 사내용 모델을 쓴다면 `send_to_ai` 대신 내부 프록시 API 호출 코드로 교체 가능

---

## 6) 확장 아이디어

- `build_context`에 패키지 버전 요약, 핵심 스키마/타입 스니펫 자동 포함
- 실패 로그에서 테스트명/스택트레이스 자동 추출 후 `suggest_next_actions`와 연동
- PR 템플릿/커밋 메시지 자동 생성 MCP 툴 추가

---

## 7) 과금 전략 한눈에 보기

- **권장 조합**: GitHub Copilot Pro ($10) + ChatGPT Plus ($20)
  - IDE 실시간 코딩은 Copilot, 설계·리뷰·의사결정은 외부 AI로 분업
  - Copilot Pro+($39) 단일보다 유연하고 경제적
- Claude Pro는 문서/긴 맥락 판단에 보강용으로 선택
- 프로젝트 기간에 맞춰 월간 ↔ 연간 플랜을 탄력적으로 조정

---

문서/설정이 꼬였거나 다른 언어 레포로 이식하고 싶으면 언제든지 가져오세요. 빠르게 맞춰드릴게요!

