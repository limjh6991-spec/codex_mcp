#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ContextBridgeMcp server entrypoint.

이 스크립트는 fastmcp 프레임워크를 이용해 MCP 서버를 구동하며
다음과 같은 툴을 제공합니다.

- build_context: git diff / 레포맵 / 테스트 로그를 수집해 context_bundle.md 생성
- suggest_next_actions: 변경 요약과 실패 로그를 기반으로 다음 작업 후보 제안
- make_prompt: 외부 AI에 붙여넣기 위한 표준 프롬프트 템플릿 생성
- send_to_ai: (선택) OpenAI/Anthropic API 호출 또는 드라이런 메시지 출력

필수 의존성:
    pip install fastmcp
선택 의존성:
    OpenAI, anthropic Python SDK (send_to_ai 사용 시)
"""
from __future__ import annotations

import io
import os
import shutil
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
    """Run *cmd* in shell and return stdout (stderr merged).

    allow_fail=False이면 CalledProcessError가 그대로 전파됩니다.
    """
    try:
        return subprocess.check_output(cmd, shell=True, text=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as exc:  # pragma: no cover - 단순 헬퍼
        if allow_fail:
            return exc.output or ""
        raise


def _tail_last(text: str, lines: int) -> str:
    parts = text.splitlines()
    return "\n".join(parts[-lines:]) if parts else ""


def tail_logs(cmd: str = "npm test --silent", lines: int = 200) -> str:
    """
    테스트/런너 자동 탐지:
      0) 명시 cmd가 있다면 우선 시도
      1) pytest -q
      2) npm test --silent
    비어 있지 않은 출력의 tail만 반환합니다.
    """
    # 0) 명시 커맨드 우선
    out = sh(f"{cmd} || true").strip()
    if out:
        return _tail_last(out, lines)

    # 1) pytest
    out = sh("pytest -q || true").strip()
    if out:
        return "[pytest]\n" + _tail_last(out, lines)

    # 2) npm test
    out = sh("npm test --silent || true").strip()
    if out:
        return "[npm test]\n" + _tail_last(out, lines)

    return "(로그 없음)"


def repo_map(depth: int = 2) -> str:
    """Return repository map using tree → find(pruned) → python(os.walk) fallback."""
    ignores = "|".join(IGNORES)

    # 1) tree 우선
    if shutil.which("tree"):
        out = sh(f"tree -L {depth} -I '{ignores}'")
        if out.strip():
            return out

    # 2) find (불필요 디렉터리 prune)
    if shutil.which("find"):
        # .git, .venv, node_modules, dist, build 등은 제외
        prunes = (
            "-path './.git*' -o "
            "-path './.venv*' -o "
            "-path './node_modules*' -o "
            "-path './dist*' -o "
            "-path './build*' -o "
            "-path './__pycache__*'"
        )
        out = sh(
            f"find . -maxdepth {depth} \\( {prunes} \\) -prune -o -type d -print"
        )
        if out.strip():
            return "[find summary]\n" + out

    # 3) 100% 파이썬 폴백
    buf = io.StringIO()
    buf.write("[python os.walk fallback]\n")
    for dirpath, dirnames, filenames in os.walk(ROOT):
        rel = os.path.relpath(dirpath, ROOT)
        level = 0 if rel == "." else rel.count(os.sep)
        if level > depth:
            # 깊이 제한 초과 시 하위 탐색 중지
            dirnames[:] = []
            continue

        base = os.path.basename(dirpath) or "."
        # 숨김/무시 목록 필터링
        if base in (".git", ".venv", "node_modules", "dist", "build", "__pycache__"):
            dirnames[:] = []
            continue

        buf.write(f"{'  '*level}{base}/\n")
        if level + 1 <= depth:
            for fn in sorted(filenames):
                buf.write(f"{'  '*(level+1)}{fn}\n")

    return buf.getvalue()


def git_diff(paths: Optional[List[str]] = None) -> str:
    """Return git diff for given paths (default entire repo)."""
    path_args = " ".join(paths) if paths else "."
    excludes = " :!".join(["", "package-lock.json", "yarn.lock", "pnpm-lock.yaml"])
    cmd = f"git diff HEAD -- {path_args}{excludes}"
    out = sh(cmd)
    return out or "(diff 없음 또는 커밋된 변경 없음)"


def read_text(path: Optional[str]) -> str:
    if not path:
        return ""
    candidate = Path(path)
    if not candidate.is_absolute():
        file_path = ROOT / candidate
        if not str(candidate).startswith("context_bridge/"):
            file_path = CONTEXT_DIR / candidate
    else:
        file_path = candidate
    if not file_path.exists():
        return ""
    return file_path.read_text(encoding="utf-8", errors="ignore")


def write_text(path: str, content: str) -> None:
    candidate = Path(path)
    if not candidate.is_absolute():
        target = ROOT / candidate
        if not str(candidate).startswith("context_bridge/"):
            target = CONTEXT_DIR / candidate
    else:
        target = candidate
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
    """Generate context bundle that aggregates repo map, goal, diff, and logs."""
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    parts = []

    # Repo Map
    parts.append("## Repo Map\n\n" + repo_map(2))

    # Goal/Constraints
    parts.append(
        "## Goal / Constraints\n\n"
        + textwrap.dedent(
            f"""
            - Goal:
                  - {goal}
            - Constraints:
                  - {constraints}
            """
        ).strip()
    )

    # Changes
    parts.append("## Changes (git diff HEAD)\n\n" + git_diff(diff_paths))

    # Logs (Python 버전 + git HEAD + 테스트 tail)
    py_ver = (sh("python3 -V").strip() or sh("python -V").strip() or "(python version unknown)")
    git_head = (sh("git rev-parse --short HEAD").strip() or "(git HEAD unknown)")
    test_tail = tail_logs(test_cmd, log_lines)
    logs_block = "\n".join(
        [
            f"python: {py_ver}",
            f"git HEAD: {git_head}",
            test_tail,
        ]
    )
    parts.append("## Test / Run Logs\n\n" + logs_block)

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
    """Return a formatted prompt listing k candidate next actions."""
    candidates = [
        "에러/실패 테스트 우선 해결 및 최소 패치 적용",
        "타입 안정성 강화 (strict 옵션/DTO 정합성)",
        "로깅/모니터링 지표 추가 및 성능 병목 추적",
        "경계 케이스 테스트 추가 및 회귀 방지",
        "데이터베이스 쿼리 최적화 (N+1 제거/인덱스 검토)",
    ]
    take = candidates[: max(1, min(k, len(candidates)))]

    prompt = textwrap.dedent(
        f"""
        다음은 Copilot 변경사항 일부와 실패 로그 요약입니다. 상위 목표를 고려하여 다음 작업 후보를 정리했습니다.

        [Objective]\n{objective}\n
        [Diff Excerpt]\n{diff_excerpt}\n
        [Failures]\n{failures}\n
        [Next Action Options]\n- {"\n- ".join(take)}

        우선순위 추천 기준\n- 위험도/안전성\n- 고객 임팩트\n- 리드타임 대비 효과\n- 테스트/릴리스 용이성
        """
    ).strip()
    return prompt


@app.tool()
def make_prompt(
    context_file: str = "context_bridge/context_bundle.md",
    include_checklist: bool = True,
) -> str:
    """Compose a standard external AI prompt using the context bundle."""
    ctx = read_text(context_file)
    checklist = (
        "\n- 타입/스키마 정합성\n- 실패 테스트 재현 및 통과\n"
        "- 성능 임팩트 및 회귀 위험\n- 보안/권한/검증\n- 롤백 전략"
        if include_checklist
        else ""
    )

    prompt = textwrap.dedent(
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
    ).strip()
    return prompt


@app.tool()
def send_to_ai(
    provider: str = "openai",
    system_prompt: str = "You are a senior software engineer.",
    user_prompt: str = "",
    model: str = "gpt-4o-mini",
) -> str:
    """Optionally call external AI providers when API keys are present."""
    if provider not in {"openai", "anthropic"}:
        return "지원하지 않는 provider"

    if provider == "openai" and not os.getenv("OPENAI_API_KEY"):
        return "[DRY RUN] OPENAI_API_KEY 없음 → 아래 프롬프트를 수동으로 복사해 사용:\n\n" + user_prompt
    if provider == "anthropic" and not os.getenv("ANTHROPIC_API_KEY"):
        return "[DRY RUN] ANTHROPIC_API_KEY 없음 → 아래 프롬프트를 수동으로 복사해 사용:\n\n" + user_prompt

    try:  # pragma: no cover - 네트워크 호출은 기본적으로 차단됨
        if provider == "openai":
            return "[샘플] OpenAI 호출부 주석 해제 후 사용하세요."
        return "[샘플] Anthropic 호출부 주석 해제 후 사용하세요."
    except Exception as exc:  # pragma: no cover
        return f"[호출 실패]\n{exc}"


if __name__ == "__main__":
    print(build_context())
