# Logs 디렉토리 구조

날짜별 폴더(`YYYY-MM-DD`) 하위에 진행 로그 및 향후 필요 시 학습/평가 로그 저장.

예:
```
logs/
  2025-10-08/
    progress.log       # 전체 개발 진행 요약
    train/             # (옵션) 학습 세션 로그
    mcp/               # (옵션) MCP 서버별 로깅
```

## 운영 가이드
- 하루 마감 전 `progress.log` 최신화 (자동화 스크립트 고려 가능)
- 대용량 학습 로그는 별도 압축 또는 Git LFS 고려
- 민감 정보(API 키 등)는 기록 금지

## 향후 자동화 아이디어
- scripts/update_progress_log.py 작성 후 git hook(pre-commit) 사용
- 정책 성능 요약(평균 리워드, 성공률) 자동 append
