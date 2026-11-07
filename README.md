# heven-hackathon-2026

HEVEN 2026 Docker 배포본 기반 참가자 실행 환경·스켈레톤·사용 문서.

현재 공통 템플릿 bootstrap 단계이며 시뮬레이터·배포 이미지·참가자 코드는 아직 포함하지 않는다.

## 시작하기

GitHub Issue·PR·Project를 사용한다. StrictDoc OFF 저장소라 이슈 → `issue-worker` → Draft PR → 리뷰의 기본 흐름만 사용한다. 저장소별 선택과 스킬은 [작업 흐름](docs/workflow.md), [에이전트 규칙](AGENTS.md)을 참고한다.

## 배포 경계

개발 저장소 [heven-hackathon-2026-dev](https://github.com/skku-heven/heven-hackathon-2026-dev)가 검증한 Docker 이미지를 사용한다. 이미지와 실행 명령은 배포가 준비되면 추가한다. StrictDoc은 OFF이며 요구사항 정본을 중복 관리하지 않는다.

## 협업 설정

저장소 전용 Notion·Discord 알림/멘션·Codex 자동 리뷰는 OFF다. Project 상태 전환은 사용한다.

- 이 repository의 Notion hub: 사용 안 함 (NOTION_MODE=disabled)
- [공통 Notion 가이드](https://app.notion.com/p/39c3bf068300807c9e5bcf6b469f94ee)
