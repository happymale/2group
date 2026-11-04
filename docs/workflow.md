# 저장소 워크플로우

기본 흐름은 [README](../README.md#작업-흐름)를 따른다. 이 문서에는 저장소별 선택을 기록한다.

## 저장소별 선택

bootstrap 때 기록하고, 선택을 바꾸면 함께 고친다. 답하지 않은 항목을 OFF로 추정하지 않는다.

- StrictDoc: 미선택 (ON / OFF). `heven.toml`이 기준이다.
- 저장소 전용 Notion: 미선택 (ON / OFF). 공통 Notion 가이드는 항상 유지한다.
- Discord 알림 / 멘션: 미선택 (각각 ON / OFF)
- Codex PR 리뷰: 미선택 (ON / OFF)
- runner 머신: 미선택
- Project 상태 전환: 항상 사용 (self-hosted runner + `PROJECT_PAT`)

## 원본과의 차이

원본 스킬의 고정 커밋은 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)에 있다. 원본 파일은 수정하지 않고, HEVEN 규칙은 [이슈 트래커 설정](agents/issue-tracker.md)과 래퍼 스킬에 둔다.

- `to-spec`은 쓰지 않는다. 명세는 StrictDoc ON 저장소의 `.sdoc`이거나 이슈 본문이다.
- `triage`는 쓰지 않는다. GitHub Project 상태가 그 역할을 한다.
- ISO/IEC/IEEE 29148은 참고만 하며 표준 준수를 주장하지 않는다.
