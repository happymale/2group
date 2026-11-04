---
name: requirements-to-issues
description: StrictDoc ON 저장소에서 검토한 요구사항을 합격 기준과 선후관계가 있는 GitHub 이슈로 나눈다. 명시적으로 호출한다.
disable-model-invocation: true
---

# Requirements → Issues

## 먼저 확인

`heven.toml`을 읽는다. `requirements.enabled`가 `false`이면 "이 저장소는 StrictDoc OFF입니다. 이슈를 직접 만들거나 필요하면 `to-tickets`를 사용하세요"라고 알리고 끝낸다.

## 진행

1. [to-tickets](../to-tickets/SKILL.md)와 [이슈 트래커 설정](../../../docs/agents/issue-tracker.md)을 읽고 그 절차로 나눈다. 두 내용이 다르면 트래커 설정이 우선한다.
2. 요구사항, 기존 이슈, Project 상태를 읽는다. 미정 항목이 있으면 임의로 정하지 않고 `idea-to-requirements`로 돌린다. `Draft` 요구사항을 이슈로 나눌 때는 사용자에게 확인한다. 기능을 유지만 하는 요구사항은 검증 이슈로, 수정·신규는 구현과 검증 이슈로 만든다.
3. 이슈별 제목, UID, 결과, 합격 기준, 선행 이슈를 보여 주고 승인을 받는다.
4. `make requirements-check`를 실행한 뒤 이슈를 만들고, 본문에 `docs/requirements-workflow.md`의 UID 블록을 넣는다. UID가 없는 이슈도 허용한다.
5. `make requirements-sync`로 변경 사항을 확인한다. 시작 가능한 이슈를 알려 주되, 브랜치와 PR은 만들지 않는다. 다음 단계는 `issue-worker`다.

요구사항 본문은 StrictDoc에 두고, 이슈에는 UID와 구현 작업만 적는다.
