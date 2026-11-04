---
name: idea-to-requirements
description: StrictDoc ON 저장소에서 질문을 통해 아이디어를 구체화하고 선택한 요구사항 계층에 기록한다. 구현 전이나 요구사항을 고칠 때 명시적으로 호출한다.
disable-model-invocation: true
---

# Idea → Requirements

원본 `grill-with-docs`에 StrictDoc 기록을 더한 HEVEN 스킬이다.

## 먼저 확인

`heven.toml`을 읽는다. `requirements.enabled`가 `false`이면 "이 저장소는 StrictDoc OFF입니다"라고 알리고 끝낸다. 파일을 만들거나 설정을 바꾸지 않는다. 사용자가 직접 요청할 때만 `enabled`와 `levels`를 바꾸고 `make requirements-init`을 실행한다.

## 진행

1. `docs/workflow.md`, 기존 요구사항, 용어집, 관련 코드를 읽는다. 기존 코드를 확장하는 경우 기능별로 유지·수정·신규·제외를 구분한다.
2. [grilling](../grilling/SKILL.md)으로 인터뷰하고 [domain-modeling](../domain-modeling/SKILL.md)으로 용어와 결정을 기록한다. 두 파일을 읽어 그대로 따른다.
3. 결정된 내용을 [요구사항 프로필](references/requirements-profile.md)에 맞춰 `docs/requirements/<level>.sdoc`에 Draft로 쓴다. 기존 UID는 유지한다. 답을 받지 못한 항목은 지어내지 않고 미정으로 남긴다.
4. `make requirements-check`를 실행하고, 변경한 요구사항과 미정 항목을 보여 준다.

이 스킬에서는 제품 구현과 GitHub 이슈 생성을 하지 않는다. 다음 단계는 `requirements-to-issues`이며 자동으로 호출하지 않는다.
