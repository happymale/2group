# heven-common

HEVEN 팀의 private GitHub 저장소 템플릿이다. 이슈와 PR 흐름, GitHub Project 자동화, 공통 스킬을 제공한다. 요구사항 관리(StrictDoc)는 필요한 저장소에서만 켠다.

## 작업 흐름

**모든 저장소 공통:**

```text
이슈 작성 → Backlog → Ready → issue-worker → Draft PR → Ready for review → 리뷰 → merge
                               (In progress)              (In review)              (Done)
```

- 새 이슈는 Project의 Backlog로 들어온다.
- 먼저 끝나야 하는 이슈는 GitHub의 **Blocked by**로 연결한다. `issue-worker`는 열린 선행 이슈가 있으면 시작하지 않는다.
- PR은 `dev`를 대상으로 만들고 `Resolves #번호`로 이슈를 연결한다.
- PR 상태에 따라 이슈 상태가 자동으로 바뀐다. merge 없이 PR을 닫으면 이슈는 Ready로 돌아간다.

**StrictDoc ON 저장소에서는 앞 단계가 추가된다:**

```text
idea-to-requirements → requirements-to-issues → (위 공통 흐름)
```

## 스킬

| 스킬 | 용도 | 사용 시점 |
|---|---|---|
| [bootstrap-repo](.claude/skills/bootstrap-repo/SKILL.md) | 새 저장소 초기 설정 | 공통 |
| [issue-worker](.claude/skills/issue-worker/SKILL.md) | Ready 이슈의 브랜치와 Draft PR 생성 | 공통 |
| [idea-to-requirements](.claude/skills/idea-to-requirements/SKILL.md) | 질문으로 요구사항을 구체화해 StrictDoc에 기록 | StrictDoc ON |
| [requirements-to-issues](.claude/skills/requirements-to-issues/SKILL.md) | 요구사항을 이슈로 분할 | StrictDoc ON |
| [grill-me](.claude/skills/grill-me/SKILL.md), [grilling](.claude/skills/grilling/SKILL.md) | 계획과 설계를 질문으로 점검 | 필요할 때 |
| [domain-modeling](.claude/skills/domain-modeling/SKILL.md) | 용어집과 결정 기록(ADR) 관리 | 필요할 때 |
| [to-tickets](.claude/skills/to-tickets/SKILL.md) | 계획을 선후관계가 있는 이슈로 분할 | 필요할 때 |
| [handoff](.claude/skills/handoff/SKILL.md) | 다음 세션용 인수인계 문서 작성 | 필요할 때 |
| [teach](.claude/skills/teach/SKILL.md) | 주제별 학습 관리 | 필요할 때 |
| [process-improvement](.claude/skills/process-improvement/SKILL.md) | 회고와 공통 프로세스 개선 제안 | 필요할 때 |

Matt Pocock 원본 스킬은 수정하지 않고 사용한다. 출처는 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)에 있다.

## 새 저장소 만들기

1. **Use this template**으로 `skku-heven/<new-repo>` private 저장소를 만든다.
2. 저장소를 clone하고 Codex 또는 Claude Code에 요청한다.

> 이 repo의 bootstrap-repo skill을 사용해서 skku-heven/<new-repo>를 부트스트랩해줘.

스킬이 필요한 결정을 한 번에 묻고 GitHub, Project, 저장소 파일을 설정한다. 비밀값은 채팅이나 문서에 적지 않는다.

| 질문 | 값 |
|---|---|
| StrictDoc | ON/OFF. ON이면 요구사항 계층 선택 |
| 저장소 전용 Notion | ON/OFF (기본 OFF) |
| Discord 알림, 멘션 | 각각 ON/OFF |
| Codex PR 리뷰 | ON/OFF (기본 OFF) |
| runner 머신 | Project 상태 전환을 실행할 머신 |

선택은 `docs/workflow.md`에, StrictDoc 설정과 기준이 되는 템플릿 커밋은 `heven.toml`에 기록한다. Project 상태 전환은 항상 켜져 있다.

- 이 repository의 Notion hub: `<NOTION_REPO_URL>`

## StrictDoc

`heven.toml`에서 켜고 계층을 고른다. 기본은 OFF이며, OFF면 관련 스킬과 검사는 아무 작업도 하지 않는다.

```toml
[requirements]
enabled = true
levels = ["stakeholder", "software"]  # business, stakeholder, system, software, hardware
```

```bash
make requirements-init   # 선택한 계층의 없는 문서만 생성
make requirements-check  # 문법, UID, 관계 검사
make serve               # http://127.0.0.1:5111/ 에서 문서 보기, 종료 Ctrl+C
make docs                # HTML 내보내기
make requirements-sync   # Issue → PR UID 동기화 미리보기
```

uv와 make가 필요하다(Windows: WSL). 도구는 프로젝트의 Python·ROS 환경과 분리된 환경에서 실행된다. 포트는 `make serve PORT=5112`로 바꾼다. 자세한 내용은 [요구사항 작업 흐름](docs/requirements-workflow.md)을 따른다.

## 자동화

| 구성 | 역할 | 기본 |
|---|---|---|
| `project-status` | PR 상태에 맞춰 이슈 상태 변경 | 항상 켜짐 |
| `requirements`, `requirements-links` | 요구사항 검사, Issue의 UID를 PR로 동기화 | StrictDoc ON에서 동작 |
| `codex-pr-review` | Ready PR을 Codex로 리뷰 | OFF |
| `notify-discord` | PR과 이슈 알림 | OFF |

runner와 토큰 설정은 [운영 문서](ops/runner/README.md)를 따른다.

## 문서

- [에이전트 규칙](AGENTS.md) · [저장소별 선택](docs/workflow.md) · [요구사항 작업 흐름](docs/requirements-workflow.md) · [이슈 트래커 설정](docs/agents/issue-tracker.md)
- [공통 Notion 가이드](https://app.notion.com/p/39c3bf068300807c9e5bcf6b469f94ee) · [repo wiki](https://app.notion.com/p/39c3bf06830080b5a024c7ad91855240)
- [Getting Started](https://app.notion.com/p/39c3bf06830081fb8e76d5c9a1be6d82) · [Team Workflow](https://app.notion.com/p/39c3bf068300810db8adc15fce65450d) · [Operations](https://app.notion.com/p/39c3bf068300815fb880e1d20602e662)
