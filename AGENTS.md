# AGENTS.md

이 문서는 이 저장소와 파생 저장소에서 에이전트가 따라야 할 최소 규칙이다.

## 정본

| 위치 | 내용 |
|---|---|
| README.md | repo 목적과 setup/build/run/test |
| AGENTS.md | 에이전트 작업 규칙 |
| GitHub Issue / Project / PR / Milestone | 작업 내용·상태·일정·결정 기록 |
| `docs/requirements/*.sdoc` | 요구사항 (StrictDoc ON 저장소만) |
| 공통 Notion 가이드 | 공통 workflow·convention |
| repo별 Notion hub | domain·대회·hardware·환경·제약 |

- 공통 가이드: [heven-common repo wiki](https://app.notion.com/p/39c3bf068300807c9e5bcf6b469f94ee)
- 현재 repo hub: `사용 안 함 (NOTION_MODE=disabled)`
- Notion enabled인 경우 domain·대회·hardware·환경·제약에 관련된 작업은 Notion MCP로 repo별 hub를 먼저 읽는다. 공통 가이드만 보고 repo 제약을 추측하지 않는다.
- Notion disabled이면 저장소 문서를 사용한다.
- Notion에 접근할 수 없으면 Notion 명세가 필요한 작업만 멈추고, 일반 코드·CI 작업은 계속한다.

## 작업 흐름

- 작업 전 README, 관련 Issue와 Project 상태, 기존 PR, `git status`를 확인한다.
- Ready Issue는 `.claude/skills/issue-worker/SKILL.md`를 읽고 그 절차로 시작한다. 연결된 open PR이 있으면 새로 만들지 않고 재사용한다.
- `main`은 현재 stable/release-state branch, `dev`는 integration branch다.
- Issue branch는 `<type>/<N>-<slug>`를 사용한다: Bug→`fix`, Task→`feat`, Experiment→`exp`, Chore→`chore`.
- `dev`와 `main`에 직접 push하지 않는다. PR은 draft로 시작하고 Issue가 있으면 `Resolves #N`을 적는다.
- 구현 뒤 관련 검증을 실행하고 PR을 Ready for review로 바꾼다. CI와 review가 끝나면 maintainer가 repository에서 허용된 merge 방식 중 적절한 방식을 선택한다.
- merge 뒤 remote 작업 branch는 repository 설정에 따라 GitHub가 자동 삭제한다.
- GitHub Free private repository가 위 규칙을 강제하지 않아도 우회하지 않는다.

## 작업 경계

- 요청된 변경만 수행하고 기존 사용자 변경을 보존한다.
- 명시적 요청이나 관련 skill 절차 없이 label, Issue Type, Priority, milestone, Project field, Issue close, PR merge·review, Notion 내용을 바꾸지 않는다.
- ROS bag·pcap, build artifact, model weight, log, local data, external clone을 commit하지 않는다.
- 설정에는 machine absolute path 대신 environment variable 또는 repo-relative path를 사용한다.
- 완료 전에 가장 작은 관련 검증부터 실행하고 실제 결과를 기록한다.

## Skills

- 반복 작업은 `.claude/skills/<name>/SKILL.md`를 먼저 읽고 따른다. 스킬 안의 "Skill tool" 호출은, 그 도구가 없으면 해당 스킬 파일을 읽어 수행한다.
- 새 저장소 설정은 `bootstrap-repo`, Ready 이슈 시작은 `issue-worker`를 사용한다.
- StrictDoc ON 저장소의 요구사항 작업은 `idea-to-requirements`, `requirements-to-issues`를 명시적으로 호출한다. OFF 저장소에서는 사용하지 않는다.
- `grill-me`, `to-tickets`, `handoff`, `teach`, `process-improvement`는 명시적으로 요청했을 때 사용한다.
- 저장소별 선택은 [docs/workflow.md](docs/workflow.md), runner 운영은 [ops/runner/README.md](ops/runner/README.md)를 따른다.

## Agent skills

### Issue tracker

GitHub Issues와 연결된 Project를 사용한다. See `docs/agents/issue-tracker.md`.
