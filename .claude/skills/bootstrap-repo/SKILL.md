---
name: bootstrap-repo
description: heven-common 템플릿으로 만든 새 private 저장소를 설정하거나 bootstrap을 다시 실행할 때 사용한다.
---

# bootstrap-repo — 새 저장소 초기 설정

요청 문구:

> 이 repo의 `bootstrap-repo` skill을 사용해서 `skku-heven/<new-repo>`를 부트스트랩해줘.

## 1. 결정 사항 질문

아래 항목을 **한 번에 모두 묻는다.** `docs/workflow.md`에 이미 있는 선택은 다시 묻지 않는다. 답하지 않은 항목을 OFF로 추정하지 않는다.

| 항목 | 값 | 비고 |
|---|---|---|
| 저장소 목적 | 한 줄 | `REPO_PURPOSE` |
| StrictDoc | ON / OFF | OFF면 요구사항 문서 없이 기본 흐름만 사용 |
| 요구사항 계층 | business, stakeholder, system, software, hardware 중 선택 | StrictDoc ON일 때만. 예: stakeholder, software |
| 저장소 전용 Notion | ON / OFF | 기본 OFF. 공통 가이드 링크는 항상 유지 |
| Discord 알림 | ON / OFF | ON이면 webhook 필요 |
| Discord 멘션 | ON / OFF | 알림 ON일 때만. 사용자 맵 필요 |
| Codex PR 리뷰 | ON / OFF | 기본 OFF. Project 상태 전환과 별개 |
| runner 머신 | 머신 이름 | Project 상태 전환에 필수 |

Project 상태 전환은 항상 사용하므로 묻지 않는다.

## 2. 사전 확인

템플릿 생성 직후에는 내용이 아직 준비되지 않을 수 있다. 저장소 내용과 `dev` 브랜치가 보일 때까지 읽기 전용 확인을 최대 60초 반복한다.

```bash
.claude/skills/bootstrap-repo/scripts/bootstrap-repo.sh --preflight-only skku-heven/<new-repo>
```

private, 기본 브랜치 `dev`, 관리자 권한, `skku-heven/heven-common` 템플릿 출처 중 하나라도 맞지 않으면 아무것도 바꾸지 않고 멈춘다.

## 3. 인증과 비밀값

- 새 토큰을 요구하기 전에 기존 인증을 확인한다. 저장소마다 새 토큰이 필요하지는 않다.
- CLI의 `gh project` 명령에는 `project`와 `read:org`, `PROJECT_PAT`에는 `project`와 `repo` 권한이 필요하다.
- 비밀값은 `DISCORD_WEBHOOK`, `PROJECT_PAT`이다. 채팅, 문서, 명령 인자, 로그에 쓰지 않고, 안전한 입력으로만 받는다. 값이 있는지만 확인한다.
- 안전한 입력 수단이 없으면 멈추고, 사용자의 터미널에서 직접 실행하도록 안내한다.

```bash
read -r -s -p 'Project PAT: ' PROJECT_PAT; export PROJECT_PAT; echo
.claude/skills/bootstrap-repo/scripts/bootstrap-repo.sh skku-heven/<new-repo>
unset PROJECT_PAT
```

## 4. Notion

저장소 전용 Notion이 ON일 때만 [Notion 절차](references/notion.md)를 따른다. OFF면 `README.md`, `AGENTS.md`의 `<NOTION_REPO_URL>`을 `사용 안 함 (NOTION_MODE=disabled)`로 바꾸고 `ops/runner/repo-context.sh`의 값은 비운다.

## 5. GitHub 설정

선택한 값을 환경 변수로 준비하고 실행한다. 여러 번 실행해도 된다.

```bash
.claude/skills/bootstrap-repo/scripts/bootstrap-repo.sh skku-heven/<new-repo>
```

- `main` 생성, 기본 브랜치 `dev`, Wiki 끄기, merge 후 브랜치 삭제, squash 허용
- 선택한 기능의 secret과 variable 설정
- Project #14를 `<repo> Roadmap`으로 복사하거나 재사용하고 저장소에 연결

## 6. 저장소 파일 정리

저장소 clone의 root에서 실행한다. 기준 커밋과 선택을 `heven.toml`에 기록하고, 템플릿 전용 테스트와 CI를 제거한다. StrictDoc ON이면 선택한 계층의 문서를 만든다.

```bash
TEMPLATE_COMMIT=$(gh api repos/skku-heven/heven-common/commits/dev --jq .sha) \
STRICTDOC=enabled REQUIREMENT_LEVELS=stakeholder,software \
  .claude/skills/bootstrap-repo/scripts/prepare-derived.sh
```

그 뒤 `docs/workflow.md`에 선택을 기록하고, README의 이름, 목적, 시작 안내를 프로젝트에 맞춘다. 아직 없는 build/run/test 절차를 추측하지 않는다. 변경은 이슈와 PR로 올린다. 요구사항과 구현은 만들지 않는다.

## 7. Runner

1. 저장소에 연결된 online runner가 있는지 확인한다.
2. 선택한 머신에 저장소용 runner가 없으면, 대상 머신의 서비스와 설치 경로를 먼저 살핀다. 다른 머신에 설치하는 일이므로 **실행 전에 사용자에게 확인받는다.**
3. 머신에는 `gh`, `git`이 필요하다. Codex 리뷰를 켜면 인증된 Codex CLI도 필요하다.
4. org runner를 쓰는 경우 public 저장소 접근을 막고, 가능하면 대상 private 저장소만 허용한다.

## 8. 완료 확인

1. `<repo> Roadmap`의 Auto-add를 GitHub 화면에서 설정한다. 필터 `is:issue is:open`, 상태 `Backlog`. 이 항목만 직접 설정한다.
2. 팀원이 저장소와 Project에 접근할 수 있는지 확인한다.
3. 연결된 이슈가 있는 PR로 상태 전환을 확인한다. 첫 bootstrap PR을 써도 된다.
4. 결과를 보고한다. 비밀값은 출력하지 않는다.
