---
name: issue-worker
description: Use when a team member starts work on a Ready GitHub Project issue. Reuses an existing canonical PR without mutation, or creates the normal branch, empty commit, and draft PR path for a new claim.
---

# issue-worker — Ready 이슈 잡고 작업 시작하기

보이는 Project item이 정확히 하나이고 Status가 **Ready**인 이슈를 작업 시작 상태로
만드는 skill이다. 먼저 issue의 linked open PR references를 조회하므로 제목이나 Issue Type이
바뀌어도 기존 canonical PR을 찾는다. 새 claim이면 branch와 첫 empty commit을 만든 뒤
push하고 draft PR을 만들며, 검증된 linked PR이 있으면 아무것도 바꾸지 않고 재사용한다.

## 문서 경계

- 에이전트가 따라야 할 자족적인 machine contract: repo `AGENTS.md`
- 사람용 공통 workflow·컨벤션·운영 가이드:
  [repo wiki의 공통 가이드](https://app.notion.com/p/39c3bf068300807c9e5bcf6b469f94ee)
- claim부터 merge까지의 사람용 절차:
  [Team Workflow](https://app.notion.com/p/39c3bf068300810db8adc15fce65450d)

Notion은 이 자동화의 필수 의존성이 아니다.

## 사용법

정규 경로는 repo root에서 다음과 같이 요청하는 것이다.

> 이 repo의 `issue-worker` skill을 사용해서 이슈 #42를 잡아줘.

직접 실행할 때는 다음 script를 사용한다.

```bash
.claude/skills/issue-worker/scripts/claim_issue.sh 42
```

## 사전 조건

### 모든 호출

- `gh` CLI가 설치되어 있고 로그인되어 있어야 한다.
- git repo 안에서 실행해야 한다.
- repo clone 안의 working tree가 깨끗해야 한다.
- repo와 base branch는 clone에서 자동 감지하며 필요하면 `GH_REPO`, `DEFAULT_BRANCH`로 덮어쓴다.
- standalone `jq`가 설치되어 있어야 한다.
- issue가 open이어야 한다.

기존 canonical open PR 재사용 경로는 `Ready` 상태나 GitHub Project read 권한을 요구하지 않는다.
재사용 PR은 draft 또는 Ready for review 어느 상태든 가능하며 그 상태를 결과에 표시한다.

### 새 claim에만 추가

- 보이는 Project item이 정확히 하나여야 한다.
- 그 item의 Status가 정확히 `Ready`여야 한다.
- GitHub Blocked by에 열린 선행 이슈가 없어야 한다. 있으면 그 번호를 보여 주고 멈춘다.
- `gh` token에 Project 읽기 권한이 있고 해당 Project에 접근할 수 있어야 한다.
- `projectItems pagination`이 감지되면 일부 item만 보고 판단하지 않고 실패한다.

Backlog / In review / Done, Status를 읽을 수 없는 상태, Project item이 없거나 여러 개인
상태는 새 claim을 거부한다.

## 실행 계약

1. issue가 open인지 확인하고 그 issue의 closing/linked open PR references를 조회한다.
2. open linked PR이 정확히 하나이면 다음 canonical 조건을 모두 검증한다.
   - issue를 실제 close/link하고 base가 감지된 base branch와 같다.
   - base/head repository가 현재 repo와 같고 head가
     `fix|feat|exp|chore/<N>-...` 형식이다.
   - 조건을 만족하면 Ready를 다시 검사하지 않고 PR 번호와 draft/Ready 상태를 보고한 뒤
     git/GitHub mutation 없이 종료한다. 여러 개거나 malformed이면 실패한다.
3. linked PR이 없을 때만 Issue Type과 제목으로 새 canonical branch
   `<type>/<N>-<slug>`를 계산한다. 그 head에 unlinked open PR이 있으면 conflict로 실패한다.
4. 새 claim이면 전체 `projectItems`가 보이는지와 정확히 하나의 `Ready` item인지, 열린 선행 이슈가 없는지 확인한다.
5. Ready 확인 후 remote issue branch를 조회한다. 하나면 이전 push/PR-create 실패의
   retry로 복구하고, 여러 개면 실패하며, 없으면 최신 base에서 canonical branch를 만든다.
6. `origin/$BASE..HEAD` commit 수가 0일 때만 첫 empty commit을 만든다. tree diff가
   비어 있어도 기존 empty commit이 있으면 추가하지 않는다.
7. branch를 push하고 PR 템플릿과 `Resolves #N`으로 **draft PR**을 만든다.
8. GitHub Project의 `PR linked to issue` 자동화가 issue를 In progress로 옮긴다.

Issue Type mapping:

- Bug → `fix`
- Task → `feat`
- Experiment → `exp`
- Chore → `chore`
- 없거나 읽지 못하면 `feat`

## 작업 중과 완료

- 작업 중에는 `git commit && git push`로 draft PR을 갱신한다.
- 저장소 CI를 언제 돌릴지는 각 workflow 설정을 따른다. StrictDoc ON이면 요구사항 검사는 draft에서도 실행된다.
- 구현과 자체 검증을 끝낸 뒤 PR에서 **Ready for review**를 선택한다. `ENABLE_CODEX_REVIEW=true`이면 이때 Codex 리뷰가 실행된다.
- `project-status`가 PR 상태에 따라 이슈를 옮긴다: draft는 In progress, Ready는 In review, merge 없이 닫으면 Ready.
- 상태가 바뀌지 않으면 Actions 결과와 runner, `PROJECT_PAT`을 확인한다.
- 사람 리뷰 후 maintainer가 repository에서 허용된 merge 방식 중 변경에 맞는 방식을
  선택해 `dev`에 merge한다.
- merge 뒤 remote 작업 branch는 repository 설정에 따라 GitHub가 자동 삭제한다.
- local 작업 branch 정리는 작업자가 선택한다.

## mutation 경계

- label / Issue Type / Priority / milestone / Notion은 직접 변경하지 않는다.
- Project field를 직접 변경하지 않는다. lifecycle built-in과 별도 status 자동화를 사용한다.
- PR merge/approve/request-changes를 수행하지 않는다.
- `dev`/`main`에 직접 commit하지 않는다.
- PR은 항상 draft로 시작하고 완성 전 Ready for review로 올리지 않는다.

## StrictDoc ON 저장소

`heven.toml`에서 StrictDoc이 켜져 있으면 이슈의 UID 블록과 해당 요구사항을 읽고 작업한다. PR에는 `Resolves #N`만 유지한다. UID는 연결된 이슈에서 자동으로 복사되므로 직접 관리하지 않는다. 자세한 내용은 `docs/requirements-workflow.md`를 따른다.
