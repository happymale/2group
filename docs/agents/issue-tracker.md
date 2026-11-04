# 이슈 트래커: GitHub

이 저장소의 이슈는 GitHub Issues와 연결된 Project에서 관리한다. `gh` CLI를 사용한다.

## HEVEN 규칙

- 라벨로 상태를 관리하지 않는다. `ready-for-agent` 등 triage 라벨을 만들거나 붙이지 않는다.
- 상태는 Project의 `Backlog → Ready → In progress → In review → Done`을 쓴다.
- 새 이슈는 Backlog에서 시작한다. Ready 전환은 사용자가 허용할 때만 한다.
- 선후관계는 GitHub의 Blocked by만 쓰고, 본문에 따로 적지 않는다.
- 브랜치와 PR은 `issue-worker`가 만든다.

## 명령

- **생성:** `gh issue create --title "..." --body "..."`
- **조회:** `gh issue view <number> --json number,title,body,labels,comments`
- **목록:** `gh issue list --state open --json number,title,body`
- **선후관계 추가:** `gh issue edit <이슈> --add-blocked-by <선행 이슈>` (`gh` 2.94 이상)
- **열린 선행 이슈 수:** `gh api repos/<owner>/<repo>/issues/<n> --jq .issue_dependencies_summary.blocked_by`
- **하위 이슈:** `gh issue edit <상위> --add-sub-issue <하위>` (`gh` 2.94 이상)
- **코멘트:** `gh issue comment <number> --body "..."`

저장소는 `git remote`에서 판단한다. 다른 저장소에 이슈를 만들지 않는다.

## 스킬이 트래커를 말할 때

- "트래커에 올린다"는 GitHub 이슈를 만든다는 뜻이다.
- "티켓을 가져온다"는 위 조회 명령을 뜻한다.
