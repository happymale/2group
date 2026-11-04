# ops/runner

GitHub Actions self-hosted runner에서 실행하는 스크립트다. workflow가 기본 브랜치를 checkout해 직접 실행하므로 머신에 따로 배포하지 않는다.

| 파일 | 역할 |
|---|---|
| `bin/pr-set-in-review.sh` | 이슈의 Project Status를 PR 상태에 맞춤 |
| `bin/codex-pr-review.sh` | Codex PR 리뷰 (선택) |
| `bin/render_prompt.sh`, `repo-context.sh`, `prompts/` | 리뷰 프롬프트 생성 |

## Runner

저장소에 연결된 self-hosted runner가 필요하다. 없으면 상태 전환 job이 대기한다. 머신에는 `gh`와 `git`이 필요하고, Codex 리뷰를 켜면 인증된 Codex CLI도 필요하다. 스크립트는 구버전 `gh`(2.4)에서도 동작하도록 `gh api`를 우선 사용한다.

org runner를 쓰면 **Organization Settings → Actions → Runner groups**에서 public 저장소 접근을 막고, 가능하면 대상 private 저장소만 허용한다.

## Project 상태 전환

`project-status.yml`은 항상 켜져 있다.

| PR 상태 | 이슈 상태 |
|---|---|
| Draft | In progress |
| Ready for review | In review |
| merge 없이 닫힘 | Ready (In progress, In review일 때만) |
| merge | GitHub Project 기본 동작으로 Done |

- 스크립트는 이슈가 속한 Project 하나에서 필요한 값을 읽는다. 이슈가 Project에 없거나 여러 개에 있으면 실패한다.
- Done이나 이미 같은 상태인 이슈는 그대로 둔다. 일부 실패 후 다시 실행해도 남은 이슈만 처리한다.
- 연결 이슈가 없는 PR은 경고만 남기고 성공한다.

## 설정

| 종류 | 이름 | 내용 |
|---|---|---|
| secret | `PROJECT_PAT` | 상태 전환용 classic PAT (`project`, `repo`). 필수 |
| secret | `DISCORD_WEBHOOK` | Discord 알림 webhook (선택) |
| variable | `ENABLE_CODEX_REVIEW` | `true`면 Codex 리뷰 실행 |
| variable | `ENABLE_DISCORD_NOTIFICATIONS` | `true`면 Discord 알림 실행 |
| variable | `ENABLE_DISCORD_MENTIONS` | `true`면 실제 멘션 사용 |
| variable | `DISCORD_USER_MAP` | GitHub 로그인→Discord ID JSON (선택) |

`PROJECT_PAT`은 개인 계정에 묶인다. 발급자가 팀을 떠나거나 토큰이 노출되면 새 토큰을 발급하고 저장소마다 교체한 뒤 이전 토큰을 폐기한다. 값은 채팅이나 문서에 적지 않는다.

```bash
read -r -s PROJECT_PAT; printf '%s' "$PROJECT_PAT" | gh secret set PROJECT_PAT -R <owner>/<repo>; unset PROJECT_PAT
```

## Codex PR 리뷰

`ENABLE_CODEX_REVIEW=true`이면 PR이 Ready가 될 때 한 번 실행한다. 이후 push에는 다시 실행하지 않는다. 스크립트가 GitHub 데이터를 먼저 받아 프롬프트에 넣고, Codex는 GitHub 토큰 없이 읽기 전용으로 실행한다. 다만 runner 머신의 `gh` 로그인이 있으면 Codex도 쓸 수 있으므로, 가능하면 runner 계정에 상시 로그인을 두지 않는다.
